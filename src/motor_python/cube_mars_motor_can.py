"""AK60-6 Motor Control Class - CubeMars Native CAN Protocol."""

import struct
import threading
import time
from dataclasses import dataclass

import can
import numpy as np
from loguru import logger

from motor_python.definitions import (
    CAN_DEFAULTS,
    MOTOR_DEFAULTS,
    MOTOR_LIMITS,
    TendonAction,
)


# Error code descriptions from CubeMars CAN protocol spec (section 4.3.1)
CAN_ERROR_CODES: dict[int, str] = {
    0: "No fault",
    1: "Motor over-temperature",
    2: "Over-current",
    3: "Over-voltage",
    4: "Under-voltage",
    5: "Encoder fault",
    6: "MOSFET over-temperature",
    7: "Motor lock-up",
}


@dataclass
class CANMotorFeedback:
    """Motor feedback data from the CAN upload message (8 bytes, section 4.3.1).

    The motor transmits this frame periodically at the configured rate (1–500 Hz).
    Byte layout (big-endian):

      [0][1] — Position  : int16, raw -32000..32000 → -3200°..3200°  (× 0.1)
      [2][3] — Speed     : int16, raw -32000..32000 → -320000..320000 ERPM (× 10)
      [4][5] — Current   : int16, raw -6000..6000   → -60..60 A  (× 0.01)
      [6]    — Temp      : int8,  raw -20..127       → -20..127 °C (driver board)
      [7]    — Error     : uint8, 0 = no fault, see CAN_ERROR_CODES
    """

    position_degrees: float   # Motor position in degrees   (-3200° to +3200°)
    speed_erpm: int           # Electrical speed in ERPM    (-320000 to +320000)
    current_amps: float       # Phase current in amps        (-60 to +60 A)
    temperature_celsius: int  # Driver board temperature     (-20 to +127 °C)
    error_code: int           # Fault code (0 = OK), see CAN_ERROR_CODES

    @property
    def error_description(self) -> str:
        """Human-readable error description from the CubeMars spec."""
        return CAN_ERROR_CODES.get(self.error_code, f"Unknown error ({self.error_code})")


class CANControlMode:
    """CAN extended ID control modes per official CubeMars documentation.

    The control mode is encoded in the upper bytes of the 29-bit extended CAN ID.
    Format: 00 00 0M XX where M is mode and XX is motor ID.
    """

    DUTY_CYCLE = 0x00  # Duty cycle control mode (square wave voltage)
    CURRENT_LOOP = 0x01  # Current control (IQ current, torque mode)
    CURRENT_BRAKE = 0x02  # Brake current mode (hold position with current)
    VELOCITY_LOOP = 0x03  # Velocity control (ERPM with max acceleration)
    POSITION_LOOP = 0x04  # Position control (degrees with max speed/accel)
    SET_ORIGIN = 0x05  # Set origin position mode (temporary or permanent)
    POSITION_VELOCITY = 0x06  # Position + velocity + acceleration control
    MIT_MODE = 0x08  # MIT control mode (Force mode with KP, KD)


class CubeMarsAK606v3CAN:
    """AK60-6 Motor Controller using native CAN bus protocol.

    Uses SocketCAN interface on Jetson Orin Nano with SN65HVD230 CAN transceiver.
    Implements the CubeMars native CAN protocol with extended IDs and 8-byte payloads.

    CAN Protocol Details:
    - Extended 29-bit CAN IDs encode control mode: 00 00 0M XX (M=mode, XX=motor ID)
    - Command payloads: 8 bytes direct data (not UART protocol)
    - Feedback ID: 0x2903 (extended 29-bit), format: pos int16 / spd int16 / cur int16 / tmp int8 / err uint8
    - Feedback mode: 50 Hz periodic broadcast on 0x2903 (extended 29-bit) once motor is enabled
      and UART cable is disconnected.  The motor ALSO sends one 0x2903 frame immediately after
      each command it receives.  _capture_response() buffers that immediate reply so callers
      can use _receive_feedback() right after any command without waiting for the next broadcast.
    """

    def __init__(
        self,
        motor_can_id: int = CAN_DEFAULTS.motor_can_id,
        interface: str = CAN_DEFAULTS.interface,
        bitrate: int = CAN_DEFAULTS.bitrate,
        feedback_can_id: int | None = None,
    ) -> None:
        """Initialize CAN motor connection.

        :param motor_can_id: Motor CAN ID (default: 0x03 from motor config).
        :param interface: CAN interface name (default: 'can0').
        :param bitrate: CAN bitrate in bits/sec (default: 1000000 = 1 Mbps).
        :param feedback_can_id: Override the expected feedback CAN ID.  Pass
            the exact ID observed via candump/sniffer if auto-detection fails.
            Default None = auto-detect using all known CubeMars ID schemes.
        :return: None
        """
        self.motor_can_id = motor_can_id
        self.interface = interface
        self.bitrate = bitrate
        self.bus: can.BusABC | None = None
        self.connected = False
        self.communicating = False
        self._last_feedback: CANMotorFeedback | None = None
        # Feedback from the most-recent command response (consumed by next
        # _receive_feedback call).  The motor operates in response-only mode:
        # it sends exactly one 0x2903 status frame per command it receives.
        self._pending_feedback: CANMotorFeedback | None = None
        self._consecutive_no_response = 0
        self._max_no_response = MOTOR_DEFAULTS.max_no_response_attempts

        # Background refresh thread — re-sends the active command at 50 Hz.
        # The motor watchdog cuts power if no command arrives within ~100 ms.
        self._refresh_mode: int | None = None
        self._refresh_data: bytes | None = None
        self._refresh_stop = threading.Event()
        self._refresh_thread: threading.Thread | None = None
        self._refresh_interval: float = 0.02  # 50 Hz

        # Build the set of candidate feedback IDs so _receive_feedback can
        # accept whichever scheme this firmware version uses.
        #   0x2900 | id  — extended-ID scheme documented in CAN journey notes
        #   0x0080 | id  — standard-frame scheme (0x83 for id=3)
        #   id itself    — direct-match fallback
        self._feedback_ids: set[int] = {
            motor_can_id,
            0x2900 | motor_can_id,
            0x0080 | motor_can_id,
        }
        if feedback_can_id is not None:
            self._feedback_ids.add(feedback_can_id)
            logger.info(
                f"Feedback CAN ID override: 0x{feedback_can_id:08X} "
                f"(candidates: {[hex(x) for x in sorted(self._feedback_ids)]})"
            )
        else:
            logger.debug(
                f"Feedback ID candidates: {[hex(x) for x in sorted(self._feedback_ids)]}"
            )

        self._connect()

    def _connect(self) -> None:
        """Establish CAN bus connection to motor.

        :return: None
        """
        try:
            # Only accept extended-ID feedback frames from this motor.
            # An unknown device on the bus (0x0088) floods at ~30 kHz with
            # standard 11-bit frames; without this filter it fills the socket
            # receive buffer and starves every bus.recv() call.
            feedback_id = 0x2900 | self.motor_can_id
            can_filters = [
                {"can_id": feedback_id, "can_mask": 0x1FFFFFFF, "extended": True}
            ]
            self.bus = can.interface.Bus(
                channel=self.interface,
                interface="socketcan",
                bitrate=self.bitrate,
                can_filters=can_filters,
            )
            time.sleep(CAN_DEFAULTS.connection_stabilization_delay)
            self.connected = True
            logger.info(
                f"Connected to motor CAN ID 0x{self.motor_can_id:02X} on {self.interface} "
                f"at {self.bitrate} bps"
            )
        except can.CanError as e:
            logger.warning(f"Failed to connect to CAN interface {self.interface}: {e}")
            self.connected = False
        except Exception as e:
            logger.warning(f"Unexpected error connecting to CAN bus: {e}")
            self.connected = False

    def enable_motor(self) -> None:
        """Power on the motor and enter Servo mode.

        Must be called once after connecting before any control command
        (set_velocity, set_position, etc.) will have any effect.
        """
        if not self.connected or self.bus is None:
            logger.warning("Cannot enable motor - CAN bus not connected")
            return

        try:
            msg = can.Message(
                arbitration_id=self.motor_can_id,
                data=bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]),
                is_extended_id=True,
            )
            self.bus.send(msg, timeout=0.1)
            logger.info("Motor enabled (Servo mode)")
            self._capture_response()
        except can.CanError as e:
            logger.error(f"Failed to enable motor: {e}")

    def disable_motor(self) -> None:
        """Power off the motor.

        The motor will coast to a stop and reject any further control
        commands until enable_motor() is called again.
        """
        if not self.connected or self.bus is None:
            logger.warning("Cannot disable motor - CAN bus not connected")
            return

        try:
            msg = can.Message(
                arbitration_id=self.motor_can_id,
                data=bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]),
                is_extended_id=True,
            )
            self.bus.send(msg, timeout=0.1)
            logger.info("Motor disabled")
            self._capture_response()
        except can.CanError as e:
            logger.error(f"Failed to disable motor: {e}")

    def _build_extended_id(self, mode: int) -> int:
        """Build extended 29-bit CAN ID with mode and motor ID.

        Format: 00 00 0M XX where M is control mode and XX is motor ID.

        :param mode: Control mode (from CANControlMode class).
        :return: Extended CAN arbitration ID.
        """
        extended_id = (mode << 8) | self.motor_can_id
        return extended_id

    def _send_can_command(self, mode: int, data: bytes) -> None:
        """Send CAN command to motor.

        :param mode: Control mode (from CANControlMode).
        :param data: 8-byte payload (padded with zeros if shorter).
        :return: None
        """
        if not self.connected or self.bus is None:
            logger.debug("CAN bus not connected - skipping command")
            return

        # Ensure data is exactly 8 bytes (pad with zeros if needed)
        if len(data) < 8:
            data = data + bytes(8 - len(data))
        elif len(data) > 8:
            data = data[:8]

        try:
            arbitration_id = self._build_extended_id(mode)
            msg = can.Message(
                arbitration_id=arbitration_id,
                data=data,
                is_extended_id=True,  # Use extended 29-bit CAN ID
            )

            self.bus.send(msg)
            logger.debug(
                f"TX CAN ID 0x{msg.arbitration_id:08X}: "
                f"{' '.join(f'{b:02X}' for b in msg.data)}"
            )
            # Motor sends exactly one 0x2903 status frame per received command.
            self._capture_response()
        except can.CanError as e:
            logger.error(f"CAN error sending command: {e}")

    def _parse_feedback_msg(self, msg: can.Message) -> CANMotorFeedback | None:
        """Parse a raw CAN message into a CANMotorFeedback if it matches our motor.

        :param msg: Raw python-can Message.
        :return: Parsed feedback or None if message is not valid motor feedback.
        """
        if msg.arbitration_id not in self._feedback_ids:
            return None
        if len(msg.data) < 8:
            logger.warning(f"Received short CAN message: {len(msg.data)} bytes")
            return None

        # Position: int16, range -32000..32000 → -3200°..3200° (scale × 0.1)
        pos_int = struct.unpack(">h", msg.data[0:2])[0]
        position_degrees = pos_int * 0.1

        # Speed: int16, range -32000..32000 → -320000..320000 ERPM (scale × 10)
        speed_int = struct.unpack(">h", msg.data[2:4])[0]
        speed_erpm = speed_int * 10

        # Current: int16, range -6000..6000 → -60..60 A (scale × 0.01)
        current_int = struct.unpack(">h", msg.data[4:6])[0]
        current_amps = current_int * 0.01

        # Temperature: int8, range -20..127 → -20..127 °C (driver board, direct)
        temperature_celsius = struct.unpack("b", bytes([msg.data[6]]))[0]

        # Error code: uint8 (0=OK, 1-7 see CAN_ERROR_CODES)
        error_code = msg.data[7]

        feedback = CANMotorFeedback(
            position_degrees=position_degrees,
            speed_erpm=speed_erpm,
            current_amps=current_amps,
            temperature_celsius=temperature_celsius,
            error_code=error_code,
        )

        logger.debug(
            f"RX Feedback 0x{msg.arbitration_id:04X}: "
            f"Pos={position_degrees:.1f}°  Speed={speed_erpm} ERPM  "
            f"Cur={current_amps:.2f} A  Tmp={temperature_celsius} °C  Err={error_code}"
        )
        return feedback

    def _capture_response(self, timeout: float = 0.05) -> None:
        """Read the motor's one-shot response frame and store it as pending feedback.

        The AK60-6 (in its default CubeMars firmware mode) sends exactly one
        0x2903 status frame in response to each command it receives.  This
        method should be called immediately after every bus.send() so the
        response is buffered and available to the next _receive_feedback() call.

        :param timeout: How long to wait for the response frame (seconds).
        :return: None
        """
        if not self.connected or self.bus is None:
            return
        try:
            msg = self.bus.recv(timeout=timeout)
            if msg is None:
                return
            fb = self._parse_feedback_msg(msg)
            if fb is not None:
                self._pending_feedback = fb
                self._last_feedback = fb
                self._consecutive_no_response = 0
                self.communicating = True
            else:
                logger.debug(
                    f"_capture_response: ignoring 0x{msg.arbitration_id:08X} "
                    f"[{' '.join(f'{b:02X}' for b in msg.data)}]"
                )
        except (can.CanError, Exception) as e:
            logger.debug(f"_capture_response error: {e}")

    def _receive_feedback(self, timeout: float = 0.1) -> CANMotorFeedback | None:
        """Return the most recent motor status frame.

        The AK60-6 operates in response-only mode: it sends one 0x2903 status
        frame per command it receives (not a continuous broadcast).  Each
        _send_can_command / enable_motor / disable_motor call automatically
        captures that response via _capture_response().  This method returns
        the buffered result immediately, or falls back to a live bus.recv()
        if nothing is pending (e.g. when called independently).

        Feedback frame format (CAN ID 0x2903, extended 29-bit):
        - Data[0-1]: Position int16 x 0.1 = degrees
        - Data[2-3]: Speed    int16 x 10  = ERPM
        - Data[4-5]: Current  int16 x 0.01 = Amps
        - Data[6]:   Temperature int8 = °C
        - Data[7]:   Error code uint8

        :param timeout: Fallback bus recv timeout (seconds).
        :return: CANMotorFeedback or None.
        """
        # Fast path: return the response already captured by _capture_response.
        if self._pending_feedback is not None:
            fb = self._pending_feedback
            self._pending_feedback = None
            return fb

        if not self.connected or self.bus is None:
            return None

        # Slow path: wait on the bus (caller sent a command externally, or
        # motor is configured for periodic broadcast).
        try:
            msg = self.bus.recv(timeout=timeout)
            if msg is None:
                self._consecutive_no_response += 1
                return None

            fb = self._parse_feedback_msg(msg)
            if fb is None:
                logger.debug(
                    f"_receive_feedback: ignoring 0x{msg.arbitration_id:08X} "
                    f"[{' '.join(f'{b:02X}' for b in msg.data)}] "
                    f"(want one of {[hex(x) for x in sorted(self._feedback_ids)]})"
                )
                return None

            self._last_feedback = fb
            self._consecutive_no_response = 0
            self.communicating = True
            return fb

        except (can.CanError, Exception) as e:
            logger.error(f"CAN error receiving feedback: {e}")
            return None

    def _refresh_loop(self) -> None:
        """Background thread: re-send the active command at 50 Hz until stopped."""
        while not self._refresh_stop.is_set():
            if self._refresh_mode is not None and self._refresh_data is not None:
                if self.connected and self.bus is not None:
                    try:
                        arb_id = self._build_extended_id(self._refresh_mode)
                        self.bus.send(
                            can.Message(
                                arbitration_id=arb_id,
                                data=self._refresh_data,
                                is_extended_id=True,
                            )
                        )
                        # Capture the one-shot reply without blocking long
                        msg = self.bus.recv(timeout=0.015)
                        if msg and msg.arbitration_id in self._feedback_ids:
                            fb = self._parse_feedback_msg(msg)
                            if fb:
                                self._last_feedback = fb
                                self.communicating = True
                    except can.CanError:
                        pass
            self._refresh_stop.wait(self._refresh_interval)

    def _start_refresh(self, mode: int, data: bytes) -> None:
        """Store command and ensure refresh thread is running.

        :param mode: CANControlMode constant.
        :param data: 8-byte command payload.
        """
        # Pad/truncate to exactly 8 bytes
        if len(data) < 8:
            data = data + bytes(8 - len(data))
        elif len(data) > 8:
            data = data[:8]
        self._refresh_mode = mode
        self._refresh_data = data
        if self._refresh_thread is None or not self._refresh_thread.is_alive():
            self._refresh_stop.clear()
            self._refresh_thread = threading.Thread(
                target=self._refresh_loop, daemon=True, name="can-refresh"
            )
            self._refresh_thread.start()
            logger.debug("Refresh thread started")

    def _stop_refresh(self) -> None:
        """Stop the background refresh thread and clear the active command."""
        self._refresh_mode = None
        self._refresh_data = None
        self._refresh_stop.set()
        if self._refresh_thread is not None:
            self._refresh_thread.join(timeout=0.2)
            self._refresh_thread = None
        self._refresh_stop.clear()
        logger.debug("Refresh thread stopped")

    def _soft_start(self, direction: int) -> None:
        """Pre-spin motor with gentle current to pass the noisy low-speed zone.

        The firmware's velocity PID has a fixed high acceleration that causes
        current oscillations and audible noise at low speeds (0-5000 ERPM).
        By first commanding a moderate current, the motor accelerates gently
        under direct torque control (no velocity PID) until it is past the
        noisy zone, then the caller switches to velocity mode.

        Uses the refresh thread so the current command is held continuously
        for the soft-start duration (respects the motor watchdog).

        :param direction: 1 for forward, -1 for reverse.
        :return: None
        """
        current_ma = int(MOTOR_LIMITS.soft_start_current_ma * direction)
        data = struct.pack(">i", current_ma)
        self._start_refresh(CANControlMode.CURRENT_LOOP, data)
        time.sleep(MOTOR_LIMITS.soft_start_duration)
        # Caller will immediately call _start_refresh with velocity/duty which
        # overwrites the stored command in place — no explicit stop needed.

    def _estimate_movement_time(
        self, target_degrees: float, motor_speed_erpm: int
    ) -> float:
        """Estimate time needed to reach target position at given speed.

        Converts ERPM to degrees/second and calculates travel time from the
        current reported position (obtained via CAN feedback).

        :param target_degrees: Target position in degrees.
        :param motor_speed_erpm: Motor speed in ERPM (absolute value used).
        :return: Estimated travel time in seconds. Returns 0.0 if speed is zero.
        """
        if motor_speed_erpm == 0:
            return 0.0

        current_position = self.get_position() or 0.0

        # ERPM -> degrees/second: (ERPM / 60) * 360
        degrees_per_second = abs(motor_speed_erpm) / 60.0 * 360.0
        distance = abs(target_degrees - current_position)
        return distance / degrees_per_second

    def set_position(self, position_degrees: float) -> None:
        """Move the motor to a target angle and hold it there.

        The motor moves at its firmware-default speed and holds the
        position against any external load.  For a controlled-speed move
        use set_position_velocity_accel() or move_to_position_with_speed().

        :param position_degrees: Target angle in degrees (-3200° to 3200°).
            Values outside this range are clamped automatically.
        :return: None
        """
        # Clamp to CAN protocol feedback limits
        position_degrees = np.clip(position_degrees, -3200.0, 3200.0)

        # Convert to int32: degrees * 10000
        # Example from doc: 600 degrees = 0x005B8D80 = 6000000
        position_int = int(position_degrees * 10000.0)

        # Pack as big-endian int32
        data = struct.pack(">i", position_int)

        self._start_refresh(CANControlMode.POSITION_LOOP, data)
        logger.info(f"Set position: {position_degrees:.2f}°")

    def get_position(self) -> float | None:
        """Return the current motor angle in degrees.

        Reads from the most recent CAN feedback frame.  Returns the last
        known position if no new frame arrives within 0.2 s.

        :return: Current angle in degrees, or None if the motor has never
            responded (e.g. not yet enabled or cable disconnected).
        """
        # Listen for feedback message
        feedback = self._receive_feedback(timeout=0.2)
        if feedback:
            return feedback.position_degrees
        elif self._last_feedback:
            return self._last_feedback.position_degrees
        return None

    def set_velocity(self, velocity_erpm: int, allow_low_speed: bool = False) -> None:
        """Spin the motor continuously at the given speed.

        Positive values spin forward (tendon pull direction), negative values
        spin in reverse (tendon release direction).  The motor keeps spinning
        until stop() or another command is issued.

        Safe operating range is ±5 000–100 000 ERPM.  Speeds between 1 and
        4 999 ERPM cause current oscillations and audible noise due to the
        firmware's velocity PID — they are blocked by default.
        Passing velocity_erpm=0 is equivalent to calling stop().

        :param velocity_erpm: Target speed in electrical RPM.  Negative = reverse.
            Typical exosuit values: 8 000–15 000 ERPM.
        :param allow_low_speed: Set True to bypass the 5 000 ERPM safety floor.
        :return: None
        :raises ValueError: If 1 ≤ abs(velocity_erpm) < 5 000 and allow_low_speed=False.
        """
        velocity_erpm_int = int(velocity_erpm)

        # Velocity 0 means stop -- use current=0 to release windings cleanly
        if velocity_erpm_int == 0:
            self.stop()
            return

        # Block dangerously low speeds unless explicitly allowed
        if not allow_low_speed:
            if 0 < abs(velocity_erpm_int) < MOTOR_LIMITS.min_safe_velocity_erpm:
                raise ValueError(
                    f"Velocity {velocity_erpm_int} ERPM below safe threshold "
                    f"({MOTOR_LIMITS.min_safe_velocity_erpm} ERPM min). "
                    f"Use allow_low_speed=True to bypass."
                )

        # Clamp to protocol limits (based on feedback range -320000 to 320000)
        velocity_erpm = np.clip(velocity_erpm_int, -320000, 320000)
        if velocity_erpm != velocity_erpm_int:
            logger.warning(
                f"Velocity {velocity_erpm_int} ERPM clamped to {velocity_erpm} ERPM"
            )

        # Soft-start: pre-spin with current to bypass the noisy low-speed zone
        # (0-5000 ERPM region causes oscillations under velocity PID).
        # Skip if already running in velocity mode — avoids thrashing CURRENT
        # → VELOCITY → CURRENT on every P-controller iteration.
        direction = 1 if velocity_erpm > 0 else -1
        already_in_velocity = self._refresh_mode == CANControlMode.VELOCITY_LOOP
        if not already_in_velocity:
            self._soft_start(direction)

        # Pack as int32 directly (no scaling needed based on examples)
        # Example from doc: 5000 ERPM = 0x00001388
        data = struct.pack(">i", velocity_erpm)

        self._start_refresh(CANControlMode.VELOCITY_LOOP, data)
        logger.info(f"Set velocity: {velocity_erpm} ERPM")

    def set_current(self, current_amps: float) -> None:
        """Command a specific motor torque via direct current control.

        Bypasses the velocity PID — the motor accelerates or holds a load
        purely by the requested phase current.  Positive = forward torque,
        negative = reverse torque.  Useful for compliant or gentle motion.

        :param current_amps: Target phase current in amps (-60 to 60 A).
            Values outside this range are clamped automatically.
        :return: None
        """
        # Clamp to protocol limits
        current_amps = np.clip(current_amps, -60.0, 60.0)

        # Convert to int32: amps * 1000
        # Example from doc: -4.4 A = 0xFFFFEED8 = -4400
        current_int = int(current_amps * 1000.0)

        # Pack as big-endian int32
        data = struct.pack(">i", current_int)

        self._start_refresh(CANControlMode.CURRENT_LOOP, data)
        logger.info(f"Set current: {current_amps:.2f} A")

    def set_duty_cycle(self, duty: float) -> None:
        """Apply a raw PWM voltage to the motor windings.

        No speed or current feedback loop — output is a fixed fraction of
        the supply voltage.  Useful for open-loop testing; prefer
        set_velocity() or set_current() for exosuit control.

        :param duty: Fraction of supply voltage from -1.0 (full reverse)
            to 1.0 (full forward).  Values outside this range are clamped.
        :return: None
        """
        duty = float(np.clip(duty, -1.0, 1.0))

        # Convert to int32: fraction * 100000
        duty_int = int(duty * 100000.0)

        # Pack as big-endian int32
        data = struct.pack(">i", duty_int)

        self._start_refresh(CANControlMode.DUTY_CYCLE, data)
        logger.info(f"Set duty cycle: {duty:.4f} ({duty * 100:.1f}%)")

    def set_origin(self, permanent: bool = False) -> None:
        """Define the current motor angle as the new zero reference (home position).

        After this call, get_position() will return 0.0 for the current
        physical position.  Use permanent=True to save it to the motor's
        internal memory so it survives power cycles.

        :param permanent: If True, the new origin is saved to EEPROM and
            survives power-off.  If False (default), it resets on next
            power cycle.
        :return: None
        """
        origin_byte = 0x02 if permanent else 0x01
        data = bytes([origin_byte])  # _send_can_command pads to 8 bytes

        self._send_can_command(CANControlMode.SET_ORIGIN, data)
        persistence = (
            "permanent (saved to EEPROM)"
            if permanent
            else "temporary (until power cycle)"
        )
        logger.info(f"Set origin: {persistence}")

    def set_position_velocity_accel(
        self,
        position_degrees: float,
        velocity_erpm: int,
        accel_erpm_per_sec: int = 0,
    ) -> None:
        """Move to a target angle with controlled speed and acceleration.

        Produces a smooth trapezoidal motion profile: the motor accelerates
        up to velocity_erpm, travels at that speed, then decelerates into
        the target angle and holds it.  This is the preferred command for
        precise, smooth exosuit joint movements.

        :param position_degrees: Target angle in degrees (-3200° to 3200°).
        :param velocity_erpm: Maximum travel speed in ERPM.  0 = firmware
            default (usually very fast — always set a value).
            Typical exosuit value: 8 000–15 000 ERPM.
        :param accel_erpm_per_sec: Maximum acceleration in ERPM/s.  0 =
            firmware default.  Lower values give smoother, gentler motion.
        :return: None
        """
        position_degrees = float(np.clip(position_degrees, -3200.0, 3200.0))
        position_int = int(position_degrees * 10000.0)

        # Velocity and accel are signed 16-bit, scaled /10 per the CAN protocol spec.
        # Doc example: 10000 ERPM → raw 0x03E8 (1000), so raw = ERPM / 10.
        # Signs are preserved so direction is encoded correctly.
        velocity_int = int(np.clip(velocity_erpm, -327670, 327670) // 10)
        accel_int = int(np.clip(accel_erpm_per_sec, -327670, 327670) // 10)

        # Pack: int32 + int16 + int16 = 8 bytes, big-endian
        data = struct.pack(">ihh", position_int, velocity_int, accel_int)

        self._start_refresh(CANControlMode.POSITION_VELOCITY, data)
        logger.info(
            f"Set position: {position_degrees:.2f}° at {velocity_erpm} ERPM, "
            f"accel {accel_erpm_per_sec} ERPM/s"
        )

    def get_status(self) -> CANMotorFeedback | None:
        """Read and log the full motor state.

        Returns a CANMotorFeedback dataclass with:
          - position_degrees    — current angle in degrees
          - speed_erpm          — current speed in electrical RPM
          - current_amps        — phase current draw in amps
          - temperature_celsius — driver board temperature
          - error_code          — 0 = no fault (see FaultCode in definitions.py)

        :return: CANMotorFeedback dataclass, or None if the motor does not
            respond within 0.5 s.
        """
        feedback = self._receive_feedback(timeout=0.5)
        if feedback:
            logger.info("=" * 50)
            logger.info("MOTOR STATUS:")
            logger.info(f"  Position: {feedback.position_degrees:.2f}°")
            logger.info(f"  Speed: {feedback.speed_erpm} ERPM")
            logger.info(f"  Current: {feedback.current_amps:.2f} A")
            logger.info(f"  Temperature: {feedback.temperature_celsius} °C")
            logger.info(f"  Error Code: {feedback.error_code} — {feedback.error_description}")
            logger.info("=" * 50)
        return feedback

    def get_temperature(self) -> int | None:
        """Return the motor/driver board temperature in °C, or None if unavailable."""
        fb = self._last_feedback or self._receive_feedback(timeout=0.5)
        return fb.temperature_celsius if fb else None

    def get_current(self) -> float | None:
        """Return the phase current draw in amps, or None if unavailable."""
        fb = self._last_feedback or self._receive_feedback(timeout=0.5)
        return fb.current_amps if fb else None

    def get_speed(self) -> int | None:
        """Return the current speed in electrical RPM, or None if unavailable."""
        fb = self._last_feedback or self._receive_feedback(timeout=0.5)
        return fb.speed_erpm if fb else None

    def get_motor_data(self) -> dict | None:
        """Return all motor telemetry as a dictionary.

        Keys: position_degrees, speed_erpm, current_amps,
              temperature_celsius, error_code.
        Returns None if the motor does not respond.
        """
        fb = self._last_feedback or self._receive_feedback(timeout=0.5)
        if fb is None:
            return None
        return {
            "position_degrees": fb.position_degrees,
            "speed_erpm": fb.speed_erpm,
            "current_amps": fb.current_amps,
            "temperature_celsius": fb.temperature_celsius,
            "error_code": fb.error_code,
            "error_description": fb.error_description,
        }

    def check_communication(self) -> bool:
        """Check whether the motor is alive and responding over CAN.

        Sends no command — just listens for a feedback frame.  Call this
        after enable_motor() to confirm the motor is ready before sending
        control commands.

        :return: True if at least one feedback frame is received, False if
            the motor is silent (check power, wiring, and CAN ID).
        """
        if not self.connected:
            return False

        # Try to receive feedback multiple times
        for _attempt in range(MOTOR_DEFAULTS.max_communication_attempts):
            feedback = self._receive_feedback(timeout=0.5)
            if feedback:
                self.communicating = True
                self._consecutive_no_response = 0
                logger.info("Motor communication verified")
                return True
            time.sleep(MOTOR_DEFAULTS.communication_retry_delay)

        logger.warning("Motor not responding - no feedback messages received")
        self.communicating = False
        return False

    def move_to_position_with_speed(
        self,
        target_degrees: float,
        motor_speed_erpm: int,
        step_delay: float = MOTOR_DEFAULTS.step_delay,
    ) -> None:
        """Drive to a target angle at a given speed, then hold it.

        The motor first spins at motor_speed_erpm toward the target
        (direction is determined automatically from the current position),
        waits for the estimated travel time, then switches to position-hold.
        For a single-command trapezoidal profile, prefer
        set_position_velocity_accel() instead.

        :param target_degrees: Target angle in degrees.
        :param motor_speed_erpm: Travel speed in ERPM (absolute value —
            direction is chosen automatically).  Min safe value: 5 000 ERPM.
        :param step_delay: Unused in CAN mode (kept for API compatibility).
        :return: None
        """
        # Determine direction from current position
        current_pos = self.get_position() or 0.0
        direction = 1 if target_degrees > current_pos else -1

        # Use velocity control to move toward target
        self.set_velocity(
            velocity_erpm=motor_speed_erpm * direction, allow_low_speed=True
        )

        # Wait for estimated travel time (capped at max_movement_time safety limit)
        estimated_time = self._estimate_movement_time(target_degrees, motor_speed_erpm)
        time.sleep(min(estimated_time, MOTOR_LIMITS.max_movement_time))

        # Switch to position hold
        self.set_position(target_degrees)
        logger.info(
            f"Reached position: {target_degrees:.1f}° at {motor_speed_erpm} ERPM"
        )

    def control_exosuit_tendon(
        self,
        action: TendonAction,
        velocity_erpm: int = MOTOR_LIMITS.default_tendon_velocity_erpm,
    ) -> None:
        """High-level tendon command — the primary function for the hip controller.

        Translates a simple PULL / RELEASE / STOP intent into the correct
        motor velocity command.  This hides all ERPM direction logic.

        Example::

            motor.control_exosuit_tendon(TendonAction.PULL)    # lift / assist
            motor.control_exosuit_tendon(TendonAction.RELEASE) # lower / relax
            motor.control_exosuit_tendon(TendonAction.STOP)    # hold still

        :param action: TendonAction.PULL, TendonAction.RELEASE, or
            TendonAction.STOP (import from motor_python.definitions).
        :param velocity_erpm: Speed for PULL and RELEASE in ERPM.
            Default is 10 000 ERPM.  Minimum safe value is 5 000 ERPM.
        :return: None
        :raises ValueError: If action is not a valid TendonAction.
        """
        if action == TendonAction.PULL:
            logger.info(f"Pulling tendon at {velocity_erpm} ERPM")
            self.set_velocity(velocity_erpm=abs(velocity_erpm))
        elif action == TendonAction.RELEASE:
            logger.info(f"Releasing tendon at {velocity_erpm} ERPM")
            self.set_velocity(velocity_erpm=-abs(velocity_erpm))
        elif action == TendonAction.STOP:
            logger.info("Stopping tendon motion")
            self.stop()
        else:
            raise ValueError(
                f"Invalid action {action}. Use TendonAction.PULL, TendonAction.RELEASE, or TendonAction.STOP"
            )

    def stop(self) -> None:
        """Stop the motor immediately and release the windings.

        Sets current to zero — the rotor coasts to a mechanical stop with
        no active braking.  Safe to call at any time.

        :return: None
        """
        self._stop_refresh()
        # Send zero-current command twice for reliable stop
        self._send_can_command(CANControlMode.CURRENT_LOOP, struct.pack(">i", 0))
        time.sleep(0.05)
        self._send_can_command(CANControlMode.CURRENT_LOOP, struct.pack(">i", 0))
        time.sleep(0.1)
        logger.info("Motor stopped (current=0, windings released)")

    def close(self) -> None:
        """Stop the motor and release the CAN bus connection.

        Called automatically when using the motor as a context manager
        (``with Motor() as motor:``).  Safe to call manually otherwise.

        :return: None
        """
        if self.bus:
            self._stop_refresh()
            self._send_can_command(CANControlMode.CURRENT_LOOP, struct.pack(">i", 0))
            time.sleep(0.05)
            self.bus.shutdown()
            logger.info("CAN bus connection closed")

    def __enter__(self) -> "CubeMarsAK606v3CAN":
        """Context manager entry.

        :return: Self instance for use in with statement.
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit.

        :param exc_type: Exception type if an exception occurred.
        :param exc_val: Exception value if an exception occurred.
        :param exc_tb: Exception traceback if an exception occurred.
        :return: None
        """
        self.close()
