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


@dataclass
class CANMotorFeedback:
    """Motor feedback data from CAN upload message (8 bytes).

    Real-time motor state transmitted periodically at configured rate (1-500 Hz).
    """

    position_degrees: float  # Motor position in degrees (-3200° to 3200°)
    speed_erpm: int  # Electrical speed in RPM (-320000 to 320000)
    current_amps: float  # Motor current in amps (-60 to 60 A)
    temperature_celsius: int  # Driver board temperature (-20 to 127deg C)
    error_code: int  # Error code (0=no fault, 1-7=various faults)


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
            # Initialize SocketCAN interface
            self.bus = can.interface.Bus(
                channel=self.interface,
                interface="socketcan",
                bitrate=self.bitrate,
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
        """Enable the motor for Servo mode control.

        Sends power-on command 0xFFFFFFFFFFFFFFFC to motor ID.
        Must be called before sending control commands.
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
        """Disable the motor (power off).

        Sends power-off command 0xFFFFFFFFFFFFFFFD to motor ID.
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

        # Position: int16 x 0.1 = degrees
        pos_int = struct.unpack(">h", msg.data[0:2])[0]
        position_degrees = pos_int * 0.1

        # Speed: int16 x 10 = ERPM
        speed_int = struct.unpack(">h", msg.data[2:4])[0]
        speed_erpm = speed_int * 10

        # Current: int16 x 0.01 = Amps
        current_int = struct.unpack(">h", msg.data[4:6])[0]
        current_amps = current_int * 0.01

        # Temperature: int8 direct deg C
        temperature_celsius = struct.unpack("b", bytes([msg.data[6]]))[0]

        # Error code: uint8
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
            f"Cur={current_amps:.2f}A  Tmp={temperature_celsius}deg C  Err={error_code}"
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
        - Data[6]:   Temperature int8 = deg C
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
        """Set motor target position in degrees using CAN Position Loop mode.

        Sends position command via CAN extended ID 0x00000468 (mode 04, motor ID in example 68).
        Position is sent as int32 value (degrees * 10000).
        Example: 600° = 6000000 = 0x005B8D80

        :param position_degrees: Target position in degrees (-3200° to 3200°).
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
        """Get current motor position from periodic feedback.

        :return: Current position in degrees, or None if no recent feedback.
        """
        # Listen for feedback message
        feedback = self._receive_feedback(timeout=0.2)
        if feedback:
            return feedback.position_degrees
        elif self._last_feedback:
            return self._last_feedback.position_degrees
        return None

    def set_velocity(self, velocity_erpm: int, allow_low_speed: bool = False) -> None:
        """Set motor velocity in electrical RPM using CAN Velocity Loop mode.

        Sends velocity command via CAN extended ID 0x00000368 (mode 03).
        Velocity is sent as int32 value (ERPM directly).
        Example: 5000 ERPM = 0x00001388, -5000 ERPM = 0xFFFFEC78

        Low speeds (<5000 ERPM) with high firmware acceleration cause current
        oscillations and audible noise. Use medium-to-high speeds for exosuit.

        :param velocity_erpm: Target velocity in ERPM (safe range: 5000-100000)
        :param allow_low_speed: Bypass the 5000 ERPM safety floor (default: False)
        :return: None
        :raises ValueError: If velocity below safe threshold and allow_low_speed=False
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
        direction = 1 if velocity_erpm > 0 else -1
        self._soft_start(direction)

        # Pack as int32 directly (no scaling needed based on examples)
        # Example from doc: 5000 ERPM = 0x00001388
        data = struct.pack(">i", velocity_erpm)

        self._start_refresh(CANControlMode.VELOCITY_LOOP, data)
        logger.info(f"Set velocity: {velocity_erpm} ERPM")

    def set_current(self, current_amps: float) -> None:
        """Set motor current in amps using CAN Current Loop mode.

        Sends current command via CAN extended ID 0x00000168 (mode 01).
        Current is sent as int32 value (amps * 1000).
        Example: -4.4 A (IQ) = -4400 = 0xFFFFEED8 (from doc example)

        :param current_amps: Target current in amps (-60 to 60 A).
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
        """Set motor duty cycle (square-wave voltage) using CAN Duty Cycle mode.

        Sends duty command via CAN extended ID 0x00000003 (mode 00).
        Duty is encoded as int32 (duty * 100000).
        Example: 0.5 (50%) = 50000 = 0x0000C350, -1.0 = -100000 = 0xFFFE7960

        :param duty: Duty cycle fraction from -1.0 (full reverse) to 1.0 (full forward).
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
        """Set current motor position as the new zero origin.

        Sends set-origin command via CAN extended ID 0x00000503 (mode 05).
        Data byte 0: 0x01 = temporary (lost on power cycle), 0x02 = permanent (EEPROM).

        :param permanent: If True, saves origin to EEPROM (survives power cycle).
                          If False, origin is temporary until next power-on.
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
        """Set motor position with velocity and acceleration limits (trapezoidal profile).

        Sends command via CAN extended ID 0x00000603 (mode 06).
        Payload encoding (8 bytes, big-endian):
          - Bytes 0-3: int32 position (degrees * 10000)
          - Bytes 4-5: int16 velocity limit (ERPM, 0 = no limit / use firmware default)
          - Bytes 6-7: int16 acceleration limit (ERPM/s, 0 = no limit)

        Example: 90° at 10000 ERPM → position_int=900000, vel_int=10000, accel_int=0

        :param position_degrees: Target position in degrees (-3200° to 3200°).
        :param velocity_erpm: Maximum velocity during move in ERPM (0 = firmware default).
        :param accel_erpm_per_sec: Maximum acceleration in ERPM/s (0 = firmware default).
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
            f"Set position: {position_degrees:.2f}° at {velocity_int} ERPM, "
            f"accel {accel_int} ERPM/s"
        )

    def get_status(self) -> CANMotorFeedback | None:
        """Get motor status from periodic feedback.

        In CAN mode, the motor automatically sends feedback at configured rate.
        This method listens for the next feedback message.

        :return: CANMotorFeedback object with current motor state, or None.
        """
        feedback = self._receive_feedback(timeout=0.5)
        if feedback:
            logger.info("=" * 50)
            logger.info("MOTOR STATUS:")
            logger.info(f"  Position: {feedback.position_degrees:.2f}°")
            logger.info(f"  Speed: {feedback.speed_erpm} ERPM")
            logger.info(f"  Current: {feedback.current_amps:.2f} A")
            logger.info(f"  Temperature: {feedback.temperature_celsius}deg C")
            logger.info(f"  Error Code: {feedback.error_code}")
            logger.info("=" * 50)
        return feedback

    def check_communication(self) -> bool:
        """Verify motor is responding by listening for feedback messages.

        :return: True if motor responds, False otherwise
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
        """Reach a target position using velocity control then hold with position.

        :param target_degrees: Target position in degrees.
        :param motor_speed_erpm: Motor speed in ERPM (absolute, direction auto).
        :param step_delay: Delay between steps in seconds (ignored in CAN mode).
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
        """Control exosuit tendon using safe velocity commands.

        :param action: TendonAction enum (PULL, RELEASE, or STOP)
        :param velocity_erpm: Velocity in ERPM (default: 10000, min: 5000)
        :return: None
        :raises ValueError: If action is invalid
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
        """Stop the motor by halting the refresh thread and zeroing current.

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
        """Close CAN bus connection to motor.

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
