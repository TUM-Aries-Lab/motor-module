"""AK60-6 Motor Control Class - CubeMars Native CAN Protocol."""

import struct
import threading
import time

import can
import numpy as np
from loguru import logger

from motor_python.base_motor import BaseMotor, MotorState
from motor_python.can_protocol import CANControlMode
from motor_python.can_utils import get_can_state, reset_can_interface
from motor_python.definitions import (
    CAN_DEFAULTS,
    MOTOR_DEFAULTS,
    MOTOR_LIMITS,
)


class CubeMarsAK606v3CAN(BaseMotor):
    """AK60-6 Motor Controller using native CAN bus protocol.

    Inherits shared safety checks, tendon helpers, and context-manager
    protocol from :class:`BaseMotor`.

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

    # ── CAN protocol command payloads (CubeMars spec, section 4.4.1) ───────────
    # These 8-byte sequences use the base motor_id arbitration ID (no mode prefix).
    # Defined as class constants to avoid inline magic-byte duplication.
    _CAN_SERVO_ENABLE: bytes = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
    _CAN_SERVO_DISABLE: bytes = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
    _CAN_MIT_ENABLE: bytes = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
    _CAN_MIT_DISABLE: bytes = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])

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
        super().__init__()
        self.motor_can_id = motor_can_id
        self.interface = interface
        self.bitrate = bitrate
        self.bus: can.BusABC | None = None
        self._last_feedback: MotorState | None = None
        # Feedback from the most-recent command response (consumed by next
        # _receive_feedback call).  The motor operates in response-only mode:
        # it sends exactly one 0x2903 status frame per command it receives.
        self._pending_feedback: MotorState | None = None
        # Most recent feedback captured by the background refresh thread.
        # Cleared by _start_refresh() so _receive_feedback() never returns a
        # stale one-shot response (e.g. the enable ACK) when polling refresh data.
        self._refresh_feedback: MotorState | None = None

        # Background refresh thread — re-sends the active command at 50 Hz.
        # The motor watchdog cuts power if no command arrives within ~100 ms.
        self._refresh_mode: int | None = None
        self._refresh_data: bytes | None = None
        self._refresh_stop = threading.Event()
        self._refresh_thread: threading.Thread | None = None
        self._refresh_interval: float = 1.0 / CAN_DEFAULTS.feedback_rate_hz  # 50 Hz

        # Build the set of candidate feedback IDs so _receive_feedback can
        # accept whichever scheme this firmware version uses.
        #   0x2900 | id  — extended-ID scheme documented in CAN journey notes
        #   0x0080 | id  — standard-frame scheme (0x83 for id=3)
        #   id itself    — direct-match fallback
        #   id + 1       — CubeMars firmware variant: feedback ID = motor_id + 1
        #                  (observed: motor_id=0x03 → feedback on 0x04)
        #   0x2900       — enable-response frame seen in candump (base, no motor_id suffix)
        self._feedback_ids: set[int] = {
            motor_can_id,
            motor_can_id + 1,
            0x2900 | motor_can_id,
            0x0080 | motor_can_id,
            0x2900,  # enable-response frame observed on bus (motor_id=0x03 setup)
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

        Checks the CAN controller state first and resets the interface if it
        is in ERROR-PASSIVE or BUS-OFF (accumulated errors from a previous
        run).  Then opens the SocketCAN bus with kernel-level receive filters
        that accept every known motor feedback ID while blocking unrelated
        traffic (e.g. the 0x0088 standard-frame flood).

        :return: None
        """
        try:
            # ── Pre-flight: ensure the CAN controller is healthy ──────────
            bus_state = get_can_state(self.interface)
            logger.info(
                f"CAN bus state: {bus_state['state']}  "
                f"(tx_err={bus_state['tx_err']} rx_err={bus_state['rx_err']})"
            )
            if bus_state["state"] == "BUS-OFF":
                # BUS-OFF: controller locked out.  setup_can.sh sets restart-ms 100
                # so the kernel auto-restarts after 100 ms.  Wait up to 500 ms.
                logger.warning(
                    "CAN bus in BUS-OFF — waiting for restart-ms auto-recovery"
                )
                for _ in range(5):
                    time.sleep(0.15)
                    bus_state = get_can_state(self.interface)
                    if bus_state["state"] != "BUS-OFF":
                        break
                logger.info(f"After BUS-OFF recovery: {bus_state['state']}")
            elif bus_state["state"] in ("ERROR-PASSIVE", "ERROR-WARNING"):
                # ERROR-PASSIVE / WARNING: high error counters but TX/RX still
                # works.  Do NOT call _reset_can_interface() — ip link down/up
                # causes the motor to enter BUS-OFF and go completely silent.
                # Just log and proceed; the socket will communicate normally.
                logger.warning(
                    f"CAN bus in {bus_state['state']} "
                    f"(tx_err={bus_state['tx_err']}) — proceeding without reset"
                )

            # ── Kernel CAN filters ────────────────────────────────────────
            # Accept every ID the motor firmware is known to use for
            # feedback / ACKs.  The 0x0088 standard-frame flood from
            # unrelated devices is blocked because it matches none of
            # these filters.
            #
            # Known response IDs (motor_id = 0x03):
            #   0x2903 — extended 29-bit status (mode 0x29 | motor_id)
            #   0x2900 — extended 29-bit enable ACK
            #   0x0004 — extended 29-bit (motor_id + 1, firmware variant)
            #   0x0003 — extended 29-bit (motor_id itself, direct echo)
            #   0x0083 — extended 29-bit (0x80 | motor_id, variant)
            can_filters = [
                {
                    "can_id": 0x2900 | self.motor_can_id,
                    "can_mask": 0x1FFFFFFF,
                    "extended": True,
                },
                {"can_id": 0x2900, "can_mask": 0x1FFFFFFF, "extended": True},
                {
                    "can_id": self.motor_can_id + 1,
                    "can_mask": 0x1FFFFFFF,
                    "extended": True,
                },
                {"can_id": self.motor_can_id, "can_mask": 0x1FFFFFFF, "extended": True},
                {
                    "can_id": 0x0080 | self.motor_can_id,
                    "can_mask": 0x1FFFFFFF,
                    "extended": True,
                },
                # NOTE: No standard-frame (11-bit) filter entry.
                # The motor only sends extended 29-bit feedback frames.
                # Adding a permissive standard-frame entry was found to
                # confuse mttcan's hardware filter on Jetson.
            ]
            self._can_filters = can_filters  # stored so _start_refresh can re-apply
            self.bus = can.interface.Bus(
                channel=self.interface,
                interface="socketcan",
                bitrate=self.bitrate,
                can_filters=can_filters,
            )
            # Suppress CAN error frames from reaching this socket.
            # Jetson's mttcan re-delivers each CAN_ERR_BUSERROR/CAN_ERR_PROT
            # error frame thousands of times, flooding the receive buffer and
            # making real 0x2903 feedback frames unreachable.  Setting
            # CAN_RAW_ERR_FILTER=0 tells the kernel not to route any error
            # frames to this socket at all.
            try:
                import socket as _socket
                import struct as _struct

                # CAN_RAW_ERR_FILTER = 2 (not always exported by the socket module)
                _CAN_RAW_ERR_FILTER = getattr(_socket, "CAN_RAW_ERR_FILTER", 2)
                self.bus.socket.setsockopt(  # type: ignore[union-attr]
                    _socket.SOL_CAN_RAW,
                    _CAN_RAW_ERR_FILTER,
                    _struct.pack("=I", 0),
                )
                logger.info("CAN error frames suppressed (CAN_RAW_ERR_FILTER=0)")
            except Exception as exc:
                logger.warning(f"Could not suppress CAN error frames: {exc}")
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
                data=self._CAN_SERVO_ENABLE,
                is_extended_id=True,
            )
            # Retry up to 3 times — the motor may need a few stimulations to
            # respond after a quiet period (watchdog fired, post-reset, etc.).
            for _attempt in range(3):
                self.bus.send(msg, timeout=0.1)
                self._capture_response()
                if self._pending_feedback is not None:
                    break
                if _attempt < 2:
                    time.sleep(0.1)
            logger.info("Motor enabled (Servo mode)")
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
                data=self._CAN_SERVO_DISABLE,
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

    def _parse_feedback_msg(self, msg: can.Message) -> MotorState | None:
        """Parse a raw CAN message into a MotorState if it matches our motor.

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
        temperature_raw = struct.unpack("b", bytes([msg.data[6]]))[0]
        # Validate: the driver board temperature must be physically plausible.
        # At startup the enable-response frame sometimes carries uninitialised
        # data in byte 6 (observed: -112, -120, -104 °C).  Clamp to the spec
        # range and fall back to the last known good reading if available.
        if -20 <= temperature_raw <= 127:
            temperature_celsius = temperature_raw
        else:
            logger.debug(
                f"Temperature out of range ({temperature_raw} °C) — "
                f"using last known value"
            )
            temperature_celsius = (
                self._last_feedback.temperature_celsius
                if self._last_feedback is not None
                and -20 <= self._last_feedback.temperature_celsius <= 127
                else 0
            )

        # Error code: uint8 (0=OK, 1-7 see CAN_ERROR_CODES)
        error_code = msg.data[7]

        feedback = MotorState(
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

    def _capture_response(self, timeout: float = 0.20) -> None:
        """Read the motor's one-shot response frame and store it as pending feedback.

        The AK60-6 (in its default CubeMars firmware mode) sends exactly one
        status frame in response to each command it receives.  This method
        should be called immediately after every bus.send() so the response
        is buffered and available to the next _receive_feedback() call.

        Loops for the entire timeout window so that non-motor frames (e.g. the
        0x0088 flood from an unrelated device) are skipped rather than causing
        an immediate early return.

        :param timeout: How long to wait for the response frame (seconds).
            Default 200 ms — the motor typically replies within 2 ms but we
            allow plenty of margin for bus contention, ERROR-WARNING
            recovery, and kernel scheduling jitter.
        :return: None
        """
        if not self.connected or self.bus is None:
            return
        deadline = time.monotonic() + timeout
        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return
                msg = self.bus.recv(timeout=remaining)
                if msg is None:
                    return
                fb = self._parse_feedback_msg(msg)
                if fb is not None:
                    self._pending_feedback = fb
                    self._last_feedback = fb
                    self._consecutive_no_response = 0
                    self.communicating = True
                    return
                logger.debug(
                    f"_capture_response: ignoring 0x{msg.arbitration_id:08X} "
                    f"[{' '.join(f'{b:02X}' for b in msg.data)}]"
                )
        except (can.CanError, Exception) as e:
            logger.debug(f"_capture_response error: {e}")

    def _receive_feedback(self, timeout: float = 0.1) -> MotorState | None:
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
        :return: MotorState or None.
        """
        # Fast path: return the response already captured by _capture_response.
        # _start_refresh() clears _pending_feedback, so this path is only
        # taken for one-shot commands (enable/disable/set_origin) that are NOT
        # followed by a refresh loop.
        if self._pending_feedback is not None:
            fb = self._pending_feedback
            self._pending_feedback = None
            return fb

        if not self.connected or self.bus is None:
            return None

        # Refresh-thread path: the background loop is continuously sending
        # commands and capturing responses into _refresh_feedback.  Racing on
        # bus.recv() here would steal messages from the refresh thread (or
        # vice versa).  Instead, poll _refresh_feedback until it is populated
        # or the timeout expires.  _start_refresh() clears this field each
        # time a new command is issued, so stale data from previous commands
        # (e.g. the enable ACK) is never returned here.
        if self._refresh_thread is not None and self._refresh_thread.is_alive():
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if self._refresh_feedback is not None:
                    self._consecutive_no_response = 0
                    self.communicating = True
                    return self._refresh_feedback
                time.sleep(self._refresh_interval)
            # The refresh loop's 25 ms receive window often misses the motor's
            # low-rate broadcast (~3 Hz after commands).  If _last_feedback was
            # captured earlier in this session (e.g. by enable_motor) the motor
            # is confirmed alive and _last_feedback is current enough to return.
            if self._last_feedback is not None:
                self._consecutive_no_response = 0
                self.communicating = True
                return self._last_feedback
            self._consecutive_no_response += 1
            return None

        # Slow path: no refresh thread — wait on the bus directly (used when
        # the caller sent a command externally, or the motor is configured for
        # periodic broadcast).  Loop for the full timeout window, skipping
        # frames from unrelated devices (e.g. the 0x0088 flood).
        deadline = time.monotonic() + timeout
        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self._consecutive_no_response += 1
                    return None
                msg = self.bus.recv(timeout=remaining)
                if msg is None:
                    self._consecutive_no_response += 1
                    return None

                fb = self._parse_feedback_msg(msg)
                if fb is not None:
                    self._last_feedback = fb
                    self._consecutive_no_response = 0
                    self.communicating = True
                    return fb

                logger.debug(
                    f"_receive_feedback: ignoring 0x{msg.arbitration_id:08X} "
                    f"[{' '.join(f'{b:02X}' for b in msg.data)}] "
                    f"(want one of {[hex(x) for x in sorted(self._feedback_ids)]})"
                )

        except (can.CanError, Exception) as e:
            logger.error(f"CAN error receiving feedback: {e}")
            return None

    def _refresh_loop(self) -> None:
        """Background thread: re-send the active command at 50 Hz until stopped.

        The motor has a ~100 ms CAN watchdog on arb_id=motor_can_id (0x03).
        Only DUTY_CYCLE (mode 0x00) naturally lands on that arb_id.  All other
        modes (velocity 0x0303, current 0x0103, position 0x0403, …) use a
        different arb_id and do NOT feed the watchdog.

        Fix: for non-duty-cycle modes, send a brief ENABLE keepalive on
        motor_can_id before the real command each iteration (~50 Hz, well
        within the 100 ms window).  The motor's ACK to the keepalive always
        reports Speed=0; to avoid poisoning feedback we read ALL motor frames
        up to the deadline and keep the **last** one (the real command reply).
        """
        while not self._refresh_stop.is_set():
            # Capture atomically to avoid a race where _stop_refresh() sets
            # _refresh_mode / _refresh_data to None between the None-check
            # and the actual use of the values.
            _mode = self._refresh_mode
            _data = self._refresh_data
            if _mode is not None and _data is not None:
                if self.connected and self.bus is not None:
                    try:
                        needs_keepalive = _mode != CANControlMode.DUTY_CYCLE

                        # ── Watchdog keepalive (non-duty-cycle modes) ──────
                        if needs_keepalive:
                            self.bus.send(
                                can.Message(
                                    arbitration_id=self.motor_can_id,
                                    data=self._CAN_SERVO_ENABLE,
                                    is_extended_id=True,
                                )
                            )

                        # ── Main control command ───────────────────────────
                        arb_id = self._build_extended_id(_mode)
                        self.bus.send(
                            can.Message(
                                arbitration_id=arb_id,
                                data=_data,
                                is_extended_id=True,
                            )
                        )

                        # Capture reply.  When a keepalive was sent we expect
                        # TWO motor frames (enable ACK + command reply); read
                        # until the deadline and keep the last motor frame so
                        # the Speed=0 ACK is overwritten by the real response.
                        # Use CAN_DEFAULTS.refresh_capture_window_s (25 ms) —
                        # measured round-trip for enable+velocity is ~20 ms on
                        # this Jetson; the 50 Hz loop period is 20 ms but the
                        # watchdog is 100 ms, so 25 ms is safe.
                        deadline = (
                            time.monotonic() + CAN_DEFAULTS.refresh_capture_window_s
                        )
                        _iter_frames = 0
                        while True:
                            remaining = deadline - time.monotonic()
                            if remaining <= 0:
                                break
                            msg = self.bus.recv(timeout=remaining)
                            if msg is None:
                                break
                            _iter_frames += 1
                            # Skip error frames and standard (11-bit) frames.
                            # Motor feedback is always extended 29-bit.
                            if msg.is_error_frame or not msg.is_extended_id:
                                continue
                            fb = self._parse_feedback_msg(msg)
                            if fb:
                                self._last_feedback = fb
                                self._refresh_feedback = fb
                                self.communicating = True
                                if not needs_keepalive:
                                    break
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
        # Clear any stale one-shot response buffered by a previous enable/disable
        # or single-command call.  Without this, _receive_feedback() returns the
        # old response (e.g. the enable ACK with Speed=0) instead of waiting for
        # the refresh thread's live data.
        self._pending_feedback = None
        # Also clear the refresh-specific feedback so _receive_feedback() polls
        # for a new frame captured by this refresh cycle, not a prior one.
        self._refresh_feedback = None
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
        self._refresh_feedback = None
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

    def _get_current_position_for_estimate(self) -> float:
        """Return current CAN position as float for movement estimation."""
        return self.get_position() or 0.0

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

    def _pre_velocity_hook(self, velocity_erpm: int, direction: int) -> None:
        """Skip soft-start if already running in velocity mode.

        Avoids thrashing CURRENT → VELOCITY → CURRENT on every
        P-controller iteration.
        """
        already_in_velocity = self._refresh_mode == CANControlMode.VELOCITY_LOOP
        if not already_in_velocity:
            self._soft_start(direction)

    def _send_velocity_command(self, velocity_erpm: int) -> None:
        """Send velocity command over CAN and start refresh loop.

        CAN velocity uses wider ±320 000 ERPM limits (based on feedback
        range) compared to the UART ±100 000 default.  Re-clamp here.
        """
        velocity_erpm = int(np.clip(velocity_erpm, -320000, 320000))

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

    def set_brake_current(self, current_amps: float) -> None:
        """Hold the motor shaft in place using brake current (mode 0x02).

        Unlike set_current() (which drives rotation), brake current actively
        resists motion — useful for holding a joint against gravity or external
        loads without a position loop.

        Payload encoding is identical to set_current() (int32, amps × 1000),
        but uses CURRENT_BRAKE mode (arb_id = 0x0200 | motor_id).

        Spec example for motor 0x68:
          4A brake  → arb_id 0x00000268, data 0x00000FA0
          -4A brake → arb_id 0x00000268, data 0xFFFFF060

        :param current_amps: Braking current in amps (-60 to 60 A).
        :return: None
        """
        current_amps = float(np.clip(current_amps, -60.0, 60.0))
        current_int = int(current_amps * 1000.0)
        data = struct.pack(">i", current_int)
        self._start_refresh(CANControlMode.CURRENT_BRAKE, data)
        logger.info(f"Set brake current: {current_amps:.2f} A")

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

    def get_status(self) -> MotorState | None:
        """Read and log the full motor state.

        Returns a MotorState dataclass with:
          - position_degrees    — current angle in degrees
          - speed_erpm          — current speed in electrical RPM
          - current_amps        — phase current draw in amps
          - temperature_celsius — driver board temperature
          - error_code          — 0 = no fault (see FaultCode in definitions.py)

        The AK60-6 is response-only: it sends one status frame per command
        received.  If the command response was already captured by
        _capture_response() or the refresh thread, this returns that buffered
        frame.  Otherwise, falls back to the most recent feedback (_last_feedback)
        which is kept fresh by the 50 Hz refresh loop during active control.

        :return: MotorState dataclass, or None if the motor has never
            responded (e.g. not yet enabled or cable disconnected).
        """
        feedback = self._receive_feedback(timeout=0.5)
        if feedback is None:
            # The motor is response-only — if no new frame arrived, return
            # the most recent feedback from the refresh thread or a prior
            # command.  This mirrors get_position() / get_temperature().
            feedback = self._last_feedback
        if feedback:
            logger.info("=" * 50)
            logger.info("MOTOR STATUS:")
            logger.info(f"  Position: {feedback.position_degrees:.2f}°")
            logger.info(f"  Speed: {feedback.speed_erpm} ERPM")
            logger.info(f"  Current: {feedback.current_amps:.2f} A")
            logger.info(f"  Temperature: {feedback.temperature_celsius} °C")
            logger.info(
                f"  Error Code: {feedback.error_code} — {feedback.error_description}"
            )
            logger.info("=" * 50)
        return feedback

    def check_communication(self) -> bool:
        """Check whether the motor is alive and responding over CAN.

        Sends an enable command to put the motor into servo mode, then listens
        for a feedback frame.  The AK60-6 only transmits status frames while in
        servo mode — a zero-duty or passive-listen approach will always time out
        if the motor is currently disabled.

        Side effect: leaves the motor enabled (in servo mode) on success.
        Call disable_motor() afterwards if you do not want it enabled.

        :return: True if a feedback frame is received, False if the motor is
            silent (check power, wiring, CAN ID, and UART cable).
        """
        if not self.connected:
            return False

        # Send the enable command — this puts the motor into servo mode and
        # triggers a feedback response.  The motor is response-only (no
        # autonomous broadcast) so we must actively command it each attempt.
        max_attempts = max(MOTOR_DEFAULTS.max_communication_attempts, 5)
        for attempt in range(1, max_attempts + 1):
            logger.debug(f"check_communication attempt {attempt}/{max_attempts}")
            self.enable_motor()
            # enable_motor() already calls _capture_response(200 ms).
            # If the reply was captured, _pending_feedback is set.
            feedback = self._receive_feedback(timeout=0.5)
            if feedback:
                self.communicating = True
                self._consecutive_no_response = 0
                logger.info(f"Motor communication verified on attempt {attempt}")
                return True
            time.sleep(MOTOR_DEFAULTS.communication_retry_delay)

        logger.warning(
            f"Motor not responding after {max_attempts} attempts — "
            "check power, wiring, CAN ID, and ensure UART cable is disconnected"
        )
        self.communicating = False
        return False

    # ──────────────────────────────────────────────────────────────────────
    # MIT Impedance Control (mode 0x08)
    # ──────────────────────────────────────────────────────────────────────

    # Physical limits for MIT bit-field encoding (from CubeMars AK60-6 spec)
    _MIT_POS_MIN: float = -12.5  # rad
    _MIT_POS_MAX: float = 12.5  # rad
    _MIT_VEL_MIN: float = -45.0  # rad/s
    _MIT_VEL_MAX: float = 45.0  # rad/s
    _MIT_KP_MIN: float = 0.0  # Nm/rad
    _MIT_KP_MAX: float = 500.0  # Nm/rad
    _MIT_KD_MIN: float = 0.0  # Nms/rad
    _MIT_KD_MAX: float = 5.0  # Nms/rad
    _MIT_TAU_MIN: float = -18.0  # Nm
    _MIT_TAU_MAX: float = 18.0  # Nm

    @staticmethod
    def _float_to_uint(value: float, v_min: float, v_max: float, n_bits: int) -> int:
        """Map a physical value to an unsigned integer within an n-bit range.

        :param value:  Physical value to encode.
        :param v_min:  Minimum physical value (maps to 0).
        :param v_max:  Maximum physical value (maps to 2**n_bits - 1).
        :param n_bits: Number of bits in the output field (12 or 16).
        :return: Unsigned integer in [0, 2**n_bits - 1].
        """
        value = float(np.clip(value, v_min, v_max))
        span = v_max - v_min
        return int((value - v_min) * ((1 << n_bits) - 1) / span)

    @staticmethod
    def _uint_to_float(raw: int, v_min: float, v_max: float, n_bits: int) -> float:
        """Decode an unsigned integer field back to a physical value.

        :param raw:    Unsigned integer read from the CAN frame.
        :param v_min:  Minimum physical value.
        :param v_max:  Maximum physical value.
        :param n_bits: Width of the field in bits (12 or 16).
        :return: Physical value in [v_min, v_max].
        """
        return raw * (v_max - v_min) / ((1 << n_bits) - 1) + v_min

    def _pack_mit_frame(
        self,
        pos_rad: float,
        vel_rad_s: float,
        kp: float,
        kd: float,
        torque_ff_nm: float,
    ) -> bytes:
        """Encode five MIT control fields into the 8-byte bit-packed CAN payload.

        Bit layout (big-endian, 64 bits):
          [63:48] pos  16 bits  [-12.5, 12.5] rad
          [47:36] vel  12 bits  [-45.0, 45.0] rad/s
          [35:24] kp   12 bits  [0, 500] Nm/rad
          [23:12] kd   12 bits  [0, 5]   Nms/rad
          [11:0]  tau  12 bits  [-18, 18] Nm

        :param pos_rad:      Target position in radians.
        :param vel_rad_s:    Target velocity in rad/s (used as feedforward).
        :param kp:           Position stiffness in Nm/rad.
        :param kd:           Velocity damping in Nms/rad.
        :param torque_ff_nm: Feed-forward torque in Nm.
        :return: 8-byte payload ready for a CAN message.
        """
        pos_int = self._float_to_uint(pos_rad, self._MIT_POS_MIN, self._MIT_POS_MAX, 16)
        vel_int = self._float_to_uint(
            vel_rad_s, self._MIT_VEL_MIN, self._MIT_VEL_MAX, 12
        )
        kp_int = self._float_to_uint(kp, self._MIT_KP_MIN, self._MIT_KP_MAX, 12)
        kd_int = self._float_to_uint(kd, self._MIT_KD_MIN, self._MIT_KD_MAX, 12)
        tau_int = self._float_to_uint(
            torque_ff_nm, self._MIT_TAU_MIN, self._MIT_TAU_MAX, 12
        )

        # Pack 5 fields into 8 bytes (big-endian bit stream)
        return bytes(
            [
                pos_int >> 8,  # pos [15:8]
                pos_int & 0xFF,  # pos [7:0]
                vel_int >> 4,  # vel [11:4]
                ((vel_int & 0xF) << 4) | (kp_int >> 8),  # vel [3:0] | kp [11:8]
                kp_int & 0xFF,  # kp  [7:0]
                kd_int >> 4,  # kd  [11:4]
                ((kd_int & 0xF) << 4) | (tau_int >> 8),  # kd  [3:0] | tau [11:8]
                tau_int & 0xFF,  # tau [7:0]
            ]
        )

    def enable_mit_mode(self) -> None:
        """Switch the motor from Servo mode to MIT impedance mode.

        **Important:** Call this instead of (or after) enable_motor() when you
        intend to use set_mit_mode().  MIT mode uses a different enable command
        (0xFFFFFFFFFFFFFFFF) than Servo mode (0xFFFFFFFFFFFFFFFC).

        Mixing MIT and Servo commands (e.g. calling set_position() while in MIT
        mode) is not supported — call disable_mit_mode() then enable_motor()
        before switching back to Servo commands.

        :return: None
        """
        if not self.connected or self.bus is None:
            logger.warning("Cannot enable MIT mode — CAN bus not connected")
            return
        try:
            msg = can.Message(
                arbitration_id=self.motor_can_id,
                data=self._CAN_MIT_ENABLE,
                is_extended_id=True,
            )
            self.bus.send(msg, timeout=0.1)
            logger.info("MIT mode enabled")
            self._capture_response()
        except can.CanError as e:
            logger.error(f"Failed to enable MIT mode: {e}")

    def disable_mit_mode(self) -> None:
        """Exit MIT mode and cut motor drive output.

        The motor will coast to a stop.  Call enable_motor() before using
        any Servo-mode command (set_velocity, set_position, etc.).

        :return: None
        """
        if not self.connected or self.bus is None:
            logger.warning("Cannot disable MIT mode — CAN bus not connected")
            return
        self._stop_refresh()
        try:
            msg = can.Message(
                arbitration_id=self.motor_can_id,
                data=self._CAN_MIT_DISABLE,
                is_extended_id=True,
            )
            self.bus.send(msg, timeout=0.1)
            logger.info("MIT mode disabled")
            self._capture_response()
        except can.CanError as e:
            logger.error(f"Failed to disable MIT mode: {e}")

    def set_mit_mode(
        self,
        pos_rad: float,
        vel_rad_s: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0,
        torque_ff_nm: float = 0.0,
    ) -> None:
        """Send an impedance control command using the MIT actuator protocol.

        MIT mode gives direct control over the motor's virtual spring-damper:
        the output torque is computed as::

            τ = kp * (pos_target − pos_actual) + kd * (vel_target − vel_actual) + tau_ff

        This allows smooth, compliant motion ideal for exosuit joints.

        **You must call enable_mit_mode() once before using this method.**

        Common use cases:

        * Position + stiffness (spring mode)::

            motor.set_mit_mode(pos_rad=1.57, kp=100, kd=2)

        * Pure velocity damping (velocity loop)::

            motor.set_mit_mode(pos_rad=0, vel_rad_s=6.0, kd=2)

        * Pure torque feed-forward (torque loop)::

            motor.set_mit_mode(pos_rad=0, torque_ff_nm=5.0)

        * Zero-torque (safe float / gravity comp off)::

            motor.set_mit_mode(pos_rad=0)   # kp=kd=tau=0

        Physical limits (values are clamped, not rejected):

        =========== =================== ===========
        Field       Range               Unit
        =========== =================== ===========
        pos_rad     −12.5 … +12.5       rad
        vel_rad_s   −45.0 … +45.0       rad/s
        kp          0 … 500             Nm/rad
        kd          0 … 5               Nms/rad
        torque_ff   −18 … +18           Nm
        =========== =================== ===========

        CAN frame: arb_id = 0x0800 | motor_id, 8-byte bit-packed payload.
        See _pack_mit_frame() for encoding details.

        :param pos_rad:      Target joint angle in radians.
        :param vel_rad_s:    Target / feedforward velocity in rad/s.
        :param kp:           Position stiffness gain in Nm/rad.
        :param kd:           Velocity damping gain in Nms/rad.
        :param torque_ff_nm: Feed-forward torque in Nm.
        :return: None
        """
        data = self._pack_mit_frame(pos_rad, vel_rad_s, kp, kd, torque_ff_nm)
        self._start_refresh(CANControlMode.MIT_MODE, data)
        logger.info(
            f"MIT mode: pos={pos_rad:.3f} rad  vel={vel_rad_s:.2f} rad/s  "
            f"kp={kp:.1f}  kd={kd:.2f}  tau_ff={torque_ff_nm:.2f} Nm"
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

    def _stop_motor_transport(self) -> None:
        """Stop the motor and release the CAN bus connection."""
        if self.bus:
            self._stop_refresh()
            self._send_can_command(CANControlMode.CURRENT_LOOP, struct.pack(">i", 0))
            time.sleep(0.05)
            self.bus.shutdown()
            logger.info("CAN bus connection closed")
