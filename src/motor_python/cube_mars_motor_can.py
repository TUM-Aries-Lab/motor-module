"""AK60-6 CAN controller using CubeMars Force Control Mode (MIT) only."""

from __future__ import annotations

import struct
import threading
import time

import can
import numpy as np
from loguru import logger

from motor_python.base_motor import BaseMotor, MotorState
from motor_python.can_protocol import CANControlMode
from motor_python.can_utils import get_can_state
from motor_python.definitions import CAN_DEFAULTS, MOTOR_DEFAULTS
from motor_python.mit_mode_packer import AK60_6_MIT_LIMITS, pack_mit_frame


class CubeMarsAK606v3CAN(BaseMotor):
    """AK60-6 Motor Controller over CAN with MIT force-control protocol.

    Hardware target:
    - Jetson Orin Nano (SocketCAN interface, typically ``can0``)
    - CubeMars AK60-6

    Protocol scope:
    - Extended CAN IDs only.
    - Control mode ID ``0x08`` (Force Control / MIT) only.
    - Feedback parsing follows CubeMars status frame format:
      ``pos(int16*0.1deg), speed(int16*10ERPM), current(int16*0.01A), temp(int8), err(uint8)``.
    """

    _CAN_MIT_ENABLE: bytes = bytes([0xFF] * 8)
    _CAN_MIT_DISABLE: bytes = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])

    def __init__(
        self,
        motor_can_id: int = CAN_DEFAULTS.motor_can_id,
        interface: str = CAN_DEFAULTS.interface,
        bitrate: int = CAN_DEFAULTS.bitrate,
        feedback_can_id: int | None = None,
    ) -> None:
        """Initialize CAN motor connection.

        :param motor_can_id: Motor CAN ID (driver ID in low 8 bits).
        :param interface: SocketCAN interface name.
        :param bitrate: CAN bitrate in bits/sec.
        :param feedback_can_id: Optional explicit status frame ID.
        """
        super().__init__()
        self.motor_can_id = motor_can_id
        self.interface = interface
        self.bitrate = bitrate
        self.bus: can.BusABC | None = None

        self._mit_enabled: bool = False

        # Last responses
        self._last_feedback: MotorState | None = None
        self._pending_feedback: MotorState | None = None
        self._refresh_feedback: MotorState | None = None

        # MIT command refresh thread (re-send active command at fixed rate)
        self._refresh_payload: bytes | None = None
        self._refresh_stop = threading.Event()
        self._refresh_thread: threading.Thread | None = None
        self._refresh_interval = 1.0 / CAN_DEFAULTS.feedback_rate_hz

        # Candidate feedback IDs observed across firmware variants.
        self._feedback_ids: set[int] = {
            motor_can_id,
            motor_can_id + 1,
            0x2900 | motor_can_id,
            0x0080 | motor_can_id,
            0x2900,
        }
        if feedback_can_id is not None:
            self._feedback_ids.add(feedback_can_id)

        self._connect()

    # ------------------------------------------------------------------
    # Connection and transport
    # ------------------------------------------------------------------

    def _connect(self) -> None:
        """Establish the SocketCAN connection and apply receive filters."""
        try:
            bus_state = get_can_state(self.interface)
            logger.info(
                f"CAN bus state: {bus_state['state']} "
                f"(tx_err={bus_state['tx_err']} rx_err={bus_state['rx_err']})"
            )

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
                {
                    "can_id": self.motor_can_id,
                    "can_mask": 0x1FFFFFFF,
                    "extended": True,
                },
                {
                    "can_id": 0x0080 | self.motor_can_id,
                    "can_mask": 0x1FFFFFFF,
                    "extended": True,
                },
            ]

            self.bus = can.interface.Bus(
                channel=self.interface,
                interface="socketcan",
                bitrate=self.bitrate,
                can_filters=can_filters,
            )

            # Suppress CAN error frames to reduce recv flood on some mttcan setups.
            try:
                import socket as _socket

                _CAN_RAW_ERR_FILTER = getattr(_socket, "CAN_RAW_ERR_FILTER", 2)
                self.bus.socket.setsockopt(  # type: ignore[union-attr]
                    _socket.SOL_CAN_RAW,
                    _CAN_RAW_ERR_FILTER,
                    struct.pack("=I", 0),
                )
            except Exception as exc:  # pragma: no cover - platform dependent
                logger.debug(f"Could not set CAN_RAW_ERR_FILTER=0: {exc}")

            time.sleep(CAN_DEFAULTS.connection_stabilization_delay)
            self.connected = True
            logger.info(
                f"Connected to motor CAN ID 0x{self.motor_can_id:02X} on "
                f"{self.interface} at {self.bitrate} bps"
            )
        except can.CanError as exc:
            logger.warning(
                f"Failed to connect to CAN interface {self.interface}: {exc}"
            )
            self.connected = False
        except Exception as exc:
            logger.warning(f"Unexpected error connecting to CAN bus: {exc}")
            self.connected = False

    def _build_extended_id(self, mode: int) -> int:
        """Build extended arbitration ID: ``(mode << 8) | motor_can_id``."""
        return (mode << 8) | self.motor_can_id

    def _send_raw(
        self,
        arbitration_id: int,
        data: bytes,
        *,
        capture_response: bool = True,
        timeout: float = 0.1,
    ) -> bool:
        """Send one extended CAN frame and optionally capture immediate feedback."""
        if not self.connected or self.bus is None:
            logger.debug("CAN bus not connected - skipping command")
            return False

        if len(data) < 8:
            payload = data + bytes(8 - len(data))
        elif len(data) > 8:
            raise ValueError(f"CAN payload too large: {len(data)} bytes (max 8)")
        else:
            payload = data

        try:
            msg = can.Message(
                arbitration_id=arbitration_id,
                data=payload,
                is_extended_id=True,
            )
            self.bus.send(msg, timeout=timeout)
            logger.debug(
                f"TX CAN ID 0x{msg.arbitration_id:08X}: "
                f"{' '.join(f'{b:02X}' for b in msg.data)}"
            )
            if capture_response:
                self._capture_response()
            return True
        except can.CanError as exc:
            logger.error(f"CAN error sending command: {exc}")
            return False

    def _send_mit_payload(
        self, payload: bytes, *, capture_response: bool = True
    ) -> bool:
        """Send one Force Control Mode (MIT) command frame."""
        return self._send_raw(
            self._build_extended_id(CANControlMode.MIT_MODE),
            payload,
            capture_response=capture_response,
        )

    # ------------------------------------------------------------------
    # Feedback parsing and receive
    # ------------------------------------------------------------------

    def _parse_feedback_msg(self, msg: can.Message) -> MotorState | None:
        """Parse a CAN frame into MotorState if it belongs to this motor."""
        if msg.arbitration_id not in self._feedback_ids:
            return None
        if len(msg.data) < 8:
            logger.warning(f"Received short CAN message: {len(msg.data)} bytes")
            return None

        pos_int = struct.unpack(">h", msg.data[0:2])[0]
        speed_int = struct.unpack(">h", msg.data[2:4])[0]
        current_int = struct.unpack(">h", msg.data[4:6])[0]
        temperature_raw = struct.unpack("b", bytes([msg.data[6]]))[0]

        if -20 <= temperature_raw <= 127:
            temperature_celsius = temperature_raw
        else:
            temperature_celsius = (
                self._last_feedback.temperature_celsius
                if self._last_feedback is not None
                and -20 <= self._last_feedback.temperature_celsius <= 127
                else 0
            )

        feedback = MotorState(
            position_degrees=pos_int * 0.1,
            speed_erpm=speed_int * 10,
            current_amps=current_int * 0.01,
            temperature_celsius=temperature_celsius,
            error_code=msg.data[7],
        )
        return feedback

    def _capture_response(
        self,
        timeout: float = 0.20,
        *,
        store_for_refresh: bool = False,
    ) -> MotorState | None:
        """Capture one feedback frame from the bus within timeout."""
        if not self.connected or self.bus is None:
            return None

        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None

            try:
                msg = self.bus.recv(timeout=remaining)
            except (can.CanError, Exception) as exc:
                logger.debug(f"_capture_response recv error: {exc}")
                return None

            if msg is None:
                return None

            feedback = self._parse_feedback_msg(msg)
            if feedback is None:
                continue

            self._last_feedback = feedback
            self._consecutive_no_response = 0
            self.communicating = True

            if store_for_refresh:
                self._refresh_feedback = feedback
            else:
                self._pending_feedback = feedback
            return feedback

    def _receive_feedback(self, timeout: float = 0.2) -> MotorState | None:
        """Return latest motor feedback (pending, refresh, or direct recv)."""
        if self._pending_feedback is not None:
            feedback = self._pending_feedback
            self._pending_feedback = None
            return feedback

        if not self.connected or self.bus is None:
            return None

        if self._refresh_thread is not None and self._refresh_thread.is_alive():
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if self._refresh_feedback is not None:
                    self._consecutive_no_response = 0
                    self.communicating = True
                    return self._refresh_feedback
                time.sleep(self._refresh_interval)

            if self._last_feedback is not None:
                return self._last_feedback

            self._consecutive_no_response += 1
            return None

        feedback = self._capture_response(timeout=timeout)
        if feedback is not None:
            return feedback

        self._consecutive_no_response += 1
        return None

    # ------------------------------------------------------------------
    # MIT mode state and refresh loop
    # ------------------------------------------------------------------

    def _refresh_loop(self) -> None:
        """Re-send active MIT command at fixed rate while enabled."""
        while not self._refresh_stop.is_set():
            payload = self._refresh_payload
            if payload is not None and self.connected and self.bus is not None:
                if not self._mit_enabled:
                    self.enable_mit_mode()
                self._send_mit_payload(payload, capture_response=False)
                self._capture_response(
                    timeout=CAN_DEFAULTS.refresh_capture_window_s,
                    store_for_refresh=True,
                )
            self._refresh_stop.wait(self._refresh_interval)

    def _start_refresh(self, payload: bytes) -> None:
        """Store active MIT payload and ensure refresh thread is running."""
        if len(payload) != 8:
            raise ValueError(f"MIT payload must be 8 bytes, got {len(payload)}")

        self._refresh_payload = payload
        self._pending_feedback = None
        self._refresh_feedback = None

        if self._refresh_thread is None or not self._refresh_thread.is_alive():
            self._refresh_stop.clear()
            self._refresh_thread = threading.Thread(
                target=self._refresh_loop,
                daemon=True,
                name="mit-refresh",
            )
            self._refresh_thread.start()

    def _stop_refresh(self) -> None:
        """Stop refresh thread and clear active MIT payload."""
        self._refresh_payload = None
        self._refresh_feedback = None
        self._refresh_stop.set()
        if self._refresh_thread is not None:
            self._refresh_thread.join(timeout=0.2)
            self._refresh_thread = None
        self._refresh_stop.clear()

    # ------------------------------------------------------------------
    # BaseMotor-required API
    # ------------------------------------------------------------------

    def enable_motor(self) -> None:
        """Compatibility alias: enable MIT mode."""
        self.enable_mit_mode()

    def disable_motor(self) -> None:
        """Compatibility alias: disable MIT mode."""
        self.disable_mit_mode()

    def _soft_start(self, direction: int) -> None:
        """No-op in MIT-only implementation."""
        _ = direction

    def _erpm_to_rad_s(self, erpm: int) -> float:
        """Convert electrical RPM to mechanical rad/s for AK60-6."""
        return (
            float(erpm) * (2.0 * np.pi) / (60.0 * float(CAN_DEFAULTS.motor_pole_pairs))
        )

    def _get_current_position_for_estimate(self) -> float:
        """Return current motor position in degrees, or 0.0 if unavailable."""
        return self.get_position() or 0.0

    def set_position(self, position_degrees: float) -> None:
        """Position loop in MIT mode using default ``kp/kd`` gains."""
        pos_rad = float(np.deg2rad(position_degrees))
        self.set_mit_mode(
            pos_rad=pos_rad,
            vel_rad_s=0.0,
            kp=CAN_DEFAULTS.mit_position_kp,
            kd=CAN_DEFAULTS.mit_position_kd,
            torque_ff_nm=0.0,
        )

    def _send_velocity_command(self, velocity_erpm: int) -> None:
        """Velocity loop in MIT mode (``kp=0``, ``kd>0``)."""
        vel_rad_s = self._erpm_to_rad_s(velocity_erpm)
        self.set_mit_mode(
            pos_rad=0.0,
            vel_rad_s=vel_rad_s,
            kp=0.0,
            kd=CAN_DEFAULTS.mit_velocity_kd,
            torque_ff_nm=0.0,
        )

    def set_current(self, current_amps: float) -> None:
        """Compatibility wrapper for force control torque command.

        In MIT-only mode this method maps the input to feedforward torque (Nm).
        """
        logger.warning("MIT-only mode: interpreting set_current() input as torque (Nm)")
        self.set_mit_mode(
            pos_rad=0.0,
            vel_rad_s=0.0,
            kp=0.0,
            kd=0.0,
            torque_ff_nm=float(current_amps),
        )

    def set_brake_current(self, current_amps: float) -> None:
        """Brake-current mode is not implemented in MIT-only transport."""
        raise NotImplementedError(
            "MIT-only CAN implementation: use set_mit_mode(..., kd=...) or torque feedforward"
        )

    def set_duty_cycle(self, duty: float) -> None:
        """Duty-cycle mode is intentionally unsupported in MIT-only transport."""
        raise NotImplementedError(
            "MIT-only CAN implementation: duty-cycle mode is disabled"
        )

    def set_origin(self, permanent: bool = False) -> None:
        """Origin reset is not part of Force Control Mode command set."""
        _ = permanent
        raise NotImplementedError(
            "MIT-only CAN implementation: set_origin is not supported"
        )

    def set_position_velocity_accel(
        self,
        position_degrees: float,
        velocity_erpm: int,
        accel_erpm_per_sec: int = 0,
    ) -> None:
        """Profile mode (0x06) is intentionally disabled in MIT-only transport."""
        _ = (position_degrees, velocity_erpm, accel_erpm_per_sec)
        raise NotImplementedError(
            "MIT-only CAN implementation: set_position_velocity_accel is disabled"
        )

    def get_position(self) -> float | None:
        """Return current position in degrees."""
        feedback = self._receive_feedback(timeout=0.2)
        if feedback is not None:
            return feedback.position_degrees
        if self._last_feedback is not None:
            return self._last_feedback.position_degrees
        return None

    def get_status(self) -> MotorState | None:
        """Return latest parsed motor telemetry."""
        feedback = self._receive_feedback(timeout=0.5)
        if feedback is None:
            feedback = self._last_feedback
        return feedback

    def check_communication(self) -> bool:
        """Verify communication by enabling MIT mode and sending neutral command."""
        if not self.connected:
            return False

        neutral = pack_mit_frame(0.0, 0.0, 0.0, 0.0, 0.0, limits=AK60_6_MIT_LIMITS)

        max_attempts = max(MOTOR_DEFAULTS.max_communication_attempts, 3)
        for attempt in range(1, max_attempts + 1):
            logger.debug(f"check_communication attempt {attempt}/{max_attempts}")
            self.enable_mit_mode()
            self._send_mit_payload(neutral, capture_response=True)
            feedback = self._receive_feedback(timeout=0.3)
            if feedback is not None:
                self.communicating = True
                self._consecutive_no_response = 0
                return True
            time.sleep(MOTOR_DEFAULTS.communication_retry_delay)

        self.communicating = False
        return False

    # ------------------------------------------------------------------
    # Force Control Mode (MIT)
    # ------------------------------------------------------------------

    def enable_mit_mode(self) -> None:
        """Enable MIT operation using this driver's compatibility sequence.

        Note: the CubeMars manual section 4.2 documents MIT command frames
        (mode ID 0x08) but does not explicitly define a dedicated enable frame.
        This method keeps the existing project behavior by sending the legacy
        all-0xFF frame on ``arb_id=motor_id`` before MIT commands.
        """
        if not self.connected:
            logger.warning("Cannot enable MIT mode - CAN bus not connected")
            return

        sent = self._send_raw(
            arbitration_id=self.motor_can_id,
            data=self._CAN_MIT_ENABLE,
            capture_response=True,
        )
        if sent:
            self._mit_enabled = True
            logger.info("MIT mode enabled")

    def disable_mit_mode(self) -> None:
        """Disable Force Control Mode and stop active command refresh."""
        if not self.connected:
            logger.warning("Cannot disable MIT mode - CAN bus not connected")
            return

        self._stop_refresh()
        sent = self._send_raw(
            arbitration_id=self.motor_can_id,
            data=self._CAN_MIT_DISABLE,
            capture_response=True,
        )
        if sent:
            self._mit_enabled = False
            logger.info("MIT mode disabled")

    def set_mit_mode(
        self,
        pos_rad: float,
        vel_rad_s: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0,
        torque_ff_nm: float = 0.0,
    ) -> None:
        """Send Force Control Mode command and keep it alive via refresh thread.

        Frame details implemented here follow the CubeMars manual:
        - Extended ID: ``(0x08 << 8) | motor_id``
        - Payload order: ``KP, KD, Position, Speed, Torque`` bit-packed in 8 bytes.
        - AK60-6 limits:
          ``pos ±12.56 rad, vel ±60 rad/s, torque ±12 Nm, kp 0..500, kd 0..5``.
        """
        if not self.connected:
            logger.warning("Cannot send MIT command - CAN bus not connected")
            return

        if not self._mit_enabled:
            self.enable_mit_mode()

        payload = pack_mit_frame(
            p_des=pos_rad,
            v_des=vel_rad_s,
            kp=kp,
            kd=kd,
            t_ff=torque_ff_nm,
            limits=AK60_6_MIT_LIMITS,
        )

        # Send once immediately, then keep alive in the background.
        self._send_mit_payload(payload, capture_response=True)
        self._start_refresh(payload)

        logger.info(
            f"MIT cmd: pos={pos_rad:.3f} rad vel={vel_rad_s:.3f} rad/s "
            f"kp={kp:.2f} kd={kd:.2f} tau={torque_ff_nm:.2f} Nm"
        )

    def stop(self) -> None:
        """Send neutral MIT command and disable MIT mode."""
        self._stop_refresh()
        if not self.connected:
            return

        neutral = pack_mit_frame(0.0, 0.0, 0.0, 0.0, 0.0, limits=AK60_6_MIT_LIMITS)
        self._send_mit_payload(neutral, capture_response=False)
        time.sleep(0.02)
        self.disable_mit_mode()

    def _stop_motor_transport(self) -> None:
        """Stop motor and release CAN bus connection."""
        try:
            self.stop()
        except Exception as exc:
            logger.debug(f"stop() during close raised: {exc}")

        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None

        self.connected = False
