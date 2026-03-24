"""AK60-6 CAN controller using CubeMars Force Control Mode (MIT) only."""

from __future__ import annotations

import struct
import threading
import time
from typing import ClassVar, Literal

import can
import numpy as np
from loguru import logger

from motor_python.base_motor import BaseMotor, MotorState
from motor_python.can_protocol import CANControlMode
from motor_python.can_utils import get_can_state, reset_can_interface
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

    _CAN_HELPER_ENABLE: bytes = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
    _CAN_HELPER_DISABLE: bytes = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
    # Legacy helper fallback for firmware variants that still expect FF..FF / FF..FE.
    _CAN_HELPER_ENABLE_LEGACY: bytes = bytes([0xFF] * 8)
    _CAN_HELPER_DISABLE_LEGACY: bytes = bytes(
        [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
    )
    _HELPER_POLICIES: ClassVar[set[str]] = {"strict", "fcfd", "legacy"}

    def __init__(  # noqa: PLR0913, PLR0915
        self,
        motor_can_id: int = CAN_DEFAULTS.motor_can_id,
        interface: str = CAN_DEFAULTS.interface,
        bitrate: int = CAN_DEFAULTS.bitrate,
        feedback_can_id: int | None = None,
        mit_velocity_kd: float | None = None,
        helper_policy: Literal["strict", "fcfd", "legacy"] = "fcfd",
        auto_recover_bus: bool = True,
        allow_legacy_feedback_ids: bool = False,
        aggressive_bus_reset: bool = False,
    ) -> None:
        """Initialize CAN motor connection.

        :param motor_can_id: Motor CAN ID (driver ID in low 8 bits).
        :param interface: SocketCAN interface name.
        :param bitrate: CAN bitrate in bits/sec.
        :param feedback_can_id: Optional explicit status frame ID.
        :param mit_velocity_kd: Optional default KD used by ``set_velocity()``.
            If omitted, ``CAN_DEFAULTS.mit_velocity_kd`` is used.
        :param helper_policy: MIT helper-frame policy:
            ``strict`` = no helper frames,
            ``fcfd`` = use FF..FC / FF..FD compatibility frames,
            ``legacy`` = additionally allow FF..FF / FF..FE fallback.
        :param auto_recover_bus: If True, auto-reset CAN interface when unhealthy.
        :param allow_legacy_feedback_ids: If True, include additional historical
            feedback IDs observed in older firmware experiments.
        :param aggressive_bus_reset: If True, allow runtime kernel-level CAN reset
            during BUS-OFF conditions. Disabled by default because it is disruptive.
        """
        super().__init__()
        self.motor_can_id = motor_can_id
        self.interface = interface
        self.bitrate = bitrate
        self.bus: can.BusABC | None = None
        policy = helper_policy.lower().strip()
        if policy not in self._HELPER_POLICIES:
            allowed = ", ".join(sorted(self._HELPER_POLICIES))
            raise ValueError(
                f"Invalid helper_policy '{helper_policy}'. Expected one of: {allowed}"
            )
        self._helper_policy = policy
        self._auto_recover_bus = auto_recover_bus
        self._aggressive_bus_reset = aggressive_bus_reset
        velocity_kd = (
            CAN_DEFAULTS.mit_velocity_kd
            if mit_velocity_kd is None
            else float(mit_velocity_kd)
        )
        if not (AK60_6_MIT_LIMITS.kd_min <= velocity_kd <= AK60_6_MIT_LIMITS.kd_max):
            raise ValueError(
                f"mit_velocity_kd must be in [{AK60_6_MIT_LIMITS.kd_min}, {AK60_6_MIT_LIMITS.kd_max}]"
            )
        self._mit_velocity_kd = velocity_kd

        self._mit_enabled: bool = False

        # Last responses
        self._last_feedback: MotorState | None = None
        self._last_feedback_monotonic: float = 0.0
        self._active_feedback_id: int | None = None
        self._pending_feedback: MotorState | None = None
        self._refresh_feedback: MotorState | None = None
        self._refresh_feedback_monotonic: float = 0.0

        # MIT command refresh thread (re-send active command at fixed rate)
        self._refresh_payload: bytes | None = None
        self._refresh_stop = threading.Event()
        self._refresh_thread: threading.Thread | None = None
        self._refresh_interval = 1.0 / CAN_DEFAULTS.feedback_rate_hz
        self._refresh_send_failures = 0
        self._refresh_no_feedback = 0

        # Transport health / pacing
        self._send_lock = threading.Lock()
        self._recv_lock = threading.Lock()
        self._last_tx_monotonic = 0.0
        self._tx_min_interval = max(0.0, self._refresh_interval / 2.0)  # <=100 Hz
        self._send_retry_backoff = 0.01
        self._transport_fault: str | None = None
        self._last_unhealthy_log_monotonic = 0.0
        self._last_can_state_cache_monotonic = 0.0
        self._last_can_state_cache: dict[str, int | str] = {
            "state": "UNKNOWN",
            "tx_err": 0,
            "rx_err": 0,
        }
        self._can_state_cache_ttl = 0.20

        # Canonical feedback IDs (extended) plus optional compatibility IDs.
        self._feedback_ids_ext: set[int] = {0x2900 | motor_can_id, 0x2900}
        self._feedback_ids_std: set[int] = set()
        if feedback_can_id is not None:
            self._feedback_ids_ext.add(feedback_can_id)
            if feedback_can_id <= 0x7FF:
                self._feedback_ids_std.add(feedback_can_id)
        if allow_legacy_feedback_ids:
            legacy_ids = {motor_can_id, motor_can_id + 1, 0x0080 | motor_can_id}
            self._feedback_ids_ext.update(legacy_ids)
            self._feedback_ids_std.update(legacy_ids)
        self._feedback_ids: set[int] = self._feedback_ids_ext | self._feedback_ids_std

        self._connect()

    # ------------------------------------------------------------------
    # Connection and transport
    # ------------------------------------------------------------------

    def _connect(self) -> None:
        """Establish the SocketCAN connection and apply receive filters."""
        try:
            bus_state = get_can_state(self.interface)
            self._last_can_state_cache = {
                "state": str(bus_state.get("state", "UNKNOWN")),
                "tx_err": int(bus_state.get("tx_err", 0)),
                "rx_err": int(bus_state.get("rx_err", 0)),
            }
            self._last_can_state_cache_monotonic = time.monotonic()
            logger.info(
                f"CAN bus state: {bus_state['state']} "
                f"(tx_err={bus_state['tx_err']} rx_err={bus_state['rx_err']})"
            )

            can_filters = [
                {
                    "can_id": can_id,
                    "can_mask": 0x1FFFFFFF,
                    "extended": True,
                }
                for can_id in sorted(self._feedback_ids_ext)
            ]
            can_filters.extend(
                [
                    {
                        "can_id": can_id,
                        "can_mask": 0x7FF,
                        "extended": False,
                    }
                    for can_id in sorted(self._feedback_ids_std)
                ]
            )

            self.bus = can.interface.Bus(
                channel=self.interface,
                interface="socketcan",
                bitrate=self.bitrate,
                can_filters=can_filters,
                receive_own_messages=False,
                ignore_rx_error_frames=True,
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

    def _pace_tx(self) -> None:
        """Enforce a small minimum gap between outgoing CAN frames."""
        now = time.monotonic()
        elapsed = now - self._last_tx_monotonic
        if elapsed < self._tx_min_interval:
            time.sleep(self._tx_min_interval - elapsed)
        self._last_tx_monotonic = time.monotonic()

    def _drain_rx_queue(self, max_frames: int = 256) -> int:
        """Drain queued receive frames to reduce socket backlog after send faults."""
        if self.bus is None:
            return 0
        drained = 0
        for _ in range(max_frames):
            try:
                with self._recv_lock:
                    bus = self.bus
                    if bus is None:
                        break
                    msg = bus.recv(timeout=0.0)
            except (can.CanError, Exception):
                break
            if msg is None:
                break
            drained += 1
        return drained

    @staticmethod
    def _is_tx_buffer_full_error(exc: can.CanError) -> bool:
        """Return True when python-can reports socket TX queue saturation."""
        text = str(exc).lower()
        return "buffer" in text and "full" in text

    @staticmethod
    def _is_transport_handle_error(exc: Exception) -> bool:
        """Return True for transport/socket-handle errors that need reconnect."""
        text = str(exc).lower()
        return any(
            key in text
            for key in (
                "no such device",
                "no such device or address",
                "network is down",
                "not connected",
                "bad file descriptor",
                "socket closed",
                "nonetype",
            )
        )

    def _reconnect_transport(self) -> bool:
        """Reconnect the local SocketCAN bus object to clear driver-side queues."""
        old_bus: can.BusABC | None
        with self._send_lock:
            old_bus = self.bus
            self.bus = None
            self.connected = False

        if old_bus is not None:
            try:
                old_bus.shutdown()
            except Exception as exc:  # pragma: no cover - defensive
                logger.debug(f"Ignoring error during bus shutdown: {exc}")

        # Small pause so kernel/socket state settles before re-opening.
        time.sleep(0.05)
        self._connect()
        return self.connected and self.bus is not None

    def _record_transport_fault(self, message: str) -> None:
        """Store a transport fault and mark communication unhealthy."""
        self._transport_fault = message
        self.communicating = False
        self._mit_enabled = False

    def clear_transport_fault(self) -> None:
        """Clear a previously recorded transport fault after successful recovery."""
        self._transport_fault = None

    def _read_can_state(self, *, force: bool = False) -> dict[str, int | str]:
        """Return cached CAN controller state unless an immediate refresh is needed."""
        now = time.monotonic()
        age = now - self._last_can_state_cache_monotonic
        if not force and age <= self._can_state_cache_ttl:
            return self._last_can_state_cache

        state = get_can_state(self.interface)
        self._last_can_state_cache = {
            "state": str(state.get("state", "UNKNOWN")),
            "tx_err": int(state.get("tx_err", 0)),
            "rx_err": int(state.get("rx_err", 0)),
        }
        self._last_can_state_cache_monotonic = now
        return self._last_can_state_cache

    @staticmethod
    def _is_unhealthy_bus_state(state: dict[str, int | str]) -> bool:
        """Return True when CAN state indicates communication instability."""
        status = str(state.get("state", "UNKNOWN"))
        tx_err = int(state.get("tx_err", 0))
        rx_err = int(state.get("rx_err", 0))
        if status == "UNKNOWN":
            return tx_err >= 96 or rx_err >= 96
        return status != "ERROR-ACTIVE" or tx_err >= 96 or rx_err >= 96

    @staticmethod
    def _can_tx_viable_in_degraded_state(state: dict[str, int | str]) -> bool:
        """Return True when transmit can proceed despite degraded CAN state."""
        status = str(state.get("state", "UNKNOWN"))
        tx_err = int(state.get("tx_err", 0))
        return status != "BUS-OFF" and tx_err < 128

    def _recover_bus_if_needed(self, reason: str) -> bool:
        """Recover CAN transport when controller state is unhealthy.

        Runtime behavior balances safety and continuity:
        - BUS-OFF or high transmit-error states are treated as hard faults.
        - WARNING/PASSIVE states with low tx_err may continue transmitting while
          recovery is attempted opportunistically.
        """
        can_state = self._read_can_state()
        if not self._is_unhealthy_bus_state(can_state):
            return True

        status = str(can_state.get("state", "UNKNOWN"))
        now = time.monotonic()
        if now - self._last_unhealthy_log_monotonic >= 2.0:
            logger.warning(
                f"CAN unhealthy before/after {reason}: "
                f"state={can_state['state']} tx_err={can_state['tx_err']} rx_err={can_state['rx_err']}"
            )
            self._last_unhealthy_log_monotonic = now

        if not self._auto_recover_bus:
            return False

        # In passive/warning states, reconnect churn can worsen short-lived
        # disturbances. If TX is still viable, keep commands flowing.
        if status in {"ERROR-PASSIVE", "ERROR-WARNING"}:
            if self._can_tx_viable_in_degraded_state(can_state):
                return True
            return False

        if status == "BUS-OFF":
            # With restart-ms configured, controller can recover automatically.
            time.sleep(0.12)

        if self._reconnect_transport():
            recovered = self._read_can_state(force=True)
            logger.info(
                f"CAN transport reconnected: state={recovered['state']} "
                f"tx_err={recovered['tx_err']} rx_err={recovered['rx_err']}"
            )
            return not self._is_unhealthy_bus_state(
                recovered
            ) or self._can_tx_viable_in_degraded_state(recovered)

        if self._aggressive_bus_reset and status == "BUS-OFF":
            logger.warning("Attempting aggressive kernel-level CAN reset")
            if (
                reset_can_interface(self.interface, self.bitrate)
                and self._reconnect_transport()
            ):
                recovered = self._read_can_state(force=True)
                logger.info(
                    f"CAN recovered after aggressive reset: state={recovered['state']} "
                    f"tx_err={recovered['tx_err']} rx_err={recovered['rx_err']}"
                )
                return not self._is_unhealthy_bus_state(
                    recovered
                ) or self._can_tx_viable_in_degraded_state(recovered)
            logger.error("Aggressive CAN reset failed")
            return False

        return False

    @staticmethod
    def _mit_neutral_payload() -> bytes:
        """Return a neutral MIT payload (no position/speed/torque command)."""
        return pack_mit_frame(0.0, 0.0, 0.0, 0.0, 0.0, limits=AK60_6_MIT_LIMITS)

    def _send_raw(  # noqa: PLR0912, PLR0915
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
        if not self._recover_bus_if_needed(reason="transmit"):
            self._record_transport_fault(
                "CAN interface unhealthy and auto-recovery failed before transmit"
            )
            return False

        if len(data) < 8:
            payload = data + bytes(8 - len(data))
        elif len(data) > 8:
            raise ValueError(f"CAN payload too large: {len(data)} bytes (max 8)")
        else:
            payload = data

        msg = can.Message(
            arbitration_id=arbitration_id,
            data=payload,
            is_extended_id=True,
        )

        reconnected = False
        final_state: dict[str, int | str] | None = None
        for attempt in range(1, CAN_DEFAULTS.max_retries + 1):
            try:
                with self._send_lock:
                    bus = self.bus
                    if bus is None:
                        raise can.CanError("CAN transport handle missing")
                    self._pace_tx()
                    bus.send(msg, timeout=timeout)
                self.clear_transport_fault()
                logger.debug(
                    f"TX CAN ID 0x{msg.arbitration_id:08X}: "
                    f"{' '.join(f'{b:02X}' for b in msg.data)}"
                )
                if capture_response:
                    self._capture_response()
                return True
            except can.CanError as exc:
                can_state = self._read_can_state(force=True)
                final_state = can_state
                is_buffer_full = self._is_tx_buffer_full_error(exc)
                is_handle_error = self._is_transport_handle_error(exc)
                logger.error(
                    "CAN send failure "
                    f"({attempt}/{CAN_DEFAULTS.max_retries}) on ID "
                    f"0x{arbitration_id:08X}: {exc} | "
                    f"state={can_state['state']} tx_err={can_state['tx_err']} rx_err={can_state['rx_err']}"
                )
                self.communicating = False
                self._drain_rx_queue()

                state_name = str(can_state.get("state", "UNKNOWN"))
                needs_recovery = (
                    is_buffer_full or is_handle_error or state_name == "BUS-OFF"
                )
                if needs_recovery and not reconnected:
                    logger.warning(
                        "Detected CAN transport issue; reconnecting/resetting SocketCAN transport"
                    )
                    if is_handle_error:
                        reconnected = self._reconnect_transport()
                        if not reconnected:
                            reconnected = self._recover_bus_if_needed(
                                reason="send handle error"
                            )
                    else:
                        reconnected = self._recover_bus_if_needed(reason="send failure")
                    if reconnected:
                        logger.info("SocketCAN transport recovered; retrying transmit")
                    else:
                        logger.warning(
                            "SocketCAN transport recovery failed; continuing retries"
                        )
                        if self._is_unhealthy_bus_state(
                            can_state
                        ) and not self._can_tx_viable_in_degraded_state(can_state):
                            break

                if attempt < CAN_DEFAULTS.max_retries:
                    time.sleep(self._send_retry_backoff * attempt)
            except Exception as exc:
                can_state = self._read_can_state(force=True)
                final_state = can_state
                logger.error(
                    "Unexpected CAN send exception "
                    f"({attempt}/{CAN_DEFAULTS.max_retries}) on ID 0x{arbitration_id:08X}: {exc} | "
                    f"state={can_state['state']} tx_err={can_state['tx_err']} rx_err={can_state['rx_err']}"
                )
                self.communicating = False
                self._drain_rx_queue()
                if not reconnected:
                    reconnected = self._reconnect_transport()
                    if not reconnected:
                        reconnected = self._recover_bus_if_needed(
                            reason="unexpected send exception"
                        )
                    if reconnected:
                        logger.info(
                            "SocketCAN transport recovered after unexpected exception; retrying transmit"
                        )
                    elif self._is_unhealthy_bus_state(
                        can_state
                    ) and not self._can_tx_viable_in_degraded_state(can_state):
                        break
                if attempt < CAN_DEFAULTS.max_retries:
                    time.sleep(self._send_retry_backoff * attempt)

        state_hint = ""
        if final_state is None:
            final_state = self._read_can_state(force=True)
        tx_err = int(final_state.get("tx_err", 0))
        rx_err = int(final_state.get("rx_err", 0))
        status = str(final_state.get("state", "UNKNOWN"))
        if tx_err >= 128 and rx_err <= 8:
            state_hint = " Likely no ACK from another CAN node (motor power/wiring/UART/cabling)."

        self._record_transport_fault(
            "CAN transmit failed after retries "
            f"(arb_id=0x{arbitration_id:08X}, interface={self.interface}, "
            f"state={status}, tx_err={tx_err}, rx_err={rx_err}).{state_hint}"
        )
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
        if msg.is_error_frame:
            return None
        if getattr(msg, "is_remote_frame", False):
            return None
        is_rx = getattr(msg, "is_rx", None)
        if is_rx is False:
            return None

        allowed_ids = (
            self._feedback_ids_ext if msg.is_extended_id else self._feedback_ids_std
        )
        if msg.arbitration_id not in allowed_ids:
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

        error_code = int(msg.data[7]) & 0xFF

        feedback = MotorState(
            position_degrees=pos_int * 0.1,
            speed_erpm=speed_int * 10,
            current_amps=current_int * 0.01,
            temperature_celsius=temperature_celsius,
            error_code=error_code,
        )
        if self._active_feedback_id != msg.arbitration_id:
            self._active_feedback_id = msg.arbitration_id
            logger.info(f"Active feedback CAN ID: 0x{msg.arbitration_id:08X}")
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
                with self._recv_lock:
                    bus = self.bus
                    if bus is None:
                        return None
                    msg = bus.recv(timeout=remaining)
            except (can.CanError, Exception) as exc:
                logger.debug(f"_capture_response recv error: {exc}")
                return None

            if msg is None:
                return None

            feedback = self._parse_feedback_msg(msg)
            if feedback is None:
                continue

            self._last_feedback = feedback
            ts = time.monotonic()
            self._last_feedback_monotonic = ts
            self._consecutive_no_response = 0
            self.communicating = True

            if store_for_refresh:
                self._refresh_feedback = feedback
                self._refresh_feedback_monotonic = ts
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
            fresh_window = max(0.15, 3.0 * self._refresh_interval)
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if self._refresh_feedback is not None:
                    age = time.monotonic() - self._refresh_feedback_monotonic
                    if age <= fresh_window:
                        self._consecutive_no_response = 0
                        self.communicating = True
                        return self._refresh_feedback
                time.sleep(self._refresh_interval)

            if self._last_feedback is not None:
                age = time.monotonic() - self._last_feedback_monotonic
                if age <= max(0.5, fresh_window):
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
                    try:
                        self.enable_mit_mode()
                    except Exception as exc:
                        self._record_transport_fault(f"Refresh enable failed: {exc}")
                        self._refresh_stop.set()
                        continue

                sent = self._send_mit_payload(payload, capture_response=False)
                if not sent:
                    self._refresh_send_failures += 1
                    if self._refresh_send_failures >= CAN_DEFAULTS.max_retries:
                        self._record_transport_fault(
                            "Refresh loop stopped after repeated CAN transmit failures"
                        )
                        self._refresh_stop.set()
                    self._refresh_stop.wait(self._refresh_interval)
                    continue

                self._refresh_send_failures = 0
                feedback = self._capture_response(
                    timeout=CAN_DEFAULTS.refresh_capture_window_s,
                    store_for_refresh=True,
                )
                if feedback is None:
                    self._refresh_no_feedback += 1
                    if self._refresh_no_feedback >= max(12, self._max_no_response * 4):
                        self._record_transport_fault(
                            "No motor feedback during MIT refresh loop; stopping keepalive "
                            "to avoid flooding an unhealthy CAN bus"
                        )
                        self._refresh_stop.set()
                        continue
                else:
                    self._refresh_no_feedback = 0
            self._refresh_stop.wait(self._refresh_interval)

    def _start_refresh(self, payload: bytes) -> None:
        """Store active MIT payload and ensure refresh thread is running."""
        if len(payload) != 8:
            raise ValueError(f"MIT payload must be 8 bytes, got {len(payload)}")

        self._refresh_payload = payload
        self._pending_feedback = None
        self._refresh_feedback = None
        self._refresh_feedback_monotonic = 0.0
        self._refresh_send_failures = 0
        self._refresh_no_feedback = 0

        if (
            self._refresh_thread is not None
            and self._refresh_thread.is_alive()
            and self._refresh_stop.is_set()
        ):
            self._refresh_thread.join(timeout=0.3)

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
            self._refresh_thread.join(timeout=1.0)
            if self._refresh_thread.is_alive():
                logger.warning(
                    "MIT refresh thread did not stop within timeout; keeping stop flag set"
                )
                return
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

    def send_keepalive(self) -> bool:
        """Send a benign MIT keepalive payload and attempt to capture feedback."""
        payload = (
            self._refresh_payload
            if self._refresh_payload is not None
            else self._mit_neutral_payload()
        )
        sent = self._send_mit_payload(payload, capture_response=True)
        if not sent:
            logger.warning("MIT keepalive transmit failed")
            return False
        return True

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
            kd=self._mit_velocity_kd,
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

        neutral = self._mit_neutral_payload()

        max_attempts = max(MOTOR_DEFAULTS.max_communication_attempts, 3)

        for attempt in range(1, max_attempts + 1):
            logger.debug(f"check_communication attempt {attempt}/{max_attempts}")
            self._pending_feedback = None
            feedback_before = self._last_feedback_monotonic
            try:
                self.enable_mit_mode()
            except Exception as exc:
                logger.debug(f"enable_mit_mode failed during comm check: {exc}")
                time.sleep(MOTOR_DEFAULTS.communication_retry_delay)
                continue

            sent = self._send_mit_payload(neutral, capture_response=True)
            if not sent:
                time.sleep(MOTOR_DEFAULTS.communication_retry_delay)
                continue

            feedback = self._receive_feedback(timeout=0.3)
            if feedback is not None and self._last_feedback_monotonic > feedback_before:
                self.communicating = True
                self._consecutive_no_response = 0
                return True
            time.sleep(MOTOR_DEFAULTS.communication_retry_delay)

        self.communicating = False
        can_state = self._read_can_state(force=True)
        logger.error(
            "No motor feedback during check_communication. "
            f"state={can_state['state']} tx_err={can_state['tx_err']} rx_err={can_state['rx_err']}. "
            "Likely causes: UART cable connected, CAN feedback mode not periodic/query-reply, "
            "or firmware uses a different feedback ID mapping."
        )
        return False

    # ------------------------------------------------------------------
    # Force Control Mode (MIT)
    # ------------------------------------------------------------------

    def enable_mit_mode(self) -> None:
        """Enable MIT operation and verify with fresh feedback."""
        if not self.connected:
            logger.warning("Cannot enable MIT mode - CAN bus not connected")
            return

        if self._transport_fault is not None:
            logger.warning(
                f"Recovering CAN transport before enable_mit_mode(): {self._transport_fault}"
            )
            if not self._reconnect_transport():
                raise RuntimeError(f"CAN transport fault: {self._transport_fault}")
            self.clear_transport_fault()
        if not self._recover_bus_if_needed(reason="enable_mit_mode"):
            raise RuntimeError(
                "Failed to recover CAN interface before enabling MIT mode"
            )

        self._mit_enabled = False
        neutral = self._mit_neutral_payload()
        feedback_before = self._last_feedback_monotonic
        sent = self._send_mit_payload(neutral, capture_response=True)
        if sent and self._last_feedback_monotonic > feedback_before:
            self._mit_enabled = True
            logger.info("MIT mode ready via direct MIT command handshake")
            return

        helper_frames: list[tuple[str, bytes]] = []
        if self._helper_policy in {"fcfd", "legacy"}:
            helper_frames.append(("FF..FC", self._CAN_HELPER_ENABLE))
        if self._helper_policy == "legacy":
            helper_frames.append(("FF..FF", self._CAN_HELPER_ENABLE_LEGACY))

        for label, frame in helper_frames:
            helper_sent = self._send_raw(
                arbitration_id=self.motor_can_id,
                data=frame,
                capture_response=True,
            )
            if not helper_sent:
                continue

            feedback_before = self._last_feedback_monotonic
            sent = self._send_mit_payload(neutral, capture_response=True)
            if sent and self._last_feedback_monotonic > feedback_before:
                self._mit_enabled = True
                logger.info(f"MIT mode ready after helper frame {label}")
                return

        can_state = self._read_can_state(force=True)
        raise RuntimeError(
            "Failed to enable MIT mode: no fresh feedback observed after MIT handshake "
            f"(state={can_state['state']} tx_err={can_state['tx_err']} rx_err={can_state['rx_err']})"
        )

    def disable_mit_mode(self) -> None:
        """Disable Force Control Mode and stop active command refresh."""
        if not self.connected:
            logger.warning("Cannot disable MIT mode - CAN bus not connected")
            return

        if not self._recover_bus_if_needed(reason="disable_mit_mode"):
            logger.warning(
                "Skipping MIT disable helper frame because CAN bus is unhealthy"
            )
            self._mit_enabled = False
            return

        self._stop_refresh()
        neutral = self._mit_neutral_payload()
        # Send a short neutral burst before disable helper to quench motion quickly.
        for _ in range(3):
            _ = self._send_mit_payload(neutral, capture_response=False)
            time.sleep(0.01)

        sent = True
        if self._helper_policy in {"fcfd", "legacy"}:
            sent = self._send_raw(
                arbitration_id=self.motor_can_id,
                data=self._CAN_HELPER_DISABLE,
                capture_response=True,
            )
            if not sent and self._helper_policy == "legacy":
                sent = self._send_raw(
                    arbitration_id=self.motor_can_id,
                    data=self._CAN_HELPER_DISABLE_LEGACY,
                    capture_response=True,
                )
            if not sent:
                raise RuntimeError(
                    "Failed to disable MIT mode: helper-frame transmit path unhealthy"
                )

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

        if self._transport_fault is not None:
            logger.warning(
                f"Recovering CAN transport before set_mit_mode(): {self._transport_fault}"
            )
            if not self._reconnect_transport():
                raise RuntimeError(f"CAN transport fault: {self._transport_fault}")
            self.clear_transport_fault()

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
        refresh_alive = (
            self._refresh_thread is not None and self._refresh_thread.is_alive()
        )
        sent = self._send_mit_payload(payload, capture_response=not refresh_alive)
        if not sent:
            raise RuntimeError(
                "Failed to send MIT command: CAN transmit path unhealthy"
            )

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

        if not self._recover_bus_if_needed(reason="stop"):
            logger.warning("Skipping MIT stop payload because CAN bus is unhealthy")
            self._mit_enabled = False
            return

        neutral = self._mit_neutral_payload()
        sent_any = False
        for _ in range(4):
            sent = self._send_mit_payload(neutral, capture_response=False)
            sent_any = sent_any or sent
            time.sleep(0.01)
        if not sent_any:
            logger.warning("Failed to send neutral MIT stop payload")
        time.sleep(0.02)

        disable_error: Exception | None = None
        for _ in range(2):
            try:
                self.disable_mit_mode()
                disable_error = None
                break
            except Exception as exc:
                disable_error = exc
                time.sleep(0.02)
        if disable_error is not None:
            logger.warning(f"disable_mit_mode() failed during stop: {disable_error}")

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
