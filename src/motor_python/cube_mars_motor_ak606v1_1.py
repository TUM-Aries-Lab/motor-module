"""Class for the CubeMars AK60-6 v1.1 KV140 Motor using CAN protocol."""

import time
from typing import Literal

import can
import numpy as np
from loguru import logger

from motor_python.base_motor import MotorState
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN
from motor_python.definitions import (
    AK60_6_V1_1_MOTOR_SPEC,
    CAN_DEFAULTS,
    CURRENT_MOTOR_SPEC,
    MotorSpec,
)
from motor_python.mit_mode_packer import (
    AK60_6_V1_1_MIT_LIMITS,
    MITModeLimits,
    float_to_uint,
    uint_to_float,
)


# TODO: add tests
class CubeMarsAK806v2CAN(CubeMarsAK606v3CAN):  # pragma: no cover
    """AK60-6 V1.1 Motor Controller over CAN with MIT force-control protocol."""

    def __init__(
        self,
        motor_can_id: int = CAN_DEFAULTS.motor_can_id,
        interface: str = CAN_DEFAULTS.interface,
        bitrate: int = CAN_DEFAULTS.bitrate,
        feedback_can_id: int | None = None,
        mit_velocity_kd: float | None = None,
        motor_spec: MotorSpec = CURRENT_MOTOR_SPEC,
        helper_policy: Literal["strict", "fcfd", "legacy"] = "fcfd",
        auto_recover_bus: bool = True,
        allow_legacy_feedback_ids: bool = False,
        aggressive_bus_reset: bool = False,
    ) -> None:
        """Initialize CAN motor connection for the AK80-6 V2.

        :param motor_spec: If not provided, defaults to AK60_6_V1_1_MOTOR_SPEC
        """
        if motor_spec is None:
            motor_spec = AK60_6_V1_1_MOTOR_SPEC
        super().__init__(
            motor_can_id=motor_can_id,
            interface=interface,
            bitrate=bitrate,
            feedback_can_id=feedback_can_id,
            mit_velocity_kd=mit_velocity_kd,
            motor_spec=motor_spec,
            helper_policy=helper_policy,
            auto_recover_bus=auto_recover_bus,
            allow_legacy_feedback_ids=allow_legacy_feedback_ids,
            aggressive_bus_reset=aggressive_bus_reset,
        )

    def _parse_feedback_msg(self, msg: can.Message) -> MotorState | None:
        """Parse AK60-6 v1.1 MIT feedback frame.

        The CubeMars feedback frame is 6 bytes:
            byte 0      : motor ID
            byte 1      : position[15:8]
            byte 2      : position[7:0]
            byte 3      : velocity[11:4]
            byte 4      : velocity[3:0] | torque[11:8]
            byte 5      : torque[7:0]
        """
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
        if len(msg.data) < 7:
            return None

        d = msg.data

        motor_id = d[0]
        p_int = (d[1] << 8) | d[2]
        v_int = (d[3] << 4) | (d[4] >> 4)
        i_int = ((d[4] & 0x0F) << 8) | d[5]
        temp_raw = d[6]
        error_code = d[7] if len(d) > 7 else 0

        # Convert back to physical units
        position_rad = uint_to_float(
            p_int,
            self._mit_limits.p_min,
            self._mit_limits.p_max,
            16,
        )
        velocity_rad_s = uint_to_float(
            v_int,
            self._mit_limits.v_min,
            self._mit_limits.v_max,
            12,
        )
        current_amps = uint_to_float(
            i_int,
            self._mit_limits.t_min,
            self._mit_limits.t_max,
            12,
        )

        temperature_celsius = temp_raw - 40

        # Convert velocity to ERPM for compatibility with MotorState
        speed_erpm = self._rad_s_to_erpm(velocity_rad_s)

        logger.debug(
            f"Parsed AK60 v1.1 MIT feedback ints: motor_id={motor_id} p={position_rad} v={velocity_rad_s} i={current_amps} temp={temperature_celsius} error={error_code}"
        )

        feedback = MotorState(
            position_degrees=np.degrees(position_rad),
            speed_erpm=speed_erpm,
            current_amps=current_amps,
            temperature_celsius=temperature_celsius,
            error_code=error_code,
        )

        if self._active_feedback_id != msg.arbitration_id:
            self._active_feedback_id = msg.arbitration_id
            logger.info(
                f"Active AK60 v1.1 MIT feedback CAN ID: 0x{msg.arbitration_id:08X}"
            )

        return feedback

    # ruff: noqa: PLR0913
    def pack_mit_frame(
        self,
        p_des: float,
        v_des: float,
        kp: float,
        kd: float,
        t_ff: float,
        limits: MITModeLimits = AK60_6_V1_1_MIT_LIMITS,
    ) -> bytes:
        """Pack MIT command frame for AK60-6 v1.1.

        Bit layout (matches the Simulink Buff 0..7 subsystem):
            byte 0 : p[15:8]
            byte 1 : p[7:0]
            byte 2 : v[11:4]
            byte 3 : v[3:0] | kp[11:8]
            byte 4 : kp[7:0]
            byte 5 : kd[11:4]
            byte 6 : kd[3:0] | t[11:8]
            byte 7 : t[7:0]
        """
        p_int = float_to_uint(
            p_des,
            limits.p_min,
            limits.p_max,
            16,
        )
        v_int = float_to_uint(
            v_des,
            limits.v_min,
            limits.v_max,
            12,
        )
        kp_int = float_to_uint(
            kp,
            limits.kp_min,
            limits.kp_max,
            12,
        )
        kd_int = float_to_uint(
            kd,
            limits.kd_min,
            limits.kd_max,
            12,
        )
        t_int = float_to_uint(
            t_ff,
            limits.t_min,
            limits.t_max,
            12,
        )

        logger.debug(
            f"Packing AK80 MIT frame: p={p_des:.3f} rad (int {p_int}) "
            f"v={v_des:.3f} rad/s (int {v_int}) kp={kp:.2f} (int {kp_int}) "
            f"kd={kd:.2f} (int {kd_int}) t_ff={t_ff:.2f} Nm (int {t_int})"
        )

        return bytes(
            [
                (p_int >> 8) & 0xFF,
                p_int & 0xFF,
                (v_int >> 4) & 0xFF,
                ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F),
                kp_int & 0xFF,
                (kd_int >> 4) & 0xFF,
                ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F),
                t_int & 0xFF,
            ]
        )

    def _send_velocity_command(self, velocity_erpm: int) -> None:
        vel_rad_s = self._erpm_to_rad_s(velocity_erpm)
        logger.info(
            f"Sending velocity command: {velocity_erpm} ERPM ({vel_rad_s:.3f} rad/s)"
        )

        self.set_mit_mode(  # TODO
            pos_rad=0.0,
            vel_rad_s=vel_rad_s,
            kp=0.0,
            kd=3.0,
            torque_ff_nm=0.0,
        )

    def _connect(self) -> None:
        """Connect and reset MIT state for clean startup."""
        super()._connect()

        if not self.connected:
            return

        # Force exit MIT mode first in case motor is still in it from previous session
        self._send_raw(
            arbitration_id=self.motor_can_id,
            data=self._CAN_HELPER_DISABLE,  # 0xFD
            capture_response=False,
        )
        time.sleep(0.15)  # motor needs ~100ms to fully exit MIT mode

        logger.info("AK60-6 startup MIT reset complete")
