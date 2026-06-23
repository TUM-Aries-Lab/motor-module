"""Motor control module for CubeMars AK60-6 exosuit actuators.

Primary interface: CAN (CubeMarsAK606v3CAN / Motor).
Legacy UART interface: CubeMarsAK606v3.
Base class: BaseMotor (for shared interface & safety logic).
"""

__version__ = "0.0.7"

from typing import Literal

from motor_python.base_motor import BaseMotor
from motor_python.cube_mars_motor import CubeMarsAK606v3, CubeMarsAK806v2
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN, CubeMarsAK806v2CAN
from motor_python.definitions import (
    AK60_6_MOTOR_SPEC,
    AK80_6_MOTOR_SPEC,
    CAN_DEFAULTS,
    MotorSpec,
)

# Convenience alias — CAN is the primary interface. Later can be changed to AK806v2CAN.
Motor = CubeMarsAK606v3CAN


# ruff: noqa: PLR0913
def create_can_motor(
    motor_model: str = "AK60-6",
    *,
    motor_can_id: int = CAN_DEFAULTS.motor_can_id,
    interface: str = CAN_DEFAULTS.interface,
    bitrate: int = CAN_DEFAULTS.bitrate,
    feedback_can_id: int | None = None,
    mit_velocity_kd: float | None = None,
    motor_spec: MotorSpec | None = None,
    helper_policy: Literal["strict", "fcfd", "legacy"] = "fcfd",
    auto_recover_bus: bool = True,
    allow_legacy_feedback_ids: bool = True,
    aggressive_bus_reset: bool = False,
) -> CubeMarsAK606v3CAN | CubeMarsAK806v2CAN:
    """Build a CAN motor instance for the requested model."""
    model = motor_model.strip().upper()
    if model in {"AK60-6", "AK60_6"}:
        return CubeMarsAK606v3CAN(
            motor_can_id=motor_can_id,
            interface=interface,
            bitrate=bitrate,
            feedback_can_id=feedback_can_id,
            mit_velocity_kd=mit_velocity_kd,
            motor_spec=motor_spec if motor_spec is not None else AK60_6_MOTOR_SPEC,
            helper_policy=helper_policy,
            auto_recover_bus=auto_recover_bus,
            allow_legacy_feedback_ids=allow_legacy_feedback_ids,
            aggressive_bus_reset=aggressive_bus_reset,
        )
    if model in {"AK80-6", "AK80_6"}:
        return CubeMarsAK806v2CAN(
            motor_can_id=motor_can_id,
            interface=interface,
            bitrate=bitrate,
            feedback_can_id=feedback_can_id,
            mit_velocity_kd=mit_velocity_kd,
            motor_spec=motor_spec if motor_spec is not None else AK80_6_MOTOR_SPEC,
            helper_policy=helper_policy,
            auto_recover_bus=auto_recover_bus,
            allow_legacy_feedback_ids=allow_legacy_feedback_ids,
            aggressive_bus_reset=aggressive_bus_reset,
        )
    raise ValueError("Unknown motor model: must be AK60-6 or AK80-6")


__all__ = [
    "BaseMotor",
    "CubeMarsAK606v3",
    "CubeMarsAK606v3CAN",
    "CubeMarsAK806v2",
    "CubeMarsAK806v2CAN",
    "Motor",
    "create_can_motor",
]
