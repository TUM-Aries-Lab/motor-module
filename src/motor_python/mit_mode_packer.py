"""Helpers for CubeMars Force Control Mode (MIT) CAN frame packing."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MITModeLimits:
    """Physical limits for CubeMars Force Control Mode fields."""

    p_min: float
    p_max: float
    v_min: float
    v_max: float
    t_min: float
    t_max: float
    kp_min: float
    kp_max: float
    kd_min: float
    kd_max: float


# AK60-6 limits from CubeMars manual force-control parameter table.
AK60_6_MIT_LIMITS = MITModeLimits(
    p_min=-12.56,
    p_max=12.56,
    v_min=-60.0,
    v_max=60.0,
    t_min=-12.0,
    t_max=12.0,
    kp_min=0.0,
    kp_max=500.0,
    kd_min=0.0,
    kd_max=5.0,
)


def float_to_uint(value: float, v_min: float, v_max: float, n_bits: int) -> int:
    """Convert a float to an n-bit unsigned integer with saturation."""
    span = v_max - v_min
    if span <= 0:
        raise ValueError("Invalid range: v_max must be greater than v_min")
    if n_bits <= 0:
        raise ValueError("n_bits must be > 0")

    max_int = (1 << n_bits) - 1
    value_clamped = min(max(value, v_min), v_max)
    if value_clamped <= v_min:
        return 0
    if value_clamped >= v_max:
        return max_int
    raw = int(((value_clamped - v_min) * max_int) / span)
    return max(0, min(raw, max_int))


def uint_to_float(raw: int, v_min: float, v_max: float, n_bits: int) -> float:
    """Convert an n-bit unsigned integer back to a physical float value."""
    max_int = (1 << n_bits) - 1
    if max_int <= 0:
        raise ValueError("n_bits must be > 0")
    raw_clamped = max(0, min(int(raw), max_int))
    return (raw_clamped * (v_max - v_min) / max_int) + v_min


def pack_mit_frame(  # noqa: PLR0913
    p_des: float,
    v_des: float,
    kp: float,
    kd: float,
    t_ff: float,
    limits: MITModeLimits = AK60_6_MIT_LIMITS,
) -> bytes:
    """Pack Force Control Mode payload according to CubeMars manual layout.

    Byte layout (AK manual, mode ID = 8):
      DATA[0] = KP high 8 bits
      DATA[1] = KP low 4 bits | KD high 4 bits
      DATA[2] = KD low 8 bits
      DATA[3] = Position high 8 bits
      DATA[4] = Position low 8 bits
      DATA[5] = Speed high 8 bits
      DATA[6] = Speed low 4 bits | Torque high 4 bits
      DATA[7] = Torque low 8 bits
    """
    p_int = float_to_uint(p_des, limits.p_min, limits.p_max, 16)
    v_int = float_to_uint(v_des, limits.v_min, limits.v_max, 12)
    kp_int = float_to_uint(kp, limits.kp_min, limits.kp_max, 12)
    kd_int = float_to_uint(kd, limits.kd_min, limits.kd_max, 12)
    t_int = float_to_uint(t_ff, limits.t_min, limits.t_max, 12)

    return bytes(
        [
            kp_int >> 4,
            ((kp_int & 0xF) << 4) | (kd_int >> 8),
            kd_int & 0xFF,
            p_int >> 8,
            p_int & 0xFF,
            v_int >> 4,
            ((v_int & 0xF) << 4) | (t_int >> 8),
            t_int & 0xFF,
        ]
    )
