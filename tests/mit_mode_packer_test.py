"""Unit tests for MIT frame packing helpers."""

import pytest

from motor_python.mit_mode_packer import (
    AK60_6_MIT_LIMITS,
    float_to_uint,
    pack_mit_frame,
    uint_to_float,
)


def test_float_to_uint_clamps_to_range():
    assert float_to_uint(-999.0, -1.0, 1.0, 12) == 0
    assert float_to_uint(999.0, -1.0, 1.0, 12) == (1 << 12) - 1


def test_uint_to_float_round_trip_midpoint():
    raw = float_to_uint(0.0, -12.56, 12.56, 16)
    value = uint_to_float(raw, -12.56, 12.56, 16)
    assert value == pytest.approx(0.0, abs=0.001)


def test_pack_mit_frame_uses_manual_byte_order():
    # Pick deterministic values and compare against manual byte mapping.
    p_int = float_to_uint(1.0, AK60_6_MIT_LIMITS.p_min, AK60_6_MIT_LIMITS.p_max, 16)
    v_int = float_to_uint(2.0, AK60_6_MIT_LIMITS.v_min, AK60_6_MIT_LIMITS.v_max, 12)
    kp_int = float_to_uint(30.0, AK60_6_MIT_LIMITS.kp_min, AK60_6_MIT_LIMITS.kp_max, 12)
    kd_int = float_to_uint(1.5, AK60_6_MIT_LIMITS.kd_min, AK60_6_MIT_LIMITS.kd_max, 12)
    t_int = float_to_uint(3.0, AK60_6_MIT_LIMITS.t_min, AK60_6_MIT_LIMITS.t_max, 12)

    expected = bytes(
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

    payload = pack_mit_frame(1.0, 2.0, 30.0, 1.5, 3.0, limits=AK60_6_MIT_LIMITS)
    assert payload == expected


def test_pack_mit_frame_ak60_6_limits_are_enforced():
    payload = pack_mit_frame(
        999.0, 999.0, 999.0, 999.0, 999.0, limits=AK60_6_MIT_LIMITS
    )

    # Decode only boundary-sensitive fields to confirm top saturation.
    kp_high = payload[0]
    kp_low = payload[1] >> 4
    kp_raw = (kp_high << 4) | kp_low

    kd_high = payload[1] & 0xF
    kd_low = payload[2]
    kd_raw = (kd_high << 8) | kd_low

    pos_raw = (payload[3] << 8) | payload[4]

    assert kp_raw == (1 << 12) - 1
    assert kd_raw == (1 << 12) - 1
    assert pos_raw == (1 << 16) - 1
