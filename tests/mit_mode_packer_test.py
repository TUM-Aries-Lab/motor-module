"""Unit tests for MIT frame packing helpers."""

import pytest

from motor_python.utils import float_to_uint, uint_to_float


def test_float_to_uint_clamps_to_range():
    assert float_to_uint(-999.0, -1.0, 1.0, 12) == 0
    assert float_to_uint(999.0, -1.0, 1.0, 12) == (1 << 12) - 1


def test_uint_to_float_round_trip_midpoint():
    raw = float_to_uint(0.0, -12.56, 12.56, 16)
    value = uint_to_float(raw, -12.56, 12.56, 16)
    assert value == pytest.approx(0.0, abs=0.001)
