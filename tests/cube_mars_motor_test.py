"""Test the CubeMars motor module."""

from motor_python.cube_mars_motor import AK60Motor


def test_ak60motor_initialization():
    """Test that AK60Motor class can be imported."""
    assert AK60Motor is not None
