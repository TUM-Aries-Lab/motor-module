"""Test the CubeMars motor module."""

from motor_python.cube_mars_motor import AK60Motor


def test_ak60motor_initialization():
    """Test that AK60Motor class can be imported."""
    assert AK60Motor is not None
"""Test the main program."""

from motor_python.cube_mars_motor import motor_v3


def test_motor_v3():
    """Test the main function."""
    assert motor_v3() is None
