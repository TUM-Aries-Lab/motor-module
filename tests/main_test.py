"""Test the main program."""

from motor_python.cube_mars_motor import motor_v3


def test_main():
    """Test the main function."""
    assert motor_v3() is None
