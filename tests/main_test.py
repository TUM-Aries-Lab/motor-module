"""Test the main program."""

from motor_python.__main__ import main


def test_main():
    """Test the main function."""
    assert main() is None
