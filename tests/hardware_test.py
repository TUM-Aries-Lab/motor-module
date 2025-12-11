"""Hardware integration tests - only run when hardware is available.

These tests require actual motor hardware connected.
Run with: pytest -m hardware
"""

import time

import pytest

from motor_python.cube_mars_motor import CubeMarsAK606v3

# Mark all tests in this file as hardware tests
pytestmark = pytest.mark.hardware


@pytest.fixture
def motor():
    """Provide a motor instance with real hardware connection."""
    motor = CubeMarsAK606v3()
    if not motor.connected:
        pytest.skip("Motor hardware not available")
    yield motor
    motor.close()


class TestHardwareConnection:
    """Test actual hardware connection and communication."""

    def test_motor_connection(self, motor):
        """Test that motor connects successfully."""
        assert motor.connected
        assert motor.serial is not None
        assert motor.serial.is_open

    def test_motor_communication(self, motor):
        """Test that motor responds to status queries."""
        result = motor.check_communication()
        assert result
        assert motor.communicating

    def test_get_status(self, motor):
        """Test getting motor status."""
        status = motor.get_status()
        assert status is not None
        assert len(status) > 0

    def test_get_position(self, motor):
        """Test getting motor position."""
        response = motor.get_position()
        assert response is not None


class TestHardwareMotorControl:
    """Test motor control with actual hardware."""

    def test_set_position(self, motor):
        """Test setting motor position."""
        # Set to zero position
        motor.set_position(0.0)
        time.sleep(0.2)

        # Get status to verify
        status = motor.get_status()
        assert status is not None

    def test_set_velocity(self, motor):
        """Test setting motor velocity."""
        # Set low velocity
        motor.set_velocity(1000)
        time.sleep(0.2)

        # Stop
        motor.set_velocity(0)
        time.sleep(0.1)

        # Get status
        status = motor.get_status()
        assert status is not None

    def test_set_duty_cycle(self, motor):
        """Test setting duty cycle."""
        # Set small duty cycle
        motor.set_duty_cycle(0.05)
        time.sleep(0.2)

        # Stop
        motor.set_duty_cycle(0.0)
        time.sleep(0.1)

        # Get status
        status = motor.get_status()
        assert status is not None

    def test_set_current(self, motor):
        """Test setting motor current."""
        # Set low current
        motor.set_current(0.5)
        time.sleep(0.2)

        # Stop
        motor.set_current(0.0)
        time.sleep(0.1)

        # Get status
        status = motor.get_status()
        assert status is not None

    def test_stop(self, motor):
        """Test motor stop function."""
        motor.stop()
        time.sleep(0.2)

        # Get status
        status = motor.get_status()
        assert status is not None


class TestHardwareEdgeCases:
    """Test edge cases with real hardware."""

    def test_position_clamping(self, motor):
        """Test that position values are properly clamped."""
        # Try to set beyond limits
        motor.set_position(500.0)  # Should be clamped to 360
        time.sleep(0.2)

        motor.set_position(-500.0)  # Should be clamped to -360
        time.sleep(0.2)

        # Return to zero
        motor.set_position(0.0)
        time.sleep(0.2)

    def test_velocity_clamping(self, motor):
        """Test that velocity values are properly clamped."""
        # Try to set beyond limits
        motor.set_velocity(150000)  # Should be clamped to 100000
        time.sleep(0.2)

        motor.set_velocity(0)  # Stop
        time.sleep(0.1)

    def test_consecutive_commands(self, motor):
        """Test sending multiple commands in sequence."""
        motor.set_position(10.0)
        time.sleep(0.1)
        motor.set_position(-10.0)
        time.sleep(0.1)
        motor.set_position(0.0)
        time.sleep(0.1)

        status = motor.get_status()
        assert status is not None
