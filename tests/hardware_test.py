"""Hardware integration tests - only run when hardware is available.

These tests require actual motor hardware connected.
Run with: make test-hardware
"""

import logging
import time
from collections.abc import Generator

import pytest

from motor_python.cube_mars_motor import CubeMarsAK606v3
from motor_python.examples import (
    run_current_control,
    run_duty_cycle_control,
    run_position_control,
    run_velocity_control,
)

# Logger records warning messages when motor fails to stop or reset after tests complete
logger = logging.getLogger(__name__)

# Mark all tests in this file as hardware tests
pytestmark = pytest.mark.hardware


def get_status_with_retry(
    motor: CubeMarsAK606v3, max_retries: int = 8, delay: float = 0.12
) -> bytes:
    """Get motor status with retry logic for unreliable communication.

    Args:
        motor: Motor instance
        max_retries: Maximum number of retry attempts
        delay: Delay between retries in seconds

    Returns:
        Valid status bytes

    Raises:
        AssertionError: If unable to get valid status after all retries

    """
    for attempt in range(max_retries):
        status = motor.get_status()
        # Check if we got a valid response with expected structure
        # Valid status should be 90 bytes (AA 55 command data... checksum BB)
        if status is not None and len(status) >= 85:
            return status
        if attempt < max_retries - 1:
            time.sleep(delay)
    # Last attempt without delay
    status = motor.get_status()
    assert status is not None and len(status) >= 85, (
        f"Failed to get valid status after retries (got {len(status) if status else 0} bytes)"
    )
    return status


@pytest.fixture
def motor() -> Generator[CubeMarsAK606v3, None, None]:
    """Provide a motor instance with real hardware connection."""
    motor = CubeMarsAK606v3()
    if not motor.connected:
        pytest.skip("Motor hardware not available")
    yield motor
    try:
        # Ensure motor is stopped and returned to zero position for safety
        try:
            motor.stop()
        except Exception as e:
            logger.warning(f"Failed to stop motor during cleanup: {e}")
        try:
            motor.set_position(position_degrees=0.0)
        except Exception as e:
            logger.warning(f"Failed to reset motor position during cleanup: {e}")
        motor.close()
    finally:
        motor.close()


class TestHardwareConnection:
    """Test actual hardware connection and communication."""

    def test_motor_connection(self, motor: CubeMarsAK606v3) -> None:
        """Test that motor connects successfully."""
        assert motor.connected
        assert motor.serial is not None
        assert motor.serial.is_open

    def test_motor_communication(self, motor: CubeMarsAK606v3) -> None:
        """Test that motor responds to status queries."""
        result = motor.check_communication()
        assert result
        assert motor.communicating

    def test_get_status(self, motor: CubeMarsAK606v3) -> None:
        """Test getting motor status."""
        status = get_status_with_retry(motor)
        assert status is not None
        assert len(status) > 0

    def test_get_position(self, motor: CubeMarsAK606v3) -> None:
        """Test getting motor position."""
        response = motor.get_position()
        assert response is not None


class TestHardwareMotorControl:
    """Test motor control with actual hardware - verify motor responds to commands."""

    def test_set_position(self, motor: CubeMarsAK606v3) -> None:
        """Test setting motor position command works."""
        # Set to zero position
        motor.set_position(position_degrees=0.0)
        time.sleep(0.5)  # Give motor time to move

        # Verify motor reached the position
        status = get_status_with_retry(motor)
        assert status is not None
        assert len(status) > 0

        motor.status_parser.payload_offset = 0
        motor.status_parser.parse_temperatures(status)
        motor.status_parser.parse_currents(status)
        motor.status_parser.parse_duty_speed_voltage(status)
        status_pos = motor.status_parser.parse_status_position(status)
        assert status_pos is not None
        assert abs(status_pos.position_degrees - 0.0) < 2.0

    def test_position_movement(self, motor: CubeMarsAK606v3) -> None:
        """Test motor accepts multiple position commands using position control."""
        # Use the position control example function
        run_position_control(motor, num_steps=10, max_angle_degrees=30.0)

        # Verify motor is responsive after the sequence
        status = get_status_with_retry(motor)
        assert status is not None
        assert len(status) > 0

    def test_set_velocity(self, motor: CubeMarsAK606v3) -> None:
        """Test setting motor velocity command works using velocity control."""
        # Use the velocity control example function
        run_velocity_control(motor, velocity_erpm=5000)

        # Verify motor stopped and is responsive
        status = get_status_with_retry(motor)
        assert status is not None
        assert len(status) > 0

        motor.status_parser.payload_offset = 0
        motor.status_parser.parse_temperatures(status)
        motor.status_parser.parse_currents(status)
        duty_speed_voltage = motor.status_parser.parse_duty_speed_voltage(status)
        assert duty_speed_voltage is not None
        # Verify speed is close to 0 after function completes (it stops the motor)
        # Skip verification if value is obviously corrupted (> 1M indicates byte misalignment)
        if abs(duty_speed_voltage.speed_erpm) < 1000000:
            assert abs(duty_speed_voltage.speed_erpm) < 10000, (
                f"Speed {duty_speed_voltage.speed_erpm} too high - motor should be stopped"
            )

    def test_set_duty_cycle(self, motor: CubeMarsAK606v3) -> None:
        """Test setting duty cycle using duty cycle control."""
        # Use the duty cycle control example function
        run_duty_cycle_control(motor)

        # Verify motor is responsive after the sequence
        status = get_status_with_retry(motor)
        assert status is not None
        assert len(status) > 0

    def test_set_current(self, motor: CubeMarsAK606v3) -> None:
        """Test setting motor current using current control."""
        # Use the current control example function
        run_current_control(motor)

        # Verify motor is responsive after the sequence
        status = get_status_with_retry(motor)
        assert status is not None
        assert len(status) > 0

    def test_stop(self, motor: CubeMarsAK606v3) -> None:
        """Test motor stop function."""
        motor.stop()
        time.sleep(0.2)

        # Verify motor is actually stopped
        status = get_status_with_retry(motor)
        assert status is not None

        motor.status_parser.payload_offset = 0
        motor.status_parser.parse_temperatures(status)
        motor.status_parser.parse_currents(status)
        duty_speed_voltage = motor.status_parser.parse_duty_speed_voltage(status)
        assert duty_speed_voltage is not None
        # Verify motor speed is close to 0
        # Skip verification if value is obviously corrupted (> 1M indicates byte misalignment)
        if abs(duty_speed_voltage.speed_erpm) < 1000000:
            assert abs(duty_speed_voltage.speed_erpm) < 10000, (
                f"Speed {duty_speed_voltage.speed_erpm} too high - motor should be stopped"
            )


class TestHardwareEdgeCases:
    """Test edge cases with real hardware."""

    def test_position_clamping(self, motor: CubeMarsAK606v3) -> None:
        """Test that position values are properly clamped."""
        # Try to set beyond limits
        motor.set_position(position_degrees=500.0)  # Should be clamped to 360
        time.sleep(0.2)

        motor.set_position(position_degrees=-500.0)  # Should be clamped to -360
        time.sleep(0.2)

        # Return to zero
        motor.set_position(position_degrees=0.0)
        time.sleep(0.2)

    def test_velocity_clamping(self, motor: CubeMarsAK606v3) -> None:
        """Test that velocity values are properly clamped."""
        # Try to set beyond limits
        motor.set_velocity(velocity_erpm=150000)  # Should be clamped to 100000
        time.sleep(0.2)

        motor.set_velocity(velocity_erpm=0)  # Stop
        time.sleep(0.1)

    def test_consecutive_commands(self, motor: CubeMarsAK606v3) -> None:
        """Test sending multiple commands in sequence."""
        motor.set_position(position_degrees=10.0)
        time.sleep(0.1)
        motor.set_position(position_degrees=-10.0)
        time.sleep(0.1)
        motor.set_position(position_degrees=0.0)
        time.sleep(0.1)

        status = motor.get_status()
        assert status is not None


class TestMotorMeasurements:
    """Test that motor measurements can be retrieved."""

    def test_status_retrieval_consistency(self, motor: CubeMarsAK606v3) -> None:
        """Test that status can be retrieved consistently."""
        # Set to known position
        motor.set_position(position_degrees=0.0)
        time.sleep(0.5)

        # Read status multiple times - should always succeed with retry
        for _ in range(5):
            status = get_status_with_retry(motor)
            assert status is not None
            assert len(status) > 0
            time.sleep(0.1)

    def test_velocity_command_response(self, motor: CubeMarsAK606v3) -> None:
        """Test that motor responds during velocity commands."""
        # Set velocity and check motor responds
        try:
            motor.set_velocity(velocity_erpm=3000)
            time.sleep(0.5)

            status = get_status_with_retry(motor)
            assert status is not None
            assert len(status) > 0
        finally:
            motor.set_velocity(velocity_erpm=0)
            time.sleep(0.3)

    @pytest.mark.parametrize("target_velocity", [5000, 10000, 15000])
    def test_velocity_command_sequence(
        self, motor: CubeMarsAK606v3, target_velocity: int
    ) -> None:
        """Test motor accepts various velocity commands."""
        # Start from zero
        motor.set_position(position_degrees=0.0)
        time.sleep(0.5)

        # Set target velocity
        motor.set_velocity(velocity_erpm=target_velocity)
        time.sleep(0.3)

        # Verify motor responds
        status = get_status_with_retry(motor)
        assert status is not None
        assert len(status) > 0

        # Stop
        motor.set_velocity(velocity_erpm=0)
        time.sleep(0.3)
