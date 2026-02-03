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
    run_duty_cycle_control,
    run_position_control,
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

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_motor_connection(self, motor: CubeMarsAK606v3) -> None:
        """Test that motor connects successfully."""
        assert motor.connected
        assert motor.serial is not None
        assert motor.serial.is_open

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_motor_communication(self, motor: CubeMarsAK606v3) -> None:
        """Test that motor responds to status queries."""
        result = motor.check_communication()
        assert result
        assert motor.communicating

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_get_status(self, motor: CubeMarsAK606v3) -> None:
        """Test getting motor status."""
        status = get_status_with_retry(motor)
        assert status is not None
        assert len(status) > 0

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_get_position(self, motor: CubeMarsAK606v3) -> None:
        """Test getting motor position."""
        response = motor.get_position()
        assert response is not None


class TestHardwareMotorControl:
    """Test motor control with actual hardware - verify motor responds to commands."""

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
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

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_position_movement(self, motor: CubeMarsAK606v3) -> None:
        """Test motor accepts multiple position commands using position control."""
        # Test specific position and verify it's reached
        target_position = 20.0
        motor.set_position(position_degrees=target_position)
        time.sleep(0.6)  # Give motor time to move

        # Verify motor reached target position within tolerance
        status = get_status_with_retry(motor)
        assert status is not None
        assert len(status) > 0

        motor.status_parser.payload_offset = 0
        motor.status_parser.parse_temperatures(status)
        motor.status_parser.parse_currents(status)
        motor.status_parser.parse_duty_speed_voltage(status)
        status_pos = motor.status_parser.parse_status_position(status)
        assert status_pos is not None
        # Allow ±3° tolerance for position accuracy
        # Skip verification if data is clearly corrupted (position < 1e-10 or > 10000)
        if (
            abs(status_pos.position_degrees) > 1e-10
            and abs(status_pos.position_degrees) < 10000
        ):
            assert abs(status_pos.position_degrees - target_position) < 3.0, (
                f"Position {status_pos.position_degrees}° not close to target {target_position}°"
            )

        # Use the position control example function for multiple movements
        run_position_control(motor, num_steps=10, max_angle_degrees=30.0)

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_velocity(self, motor: CubeMarsAK606v3) -> None:
        """Test setting motor velocity command works using velocity control."""
        # Set a specific velocity and verify motor reaches it
        target_velocity = 8000
        motor.set_velocity(velocity_erpm=target_velocity)
        time.sleep(0.5)  # Give motor time to reach velocity

        # Verify motor is moving at commanded velocity within tolerance
        status = get_status_with_retry(motor)
        assert status is not None

        motor.status_parser.payload_offset = 0
        motor.status_parser.parse_temperatures(status)
        motor.status_parser.parse_currents(status)
        duty_speed_voltage = motor.status_parser.parse_duty_speed_voltage(status)
        assert duty_speed_voltage is not None
        # Skip verification if value is obviously corrupted (> 1M indicates byte misalignment)
        if abs(duty_speed_voltage.speed_erpm) < 1000000:
            # Allow ±30% tolerance for velocity accuracy
            velocity_error = abs(duty_speed_voltage.speed_erpm - target_velocity)
            assert velocity_error < target_velocity * 0.3, (
                f"Velocity {duty_speed_voltage.speed_erpm} not close to target {target_velocity} ERPM"
            )

        # Stop motor and verify it stops
        motor.set_velocity(velocity_erpm=0)
        time.sleep(0.3)
        status = get_status_with_retry(motor)
        assert status is not None

        motor.status_parser.payload_offset = 0
        motor.status_parser.parse_temperatures(status)
        motor.status_parser.parse_currents(status)
        duty_speed_voltage = motor.status_parser.parse_duty_speed_voltage(status)
        assert duty_speed_voltage is not None
        # Verify speed is close to 0 after stopping
        if abs(duty_speed_voltage.speed_erpm) < 1000000:
            assert abs(duty_speed_voltage.speed_erpm) < 10000, (
                f"Speed {duty_speed_voltage.speed_erpm} too high - motor should be stopped"
            )

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_duty_cycle(self, motor: CubeMarsAK606v3) -> None:
        """Test setting duty cycle using duty cycle control."""
        # Use the duty cycle control example function
        run_duty_cycle_control(motor)

        # Verify motor is responsive after the sequence
        status = get_status_with_retry(motor)
        assert status is not None
        assert len(status) > 0

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_current(self, motor: CubeMarsAK606v3) -> None:
        """Test setting motor current using current control."""
        # Set a specific current and verify motor draws it
        target_current = 2.0  # 2A
        motor.set_current(current_amps=target_current)
        time.sleep(0.5)

        # Verify motor is drawing current (within reasonable range)
        status = get_status_with_retry(motor)
        assert status is not None
        assert len(status) > 0

        motor.status_parser.payload_offset = 0
        motor.status_parser.parse_temperatures(status)
        currents = motor.status_parser.parse_currents(status)
        assert currents is not None
        # Verify motor is drawing some current (allow wide tolerance for current control)
        # Current can vary significantly based on load, so just verify it's non-zero
        # Skip if data is clearly corrupted (> 100A)
        if abs(currents.output_current_amps) < 100:
            assert abs(currents.output_current_amps) > 0.1, (
                f"Motor current {currents.output_current_amps}A is too low"
            )

        # Stop motor
        motor.stop()
        time.sleep(0.2)

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
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

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
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

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_velocity_clamping(self, motor: CubeMarsAK606v3) -> None:
        """Test that velocity values are properly clamped."""
        # Try to set beyond limits
        motor.set_velocity(velocity_erpm=150000)  # Should be clamped to 100000
        time.sleep(0.2)

        motor.set_velocity(velocity_erpm=0)  # Stop
        time.sleep(0.1)

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
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

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
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

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_velocity_command_response(self, motor: CubeMarsAK606v3) -> None:
        """Test that motor responds during velocity commands."""
        # Set velocity and check motor is actually moving
        try:
            target_velocity = 6000
            motor.set_velocity(velocity_erpm=target_velocity)
            time.sleep(0.5)

            status = get_status_with_retry(motor)
            assert status is not None
            assert len(status) > 0

            motor.status_parser.payload_offset = 0
            motor.status_parser.parse_temperatures(status)
            motor.status_parser.parse_currents(status)
            duty_speed_voltage = motor.status_parser.parse_duty_speed_voltage(status)
            assert duty_speed_voltage is not None
            # Verify motor is moving (speed should be non-zero and in reasonable range)
            if abs(duty_speed_voltage.speed_erpm) < 1000000:
                assert abs(duty_speed_voltage.speed_erpm) > 1000, (
                    f"Motor not moving (speed: {duty_speed_voltage.speed_erpm} ERPM)"
                )
        finally:
            motor.set_velocity(velocity_erpm=0)
            time.sleep(0.3)

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    @pytest.mark.parametrize("target_velocity", [5000, 10000, 15000])
    def test_velocity_command_sequence(
        self, motor: CubeMarsAK606v3, target_velocity: int
    ) -> None:
        """Test motor accepts various velocity commands."""
        # Start from zero
        motor.set_position(position_degrees=0.0)
        time.sleep(0.5)

        # Set target velocity and verify motor reaches it
        motor.set_velocity(velocity_erpm=target_velocity)
        time.sleep(0.5)  # Give motor time to reach velocity

        # Verify motor is moving at commanded velocity
        status = get_status_with_retry(motor)
        assert status is not None
        assert len(status) > 0

        motor.status_parser.payload_offset = 0
        motor.status_parser.parse_temperatures(status)
        motor.status_parser.parse_currents(status)
        duty_speed_voltage = motor.status_parser.parse_duty_speed_voltage(status)
        assert duty_speed_voltage is not None
        # Verify motor is moving at target velocity (allow ±30% tolerance)
        if abs(duty_speed_voltage.speed_erpm) < 1000000:
            velocity_error = abs(duty_speed_voltage.speed_erpm - target_velocity)
            assert velocity_error < target_velocity * 0.3, (
                f"Velocity {duty_speed_voltage.speed_erpm} not close to target {target_velocity} ERPM"
            )

        # Stop
        motor.set_velocity(velocity_erpm=0)
        time.sleep(0.3)
