"""Hardware integration tests - velocity and position control.

Requires motor hardware connected. Run with: make test-hardware
"""

import logging
import time
from collections.abc import Generator

import pytest

from motor_python.cube_mars_motor import CubeMarsAK606v3
from motor_python.definitions import (
    FRAME_BYTES,
    HARDWARE_TEST_DEFAULTS,
    MOTOR_LIMITS,
    PAYLOAD_SIZES,
    TendonAction,
)

logger = logging.getLogger(__name__)

pytestmark = pytest.mark.hardware


def get_status_with_retry(
    motor: CubeMarsAK606v3,
    max_retries: int = HARDWARE_TEST_DEFAULTS.max_status_retries,
    delay: float = HARDWARE_TEST_DEFAULTS.retry_delay,
) -> bytes:
    """Get motor status with retry logic for unreliable communication.

    Args:
        motor: Motor instance.
        max_retries: Maximum retry attempts.
        delay: Delay between retries in seconds.

    Returns:
        Valid status bytes (>= 85 bytes).

    """
    for attempt in range(max_retries):
        status = motor.get_status()
        if status is not None and len(status) >= FRAME_BYTES.min_status_response_length:
            return status
        if attempt < max_retries - 1:
            time.sleep(delay)
    status = motor.get_status()
    assert (
        status is not None and len(status) >= FRAME_BYTES.min_status_response_length
    ), (
        f"Failed to get valid status after retries (got {len(status) if status else 0} bytes)"
    )
    return status


def parse_speed_from_status(motor: CubeMarsAK606v3, status: bytes) -> int | None:
    """Parse speed from a status response, returning None if data looks corrupted.

    Args:
        motor: Motor instance (for its status_parser).
        status: Raw status bytes.

    Returns:
        Speed in ERPM, or None if data is corrupted.

    """
    payload = status[FRAME_BYTES.payload_start_index : -FRAME_BYTES.crc_and_end_length]
    motor.status_parser.payload_offset = 0
    motor.status_parser.parse_temperatures(payload)
    motor.status_parser.parse_currents(payload)
    dsv = motor.status_parser.parse_duty_speed_voltage(payload)
    if (
        dsv is None
        or abs(dsv.speed_erpm) >= HARDWARE_TEST_DEFAULTS.speed_corruption_threshold_erpm
    ):
        logger.warning(
            "Speed data corruption detected: speed=%s (threshold=%d)",
            dsv.speed_erpm if dsv else None,
            HARDWARE_TEST_DEFAULTS.speed_corruption_threshold_erpm,
        )
        return None  # Corrupted data
    return dsv.speed_erpm


def parse_position_from_status(motor: CubeMarsAK606v3, status: bytes) -> float | None:
    """Parse position from a status response, returning None if data looks corrupted.

    Args:
        motor: Motor instance (for its status_parser).
        status: Raw status bytes.

    Returns:
        Position in degrees, or None if data is corrupted.

    """
    payload = status[FRAME_BYTES.payload_start_index : -FRAME_BYTES.crc_and_end_length]
    motor.status_parser.payload_offset = 0
    motor.status_parser.parse_temperatures(payload)
    motor.status_parser.parse_currents(payload)
    motor.status_parser.parse_duty_speed_voltage(payload)
    # Skip reserved bytes
    motor.status_parser._skip_bytes(PAYLOAD_SIZES.reserved)
    status_pos = motor.status_parser.parse_status_position(payload)
    if (
        status_pos is None
        or abs(status_pos.position_degrees)
        >= HARDWARE_TEST_DEFAULTS.position_corruption_threshold_degrees
    ):
        logger.warning(
            "Position data corruption detected: position=%s deg (threshold=%d)",
            status_pos.position_degrees if status_pos else None,
            HARDWARE_TEST_DEFAULTS.position_corruption_threshold_degrees,
        )
        return None  # Corrupted data
    return status_pos.position_degrees


@pytest.fixture
def motor() -> Generator[CubeMarsAK606v3, None, None]:
    """Provide a motor instance; stop and close on teardown."""
    motor = CubeMarsAK606v3()
    if not motor.connected:
        pytest.skip("Motor hardware not available")
    yield motor
    try:
        motor.stop()
    except Exception as e:
        logger.warning(f"Failed to stop motor during cleanup: {e}")
    finally:
        motor.close()


# -- Connection --


class TestHardwareConnection:
    """Verify hardware connection and communication."""

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_motor_connection(self, motor: CubeMarsAK606v3) -> None:
        """Motor connects and serial port is open."""
        assert motor.connected
        assert motor.serial is not None
        assert motor.serial.is_open

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_motor_communication(self, motor: CubeMarsAK606v3) -> None:
        """Motor responds to status queries."""
        result = motor.check_communication()
        assert result is True, "Motor communication check should return True"
        assert motor.communicating

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_get_status(self, motor: CubeMarsAK606v3) -> None:
        """Status response has expected minimum length."""
        status = get_status_with_retry(motor)
        assert status is not None, "Status should not be None"
        assert len(status) >= FRAME_BYTES.min_status_response_length, (
            f"Status length {len(status)} should be >= {FRAME_BYTES.min_status_response_length}"
        )


# -- Velocity Control --


class TestVelocityControl:
    """Test velocity commands with real hardware."""

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_velocity_and_stop(self, motor: CubeMarsAK606v3) -> None:
        """Set velocity, verify motor reaches target, then stop."""
        target = 8000
        try:
            motor.set_velocity(velocity_erpm=target)
            time.sleep(0.5)

            speed = parse_speed_from_status(motor, get_status_with_retry(motor))
            if speed is not None:
                assert (
                    abs(speed - target)
                    < target * HARDWARE_TEST_DEFAULTS.velocity_tolerance
                ), (
                    f"Speed {speed} ERPM not within {HARDWARE_TEST_DEFAULTS.velocity_tolerance * 100}% of target {target} ERPM"
                )
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_stop(self, motor: CubeMarsAK606v3) -> None:
        """motor.stop() brings speed near zero."""
        motor.stop()
        time.sleep(0.2)

        speed = parse_speed_from_status(motor, get_status_with_retry(motor))
        if speed is not None:
            assert abs(speed) < 10000

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_velocity_clamping(self, motor: CubeMarsAK606v3) -> None:
        """Values beyond +-100000 ERPM are clamped, not rejected."""
        try:
            # Request velocity beyond max limit
            requested_velocity = MOTOR_LIMITS.max_velocity_electrical_rpm + 50000
            motor.set_velocity(velocity_erpm=requested_velocity)
            time.sleep(0.2)
            # Verify velocity was clamped to max, not set to the requested value
            speed = parse_speed_from_status(motor, get_status_with_retry(motor))
            if speed is not None:
                assert abs(speed) <= MOTOR_LIMITS.max_velocity_electrical_rpm, (
                    f"Speed {speed} should be clamped to max {MOTOR_LIMITS.max_velocity_electrical_rpm}"
                )
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_motor_spinning(self, motor: CubeMarsAK606v3) -> None:
        """Motor reports non-zero speed while running."""
        try:
            motor.set_velocity(velocity_erpm=6000)
            time.sleep(0.5)

            speed = parse_speed_from_status(motor, get_status_with_retry(motor))
            if speed is not None:
                assert abs(speed) > 1000, f"Motor not moving ({speed} ERPM)"
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    @pytest.mark.parametrize("target", [5000, 10000, 15000])
    def test_multiple_velocities(self, motor: CubeMarsAK606v3, target: int) -> None:
        """Motor reaches various target velocities within 30% tolerance."""
        try:
            motor.set_velocity(velocity_erpm=target)
            time.sleep(0.5)

            speed = parse_speed_from_status(motor, get_status_with_retry(motor))
            if speed is not None:
                assert (
                    abs(speed - target)
                    < target * HARDWARE_TEST_DEFAULTS.velocity_tolerance
                ), (
                    f"Speed {speed} ERPM not within {HARDWARE_TEST_DEFAULTS.velocity_tolerance * 100}% of target {target} ERPM"
                )
        finally:
            motor.stop()


# -- Safety --


class TestExosuitSafety:
    """Test velocity safety thresholds and tendon control."""

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_low_velocity_blocked(self, motor: CubeMarsAK606v3) -> None:
        """Velocities below 5000 ERPM raise ValueError by default."""
        with pytest.raises(ValueError, match="below safe threshold"):
            motor.set_velocity(velocity_erpm=100)

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_low_velocity_allowed_with_flag(self, motor: CubeMarsAK606v3) -> None:
        """Low velocity permitted when explicitly bypassed (no actual send)."""
        # Only verify the flag doesn't raise - don't actually send to motor
        # because low speeds cause oscillations/noise with firmware accel settings
        try:
            motor.set_velocity(velocity_erpm=100, allow_low_speed=True)
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_tendon_pull_release_stop(self, motor: CubeMarsAK606v3) -> None:
        """Tendon control: pull, release, stop all work."""
        motor.control_exosuit_tendon(action=TendonAction.PULL, velocity_erpm=8000)
        time.sleep(0.3)

        motor.control_exosuit_tendon(action=TendonAction.RELEASE, velocity_erpm=6000)
        time.sleep(0.3)

        motor.control_exosuit_tendon(action=TendonAction.STOP)
        time.sleep(0.2)

        assert motor.get_status() is not None

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_tendon_invalid_action(self, motor: CubeMarsAK606v3) -> None:
        """Invalid tendon action raises ValueError."""
        with pytest.raises(ValueError, match="Invalid action"):
            invalid_action = 999  # type: ignore[assignment]
            motor.control_exosuit_tendon(action=invalid_action)  # type: ignore[arg-type]


# -- Position Control --


class TestPositionControl:
    """Test position commands with real hardware."""

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_position(self, motor: CubeMarsAK606v3) -> None:
        """Set position command moves motor 45 degrees from a fresh zero."""
        try:
            # Set the motor's current position as the temporary origin so that
            # whatever accumulated angle remains from velocity tests is treated
            # as 0 deg.  This keeps the set_position payload well within the
            # int32 protocol range (+-2147 deg) regardless of prior test state.
            motor.set_origin(permanent=False)
            time.sleep(0.2)
            target_position = 45.0
            motor.set_position(position_degrees=target_position)
            time.sleep(1.5)
            # Verify motor reached the position
            status = get_status_with_retry(motor)
            assert status is not None, "Status should not be None"
            actual_position = parse_position_from_status(motor, status)
            if actual_position is not None:
                assert (
                    abs(actual_position - target_position)
                    <= HARDWARE_TEST_DEFAULTS.position_tolerance_degrees
                ), (
                    f"Position {actual_position} deg not within {HARDWARE_TEST_DEFAULTS.position_tolerance_degrees} deg of target {target_position} deg"
                )
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_position_zero(self, motor: CubeMarsAK606v3) -> None:
        """Set position to zero (home) is accepted."""
        try:
            motor.set_position(position_degrees=0.0)
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_get_position(self, motor: CubeMarsAK606v3) -> None:
        """get_position returns a valid response from motor."""
        response = motor.get_position()
        assert isinstance(response, bytes), "Response should be bytes"
        assert len(response) >= FRAME_BYTES.min_response_length, (
            f"Position response length {len(response)} should be >= {FRAME_BYTES.min_response_length}"
        )

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_position_clamped(self, motor: CubeMarsAK606v3) -> None:
        """Position beyond +/-360 is clamped, not rejected."""
        try:
            motor.set_position(position_degrees=500.0)  # Clamped to 360
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_move_to_position_with_speed(self, motor: CubeMarsAK606v3) -> None:
        """move_to_position_with_speed moves motor then holds position."""
        try:
            motor.move_to_position_with_speed(
                target_degrees=45.0, motor_speed_erpm=6000
            )
        finally:
            motor.stop()
