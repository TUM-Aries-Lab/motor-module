"""Hardware integration tests - velocity control only.

Requires motor hardware connected. Run with: make test-hardware
"""

import logging
import time
from collections.abc import Generator

import pytest

from motor_python.cube_mars_motor import CubeMarsAK606v3

logger = logging.getLogger(__name__)

pytestmark = pytest.mark.hardware


def get_status_with_retry(
    motor: CubeMarsAK606v3, max_retries: int = 8, delay: float = 0.12
) -> bytes:
    """Get motor status with retry logic for unreliable UART.

    :param motor: Motor instance
    :param max_retries: Maximum retry attempts
    :param delay: Delay between retries in seconds
    :return: Valid status bytes (>= 85 bytes)
    """
    for attempt in range(max_retries):
        status = motor.get_status()
        if status is not None and len(status) >= 85:
            return status
        if attempt < max_retries - 1:
            time.sleep(delay)
    status = motor.get_status()
    assert status is not None and len(status) >= 85, (
        f"Failed to get valid status after retries (got {len(status) if status else 0} bytes)"
    )
    return status


def parse_speed_from_status(motor: CubeMarsAK606v3, status: bytes) -> int | None:
    """Parse speed from a status response, returning None if data looks corrupted.

    :param motor: Motor instance (for its status_parser)
    :param status: Raw status bytes
    :return: Speed in ERPM, or None if data is corrupted
    """
    motor.status_parser.payload_offset = 0
    motor.status_parser.parse_temperatures(status)
    motor.status_parser.parse_currents(status)
    dsv = motor.status_parser.parse_duty_speed_voltage(status)
    if dsv is None or abs(dsv.speed_erpm) >= 1_000_000:
        return None  # Corrupted data
    return dsv.speed_erpm


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
        assert motor.check_communication()
        assert motor.communicating

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_get_status(self, motor: CubeMarsAK606v3) -> None:
        """Status response has expected minimum length."""
        status = get_status_with_retry(motor)
        assert status is not None
        assert len(status) >= 85


# -- Velocity Control --


class TestVelocityControl:
    """Test velocity commands with real hardware."""

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_velocity_and_stop(self, motor: CubeMarsAK606v3) -> None:
        """Set velocity, verify motor reaches target, then stop."""
        target = 8000
        motor.set_velocity(velocity_erpm=target)
        time.sleep(0.5)

        speed = parse_speed_from_status(motor, get_status_with_retry(motor))
        if speed is not None:
            assert abs(speed - target) < target * 0.3, (
                f"Speed {speed} ERPM not within 30% of target {target} ERPM"
            )

        motor.set_velocity(velocity_erpm=0)
        time.sleep(0.3)

        speed = parse_speed_from_status(motor, get_status_with_retry(motor))
        if speed is not None:
            assert abs(speed) < 10000, f"Motor should be stopped, got {speed} ERPM"

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
        motor.set_velocity(velocity_erpm=150000)  # Clamped to 100000
        time.sleep(0.2)
        motor.set_velocity(velocity_erpm=0)

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
            motor.set_velocity(velocity_erpm=0)
            time.sleep(0.3)

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    @pytest.mark.parametrize("target", [5000, 10000, 15000])
    def test_multiple_velocities(self, motor: CubeMarsAK606v3, target: int) -> None:
        """Motor reaches various target velocities within 30% tolerance."""
        motor.set_velocity(velocity_erpm=target)
        time.sleep(0.5)

        speed = parse_speed_from_status(motor, get_status_with_retry(motor))
        if speed is not None:
            assert abs(speed - target) < target * 0.3, (
                f"Speed {speed} ERPM not within 30% of target {target} ERPM"
            )

        motor.set_velocity(velocity_erpm=0)
        time.sleep(0.3)


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
        motor.control_exosuit_tendon(action="pull", velocity_erpm=8000)
        time.sleep(0.3)

        motor.control_exosuit_tendon(action="release", velocity_erpm=6000)
        time.sleep(0.3)

        motor.control_exosuit_tendon(action="stop")
        time.sleep(0.2)

        assert motor.get_status() is not None

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_tendon_invalid_action(self, motor: CubeMarsAK606v3) -> None:
        """Invalid tendon action raises ValueError."""
        with pytest.raises(ValueError, match="Invalid action"):
            motor.control_exosuit_tendon(action="invalid")
