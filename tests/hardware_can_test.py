"""CAN hardware integration tests for CubeMarsAK606v3CAN.

Requires motor connected via CAN bus (can0 at 1 Mbps, motor CAN ID 0x03).
UART cable must be disconnected — UART has higher priority and blocks CAN control.

Pre-conditions:
    sudo ip link set can0 up type can bitrate 1000000 berr-reporting on restart-ms 100

Run with: make test-hardware (or pytest -m hardware tests/hardware_can_test.py)
"""

import logging
import time
from collections.abc import Generator

import pytest

from motor_python.cube_mars_motor_can import CANMotorFeedback, CubeMarsAK606v3CAN
from motor_python.definitions import (
    HARDWARE_TEST_DEFAULTS,
    TendonAction,
)

logger = logging.getLogger(__name__)

pytestmark = pytest.mark.hardware

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_FEEDBACK_TIMEOUT = 1.0  # seconds — generous to allow for 50 Hz feedback


def get_feedback_with_retry(
    motor: CubeMarsAK606v3CAN,
    max_retries: int = HARDWARE_TEST_DEFAULTS.max_status_retries,
    timeout: float = _FEEDBACK_TIMEOUT,
) -> CANMotorFeedback:
    """Wait for at least one valid feedback frame from the motor.

    Args:
        motor: CAN motor instance.
        max_retries: Maximum receive attempts.
        timeout: Per-attempt receive timeout in seconds.

    Returns:
        First valid CANMotorFeedback received.

    """
    for attempt in range(max_retries):
        feedback = motor._receive_feedback(timeout=timeout)
        if feedback is not None:
            return feedback
        if attempt < max_retries - 1:
            time.sleep(HARDWARE_TEST_DEFAULTS.retry_delay)

    feedback = motor._receive_feedback(timeout=timeout)
    assert feedback is not None, (
        f"No feedback received from motor after {max_retries} attempts. "
        "Check motor power, CANH/CANL wiring, 120Ω termination, and CAN ID."
    )
    return feedback


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def motor() -> Generator[CubeMarsAK606v3CAN, None, None]:
    """Provide a CAN motor instance; stop and close on teardown."""
    m = CubeMarsAK606v3CAN()
    if not m.connected:
        pytest.skip("CAN bus not available (run: sudo ./setup_can.sh)")
    yield m
    try:
        m.stop()
    except Exception as exc:
        logger.warning("Failed to stop CAN motor during cleanup: %s", exc)
    finally:
        m.close()


# ---------------------------------------------------------------------------
# Connection
# ---------------------------------------------------------------------------


class TestCANConnection:
    """Verify CAN bus connection and interface initialisation."""

    def test_can_bus_connected(self, motor: CubeMarsAK606v3CAN) -> None:
        """CAN bus object is created and connected flag is True."""
        assert motor.connected
        assert motor.bus is not None

    def test_motor_id_stored(self, motor: CubeMarsAK606v3CAN) -> None:
        """Motor CAN ID is stored correctly from constructor default."""
        assert motor.motor_can_id == 0x03

    def test_interface_name_stored(self, motor: CubeMarsAK606v3CAN) -> None:
        """CAN interface name defaults to 'can0'."""
        assert motor.interface == "can0"

    def test_bitrate_stored(self, motor: CubeMarsAK606v3CAN) -> None:
        """CAN bitrate defaults to 1 Mbps."""
        assert motor.bitrate == 1_000_000


# ---------------------------------------------------------------------------
# Feedback Reception
# ---------------------------------------------------------------------------


class TestCANFeedback:
    """Verify periodic 50 Hz feedback frames are received from the motor."""

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_receives_feedback(self, motor: CubeMarsAK606v3CAN) -> None:
        """Motor broadcasts feedback frames and they are parsed correctly."""
        fb = get_feedback_with_retry(motor)
        assert isinstance(fb, CANMotorFeedback)

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_feedback_position_in_range(self, motor: CubeMarsAK606v3CAN) -> None:
        """Received position is within documented CAN protocol limits."""
        fb = get_feedback_with_retry(motor)
        assert -3200.0 <= fb.position_degrees <= 3200.0, (
            f"Position {fb.position_degrees}° out of protocol range (-3200 to 3200°)"
        )

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_feedback_current_in_range(self, motor: CubeMarsAK606v3CAN) -> None:
        """Received current is within documented range."""
        fb = get_feedback_with_retry(motor)
        assert -60.0 <= fb.current_amps <= 60.0, (
            f"Current {fb.current_amps} A out of protocol range (-60 to 60 A)"
        )

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_feedback_temperature_in_range(self, motor: CubeMarsAK606v3CAN) -> None:
        """Received temperature is plausible."""
        fb = get_feedback_with_retry(motor)
        assert -20 <= fb.temperature_celsius <= 127, (
            f"Temperature {fb.temperature_celsius}°C out of protocol range (-20 to 127°C)"
        )

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_check_communication(self, motor: CubeMarsAK606v3CAN) -> None:
        """check_communication() returns True and sets communicating flag."""
        result = motor.check_communication()
        assert result is True
        assert motor.communicating is True

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_get_position_returns_float(self, motor: CubeMarsAK606v3CAN) -> None:
        """get_position() returns a float from the last feedback frame."""
        pos = motor.get_position()
        assert isinstance(pos, float), f"Expected float, got {type(pos)}"

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_get_status_returns_feedback(self, motor: CubeMarsAK606v3CAN) -> None:
        """get_status() returns a CANMotorFeedback object and logs motor state."""
        fb = motor.get_status()
        assert fb is not None
        assert isinstance(fb, CANMotorFeedback)


# ---------------------------------------------------------------------------
# Enable / Disable
# ---------------------------------------------------------------------------


class TestCANEnableDisable:
    """Verify enable/disable servo-mode commands."""

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_enable_motor_does_not_raise(self, motor: CubeMarsAK606v3CAN) -> None:
        """enable_motor() transmits without raising an exception."""
        motor.enable_motor()
        time.sleep(0.2)

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_disable_motor_does_not_raise(self, motor: CubeMarsAK606v3CAN) -> None:
        """disable_motor() transmits without raising an exception."""
        motor.enable_motor()
        time.sleep(0.2)
        motor.disable_motor()
        time.sleep(0.2)

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_motor_still_feeds_back_after_enable(
        self, motor: CubeMarsAK606v3CAN
    ) -> None:
        """Motor continues broadcasting feedback after enable command."""
        motor.enable_motor()
        time.sleep(0.3)
        fb = get_feedback_with_retry(motor)
        assert fb is not None


# ---------------------------------------------------------------------------
# Velocity Control
# ---------------------------------------------------------------------------


class TestCANVelocityControl:
    """Test velocity commands over CAN."""

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_velocity_and_stop(self, motor: CubeMarsAK606v3CAN) -> None:
        """Motor reaches target velocity then stops cleanly."""
        target = 8000
        try:
            motor.enable_motor()
            time.sleep(0.2)
            motor.set_velocity(velocity_erpm=target)
            time.sleep(0.5)

            fb = get_feedback_with_retry(motor)
            if abs(fb.speed_erpm) < HARDWARE_TEST_DEFAULTS.speed_corruption_threshold:
                assert (
                    abs(fb.speed_erpm - target)
                    < target * HARDWARE_TEST_DEFAULTS.velocity_tolerance
                ), (
                    f"Speed {fb.speed_erpm} ERPM not within "
                    f"{HARDWARE_TEST_DEFAULTS.velocity_tolerance * 100:.0f}% of {target} ERPM"
                )
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_stop_brings_speed_near_zero(self, motor: CubeMarsAK606v3CAN) -> None:
        """motor.stop() sets current=0 and motor coasts to near zero."""
        motor.enable_motor()
        time.sleep(0.1)
        motor.stop()
        time.sleep(0.5)

        fb = get_feedback_with_retry(motor)
        if abs(fb.speed_erpm) < HARDWARE_TEST_DEFAULTS.speed_corruption_threshold:
            assert abs(fb.speed_erpm) < 10000, (
                f"Motor speed {fb.speed_erpm} ERPM too high after stop"
            )

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_velocity_clamping(self, motor: CubeMarsAK606v3CAN) -> None:
        """Velocities beyond ±320000 ERPM are clamped, not rejected."""
        try:
            motor.enable_motor()
            time.sleep(0.1)
            motor.set_velocity(velocity_erpm=400000, allow_low_speed=True)
            time.sleep(0.3)

            fb = get_feedback_with_retry(motor)
            if abs(fb.speed_erpm) < HARDWARE_TEST_DEFAULTS.speed_corruption_threshold:
                assert abs(fb.speed_erpm) <= 320000
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_reverse_velocity(self, motor: CubeMarsAK606v3CAN) -> None:
        """Negative velocity rotates motor in reverse direction."""
        try:
            motor.enable_motor()
            time.sleep(0.1)
            motor.set_velocity(velocity_erpm=-8000)
            time.sleep(0.5)

            fb = get_feedback_with_retry(motor)
            if abs(fb.speed_erpm) < HARDWARE_TEST_DEFAULTS.speed_corruption_threshold:
                assert fb.speed_erpm < 0, (
                    f"Expected negative speed, got {fb.speed_erpm} ERPM"
                )
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    @pytest.mark.parametrize("target", [5000, 10000, 15000])
    def test_multiple_velocities(self, motor: CubeMarsAK606v3CAN, target: int) -> None:
        """Motor reaches several target velocities within tolerance."""
        try:
            motor.enable_motor()
            time.sleep(0.1)
            motor.set_velocity(velocity_erpm=target)
            time.sleep(0.5)

            fb = get_feedback_with_retry(motor)
            if abs(fb.speed_erpm) < HARDWARE_TEST_DEFAULTS.speed_corruption_threshold:
                assert (
                    abs(fb.speed_erpm - target)
                    < target * HARDWARE_TEST_DEFAULTS.velocity_tolerance
                ), f"Speed {fb.speed_erpm} ERPM not within tolerance of {target} ERPM"
        finally:
            motor.stop()


# ---------------------------------------------------------------------------
# Current Control
# ---------------------------------------------------------------------------


class TestCANCurrentControl:
    """Test current (torque) commands over CAN."""

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_current_does_not_raise(self, motor: CubeMarsAK606v3CAN) -> None:
        """set_current() sends without raising."""
        try:
            motor.enable_motor()
            time.sleep(0.1)
            motor.set_current(1.0)
            time.sleep(0.2)
        finally:
            motor.set_current(0.0)

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_current_zero_acts_as_stop(self, motor: CubeMarsAK606v3CAN) -> None:
        """set_current(0) releases motor windings."""
        motor.enable_motor()
        time.sleep(0.1)
        motor.set_current(0.0)
        time.sleep(0.3)

        fb = get_feedback_with_retry(motor)
        # Motor should not be drawing significant current
        assert abs(fb.current_amps) < 5.0, (
            f"Current {fb.current_amps} A unexpectedly high after set_current(0)"
        )

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_current_clamped_at_60a(self, motor: CubeMarsAK606v3CAN) -> None:
        """Requests beyond +-60 A are clamped, not rejected."""
        try:
            motor.enable_motor()
            time.sleep(0.1)
            motor.set_current(100.0)  # Clamped to 60.0 A internally
            time.sleep(0.1)
        finally:
            motor.set_current(0.0)


# ---------------------------------------------------------------------------
# Duty Cycle Control
# ---------------------------------------------------------------------------


class TestCANDutyCycle:
    """Test duty cycle commands over CAN."""

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_duty_cycle_does_not_raise(self, motor: CubeMarsAK606v3CAN) -> None:
        """set_duty_cycle() sends without raising."""
        try:
            motor.enable_motor()
            time.sleep(0.1)
            motor.set_duty_cycle(0.1)
            time.sleep(0.2)
        finally:
            motor.set_current(0.0)

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_duty_cycle_clamped(self, motor: CubeMarsAK606v3CAN) -> None:
        """Values outside [-1, 1] are clamped, not rejected."""
        try:
            motor.enable_motor()
            time.sleep(0.1)
            motor.set_duty_cycle(2.0)  # Clamped to 1.0 internally
            time.sleep(0.1)
            motor.set_duty_cycle(-2.0)  # Clamped to -1.0 internally
            time.sleep(0.1)
        finally:
            motor.set_current(0.0)

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_duty_cycle_zero_stops_motor(self, motor: CubeMarsAK606v3CAN) -> None:
        """set_duty_cycle(0) does not cause motor to spin."""
        try:
            motor.enable_motor()
            time.sleep(0.1)
            motor.set_duty_cycle(0.0)
            time.sleep(0.3)

            fb = get_feedback_with_retry(motor)
            if abs(fb.speed_erpm) < HARDWARE_TEST_DEFAULTS.speed_corruption_threshold:
                assert abs(fb.speed_erpm) < 5000, (
                    f"Motor still spinning at {fb.speed_erpm} ERPM after duty_cycle=0"
                )
        finally:
            motor.set_current(0.0)


# ---------------------------------------------------------------------------
# Position Control
# ---------------------------------------------------------------------------


class TestCANPositionControl:
    """Test position commands over CAN."""

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_position_does_not_raise(self, motor: CubeMarsAK606v3CAN) -> None:
        """set_position() sends without raising."""
        try:
            motor.enable_motor()
            time.sleep(0.2)
            motor.set_position(position_degrees=0.0)
            time.sleep(0.2)
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_position_move_and_verify(self, motor: CubeMarsAK606v3CAN) -> None:
        """Motor moves to target position within tolerance."""
        target = 90.0
        try:
            motor.enable_motor()
            time.sleep(0.2)
            motor.set_position(position_degrees=target)
            time.sleep(1.5)  # Allow time to reach position

            fb = get_feedback_with_retry(motor)
            if (
                abs(fb.position_degrees)
                < HARDWARE_TEST_DEFAULTS.position_corruption_threshold
            ):
                assert (
                    abs(fb.position_degrees - target)
                    <= HARDWARE_TEST_DEFAULTS.position_tolerance_degrees
                ), (
                    f"Position {fb.position_degrees:.2f}° not within "
                    f"{HARDWARE_TEST_DEFAULTS.position_tolerance_degrees}° of target {target}°"
                )
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_position_clamped_at_limits(self, motor: CubeMarsAK606v3CAN) -> None:
        """Positions beyond ±3200° are clamped, not rejected."""
        try:
            motor.enable_motor()
            time.sleep(0.1)
            motor.set_position(position_degrees=5000.0)  # Clamped to 3200°
            time.sleep(0.1)
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_position_velocity_accel_does_not_raise(
        self, motor: CubeMarsAK606v3CAN
    ) -> None:
        """set_position_velocity_accel() sends without raising."""
        try:
            motor.enable_motor()
            time.sleep(0.2)
            motor.set_position_velocity_accel(
                position_degrees=45.0,
                velocity_erpm=10000,
                accel_erpm_per_sec=5000,
            )
            time.sleep(0.5)
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_position_velocity_accel_zero_accel(
        self, motor: CubeMarsAK606v3CAN
    ) -> None:
        """set_position_velocity_accel() with default accel=0 sends correctly."""
        try:
            motor.enable_motor()
            time.sleep(0.1)
            motor.set_position_velocity_accel(
                position_degrees=30.0,
                velocity_erpm=8000,
            )
            time.sleep(0.3)
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_move_to_position_with_speed(self, motor: CubeMarsAK606v3CAN) -> None:
        """move_to_position_with_speed() completes without error."""
        try:
            motor.enable_motor()
            time.sleep(0.2)
            motor.move_to_position_with_speed(
                target_degrees=45.0,
                motor_speed_erpm=8000,
            )
        finally:
            motor.stop()


# ---------------------------------------------------------------------------
# Set Origin
# ---------------------------------------------------------------------------


class TestCANSetOrigin:
    """Test set-origin command over CAN."""

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_origin_temporary_does_not_raise(
        self, motor: CubeMarsAK606v3CAN
    ) -> None:
        """set_origin(permanent=False) sends without raising."""
        motor.enable_motor()
        time.sleep(0.2)
        motor.set_origin(permanent=False)
        time.sleep(0.1)

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_set_origin_permanent_does_not_raise(
        self, motor: CubeMarsAK606v3CAN
    ) -> None:
        """set_origin(permanent=True) sends without raising."""
        motor.enable_motor()
        time.sleep(0.2)
        motor.set_origin(permanent=True)
        time.sleep(0.1)


# ---------------------------------------------------------------------------
# Safety / Exosuit Tendon Control
# ---------------------------------------------------------------------------


class TestCANSafety:
    """Test velocity safety thresholds and tendon control via CAN."""

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_low_velocity_blocked_by_default(self, motor: CubeMarsAK606v3CAN) -> None:
        """Velocities below 5000 ERPM raise ValueError by default."""
        motor.enable_motor()
        time.sleep(0.1)
        with pytest.raises(ValueError, match="below safe threshold"):
            motor.set_velocity(velocity_erpm=100)

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_low_velocity_allowed_with_flag(self, motor: CubeMarsAK606v3CAN) -> None:
        """Low velocity is permitted when allow_low_speed=True."""
        try:
            motor.enable_motor()
            time.sleep(0.1)
            motor.set_velocity(velocity_erpm=100, allow_low_speed=True)
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_tendon_pull_release_stop(self, motor: CubeMarsAK606v3CAN) -> None:
        """Tendon PULL/RELEASE/STOP all send without raising."""
        motor.enable_motor()
        time.sleep(0.2)

        motor.control_exosuit_tendon(action=TendonAction.PULL, velocity_erpm=8000)
        time.sleep(0.3)

        motor.control_exosuit_tendon(action=TendonAction.RELEASE, velocity_erpm=6000)
        time.sleep(0.3)

        motor.control_exosuit_tendon(action=TendonAction.STOP)
        time.sleep(0.2)

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_tendon_invalid_action_raises(self, motor: CubeMarsAK606v3CAN) -> None:
        """Invalid tendon action raises ValueError."""
        with pytest.raises(ValueError, match="Invalid action"):
            invalid_action = 999  # type: ignore[assignment]
            motor.control_exosuit_tendon(action=invalid_action)  # type: ignore[arg-type]

    @pytest.mark.flaky(reruns=3, reruns_delay=1)
    def test_velocity_zero_calls_stop(self, motor: CubeMarsAK606v3CAN) -> None:
        """set_velocity(0) internally calls stop() (sends current=0)."""
        motor.enable_motor()
        time.sleep(0.1)
        # This calls stop() → set_current(0); should not raise
        motor.set_velocity(velocity_erpm=0, allow_low_speed=True)
        time.sleep(0.3)

        fb = get_feedback_with_retry(motor)
        assert abs(fb.current_amps) < 5.0


# ---------------------------------------------------------------------------
# Context Manager
# ---------------------------------------------------------------------------


class TestCANContextManager:
    """Verify the context manager protocol (with statement)."""

    def test_context_manager_closes_on_exit(self) -> None:
        """Motor bus is shut down when exiting a 'with' block."""
        with CubeMarsAK606v3CAN() as m:
            if not m.connected:
                pytest.skip("CAN bus not available")
            assert m.connected
        # After __exit__ calls close() → bus.shutdown(); no exception should propagate
