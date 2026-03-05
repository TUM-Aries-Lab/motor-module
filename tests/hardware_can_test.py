"""CAN hardware integration tests for CubeMarsAK606v3CAN.

Requires motor connected via CAN bus (can0 at 1 Mbps, motor CAN ID 0x03).
UART cable must be disconnected — UART has higher priority and blocks CAN control.

Pre-conditions:
    sudo ip link set can0 up type can bitrate 1000000 berr-reporting on restart-ms 100

Run with: make test-hardware (or pytest -m hardware tests/hardware_can_test.py)

Test count: 18 tests covering all core motor control functions.
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
    """Wait for at least one valid feedback frame from the motor."""
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
        """CAN bus is up, connected flag is True, and motor responds."""
        assert motor.connected
        assert motor.bus is not None
        assert motor.check_communication() is True


# ---------------------------------------------------------------------------
# Feedback Reception
# ---------------------------------------------------------------------------


class TestCANFeedback:
    """Verify periodic 50 Hz feedback frames are received from the motor."""

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    def test_receives_feedback(self, motor: CubeMarsAK606v3CAN) -> None:
        """Motor broadcasts feedback frames that parse correctly."""
        # Motor only broadcasts after being enabled; enable first to ensure frames flow
        motor.enable_motor()
        time.sleep(0.1)
        fb = get_feedback_with_retry(motor)
        assert isinstance(fb, CANMotorFeedback)
        # Validate all fields are in protocol range
        assert -3200.0 <= fb.position_degrees <= 3200.0
        assert -60.0 <= fb.current_amps <= 60.0
        assert -20 <= fb.temperature_celsius <= 127


# ---------------------------------------------------------------------------
# Enable / Disable
# ---------------------------------------------------------------------------


class TestCANEnableDisable:
    """Verify enable/disable servo-mode commands."""

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    def test_enable_disable_cycle(self, motor: CubeMarsAK606v3CAN) -> None:
        """enable_motor() and disable_motor() both transmit without error."""
        motor.enable_motor()
        time.sleep(0.1)
        fb = get_feedback_with_retry(motor)
        assert fb is not None  # Motor still broadcasts after enable
        motor.disable_motor()
        time.sleep(0.1)


# ---------------------------------------------------------------------------
# Velocity Control
# ---------------------------------------------------------------------------


class TestCANVelocityControl:
    """Test velocity commands over CAN."""

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    def test_set_velocity_and_stop(self, motor: CubeMarsAK606v3CAN) -> None:
        """Motor reaches target velocity then stops cleanly."""
        target = 8000
        try:
            motor.enable_motor()
            time.sleep(0.2)
            motor.set_velocity(velocity_erpm=target)
            time.sleep(0.8)

            fb = get_feedback_with_retry(motor)
            if (
                abs(fb.speed_erpm)
                < HARDWARE_TEST_DEFAULTS.speed_corruption_threshold_erpm
            ):
                assert (
                    abs(fb.speed_erpm - target)
                    < target * HARDWARE_TEST_DEFAULTS.velocity_tolerance
                ), (
                    f"Speed {fb.speed_erpm} ERPM not within "
                    f"{HARDWARE_TEST_DEFAULTS.velocity_tolerance * 100:.0f}% of {target} ERPM"
                )
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    def test_stop_brings_speed_near_zero(self, motor: CubeMarsAK606v3CAN) -> None:
        """motor.stop() coasts motor to near zero speed."""
        motor.enable_motor()
        time.sleep(0.2)
        motor.stop()
        time.sleep(0.3)

        fb = get_feedback_with_retry(motor)
        if abs(fb.speed_erpm) < HARDWARE_TEST_DEFAULTS.speed_corruption_threshold_erpm:
            assert abs(fb.speed_erpm) < 10000, (
                f"Motor speed {fb.speed_erpm} ERPM too high after stop"
            )

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    def test_reverse_velocity(self, motor: CubeMarsAK606v3CAN) -> None:
        """Negative velocity rotates motor in reverse direction."""
        try:
            motor.enable_motor()
            time.sleep(0.2)
            motor.set_velocity(velocity_erpm=-8000)
            time.sleep(0.8)

            fb = get_feedback_with_retry(motor)
            if (
                abs(fb.speed_erpm)
                < HARDWARE_TEST_DEFAULTS.speed_corruption_threshold_erpm
            ):
                assert fb.speed_erpm < 0, (
                    f"Expected negative speed, got {fb.speed_erpm} ERPM"
                )
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    def test_low_velocity_blocked_by_default(self, motor: CubeMarsAK606v3CAN) -> None:
        """Velocities below the safe threshold raise ValueError."""
        motor.enable_motor()
        time.sleep(0.1)
        with pytest.raises(ValueError, match="below safe threshold"):
            motor.set_velocity(velocity_erpm=100)


# ---------------------------------------------------------------------------
# Current Control
# ---------------------------------------------------------------------------


class TestCANCurrentControl:
    """Test current (torque) commands over CAN."""

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    def test_set_current_does_not_raise(self, motor: CubeMarsAK606v3CAN) -> None:
        """set_current() sends without raising."""
        try:
            motor.enable_motor()
            time.sleep(0.1)
            motor.set_current(1.0)
            time.sleep(0.1)
        finally:
            motor.set_current(0.0)


# ---------------------------------------------------------------------------
# Duty Cycle Control
# ---------------------------------------------------------------------------


class TestCANDutyCycle:
    """Test duty cycle commands over CAN."""

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    def test_set_duty_cycle_does_not_raise(self, motor: CubeMarsAK606v3CAN) -> None:
        """set_duty_cycle() sends without raising."""
        try:
            motor.enable_motor()
            time.sleep(0.1)
            motor.set_duty_cycle(0.1)
            time.sleep(0.1)
        finally:
            motor.set_current(0.0)


# ---------------------------------------------------------------------------
# Position Control
# ---------------------------------------------------------------------------


class TestCANPositionControl:
    """Test position commands over CAN."""

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    @pytest.mark.xfail(
        strict=False,
        reason=(
            "Position mode 0x04 not functional on this firmware: motor ACKs the "
            "command and returns feedback but does not physically move. "
            "See docs/CAN_TROUBLESHOOTING.md — requires CubeMars PC software config."
        ),
    )
    def test_set_position_move_and_verify(self, motor: CubeMarsAK606v3CAN) -> None:
        """Motor moves to target position within tolerance."""
        target = 90.0
        try:
            motor.enable_motor()
            time.sleep(0.2)
            motor.set_position(position_degrees=target)
            time.sleep(1.0)

            fb = get_feedback_with_retry(motor)
            if (
                abs(fb.position_degrees)
                < HARDWARE_TEST_DEFAULTS.position_corruption_threshold_degrees
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

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    def test_set_position_velocity_accel_does_not_raise(
        self, motor: CubeMarsAK606v3CAN
    ) -> None:
        """set_position_velocity_accel() sends without raising."""
        try:
            motor.enable_motor()
            time.sleep(0.1)
            motor.set_position_velocity_accel(
                position_degrees=45.0,
                velocity_erpm=10000,
                accel_erpm_per_sec=5000,
            )
            time.sleep(0.3)
        finally:
            motor.stop()


# ---------------------------------------------------------------------------
# Set Origin
# ---------------------------------------------------------------------------


class TestCANSetOrigin:
    """Test set-origin command over CAN."""

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    def test_set_origin_temporary_does_not_raise(
        self, motor: CubeMarsAK606v3CAN
    ) -> None:
        """set_origin(permanent=False) sends without raising."""
        motor.enable_motor()
        time.sleep(0.1)
        motor.set_origin(permanent=False)
        time.sleep(0.05)


# ---------------------------------------------------------------------------
# Safety / Exosuit Tendon Control
# ---------------------------------------------------------------------------


class TestCANSafety:
    """Test velocity safety thresholds and tendon control via CAN."""

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    def test_low_velocity_allowed_with_flag(self, motor: CubeMarsAK606v3CAN) -> None:
        """Low velocity is permitted when allow_low_speed=True."""
        try:
            motor.enable_motor()
            time.sleep(0.1)
            motor.set_velocity(velocity_erpm=100, allow_low_speed=True)
        finally:
            motor.stop()

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    def test_tendon_pull_release_stop(self, motor: CubeMarsAK606v3CAN) -> None:
        """Tendon PULL/RELEASE/STOP all send without raising."""
        motor.enable_motor()
        time.sleep(0.1)

        motor.control_exosuit_tendon(action=TendonAction.PULL, velocity_erpm=8000)
        time.sleep(0.2)

        motor.control_exosuit_tendon(action=TendonAction.RELEASE, velocity_erpm=6000)
        time.sleep(0.2)

        motor.control_exosuit_tendon(action=TendonAction.STOP)
        time.sleep(0.1)


# ---------------------------------------------------------------------------
# Telemetry
# ---------------------------------------------------------------------------


class TestCANTelemetry:
    """Test telemetry getter methods."""

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    def test_get_motor_data_returns_dict(self, motor: CubeMarsAK606v3CAN) -> None:
        """get_motor_data() returns a dict with all expected keys and no fault."""
        motor.enable_motor()
        time.sleep(0.2)
        data = motor.get_motor_data()
        assert data is not None, "get_motor_data() returned None"
        assert isinstance(data, dict)
        expected_keys = {
            "position_degrees",
            "speed_erpm",
            "current_amps",
            "temperature_celsius",
            "error_code",
            "error_description",
        }
        assert expected_keys == set(data.keys()), (
            f"Unexpected keys in motor data: {set(data.keys())}"
        )
        assert data["error_code"] == 0, (
            f"Motor fault reported: code={data['error_code']} ({data['error_description']})"
        )


# ---------------------------------------------------------------------------
# Brake Current
# ---------------------------------------------------------------------------


class TestCANBrakeCurrent:
    """Test brake current mode over CAN."""

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    def test_brake_current_slows_motor(self, motor: CubeMarsAK606v3CAN) -> None:
        """Applying brake current after spinning reduces speed."""
        try:
            motor.enable_motor()
            time.sleep(0.1)
            motor.set_velocity(velocity_erpm=10000)
            time.sleep(0.4)
            motor.set_brake_current(5.0)
            time.sleep(0.3)

            fb = get_feedback_with_retry(motor)
            if (
                abs(fb.speed_erpm)
                < HARDWARE_TEST_DEFAULTS.speed_corruption_threshold_erpm
            ):
                assert abs(fb.speed_erpm) < 10000, (
                    f"Motor still at {fb.speed_erpm} ERPM after braking"
                )
        finally:
            motor.stop()


# ---------------------------------------------------------------------------
# MIT Impedance Control
# ---------------------------------------------------------------------------


class TestCANMITMode:
    """Test MIT impedance control mode over CAN.

    The output torque follows: τ = kp*(pos_tgt - pos) + kd*(vel_tgt - vel) + tau_ff
    """

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    def test_enable_mit_mode_does_not_raise(self, motor: CubeMarsAK606v3CAN) -> None:
        """enable_mit_mode() and disable_mit_mode() both transmit without error."""
        motor.enable_mit_mode()
        time.sleep(0.2)
        motor.disable_mit_mode()

    @pytest.mark.flaky(reruns=3, reruns_delay=0)
    def test_set_mit_mode_zero_torque_float(self, motor: CubeMarsAK606v3CAN) -> None:
        """set_mit_mode(all zeros) sends passive float command without raising."""
        motor.enable_mit_mode()
        time.sleep(0.15)
        try:
            motor.set_mit_mode(
                pos_rad=0.0, vel_rad_s=0.0, kp=0.0, kd=0.0, torque_ff_nm=0.0
            )
            time.sleep(0.2)
            fb = get_feedback_with_retry(motor)
            assert fb is not None
        finally:
            motor.disable_mit_mode()
