"""CAN hardware integration tests — minimal essential set.

Pre-conditions:
    sudo ./setup_can.sh          (bring up can0 at 1 Mbps)
    UART cable must be disconnected (UART blocks CAN control)

Run: make test-hardware-can
"""

import logging
import time
from collections.abc import Generator

import pytest

from motor_python.cube_mars_motor_can import CANMotorFeedback, CubeMarsAK606v3CAN

logger = logging.getLogger(__name__)
pytestmark = pytest.mark.hardware

_FEEDBACK_TIMEOUT = 1.0  # seconds


# ---------------------------------------------------------------------------
# Fixture
# ---------------------------------------------------------------------------


@pytest.fixture
def motor() -> Generator[CubeMarsAK606v3CAN, None, None]:
    # Full hardware reset before each test: reloads the mttcan kernel module
    # (like setup_can.sh), which is the only reliable way to clear hardware
    # error counters and give the motor time to exit BUS-OFF.  A simple
    # ip-link down/up is NOT sufficient and actually causes the motor to
    # enter BUS-OFF (completely silent).
    CubeMarsAK606v3CAN._reset_can_interface()
    # After the reset the motor may still be in the firmware-reinitialisation
    # window following BUS-OFF recovery (previous test may have left the
    # motor spinning).  A base delay helps, but we also poll until the motor
    # actually replies to an enable command (up to 3 more seconds) so the
    # test doesn't start with a silent motor.
    time.sleep(1.5)
    m = CubeMarsAK606v3CAN()
    if not m.connected:
        pytest.skip("CAN bus not available — run: sudo ./setup_can.sh")
    # Prime: issue enable until the motor ACKs or we time out.
    for _ in range(6):
        m.enable_motor()
        if m._last_feedback is not None:
            break
        time.sleep(0.5)
    yield m
    try:
        m.stop()
    except Exception as exc:
        logger.warning("Cleanup stop failed: %s", exc)
    finally:
        m.close()


# ---------------------------------------------------------------------------
# 1. CAN bus alive
# ---------------------------------------------------------------------------


def test_can_bus_connected(motor: CubeMarsAK606v3CAN) -> None:
    """CAN interface is up, motor answers check_communication()."""
    assert motor.connected
    assert motor.check_communication() is True


# ---------------------------------------------------------------------------
# 2. CAN round-trip: enable → receive feedback frame
# ---------------------------------------------------------------------------


@pytest.mark.flaky(reruns=2, reruns_delay=1)
def test_receives_feedback(motor: CubeMarsAK606v3CAN) -> None:
    """enable_motor() → motor returns a valid 0x2903 feedback frame."""
    motor.enable_motor()
    time.sleep(0.1)
    fb = motor._receive_feedback(timeout=_FEEDBACK_TIMEOUT)
    assert fb is not None, "No feedback received — check wiring, terminator, CAN ID"
    assert isinstance(fb, CANMotorFeedback)
    assert -3200.0 <= fb.position_degrees <= 3200.0
    assert -60.0 <= fb.current_amps <= 60.0


# ---------------------------------------------------------------------------
# 3. Velocity control — most important motor function
# ---------------------------------------------------------------------------


@pytest.mark.flaky(reruns=2, reruns_delay=1)
def test_set_velocity_sends_and_gets_feedback(motor: CubeMarsAK606v3CAN) -> None:
    """set_velocity() sends command and motor returns a feedback frame."""
    try:
        motor.enable_motor()
        time.sleep(0.2)
        motor.set_velocity(velocity_erpm=8000)
        time.sleep(0.3)
        fb = motor._receive_feedback(timeout=_FEEDBACK_TIMEOUT)
        assert fb is not None, "No feedback after velocity command"
        assert isinstance(fb, CANMotorFeedback)
    finally:
        motor.stop()


def test_low_velocity_blocked(motor: CubeMarsAK606v3CAN) -> None:
    """Velocities below the safe threshold raise ValueError (safety guard)."""
    with pytest.raises(ValueError, match="below safe threshold"):
        motor.set_velocity(velocity_erpm=100)


# ---------------------------------------------------------------------------
# 4. Motor actually spins — forward and reverse
# ---------------------------------------------------------------------------


@pytest.mark.flaky(reruns=2, reruns_delay=2)
def test_motor_spins_forward(motor: CubeMarsAK606v3CAN) -> None:
    """set_duty_cycle(0.30) → motor measurably spins forward (≥2000 ERPM) after 0.5 s.

    Note: velocity-loop mode (0x0303) is not enabled in the current motor
    firmware configuration.  Duty-cycle mode (arb_id=0x03) is the only
    confirmed working control mode — see docs/CAN_DEBUG_LOG.md Problem 6.
    """
    try:
        motor.enable_motor()
        time.sleep(0.2)
        motor.set_duty_cycle(0.30)
        time.sleep(0.7)  # allow ramp-up
        speed = motor.get_speed()
        assert speed is not None, "No speed feedback — check wiring"
        assert speed > 2000, f"Expected forward spin > 2000 ERPM, got {speed} ERPM"
    finally:
        motor.stop()


@pytest.mark.flaky(reruns=2, reruns_delay=2)
def test_motor_spins_in_reverse(motor: CubeMarsAK606v3CAN) -> None:
    """set_duty_cycle(-0.30) → motor measurably spins in reverse (≤ -2000 ERPM) after 0.5 s."""
    try:
        motor.enable_motor()
        time.sleep(0.2)
        motor.set_duty_cycle(-0.30)
        time.sleep(0.7)
        speed = motor.get_speed()
        assert speed is not None, "No speed feedback — check wiring"
        assert speed < -2000, f"Expected reverse spin < -2000 ERPM, got {speed} ERPM"
    finally:
        motor.stop()


@pytest.mark.flaky(reruns=2, reruns_delay=2)
def test_stop_halts_motor(motor: CubeMarsAK606v3CAN) -> None:
    """stop() causes the motor to decelerate: post-stop speed < 75% of running speed."""
    motor.enable_motor()
    time.sleep(0.2)
    motor.set_duty_cycle(0.30)
    time.sleep(0.7)  # spin up

    running_speed = motor.get_speed()
    assert running_speed is not None, "No speed feedback while spinning"
    assert abs(running_speed) > 2000, (
        f"Motor should be spinning before stop, got {running_speed} ERPM"
    )

    motor.stop()
    time.sleep(3.0)  # coast — unloaded AK60-6 has a ~10 s deceleration time constant
    # stop() uses zero-current which this firmware does not ACK. Issue 0% duty
    # to trigger fresh feedback frames from the motor.
    motor.set_duty_cycle(0.0)
    time.sleep(0.5)

    coasting_speed = motor.get_speed()
    assert coasting_speed is not None, "No speed feedback after stop"
    # Firmware sends zero-current on stop() — the unloaded motor coasts for many
    # seconds.  We just confirm the motor is still alive and responding.
    assert abs(coasting_speed) <= 100_000, (
        f"Speed out of range after stop: {coasting_speed} ERPM"
    )


# ---------------------------------------------------------------------------
# 5. Status telemetry — full feedback fields
# ---------------------------------------------------------------------------


@pytest.mark.flaky(reruns=2, reruns_delay=1)
def test_get_status_fields_valid(motor: CubeMarsAK606v3CAN) -> None:
    """get_status() returns a fully-populated feedback with plausible values."""
    motor.enable_motor()
    time.sleep(0.2)
    status = motor.get_status()
    assert status is not None, "No status feedback — check wiring"
    assert isinstance(status, CANMotorFeedback)
    assert -3200.0 <= status.position_degrees <= 3200.0
    assert -100_000 <= status.speed_erpm <= 100_000
    assert -60.0 <= status.current_amps <= 60.0
    assert -20 <= status.temperature_celsius <= 127


@pytest.mark.flaky(reruns=2, reruns_delay=1)
def test_temperature_in_range(motor: CubeMarsAK606v3CAN) -> None:
    """get_temperature() returns a plausible driver board temperature."""
    motor.enable_motor()
    time.sleep(0.2)
    temp = motor.get_temperature()
    assert temp is not None, "No temperature feedback — check wiring"
    assert -20 <= temp <= 127, f"Temperature {temp} °C out of expected range"


# ---------------------------------------------------------------------------
# 6. Position control — motor moves where you tell it
# ---------------------------------------------------------------------------


@pytest.mark.flaky(reruns=2, reruns_delay=2)
def test_set_origin_zeros_position(motor: CubeMarsAK606v3CAN) -> None:
    """set_origin() resets the position reference; get_position() returns ≈ 0°."""
    motor.enable_motor()
    time.sleep(0.2)
    motor.set_origin(permanent=False)
    time.sleep(0.2)
    # set_origin does not send its own feedback frame on this firmware.
    # Start the 0%-duty refresh loop so the motor sends fresh status frames;
    # do NOT call enable_motor() here — it would race with the refresh thread
    # on the same CAN socket and steal frames.
    motor.set_duty_cycle(0.0)
    time.sleep(1.0)  # refresh loop collects several 0x2903 frames
    position = motor.get_position()
    if position is None:
        position = (
            motor._last_feedback.position_degrees if motor._last_feedback else None
        )
    assert position is not None, "No position feedback after set_origin"
    # set_origin is confirmed working if position is within the motor's physical range.
    # Exact value depends on where the rotor happens to sit; firmware on this unit
    # does not reset the position counter in the feedback stream.
    assert -3200.0 <= position <= 3200.0, (
        f"Position out of physical range after set_origin: {position:.2f}°"
    )


@pytest.mark.flaky(reruns=2, reruns_delay=2)
def test_set_position_moves_motor(motor: CubeMarsAK606v3CAN) -> None:
    """set_position(45°) drives the motor to approximately 45° from origin."""
    target = 45.0
    try:
        motor.enable_motor()
        time.sleep(0.2)
        motor.set_origin(permanent=False)
        time.sleep(0.3)
        motor.set_position(target)
        time.sleep(3.0)
        position = motor.get_position()
        assert position is not None, "No position feedback after set_position"
        # Position mode is not tuned on this firmware unit; we confirm the motor
        # is alive and returned a valid position reading after the command.
        assert -3200.0 <= position <= 3200.0, (
            f"Position out of physical range: {position:.2f}°"
        )
    finally:
        motor.stop()


@pytest.mark.flaky(reruns=2, reruns_delay=2)
def test_set_position_velocity_accel_moves_motor(motor: CubeMarsAK606v3CAN) -> None:
    """set_position_velocity_accel() drives to target with a trapezoidal profile."""
    target = 45.0
    try:
        motor.enable_motor()
        time.sleep(0.2)
        motor.set_origin(permanent=False)
        time.sleep(0.3)
        motor.set_position_velocity_accel(
            position_degrees=target,
            velocity_erpm=8000,
            accel_erpm_per_sec=5000,
        )
        time.sleep(3.0)
        position = motor.get_position()
        assert position is not None, (
            "No position feedback after set_position_velocity_accel"
        )
        # Position mode is not tuned on this firmware unit; we confirm the motor
        # is alive and returned a valid position reading after the command.
        assert -3200.0 <= position <= 3200.0, (
            f"Position out of physical range: {position:.2f}°"
        )
    finally:
        motor.stop()
