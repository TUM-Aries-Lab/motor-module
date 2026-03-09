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
    CubeMarsAK606v3CAN._reset_can_interface()
    time.sleep(0.15)
    m = CubeMarsAK606v3CAN()
    if not m.connected:
        pytest.skip("CAN bus not available — run: sudo ./setup_can.sh")
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
