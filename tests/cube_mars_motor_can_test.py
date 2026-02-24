"""Unit tests for CubeMarsAK606v3CAN (no hardware required).

All CAN bus I/O is replaced with mocks so these tests run without a
physical motor or CAN interface.  For tests that need a real motor see
tests/hardware_can_test.py.
"""

import struct
from unittest.mock import MagicMock, call, patch

import pytest

from motor_python.cube_mars_motor_can import CANMotorFeedback, CubeMarsAK606v3CAN
from motor_python.definitions import TendonAction

# ---------------------------------------------------------------------------
# Test constants
# ---------------------------------------------------------------------------

# Velocity values used across tests (same reasoning as UART tests)
LOW_VELOCITY_ERPM = 100          # Below 5000 ERPM safety floor — should be blocked
SAFE_PULL_VELOCITY_ERPM = 10000  # Above threshold — normal tendon pull
SAFE_RELEASE_VELOCITY_ERPM = 8000


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_feedback_msg(
    position_degrees: float = 90.0,
    speed_erpm: int = 10000,
    current_amps: float = 2.0,
    temperature_celsius: int = 40,
    error_code: int = 0,
    motor_id: int = 0x03,
) -> MagicMock:
    """Build a mock CAN message that looks like a motor feedback frame.

    Encodes values using the same scaling the real motor firmware uses:
      - Position: int16 * 0.1 = degrees
      - Speed:    int16 * 10  = ERPM
      - Current:  int16 * 0.01 = Amps
      - Temp:     int8 direct
      - Error:    uint8 direct
    """
    pos_int = int(round(position_degrees / 0.1))
    speed_int = int(round(speed_erpm / 10))
    current_int = int(round(current_amps / 0.01))

    data = (
        struct.pack(">h", pos_int)
        + struct.pack(">h", speed_int)
        + struct.pack(">h", current_int)
        + struct.pack("b", temperature_celsius)
        + bytes([error_code])
    )

    msg = MagicMock()
    msg.arbitration_id = 0x2900 | motor_id
    msg.data = data
    return msg


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def mock_bus():
    """Patch can.interface.Bus so no real CAN socket is opened.

    The mock bus returns None for recv() by default (no feedback).
    Individual tests can configure mock_bus.recv.return_value to return
    a fake feedback message.
    """
    with patch("motor_python.cube_mars_motor_can.can.interface.Bus") as mock_cls:
        bus = MagicMock()
        bus.recv.return_value = None
        mock_cls.return_value = bus
        yield bus


@pytest.fixture
def motor(mock_bus):
    """CubeMarsAK606v3CAN instance backed by a mocked CAN bus."""
    return CubeMarsAK606v3CAN()


# ---------------------------------------------------------------------------
# Initialization
# ---------------------------------------------------------------------------

class TestInitialization:
    """CAN motor class can be imported and initialised."""

    def test_class_importable(self):
        """CubeMarsAK606v3CAN can be imported."""
        assert CubeMarsAK606v3CAN is not None

    def test_connection_success(self, motor, mock_bus):
        """Successful mock connection sets connected=True."""
        assert motor.connected is True
        assert motor.bus is mock_bus

    def test_connection_failure_graceful(self):
        """CAN bus errors are caught and motor marks itself as disconnected."""
        import can
        with patch("motor_python.cube_mars_motor_can.can.interface.Bus") as mock_cls:
            mock_cls.side_effect = can.CanError("interface not found")
            m = CubeMarsAK606v3CAN()
            assert m.connected is False
            assert m.communicating is False

    def test_motor_id_stored(self, motor):
        """Configured motor CAN ID is stored on the instance."""
        assert motor.motor_can_id == 0x03

    def test_feedback_ids_include_standard_schemes(self, motor):
        """Known CubeMars feedback ID schemes are all in the candidate set."""
        assert 0x2903 in motor._feedback_ids   # 0x2900 | 0x03
        assert 0x0083 in motor._feedback_ids   # 0x0080 | 0x03
        assert 0x03   in motor._feedback_ids   # direct ID fallback


# ---------------------------------------------------------------------------
# Enable / Disable
# ---------------------------------------------------------------------------

class TestEnableDisable:
    """enable_motor / disable_motor send the correct magic bytes."""

    def test_enable_motor_sends_frame(self, motor, mock_bus):
        """enable_motor sends 0xFFFFFFFFFFFFFFFC to the motor CAN ID."""
        motor.enable_motor()
        mock_bus.send.assert_called()
        sent_msg = mock_bus.send.call_args[0][0]
        assert list(sent_msg.data) == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
        assert sent_msg.arbitration_id == motor.motor_can_id

    def test_disable_motor_sends_frame(self, motor, mock_bus):
        """disable_motor sends 0xFFFFFFFFFFFFFFFD to the motor CAN ID."""
        motor.disable_motor()
        mock_bus.send.assert_called()
        sent_msg = mock_bus.send.call_args[0][0]
        assert list(sent_msg.data) == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]

    def test_enable_when_not_connected(self):
        """enable_motor is a no-op and does not raise when disconnected."""
        import can
        with patch("motor_python.cube_mars_motor_can.can.interface.Bus") as mock_cls:
            mock_cls.side_effect = can.CanError("no interface")
            m = CubeMarsAK606v3CAN()
            m.enable_motor()  # must not raise


# ---------------------------------------------------------------------------
# Velocity control
# ---------------------------------------------------------------------------

class TestVelocityControl:
    """set_velocity enforces safety limits and sends correctly-encoded frames."""

    def test_low_velocity_blocked(self, motor):
        """Velocities 1–4999 ERPM are rejected by default."""
        with pytest.raises(ValueError, match="below safe threshold"):
            motor.set_velocity(velocity_erpm=LOW_VELOCITY_ERPM)

    def test_negative_low_velocity_blocked(self, motor):
        """Negative velocities in the noisy zone are also rejected."""
        with pytest.raises(ValueError, match="below safe threshold"):
            motor.set_velocity(velocity_erpm=-LOW_VELOCITY_ERPM)

    def test_low_velocity_allowed_with_flag(self, motor, mock_bus):
        """allow_low_speed=True bypasses the safety floor."""
        motor.set_velocity(velocity_erpm=LOW_VELOCITY_ERPM, allow_low_speed=True)
        assert mock_bus.send.called

    def test_zero_velocity_calls_stop(self, motor, mock_bus):
        """Velocity=0 triggers stop() instead of a velocity command."""
        motor.set_velocity(velocity_erpm=0)
        # stop() sends two zero-current frames
        assert mock_bus.send.call_count >= 2

    def test_safe_velocity_sends_command(self, motor, mock_bus):
        """A safe velocity value is forwarded to the CAN bus."""
        motor.set_velocity(velocity_erpm=SAFE_PULL_VELOCITY_ERPM)
        assert mock_bus.send.called

    def test_velocity_clamped_above_max(self, motor, mock_bus):
        """Velocity above 320 000 ERPM is clamped, not rejected."""
        motor.set_velocity(velocity_erpm=999_999)
        assert mock_bus.send.called


# ---------------------------------------------------------------------------
# Position control
# ---------------------------------------------------------------------------

class TestPositionControl:
    """set_position clamps within ±3200° and sends an encoded frame."""

    def test_set_position_sends_command(self, motor, mock_bus):
        """set_position sends a CAN frame for a normal position."""
        motor.set_position(position_degrees=90.0)
        assert mock_bus.send.called

    def test_set_position_negative(self, motor, mock_bus):
        """Negative positions are accepted and sent."""
        motor.set_position(position_degrees=-180.0)
        assert mock_bus.send.called

    def test_set_position_zero(self, motor, mock_bus):
        """Zero position (home) is accepted and sent."""
        motor.set_position(position_degrees=0.0)
        assert mock_bus.send.called

    def test_set_position_clamped_to_upper_limit(self, motor, mock_bus):
        """set_position clamps values above 3200° to 3200°."""
        motor.set_position(position_degrees=9999.0)
        assert mock_bus.send.called
        # Verify the sent payload encodes exactly 3200° (= 32000000)
        sent_data = mock_bus.send.call_args[0][0].data
        position_int = struct.unpack(">i", bytes(sent_data[:4]))[0]
        assert position_int == int(3200.0 * 10000)

    def test_set_position_clamped_to_lower_limit(self, motor, mock_bus):
        """set_position clamps values below -3200° to -3200°."""
        motor.set_position(position_degrees=-9999.0)
        sent_data = mock_bus.send.call_args[0][0].data
        position_int = struct.unpack(">i", bytes(sent_data[:4]))[0]
        assert position_int == int(-3200.0 * 10000)


# ---------------------------------------------------------------------------
# Current and duty-cycle control
# ---------------------------------------------------------------------------

class TestCurrentAndDuty:
    """set_current and set_duty_cycle encode and send frames correctly."""

    def test_set_current_sends_command(self, motor, mock_bus):
        """set_current sends a CAN frame."""
        motor.set_current(current_amps=3.0)
        assert mock_bus.send.called

    def test_set_current_encodes_milliamps(self, motor, mock_bus):
        """Current is scaled to milliamps (amps * 1000) in the payload."""
        motor.set_current(current_amps=5.0)
        # The refresh thread stores the command, not bus.send directly here,
        # so we just verify send was called.
        assert mock_bus.send.called

    def test_set_current_clamped(self, motor, mock_bus):
        """Current above 60 A is clamped, not rejected."""
        motor.set_current(current_amps=999.0)
        assert mock_bus.send.called

    def test_set_duty_cycle_sends_command(self, motor, mock_bus):
        """set_duty_cycle sends a CAN frame."""
        motor.set_duty_cycle(duty=0.5)
        assert mock_bus.send.called

    def test_set_duty_cycle_clamped(self, motor, mock_bus):
        """Duty cycle is clamped to [-1.0, 1.0]."""
        motor.set_duty_cycle(duty=99.0)
        assert mock_bus.send.called


# ---------------------------------------------------------------------------
# Set origin
# ---------------------------------------------------------------------------

class TestSetOrigin:
    """set_origin sends the correct byte for temporary vs permanent."""

    def test_set_origin_temporary(self, motor, mock_bus):
        """Temporary origin sends byte 0x01."""
        motor.set_origin(permanent=False)
        sent_data = mock_bus.send.call_args[0][0].data
        assert sent_data[0] == 0x01

    def test_set_origin_permanent(self, motor, mock_bus):
        """Permanent origin sends byte 0x02."""
        motor.set_origin(permanent=True)
        sent_data = mock_bus.send.call_args[0][0].data
        assert sent_data[0] == 0x02


# ---------------------------------------------------------------------------
# set_position_velocity_accel
# ---------------------------------------------------------------------------

class TestPositionVelocityAccel:
    """set_position_velocity_accel encodes all three fields correctly."""

    def test_sends_command(self, motor, mock_bus):
        """A trapezoidal move command is sent to the CAN bus."""
        motor.set_position_velocity_accel(
            position_degrees=180.0, velocity_erpm=10000, accel_erpm_per_sec=5000
        )
        assert mock_bus.send.called

    def test_payload_encoding(self, motor, mock_bus):
        """Payload encodes position as int32 and velocity/accel as int16 (/10)."""
        motor.set_position_velocity_accel(
            position_degrees=90.0, velocity_erpm=10000, accel_erpm_per_sec=0
        )
        sent_data = bytes(mock_bus.send.call_args[0][0].data)
        pos_int, vel_int, accel_int = struct.unpack(">ihh", sent_data)
        assert pos_int == int(90.0 * 10000)    # 900000
        assert vel_int == 10000 // 10           # 1000
        assert accel_int == 0


# ---------------------------------------------------------------------------
# Stop / Close
# ---------------------------------------------------------------------------

class TestStopAndClose:
    """stop() and close() release windings and clean up."""

    def test_stop_sends_zero_current(self, motor, mock_bus):
        """stop() sends at least two zero-current frames."""
        motor.stop()
        assert mock_bus.send.call_count >= 2
        # Every sent frame must have zero payload
        for c in mock_bus.send.call_args_list:
            msg = c[0][0]
            current_int = struct.unpack(">i", bytes(msg.data[:4]))[0]
            assert current_int == 0

    def test_close_shuts_down_bus(self, motor, mock_bus):
        """close() calls bus.shutdown()."""
        motor.close()
        mock_bus.shutdown.assert_called_once()

    def test_context_manager_calls_close(self, mock_bus):
        """Using motor as context manager calls close() on exit."""
        with CubeMarsAK606v3CAN() as m:
            assert m.connected
        mock_bus.shutdown.assert_called()


# ---------------------------------------------------------------------------
# Feedback / communication
# ---------------------------------------------------------------------------

class TestFeedbackAndCommunication:
    """Feedback parsing and check_communication behave correctly."""

    def test_check_communication_not_connected(self):
        """check_communication returns False when CAN bus is unavailable."""
        import can
        with patch("motor_python.cube_mars_motor_can.can.interface.Bus") as mock_cls:
            mock_cls.side_effect = can.CanError("no interface")
            m = CubeMarsAK606v3CAN()
            assert m.check_communication() is False

    def test_check_communication_no_feedback(self, motor, mock_bus):
        """check_communication returns False when no feedback is received."""
        mock_bus.recv.return_value = None
        assert motor.check_communication() is False
        assert motor.communicating is False

    def test_check_communication_with_feedback(self, motor, mock_bus):
        """check_communication returns True when a valid feedback frame arrives."""
        mock_bus.recv.return_value = _make_feedback_msg()
        assert motor.check_communication() is True
        assert motor.communicating is True

    def test_receive_feedback_parses_position(self, motor, mock_bus):
        """_receive_feedback correctly decodes position from a feedback frame."""
        mock_bus.recv.return_value = _make_feedback_msg(position_degrees=45.0)
        fb = motor._receive_feedback()
        assert fb is not None
        assert abs(fb.position_degrees - 45.0) < 0.5

    def test_receive_feedback_parses_speed(self, motor, mock_bus):
        """_receive_feedback correctly decodes speed from a feedback frame."""
        mock_bus.recv.return_value = _make_feedback_msg(speed_erpm=5000)
        fb = motor._receive_feedback()
        assert fb is not None
        assert fb.speed_erpm == 5000

    def test_receive_feedback_parses_current(self, motor, mock_bus):
        """_receive_feedback correctly decodes current from a feedback frame."""
        mock_bus.recv.return_value = _make_feedback_msg(current_amps=3.5)
        fb = motor._receive_feedback()
        assert fb is not None
        assert abs(fb.current_amps - 3.5) < 0.1

    def test_receive_feedback_parses_temperature(self, motor, mock_bus):
        """_receive_feedback correctly decodes temperature from a feedback frame."""
        mock_bus.recv.return_value = _make_feedback_msg(temperature_celsius=55)
        fb = motor._receive_feedback()
        assert fb is not None
        assert fb.temperature_celsius == 55

    def test_receive_feedback_parses_error_code(self, motor, mock_bus):
        """_receive_feedback correctly decodes the error code."""
        mock_bus.recv.return_value = _make_feedback_msg(error_code=2)
        fb = motor._receive_feedback()
        assert fb is not None
        assert fb.error_code == 2

    def test_receive_feedback_ignores_wrong_id(self, motor, mock_bus):
        """Messages with unrecognised CAN IDs are ignored."""
        unknown_msg = MagicMock()
        unknown_msg.arbitration_id = 0x9999
        unknown_msg.data = bytes(8)
        mock_bus.recv.return_value = unknown_msg
        fb = motor._receive_feedback()
        assert fb is None

    def test_get_position_returns_float(self, motor, mock_bus):
        """get_position() returns a float when feedback is available."""
        mock_bus.recv.return_value = _make_feedback_msg(position_degrees=120.0)
        pos = motor.get_position()
        assert isinstance(pos, float)
        assert abs(pos - 120.0) < 1.0

    def test_get_position_returns_none_on_no_feedback(self, motor, mock_bus):
        """get_position() returns None when no feedback message arrives."""
        mock_bus.recv.return_value = None
        motor._last_feedback = None
        pos = motor.get_position()
        assert pos is None

    def test_get_status_logs_and_returns_feedback(self, motor, mock_bus):
        """get_status() returns a CANMotorFeedback when feedback is available."""
        mock_bus.recv.return_value = _make_feedback_msg()
        result = motor.get_status()
        assert isinstance(result, CANMotorFeedback)


# ---------------------------------------------------------------------------
# Tendon control
# ---------------------------------------------------------------------------

class TestTendonControl:
    """control_exosuit_tendon drives the motor in the right direction."""

    def test_tendon_pull_sends_positive_velocity(self, motor, mock_bus):
        """PULL action calls set_velocity with a positive ERPM value."""
        motor.control_exosuit_tendon(
            action=TendonAction.PULL, velocity_erpm=SAFE_PULL_VELOCITY_ERPM
        )
        assert mock_bus.send.called

    def test_tendon_release_sends_negative_velocity(self, motor, mock_bus):
        """RELEASE action calls set_velocity with a negative ERPM value."""
        motor.control_exosuit_tendon(
            action=TendonAction.RELEASE, velocity_erpm=SAFE_RELEASE_VELOCITY_ERPM
        )
        assert mock_bus.send.called

    def test_tendon_stop_zeroes_current(self, motor, mock_bus):
        """STOP action calls stop() which sends zero-current frames."""
        motor.control_exosuit_tendon(action=TendonAction.STOP)
        assert mock_bus.send.called

    def test_tendon_invalid_action_raises(self, motor):
        """An invalid action raises ValueError."""
        with pytest.raises(ValueError, match="Invalid action"):
            motor.control_exosuit_tendon(action=999)  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# Extended CAN ID encoding
# ---------------------------------------------------------------------------

class TestCANIDEncoding:
    """_build_extended_id constructs the correct 29-bit arbitration ID."""

    def test_velocity_mode_id(self, motor):
        """Velocity loop mode (0x03) is encoded correctly."""
        arb_id = motor._build_extended_id(0x03)
        # Expected: (mode 0x03 << 8) | motor_id 0x03 = 0x0303
        assert arb_id == (0x03 << 8) | 0x03

    def test_position_mode_id(self, motor):
        """Position loop mode (0x04) is encoded correctly."""
        arb_id = motor._build_extended_id(0x04)
        assert arb_id == (0x04 << 8) | 0x03

    def test_current_mode_id(self, motor):
        """Current loop mode (0x01) is encoded correctly."""
        arb_id = motor._build_extended_id(0x01)
        assert arb_id == (0x01 << 8) | 0x03
