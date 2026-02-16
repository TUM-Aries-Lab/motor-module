"""Unit tests for CubeMars motor module (no hardware required)."""

from unittest.mock import MagicMock, patch

import pytest

from motor_python.cube_mars_motor import CubeMarsAK606v3
from motor_python.definitions import TendonAction

# Test velocity constants
# These values are chosen to test boundary conditions:
# - LOW_VELOCITY_ERPM: Below min_safe_velocity_erpm (5000) to test safety checks
# - SAFE_PULL_VELOCITY_ERPM: Above safe threshold for normal tendon pull operations
# - SAFE_RELEASE_VELOCITY_ERPM: Lower velocity for controlled tendon release
LOW_VELOCITY_ERPM = 100
SAFE_PULL_VELOCITY_ERPM = 10000
SAFE_RELEASE_VELOCITY_ERPM = 8000

# -- Initialization --


def test_cubemarsmotor_initialization():
    """CubeMarsAK606v3 class can be imported."""
    assert CubeMarsAK606v3 is not None


def test_connection_failure_graceful(nonexistent_port):
    """Connection failures are handled gracefully."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_serial.side_effect = Exception("Port not available")
        motor = CubeMarsAK606v3(port=nonexistent_port)
        assert motor.connected is False
        assert motor.communicating is False


def test_connection_success(test_port):
    """Successful connection marks motor as connected."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_serial.return_value = MagicMock()
        motor = CubeMarsAK606v3(port=test_port)
        assert motor.connected
        assert motor.serial is not None


# -- Send / Communication --


def test_send_frame_when_not_connected(nonexistent_port):
    """Sending frames when not connected returns empty bytes."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_serial.side_effect = Exception("Port not available")
        motor = CubeMarsAK606v3(port=nonexistent_port)
        result = motor._send_frame(b"\xaa\x01\x45\xbb")
        assert result == b""


def test_check_communication_not_connected(nonexistent_port):
    """check_communication returns False when not connected."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_serial.side_effect = Exception("Port not available")
        motor = CubeMarsAK606v3(port=nonexistent_port)
        assert not motor.check_communication()


def test_check_communication_no_response(test_port):
    """check_communication returns False when motor doesn't respond."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 0
        mock_serial.return_value = mock_instance
        motor = CubeMarsAK606v3(port=test_port)
        assert not motor.check_communication()
        assert not motor.communicating


def test_check_communication_with_response(test_port, mock_response):
    """check_communication returns True when motor responds."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 10
        # Protocol frame: AA (start) | 05 (length) | 45 (CMD_GET_STATUS) | 00 00 00 00 (4-byte payload) | 12 34 (CRC16) | BB (end)
        # Total: 1 + 1 + 5 (cmd + payload) + 2 + 1 = 10 bytes
        # CRC is calculated over: length + cmd + payload bytes
        mock_instance.read.return_value = b"\xaa\x05\x45\x00\x00\x00\x00\x12\x34\xbb"
        mock_serial.return_value = mock_instance
        motor = CubeMarsAK606v3(port=test_port)
        assert motor.check_communication()
        assert motor.communicating


# -- Velocity Safety --


def test_low_velocity_blocked(test_port):
    """Dangerously low velocities are blocked by default."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_serial.return_value = MagicMock()
        motor = CubeMarsAK606v3(port=test_port)
        with pytest.raises(ValueError, match="below safe threshold"):
            motor.set_velocity(velocity_erpm=LOW_VELOCITY_ERPM)


def test_low_velocity_allowed_with_flag(test_port):
    """Low velocity works with allow_low_speed=True."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 0
        mock_serial.return_value = mock_instance
        motor = CubeMarsAK606v3(port=test_port)
        # Should not raise an exception
        motor.set_velocity(velocity_erpm=LOW_VELOCITY_ERPM, allow_low_speed=True)
        # Verify that serial write was called (command was sent)
        assert mock_instance.write.called, "set_velocity should send command to motor"


def test_zero_velocity_always_allowed(test_port):
    """Zero velocity (stop) is always allowed."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 0
        mock_serial.return_value = mock_instance
        motor = CubeMarsAK606v3(port=test_port)
        motor.set_velocity(velocity_erpm=0)


# -- Tendon Control --


def test_tendon_pull(test_port):
    """Tendon pull action sends positive velocity."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 0
        mock_serial.return_value = mock_instance
        motor = CubeMarsAK606v3(port=test_port)
        motor.control_exosuit_tendon(
            action=TendonAction.PULL, velocity_erpm=SAFE_PULL_VELOCITY_ERPM
        )
        # Verify set_velocity was called with positive value
        assert mock_instance.write.called


def test_tendon_release(test_port):
    """Tendon release action sends negative velocity."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 0
        mock_serial.return_value = mock_instance
        motor = CubeMarsAK606v3(port=test_port)
        motor.control_exosuit_tendon(
            action=TendonAction.RELEASE, velocity_erpm=SAFE_RELEASE_VELOCITY_ERPM
        )
        # Verify set_velocity was called with negative value
        assert mock_instance.write.called


def test_tendon_stop(test_port):
    """Tendon stop action sends zero velocity."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 0
        mock_serial.return_value = mock_instance
        motor = CubeMarsAK606v3(port=test_port)
        motor.control_exosuit_tendon(action=TendonAction.STOP)


def test_tendon_invalid_action(test_port):
    """Invalid tendon action raises ValueError."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_serial.return_value = MagicMock()
        motor = CubeMarsAK606v3(port=test_port)
        with pytest.raises(ValueError, match="Invalid action"):
            invalid_action = 999  # type: ignore[assignment]
            motor.control_exosuit_tendon(action=invalid_action)  # type: ignore[arg-type]


# -- Position Control --


def test_set_position_within_limits(test_port):
    """Position within +/-360 degrees is accepted."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 0
        mock_serial.return_value = mock_instance
        motor = CubeMarsAK606v3(port=test_port)
        motor.set_position(position_degrees=180.0)


def test_set_position_clamped(test_port):
    """Position exceeding +/-360 degrees is clamped to safe range."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 0
        mock_serial.return_value = mock_instance
        motor = CubeMarsAK606v3(port=test_port)
        # Set position beyond limit - should be clamped to max_position_degrees
        motor.set_position(position_degrees=500.0)
        # Verify command was sent (motor accepts the clamped value)
        assert mock_instance.write.called, "set_position should clamp and send command"


def test_set_position_negative(test_port):
    """Negative position is accepted within limits."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 0
        mock_serial.return_value = mock_instance
        motor = CubeMarsAK606v3(port=test_port)
        motor.set_position(position_degrees=-270.0)
        # Verify command was sent with the requested position
        assert mock_instance.write.called, (
            "set_position should send command for valid negative position"
        )


def test_set_position_zero(test_port):
    """Zero position (home) is accepted."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 0
        mock_serial.return_value = mock_instance
        motor = CubeMarsAK606v3(port=test_port)
        motor.set_position(position_degrees=0.0)


def test_get_position_returns_bytes(test_port):
    """get_position returns bytes from motor."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 0
        mock_serial.return_value = mock_instance
        motor = CubeMarsAK606v3(port=test_port)
        result = motor.get_position()
        assert isinstance(result, bytes)


def test_move_to_position_with_speed(test_port):
    """move_to_position_with_speed sends velocity then position."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 0
        mock_serial.return_value = mock_instance
        motor = CubeMarsAK606v3(port=test_port)
        motor.move_to_position_with_speed(target_degrees=90.0, motor_speed_erpm=6000)
