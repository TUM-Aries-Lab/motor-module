"""Test the CubeMars motor module."""

from unittest.mock import MagicMock, patch

from motor_python.cube_mars_motor import CubeMarsAK606v3


def test_cubemarsmotor_initialization():
    """Test that CubeMarsAK606v3 class can be imported."""
    assert CubeMarsAK606v3 is not None


def test_connection_failure_graceful():
    """Test that connection failures are handled gracefully."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        # Simulate serial port not available
        mock_serial.side_effect = Exception("Port not available")

        # Should not raise exception
        motor = CubeMarsAK606v3(port="/dev/nonexistent")

        # Should be marked as not connected
        assert motor.connected is False
        assert motor.communicating is False


def test_connection_success():
    """Test successful connection."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_serial.return_value = mock_instance

        motor = CubeMarsAK606v3(port="/dev/test")

        # Should be marked as connected
        assert motor.connected is True
        assert motor.serial is not None


def test_send_message_when_not_connected():
    """Test that sending messages when not connected returns empty bytes."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_serial.side_effect = Exception("Port not available")
        motor = CubeMarsAK606v3(port="/dev/nonexistent")

        # Should return empty bytes without error
        result = motor._send_message(b"\xaa\x01\x45\xbb")
        assert result == b""


def test_check_communication_not_connected():
    """Test check_communication when motor is not connected."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_serial.side_effect = Exception("Port not available")
        motor = CubeMarsAK606v3(port="/dev/nonexistent")

        # Should return False
        result = motor.check_communication()
        assert result is False


def test_check_communication_no_response():
    """Test check_communication when motor doesn't respond."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 0  # No response
        mock_serial.return_value = mock_instance

        motor = CubeMarsAK606v3(port="/dev/test")

        # Should return False after timeout
        result = motor.check_communication()
        assert result is False
        assert motor.communicating is False


def test_check_communication_with_response():
    """Test check_communication when motor responds."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 10  # Has response
        mock_instance.read.return_value = b"\xaa\x05\x45\x00\x00\x00\x00\xbb"
        mock_serial.return_value = mock_instance

        motor = CubeMarsAK606v3(port="/dev/test")

        # Should return True
        result = motor.check_communication()
        assert result is True
        assert motor.communicating is True


"""Test the main program."""
