"""Test the CubeMars motor module."""

from unittest.mock import MagicMock, patch

from motor_python.cube_mars_motor import CubeMarsAK606v3


def test_cubemarsmotor_initialization():
    """Test that CubeMarsAK606v3 class can be imported."""
    assert CubeMarsAK606v3 is not None


def test_connection_failure_graceful(nonexistent_port):
    """Test that connection failures are handled gracefully."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        # Simulate serial port not available
        mock_serial.side_effect = Exception("Port not available")

        # Should not raise exception
        motor = CubeMarsAK606v3(port=nonexistent_port)

        # Should be marked as not connected
        assert motor.connected is False
        assert motor.communicating is False


def test_connection_success(test_port):
    """Test successful connection."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_serial.return_value = mock_instance

        motor = CubeMarsAK606v3(port=test_port)

        # Should be marked as connected
        assert motor.connected
        assert motor.serial is not None


def test_send_frame_when_not_connected(nonexistent_port):
    """Test that sending frames when not connected returns empty bytes."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_serial.side_effect = Exception("Port not available")
        motor = CubeMarsAK606v3(port=nonexistent_port)

        # Should return empty bytes without error
        result = motor._send_frame(b"\xaa\x01\x45\xbb")
        assert result == b""


def test_check_communication_not_connected(nonexistent_port):
    """Test check_communication when motor is not connected."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_serial.side_effect = Exception("Port not available")
        motor = CubeMarsAK606v3(port=nonexistent_port)

        # Should return False
        result = motor.check_communication()
        assert not result


def test_check_communication_no_response(test_port):
    """Test check_communication when motor doesn't respond."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 0  # No response
        mock_serial.return_value = mock_instance

        motor = CubeMarsAK606v3(port=test_port)

        # Should return False after timeout
        result = motor.check_communication()
        assert not result
        assert not motor.communicating


def test_check_communication_with_response(test_port, mock_response):
    """Test check_communication when motor responds."""
    with patch("motor_python.cube_mars_motor.serial.Serial") as mock_serial:
        mock_instance = MagicMock()
        mock_instance.in_waiting = 10  # Has response
        mock_instance.read.return_value = mock_response
        mock_serial.return_value = mock_instance

        motor = CubeMarsAK606v3(port=test_port)

        # Should return True
        result = motor.check_communication()
        assert result
        assert motor.communicating


"""Test the main program."""
