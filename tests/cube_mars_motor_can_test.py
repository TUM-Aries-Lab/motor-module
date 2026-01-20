"""Tests for CAN motor control (cube_mars_motor_can.py) using MCP2515."""

import struct
from unittest.mock import MagicMock, patch

import pytest

from motor_python.definitions import CAN_SCALE_FACTORS


@pytest.fixture
def mock_spidev():
    """Create a mock spidev module and SpiDev class."""
    with patch.dict("sys.modules", {"spidev": MagicMock()}):
        import sys

        mock_spidev_module = sys.modules["spidev"]
        mock_spi = MagicMock()
        mock_spi.xfer2.return_value = [0, 0, 0]
        mock_spidev_module.SpiDev.return_value = mock_spi
        yield mock_spi


@pytest.fixture
def mock_mcp2515():
    """Create a mock MCP2515 controller."""
    with patch("motor_python.cube_mars_motor_can.MCP2515") as MockMCP:
        mock_instance = MagicMock()
        mock_instance.connected = True
        mock_instance.init.return_value = True
        mock_instance.send.return_value = True
        mock_instance.receive.return_value = None
        MockMCP.return_value = mock_instance
        yield mock_instance


def test_can_motor_initialization_success(mock_mcp2515):
    """Test successful CAN motor initialization."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)

    assert motor.connected is True
    assert motor.motor_id == 0x68
    assert motor.communicating is False

    motor.close()


def test_can_motor_initialization_failure():
    """Test CAN motor initialization failure."""
    with patch("motor_python.cube_mars_motor_can.MCP2515") as MockMCP:
        mock_instance = MagicMock()
        mock_instance.connected = False
        MockMCP.return_value = mock_instance

        from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

        motor = CubeMarsAK606CAN(motor_id=0x68)

        assert motor.connected is False
        motor.close()


def test_parse_motor_feedback(mock_mcp2515):
    """Test parsing CAN motor feedback message."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)

    # Create test data: position=100°, speed=1000 RPM, current=2.5A, temp=25°C, error=0
    pos_int = int(100.0 / CAN_SCALE_FACTORS.position)  # 1000
    spd_int = int(1000.0 / CAN_SCALE_FACTORS.speed)  # 100
    cur_int = int(2.5 / CAN_SCALE_FACTORS.current)  # 250
    temp = 25
    error = 0

    data = struct.pack(">hhhbb", pos_int, spd_int, cur_int, temp, error)

    status = motor._parse_motor_feedback(data)

    assert status is not None
    assert abs(status.position - 100.0) < 0.1
    assert abs(status.speed - 1000.0) < 1.0
    assert abs(status.current - 2.5) < 0.01
    assert status.temperature == 25
    assert status.error_code == 0

    motor.close()


def test_parse_motor_feedback_invalid_length(mock_mcp2515):
    """Test parsing with invalid data length."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)

    # Data too short
    data = b"\x00\x01\x02"
    status = motor._parse_motor_feedback(data)

    assert status is None

    motor.close()


def test_parse_motor_feedback_with_error(mock_mcp2515):
    """Test parsing feedback with error code."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)

    # Create data with error code 2 (over-current)
    data = struct.pack(">hhhbb", 0, 0, 0, 30, 2)

    status = motor._parse_motor_feedback(data)

    assert status is not None
    assert status.error_code == 2
    assert status.temperature == 30

    motor.close()


def test_send_can_message(mock_mcp2515):
    """Test sending CAN message."""
    from motor_python.cube_mars_motor_can import CANMotorCommand, CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)

    # Send a test message
    result = motor._send_can_message(
        CANMotorCommand.CMD_CURRENT_LOOP, b"\x00\x01\x02\x03"
    )

    assert result is True
    assert mock_mcp2515.send.called

    motor.close()


def test_send_can_message_not_connected():
    """Test sending message when not connected."""
    with patch("motor_python.cube_mars_motor_can.MCP2515") as MockMCP:
        mock_instance = MagicMock()
        mock_instance.connected = False
        MockMCP.return_value = mock_instance

        from motor_python.cube_mars_motor_can import CANMotorCommand, CubeMarsAK606CAN

        motor = CubeMarsAK606CAN(motor_id=0x68)
        result = motor._send_can_message(
            CANMotorCommand.CMD_CURRENT_LOOP, b"\x00\x01\x02\x03"
        )

        assert result is False
        motor.close()


def test_receive_feedback_timeout(mock_mcp2515):
    """Test receiving feedback with timeout."""
    mock_mcp2515.receive.return_value = None

    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)
    status = motor.receive_feedback(timeout=0.1)

    assert status is None
    assert motor._consecutive_no_response == 1

    motor.close()


def test_receive_feedback_success(mock_mcp2515):
    """Test successful feedback reception."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN
    from motor_python.mcp2515 import CANMessage

    # Create mock CAN message
    mock_msg = CANMessage(
        can_id=0x68,
        data=struct.pack(">hhhbb", 1000, 100, 250, 25, 0),
        dlc=8,
        is_extended=False,
    )
    mock_mcp2515.receive.return_value = mock_msg

    motor = CubeMarsAK606CAN(motor_id=0x68)
    status = motor.receive_feedback(timeout=0.1)

    assert status is not None
    assert motor.communicating is True
    assert motor._consecutive_no_response == 0
    assert motor.last_status == status

    motor.close()


def test_set_duty_cycle(mock_mcp2515):
    """Test setting duty cycle."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)
    motor.set_duty_cycle(0.5)

    assert mock_mcp2515.send.called

    motor.close()


def test_set_duty_cycle_clamping(mock_mcp2515):
    """Test duty cycle clamping to valid range."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)

    # Test upper limit
    motor.set_duty_cycle(1.5)  # Should clamp to 0.95
    assert mock_mcp2515.send.called

    # Test lower limit
    motor.set_duty_cycle(-1.5)  # Should clamp to -0.95
    assert mock_mcp2515.send.called

    motor.close()


def test_set_current(mock_mcp2515):
    """Test setting current."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)
    motor.set_current(5.0)

    assert mock_mcp2515.send.called

    motor.close()


def test_set_velocity(mock_mcp2515):
    """Test setting velocity."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)
    motor.set_velocity(1000.0)

    assert mock_mcp2515.send.called

    motor.close()


def test_set_position(mock_mcp2515):
    """Test setting position."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)
    motor.set_position(90.0)

    assert mock_mcp2515.send.called

    motor.close()


def test_set_position_clamping(mock_mcp2515):
    """Test position clamping to valid CAN range."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)

    # Test exceeding max (3200 degrees)
    motor.set_position(5000.0)
    assert mock_mcp2515.send.called

    # Test exceeding min (-3200 degrees)
    motor.set_position(-5000.0)
    assert mock_mcp2515.send.called

    motor.close()


def test_set_position_velocity(mock_mcp2515):
    """Test position-velocity control."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)
    motor.set_position_velocity(180.0, 5000.0, 10000.0)

    assert mock_mcp2515.send.called

    motor.close()


def test_set_mit_position(mock_mcp2515):
    """Test MIT position control."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)
    motor.set_mit_position(1.57, kp=2.0, kd=0.5)

    assert mock_mcp2515.send.called

    motor.close()


def test_set_mit_velocity(mock_mcp2515):
    """Test MIT velocity control."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)
    motor.set_mit_velocity(6.28, kd=0.5)

    assert mock_mcp2515.send.called

    motor.close()


def test_set_mit_torque(mock_mcp2515):
    """Test MIT torque control."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)
    motor.set_mit_torque(2.0)

    assert mock_mcp2515.send.called

    motor.close()


def test_check_communication_success(mock_mcp2515):
    """Test communication check with successful response."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN
    from motor_python.mcp2515 import CANMessage

    # Create mock feedback message
    mock_msg = CANMessage(
        can_id=0x68,
        data=struct.pack(">hhhbb", 0, 0, 0, 25, 0),
        dlc=8,
        is_extended=False,
    )
    mock_mcp2515.receive.return_value = mock_msg

    motor = CubeMarsAK606CAN(motor_id=0x68)
    result = motor.check_communication()

    assert result is True

    motor.close()


def test_check_communication_failure(mock_mcp2515):
    """Test communication check with no response."""
    mock_mcp2515.receive.return_value = None

    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)
    result = motor.check_communication()

    assert result is False

    motor.close()


def test_context_manager(mock_mcp2515):
    """Test context manager usage."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    with CubeMarsAK606CAN(motor_id=0x68) as motor:
        assert motor.connected is True

    # After exiting context, close should have been called
    assert mock_mcp2515.close.called


def test_close(mock_mcp2515):
    """Test closing connection."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)
    assert motor.connected is True

    motor.close()

    assert motor.mcp2515 is None
    assert motor.connected is False
    assert motor.communicating is False


def test_motor_can_status_dataclass():
    """Test MotorCANStatus dataclass."""
    from motor_python.cube_mars_motor_can import MotorCANStatus

    status = MotorCANStatus(
        position=90.0, speed=1000.0, current=2.5, temperature=30, error_code=0
    )

    assert status.position == 90.0
    assert status.speed == 1000.0
    assert status.current == 2.5
    assert status.temperature == 30
    assert status.error_code == 0


def test_get_status(mock_mcp2515):
    """Test getting motor status."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN
    from motor_python.mcp2515 import CANMessage

    # Create mock feedback message
    mock_msg = CANMessage(
        can_id=0x68,
        data=struct.pack(">hhhbb", 1000, 100, 250, 25, 0),
        dlc=8,
        is_extended=False,
    )
    mock_mcp2515.receive.return_value = mock_msg

    motor = CubeMarsAK606CAN(motor_id=0x68)
    status = motor.get_status()

    assert status is not None
    assert abs(status.position - 100.0) < 0.1  # 1000 * 0.1 = 100

    motor.close()


def test_get_status_returns_last_status(mock_mcp2515):
    """Test get_status returns last known status when no new feedback."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN, MotorCANStatus

    motor = CubeMarsAK606CAN(motor_id=0x68)

    # Set last_status manually
    motor.last_status = MotorCANStatus(
        position=45.0, speed=500.0, current=1.0, temperature=20, error_code=0
    )

    # No new feedback
    mock_mcp2515.receive.return_value = None

    status = motor.get_status()

    assert status is not None
    assert status.position == 45.0

    motor.close()


def test_get_position(mock_mcp2515):
    """Test getting motor position."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN
    from motor_python.mcp2515 import CANMessage

    # Create mock feedback message with position = 90°
    mock_msg = CANMessage(
        can_id=0x68,
        data=struct.pack(">hhhbb", 900, 0, 0, 25, 0),  # 900 * 0.1 = 90°
        dlc=8,
        is_extended=False,
    )
    mock_mcp2515.receive.return_value = mock_msg

    motor = CubeMarsAK606CAN(motor_id=0x68)
    position = motor.get_position()

    assert position is not None
    assert abs(position - 90.0) < 0.1

    motor.close()


def test_get_position_returns_none_when_no_status(mock_mcp2515):
    """Test get_position returns None when no status available."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    mock_mcp2515.receive.return_value = None

    motor = CubeMarsAK606CAN(motor_id=0x68)
    motor.last_status = None
    position = motor.get_position()

    assert position is None

    motor.close()


def test_stop(mock_mcp2515):
    """Test stopping the motor."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)
    motor.stop()

    # Should have sent 3 commands: duty=0, current=0, velocity=0
    assert mock_mcp2515.send.call_count >= 3

    motor.close()


def test_estimate_movement_time(mock_mcp2515):
    """Test movement time estimation."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)

    # At 600 RPM = 10 rev/sec = 3600°/sec
    # 360° should take 0.1 seconds
    time_est = motor._estimate_movement_time(360.0, 600.0)
    assert abs(time_est - 0.1) < 0.01

    # Zero speed should return 0
    time_est = motor._estimate_movement_time(360.0, 0.0)
    assert time_est == 0.0

    motor.close()


def test_move_to_position_with_speed(mock_mcp2515):
    """Test move to position with speed control."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

    motor = CubeMarsAK606CAN(motor_id=0x68)

    # This will set velocity, wait, then set position
    # Using very high speed so estimated time is near 0
    motor.move_to_position_with_speed(90.0, 36000.0)  # Very fast

    # Should have called send multiple times
    assert mock_mcp2515.send.call_count >= 2

    motor.close()
