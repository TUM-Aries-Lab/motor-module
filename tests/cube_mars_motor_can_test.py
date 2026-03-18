"""Unit tests for the MIT-only CAN implementation (no hardware required)."""
# ruff: noqa: D101, D102

import struct
from unittest.mock import MagicMock, patch

import can
import numpy as np
import pytest

from motor_python.base_motor import MotorState
from motor_python.can_protocol import CANControlMode
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN
from motor_python.definitions import CAN_DEFAULTS
from motor_python.mit_mode_packer import AK60_6_MIT_LIMITS, pack_mit_frame


def _make_feedback_msg(
    *,
    position_degrees: float = 90.0,
    speed_erpm: int = 10000,
    current_amps: float = 2.0,
    temperature_celsius: int = 40,
    error_code: int = 0,
    motor_id: int = 0x03,
) -> MagicMock:
    """Build a mock 8-byte feedback frame using CubeMars status scaling."""
    pos_int = round(position_degrees / 0.1)
    speed_int = round(speed_erpm / 10)
    current_int = round(current_amps / 0.01)

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


@pytest.fixture
def mock_bus():
    """Patch python-can Bus with a controllable mock."""
    with patch("motor_python.cube_mars_motor_can.can.interface.Bus") as mock_cls:
        bus = MagicMock()
        bus.recv.return_value = None
        mock_cls.return_value = bus
        yield bus


@pytest.fixture
def motor(mock_bus):
    """CAN motor fixture backed by the mocked bus."""
    m = CubeMarsAK606v3CAN()
    yield m
    m.close()


class TestInit:
    def test_connects_with_mock_bus(self, motor, mock_bus):
        assert motor.connected is True
        assert motor.bus is mock_bus

    def test_connection_failure_is_graceful(self):
        with patch("motor_python.cube_mars_motor_can.can.interface.Bus") as mock_cls:
            mock_cls.side_effect = can.CanError("interface not found")
            m = CubeMarsAK606v3CAN()
            assert m.connected is False

    def test_build_extended_id_mit(self, motor):
        arb_id = motor._build_extended_id(CANControlMode.MIT_MODE)
        assert arb_id == (0x08 << 8) | 0x03


class TestMITEnableDisable:
    def test_enable_mit_mode_sends_all_ff(self, motor, mock_bus):
        motor.enable_mit_mode()
        sent = mock_bus.send.call_args_list[-1][0][0]
        assert sent.arbitration_id == motor.motor_can_id
        assert list(sent.data) == [0xFF] * 8

    def test_disable_mit_mode_sends_ff_fe(self, motor, mock_bus):
        motor.disable_mit_mode()
        sent = mock_bus.send.call_args_list[-1][0][0]
        assert sent.arbitration_id == motor.motor_can_id
        assert list(sent.data) == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]

    def test_enable_motor_is_alias_for_mit_enable(self, motor, mock_bus):
        motor.enable_motor()
        sent = mock_bus.send.call_args_list[-1][0][0]
        assert sent.arbitration_id == motor.motor_can_id
        assert list(sent.data) == [0xFF] * 8


class TestMITCommandPath:
    def test_set_mit_mode_uses_force_control_id_and_payload(self, motor, mock_bus):
        motor.set_mit_mode(
            pos_rad=1.0, vel_rad_s=2.0, kp=30.0, kd=1.5, torque_ff_nm=3.0
        )

        expected = pack_mit_frame(
            1.0,
            2.0,
            30.0,
            1.5,
            3.0,
            limits=AK60_6_MIT_LIMITS,
        )
        mit_arb_id = (CANControlMode.MIT_MODE << 8) | motor.motor_can_id

        mit_msgs = [
            call[0][0]
            for call in mock_bus.send.call_args_list
            if call[0][0].arbitration_id == mit_arb_id
        ]
        assert mit_msgs, "Expected at least one MIT command frame"
        assert bytes(mit_msgs[0].data) == expected

    def test_set_position_uses_default_mit_gains(self, motor):
        with patch.object(motor, "set_mit_mode") as mit:
            motor.set_position(90.0)

        mit.assert_called_once()
        kwargs = mit.call_args.kwargs
        assert kwargs["pos_rad"] == pytest.approx(np.pi / 2, rel=1e-4)
        assert kwargs["vel_rad_s"] == 0.0
        assert kwargs["kp"] == CAN_DEFAULTS.mit_position_kp
        assert kwargs["kd"] == CAN_DEFAULTS.mit_position_kd

    def test_set_velocity_routes_through_mit_velocity_mode(self, motor):
        with patch.object(motor, "set_mit_mode") as mit:
            motor.set_velocity(velocity_erpm=6000, allow_low_speed=True)

        mit.assert_called_once()
        kwargs = mit.call_args.kwargs
        expected_vel = 6000 * (2 * np.pi) / (60 * CAN_DEFAULTS.motor_pole_pairs)
        assert kwargs["pos_rad"] == 0.0
        assert kwargs["vel_rad_s"] == pytest.approx(expected_vel)
        assert kwargs["kp"] == 0.0
        assert kwargs["kd"] == CAN_DEFAULTS.mit_velocity_kd

    def test_set_current_maps_to_torque_feedforward(self, motor):
        with patch.object(motor, "set_mit_mode") as mit:
            motor.set_current(3.5)

        mit.assert_called_once_with(
            pos_rad=0.0,
            vel_rad_s=0.0,
            kp=0.0,
            kd=0.0,
            torque_ff_nm=3.5,
        )


class TestUnsupportedLegacyModes:
    def test_set_duty_cycle_raises(self, motor):
        with pytest.raises(NotImplementedError, match="duty-cycle"):
            motor.set_duty_cycle(0.2)

    def test_set_origin_raises(self, motor):
        with pytest.raises(NotImplementedError, match="set_origin"):
            motor.set_origin(permanent=False)

    def test_set_profile_mode_raises(self, motor):
        with pytest.raises(NotImplementedError, match="set_position_velocity_accel"):
            motor.set_position_velocity_accel(10.0, 3000, 1000)


class TestFeedbackAndCommunication:
    def test_receive_feedback_parses_all_fields(self, motor, mock_bus):
        mock_bus.recv.return_value = _make_feedback_msg(
            position_degrees=45.0,
            speed_erpm=5000,
            current_amps=3.5,
            temperature_celsius=55,
            error_code=2,
        )
        fb = motor._receive_feedback()
        assert fb is not None
        assert isinstance(fb, MotorState)
        assert fb.position_degrees == pytest.approx(45.0, abs=0.5)
        assert fb.speed_erpm == 5000
        assert fb.current_amps == pytest.approx(3.5, abs=0.1)
        assert fb.temperature_celsius == 55
        assert fb.error_code == 2

    def test_get_position_returns_none_without_feedback(self, motor, mock_bus):
        mock_bus.recv.return_value = None
        motor._last_feedback = None
        assert motor.get_position() is None

    def test_check_communication_true_when_feedback_arrives(self, motor, mock_bus):
        mock_bus.recv.return_value = _make_feedback_msg()
        assert motor.check_communication() is True
        assert motor.communicating is True

    def test_check_communication_false_when_disconnected(self):
        with patch("motor_python.cube_mars_motor_can.can.interface.Bus") as mock_cls:
            mock_cls.side_effect = can.CanError("no interface")
            m = CubeMarsAK606v3CAN()
            assert m.check_communication() is False
