"""Unit tests for the MIT-only CAN implementation (no hardware required)."""
# ruff: noqa: D101, D102

import struct
from contextlib import contextmanager
from unittest.mock import MagicMock, patch

import can
import numpy as np
import pytest

from motor_python import definitions
from motor_python.base_motor import MotorState
from motor_python.can_protocol import CANControlMode
from motor_python.cube_mars_motor_can import (
    CubeMarsAK606v1CAN,
    CubeMarsAK606v3CAN,
    CubeMarsAK806v2CAN,
    CubeMarsBaseCAN,
)
from motor_python.definitions import (
    AK60_6_V1_1_MOTOR_SPEC,
    AK60_6_V3_0_MOTOR_SPEC,
    AK80_6_MOTOR_SPEC,
    MotorModel,
    set_current_motor_model,
)
from motor_python.utils import float_to_uint


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
    msg.is_error_frame = False
    msg.is_remote_frame = False
    msg.is_rx = True
    msg.is_extended_id = True
    return msg


def _make_mit_feedback_msg(
    motor,
    *,
    position_rad: float,
    velocity_rad_s: float,
    current_amps: float,
    temperature_celsius: int = 40,
    error_code: int = 0,
) -> MagicMock:
    """Build a mock MIT feedback frame using the motor's own MIT limits."""
    limits = motor._motor_spec.mit_mode_limits
    p_int = float_to_uint(position_rad, limits.p_min, limits.p_max, 16)
    v_int = float_to_uint(velocity_rad_s, limits.v_min, limits.v_max, 12)
    i_int = float_to_uint(current_amps, limits.t_min, limits.t_max, 12)

    data = bytes(
        [
            motor.motor_can_id,
            (p_int >> 8) & 0xFF,
            p_int & 0xFF,
            (v_int >> 4) & 0xFF,
            ((v_int & 0x0F) << 4) | ((i_int >> 8) & 0x0F),
            i_int & 0xFF,
            temperature_celsius + 40,
            error_code,
        ]
    )

    msg = MagicMock()
    msg.arbitration_id = motor.motor_can_id
    msg.data = data
    msg.is_error_frame = False
    msg.is_remote_frame = False
    msg.is_rx = True
    msg.is_extended_id = False
    return msg


@contextmanager
def _connected(motor_cls, **kwargs):
    """Build a motor on the mocked bus and always close it again."""
    motor = motor_cls(**kwargs)
    try:
        yield motor
    finally:
        motor.close()


@pytest.fixture
def mock_bus():
    """Patch python-can Bus with a controllable mock."""
    with patch("motor_python.cube_mars_motor_can.can.interface.Bus") as mock_cls:
        bus = MagicMock()
        bus.recv.return_value = None
        mock_cls.return_value = bus
        yield bus


@pytest.fixture(autouse=True)
def mock_can_state():
    """Keep unit tests independent from host machine CAN controller state."""
    with patch(
        "motor_python.cube_mars_motor_can.get_can_state",
        return_value={"state": "ERROR-ACTIVE", "tx_err": 0, "rx_err": 0},
    ):
        yield


@pytest.fixture
def motor(mock_bus):
    """CAN motor fixture backed by the mocked bus."""
    m = CubeMarsAK606v3CAN()
    yield m
    m.close()


@pytest.fixture(
    params=[
        (CubeMarsAK806v2CAN, AK80_6_MOTOR_SPEC),
        (CubeMarsAK606v1CAN, AK60_6_V1_1_MOTOR_SPEC),
    ],
    ids=["ak80_6_v2", "ak60_6_v1_1"],
)
def base_mit_motor(request, mock_bus):
    """Motors that inherit the shared base-class MIT protocol."""
    motor_cls, motor_spec = request.param
    m = motor_cls(motor_spec=motor_spec)
    yield m
    m.close()


@pytest.fixture
def restore_current_motor_model():
    """Undo any global motor-model selection a test makes."""
    saved = definitions.CURRENT_MOTOR_MODEL
    yield
    set_current_motor_model(saved)


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
    def test_enable_mit_mode_handshakes_via_mit_id(self, motor, mock_bus):
        mock_bus.recv.return_value = _make_feedback_msg()
        motor.enable_mit_mode()
        first = mock_bus.send.call_args_list[0][0][0]
        assert (
            first.arbitration_id == (CANControlMode.MIT_MODE << 8) | motor.motor_can_id
        )
        assert motor._mit_enabled is True

    def test_disable_mit_mode_sends_ff_fd(self, motor, mock_bus):
        motor.disable_mit_mode()
        sent = mock_bus.send.call_args_list[-1][0][0]
        assert sent.arbitration_id == motor.motor_can_id
        assert list(sent.data) == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]

    def test_enable_motor_is_alias_for_mit_enable(self, motor, mock_bus):
        mock_bus.recv.return_value = _make_feedback_msg()
        motor.enable_motor()
        first = mock_bus.send.call_args_list[0][0][0]
        assert (
            first.arbitration_id == (CANControlMode.MIT_MODE << 8) | motor.motor_can_id
        )


class TestTransportRecovery:
    def test_send_raw_reconnects_once_on_tx_buffer_full(self, motor, mock_bus):
        mock_bus.send.side_effect = [can.CanError("Transmit buffer full"), None]

        with patch.object(
            motor, "_recover_bus_if_needed", return_value=True
        ) as recover:
            ok = motor._send_raw(
                arbitration_id=motor.motor_can_id,
                data=bytes([0x00] * 8),
                capture_response=False,
            )

        assert ok is True
        assert recover.call_count >= 1

    def test_enable_mit_mode_recovers_existing_transport_fault(self, motor, mock_bus):
        motor._transport_fault = "previous send failure"
        mock_bus.recv.return_value = _make_feedback_msg()

        with patch.object(
            motor, "_reconnect_transport", return_value=True
        ) as reconnect:
            motor.enable_mit_mode()

        reconnect.assert_called_once()
        assert motor._transport_fault is None

    def test_send_raw_recovers_on_no_such_device_error(self, motor, mock_bus):
        mock_bus.send.side_effect = [can.CanError("No such device or address"), None]

        with patch.object(
            motor, "_reconnect_transport", return_value=True
        ) as reconnect:
            ok = motor._send_raw(
                arbitration_id=motor.motor_can_id,
                data=bytes([0x00] * 8),
                capture_response=False,
            )

        assert ok is True
        reconnect.assert_called_once()

    def test_send_raw_recovers_on_attribute_error(self, motor, mock_bus):
        mock_bus.send.side_effect = [
            AttributeError("'NoneType' object has no attribute 'send'"),
            None,
        ]

        with patch.object(
            motor, "_reconnect_transport", return_value=True
        ) as reconnect:
            ok = motor._send_raw(
                arbitration_id=motor.motor_can_id,
                data=bytes([0x00] * 8),
                capture_response=False,
            )

        assert ok is True
        reconnect.assert_called_once()

    def test_recover_bus_allows_warning_state_when_tx_is_viable(self, motor):
        with (
            patch.object(
                motor,
                "_read_can_state",
                return_value={"state": "ERROR-WARNING", "tx_err": 0, "rx_err": 102},
            ),
            patch.object(motor, "_reconnect_transport") as reconnect,
        ):
            ok = motor._recover_bus_if_needed(reason="transmit")

        assert ok is True
        reconnect.assert_not_called()


class TestMITCommandPath:
    def test_set_mit_mode_uses_force_control_id_and_payload(self, motor, mock_bus):
        mock_bus.recv.return_value = _make_feedback_msg()
        motor.set_mit_mode(
            pos_rad=1.0, vel_rad_s=2.0, kp=30.0, kd=1.5, torque_ff_nm=3.0
        )

        expected = motor.pack_mit_frame(1.0, 2.0, 30.0, 1.5, 3.0)
        mit_arb_id = (CANControlMode.MIT_MODE << 8) | motor.motor_can_id

        mit_msgs = [
            call[0][0]
            for call in mock_bus.send.call_args_list
            if call[0][0].arbitration_id == mit_arb_id
        ]
        assert mit_msgs, "Expected at least one MIT command frame"
        assert any(bytes(msg.data) == expected for msg in mit_msgs)

    def test_set_position_uses_default_mit_gains(self, motor):
        with patch.object(motor, "set_mit_mode") as mit:
            motor.set_position(90.0)

        mit.assert_called_once()
        kwargs = mit.call_args.kwargs
        assert kwargs["pos_rad"] == pytest.approx(np.pi / 2, rel=1e-4)
        assert kwargs["vel_rad_s"] == 0.0
        assert kwargs["kp"] == motor._motor_spec.mit_position_kp
        assert kwargs["kd"] == motor._motor_spec.mit_position_kd

    def test_set_velocity_routes_through_mit_velocity_mode(self, motor):
        with patch.object(motor, "set_mit_mode") as mit:
            motor.set_velocity(velocity_erpm=6000)

        mit.assert_called_once()
        kwargs = mit.call_args.kwargs
        expected_vel = (
            6000
            * (2 * np.pi)
            / (60 * motor._motor_spec.pole_pairs * motor._motor_spec.gear_ratio)
        )
        assert kwargs["pos_rad"] == 0.0
        assert kwargs["vel_rad_s"] == pytest.approx(expected_vel)
        assert kwargs["kp"] == 0.0
        assert kwargs["kd"] == motor._motor_spec.mit_velocity_kd

    def test_set_velocity_uses_constructor_velocity_kd_override(self, mock_bus):
        motor = CubeMarsAK606v3CAN(mit_velocity_kd=0.5)
        try:
            with patch.object(motor, "set_mit_mode") as mit:
                motor.set_velocity(velocity_erpm=6000)
            kwargs = mit.call_args.kwargs
            assert kwargs["kd"] == pytest.approx(0.5)
        finally:
            motor.close()

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

    def test_parse_feedback_ignores_error_frames(self, motor):
        msg = _make_feedback_msg()
        msg.is_error_frame = True
        assert motor._parse_feedback_msg(msg) is None

    def test_parse_feedback_ignores_non_rx_loopback(self, motor):
        msg = _make_feedback_msg()
        msg.is_rx = False
        assert motor._parse_feedback_msg(msg) is None

    def test_parse_feedback_keeps_full_uint8_error_code(self, motor):
        msg = _make_feedback_msg(error_code=9)
        feedback = motor._parse_feedback_msg(msg)
        assert feedback is not None
        assert feedback.error_code == 9


class TestPackMITFrame:
    def test_pack_mit_frame_uses_manual_byte_order(self, motor):
        # Pick deterministic values and compare against manual byte mapping.
        p_int = float_to_uint(
            1.0,
            motor._motor_spec.mit_mode_limits.p_min,
            motor._motor_spec.mit_mode_limits.p_max,
            16,
        )
        v_int = float_to_uint(
            2.0,
            motor._motor_spec.mit_mode_limits.v_min,
            motor._motor_spec.mit_mode_limits.v_max,
            12,
        )
        kp_int = float_to_uint(
            30.0,
            motor._motor_spec.mit_mode_limits.kp_min,
            motor._motor_spec.mit_mode_limits.kp_max,
            12,
        )
        kd_int = float_to_uint(
            1.5,
            motor._motor_spec.mit_mode_limits.kd_min,
            motor._motor_spec.mit_mode_limits.kd_max,
            12,
        )
        t_int = float_to_uint(
            3.0,
            motor._motor_spec.mit_mode_limits.t_min,
            motor._motor_spec.mit_mode_limits.t_max,
            12,
        )

        expected = bytes(
            [
                kp_int >> 4,
                ((kp_int & 0xF) << 4) | (kd_int >> 8),
                kd_int & 0xFF,
                p_int >> 8,
                p_int & 0xFF,
                v_int >> 4,
                ((v_int & 0xF) << 4) | (t_int >> 8),
                t_int & 0xFF,
            ]
        )

        payload = motor.pack_mit_frame(
            1.0, 2.0, 30.0, 1.5, 3.0, limits=motor._motor_spec.mit_mode_limits
        )
        assert payload == expected

    def test_pack_mit_frame_ak60_6_limits_are_enforced(self, motor):
        payload = motor.pack_mit_frame(
            999.0, 999.0, 999.0, 999.0, 999.0, limits=motor._motor_spec.mit_mode_limits
        )

        # Decode only boundary-sensitive fields to confirm top saturation.
        kp_high = payload[0]
        kp_low = payload[1] >> 4
        kp_raw = (kp_high << 4) | kp_low

        kd_high = payload[1] & 0xF
        kd_low = payload[2]
        kd_raw = (kd_high << 8) | kd_low

        pos_raw = (payload[3] << 8) | payload[4]

        assert kp_raw == (1 << 12) - 1
        assert kd_raw == (1 << 12) - 1
        assert pos_raw == (1 << 16) - 1


class TestMITProtocolByModel:
    def test_shared_packer_defaults_to_spec_limits(self, base_mit_motor):
        limits = base_mit_motor._motor_spec.mit_mode_limits
        p_int = float_to_uint(1.0, limits.p_min, limits.p_max, 16)
        v_int = float_to_uint(2.0, limits.v_min, limits.v_max, 12)
        kp_int = float_to_uint(30.0, limits.kp_min, limits.kp_max, 12)
        kd_int = float_to_uint(1.5, limits.kd_min, limits.kd_max, 12)
        t_int = float_to_uint(3.0, limits.t_min, limits.t_max, 12)

        expected = bytes(
            [
                (p_int >> 8) & 0xFF,
                p_int & 0xFF,
                (v_int >> 4) & 0xFF,
                ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F),
                kp_int & 0xFF,
                (kd_int >> 4) & 0xFF,
                ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F),
                t_int & 0xFF,
            ]
        )

        # Omitting ``limits`` must resolve to the motor's own spec limits.
        assert base_mit_motor.pack_mit_frame(1.0, 2.0, 30.0, 1.5, 3.0) == expected
        assert (
            base_mit_motor.pack_mit_frame(1.0, 2.0, 30.0, 1.5, 3.0, limits=limits)
            == expected
        )

    def test_shared_parser_round_trips_physical_values(self, base_mit_motor):
        limits = base_mit_motor._motor_spec.mit_mode_limits
        msg = _make_mit_feedback_msg(
            base_mit_motor,
            position_rad=0.5,
            velocity_rad_s=-10.0,
            current_amps=2.0,
        )

        state = base_mit_motor._parse_feedback_msg(msg)

        assert state is not None
        assert state.position_degrees == pytest.approx(np.degrees(0.5), abs=0.05)
        assert state.speed_erpm == pytest.approx(
            base_mit_motor._rad_s_to_erpm(-10.0), rel=0.01
        )
        assert state.current_amps == pytest.approx(
            2.0, abs=(limits.t_max - limits.t_min) / ((1 << 12) - 1)
        )
        assert state.temperature_celsius == 40
        assert state.error_code == 0

    def test_standard_frames_for_ak80_and_ak60_v1_1(self, base_mit_motor, mock_bus):
        mock_bus.send.reset_mock()

        base_mit_motor._send_mit_payload(bytes(8), capture_response=False)

        sent = mock_bus.send.call_args_list[-1][0][0]
        assert sent.is_extended_id is False
        assert sent.arbitration_id == base_mit_motor.motor_can_id

    def test_extended_frames_for_ak60_v3(self, motor, mock_bus):
        mock_bus.send.reset_mock()

        motor._send_mit_payload(bytes(8), capture_response=False)

        sent = mock_bus.send.call_args_list[-1][0][0]
        assert sent.is_extended_id is True
        assert (
            sent.arbitration_id == (CANControlMode.MIT_MODE << 8) | motor.motor_can_id
        )

    def test_connect_resets_mit_state_for_ak80_and_ak60_v1_1(
        self, base_mit_motor, mock_bus
    ):
        reset_frames = [
            call[0][0]
            for call in mock_bus.send.call_args_list
            if bytes(call[0][0].data) == base_mit_motor._CAN_HELPER_DISABLE
        ]

        assert reset_frames, "Expected an MIT reset frame while connecting"

    def test_connect_does_not_reset_mit_state_for_ak60_v3(self, motor, mock_bus):
        reset_frames = [
            call[0][0]
            for call in mock_bus.send.call_args_list
            if bytes(call[0][0].data) == motor._CAN_HELPER_DISABLE
        ]

        assert not reset_frames


class TestMotorSpecResolution:
    @pytest.mark.parametrize(
        ("motor_cls", "expected_spec"),
        [
            (CubeMarsAK606v3CAN, AK60_6_V3_0_MOTOR_SPEC),
            (CubeMarsAK806v2CAN, AK80_6_MOTOR_SPEC),
            (CubeMarsAK606v1CAN, AK60_6_V1_1_MOTOR_SPEC),
        ],
        ids=["ak60_6_v3", "ak80_6_v2", "ak60_6_v1_1"],
    )
    def test_bare_construction_uses_the_class_own_spec(
        self, mock_bus, motor_cls, expected_spec
    ):
        with _connected(motor_cls) as motor:
            assert motor._motor_spec is expected_spec
            assert motor.motor_model == expected_spec.model_name

    def test_explicit_spec_still_wins(self, mock_bus):
        with _connected(CubeMarsAK606v1CAN, motor_spec=AK80_6_MOTOR_SPEC) as motor:
            assert motor._motor_spec is AK80_6_MOTOR_SPEC

    def test_subclass_ignores_the_global_selection(
        self, mock_bus, restore_current_motor_model
    ):
        set_current_motor_model(MotorModel.AK80_6)

        with _connected(CubeMarsAK606v1CAN) as motor:
            assert motor._motor_spec is AK60_6_V1_1_MOTOR_SPEC

    def test_base_class_follows_the_global_selection(
        self, mock_bus, restore_current_motor_model
    ):
        set_current_motor_model(MotorModel.AK80_6)

        # Resolved at call time, so the selection made above is picked up.
        with _connected(CubeMarsBaseCAN) as motor:
            assert motor._motor_spec is AK80_6_MOTOR_SPEC

    def test_explicit_none_is_accepted(self, mock_bus):
        # Used to raise AttributeError before the spec was resolved up front.
        with _connected(CubeMarsBaseCAN, motor_spec=None) as motor:
            assert motor._motor_spec is definitions.CURRENT_MOTOR_SPEC
