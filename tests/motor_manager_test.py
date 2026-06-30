"""Unit tests for Motor manager."""

from unittest.mock import Mock

import pytest

from motor_python.motor_manager import MotorManager


def create_fake_motor(motor_id: int):
    motor = Mock()
    motor.motor_can_id = motor_id
    motor.stop = Mock()
    motor.close = Mock()
    motor.enable_motor = Mock()
    motor.disable_motor = Mock()
    motor.check_communication = Mock(return_value=True)
    return motor


@pytest.fixture(autouse=True)
def patch_motor_class(monkeypatch):
    import motor_python.motor_manager as mm

    motors = {}

    def fake_init(self, motor_can_id, interface="can0"):
        fake = create_fake_motor(motor_can_id)
        motors[motor_can_id] = fake
        self._fake = fake

        # forward calls
        self.motor_can_id = motor_can_id
        self.stop = fake.stop
        self.close = fake.close
        self.enable_motor = fake.enable_motor
        self.disable_motor = fake.disable_motor
        self.check_communication = fake.check_communication

    monkeypatch.setattr(mm.CubeMarsAK606v3CAN, "__init__", fake_init)

    return motors


def test_motor_manager_basic_construction():
    m = MotorManager([1, 2, 3])

    assert len(m) == 3
    assert set(m._motors.keys()) == {1, 2, 3}

    # iteration
    assert len(list(m)) == 3


def test_label_access_and_validation():
    m = MotorManager(
        motor_ids=[1, 2],
        labels={"left": 1, "right": 2},
    )

    assert m["left"].motor_can_id == 1
    assert m["right"].motor_can_id == 2

    with pytest.raises(KeyError):
        _ = m["unknown"]

    with pytest.raises(KeyError):
        _ = m[999]


def test_label_validation_error():
    with pytest.raises(ValueError):
        MotorManager(
            motor_ids=[1, 2],
            labels={"bad": 99},  # invalid ID
        )


def test_duplicate_motor_ids():
    with pytest.raises(ValueError):
        MotorManager([1, 1, 2])


def test_contains_and_iteration():
    m = MotorManager([1, 2])

    assert 1 in m
    assert "missing" not in m
    assert 2 in m
    assert 99 not in m

    motors = list(m)
    assert all(hasattr(x, "motor_can_id") for x in motors)


def test_enable_disable_all():
    m = MotorManager([1, 2])

    m.enable_all()
    m.disable_all()

    for motor in m:
        motor.enable_motor.assert_called_once()
        motor.disable_motor.assert_called_once()


def test_stop_all_collects_errors(monkeypatch):
    m = MotorManager([1, 2])

    first_motor = next(iter(m))
    first_motor.stop.side_effect = RuntimeError("fail")

    with pytest.raises(ExceptionGroup):
        m.stop_all()


def test_close_all_and_idempotent(monkeypatch):
    m = MotorManager([1, 2])

    m.close()
    m.close()  # should do nothing second time

    for motor in m:
        motor.close.assert_called_once()


def test_context_manager_cleanup():
    m = MotorManager([1, 2])

    with m as mgr:
        assert mgr is m

    for motor in m:
        motor.stop.assert_called_once()
        motor.close.assert_called_once()


def test_check_all():
    m = MotorManager([1, 2])

    result = m.check_all()

    assert result == {1: True, 2: True}
