"""Unit tests for PID controller helpers."""

from unittest.mock import patch

import numpy as np
import pytest

from motor_python.definitions import LowPassFilterConfig, PIDConfig, SolverType
from motor_python.pid_controller import PIDController
from motor_python.second_order_low_pass_filter import SecondOrderLowPassFilter


def test_compute_output_returns_zero():
    """Covers if self._prev_timestamp is None."""
    pid = PIDController(
        PIDConfig(proportional_gain=2, integral_gain=1, derivative_gain=0.5),
        LowPassFilterConfig(),
    )

    output = pid.compute_output(
        timestamp=1,
        motor_reference=1.0,
        motor_position=0.0,
    )

    assert output == 0.0


def test_proportional_term_only():
    """Disable I and D."""
    pid = PIDController(
        PIDConfig(
            proportional_gain=2.0,
            integral_gain=0.0,
            derivative_gain=0.0,
            output_limits=(-100.0, 100.0),
        ),
        LowPassFilterConfig(),
    )

    pid.compute_output(0.0, 0.0, 0.0)

    output = pid.compute_output(
        timestamp=1.0,
        motor_reference=1.0,
        motor_position=0.5,
    )

    expected = 2.0 * 0.5

    assert output == pytest.approx(expected)


def test_integral_term_accumulates():
    """Not comparing exact values because the controller clips the error to ±60°."""
    pid = PIDController(
        PIDConfig(
            proportional_gain=0.0,
            integral_gain=1.0,
            derivative_gain=0.0,
            output_limits=(-100.0, 100.0),
        ),
        LowPassFilterConfig(),
    )

    pid.compute_output(0.0, 1.0, 0.0)

    out1 = pid.compute_output(1.0, 1.0, 0.0)
    out2 = pid.compute_output(2.0, 1.0, 0.0)

    assert out2 > out1


def test_derivative_term(mock_response):
    """Mocking the Lock-pass filter output."""
    pid = PIDController(
        PIDConfig(
            proportional_gain=0.0,
            integral_gain=0.0,
            derivative_gain=2.0,
            output_limits=(-100.0, 100.0),
        ),
        LowPassFilterConfig(),
    )
    # Mock the low pass filter output
    with patch.object(
        pid.low_pass_filter,
        "step",
        return_value=(3.5, None),
    ) as step:
        pid.compute_output(0.0, 0.0, 0.0)

        output = pid.compute_output(1.0, 0.0, 0.0)

        step.assert_called_once_with(x=0.0, time_difference=1.0)

    assert output == pytest.approx(-7.0)  # Output = 0 - kd * filtered_value


def test_error_is_clamped_to_sixty_degrees():
    """Covers the clipping of the errors."""
    pid = PIDController(
        PIDConfig(
            proportional_gain=1.0,
            integral_gain=0.0,
            derivative_gain=0.0,
            output_limits=(-100.0, 100.0),
        ),
        LowPassFilterConfig(),
    )

    pid.compute_output(0.0, 0.0, 0.0)

    output = pid.compute_output(
        1.0,
        motor_reference=np.radians(500),
        motor_position=0.0,
    )

    assert output == pytest.approx(np.radians(60))


def test_output_is_clamped():
    pid = PIDController(
        PIDConfig(
            proportional_gain=1000.0,
            integral_gain=0.0,
            derivative_gain=0.0,
            output_limits=(-5.0, 5.0),
        ),
        LowPassFilterConfig(),
    )

    pid.compute_output(0.0, 0.0, 0.0)

    output = pid.compute_output(
        1.0,
        motor_reference=1.0,
        motor_position=0.0,
    )

    assert output == 5.0


def test_integral_term_is_clamped():
    """Output should stay saturated at 2.0."""
    pid = PIDController(
        PIDConfig(
            proportional_gain=0.0,
            integral_gain=100.0,
            derivative_gain=0.0,
            output_limits=(-2.0, 2.0),
        ),
        LowPassFilterConfig(),
    )

    pid.compute_output(0.0, 1.0, 0.0)

    for t in range(1, 20):
        pid.compute_output(float(t), 1.0, 0.0)

    assert pid._integral > 0


def test_reset_clears_internal_state():
    pid = PIDController(
        PIDConfig(
            proportional_gain=1.0,
            integral_gain=1.0,
            derivative_gain=1.0,
        ),
        LowPassFilterConfig(),
    )

    pid.compute_output(0.0, 1.0, 0.0)
    pid.compute_output(1.0, 1.0, 0.0)

    pid.reset()

    assert pid._integral == 0.0
    assert pid._prev_velocity == 0.0


######################################################
"""Unit tests for second_order_low_pass_filter.py."""
######################################################


def create_filter():
    return SecondOrderLowPassFilter(
        LowPassFilterConfig(20, 1, 0, SolverType.RUNGE_KUTTA)
    )


def test_initial_condition():
    """Filter should always start from its configured initial condition."""
    cfg = LowPassFilterConfig(
        cut_off_frequency_rad_per_sec=20.0,
        damping_ratio=1.0,
        initial_condition=5.0,
        solver_type=SolverType.RUNGE_KUTTA,
    )

    filter = SecondOrderLowPassFilter(cfg)

    assert filter._state.feedback_state == pytest.approx(5.0)
    assert filter._state.filtered_output == pytest.approx(5.0)


def test_dc_gain_converges_to_constant():
    """Constant input should eventually appear unchanged at the output."""
    filt = create_filter()

    output = 0.0

    for _ in range(500):
        output, _ = filt.step(x=1.0, time_difference=0.01)

    assert output == pytest.approx(1.0, abs=1e-2)


def test_step_response_moves_towards_input():
    """A step input should smoothly approach the target."""
    filt = create_filter()

    outputs = []

    for _ in range(100):
        y, _ = filt.step(x=1.0, time_difference=0.01)
        outputs.append(y)

    assert outputs[0] < outputs[-1]
    assert outputs[-1] <= 1.05
    assert outputs[-1] > 0.9


def test_high_frequency_signal_is_attenuated():
    """Alternating ±1 input should be heavily attenuated."""
    filt = create_filter()

    outputs = []

    for i in range(500):
        x = 1.0 if i % 2 == 0 else -1.0
        y, _ = filt.step(x=x, time_difference=0.01)
        outputs.append(y)

    steady_state = outputs[-100:]

    amplitude = max(abs(v) for v in steady_state)

    assert amplitude < 0.5


def test_run_resets_filter_state():
    filt = create_filter()

    for _ in range(20):
        filt.step(1.0, 0.01)

    outputs1, derivs1 = filt.run([1.0] * 20)
    outputs2, derivs2 = filt.run([1.0] * 20)

    assert outputs1 == pytest.approx(outputs2)
    assert derivs1 == pytest.approx(derivs2)


def test_run_rejects_wrong_timestamp_length():
    filt = create_filter()

    with pytest.raises(ValueError):
        filt.run([1.0, 2.0], [0.0])


def test_run_rejects_none_timestamp():
    filt = create_filter()

    with pytest.raises(ValueError):
        filt.run([1.0], [None])


def test_reset_restores_initial_condition():
    cfg = LowPassFilterConfig(
        cut_off_frequency_rad_per_sec=20.0,
        damping_ratio=1.0,
        initial_condition=2.5,
        solver_type=SolverType.RUNGE_KUTTA,
    )

    filt = SecondOrderLowPassFilter(cfg)

    for _ in range(20):
        filt.step(10.0, 0.01)

    filt.reset()

    assert filt._state.feedback_state == pytest.approx(2.5)
    assert filt._state.filtered_output == pytest.approx(2.5)


def test_motor_control_pid_import():
    import motor_python.motor_control_using_pid

    assert motor_python.motor_control_using_pid is not None
