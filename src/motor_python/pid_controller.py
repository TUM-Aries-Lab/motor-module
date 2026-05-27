"""A pid controller for the motor."""

from venv import logger

from numpy import clip

from motor_python.definitions import LowPassFilterConfig, PIDConfig
from motor_python.second_order_low_pass_filter import (
    SecondOrderLowPassFilter,
)


class PIDController:
    """PID controller.

    The derivative term is NOT the derivative of the error.
    The previous velocity motor command is filtered by a second-order low-pass filter
    and subtracted. This is velocity feedback damping smoothly resists fast changes
    in the output without differentiating noisy sensor data.
    """

    def __init__(
        self, pid_config: PIDConfig, filter_config: LowPassFilterConfig
    ) -> None:
        """Initialize the pid controller.

        :param PIDConfig pid_config: Configurations of the pid controller, containing proportional gain kp, integral gain ki, derivative gain kd, and min, max clamp on the output.
        :param FilterConfig filter_config: Configurations of the second-order low-pass filter, containing cut-off frequency, damping ratio, initial condition, filter solver type. Time difference should not be initialized here.
        :return: None
        """
        self.config: PIDConfig = pid_config
        self.low_pass_filter: SecondOrderLowPassFilter = SecondOrderLowPassFilter(
            config=filter_config
        )

        self._integral: float = 0.0
        self._prev_velocity: float = 0.0  # previous velocity output fed into LPF
        self._prev_timestamp: float | None = None

    def compute_output(
        self, timestamp: float, motor_reference: float, motor_position: float
    ) -> float:
        """Compute the velocity command for one control cycle using the PID controller.

        :param float timestamp: Current timestamp of the controller.
        :param float motor_reference: Desired target velocity motor command of reference motion.
        :param float motor_position: Current measured motor position of actual motion.

        :return: Commanded velocity, clamped to output_limits if set.
        :rtype: float
        """
        # Initialize previous timestamp on first call
        if self._prev_timestamp is None:
            self._prev_timestamp = timestamp
            return 0.0

        #  calculate how much the motors have to move
        error = motor_reference - motor_position

        MAX_ERROR = 180  # degrees (or rad depending on your system)
        if abs(error) > MAX_ERROR:
            logger.debug(
                f"Warning: PID error {error:.2f} exceeds max expected range, clipping to ±{MAX_ERROR}"
            )
            error = clip(error, -MAX_ERROR, MAX_ERROR)

        proportional = self.config.proportional_gain * error

        time_difference = timestamp - self._prev_timestamp
        self._integral += error * time_difference
        integral_term = self.config.integral_gain * self._integral

        if self.config.output_limits is not None:
            low, high = self.config.output_limits
        else:
            logger.debug(
                "Warning: PID output limits not set, using default integral clamp of ±100.0"
            )
            low, high = -100, 100

        integral = clip(a=integral_term, a_min=low, a_max=high)

        _, damping_derivative = self.low_pass_filter.step(
            x=self._prev_velocity, time_difference=time_difference
        )

        derivative = self.config.derivative_gain * damping_derivative
        output = proportional + integral - derivative
        if self.config.output_limits is not None:
            low, high = self.config.output_limits
        else:
            logger.debug(
                "Warning: PID output limits not set, using default output clamp of ±100.0"
            )
            low, high = -100.0, 100.0
        output = clip(a=output, a_min=low, a_max=high)

        # Store velocity for next cycle's D term
        self._prev_velocity = output
        self._prev_timestamp = timestamp
        return output

    def reset(self) -> None:
        """Reset internal state integral and velocity feedback.

        :return: None
        """
        self._integral = 0.0
        self._prev_velocity = 0.0
