"""AK60-6 PID controller using CubeMars Force Control Mode (MIT) only."""

import time
from venv import logger

import numpy as np

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN
from motor_python.definitions import LowPassFilterConfig, PIDConfig
from motor_python.pid_controller import PIDController


class PIDMotorController:
    """PID Controller methods specific for the motor.

    High-level wrapper around CubeMars motor using PID position control.
    Connects to the motor, checks communication, and provides methods to move to a target position using PID control.
    """

    def __init__(self, motor: CubeMarsAK606v3CAN):
        self.motor = motor

        filter_config = LowPassFilterConfig()

        self.pid = PIDController(
            pid_config=PIDConfig(
                proportional_gain=2.0,
                integral_gain=0.0,
                derivative_gain=0.3,
            ),
            filter_config=filter_config,
        )

    def move_to(
        self,
        target_degrees: float,
        duration: float = 2.0,
        rate_hz: int = 10,
        kp: float = 0.0,
        kd: float = 0.0,
    ):
        """Move motor to a target position using PID control.

        :param target_degrees: target angle in degrees
        :param velocity_rad_s: desired velocity in radians per second
        :param duration: how long to run control loop
        :param rate_hz: control frequency
        """
        dt = 1.0 / rate_hz
        start_time = time.time()

        while time.time() - start_time < duration:
            current_position = self.motor.get_position()
            if current_position is None:
                continue

            error = target_degrees - current_position
            if abs(error) < 0.2:  # degrees
                logger.info(
                    f"Target reached within tolerance: {current_position:.2f} deg"
                )
                break

            now = time.monotonic()

            # --- PID compute (deg in, deg in) ---
            velocity_cmd = self.pid.compute_output(
                timestamp=now,
                motor_reference=target_degrees,
                motor_position=current_position,
            )

            logger.info(f"PID raw output: {velocity_cmd:.2f} deg/s")

            MAX_VELOCITY_DEG_S = 30
            velocity_cmd = np.clip(
                velocity_cmd, -MAX_VELOCITY_DEG_S, MAX_VELOCITY_DEG_S
            )

            # convert deg/s → rad/s (MIT expects rad/s)
            velocity_rad_s = np.radians(velocity_cmd)

            self.motor.set_mit_mode(
                pos_rad=np.radians(target_degrees),
                vel_rad_s=velocity_rad_s,
                kp=kp,
                kd=kd,
            )
            logger.info(
                f"Current = {current_position:.2f} deg | Target = {target_degrees:.2f} deg | "
                f"PID vel = {velocity_cmd:.2f} deg/s ({velocity_rad_s:.2f} rad/s)"
            )

            time.sleep(dt)

    def hold(self, target_degrees: float, kp: float = 20.0, kd: float = 1.0):
        """Hold motor at target position indefinitely."""
        while True:
            self.motor.set_mit_mode(
                pos_rad=np.radians(target_degrees),
                vel_rad_s=0.0,
                kp=kp,
                kd=kd,
            )
            time.sleep(0.01)

    def stop(self):
        """Stop the motor."""
        self.motor.stop()
