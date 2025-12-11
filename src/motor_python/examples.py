"""Example usage functions for motor control."""

import math
import time

from loguru import logger

from motor_python.cube_mars_motor import CubeMarsAK606v3


def run_position_control(motor: CubeMarsAK606v3) -> None:
    """Run position control mode with sine wave motion.

    :param motor: Motor controller instance.
    :return: None
    """
    for step in range(10):
        # Sine wave: -30° to +30° over 10 steps
        angle = 30 * math.sin(step * 2 * math.pi / 10)
        motor.set_position(angle)
        time.sleep(0.1)

        # Query status periodically
        if step % 3 == 0:
            motor.get_status()


def run_velocity_control(motor: CubeMarsAK606v3) -> None:
    """Run velocity control mode with forward and reverse.

    :param motor: Motor controller instance.
    :return: None
    """
    logger.info("Forward velocity...")
    motor.set_velocity(5000)  # Forward
    time.sleep(0.5)
    motor.get_status()

    logger.info("Reverse velocity...")
    motor.set_velocity(-5000)  # Reverse
    time.sleep(0.5)
    motor.get_status()

    motor.set_velocity(0)  # Stop


def run_duty_cycle_control(motor: CubeMarsAK606v3) -> None:
    """Run duty cycle control mode.

    :param motor: Motor controller instance.
    :return: None
    """
    for duty in [0.1, 0.0, -0.1, 0.0]:
        motor.set_duty_cycle(duty)
        time.sleep(0.3)
    motor.get_status()


def run_current_control(motor: CubeMarsAK606v3) -> None:
    """Run current control mode with precise torque.

    :param motor: Motor controller instance.
    :return: None
    """
    for current in [1.0, 0.0, -1.0, 0.0]:
        motor.set_current(current)
        time.sleep(0.3)
    motor.get_status()


def run_motor_loop(motor: CubeMarsAK606v3) -> None:
    """Run continuous motor control loop cycling through different modes.

    :param motor: Motor controller instance.
    :return: None
    """
    logger.info("Starting continuous motor control loop...")
    logger.info("Press Ctrl+C to stop")

    control_modes = [
        ("Position control mode", run_position_control),
        ("Velocity control mode", run_velocity_control),
        ("Duty cycle control mode", run_duty_cycle_control),
        ("Current control mode", run_current_control),
    ]

    try:
        loop_count = 0
        while True:
            loop_count += 1
            mode_name, mode_func = control_modes[loop_count % len(control_modes)]
            logger.info(f"Running {mode_name} (iteration {loop_count})...")
            mode_func(motor)
            time.sleep(0.5)
    except KeyboardInterrupt:
        logger.info("Loop stopped by user")
        motor.stop()
