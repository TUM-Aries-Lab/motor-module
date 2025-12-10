"""Example motor control functions demonstrating different control modes."""

import math
import time

from loguru import logger

from motor_python.cube_mars_motor import CubeMarsAK606v3


def run_position_control(
    motor: CubeMarsAK606v3, steps: int = 10, max_angle: float = 30.0
) -> None:
    """Run position control mode with sine wave motion.

    :param motor: Motor controller instance.
    :param steps: Number of steps in the sine wave motion.
    :param max_angle: Maximum angle in degrees for the sine wave amplitude.
    :return: None
    """
    for step in range(steps):
        # Sine wave: -max_angle to +max_angle over specified steps
        angle = max_angle * math.sin(step * 2 * math.pi / steps)
        motor.set_position(angle)
        time.sleep(0.1)

        # Query status periodically
        if step % 3 == 0:
            motor.get_status()


def run_velocity_control(motor: CubeMarsAK606v3, velocity: int = 5000) -> None:
    """Run velocity control mode with forward and reverse.

    :param motor: Motor controller instance.
    :param velocity: Target velocity in ERPM (Electrical RPM). Positive for forward,
                     negative for reverse. Default is 5000 ERPM.
    :return: None
    """
    logger.info("Forward velocity...")
    motor.set_velocity(velocity)  # Forward
    time.sleep(0.5)
    motor.get_status()

    logger.info("Reverse velocity...")
    motor.set_velocity(-velocity)  # Reverse
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
            # Cycle through control modes
            mode_index = loop_count % len(control_modes)
            mode_name, control_mode_func = control_modes[mode_index]

            logger.info(f"[Loop {loop_count}] {mode_name}")
            control_mode_func(motor)

            # Brief pause between cycles
            time.sleep(0.2)
            loop_count += 1

    except KeyboardInterrupt:
        logger.info("Interrupted by user - stopping motor loop")


def run_two_rotation_test(motor: CubeMarsAK606v3) -> None:
    """Run a test with two full rotations: slow then fast.

    :param motor: Motor controller instance.
    :return: None
    """
    logger.info("Starting motor test: 2 full rotations (slow then fast)...")

    try:
        # First rotation: SLOW (0.5 second delays)
        logger.info("=== ROTATION 1: SLOW ===")
        for i in range(12):
            target_position = (i * 30) % 360  # 0, 30, 60, 90...
            logger.info(f"Slow: Moving to {target_position}° (position {i + 1}/12)")
            motor.set_position(target_position)
            time.sleep(0.5)  # Slow movement

            # Query status at key positions
            if i % 3 == 0:
                motor.get_status()

        logger.info("First rotation complete!")
        time.sleep(1.0)

        # Second rotation: FAST (0.2 second delays)
        logger.info("=== ROTATION 2: FAST ===")
        for i in range(12):
            target_position = (i * 30) % 360  # 0, 30, 60, 90...
            logger.info(f"Fast: Moving to {target_position}° (position {i + 1}/12)")
            motor.set_position(target_position)
            time.sleep(0.2)  # Fast movement

            # Query status at key positions
            if i % 6 == 0:
                motor.get_status()

        logger.info("Second rotation complete! Test finished.")

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
