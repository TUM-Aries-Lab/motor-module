"""Motor control main entry point using CubeMarsAK606v3 class."""

import argparse
import math
import time

from loguru import logger

from motor_python.cube_mars_motor import CubeMarsAK606v3
from motor_python.definitions import DEFAULT_LOG_LEVEL, LogLevel
from motor_python.utils import setup_logger


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

            logger.info(f"[Loop {loop_count}] {mode_name}")
            mode_func(motor)

            # Brief pause between cycles
            time.sleep(0.2)

    except KeyboardInterrupt:
        logger.info("Interrupted by user - stopping motor loop")


def main(
    log_level: str = DEFAULT_LOG_LEVEL, stderr_level: str = DEFAULT_LOG_LEVEL
) -> None:
    """Run the main motor control loop.

    :param log_level: The log level to use.
    :param stderr_level: The std err level to use.
    :return: None
    """
    setup_logger(log_level=log_level, stderr_level=stderr_level)
    logger.info("Starting motor control loop...")

    # Use the CubeMarsAK606v3 class with context manager
    try:
        motor = CubeMarsAK606v3()
    except Exception as e:
        logger.error(f"Failed to initialize motor controller: {e}")
        logger.info("Exiting gracefully - hardware not available")
        return

    with motor:
        if not motor.connected:
            logger.warning("Motor hardware not connected - cannot run motor test")
            logger.info("Exiting gracefully - hardware not available")
            return

        logger.info("Testing motor feedback response...")

        # Verify motor is communicating
        logger.info("Verifying motor communication...")
        if not motor.check_communication():
            logger.warning(
                "Motor is connected but not responding - hardware may be powered off or cables disconnected"
            )
            logger.info("Exiting gracefully - hardware not communicating")
            return

        # Query motor status at startup
        logger.info("Initial motor status query:")
        motor.get_status()
        time.sleep(0.5)

        # Run continuous motor control loop
        run_motor_loop(motor)

        # Stop motor safely
        logger.info("Stopping motor...")
        motor.stop()

        # Final status
        logger.info("Final motor status:")
        motor.get_status()
        time.sleep(0.5)

    logger.info("Motor control loop complete!")


if __name__ == "__main__":  # pragma: no cover
    parser = argparse.ArgumentParser("Run the pipeline.")
    parser.add_argument(
        "--log-level",
        default=DEFAULT_LOG_LEVEL,
        choices=list(LogLevel()),
        help="Set the log level.",
        required=False,
        type=str,
    )
    parser.add_argument(
        "--stderr-level",
        default=DEFAULT_LOG_LEVEL,
        choices=list(LogLevel()),
        help="Set the std err level.",
        required=False,
        type=str,
    )
    args = parser.parse_args()

    main(log_level=args.log_level)
