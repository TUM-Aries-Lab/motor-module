"""Motor control main entry point using AK60Motor class."""

import argparse
import time

from loguru import logger

from motor_python.cube_mars_motor import AK60Motor
from motor_python.definitions import DEFAULT_LOG_LEVEL, LogLevel
from motor_python.utils import setup_logger


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
    logger.info("Hello, world!")
    logger.info("Starting motor control loop...")

    # Use the AK60Motor class with context manager
    with AK60Motor() as motor:
        logger.info("Testing motor feedback response...")

        # Query motor status at startup
        logger.info("Initial motor status query:")
        motor.get_status()
        time.sleep(0.5)

        # Simulate continuous operation like with IMU input
        # In real application: while True, read IMU, calculate position, send command
        logger.info("Simulating continuous motor control (like with IMU feedback)...")

        try:
            loop_count = 0
            while True:
                # Simulate IMU reading -> position calculation
                # For testing, we'll just cycle through some positions
                target_position = (loop_count * 30) % 360  # 0, 30, 60, 90...

                # Send position command (like you would from IMU data)
                motor.set_position(target_position)

                # Query motor status periodically (every 10 commands)
                if loop_count % 10 == 0:
                    logger.info(f"=== Status Check at loop {loop_count} ===")
                    motor.get_status()

                # Small delay between commands (adjust based on your control loop rate)
                time.sleep(0.1)  # 10 Hz control loop

                loop_count += 1

                # Stop after 50 iterations for testing (remove this in production)
                if loop_count >= 50:
                    logger.info("Test complete - stopping loop")
                    break

        except KeyboardInterrupt:
            logger.info("Interrupted by user")

        # Stop motor safely
        logger.info("Stopping motor...")
        motor.stop()

        # Final status
        logger.info("Final motor status:")
        motor.get_status()
        time.sleep(0.5)

    logger.info("Motor control loop complete!")

    # Use the AK60Motor class with context manager
    with AK60Motor() as motor:
        logger.info("Testing motor feedback response...")

        # Query motor status at startup
        logger.info("Initial motor status query:")
        motor.get_status()
        time.sleep(0.5)

        # Simulate continuous operation like with IMU input
        # In real application: while True, read IMU, calculate position, send command
        logger.info("Simulating continuous motor control (like with IMU feedback)...")

        try:
            loop_count = 0
            while True:
                # Simulate IMU reading -> position calculation
                # For testing, we'll just cycle through some positions
                target_position = (loop_count * 30) % 360  # 0, 30, 60, 90...

                # Send position command (like you would from IMU data)
                motor.set_position(target_position)

                # Query motor status periodically (every 10 commands)
                if loop_count % 10 == 0:
                    logger.info(f"=== Status Check at loop {loop_count} ===")
                    motor.get_status()

                # Small delay between commands (adjust based on your control loop rate)
                time.sleep(0.1)  # 10 Hz control loop

                loop_count += 1

                # Stop after 50 iterations for testing (remove this in production)
                if loop_count >= 50:
                    logger.info("Test complete - stopping loop")
                    break

        except KeyboardInterrupt:
            logger.info("Interrupted by user")

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
