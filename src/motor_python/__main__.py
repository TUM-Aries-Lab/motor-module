"""Motor control main entry point using CubeMarsAK606v3 class."""

import argparse
import time

from loguru import logger

from motor_python.cube_mars_motor import CubeMarsAK606v3
from motor_python.definitions import DEFAULT_LOG_LEVEL, LogLevel
from motor_python.examples import run_motor_loop
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

    # Use the CubeMarsAK606v3 class with context manager
    try:
        motor = CubeMarsAK606v3()
    except Exception as e:
        logger.error(f"Failed to initialize motor controller: {e}")
        return

    with motor:
        if not motor.connected or not motor.check_communication():
            logger.warning(
                "Motor hardware not connected - cannot run motor test"
                if not motor.connected
                else "Motor is connected but not responding - hardware may be powered off or cables disconnected"
            )
            return

        logger.info("Testing motor feedback response...")

        # Query motor status at startup
        logger.info("Initial motor status query:")
        motor.get_status()
        time.sleep(0.5)

        # Run the motor control loop with all modes
        try:
            run_motor_loop(motor)
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
