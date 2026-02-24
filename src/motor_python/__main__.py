"""Motor control main entry point — CAN interface (CubeMarsAK606v3CAN)."""

import argparse

from loguru import logger

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN
from motor_python.definitions import DEFAULT_LOG_LEVEL, LogLevel
from motor_python.examples_can import run_motor_demo_can
from motor_python.utils import setup_logger


def main(
    log_level: str = DEFAULT_LOG_LEVEL, stderr_level: str = DEFAULT_LOG_LEVEL
) -> None:
    """Run the main CAN motor control loop.

    Pre-condition: CAN interface must be up.
        sudo ip link set can0 up type can bitrate 1000000 berr-reporting on restart-ms 100

    :param log_level: The log level to use.
    :param stderr_level: The std err level to use.
    :return: None
    """
    setup_logger(log_level=log_level, stderr_level=stderr_level)
    logger.info("Starting CAN motor control loop...")

    try:
        motor = CubeMarsAK606v3CAN()
    except Exception as e:
        logger.error(f"Failed to initialize CAN motor controller: {e}")
        return

    with motor:
        if not motor.connected:
            logger.warning(
                "CAN bus not available. Run: sudo ip link set can0 up "
                "type can bitrate 1000000 berr-reporting on restart-ms 100"
            )
            return

        # Enter Servo mode before sending any control commands
        motor.enable_motor()

        if not motor.check_communication():
            logger.warning(
                "Motor not responding. Check power, CANH/CANL wiring, "
                "120 ohm termination, and disconnect UART cable."
            )
            return

        logger.info("Motor online - querying initial status...")
        motor.get_status()

        try:
            run_motor_demo_can(motor)
        except KeyboardInterrupt:
            logger.info("Interrupted by user")

        # stop() + bus.shutdown() called automatically by context manager

    logger.info("CAN motor control loop complete!")


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
