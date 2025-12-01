"""Sample doc string."""

import argparse

import serial
from loguru import logger

from motor_python.config.definitions import DEFAULT_LOG_LEVEL, LogLevel
from motor_python.utils import setup_logger


def main(
    log_level: str = DEFAULT_LOG_LEVEL, stderr_level: str = DEFAULT_LOG_LEVEL
) -> None:
    """Run the main pipeline.

    :param log_level: The log level to use.
    :param stderr_level: The std err level to use.
    :return: None
    """
    setup_logger(log_level=log_level, stderr_level=stderr_level)
    logger.info("Hello, world!")

    ser = serial.Serial("/dev/ttyTHS1", 921600, timeout=1)
    logger.info(ser.name)
    logger.info(ser.is_open)

    position_loop_cmd = b"\xaa\x05\x4a\x05\x5d\x4a\x80\x84\x93\xbb"
    ser.write(position_loop_cmd)
    ser.close()
    logger.info(ser.is_open)


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
