"""Motor control main entry point — CAN interface (CubeMarsAK606v3CAN)."""

import argparse

from loguru import logger

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN
from motor_python.definitions import CAN_DEFAULTS, DEFAULT_LOG_LEVEL, LogLevel
from motor_python.examples_can import multi_motor_can_example, run_motor_demo_can
from motor_python.utils import setup_logger


def main(
    log_level: str = DEFAULT_LOG_LEVEL,
    stderr_level: str = DEFAULT_LOG_LEVEL,
    dual: bool = False,
    motor_id_left: int = CAN_DEFAULTS.motor_can_id,
    motor_id_right: int = 0x04,
) -> None:
    """Run the main CAN motor control loop.

    Pre-condition: CAN interface must be up.
        sudo ip link set can0 up type can bitrate 1000000 berr-reporting on restart-ms 100

    :param log_level: The log level to use.
    :param stderr_level: The std err level to use.
    :param dual: If True, run the two-motor synchronized demo instead.
    :param motor_id_left: CAN ID for the left / primary motor (default: 0x03).
    :param motor_id_right: CAN ID for the right / secondary motor (default: 0x04).
    :return: None
    """
    setup_logger(log_level=log_level, stderr_level=stderr_level)

    # --- Two-motor mode ---
    if dual:
        logger.info(
            f"Starting dual-motor CAN demo "
            f"(left=0x{motor_id_left:02X}, right=0x{motor_id_right:02X})..."
        )
        multi_motor_can_example(
            left_can_id=motor_id_left, right_can_id=motor_id_right
        )
        logger.info("Dual-motor CAN demo complete!")
        return

    # --- Single-motor mode ---
    logger.info("Starting CAN motor control loop...")

    try:
        motor = CubeMarsAK606v3CAN(motor_can_id=motor_id_left)
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
    parser.add_argument(
        "--dual",
        action="store_true",
        default=False,
        help="Run the two-motor synchronized demo (requires two motors on the bus).",
    )
    parser.add_argument(
        "--motor-id-left",
        default=CAN_DEFAULTS.motor_can_id,
        type=lambda x: int(x, 0),  # accepts 0x03 or 3
        help="CAN ID of the left / primary motor (default: 0x03).",
    )
    parser.add_argument(
        "--motor-id-right",
        default=0x04,
        type=lambda x: int(x, 0),
        help="CAN ID of the right / secondary motor (default: 0x04).",
    )
    args = parser.parse_args()

    main(
        log_level=args.log_level,
        stderr_level=args.stderr_level,
        dual=args.dual,
        motor_id_left=args.motor_id_left,
        motor_id_right=args.motor_id_right,
    )
