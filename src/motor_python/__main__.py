"""Motor control main entry point — CAN interface (CubeMarsAK606v3CAN).

Example usage:
python -m motor_python --motor-ids 0x03
python -m motor_python --motor-ids 0x03 0x04
python -m motor_python --discover
python -m motor_python --dual

"""

import argparse

from loguru import logger

from motor_python.definitions import CAN_DEFAULTS, DEFAULT_LOG_LEVEL, LogLevel
from motor_python.examples_can import run_motor_demo_can, run_multi_motor_demo
from motor_python.motor_manager import MotorManager
from motor_python.utils import setup_logger
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN
from motor_python.definitions import (
    CAN_DEFAULTS,
    DEFAULT_LOG_LEVEL,
    DEFAULT_MOTOR_SPEC,
    LogLevel,
    MotorModel,
    set_current_motor_model_by_name,
)
from motor_python.examples_can import multi_motor_can_example, run_motor_demo_can
from motor_python.utils import setup_logger


def main(  # noqa: PLR0913
    log_level: str = DEFAULT_LOG_LEVEL,
    stderr_level: str = DEFAULT_LOG_LEVEL,
    dual: bool = False,
    discover: bool = False,
    motor_ids: list[int] | None = None,
    interface: str = CAN_DEFAULTS.interface,
    motor_model: str = DEFAULT_MOTOR_SPEC.model_name,
) -> None:
    """Run the main CAN motor control loop.

    Pre-condition: CAN interface must be up.
        sudo ip link set can0 up type can bitrate 1000000 berr-reporting on restart-ms 100

    :param log_level: The log level to use.
    :param stderr_level: The std err level to use.
    :param dual: If True, run the two-motor synchronized demo instead.
    :param motor_ids: List of CAN motor IDs to control.
    :param interface: The CAN interface to use.
    :return: None
    """
    setup_logger(log_level=log_level, stderr_level=stderr_level)

    set_current_motor_model_by_name(motor_model)
    logger.info(f"Selected motor model: {motor_model}")

    # --- Two-motor mode ---
    if dual:
        motor_ids = [CAN_DEFAULTS.motor_can_id, CAN_DEFAULTS.motor_can_id_2]

    if motor_ids is None:
        motor_ids = [CAN_DEFAULTS.motor_can_id]

    try:
        if discover:
            manager = MotorManager.discover(
                interface=interface
            )  # sets the discovered ids as motor_ids
        else:
            logger.info(
                f"Starting CAN motor control loop on interface '{interface}' with IDs: {motor_ids}"
            )
            manager = MotorManager(motor_ids=motor_ids, interface=interface)
    except Exception as e:
        logger.error(f"Failed to initialize CAN motor manager: {e}")
        return

    with manager:
        first_motor = next(iter(manager))
        if not first_motor.connected:
            logger.warning(
                "CAN bus not available. Run: sudo ip link set can0 up "
                "type can bitrate 1000000 berr-reporting on restart-ms 100"
            )
            return

        manager.enable_all()  # Enable all motors before checking communication
        status = (
            manager.check_all()
        )  # Check communication with all motors before proceeding
        if not any(status.values()):
            logger.warning(
                "No motors responding. Check power, CANH/CANL wiring, "
                "120 ohm termination, and CAN IDs."
            )
            return

        logger.info(
            f"Motor communication verified for IDs: {[motor_id for motor_id, ok in status.items() if ok]}"
        )
        for motor in manager:
            motor.get_status()

        try:
            if len(manager) == 1:
                run_motor_demo_can(next(iter(manager)))
            else:
                run_multi_motor_demo(manager)
        except KeyboardInterrupt:
            logger.info("Interrupted by user")

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
        "--discover",
        action="store_true",
        default=False,
        help="Automatically discover connected motors on the CAN bus.",
    )
    parser.add_argument(
        "--motor-ids",
        nargs="+",
        type=lambda x: int(x, 0),
        default=None,
        help=(
            "List of CAN motor IDs to control. "
            "Use space-separated values like --motor-ids 0x03 0x04."
        ),
    )
    parser.add_argument(
        "--interface",
        default=CAN_DEFAULTS.interface,
        help="SocketCAN interface to use (default: can0).",
    )
    parser.add_argument(
        "--motor-model",
        type=str,
        choices=list(MotorModel),
        default=DEFAULT_MOTOR_SPEC.model_name,
        help="Select motor type (AK60-6 or AK80-6).",
    )
    args = parser.parse_args()

    main(
        log_level=args.log_level,
        stderr_level=args.stderr_level,
        dual=args.dual,
        discover=args.discover,
        motor_ids=args.motor_ids,
        interface=args.interface,
    )
