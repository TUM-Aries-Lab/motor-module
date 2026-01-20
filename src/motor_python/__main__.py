"""Motor control main entry point using CubeMarsAK606v3 (UART) or CubeMarsAK606CAN (CAN) class."""

import argparse
import time

from loguru import logger

from motor_python.cube_mars_motor import CubeMarsAK606v3
from motor_python.cube_mars_motor_can import CubeMarsAK606CAN
from motor_python.definitions import DEFAULT_LOG_LEVEL, LogLevel
from motor_python.examples import run_motor_loop
from motor_python.utils import setup_logger


def main_uart(
    log_level: str = DEFAULT_LOG_LEVEL, stderr_level: str = DEFAULT_LOG_LEVEL
) -> None:
    """Run the main motor control loop using UART interface.

    :param log_level: The log level to use.
    :param stderr_level: The std err level to use.
    :return: None
    """
    setup_logger(log_level=log_level, stderr_level=stderr_level)
    logger.info("Starting motor control loop (UART interface)...")

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


def main_can(
    log_level: str = DEFAULT_LOG_LEVEL,
    stderr_level: str = DEFAULT_LOG_LEVEL,
    motor_id: int = 0x68,
) -> None:
    """Run the main motor control loop using CAN interface.

    :param log_level: The log level to use.
    :param stderr_level: The std err level to use.
    :param motor_id: CAN motor ID (default: 0x68).
    :return: None
    """
    setup_logger(log_level=log_level, stderr_level=stderr_level)
    logger.info("Starting motor control loop (CAN interface)...")

    # Use the CubeMarsAK606CAN class with context manager
    try:
        motor = CubeMarsAK606CAN(motor_id=motor_id)
    except Exception as e:
        logger.error(f"Failed to initialize CAN motor controller: {e}")
        return

    with motor:
        if not motor.connected:
            logger.warning("CAN bus not connected - cannot run motor test")
            return

        logger.info("Testing motor feedback response...")

        # Receive initial feedback
        logger.info("Waiting for motor feedback...")
        for _ in range(5):
            status = motor.receive_feedback(timeout=0.5)
            if status:
                break
            time.sleep(0.1)

        if not motor.communicating:
            logger.warning(
                "Motor not responding via CAN - hardware may be powered off or disconnected"
            )
            return

        # Demonstrate various control modes
        logger.info("\n=== Testing CAN Motor Control Modes ===\n")

        try:
            # Test 1: Duty cycle control
            logger.info("Test 1: Duty cycle control (10%)")
            motor.set_duty_cycle(0.1)
            time.sleep(2)
            motor.receive_feedback()

            logger.info("Setting duty cycle to 0%")
            motor.set_duty_cycle(0.0)
            time.sleep(1)

            # Test 2: Current control
            logger.info("\nTest 2: Current control (2A)")
            motor.set_current(2.0)
            time.sleep(2)
            motor.receive_feedback()

            motor.set_current(0.0)
            time.sleep(1)

            # Test 3: Velocity control
            logger.info("\nTest 3: Velocity control (1000 RPM)")
            motor.set_velocity(1000.0)
            time.sleep(3)
            motor.receive_feedback()

            motor.set_velocity(0.0)
            time.sleep(1)

            # Test 4: Position control
            logger.info("\nTest 4: Position control (180°)")
            motor.set_position(180.0)
            time.sleep(3)
            motor.receive_feedback()

            logger.info("Returning to 0°")
            motor.set_position(0.0)
            time.sleep(3)
            motor.receive_feedback()

            # Test 5: Position-Velocity control
            logger.info("\nTest 5: Position-Velocity control (360° at 5000 RPM)")
            motor.set_position_velocity(360.0, 5000.0, 10000.0)
            time.sleep(4)
            motor.receive_feedback()

            # Stop motor
            logger.info("\nStopping motor...")
            motor.set_current(0.0)
            time.sleep(1)

        except KeyboardInterrupt:
            logger.info("Interrupted by user")
            motor.set_current(0.0)

        # Final feedback
        logger.info("\nFinal motor status:")
        motor.receive_feedback(timeout=1.0)

    logger.info("Motor control loop complete!")


def main(
    log_level: str = DEFAULT_LOG_LEVEL,
    stderr_level: str = DEFAULT_LOG_LEVEL,
    interface: str = "uart",
    motor_id: int = 0x68,
) -> None:
    """Run the main motor control loop.

    :param log_level: The log level to use.
    :param stderr_level: The std err level to use.
    :param interface: Interface to use ('uart' or 'can').
    :param motor_id: CAN motor ID (only used for CAN interface).
    :return: None
    """
    if interface.lower() == "can":
        main_can(log_level=log_level, stderr_level=stderr_level, motor_id=motor_id)
    else:
        main_uart(log_level=log_level, stderr_level=stderr_level)


if __name__ == "__main__":  # pragma: no cover
    parser = argparse.ArgumentParser(
        description="Run motor control loop with UART or CAN interface."
    )
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
        "--interface",
        default="uart",
        choices=["uart", "can"],
        help="Communication interface to use (default: uart).",
        required=False,
        type=str,
    )
    parser.add_argument(
        "--motor-id",
        default=0x68,
        help="CAN motor ID in hex (default: 0x68). Only used for CAN interface.",
        required=False,
        type=lambda x: int(x, 0),  # Accepts hex (0x68) or decimal (104)
    )
    args = parser.parse_args()

    main(
        log_level=args.log_level,
        stderr_level=args.stderr_level,
        interface=args.interface,
        motor_id=args.motor_id,
    )
