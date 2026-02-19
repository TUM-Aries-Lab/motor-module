"""Example script for CAN motor control on Jetson Orin Nano."""

import time

from loguru import logger

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN
from motor_python.definitions import TendonAction


def basic_can_example():
    """Demonstrate basic CAN motor control."""
    logger.info("Starting CAN motor control example")

    # Initialize motor with CAN ID 3 (from motor configuration)
    # Default interface is "can0", bitrate is 1 Mbps
    motor = CubeMarsAK606v3CAN(motor_can_id=0x03, interface="can0", bitrate=1000000)

    try:
        # Check if motor is communicating
        if not motor.check_communication():
            logger.error("Motor not responding - check connections and power")
            return

        logger.info("Motor communication verified!")

        # Get motor status
        logger.info("Querying motor status...")
        motor.get_status()
        time.sleep(1)

        # Velocity control example
        logger.info("Testing velocity control...")
        motor.set_velocity(velocity_erpm=10000)
        time.sleep(2)
        motor.stop()
        time.sleep(1)

        # Reverse direction
        motor.set_velocity(velocity_erpm=-8000)
        time.sleep(2)
        motor.stop()
        time.sleep(1)

        # Position control example
        logger.info("Testing position control...")
        motor.set_position(position_degrees=90.0)
        time.sleep(3)
        motor.set_position(position_degrees=0.0)
        time.sleep(3)

        # Tendon control helper
        logger.info("Testing tendon control...")
        motor.control_exosuit_tendon(action=TendonAction.PULL, velocity_erpm=10000)
        time.sleep(2)
        motor.control_exosuit_tendon(action=TendonAction.RELEASE, velocity_erpm=8000)
        time.sleep(2)
        motor.control_exosuit_tendon(action=TendonAction.STOP)

        logger.success("CAN motor control example completed successfully!")

    finally:
        # Always stop and close connection
        motor.close()


def multi_motor_can_example():
    """Control multiple motors on the same CAN bus."""
    logger.info("Starting multi-motor CAN example")

    # Initialize two motors with different CAN IDs
    motor_left = CubeMarsAK606v3CAN(motor_can_id=0x03, interface="can0")
    motor_right = CubeMarsAK606v3CAN(motor_can_id=0x04, interface="can0")

    try:
        # Check both motors
        left_ok = motor_left.check_communication()
        right_ok = motor_right.check_communication()

        if not left_ok or not right_ok:
            logger.error(
                f"Motor status - Left: {'OK' if left_ok else 'FAIL'}, "
                f"Right: {'OK' if right_ok else 'FAIL'}"
            )
            return

        # Synchronize motor movements
        logger.info("Synchronized pull...")
        motor_left.set_velocity(velocity_erpm=10000)
        motor_right.set_velocity(velocity_erpm=10000)
        time.sleep(2)

        logger.info("Synchronized release...")
        motor_left.set_velocity(velocity_erpm=-8000)
        motor_right.set_velocity(velocity_erpm=-8000)
        time.sleep(2)

        # Stop both motors
        motor_left.stop()
        motor_right.stop()

        logger.success("Multi-motor example completed!")

    finally:
        motor_left.close()
        motor_right.close()


def context_manager_example():
    """Use context manager for automatic motor cleanup."""
    logger.info("Starting context manager example")

    # Using 'with' ensures motor is properly closed even if errors occur
    with CubeMarsAK606v3CAN(motor_can_id=0x03) as motor:
        if motor.check_communication():
            motor.set_velocity(velocity_erpm=10000)
            time.sleep(2)
            motor.stop()
            logger.success("Context manager example completed!")
        else:
            logger.error("Motor not responding")
    # motor.close() is called automatically when exiting the 'with' block


if __name__ == "__main__":
    # Choose which example to run
    basic_can_example()
