"""Example usage of CAN motor control."""

import time

from loguru import logger

from motor_python.cube_mars_motor_can import CubeMarsAK606CAN


def run_can_motor_test(motor_id: int = 0x68) -> None:
    """Run a simple CAN motor test sequence.

    :param motor_id: CAN motor ID (default: 0x68).
    :return: None
    """
    logger.info(f"Initializing CAN motor with ID 0x{motor_id:02X}")

    # Create motor controller with context manager
    with CubeMarsAK606CAN(motor_id=motor_id) as motor:
        if not motor.connected:
            logger.error("Failed to connect to CAN bus")
            logger.info(
                "Troubleshooting steps:\n"
                "  1. Enable SPI: sudo /opt/nvidia/jetson-io/jetson-io.py\n"
                "  2. Load CAN modules: sudo modprobe can && sudo modprobe can_raw\n"
                "  3. Configure CAN interface: sudo ip link set can0 type can bitrate 500000\n"
                "  4. Bring up interface: sudo ip link set can0 up\n"
                "  5. Install python-can: pip install python-can\n"
            )
            return

        # Check communication
        logger.info("Checking motor communication...")
        if not motor.check_communication():
            logger.warning("Motor not responding - is it powered on?")
            return

        logger.success("Motor is communicating via CAN!")

        # Receive and display current status
        logger.info("\n=== Current Motor Status ===")
        status = motor.receive_feedback(timeout=1.0)
        if status:
            logger.info(f"Position: {status.position:.2f}°")
            logger.info(f"Speed: {status.speed:.0f} RPM")
            logger.info(f"Current: {status.current:.2f} A")
            logger.info(f"Temperature: {status.temperature}°C")
            logger.info(f"Error code: {status.error_code}")

        # Test different control modes
        logger.info("\n=== Running Control Tests ===\n")

        try:
            # Test 1: Small duty cycle
            logger.info("Test 1: Applying 5% duty cycle for 2 seconds")
            motor.set_duty_cycle(0.05)
            time.sleep(2)
            motor.receive_feedback()
            motor.set_duty_cycle(0.0)
            time.sleep(1)

            # Test 2: Current control
            logger.info("\nTest 2: Applying 1A current for 2 seconds")
            motor.set_current(1.0)
            time.sleep(2)
            motor.receive_feedback()
            motor.set_current(0.0)
            time.sleep(1)

            # Test 3: Velocity control
            logger.info("\nTest 3: Setting velocity to 500 RPM for 3 seconds")
            motor.set_velocity(500.0)
            time.sleep(3)
            motor.receive_feedback()
            motor.set_velocity(0.0)
            time.sleep(1)

            # Test 4: Position control
            logger.info("\nTest 4: Moving to 90° position")
            motor.set_position(90.0)
            time.sleep(3)
            motor.receive_feedback()

            logger.info("Returning to 0° position")
            motor.set_position(0.0)
            time.sleep(3)
            motor.receive_feedback()

            # Test 5: Position with velocity limit
            logger.info("\nTest 5: Moving to 180° with 3000 RPM velocity limit")
            motor.set_position_velocity(180.0, 3000.0, 5000.0)
            time.sleep(4)
            motor.receive_feedback()

            logger.info("Returning to 0°")
            motor.set_position_velocity(0.0, 3000.0, 5000.0)
            time.sleep(4)
            motor.receive_feedback()

        except KeyboardInterrupt:
            logger.warning("Test interrupted by user")

        finally:
            # Always stop the motor safely
            logger.info("\n=== Stopping Motor ===")
            motor.set_current(0.0)
            time.sleep(0.5)

            # Final status
            logger.info("\n=== Final Motor Status ===")
            status = motor.receive_feedback(timeout=1.0)
            if status:
                logger.info(f"Position: {status.position:.2f}°")
                logger.info(f"Speed: {status.speed:.0f} RPM")
                logger.info(f"Current: {status.current:.2f} A")
                logger.info(f"Temperature: {status.temperature}°C")

    logger.success("CAN motor test complete!")


def run_can_continuous_feedback(motor_id: int = 0x68, duration: float = 10.0) -> None:
    """Continuously receive and display CAN motor feedback.

    :param motor_id: CAN motor ID (default: 0x68).
    :param duration: Duration in seconds to receive feedback (default: 10.0).
    :return: None
    """
    logger.info(
        f"Starting continuous CAN feedback monitor (Motor ID: 0x{motor_id:02X})"
    )
    logger.info(f"Will run for {duration} seconds. Press Ctrl+C to stop early.\n")

    with CubeMarsAK606CAN(motor_id=motor_id) as motor:
        if not motor.connected:
            logger.error("Failed to connect to CAN bus")
            return

        start_time = time.time()
        feedback_count = 0

        try:
            while (time.time() - start_time) < duration:
                status = motor.receive_feedback(timeout=0.1)
                if status:
                    feedback_count += 1
                    logger.info(
                        f"[{feedback_count:04d}] "
                        f"Pos: {status.position:7.2f}° | "
                        f"Spd: {status.speed:7.0f} RPM | "
                        f"Cur: {status.current:6.2f} A | "
                        f"Temp: {status.temperature:3d}°C | "
                        f"Err: {status.error_code}"
                    )

        except KeyboardInterrupt:
            logger.info("\nStopped by user")

        logger.info(
            f"\nReceived {feedback_count} feedback messages in {time.time() - start_time:.1f} seconds"
        )
        if feedback_count > 0:
            logger.info(
                f"Average rate: {feedback_count / (time.time() - start_time):.1f} Hz"
            )


if __name__ == "__main__":
    # Run the test
    run_can_motor_test()

    # Optionally, run continuous feedback monitor
    # run_can_continuous_feedback(duration=5.0)
