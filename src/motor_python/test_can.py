"""Test script for CAN motor communication."""

import time

from loguru import logger

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN


def test_can_interface():
    """Test basic CAN interface connectivity."""
    logger.info("=" * 60)
    logger.info("TEST 1: CAN Interface Connection")
    logger.info("=" * 60)

    try:
        motor = CubeMarsAK606v3CAN(motor_can_id=0x03, interface="can0", bitrate=1000000)
        if motor.connected:
            logger.success("✓ CAN interface 'can0' initialized successfully")
            motor.close()
            return True
        else:
            logger.error("✗ Failed to initialize CAN interface")
            return False
    except Exception as e:
        logger.error(f"✗ CAN interface error: {e}")
        logger.info("  → Make sure CAN interface is configured:")
        logger.info("     sudo ip link set can0 type can bitrate 1000000")
        logger.info("     sudo ip link set can0 up")
        return False


def test_motor_feedback():
    """Test receiving periodic feedback from motor."""
    logger.info("\n" + "=" * 60)
    logger.info("TEST 2: Motor Periodic Feedback")
    logger.info("=" * 60)
    logger.info("Listening for motor feedback messages (50 Hz)...")
    logger.info("Make sure motor is powered on and configured for CAN!")

    motor = CubeMarsAK606v3CAN(motor_can_id=0x03)

    if not motor.connected:
        logger.error("✗ CAN interface not connected")
        return False

    # Try to receive feedback for 3 seconds
    feedback_received = False
    start_time = time.time()

    while (time.time() - start_time) < 3.0:
        feedback = motor._receive_feedback(timeout=0.5)
        if feedback:
            logger.success("✓ Received motor feedback!")
            logger.info(f"  Position: {feedback.position_degrees:.2f}°")
            logger.info(f"  Speed: {feedback.speed_erpm} ERPM")
            logger.info(f"  Current: {feedback.current_amps:.2f} A")
            logger.info(f"  Temperature: {feedback.temperature_celsius}°C")
            logger.info(f"  Error Code: {feedback.error_code}")
            feedback_received = True
            break

    motor.close()

    if not feedback_received:
        logger.error("✗ No feedback received from motor")
        logger.info("  → Check motor power and CAN ID configuration")
        logger.info("  → Verify CANH/CANL wiring and 120Ω termination")
        return False

    return True


def test_communication_check():
    """Test the check_communication method."""
    logger.info("\n" + "=" * 60)
    logger.info("TEST 3: Communication Check")
    logger.info("=" * 60)

    motor = CubeMarsAK606v3CAN(motor_can_id=0x03)

    if motor.check_communication():
        logger.success("✓ Motor communication verified")
        motor.close()
        return True
    else:
        logger.error("✗ Motor not responding")
        motor.close()
        return False


def test_velocity_command():
    """Test sending velocity command."""
    logger.info("\n" + "=" * 60)
    logger.info("TEST 4: Velocity Command")
    logger.info("=" * 60)
    logger.warning(
        "Motor will rotate! Make sure tendon is disconnected or area is clear."
    )

    response = input("Continue with velocity test? (yes/no): ")
    if response.lower() != "yes":
        logger.info("Test skipped by user")
        return None

    motor = CubeMarsAK606v3CAN(motor_can_id=0x03)

    if not motor.check_communication():
        logger.error("✗ Motor not responding - skipping velocity test")
        motor.close()
        return False

    try:
        logger.info("Sending velocity command: 5000 ERPM...")
        motor.set_velocity(velocity_erpm=5000)
        time.sleep(1.0)

        # Check feedback
        feedback = motor._receive_feedback(timeout=0.5)
        if feedback:
            logger.info(f"Motor speed: {feedback.speed_erpm} ERPM")
            if abs(feedback.speed_erpm) > 1000:
                logger.success("✓ Motor is rotating")
            else:
                logger.warning("⚠ Motor speed lower than expected")

        logger.info("Stopping motor...")
        motor.stop()
        time.sleep(0.5)

        logger.success("✓ Velocity command test completed")
        motor.close()
        return True

    except Exception as e:
        logger.error(f"✗ Velocity test error: {e}")
        motor.stop()
        motor.close()
        return False


def test_position_command():
    """Test sending position command."""
    logger.info("\n" + "=" * 60)
    logger.info("TEST 5: Position Command")
    logger.info("=" * 60)
    logger.warning("Motor will move to position! Make sure area is clear.")

    response = input("Continue with position test? (yes/no): ")
    if response.lower() != "yes":
        logger.info("Test skipped by user")
        return None

    motor = CubeMarsAK606v3CAN(motor_can_id=0x03)

    if not motor.check_communication():
        logger.error("✗ Motor not responding - skipping position test")
        motor.close()
        return False

    try:
        logger.info("Getting current position...")
        current_pos = motor.get_position()
        if current_pos is not None:
            logger.info(f"Current position: {current_pos:.2f}°")

            # Move to +90 degrees relative
            target = current_pos + 90.0
            logger.info(f"Moving to position: {target:.2f}°...")
            motor.set_position(target)
            time.sleep(2.0)

            # Check position
            new_pos = motor.get_position()
            if new_pos is not None:
                logger.info(f"New position: {new_pos:.2f}°")
                error = abs(new_pos - target)
                if error < 10.0:
                    logger.success(f"✓ Position reached (error: {error:.2f}°)")
                else:
                    logger.warning(f"⚠ Position error: {error:.2f}°")

        motor.stop()
        motor.close()
        return True

    except Exception as e:
        logger.error(f"✗ Position test error: {e}")
        motor.stop()
        motor.close()
        return False


def run_all_tests():
    """Run all CAN communication tests."""
    logger.info("CAN Motor Communication Test Suite")
    logger.info("Make sure:")
    logger.info("  1. CAN interface (can0) is configured and up")
    logger.info("  2. Motor is powered on (24-48V)")
    logger.info("  3. SN65HVD230 transceiver is properly wired")
    logger.info("  4. Motor is configured for CAN mode (ID: 0x03, 1 Mbps)")

    input("\nPress Enter to start tests...")

    results = {}

    # Test 1: Interface
    results["interface"] = test_can_interface()
    if not results["interface"]:
        logger.error("\nCAN interface test failed - stopping here")
        return results

    # Test 2: Feedback
    results["feedback"] = test_motor_feedback()
    if not results["feedback"]:
        logger.error("\nMotor feedback test failed - stopping here")
        return results

    # Test 3: Communication check
    results["communication"] = test_communication_check()

    # Test 4: Velocity (optional)
    results["velocity"] = test_velocity_command()

    # Test 5: Position (optional)
    results["position"] = test_position_command()

    # Summary
    logger.info("\n" + "=" * 60)
    logger.info("TEST SUMMARY")
    logger.info("=" * 60)

    for test_name, result in results.items():
        if result is True:
            status = "✓ PASS"
        elif result is False:
            status = "✗ FAIL"
        else:
            status = "⊘ SKIPPED"
        logger.info(f"{test_name.upper():20s}: {status}")

    return results


if __name__ == "__main__":
    run_all_tests()
