"""Example usage functions for CAN motor control on Jetson Orin Nano."""

import time

from loguru import logger

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN
from motor_python.definitions import MOTOR_LIMITS, TendonAction

# ---------------------------------------------------------------------------
# Composable helpers (accept a motor instance)
# ---------------------------------------------------------------------------


def run_velocity_control_can(
    motor: CubeMarsAK606v3CAN,
    velocity_erpm: int = MOTOR_LIMITS.default_velocity_demo_erpm,
) -> None:
    """Run forward/reverse velocity control and stop.

    :param motor: CAN motor instance (must already be enabled).
    :param velocity_erpm: Target velocity magnitude in ERPM.
    :return: None
    """
    logger.info(f"Forward velocity ({velocity_erpm} ERPM)...")
    motor.set_velocity(velocity_erpm=velocity_erpm)
    time.sleep(0.5)
    motor.get_status()

    logger.info(f"Reverse velocity (-{velocity_erpm} ERPM)...")
    motor.set_velocity(velocity_erpm=-velocity_erpm)
    time.sleep(0.5)
    motor.get_status()

    logger.info("Stop...")
    motor.stop()
    motor.get_status()


def run_position_control_can(motor: CubeMarsAK606v3CAN) -> None:
    """Run position control demo: 90°, -90°, 180° (with speed), back to 0°.

    :param motor: CAN motor instance (must already be enabled).
    :return: None
    """
    logger.info("Position control: move to 90°...")
    motor.set_position(position_degrees=90.0)
    time.sleep(1.5)
    motor.get_status()

    logger.info("Position control: move to -90°...")
    motor.set_position(position_degrees=-90.0)
    time.sleep(1.5)
    motor.get_status()

    logger.info("Position control: move to 180° at 8000 ERPM (trapezoidal)...")
    motor.set_position_velocity_accel(
        position_degrees=180.0,
        velocity_erpm=8000,
    )
    time.sleep(2.0)
    motor.get_status()

    logger.info("Position control: return to home (0°)...")
    motor.set_position(position_degrees=0.0)
    time.sleep(1.5)
    motor.get_status()
    motor.stop()


def run_exosuit_tendon_control_can(motor: CubeMarsAK606v3CAN) -> None:
    """Demonstrate PULL / RELEASE / STOP tendon control sequence.

    :param motor: CAN motor instance (must already be enabled).
    :return: None
    """
    logger.info("Pull tendon (lift)...")
    motor.control_exosuit_tendon(action=TendonAction.PULL, velocity_erpm=12000)
    time.sleep(0.8)

    logger.info("Release tendon (lower)...")
    motor.control_exosuit_tendon(action=TendonAction.RELEASE, velocity_erpm=8000)
    time.sleep(0.8)

    logger.info("Stop...")
    motor.control_exosuit_tendon(action=TendonAction.STOP)
    time.sleep(0.3)
    motor.get_status()


def run_max_rpm_test_can(
    motor: CubeMarsAK606v3CAN, duration_seconds: float = 3.0
) -> None:
    """Spin at maximum safe ERPM for the given duration then stop.

    :param motor: CAN motor instance (must already be enabled).
    :param duration_seconds: How long to hold max speed (default: 3 s).
    :return: None
    """
    max_erpm = MOTOR_LIMITS.max_velocity_electrical_rpm
    logger.info(f"Spinning at max RPM ({max_erpm} ERPM) for {duration_seconds}s...")
    motor.set_velocity(velocity_erpm=max_erpm)
    time.sleep(duration_seconds)
    logger.info("Stopping motor...")
    motor.stop()
    time.sleep(0.5)
    motor.get_status()


def run_motor_demo_can(motor: CubeMarsAK606v3CAN) -> None:
    """Run the full CAN motor control demonstration for exosuit use.

    Assumes the motor is already enabled (enable_motor() already called).

    :param motor: CAN motor instance.
    :return: None
    """
    logger.info("Starting CAN exosuit motor demo...")
    logger.info("Press Ctrl+C to stop")

    try:
        logger.info("\n" + "=" * 60)
        logger.info(
            f">>> RUNNING: run_velocity_control_can({MOTOR_LIMITS.default_velocity_demo_erpm} ERPM)"
        )
        logger.info("=" * 60)
        run_velocity_control_can(
            motor, velocity_erpm=MOTOR_LIMITS.default_velocity_demo_erpm
        )
        motor.stop()
        time.sleep(2.0)

        logger.info("\n" + "=" * 60)
        logger.info(">>> RUNNING: run_position_control_can()")
        logger.info("=" * 60)
        run_position_control_can(motor)
        motor.stop()
        time.sleep(2.0)

        logger.info("\n" + "=" * 60)
        logger.info(">>> RUNNING: run_exosuit_tendon_control_can()")
        logger.info("=" * 60)
        run_exosuit_tendon_control_can(motor)
        motor.stop()
        time.sleep(2.0)

        logger.info("\n" + "=" * 60)
        logger.info(">>> RUNNING: run_max_rpm_test_can(2s)")
        logger.info("=" * 60)
        run_max_rpm_test_can(motor, duration_seconds=2.0)

    except KeyboardInterrupt:
        logger.info("Demo stopped by user")
        motor.stop()


# ---------------------------------------------------------------------------
# Standalone scripts (manage their own motor instances)
# ---------------------------------------------------------------------------


def basic_can_example() -> None:
    """Demonstrate basic CAN motor control (standalone — manages own motor)."""
    logger.info("Starting CAN motor control example")

    with CubeMarsAK606v3CAN(
        motor_can_id=0x03, interface="can0", bitrate=1000000
    ) as motor:
        motor.enable_motor()

        if not motor.check_communication():
            logger.error("Motor not responding - check connections and power")
            return

        logger.info("Motor communication verified!")
        run_motor_demo_can(motor)
        logger.success("CAN motor control example completed successfully!")


def multi_motor_can_example() -> None:
    """Control multiple motors on the same CAN bus (standalone)."""
    logger.info("Starting multi-motor CAN example")

    motor_left = CubeMarsAK606v3CAN(motor_can_id=0x03, interface="can0")
    motor_right = CubeMarsAK606v3CAN(motor_can_id=0x04, interface="can0")

    try:
        motor_left.enable_motor()
        motor_right.enable_motor()

        left_ok = motor_left.check_communication()
        right_ok = motor_right.check_communication()

        if not left_ok or not right_ok:
            logger.error(
                f"Motor status - Left: {'OK' if left_ok else 'FAIL'}, "
                f"Right: {'OK' if right_ok else 'FAIL'}"
            )
            return

        logger.info("Synchronized pull...")
        motor_left.set_velocity(velocity_erpm=10000)
        motor_right.set_velocity(velocity_erpm=10000)
        time.sleep(2)

        logger.info("Synchronized release...")
        motor_left.set_velocity(velocity_erpm=-8000)
        motor_right.set_velocity(velocity_erpm=-8000)
        time.sleep(2)

        motor_left.stop()
        motor_right.stop()
        logger.success("Multi-motor example completed!")

    finally:
        motor_left.close()
        motor_right.close()


if __name__ == "__main__":
    basic_can_example()
