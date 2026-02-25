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


def run_dual_motor_demo_can(
    motor_left: CubeMarsAK606v3CAN,
    motor_right: CubeMarsAK606v3CAN,
) -> None:
    """Run a synchronized two-motor exosuit demo.

    Both motors must already be enabled before calling this function.
    The left motor uses CAN ID 0x03 and the right motor uses CAN ID 0x04
    by convention, but any two distinct IDs work.

    Sequence:
      1. Synchronized pull  — both tendons pulled at the same time
      2. Synchronized release — both tendons released at the same time
      3. Left pull / right release — cross pattern
      4. Right pull / left release — cross pattern
      5. Full stop

    :param motor_left: CAN motor instance for the left actuator.
    :param motor_right: CAN motor instance for the right actuator.
    :return: None
    """
    try:
        logger.info("=== Dual-motor demo: synchronized PULL ===")
        motor_left.control_exosuit_tendon(TendonAction.PULL, velocity_erpm=10000)
        motor_right.control_exosuit_tendon(TendonAction.PULL, velocity_erpm=10000)
        time.sleep(1.5)
        motor_left.get_status()
        motor_right.get_status()

        logger.info("=== Dual-motor demo: synchronized RELEASE ===")
        motor_left.control_exosuit_tendon(TendonAction.RELEASE, velocity_erpm=8000)
        motor_right.control_exosuit_tendon(TendonAction.RELEASE, velocity_erpm=8000)
        time.sleep(1.5)

        logger.info("=== Dual-motor demo: left PULL / right RELEASE ===")
        motor_left.control_exosuit_tendon(TendonAction.PULL, velocity_erpm=10000)
        motor_right.control_exosuit_tendon(TendonAction.RELEASE, velocity_erpm=8000)
        time.sleep(1.5)

        logger.info("=== Dual-motor demo: right PULL / left RELEASE ===")
        motor_left.control_exosuit_tendon(TendonAction.RELEASE, velocity_erpm=8000)
        motor_right.control_exosuit_tendon(TendonAction.PULL, velocity_erpm=10000)
        time.sleep(1.5)

        logger.info("=== Dual-motor demo: STOP both ===")
        motor_left.control_exosuit_tendon(TendonAction.STOP)
        motor_right.control_exosuit_tendon(TendonAction.STOP)

    except KeyboardInterrupt:
        logger.info("Dual-motor demo interrupted by user")
        motor_left.stop()
        motor_right.stop()


def multi_motor_can_example(
    left_can_id: int = 0x03,
    right_can_id: int = 0x04,
) -> None:
    """Control two motors on the same CAN bus (standalone).

    Both motors share the can0 interface. Each listens only to its own
    feedback ID (0x2900 | motor_can_id) so they do not interfere.

    :param left_can_id: CAN ID of the left motor (default: 0x03).
    :param right_can_id: CAN ID of the right motor (default: 0x04).
    :return: None
    """
    logger.info(
        f"Starting dual-motor CAN example "
        f"(left=0x{left_can_id:02X}, right=0x{right_can_id:02X})"
    )

    motor_left = CubeMarsAK606v3CAN(motor_can_id=left_can_id, interface="can0")
    motor_right = CubeMarsAK606v3CAN(motor_can_id=right_can_id, interface="can0")

    try:
        motor_left.enable_motor()
        motor_right.enable_motor()

        left_ok = motor_left.check_communication()
        right_ok = motor_right.check_communication()

        if not left_ok:
            logger.error(
                f"Left motor (0x{left_can_id:02X}) not responding — "
                "check power, wiring, and CAN ID"
            )
        if not right_ok:
            logger.error(
                f"Right motor (0x{right_can_id:02X}) not responding — "
                "check power, wiring, and CAN ID"
            )
        if not left_ok or not right_ok:
            return

        logger.info("Both motors online — starting synchronized demo...")
        run_dual_motor_demo_can(motor_left, motor_right)
        logger.success("Dual-motor example completed!")

    finally:
        motor_left.close()
        motor_right.close()


if __name__ == "__main__":
    basic_can_example()
