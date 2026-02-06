"""Example usage functions for exosuit motor control."""

import time

from loguru import logger

from motor_python.cube_mars_motor import CubeMarsAK606v3


def run_velocity_control(motor: CubeMarsAK606v3, velocity_erpm: int = 8000) -> None:
    """Run velocity control with forward and reverse.

    :param motor: Motor controller instance
    :param velocity_erpm: Velocity in ERPM (positive=forward, negative=reverse)
    :return: None
    """
    logger.info("Forward velocity...")
    motor.set_velocity(velocity_erpm=velocity_erpm)
    time.sleep(0.5)
    motor.get_status()

    logger.info("Reverse velocity...")
    motor.set_velocity(velocity_erpm=-velocity_erpm)
    time.sleep(0.5)
    motor.get_status()

    logger.info("Stop...")
    motor.set_velocity(velocity_erpm=0)
    motor.get_status()


def run_max_rpm_test(motor: CubeMarsAK606v3, duration_seconds: float = 3.0) -> None:
    """Spin motor at maximum RPM.

    :param motor: Motor controller instance
    :param duration_seconds: Duration to run at max RPM (default: 3s)
    :return: None
    """
    max_erpm = 100000
    logger.info(f"Spinning at max RPM ({max_erpm} ERPM) for {duration_seconds}s...")

    motor.set_velocity(velocity_erpm=max_erpm)
    time.sleep(duration_seconds)

    logger.info("Stopping motor...")
    motor.set_velocity(velocity_erpm=0)
    time.sleep(0.5)
    motor.get_status()


def run_exosuit_tendon_control(motor: CubeMarsAK606v3) -> None:
    """Demonstrate exosuit tendon control.

    Recommended control pattern for spool-based tendon systems.

    :param motor: Motor controller instance
    :return: None
    """
    logger.info("Exosuit tendon control demo...")

    logger.info("Pull tendon (lift)")
    motor.control_exosuit_tendon(action="pull", velocity_erpm=12000)
    time.sleep(0.8)

    logger.info("Release tendon (lower)")
    motor.control_exosuit_tendon(action="release", velocity_erpm=8000)
    time.sleep(0.8)

    logger.info("Stop")
    motor.control_exosuit_tendon(action="stop")
    time.sleep(0.3)
    motor.get_status()


def run_motor_demo(motor: CubeMarsAK606v3) -> None:
    """Run motor control demonstration for exosuit.

    :param motor: Motor controller instance
    :return: None
    """
    logger.info("Starting exosuit motor demo...")
    logger.info("Press Ctrl+C to stop")

    try:
        # Demo 1: Basic velocity control
        logger.info("\n" + "=" * 60)
        logger.info(">>> RUNNING: run_velocity_control(8000 ERPM)")
        logger.info("=" * 60)
        run_velocity_control(motor, velocity_erpm=8000)
        motor.stop()
        time.sleep(2.0)

        # Demo 2: Exosuit tendon control
        logger.info("\n" + "=" * 60)
        logger.info(">>> RUNNING: run_exosuit_tendon_control()")
        logger.info("=" * 60)
        run_exosuit_tendon_control(motor)
        motor.stop()
        time.sleep(2.0)

        # Demo 3: Max RPM test
        logger.info("\n" + "=" * 60)
        logger.info(">>> RUNNING: run_max_rpm_test(2s)")
        logger.info("=" * 60)
        run_max_rpm_test(motor, duration_seconds=2.0)

    except KeyboardInterrupt:
        logger.info("Demo stopped by user")
        motor.stop()
