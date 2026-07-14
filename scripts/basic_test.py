#!/usr/bin/env python3
# ruff: noqa: T201

"""
Simple velocity test for AK60-6 / AK80-6.
Testing the functionality of set_velocity().

Goal:
- test ONLY set_velocity()
- continuous update (important for MIT backend)
 .venv/bin/python scripts/test_test.py
"""

import time
from motor_python import create_can_motor
from scripts.motor_data_logger import MotorDataLogger

MOTOR_ID = 0x03
VELOCITY_ERPM = -3000
DURATION = 2.0


def main():
    logger = MotorDataLogger(
            f"data/logs/motor_log_{int(time.time())}.csv"
        )
    motor = create_can_motor(
        "AK80-6",
        motor_can_id=MOTOR_ID,
        interface="can0",
        bitrate=1_000_000,
    )

    try:
        print("Checking communication...")
        if not motor.check_communication():
            print("Communication failed")
            return

        print("Enabling motor...")
        motor.enable_motor()

        time.sleep(0.2)

        print(f"Spinning at {VELOCITY_ERPM} ERPM for {DURATION}s")

        motor.send_neutral_command()  # send neutral command to keep motor in MIT mode
        motor.set_velocity(VELOCITY_ERPM)
        time.sleep(DURATION)  # give the motor a moment to respond

        status = motor.get_status()
        logger.log(
                cmd_pos=0.0,
                cmd_vel=VELOCITY_ERPM,
                cmd_tau=0.0,
                act_pos=status.position_degrees,
                act_vel=status.speed_erpm,
                act_current=status.current_amps,
                temperature=status.temperature_celsius,
        )

        print("Stopping motor...")
        motor.set_velocity(0)
        time.sleep(0.2)
        motor.stop()

    finally:
        try:
            motor.stop()
        except Exception:
            pass

        try:
            motor.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
