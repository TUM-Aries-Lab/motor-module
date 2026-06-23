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

MOTOR_ID = 0x03
VELOCITY_ERPM = 5000
DURATION = 2.0


def main():
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

        t0 = time.time()

        while time.time() - t0 < DURATION:
            # IMPORTANT: continuous command (NOT one-shot)
            motor.set_velocity(VELOCITY_ERPM)

            pos = motor.get_position()
            vel = motor.get_speed()

            print(f"pos={pos:.2f} deg  vel={vel:.2f}")

            time.sleep(0.01)  # 100 Hz update

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
