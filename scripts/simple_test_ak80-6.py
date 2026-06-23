#!/usr/bin/env python3
# ruff: noqa: T201
"""
    Only for AK80-6: Spin the motor forward for a few seconds.
    .venv/bin/python scripts/simple_test_ak80-6.py
"""


import time

from motor_python import create_can_motor

MOTOR_ID = 0x03
TORQUE = 1.0      # Nm
DURATION = 1.0    # seconds


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

        print(f"Applying {TORQUE} Nm torque for {DURATION}s")

        t0 = time.time()

        while time.time() - t0 < DURATION:
            motor.set_mit_mode(
                pos_rad=0.0,
                vel_rad_s=2.0,
                kp=0.0,
                kd=0.2,
                torque_ff_nm=3,
            )

            pos = motor.get_position()
            vel = motor.get_speed()

            print(
                f"pos={pos:.3f} "
                f"vel={vel:.3f}"
            )

            time.sleep(0.02)

        print("Stopping...")
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
