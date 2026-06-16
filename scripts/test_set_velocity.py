#!/usr/bin/env python3
"""
This script tests the set_velocity() method of the CubeMarsAK606v3CAN motor class.
It spins the motor forward and backward at a specified velocity for a few seconds each,
with stops in between. This allows you to verify that the motor responds correctly to
velocity commands and that the direction changes as expected.
"""


import time
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN


def main():
    MOTOR_ID = 0x03
    VELOCITY_ERPM = 8000
    DURATION = 3  # seconds

    motor = CubeMarsAK606v3CAN(
        motor_can_id=MOTOR_ID,
        interface="can0",
        bitrate=1000000,
    )

    if not motor.connected:
        print("Motor not connected")
        return

    if not motor.check_communication():
        print("No communication with motor")
        motor.close()
        return

    print("Connected. Starting test...")

    try:
        # Forward spin
        print(f"Spinning +{VELOCITY_ERPM} ERPM")
        motor.set_velocity(VELOCITY_ERPM)
        time.sleep(DURATION)

        # Stop
        print("Stopping")
        motor.set_velocity(0)
        time.sleep(2)

        # Reverse spin
        print(f"Spinning -{VELOCITY_ERPM} ERPM")
        motor.set_velocity(-VELOCITY_ERPM)
        time.sleep(DURATION)

        # Stop again
        print("Final stop")
        motor.set_velocity(0)
        time.sleep(2)

        print("Test complete")

    except KeyboardInterrupt:
        print("Interrupted, stopping motor")
        motor.set_velocity(0)

    finally:
        motor.stop()
        motor.close()
        print("Motor safely closed")


if __name__ == "__main__":
    main()
