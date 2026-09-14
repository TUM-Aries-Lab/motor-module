# ruff: noqa: T201
"""Spin the motor forward for 2 s, pause, then spin backward for 2 s.

Demonstrates CAN communication in both directions.

Run:
sudo ./setup_can.sh && .venv/bin/python scripts/spin_test.py
sudo ./setup_can.sh && .venv/bin/python scripts/spin_test_mit_mode.py --motor-model AK80-6

"""
import struct
import time
import argparse

import can
import numpy as np
from motor_python import create_can_motor
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN, CubeMarsAK806v2CAN, CubeMarsBaseCAN
from motor_python.definitions import MotorModel


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments for the simple spin test."""
    parser = argparse.ArgumentParser(
        description="Simple spin test: Spin the motor forward for 2 s, pause, then spin backward for 2 s.",
    )
    parser.add_argument(
        "--motor-id",
        type=lambda value: int(value, 0),
        default=0x03,
        help="Motor CAN ID in decimal or hex (default: 0x03)",
    )
    parser.add_argument(
        "--interface",
        default="can0",
        help="SocketCAN interface (default: can0)",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=1_000_000,
        help="CAN bitrate (default: 1000000)",
    )
    parser.add_argument(
        "--motor-model",
        choices=list(MotorModel),
        default=MotorModel.AK60_6V3,
        help="Motor model to instantiate (default: AK60-6)",
    )
    parser.add_argument(
        "--skip-check",
        action="store_true",
        help="Skip CAN communication check",
    )
    args = parser.parse_args()

    return args


DUTY      = 0.40      # 40% — adjust if you want more/less speed
SPIN_SECS = 1.0
PAUSE_SECS = 1

MAX_ERPM = 8000

def spin(motor: CubeMarsAK606v3CAN | CubeMarsAK806v2CAN, direction: int, duration: float):
    print(f"\nSpin {'FWD' if direction > 0 else 'REV'}")

    erpm = int(direction * DUTY * MAX_ERPM)
    erpm = int(np.clip(erpm, -MAX_ERPM, MAX_ERPM)) # Clip to motor limits

    motor.send_neutral_command()  # send neutral command to keep motor in MIT mode
    motor.set_velocity(erpm)
    time.sleep(duration)  # give the motor a moment to respond

    motor.stop()


def main() -> int:
    """Run simple spin forward, stop and spin reverse."""

    args = parse_args()

    print(f"Simple {args.motor_model} Velocity Spin")
    print("=" * 64)
    print(f"Interface      : {args.interface}")
    print(f"Motor model    : {args.motor_model}")
    print(f"Motor ID       : 0x{args.motor_id:02X}")
    print("Safety         : keep load clear; be ready to cut power")

    motor = None
    try:
        motor = create_can_motor(
            args.motor_model,
            motor_can_id=args.motor_id,
            interface=args.interface,
            bitrate=args.bitrate,
        )
        if not motor.connected:
            print("FAIL: could not connect to CAN bus")
            return 1

        if not args.skip_check and not motor.check_communication():
            print("FAIL: communication check failed")
            return 1


        print("Enabling motor...")
        motor.enable_motor()

        spin(motor, 1, SPIN_SECS)

        print(f"\nPause {PAUSE_SECS}s")
        time.sleep(PAUSE_SECS)

        spin(motor, -1, SPIN_SECS)

        print("\nDone")

    except KeyboardInterrupt:
        print("Interrupted by user")
        return 130
    except Exception as exc:
        print(f"FAIL: {exc}")
        return 1
    finally:
        if motor is not None:
            try:
                motor.stop()
            except Exception:
                pass
            try:
                motor.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
