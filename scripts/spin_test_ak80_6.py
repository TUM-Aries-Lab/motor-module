# ruff: noqa: T201
"""Spin the motor forward for 2 s, pause, then spin backward for 2 s.

Demonstrates CAN communication in both directions.
Run:
sudo ./setup_can.sh && .venv/bin/python scripts/spin_test.py
sudo ./setup_can.sh && .venv/bin/python scripts/spin_test_ak80_6.py --motor-model AK80-6

"""
import struct
import time
import argparse

import can
import numpy as np
from motor_python import create_can_motor


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
        choices=("AK60-6", "AK80-6"),
        default="AK60-6",
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
PAUSE_SECS = 0.1

MAX_ERPM = 8000

def spin(motor, direction: int, duration: float):
    print(f"\nSpin {'FWD' if direction > 0 else 'REV'}")

    erpm = int(direction * DUTY * MAX_ERPM)
    erpm = int(np.clip(erpm, -MAX_ERPM, MAX_ERPM)) # Clip to motor limits

    t0 = time.time()

    while time.time() - t0 < duration:
        # TODO: Both are working right now, check which one makes sense to use here
        # motor.set_mit_mode(
        #     pos_rad=0.0,
        #     vel_rad_s=2.0 * direction,
        #     kp=0.5,
        #     kd=0.2,
        #     torque_ff_nm=3.0 * direction
        # )
        motor.set_velocity(erpm*direction)

        pos = motor.get_position()
        vel = motor.get_speed()

        if pos is not None:
            print(f"pos={pos:.2f} deg  vel={vel:.2f}")

        time.sleep(0.02)

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
