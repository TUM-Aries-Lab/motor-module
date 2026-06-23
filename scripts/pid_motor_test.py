#!/usr/bin/env python3
"""PID position control test for CubeMars AK60-6 motor.


This script is intentionally simple and strict:
1. Bring CAN to a known-good state (optional auto reset).
2. Check communication and connect to the motor.
3. Use PID position control to move to a target angle for a fixed duration.
4. Always stop/disable motor on exit.

Example:

    sudo ./setup_can.sh
    .venv/bin/python scripts/pid_motor_test.py --target 120 --duration 3
"""

# ruff: noqa: T201

from __future__ import annotations

import argparse
import time

from loguru import logger

from motor_python import create_can_motor
from motor_python.motor_control_using_pid import PIDMotorController


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description="PID position control test for CubeMars AK60-6/AK80-6 motor",
    )

    parser.add_argument(
        "--target",
        type=float,
        default=90.0,
        help="Target position in degrees (default: 90)",
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="How long to run PID loop in seconds (default: 3.0)",
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
        "--rate-hz",
        type=int,
        default=100,
        help="PID update rate (default: 100 Hz)",
    )

    args = parser.parse_args()

    if args.duration <= 0:
        raise ValueError("--duration must be > 0")

    return args


def main() -> int:
    """Run PID position control test."""
    args = parse_args()

    print("AK60-6 PID Position Test")
    print("=" * 64)
    print(f"Interface : {args.interface}")
    print(f"Motor ID  : 0x{args.motor_id:02X}")
    print(f"Target    : {args.target} deg")
    print(f"Duration  : {args.duration:.2f} s")
    print(f"Rate      : {args.rate_hz} Hz")
    print("Safety    : keep load clear")

    motor = None

    try:
        motor = create_can_motor(
            args.motor_model,
            motor_can_id=args.motor_id,
            interface=args.interface,
            bitrate=args.bitrate,
        )

        if not motor.connected:
            print("FAIL: CAN connection failed")
            return 1

        print("Checking communication...")
        if not motor.check_communication():
            print("FAIL: motor communication failed")
            return 1


        # ---------------- PID SETUP ----------------

        controller = PIDMotorController(motor)

        # ---------------- CONTROL ----------------
        print("Running PID control...")
        controller.move_to(
            target_degrees=args.target,
            duration=args.duration,
            rate_hz=args.rate_hz,
            kp=8.0,
            kd=1.0,
        )

        print("Stopping motor...")
        motor.stop()

        print("PASS: PID test complete")
        return 0

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 130
    except Exception as exc:
        print(f"\nFAIL: {exc}")
        return 1

    finally:
        if motor is not None:
            try:
                motor.stop()
            except Exception:
                logger.debug("Failed to stop motor during cleanup, ignoring")
                pass
            try:
                motor.close()
            except Exception:
                logger.debug("Failed to close motor during cleanup, ignoring")
                pass


if __name__ == "__main__":
    raise SystemExit(main())
