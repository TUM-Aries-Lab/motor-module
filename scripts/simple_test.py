#!/usr/bin/env python3
"""Minimal velocity spin test for one AK60-6 motor.

This script is intentionally tiny: set velocity, hold for duration, stop.

Examples:
    .venv/bin/python scripts/simple_test.py --velocity-erpm 4000 --duration 1.5
    .venv/bin/python scripts/simple_test.py --velocity-erpm -3000 --duration 2.0

"""
# ruff: noqa: T201

from __future__ import annotations

import argparse
import time

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments for the simple spin test."""
    parser = argparse.ArgumentParser(
        description="Simple velocity spin: command ERPM for N seconds, then stop."
    )
    parser.add_argument(
        "--velocity-erpm",
        type=int,
        default=3000,
        help="Target electrical RPM (negative = reverse, default: 3000)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=2.0,
        help="How long to hold velocity in seconds (default: 2.0)",
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
        "--skip-check",
        action="store_true",
        help="Skip check_communication() before spinning.",
    )
    args = parser.parse_args()

    if args.duration <= 0:
        raise ValueError("--duration must be > 0")
    if args.velocity_erpm == 0:
        raise ValueError("--velocity-erpm must be non-zero")

    return args


def main() -> int:
    """Run one simple velocity spin and stop."""
    args = parse_args()

    print("Simple AK60-6 Velocity Spin")
    print("=" * 64)
    print(f"Interface      : {args.interface}")
    print(f"Motor ID       : 0x{args.motor_id:02X}")
    print(f"Velocity       : {args.velocity_erpm} ERPM")
    print(f"Duration       : {args.duration:.2f} s")
    print("Safety         : keep load clear; be ready to cut power")

    motor: CubeMarsAK606v3CAN | None = None
    try:
        motor = CubeMarsAK606v3CAN(
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

        print("Sending velocity command...")
        motor.set_velocity(args.velocity_erpm)
        time.sleep(args.duration)

        print("Stopping motor...")
        motor.stop()
        print("PASS: spin finished")
        return 0

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
