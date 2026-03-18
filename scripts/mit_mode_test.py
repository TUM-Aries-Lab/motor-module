#!/usr/bin/env python3
"""Bench test for AK60-6 MIT mode on a single motor (default CAN ID 0x03).

This script exercises all MIT-related public APIs in the current CAN driver:
- enable_mit_mode()
- set_mit_mode(...)
- disable_mit_mode()
- enable_motor()/disable_motor() aliases
- set_position(), set_velocity(), set_current(), stop()

Run:
    sudo ./setup_can.sh
    .venv/bin/python scripts/mit_mode_test.py
"""
# ruff: noqa: T201, PLR0915, S110

from __future__ import annotations

import argparse
import sys
import time

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

SEPARATOR = "=" * 72


def section(title: str) -> None:
    """Print a section header."""
    print(f"\n{SEPARATOR}")
    print(title)
    print(SEPARATOR)


def print_status(motor: CubeMarsAK606v3CAN, label: str, timeout: float = 0.4) -> None:
    """Print one status sample."""
    status = motor._receive_feedback(timeout=timeout)
    if status is None:
        status = motor.get_status()

    if status is None:
        print(f"  {label}: no feedback")
        return

    print(
        f"  {label}: pos={status.position_degrees:7.2f} deg | "
        f"vel={status.speed_erpm:7d} ERPM | "
        f"cur={status.current_amps:6.2f} A | "
        f"temp={status.temperature_celsius:3d} C | "
        f"err={status.error_code}"
    )


def hold_and_log(motor: CubeMarsAK606v3CAN, seconds: float, label: str) -> None:
    """Sleep for a short duration while printing live feedback."""
    end = time.time() + seconds
    while time.time() < end:
        print_status(motor, label)
        time.sleep(0.25)


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description="MIT mode hardware test for one CubeMars AK60-6 motor"
    )
    parser.add_argument(
        "--interface",
        default="can0",
        help="SocketCAN interface (default: can0)",
    )
    parser.add_argument(
        "--motor-id",
        type=lambda x: int(x, 0),
        default=0x03,
        help="Motor CAN ID (default: 0x03)",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=1_000_000,
        help="CAN bitrate (default: 1000000)",
    )
    parser.add_argument(
        "--position-deg",
        type=float,
        default=20.0,
        help="Position helper test target in degrees (default: 20)",
    )
    parser.add_argument(
        "--velocity-erpm",
        type=int,
        default=6000,
        help="Velocity helper test command in ERPM (default: 6000)",
    )
    parser.add_argument(
        "--torque-nm",
        type=float,
        default=1.0,
        help="Torque helper test command in Nm (default: 1.0)",
    )
    parser.add_argument(
        "--step-seconds",
        type=float,
        default=1.2,
        help="Duration per movement step (default: 1.2)",
    )
    return parser.parse_args()


def main() -> int:
    """Run MIT function tests on one motor."""
    args = parse_args()

    section("AK60-6 MIT Mode Test (single motor)")
    print(f"Interface : {args.interface}")
    print(f"Motor ID  : 0x{args.motor_id:02X}")
    print(f"Bitrate   : {args.bitrate}")
    print("Safety    : keep load clear; be ready to cut power")

    motor: CubeMarsAK606v3CAN | None = None
    try:
        motor = CubeMarsAK606v3CAN(
            motor_can_id=args.motor_id,
            interface=args.interface,
            bitrate=args.bitrate,
        )

        if not motor.connected:
            print("\nFAIL: CAN interface not connected")
            return 1

        section("1) check_communication()")
        if not motor.check_communication():
            print("FAIL: motor did not respond")
            return 1
        print("PASS: motor communication verified")

        section("2) enable_mit_mode()")
        motor.enable_mit_mode()
        hold_and_log(motor, 0.7, "after enable_mit_mode")

        section("3) set_mit_mode() direct calls")
        print("- passive float (all zeros)")
        motor.set_mit_mode(pos_rad=0.0, vel_rad_s=0.0, kp=0.0, kd=0.0, torque_ff_nm=0.0)
        hold_and_log(motor, args.step_seconds, "set_mit_mode passive")

        print("- position impedance")
        motor.set_mit_mode(
            pos_rad=0.30,
            vel_rad_s=0.0,
            kp=25.0,
            kd=1.0,
            torque_ff_nm=0.0,
        )
        hold_and_log(motor, args.step_seconds, "set_mit_mode position")

        print("- velocity damping")
        motor.set_mit_mode(
            pos_rad=0.0,
            vel_rad_s=3.0,
            kp=0.0,
            kd=2.0,
            torque_ff_nm=0.0,
        )
        hold_and_log(motor, args.step_seconds, "set_mit_mode velocity")

        print("- torque feedforward")
        motor.set_mit_mode(
            pos_rad=0.0,
            vel_rad_s=0.0,
            kp=0.0,
            kd=0.0,
            torque_ff_nm=args.torque_nm,
        )
        hold_and_log(motor, args.step_seconds, "set_mit_mode torque")

        section("4) set_position() helper (MIT-backed)")
        motor.set_position(args.position_deg)
        hold_and_log(motor, args.step_seconds, "set_position")

        section("5) set_velocity() helper (MIT-backed)")
        motor.set_velocity(args.velocity_erpm, allow_low_speed=True)
        hold_and_log(motor, args.step_seconds, "set_velocity")

        section("6) set_current() helper (maps to MIT torque)")
        motor.set_current(args.torque_nm)
        hold_and_log(motor, args.step_seconds, "set_current")

        section("7) stop()")
        motor.stop()
        hold_and_log(motor, 0.8, "after stop")

        section("8) enable_motor()/disable_motor() aliases")
        print("- enable_motor() alias")
        motor.enable_motor()
        hold_and_log(motor, 0.5, "after enable_motor alias")

        print("- disable_motor() alias")
        motor.disable_motor()
        hold_and_log(motor, 0.5, "after disable_motor alias")

        section("9) disable_mit_mode() explicit")
        motor.disable_mit_mode()
        print("PASS: MIT mode disabled")

        section("MIT test complete")
        print("PASS: all MIT-related API calls executed")
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
                pass
            try:
                motor.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
