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
import subprocess
import sys
import time

from motor_python.base_motor import MotorState
from motor_python.can_utils import get_can_state
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

SEPARATOR = "=" * 72


def _ensure_can_ready(interface: str) -> None:
    """Attempt automatic recovery when CAN interface is not ERROR-ACTIVE."""
    state = get_can_state(interface)
    bad_state = state["state"] != "ERROR-ACTIVE"
    high_rx = int(state["rx_err"]) >= 64

    if not bad_state and not high_rx:
        return

    print(
        f"CAN preflight: state={state['state']} tx_err={state['tx_err']} rx_err={state['rx_err']}"
    )
    print("CAN preflight: attempting automatic reset via sudo ./setup_can.sh ...")

    reset = subprocess.run(["sudo", "./setup_can.sh"], check=False)
    if reset.returncode != 0:
        raise RuntimeError(
            "CAN preflight reset failed. Run `sudo ./setup_can.sh` manually and retry."
        )

    after = get_can_state(interface)
    print(
        f"CAN preflight after reset: state={after['state']} tx_err={after['tx_err']} rx_err={after['rx_err']}"
    )

    if after["state"] != "ERROR-ACTIVE" or int(after["rx_err"]) >= 64:
        raise RuntimeError(
            "CAN bus still unhealthy after reset (error frames dominate). "
            "Check wiring/termination/power/UART disconnect before retrying."
        )


def section(title: str) -> None:
    """Print a section header."""
    print(f"\n{SEPARATOR}")
    print(title)
    print(SEPARATOR)


def print_status(
    motor: CubeMarsAK606v3CAN, label: str, timeout: float = 0.4
) -> MotorState | None:
    """Print and return one status sample."""
    status = motor._receive_feedback(timeout=timeout)
    if status is None:
        status = motor.get_status()

    if status is None:
        print(f"  {label}: no feedback")
        return None

    print(
        f"  {label}: pos={status.position_degrees:7.2f} deg | "
        f"vel={status.speed_erpm:7d} ERPM | "
        f"cur={status.current_amps:6.2f} A | "
        f"temp={status.temperature_celsius:3d} C | "
        f"err={status.error_code} ({status.error_description})"
    )
    return status


def hold_and_log(motor: CubeMarsAK606v3CAN, seconds: float, label: str) -> None:
    """Sleep for a short duration while printing live feedback and validating health."""
    end = time.time() + seconds
    no_feedback_count = 0
    while time.time() < end:
        status = print_status(motor, label)
        if status is None:
            no_feedback_count += 1
            if no_feedback_count >= 3:
                raise RuntimeError("No CAN feedback for three consecutive samples")
        else:
            no_feedback_count = 0
            if status.error_code != 0:
                raise RuntimeError(
                    f"Motor fault code {status.error_code}: {status.error_description}"
                )
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
        default=10.0,
        help="Position helper test target in degrees (default: 10)",
    )
    parser.add_argument(
        "--velocity-erpm",
        type=int,
        default=3000,
        help="Velocity helper test command in ERPM (default: 3000)",
    )
    parser.add_argument(
        "--torque-nm",
        type=float,
        default=0.5,
        help="Torque helper test command in Nm (default: 0.5)",
    )
    parser.add_argument(
        "--step-seconds",
        type=float,
        default=0.8,
        help="Duration per movement step (default: 0.8)",
    )
    parser.add_argument(
        "--safe",
        action="store_true",
        help="Extra conservative profile (reduced command magnitudes)",
    )
    parser.add_argument(
        "--skip-preflight",
        action="store_true",
        help="Skip automatic `sudo ./setup_can.sh` preflight reset.",
    )
    parser.add_argument(
        "--include-spin-tests",
        action="store_true",
        help=(
            "Include set_velocity() and set_current() sections. "
            "Disabled by default to avoid unexpected spinning."
        ),
    )
    return parser.parse_args()


def main() -> int:  # noqa: C901, PLR0912
    """Run MIT function tests on one motor."""
    args = parse_args()

    section("AK60-6 MIT Mode Test (single motor)")
    if args.safe:
        args.position_deg = max(-8.0, min(8.0, args.position_deg))
        args.velocity_erpm = max(-2500, min(2500, args.velocity_erpm))
        args.torque_nm = max(-0.3, min(0.3, args.torque_nm))
        args.step_seconds = max(0.3, min(0.6, args.step_seconds))

    print(f"Interface : {args.interface}")
    print(f"Motor ID  : 0x{args.motor_id:02X}")
    print(f"Bitrate   : {args.bitrate}")
    print(f"Safe mode : {args.safe}")
    print(f"Preflight : {'skip' if args.skip_preflight else 'auto-reset if needed'}")
    print(f"Spin tests: {args.include_spin_tests}")
    print("Safety    : keep load clear; be ready to cut power")
    if args.skip_preflight:
        state = get_can_state(args.interface)
        print(
            "CAN preflight skipped: "
            f"state={state['state']} tx_err={state['tx_err']} rx_err={state['rx_err']}"
        )
    else:
        _ensure_can_ready(args.interface)

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
            pos_rad=0.12 if args.safe else 0.30,
            vel_rad_s=0.0,
            kp=10.0 if args.safe else 25.0,
            kd=0.8 if args.safe else 1.0,
            torque_ff_nm=0.0,
        )
        hold_and_log(motor, args.step_seconds, "set_mit_mode position")

        if args.include_spin_tests:
            print("- velocity damping")
            motor.set_mit_mode(
                pos_rad=0.0,
                vel_rad_s=1.2 if args.safe else 3.0,
                kp=0.0,
                kd=1.0 if args.safe else 2.0,
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
        else:
            print("- velocity/torque MIT subtests skipped (use --include-spin-tests)")

        section("4) set_position() helper (MIT-backed)")
        motor.set_position(args.position_deg)
        hold_and_log(motor, args.step_seconds, "set_position")

        if args.include_spin_tests:
            section("5) set_velocity() helper (MIT-backed)")
            motor.set_velocity(args.velocity_erpm, allow_low_speed=True)
            hold_and_log(motor, args.step_seconds, "set_velocity")

            section("6) set_current() helper (maps to MIT torque)")
            motor.set_current(args.torque_nm)
            hold_and_log(motor, args.step_seconds, "set_current")
        else:
            section("5/6) spin-prone sections skipped")
            print("Skipped set_velocity()/set_current(). Use --include-spin-tests to run them.")

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
