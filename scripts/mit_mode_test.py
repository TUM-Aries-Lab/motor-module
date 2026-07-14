#!/usr/bin/env python3
"""Bench test for AK60-6 and AK80-6 MIT mode on a single motor (default CAN ID 0x03).

This script exercises all MIT-related public APIs in the current CAN driver:
- enable_mit_mode()
- set_mit_mode(...)
- disable_mit_mode()
- enable_motor()/disable_motor() aliases
- set_position(), set_velocity(), set_current(), stop()

Run:
    sudo ./setup_can.sh
    .venv/bin/python scripts/mit_mode_test.py

    sudo ./setup_can.sh
    .venv/bin/python scripts/mit_mode_test.py --motor-model AK80-6 --include-spin-tests

    IMPORTANT FOR SAKSHI: The parts where torque is commanded (torque feedforward and set_current) are commented out to avoid unexpected fast spinning. If motor spins too fast, it is made to stop immediately. Thats what happens in these 2 parts
    When I changed torque to 0.2, it is better now.
    """
# ruff: noqa: T201, PLR0915, S110

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from motor_python import create_can_motor
from motor_python.base_motor import MotorState
from motor_python.can_utils import get_can_state
from motor_python.cube_mars_motor_can import CubeMarsBaseCAN
from motor_python.definitions import MotorModel

SEPARATOR = "=" * 72
CSV_FIELDNAMES = [
    "wall_time_iso",
    "wall_time_epoch_s",
    "elapsed_s",
    "step",
    "command_mode",
    "command_pos_rad",
    "command_vel_rad_s",
    "command_kp",
    "command_kd",
    "command_torque_ff_nm",
    "command_position_deg",
    "command_velocity_erpm",
    "command_current_amps",
    "feedback_position_deg",
    "feedback_speed_erpm",
    "feedback_current_amps",
    "feedback_temperature_c",
    "feedback_error_code",
    "feedback_error_description",
]


def _resolve_csv_path(csv_path_arg: str | None) -> Path:
    """Resolve CSV path from CLI arg or generate a timestamped default path."""
    if csv_path_arg:
        return Path(csv_path_arg).expanduser().resolve()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return (Path("data/csv_logs") / f"mit_mode_test_{timestamp}.csv").resolve()


class MITStepCSVLogger:
    """Write MIT command/feedback rows to a CSV file."""

    def __init__(self, csv_path_arg: str | None) -> None:
        self.path = _resolve_csv_path(csv_path_arg)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=CSV_FIELDNAMES)
        self.writer.writeheader()
        self.start_time = time.time()

    def log(
        self,
        step: str,
        command_mode: str,
        command_values: dict[str, object],
        status: MotorState | None,
    ) -> None:
        """Write one command/feedback row to the CSV file."""
        row = {
            "wall_time_iso": datetime.now().isoformat(),
            "wall_time_epoch_s": time.time(),
            "elapsed_s": time.time() - self.start_time,
            "step": step,
            "command_mode": command_mode,
            "command_pos_rad": command_values.get("pos_rad", ""),
            "command_vel_rad_s": command_values.get("vel_rad_s", ""),
            "command_kp": command_values.get("kp", ""),
            "command_kd": command_values.get("kd", ""),
            "command_torque_ff_nm": command_values.get("torque_ff_nm", ""),
            "command_position_deg": command_values.get("position_deg", ""),
            "command_velocity_erpm": command_values.get("velocity_erpm", ""),
            "command_current_amps": command_values.get("current_amps", ""),
            "feedback_position_deg": "" if status is None else status.position_degrees,
            "feedback_speed_erpm": "" if status is None else status.speed_erpm,
            "feedback_current_amps": "" if status is None else status.current_amps,
            "feedback_temperature_c": "" if status is None else status.temperature_celsius,
            "feedback_error_code": "" if status is None else status.error_code,
            "feedback_error_description": "" if status is None else status.error_description,
        }
        self.writer.writerow(row)
        self.file.flush()

    def close(self) -> None:
        self.file.close()


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
    motor: CubeMarsBaseCAN , label: str, timeout: float = 0.4
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
    if abs(status.speed_erpm) >25000:
        print(" FAIL: motor speed too high, stopping motor for safety")
        motor.stop()
        hold_and_log(motor, 0.8, "after stop")
        raise RuntimeError(f"Motor speed too high: {status.speed_erpm} ERPM")
    return status


def hold_and_log(motor: CubeMarsBaseCAN, seconds: float, label: str) -> None:
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
    print(f"  {label}: hold complete ({seconds:.2f} s)")


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
        default=50.0,
        help="Position helper test target in degrees (default: 10)",
    )
    parser.add_argument(
        "--velocity-erpm",
        type=int,
        default=5000,
        help="Velocity helper test command in ERPM (default: 3000)",
    )
    parser.add_argument(
        "--torque-nm",
        type=float,
        default=0.2,
        help="Torque helper test command in Nm (default: 0.5)",
    )
    parser.add_argument(
        "--step-seconds",
        type=float,
        default=2.0,
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
    parser.add_argument(
        "--motor-model",
        choices=list(MotorModel),
        default=MotorModel.AK60_6V3,
        help="Motor model to instantiate (default: AK60-6)",
    )
    parser.add_argument(
        "--csv-path",
        default=None,
        help="Output CSV path for command/feedback logging",
    )
    return parser.parse_args()


def main() -> int:  # noqa: C901, PLR0912
    """Run MIT function tests on one motor."""
    args = parse_args()
    logger = MITStepCSVLogger(args.csv_path)

    section("AK60-6 MIT Mode Test (single motor)")
    if args.safe:
        args.position_deg = max(-8.0, min(8.0, args.position_deg))
        args.velocity_erpm = max(-2500, min(2500, args.velocity_erpm))
        args.torque_nm = max(-0.3, min(0.3, args.torque_nm))
        args.step_seconds = max(0.3, min(0.6, args.step_seconds))

    print(f"Interface : {args.interface}")
    print(f"Motor model: {args.motor_model}")
    print(f"Motor ID  : 0x{args.motor_id:02X}")
    print(f"Bitrate   : {args.bitrate}")
    print(f"Safe mode : {args.safe}")
    print(f"Preflight : {'skip' if args.skip_preflight else 'auto-reset if needed'}")
    print(f"Spin tests: {args.include_spin_tests}")
    print(f"CSV log   : {logger.path}")
    print("Safety    : keep load clear; be ready to cut power")
    if args.skip_preflight:
        state = get_can_state(args.interface)
        print(
            "CAN preflight skipped: "
            f"state={state['state']} tx_err={state['tx_err']} rx_err={state['rx_err']}"
        )
    else:
        _ensure_can_ready(args.interface)

    motor = None
    try:
        motor = create_can_motor(
            args.motor_model,
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
            try:
                motor.disable_mit_mode()
            except Exception:
                pass
            return 1
        print("PASS: motor communication verified")

        motor.send_neutral_command()  # send neutral command to keep motor in MIT mode

        section("2) enable_mit_mode()")
        motor.enable_mit_mode()
        hold_and_log(motor, 0.7, "after enable_mit_mode")
        logger.log("enable_mit_mode", "enable_mit_mode", {}, motor.get_status())

        section("3) set_mit_mode() direct calls")
        print("- passive float (all zeros)")
        motor.zero_position()
        motor.set_mit_mode(pos_rad=0.0, vel_rad_s=0.0, kp=0.0, kd=0.0, torque_ff_nm=0.0)
        hold_and_log(motor, args.step_seconds, "set_mit_mode passive")
        logger.log(
            "set_mit_mode_passive",
            "set_mit_mode",
            {"pos_rad": 0.0, "vel_rad_s": 0.0, "kp": 0.0, "kd": 0.0, "torque_ff_nm": 0.0},
            motor.get_status(),
        )

        print("- position impedance")
        motor.zero_position()  # zero the encoder before position test
        motor.set_mit_mode(
            pos_rad=1.0,
            vel_rad_s=0.0,
            kp=0.5 if args.safe else 4,
            kd=0.2 if args.safe else 0.5,
            torque_ff_nm=0.0,
        )
        hold_and_log(motor, args.step_seconds, "set_mit_mode position")
        logger.log(
            "set_mit_mode_position",
            "set_mit_mode",
            {
                "pos_rad": 1.0,
                "vel_rad_s": 0.0,
                "kp": 0.5 if args.safe else 4,
                "kd": 0.2 if args.safe else 0.5,
                "torque_ff_nm": 0.0,
            },
            motor.get_status(),
        )

        if args.include_spin_tests:
            print("- velocity damping")
            motor.zero_position()
            motor.set_mit_mode(
                pos_rad=0.0,
                vel_rad_s=1.2 if args.safe else 2.0,
                kp=0.0,
                kd=1.0 if args.safe else 2.0,
                torque_ff_nm=0.0,
            )
            hold_and_log(motor, args.step_seconds, "set_mit_mode velocity")
            logger.log(
                "set_mit_mode_velocity",
                "set_mit_mode",
                {
                    "pos_rad": 0.0,
                    "vel_rad_s": 1.2 if args.safe else 2.0,
                    "kp": 0.0,
                    "kd": 1.0 if args.safe else 2.0,
                    "torque_ff_nm": 0.0,
                },
                motor.get_status(),
            )


        # TODO: discuss if this is correct, because the motor spins extremely fast when we command a torque (even 0.5 Nm).
            print("- torque feedforward")
            motor.set_mit_mode(
                pos_rad=0.0,
                vel_rad_s=0.0,
                kp=0.0,
                kd=0.0,
                torque_ff_nm=args.torque_nm,
            )
            hold_and_log(motor, args.step_seconds/2, "set_mit_mode torque")
            logger.log(
                "set_mit_mode_torque",
                "set_mit_mode",
                {
                    "pos_rad": 0.0,
                    "vel_rad_s": 0.0,
                    "kp": 0.0,
                    "kd": 0.0,
                    "torque_ff_nm": args.torque_nm,
                },
                motor.get_status(),
            )
        else:
            print("- velocity/torque MIT subtests skipped (use --include-spin-tests)")

        section("4) set_position() helper (MIT-backed)")
        motor.zero_position()  # zero the encoder before position test
        motor.set_position(args.position_deg)
        hold_and_log(motor, args.step_seconds, "set_position")
        logger.log(
            "set_position",
            "set_position",
            {"position_deg": args.position_deg},
            motor.get_status(),
        )

        if args.include_spin_tests:
            section("5) set_velocity() helper (MIT-backed)")
            motor.set_velocity(args.velocity_erpm)
            hold_and_log(motor, args.step_seconds, "set_velocity")
            logger.log(
                "set_velocity",
                "set_velocity",
                {"velocity_erpm": args.velocity_erpm},
                motor.get_status(),
            )

            # TODO: discuss if this is correct, because the motor spins extremely fast when we command a torque (even 0.5 Nm).
            section("6) set_current() helper (maps to MIT torque)")
            motor.set_current(args.torque_nm)
            hold_and_log(motor, args.step_seconds, "set_current")
            logger.log(
                "set_current",
                "set_current",
                {"current_amps": args.torque_nm},
                motor.get_status(),
            )
        else:
            section("5/6) spin-prone sections skipped")
            print("Skipped set_velocity()/set_current(). Use --include-spin-tests to run them.")

        section("7) stop()")
        motor.stop()
        hold_and_log(motor, 0.8, "after stop")
        logger.log("stop", "stop", {}, motor.get_status())

        section("8) enable_motor()/disable_motor() aliases")
        print("- enable_motor() alias")
        motor.enable_motor()
        hold_and_log(motor, 0.5, "after enable_motor alias")
        logger.log("enable_motor", "enable_motor", {}, motor.get_status())

        print("- disable_motor() alias")
        motor.disable_motor()
        hold_and_log(motor, 0.5, "after disable_motor alias")
        logger.log("disable_motor", "disable_motor", {}, motor.get_status())

        section("9) disable_mit_mode() explicit")
        motor.disable_mit_mode()
        logger.log("disable_mit_mode", "disable_mit_mode", {}, motor.get_status())
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
        if logger is not None:
            try:
                logger.close()
            except Exception:
                pass
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
