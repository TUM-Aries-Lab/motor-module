"""

Dual-motor velocity-tracking test for CAN-controlled motors.

This script commands two motors with the same sinusoidal velocity profile and
records their feedback so you can evaluate how closely they follow the target
velocity and how well they stay synchronized with each other over time.
It is useful for comparing left/right motor response, speed tracking error,
and current draw during coordinated motion.

Typical use:
    sudo ./setup_can.sh
    .venv/bin/python scripts/dual_motor_velocity_test.py \
        --left-id 0x03 --right-id 0x04 \
        --left-motor-model AK80-6 --right-motor-model AK80-6 \
        --amplitude-erpm 4000 --control-hz 0.3 --duration 30
"""

# ruff: noqa: T201
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import NamedTuple


if __package__ in {None, ""}:
    repo_src = Path(__file__).resolve().parents[1] / "src"
    if str(repo_src) not in sys.path:
        sys.path.insert(0, str(repo_src))

from motor_python import create_can_motor
from motor_python.base_motor import MotorState, print_timing_stats
from motor_python.can_utils import get_can_state, reset_can_interface
from motor_python.cube_mars_motor_can import CubeMarsBaseCAN
from motor_python.definitions import CAN_DEFAULTS, MotorModel

SEPARATOR = "=" * 72
HEALTHY_TX_ERR_MAX = 96
HEALTHY_RX_ERR_MAX = 64
SYNC_THRESHOLD_ERPM = 500
MIT_VELOCITY_LIMIT_ERPM = 8000

CSV_FIELDNAMES = [
    "wall_time_iso",
    "wall_time_epoch_s",
    "elapsed_s",
    "sample_index",
    "commanded_velocity_erpm",
    # Left motor
    "left_feedback_received",
    "left_position_deg",
    "left_speed_erpm",
    "left_current_amps",
    "left_temperature_c",
    "left_error_code",
    "left_error_description",
    # Right motor
    "right_feedback_received",
    "right_position_deg",
    "right_speed_erpm",
    "right_current_amps",
    "right_temperature_c",
    "right_error_code",
    "right_error_description",
    # Derived
    "left_velocity_error_erpm",
    "right_velocity_error_erpm",
    "sync_error_erpm",
    "current_asymmetry_amps",
]


class VelocitySample(NamedTuple):
    """Container for one dual-motor velocity sample."""

    commanded_velocity_erpm: int
    left: MotorState | None
    right: MotorState | None

    @property
    def left_velocity_error_erpm(self) -> float | None:
        if self.left is None:
            return None
        return self.left.speed_erpm - self.commanded_velocity_erpm

    @property
    def right_velocity_error_erpm(self) -> float | None:
        if self.right is None:
            return None
        return self.right.speed_erpm - self.commanded_velocity_erpm

    @property
    def sync_error_erpm(self) -> float | None:
        if self.left is None or self.right is None:
            return None
        return float(self.left.speed_erpm - self.right.speed_erpm)

    @property
    def current_asymmetry_amps(self) -> float | None:
        if self.left is None or self.right is None:
            return None
        return self.left.current_amps - self.right.current_amps


def _is_can_healthy(state: dict[str, int | str]) -> bool:
    return (
        state["state"] == "ERROR-ACTIVE"
        and int(state["tx_err"]) < HEALTHY_TX_ERR_MAX
        and int(state["rx_err"]) < HEALTHY_RX_ERR_MAX
    )


def _ensure_can_ready(interface: str, bitrate: int) -> None:
    state = get_can_state(interface)
    if _is_can_healthy(state):
        return
    print(f"CAN preflight: state={state['state']} tx={state['tx_err']} rx={state['rx_err']}")
    print("CAN preflight: attempting automatic reset …")
    if not reset_can_interface(interface=interface, bitrate=bitrate):
        raise RuntimeError("CAN reset failed. Run `sudo ./setup_can.sh` manually.")
    after = get_can_state(interface)
    if not _is_can_healthy(after):
        raise RuntimeError("CAN still unhealthy after reset. Check wiring/termination.")


def read_status(motor: CubeMarsBaseCAN, timeout_s: float = 0.1) -> MotorState | None:
    """Read motor status with a timeout."""
    status = motor._receive_feedback(timeout=timeout_s)
    if status is not None:
        return status
    return motor.get_status()


def _check_fault_code(sample: VelocitySample) -> None:
    for label, status in (("LEFT", sample.left), ("RIGHT", sample.right)):
        if status is None:
            continue
        if status.error_code != 0:
            raise RuntimeError(
                f"{label} motor fault: code={status.error_code} desc={status.error_description}"
            )


def _clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp a value to a range."""
    return max(min_value, min(max_value, value))

def _write_csv_row(
    writer: csv.DictWriter,
    csv_file,
    *,
    run_start_time: float,
    sample_index: int,
    sample: VelocitySample,
) -> None:
    now = time.time()
    left, right = sample.left, sample.right
    row = {
        "wall_time_iso": datetime.fromtimestamp(now).isoformat(),
        "wall_time_epoch_s": f"{now:.6f}",
        "elapsed_s": f"{time.monotonic() - run_start_time:.6f}",
        "sample_index": sample_index,
        "commanded_velocity_erpm": sample.commanded_velocity_erpm,
        # Left motor
        "left_feedback_received": int(left is not None),
        "left_position_deg": "" if left is None else f"{left.position_degrees:.4f}",
        "left_speed_erpm": "" if left is None else left.speed_erpm,
        "left_current_amps": "" if left is None else f"{left.current_amps:.4f}",
        "left_temperature_c": "" if left is None else left.temperature_celsius,
        "left_error_code": "" if left is None else left.error_code,
        "left_error_description": "" if left is None else left.error_description,
        # Right motor
        "right_feedback_received": int(right is not None),
        "right_position_deg": "" if right is None else f"{right.position_degrees:.4f}",
        "right_speed_erpm": "" if right is None else right.speed_erpm,
        "right_current_amps": "" if right is None else f"{right.current_amps:.4f}",
        "right_temperature_c": "" if right is None else right.temperature_celsius,
        "right_error_code": "" if right is None else right.error_code,
        "right_error_description": "" if right is None else right.error_description,
        # Derived
        "left_velocity_error_erpm": "" if sample.left_velocity_error_erpm is None else f"{sample.left_velocity_error_erpm:.2f}",
        "right_velocity_error_erpm": "" if sample.right_velocity_error_erpm is None else f"{sample.right_velocity_error_erpm:.2f}",
        "sync_error_erpm": "" if sample.sync_error_erpm is None else f"{sample.sync_error_erpm:.2f}",
        "current_asymmetry_amps": "" if sample.current_asymmetry_amps is None else f"{sample.current_asymmetry_amps:.4f}",
    }
    writer.writerow(row)
    csv_file.flush()


def _print_sample_line(elapsed_s: float, sample_index: int, sample: VelocitySample) -> None:
    left, right = sample.left, sample.right
    sync_str = f"{sample.sync_error_erpm:+.0f}" if sample.sync_error_erpm is not None else "n/a"
    warn = " ⚠ SYNC" if (
        sample.sync_error_erpm is not None and abs(sample.sync_error_erpm) > SYNC_THRESHOLD_ERPM
    ) else ""
    l_vel = f"{left.speed_erpm:+7d}" if left is not None else "    n/a"
    r_vel = f"{right.speed_erpm:+7d}" if right is not None else "    n/a"
    print(
        f"t={elapsed_s:6.2f}s  i={sample_index:05d}  "
        f"cmd={sample.commanded_velocity_erpm:+8d} ERPM  "
        f"L={l_vel} ERPM  R={r_vel} ERPM  "
        f"sync={sync_str} ERPM{warn}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dual-motor velocity test for CAN motors")
    parser.add_argument("--interface", type=str, default=CAN_DEFAULTS.interface)
    parser.add_argument("--bitrate", type=int, default=CAN_DEFAULTS.bitrate)
    parser.add_argument("--left-id", type=lambda v: int(v, 0), default=CAN_DEFAULTS.motor_can_id)
    parser.add_argument("--right-id", type=lambda v: int(v, 0), default=CAN_DEFAULTS.motor_can_id_2)
    parser.add_argument(
        "--left-motor-model",
        choices=list(MotorModel),
        default=MotorModel.AK60_6V3,
    )
    parser.add_argument(
        "--right-motor-model",
        choices=list(MotorModel),
        default=MotorModel.AK60_6V3,
    )
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--phase-seconds", type=float, default=3.0)
    parser.add_argument("--amplitude-erpm", type=float, default=4000.0)
    parser.add_argument("--freq-hz", type=float, default=0.5)
    parser.add_argument("--control-hz", type=float, default=CAN_DEFAULTS.motor_control_rate_hz)
    parser.add_argument("--helper-policy", choices=["strict", "fcfd", "legacy"], default="fcfd")
    parser.add_argument("--skip-preflight", action="store_true")
    parser.add_argument("--csv-path", default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.left_id == args.right_id:
        print("Error: Left and right motor IDs must be different.")
        return 1
    if args.amplitude_erpm <= 0:
        print("Error: Amplitude must be positive.")
        return 1
    if args.freq_hz <= 0 or args.control_hz <= 0 or args.duration <= 0 or args.phase_seconds <= 0:
        print("Error: Frequency, control rate, and duration must be positive.")
        return 1

    period_s = 1.0 / args.control_hz
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = (
        Path(args.csv_path)
        if args.csv_path is not None
        else Path("data/csv_logs") / f"dual_velocity_{timestamp}.csv"
    )

    print(SEPARATOR)
    print(
        f"Dual-Motor Velocity Test — {args.left_motor_model} / {args.right_motor_model}  "
        f"(L=0x{args.left_id:02X}  R=0x{args.right_id:02X})"
    )
    print(SEPARATOR)
    print(f"Interface    : {args.interface}")
    print(f"Amplitude    : ±{args.amplitude_erpm:.0f} ERPM")
    print(f"Frequency    : {args.freq_hz:.2f} Hz")
    print(f"Duration     : {args.duration:.1f} s")
    print(f"Control rate : {args.control_hz:.1f} Hz")
    print(f"CSV log      : {csv_path}")
    print("Safety       : keep load clear; be ready to cut power")
    print(SEPARATOR)

    motor_left: CubeMarsBaseCAN | None = None
    motor_right: CubeMarsBaseCAN | None = None
    csv_file = None
    csv_writer: csv.DictWriter | None = None
    run_start = 0.0
    total_samples = 0
    sync_errors: list[float] = []

    try:
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = csv_path.open("w", newline="", encoding="utf-8")
        csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
        csv_writer.writeheader()

        if args.skip_preflight:
            state = get_can_state(interface=args.interface)
            print("Skipping CAN preflight check (user override).")
        else:
            _ensure_can_ready(interface=args.interface, bitrate=args.bitrate)

        print("Initializing left motor …")
        motor_left = create_can_motor(
            args.left_motor_model,
            motor_can_id=args.left_id,
            interface=args.interface,
            bitrate=args.bitrate,
            helper_policy=args.helper_policy,
        )
        print("Initializing right motor …")
        motor_right = create_can_motor(
            args.right_motor_model,
            motor_can_id=args.right_id,
            interface=args.interface,
            bitrate=args.bitrate,
            helper_policy=args.helper_policy,
        )

        if not motor_left.connected or not motor_right.connected:
            print("Error: Could not connect to one or both motors.")
            return 1

        print("Check Communication - Left Motor")
        if not motor_left.check_communication():
            print("Error: Left motor communication check failed.")
            return 1
        print("Left motor communication check passed.")
        print("Check Communication - Right Motor")
        if not motor_right.check_communication():
            print("Error: Right motor communication check failed.")
            return 1
        print("Right motor communication check passed.")

        motor_left.send_neutral_command()
        motor_right.send_neutral_command()

        print("\nStarting varying velocity sweep …")
        run_start = time.monotonic()
        deadline = run_start + args.duration
        sample_index = 0
        next_tick = run_start
        previous_command = 0
        max_step_erpm = 500   # maximum ERPM change per loop

        feedback_window_s = max(0.05, min(0.25, period_s))
        feedback_interval_s = max(0.01, min(0.02, feedback_window_s / 10.0))

        while time.monotonic() < deadline and previous_command < args.amplitude_erpm:
            elapsed_s = time.monotonic() - run_start
            target_erpm = previous_command + max_step_erpm
            print(f"Target ERPM={target_erpm} erpm…")

            commanded_erpm = _clamp(target_erpm, -MIT_VELOCITY_LIMIT_ERPM, MIT_VELOCITY_LIMIT_ERPM)
            previous_command = commanded_erpm
            print(f"t={elapsed_s:.2f}s  cmd={commanded_erpm:+.0f} ERPM  sampling feedback …")

            motor_left.set_velocity(commanded_erpm)
            motor_right.set_velocity(commanded_erpm)
            time.sleep(args.phase_seconds)

            sample_deadline = time.monotonic() + feedback_window_s
            while time.monotonic() < sample_deadline:
                left_status = read_status(motor_left, timeout_s=min(0.1, period_s))
                right_status = read_status(motor_right, timeout_s=min(0.1, period_s))
                sample = VelocitySample(
                    commanded_velocity_erpm=commanded_erpm,
                    left=left_status,
                    right=right_status,
                )

                _check_fault_code(sample)

                if csv_writer is not None:
                    _write_csv_row(
                        writer=csv_writer,
                        csv_file=csv_file,
                        run_start_time=run_start,
                        sample_index=sample_index,
                        sample=sample,
                    )

                if sample.sync_error_erpm is not None:
                    sync_errors.append(abs(sample.sync_error_erpm))

                total_samples += int(sample.left is not None) + int(sample.right is not None)

                if sample_index % 10 == 0:
                    _print_sample_line(elapsed_s, sample_index, sample)

                sample_index += 1

                if time.monotonic() < sample_deadline:
                    time.sleep(feedback_interval_s)

            next_tick += period_s
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

        print(SEPARATOR)
        print("Test complete.")
        print(f"Total samples collected: {total_samples}")
        if sync_errors:
            print(
                f"Sync error: min={min(sync_errors):.2f} ERPM  "
                f"max={max(sync_errors):.2f} ERPM  "
                f"mean={sum(sync_errors)/len(sync_errors):.2f} ERPM"
            )
        print(f"CSV log saved to: {csv_path}")
        return 0

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 130
    except Exception as exc:
        print(f"\nFAIL: {exc}")
        return 1
    finally:
        for label, motor in (("LEFT", motor_left), ("RIGHT", motor_right)):
            if motor is not None:
                try:
                    motor.set_velocity(0)
                    time.sleep(0.05)
                    # motor.send_neutral_command()
                    time.sleep(0.05)
                except Exception:
                    pass
                try:
                    print_timing_stats(motor.get_timing_stats(), total_samples, SEPARATOR)
                except Exception:
                    pass

        if csv_file is not None:
            try:
                csv_file.close()
            except Exception:
                pass

        for motor in (motor_left, motor_right):
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
