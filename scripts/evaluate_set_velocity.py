#!/usr/bin/env python3
"""Evaluate `set_velocity()` for single or dual CAN motors.

This script mirrors the verification flow from `verify_set_velocity.py` but adds:
- single or dual motor selection via CLI args,
- motor-model agnostic instantiation,
- CSV logging with one row per feedback sample,
- RMSE reporting at the end.

Examples:
    .venv/bin/python scripts/evaluate_set_velocity.py --motor-id 0x03 --speed-erpm 3000 --motor-model AK60-6_V3.0
    .venv/bin/python scripts/evaluate_set_velocity.py --motor-ids 0x03,0x04 --speed-erpm 2500 --motor-model AK80-6

"""
# ruff: noqa: T201

from __future__ import annotations

import argparse
import csv
import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from loguru import logger

try:
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover
    plt = None

from motor_python import create_can_motor
from motor_python.base_motor import MotorState
from motor_python.can_utils import get_can_state, reset_can_interface
from motor_python.cube_mars_motor_can import CubeMarsBaseCAN
from motor_python.definitions import CAN_DEFAULTS, MotorModel

SEPARATOR = "=" * 78
HEALTHY_TX_ERR_MAX = 96
HEALTHY_RX_ERR_MAX = 64
VERIFY_VELOCITY_MIN_ERPM = -9000
VERIFY_VELOCITY_MAX_ERPM = 12000

CSV_FIELDNAMES = [
    "wall_time_iso",
    "wall_time_epoch_s",
    "elapsed_s",
    "phase_index",
    "phase_command_erpm",
    "phase_duration_s",
    "sample_index",
    "command_erpm",
    "left_motor_id",
    "left_motor_label",
    "left_feedback_position_deg",
    "left_feedback_speed_erpm",
    "left_feedback_current_amps",
    "left_feedback_temperature_c",
    "left_feedback_error_code",
    "left_feedback_error_description",
    "right_motor_id",
    "right_motor_label",
    "right_feedback_position_deg",
    "right_feedback_speed_erpm",
    "right_feedback_current_amps",
    "right_feedback_temperature_c",
    "right_feedback_error_code",
    "right_feedback_error_description",
    "sync_error_erpm",
]


def _resolve_csv_path(csv_path_arg: str | None, *, prefix: str) -> Path:
    """Resolve CSV output path from CLI args or timestamped default."""
    if csv_path_arg:
        return Path(csv_path_arg).expanduser().resolve()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return (Path("data/csv_logs") / f"{prefix}_{timestamp}.csv").resolve()


@dataclass(frozen=True)
class PhaseSummary:
    """Aggregate values for one commanded phase."""

    command_erpm: int
    samples_total: int
    mean_speed_erpm: float | None
    rmse_erpm: float | None
    sample_errors_erpm: list[float]
    pass_phase: bool


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments for the evaluation script."""
    parser = argparse.ArgumentParser(
        description="Evaluate CubeMars `set_velocity()` with single or dual motor support"
    )
    parser.add_argument("--interface", default="can0", help="SocketCAN interface")
    parser.add_argument(
        "--motor-id",
        type=lambda value: int(value, 0),
        default=CAN_DEFAULTS.motor_can_id,
        help="Single motor CAN ID in decimal or hex",
    )
    parser.add_argument(
        "--motor-ids",
        default=None,
        help="Comma-separated CAN IDs for dual-motor evaluation, e.g. 0x03,0x04",
    )
    parser.add_argument(
        "--motor-model",
        choices=list(MotorModel),
        default=MotorModel.AK60_6V3,
        help="Motor model to instantiate",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=CAN_DEFAULTS.bitrate,
        help="CAN bitrate (default: 1000000)",
    )
    parser.add_argument(
        "--helper-policy",
        choices=("strict", "fcfd", "legacy"),
        default="fcfd",
        help="MIT helper-frame policy (default: fcfd)",
    )
    parser.add_argument(
        "--feedback-can-id",
        type=lambda value: int(value, 0),
        default=None,
        help="Optional explicit feedback CAN ID override (hex or decimal)",
    )
    parser.add_argument(
        "--preflight-mode",
        choices=("strict", "auto", "skip"),
        default="auto",
        help="strict=fail on unhealthy CAN, auto=try reset, skip=ignore (default: auto)",
    )
    parser.add_argument(
        "--speed-erpm",
        type=int,
        required=True,
        help="Commanded set_velocity speed in ERPM",
    )
    parser.add_argument(
        "--phase-seconds",
        type=float,
        default=5.0,
        help="Duration of each signed velocity phase in seconds (default: 5.0)",
    )
    parser.add_argument(
        "--neutral-seconds",
        type=float,
        default=0.8,
        help="Duration of neutral phases between directions (default: 0.8)",
    )
    parser.add_argument(
        "--sample-hz",
        type=float,
        default=CAN_DEFAULTS.motor_control_rate_hz,
        help="Feedback sampling rate in Hz (default: 100.0)",
    )
    parser.add_argument(
        "--forward-only",
        action="store_true",
        help="Only evaluate the positive direction and neutral",
    )
    parser.add_argument(
        "--csv-path",
        default=None,
        help="Output CSV path for feedback logging",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    """Validate argument ranges."""
    if args.bitrate <= 0:
        raise ValueError("--bitrate must be > 0")
    if args.phase_seconds <= 0:
        raise ValueError("--phase-seconds must be > 0")
    if args.neutral_seconds < 0:
        raise ValueError("--neutral-seconds must be >= 0")
    if args.sample_hz <= 0:
        raise ValueError("--sample-hz must be > 0")
    if args.speed_erpm < VERIFY_VELOCITY_MIN_ERPM or args.speed_erpm > VERIFY_VELOCITY_MAX_ERPM:
        raise ValueError(
            f"--speed-erpm must be in [{VERIFY_VELOCITY_MIN_ERPM}, {VERIFY_VELOCITY_MAX_ERPM}]"
        )


def _is_can_state_healthy(state: dict[str, int | str]) -> bool:
    """Return True when CAN state is healthy enough for motor command testing."""
    return (
        state["state"] == "ERROR-ACTIVE"
        and int(state["tx_err"]) < HEALTHY_TX_ERR_MAX
        and int(state["rx_err"]) < HEALTHY_RX_ERR_MAX
    )


def ensure_can_ready(interface: str, bitrate: int, *, mode: str) -> None:
    """Verify or recover CAN state before commanding motion."""
    state = get_can_state(interface)
    print(f"CAN preflight: state={state['state']} tx_err={state['tx_err']} rx_err={state['rx_err']}")
    if _is_can_state_healthy(state):
        return
    if mode == "skip":
        print("CAN preflight skipped by request.")
        return
    if mode == "strict":
        raise RuntimeError("CAN interface unhealthy. Run `sudo ./setup_can.sh` and retry.")

    print("CAN preflight: attempting automatic kernel-level CAN reset ...")
    if not reset_can_interface(interface=interface, bitrate=bitrate):
        raise RuntimeError("Auto preflight reset failed. Run `sudo ./setup_can.sh` manually.")

    after = get_can_state(interface)
    print(
        f"CAN preflight after reset: state={after['state']} tx_err={after['tx_err']} rx_err={after['rx_err']}"
    )
    if not _is_can_state_healthy(after):
        raise RuntimeError("CAN still unhealthy after reset. Check wiring/power/UART disconnect.")


def _parse_motor_ids(motor_id: int, motor_ids_arg: str | None) -> list[tuple[int, str]]:
    """Build a label list for single or dual-motor evaluation."""
    if motor_ids_arg:
        parsed_ids = [int(value, 0) for value in motor_ids_arg.split(",") if value.strip()]
        if len(parsed_ids) not in {1, 2}:
            raise ValueError("--motor-ids must contain 1 or 2 CAN IDs separated by commas")
        labels = ["left" if idx == 0 else "right" for idx in range(len(parsed_ids))]
        return [(parsed_id, label) for parsed_id, label in zip(parsed_ids, labels, strict=True)]

    return [(motor_id, "single")]


def _command_phase(motor: CubeMarsBaseCAN, command_erpm: int) -> None:
    """Send one velocity command phase."""
    if command_erpm == 0:
        motor.set_mit_mode(pos_rad=0.0, vel_rad_s=0.0, kp=0.0, kd=2.0, torque_ff_nm=0.0)
        return
    motor.set_velocity(command_erpm)


def _read_status(motor: CubeMarsBaseCAN, timeout: float):
    """Read freshest available feedback without long blocking."""
    status = motor._receive_feedback(timeout=timeout)
    return status if status is not None else motor._last_feedback


def _resolve_plot_path(csv_path: Path) -> Path:
    """Return a PNG path that matches the CSV path, replacing the extension with .png."""
    if csv_path.suffix.lower() == ".csv":
        return csv_path.with_suffix(".png")
    return csv_path.with_suffix(csv_path.suffix + ".png")


def plot_velocity_feedback(csv_path: Path, plot_path: Path | None = None) -> Path:
    """Build and save a PNG plot from the velocity evaluator CSV output."""
    if plot_path is None:
        plot_path = _resolve_plot_path(csv_path)

    if plt is None:
        raise RuntimeError(
            "matplotlib is required to generate plots. Install it and retry."
        )

    elapsed: list[float] = []
    command_erpm: list[float] = []
    left_speed: list[float] = []
    right_speed: list[float] = []

    with csv_path.open("r", encoding="utf-8", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            if not row:
                continue
            elapsed_s = row.get("elapsed_s", "")
            command_value = row.get("command_erpm", "")
            left_value = row.get("left_feedback_speed_erpm", "")
            right_value = row.get("right_feedback_speed_erpm", "")
            if command_value == "":
                continue
            try:
                elapsed.append(float(elapsed_s) if elapsed_s else float(len(elapsed)))
            except ValueError:
                elapsed.append(float(len(elapsed)))
            command_erpm.append(float(command_value))
            left_speed.append(float(left_value) if left_value else float("nan"))
            right_speed.append(float(right_value) if right_value else float("nan"))

    if not command_erpm:
        raise RuntimeError("No velocity rows found in CSV to plot.")

    plt.figure(figsize=(12, 6))
    x_values = elapsed
    plt.plot(x_values, command_erpm, label="cmd_erpm", color="#1f77b4", linewidth=1.5)
    if any(not math.isnan(value) for value in left_speed):
        plt.plot(x_values, left_speed, label="left_feedback_speed_erpm", color="#ff7f0e", linewidth=1.5)
    if any(not math.isnan(value) for value in right_speed):
        plt.plot(x_values, right_speed, label="right_feedback_speed_erpm", color="#2ca02c", linewidth=1.5)

    plt.xlabel("Elapsed time (s)")
    plt.ylabel("Speed (ERPM)")
    plt.title("Velocity command vs left/right motor feedback")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(plot_path, dpi=150)
    plt.close()
    return plot_path


def _rmse(values: list[float]) -> float | None:
    """Compute RMSE for a list of error values."""
    if not values:
        return None
    return math.sqrt(sum(value * value for value in values) / len(values))


def run_synchronized_phase(  # noqa: C901, PLR0913
    motors: list[CubeMarsBaseCAN],
    motor_entries: list[tuple[int, str]],
    *,
    phase_index: int,
    command_erpm: int,
    duration_s: float,
    sample_hz: float,
    run_start_monotonic: float,
    sample_logger: Callable[[dict[str, str | int]], None] | None = None,
) -> list[PhaseSummary]:
    """Evaluate one synchronized phase and log both motors in one CSV row per sample."""
    for motor in motors:
        _command_phase(motor, command_erpm)

    period_s = 1.0 / sample_hz
    phase_start = time.monotonic()
    t_end = phase_start + duration_s
    sample_index = 0
    last_feedback_ts = [float(getattr(motor, "_last_feedback_monotonic", 0.0)) for motor in motors]
    missed = [0] * len(motors)
    per_motor_errors: list[list[float]] = [[] for _ in motors]
    per_motor_speeds: list[list[int]] = [[] for _ in motors]

    while time.monotonic() < t_end:
        statuses: list[MotorState | None] = []
        fresh_cycle = True
        for motor_index, motor in enumerate(motors):
            transport_fault = getattr(motor, "_transport_fault", None)
            if transport_fault is not None:
                raise RuntimeError(
                    f"Motor 0x{motor_entries[motor_index][0]:02X} transport fault during cmd={command_erpm:+d}: {transport_fault}"
                )

            status = _read_status(motor, timeout=min(0.08, period_s))
            feedback_ts = float(getattr(motor, "_last_feedback_monotonic", 0.0))
            fresh_feedback = status is not None and feedback_ts > last_feedback_ts[motor_index]
            if not fresh_feedback:
                missed[motor_index] += 1
                if missed[motor_index] > 10:
                    raise RuntimeError(
                        f"Motor 0x{motor_entries[motor_index][0]:02X} missed feedback during cmd={command_erpm:+d}"
                    )
                fresh_cycle = False
                statuses.append(None)
                continue

            last_feedback_ts[motor_index] = feedback_ts
            missed[motor_index] = 0
            assert status is not None
            if status.error_code != 0:
                raise RuntimeError(
                    f"Motor 0x{motor_entries[motor_index][0]:02X} fault during cmd={command_erpm:+d}: "
                    f"{status.error_code} ({status.error_description})"
                )
            statuses.append(status)

        if not fresh_cycle:
            time.sleep(period_s)
            continue

        sample_index += 1
        for motor_index, status in enumerate(statuses):
            if status is None:
                continue
            per_motor_errors[motor_index].append(float(status.speed_erpm - command_erpm))
            per_motor_speeds[motor_index].append(status.speed_erpm)

        if sample_logger is not None:
            now_epoch = time.time()
            sample_logger(
                {
                    "wall_time_iso": datetime.fromtimestamp(now_epoch).isoformat(
                        timespec="milliseconds"
                    ),
                    "wall_time_epoch_s": f"{now_epoch:.6f}",
                    "elapsed_s": f"{(time.monotonic() - run_start_monotonic):.6f}",
                    "phase_index": phase_index,
                    "phase_command_erpm": command_erpm,
                    "phase_duration_s": f"{duration_s:.6f}",
                    "sample_index": sample_index,
                    "command_erpm": command_erpm,
                    "left_motor_id": f"0x{motor_entries[0][0]:02X}" if len(motor_entries) > 0 else "",
                    "left_motor_label": motor_entries[0][1] if len(motor_entries) > 0 else "",
                    "left_feedback_position_deg": f"{statuses[0].position_degrees:.6f}" if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_feedback_speed_erpm": statuses[0].speed_erpm if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_feedback_current_amps": f"{statuses[0].current_amps:.6f}" if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_feedback_temperature_c": statuses[0].temperature_celsius if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_feedback_error_code": statuses[0].error_code if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_feedback_error_description": statuses[0].error_description if len(statuses) > 0 and statuses[0] is not None else "",
                    "right_motor_id": f"0x{motor_entries[1][0]:02X}" if len(motor_entries) > 1 else "",
                    "right_motor_label": motor_entries[1][1] if len(motor_entries) > 1 else "",
                    "right_feedback_position_deg": f"{statuses[1].position_degrees:.6f}" if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_feedback_speed_erpm": statuses[1].speed_erpm if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_feedback_current_amps": f"{statuses[1].current_amps:.6f}" if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_feedback_temperature_c": statuses[1].temperature_celsius if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_feedback_error_code": statuses[1].error_code if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_feedback_error_description": statuses[1].error_description if len(statuses) > 1 and statuses[1] is not None else "",
                    "sync_error_erpm": "" if len(statuses) < 2 or statuses[0] is None or statuses[1] is None else f"{statuses[0].speed_erpm - statuses[1].speed_erpm:.2f}",
                }
            )

        time.sleep(period_s)

    summaries: list[PhaseSummary] = []
    for motor_index, (motor_id, label) in enumerate(motor_entries):
        speeds = per_motor_speeds[motor_index]
        errors = per_motor_errors[motor_index]
        mean_speed = sum(speeds) / len(speeds) if speeds else None
        summaries.append(
            PhaseSummary(
                command_erpm=command_erpm,
                samples_total=len(speeds),
                mean_speed_erpm=mean_speed,
                rmse_erpm=_rmse(errors),
                sample_errors_erpm=errors,
                pass_phase=(mean_speed is not None and abs(mean_speed - command_erpm) <= 500),
            )
        )
        print_phase_summary(motor_id, label, summaries[-1])

    return summaries


def print_phase_summary(motor_id: int, motor_label: str, phase_summary: PhaseSummary) -> None:
    """Print one formatted phase summary line."""
    rmse = phase_summary.rmse_erpm if phase_summary.rmse_erpm is not None else float("nan")
    mean = phase_summary.mean_speed_erpm if phase_summary.mean_speed_erpm is not None else 0.0
    print(
        f"Motor 0x{motor_id:02X} [{motor_label}] | "
        f"cmd={phase_summary.command_erpm:+7d} ERPM | "
        f"samples={phase_summary.samples_total:3d} | "
        f"mean={mean:8.1f} ERPM | "
        f"RMSE={rmse:8.1f} ERPM | "
        f"{'PASS' if phase_summary.pass_phase else 'FAIL'}"
    )


def main() -> int:  # noqa: C901, PLR0912, PLR0915
    """Run the evaluation for one or two motors and report total RMSE."""
    args = parse_args()
    validate_args(args)
    csv_path = _resolve_csv_path(args.csv_path, prefix="evaluate_set_velocity")

    print(SEPARATOR)
    print("Evaluate set_velocity()")
    print(SEPARATOR)
    print(f"Interface         : {args.interface}")
    print(f"Bitrate           : {args.bitrate}")
    print(f"Motor model       : {args.motor_model}")
    print(f"Speed command     : {args.speed_erpm:+d} ERPM")
    print(f"Phase seconds     : {args.phase_seconds:.2f}")
    print(f"Neutral seconds   : {args.neutral_seconds:.2f}")
    print(f"Sample rate       : {args.sample_hz:.1f} Hz")
    print(f"Forward only      : {args.forward_only}")
    print(f"CSV log           : {csv_path}")
    print(SEPARATOR)

    ensure_can_ready(args.interface, bitrate=args.bitrate, mode=args.preflight_mode)

    motor_ids = _parse_motor_ids(args.motor_id, args.motor_ids)
    motors: list[CubeMarsBaseCAN] = []
    try:
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
            csv_writer.writeheader()

            def write_sample_row(row: dict[str, str | int]) -> None:
                csv_writer.writerow(row)
                csv_file.flush()

            for motor_id, _label in motor_ids:
                motor = create_can_motor(
                    args.motor_model,
                    motor_can_id=motor_id,
                    interface=args.interface,
                    bitrate=args.bitrate,
                    helper_policy=args.helper_policy,
                    feedback_can_id=args.feedback_can_id,
                )
                motors.append(motor)
                if not motor.connected:
                    print(f"FAIL: could not connect to CAN motor 0x{motor_id:02X}")
                    return 1
                motor.send_neutral_command()
                if not motor.check_communication():
                    print(f"FAIL: communication check failed for motor 0x{motor_id:02X}")
                    return 1
                print(f"PASS: communication verified on motor 0x{motor_id:02X}")

            all_errors: list[float] = []
            phase_sequence: list[tuple[int, float]] = []
            base_speed = int(args.speed_erpm)
            run_start_monotonic = time.monotonic()
            phase_sequence.append((base_speed, args.phase_seconds))
            if args.neutral_seconds > 0:
                phase_sequence.append((0, args.neutral_seconds))
            if not args.forward_only:
                phase_sequence.append((-base_speed, args.phase_seconds))
                if args.neutral_seconds > 0:
                    phase_sequence.append((0, args.neutral_seconds))

            for phase_index, (command_erpm, duration_s) in enumerate(phase_sequence, start=1):
                print(f"\nPhase {phase_index}: cmd={command_erpm:+d} ERPM for {duration_s:.2f} s")
                phase_summaries = run_synchronized_phase(
                    motors,
                    motor_ids,
                    phase_index=phase_index,
                    command_erpm=command_erpm,
                    duration_s=duration_s,
                    sample_hz=args.sample_hz,
                    run_start_monotonic=run_start_monotonic,
                    sample_logger=write_sample_row,
                )
                for phase_summary in phase_summaries:
                    if phase_summary.sample_errors_erpm:
                        all_errors.extend(phase_summary.sample_errors_erpm)

            total_rmse = _rmse(all_errors)
            print(f"\n{SEPARATOR}")
            if total_rmse is None:
                print("FAIL: no feedback samples were recorded")
                return 1

            print(f"Overall RMSE: {total_rmse:.3f} ERPM")
            try:
                plot_path = plot_velocity_feedback(csv_path)
                print(f"Plot saved to: {plot_path}")
            except Exception as exc:
                logger.warning(f"Velocity plot not generated: {exc}")

            print("PASS: set_velocity evaluation finished")
            print(f"CSV saved to: {csv_path}")
            return 0
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 130
    except Exception as exc:
        print(f"\nFAIL: {exc}")
        return 1
    finally:
        for motor in motors:
            try:
                motor.stop()
            except Exception as exc:
                logger.warning(f"motor.stop() cleanup failed: {exc}")
            try:
                motor.close()
            except Exception as exc:
                logger.warning(f"motor.close() cleanup failed: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
