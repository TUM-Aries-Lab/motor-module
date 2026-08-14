#!/usr/bin/env python3
"""Evaluate `set_position()` for single or dual CAN motors.

This script mirrors the ping-pong command pattern used by
`mit_position_steps.py`, but it is geared toward evaluation:
- choose a single motor or two motors from the CLI,
- command `+position_deg` and `-position_deg` repeatedly,
- log all feedback samples into a CSV,
- calculate and print RMSE at the end.

Examples:
    sudo ./setup_can.sh
    .venv/bin/python scripts/evaluate_set_position.py --motor-ids 0x02,0x01 --position-deg 45 --velocity-deg-s 25 --motor-model AK60-6_V1.1
    .venv/bin/python scripts/evaluate_set_position.py --motor-ids 0x03,0x04 --position-deg 30 --velocity-deg-s 20 --motor-model AK60-6_V3.0
    .venv/bin/python scripts/evaluate_set_position.py --motor-id 0x04 --position-deg 30 --velocity-deg-s 90 --motor-model AK80-6

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
from motor_python.base_motor import MotorState, print_timing_stats
from motor_python.can_utils import get_can_state, reset_can_interface
from motor_python.cube_mars_motor_can import CubeMarsBaseCAN
from motor_python.definitions import CAN_DEFAULTS, MotorModel, MotorSpec

SEPARATOR = "=" * 78
HEALTHY_TX_ERR_MAX = 96
HEALTHY_RX_ERR_MAX = 64
MIT_POSITION_LIMIT_DEG = math.degrees(12.56)
MIN_TRAVEL_EPS_DEG = 0.5

CSV_FIELDNAMES = [
    "wall_time_iso",
    "wall_time_epoch_s",
    "elapsed_s",
    "phase_index",
    "phase_command_position_deg",
    "phase_duration_s",
    "sample_index",
    "command_position_deg",
    "left_motor_id",
    "left_motor_label",
    "left_feedback_position_deg",
    "left_feedback_speed_erpm",
    "left_feedback_current_amps",
    "left_feedback_temperature_c",
    "left_feedback_error_code",
    "left_feedback_error_description",
    "left_is_fresh_feedback",
    "right_motor_id",
    "right_motor_label",
    "right_feedback_position_deg",
    "right_feedback_speed_erpm",
    "right_feedback_current_amps",
    "right_feedback_temperature_c",
    "right_feedback_error_code",
    "right_feedback_error_description",
    "right_is_fresh_feedback",
    "sync_error_deg",
]


@dataclass(frozen=True)
class PhaseSummary:
    """Aggregate values for a commanded position phase."""

    command_position_deg: float
    samples_total: int
    mean_position_deg: float | None
    rmse_deg: float | None
    sample_errors_deg: list[float]


def _resolve_csv_path(csv_path_arg: str | None, *, prefix: str) -> Path:
    """Resolve CSV output path from CLI args or timestamped default."""
    if csv_path_arg:
        return Path(csv_path_arg).expanduser().resolve()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return (Path("data/csv_logs") / f"{prefix}_{timestamp}_{CAN_DEFAULTS.motor_control_rate_hz}.csv").resolve()


def _clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp a value into the inclusive range [min, max]."""
    return max(min_value, min(max_value, value))


def _is_can_state_healthy(state: dict[str, int | str]) -> bool:
    """Return True when CAN state is healthy enough for controlled test runs."""
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


def _read_status(motor: CubeMarsBaseCAN, timeout: float = 0.10) -> MotorState | None:
    """Read the freshest available motor status."""
    status = motor._receive_feedback(timeout=timeout)
    if status is None:
        return motor._last_feedback
    return status


def _rmse(values: list[float]) -> float | None:
    """Return RMSE for a list of error values or None for empty input."""
    if not values:
        return None
    return math.sqrt(sum(value * value for value in values) / len(values))


def _resolve_plot_path(csv_path: Path) -> Path:
    """Return a PNG path that matches the CSV path, replacing the extension with .png."""
    if csv_path.suffix.lower() == ".csv":
        return csv_path.with_suffix(".png")
    return csv_path.with_suffix(csv_path.suffix + ".png")


def plot_position_feedback(csv_path: Path, plot_path: Path | None = None) -> Path:
    """Build and save a PNG plot from the position evaluator CSV output."""
    if plot_path is None:
        plot_path = _resolve_plot_path(csv_path)

    if plt is None:
        raise RuntimeError(
            "matplotlib is required to generate plots. Install it and retry."
        )

    elapsed: list[float] = []
    command_deg: list[float] = []
    left_deg: list[float] = []
    right_deg: list[float] = []

    with csv_path.open("r", encoding="utf-8", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            if not row:
                continue
            elapsed_s = row.get("elapsed_s", "")
            command_value = row.get("command_position_deg", "")
            left_value = row.get("left_feedback_position_deg", "")
            right_value = row.get("right_feedback_position_deg", "")
            if command_value == "":
                continue
            try:
                elapsed.append(float(elapsed_s) if elapsed_s else float(len(elapsed)))
            except ValueError:
                elapsed.append(float(len(elapsed)))
            command_deg.append(float(command_value))
            left_deg.append(float(left_value) if left_value else float("nan"))
            right_deg.append(float(right_value) if right_value else float("nan"))

    if not command_deg:
        raise RuntimeError("No position rows found in CSV to plot.")

    plt.figure(figsize=(12, 6))
    x_values = elapsed
    plt.plot(x_values, command_deg, label="cmd_deg", color="#1f77b4", linewidth=1.5)
    if any(not math.isnan(value) for value in left_deg):
        plt.plot(x_values, left_deg, label="left_feedback_deg", color="#ff7f0e", linewidth=1.5)
    if any(not math.isnan(value) for value in right_deg):
        plt.plot(x_values, right_deg, label="right_feedback_deg", color="#2ca02c", linewidth=1.5)

    plt.xlabel("Elapsed time (s)")
    plt.ylabel("Position (deg)")
    plt.title("Position command vs left/right motor feedback")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(plot_path, dpi=150)
    plt.close()
    return plot_path

def calculate_average_position_delay(
    command_history: list[tuple[float, float]],
    feedback_history: list[tuple[float, float]],
) -> float | None:
    """
    Estimate average delay between commanded and measured position.
    """

    if len(command_history) < 2 or len(feedback_history) < 2:
        return None

    delays = []

    command_times = [x[0] for x in command_history]
    command_positions = [x[1] for x in command_history]

    for feedback_time, feedback_position in feedback_history:

        closest_index = min(
            range(len(command_positions)),
            key=lambda i: abs(command_positions[i] - feedback_position)
        )

        command_time = command_times[closest_index]

        delay = feedback_time - command_time

        if delay >= 0:
            delays.append(delay)

    if not delays:
        return None

    return sum(delays) / len(delays)

def run_synchronized_phase(  # noqa: C901, PLR0912, PLR0913, PLR0915
    motors: list[CubeMarsBaseCAN],
    motor_entries: list[tuple[int, str]],
    *,
    phase_index: int,
    command_position_deg: float,
    velocity_deg_s: float,
    hold_seconds: float,
    control_hz: float,
    sample_hz: float,
    run_start: float,
    start_position_deg: float = 0.0,
    sample_logger: Callable[[dict[str, str | int]], None] | None = None,
) -> list[PhaseSummary]:
    """Command one position target to both motors in sync and log paired rows."""

    ramp_distance_deg = abs(command_position_deg - start_position_deg)
    segment_time = ramp_distance_deg / max(velocity_deg_s, 1e-9)
    phase_duration_s = segment_time + hold_seconds
    sample_period_s = 1.0 / sample_hz
    move_steps = max(1, round(segment_time * control_hz))
    move_period_s = segment_time / move_steps
    deadline = time.monotonic() + phase_duration_s
    sample_index = 0
    last_feedback_ts = [float(getattr(motor, "_last_feedback_monotonic", 0.0)) for motor in motors]
    missed = [0] * len(motors)
    per_motor_errors: list[list[float]] = [[] for _ in motors]
    per_motor_positions: list[list[float]] = [[] for _ in motors]
    per_motor_feedback_history: list[list[tuple[float, float]]] = [[] for _ in motors]
    command_history: list[tuple[float, float]] = []
    command_timestamps = []
    feedback_latencies = [[] for _ in motors]


    move_start = time.monotonic()
    move_end = move_start + segment_time
    while time.monotonic() < move_end:
        if time.monotonic() >= deadline:
            break

        elapsed_move = time.monotonic() - move_start
        u = _clamp(elapsed_move / max(segment_time, 1e-9), 0.0, 1.0)
        smooth_u = 0.5 * (1.0 - math.cos(math.pi * u))

        # Ramp from start_position_deg -> command_position_deg, not 0 -> command_position_deg.
        cmd_deg = start_position_deg + (command_position_deg - start_position_deg) * smooth_u

        command_history.append(   # store commanded position for logging
            (time.monotonic(), cmd_deg)
        )
        for motor in motors:
            motor.set_position(cmd_deg)

        statuses: list[MotorState | None] = []
        fresh_cycle = True
        for motor_index, motor in enumerate(motors):
            transport_fault = getattr(motor, "_transport_fault", None)
            if transport_fault is not None:
                raise RuntimeError(
                    f"Motor 0x{motor_entries[motor_index][0]:02X} transport fault during cmd={command_position_deg:+.2f} deg: {transport_fault}"
                )

            status = _read_status(motor, timeout=min(0.08, sample_period_s))
            if status is not None:
                feedback_time = status.timestamp_monotonic

                if command_history:
                    latest_command_time = command_history[-1][0]

                    latency = feedback_time - latest_command_time

                    feedback_latencies[motor_index].append(latency)
            feedback_ts = float(getattr(motor, "_last_feedback_monotonic", 0.0))
            fresh_feedback = status is not None and feedback_ts > last_feedback_ts[motor_index]
            if not fresh_feedback:
                missed[motor_index] += 1
                if missed[motor_index] > 10:
                    raise RuntimeError(
                        f"Motor 0x{motor_entries[motor_index][0]:02X} missed feedback while moving to {command_position_deg:.2f} deg"
                    )
                fresh_cycle = False
                statuses.append(None)
                continue

            last_feedback_ts[motor_index] = feedback_ts
            missed[motor_index] = 0
            assert status is not None
            if status.error_code != 0:
                raise RuntimeError(
                    f"Motor 0x{motor_entries[motor_index][0]:02X} fault during move to {command_position_deg:.2f} deg: "
                    f"{status.error_code} ({status.error_description})"
                )
            statuses.append(status)

        if not fresh_cycle:
            time.sleep(sample_period_s)
            continue

        sample_index += 1
        for motor_index, status in enumerate(statuses):
            if status is None:
                continue
            per_motor_errors[motor_index].append(float(status.position_degrees - command_position_deg))
            per_motor_positions[motor_index].append(status.position_degrees)
            per_motor_feedback_history[motor_index].append((time.monotonic(),status.position_degrees,))

        if sample_logger is not None:
            now_epoch = time.time()
            sample_logger(
                {
                    "wall_time_iso": datetime.fromtimestamp(now_epoch).isoformat(
                        timespec="milliseconds"
                    ),
                    "wall_time_epoch_s": f"{now_epoch:.6f}",
                    "elapsed_s": f"{(time.monotonic() - run_start):.6f}",
                    "phase_index": phase_index,
                    "phase_command_position_deg": f"{command_position_deg:.6f}",
                    "phase_duration_s": f"{phase_duration_s:.6f}",
                    "sample_index": sample_index,
                    "command_position_deg": f"{cmd_deg:.6f}",
                    "left_motor_id": f"0x{motor_entries[0][0]:02X}" if len(motor_entries) > 0 else "",
                    "left_motor_label": motor_entries[0][1] if len(motor_entries) > 0 else "",
                    "left_feedback_position_deg": f"{statuses[0].position_degrees:.6f}" if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_feedback_speed_erpm": statuses[0].speed_erpm if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_feedback_current_amps": f"{statuses[0].current_amps:.6f}" if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_feedback_temperature_c": statuses[0].temperature_celsius if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_feedback_error_code": statuses[0].error_code if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_feedback_error_description": statuses[0].error_description if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_is_fresh_feedback": statuses[0].is_fresh if len(statuses) > 0 and statuses[0] is not None else "",
                    "right_motor_id": f"0x{motor_entries[1][0]:02X}" if len(motor_entries) > 1 else "",
                    "right_motor_label": motor_entries[1][1] if len(motor_entries) > 1 else "",
                    "right_feedback_position_deg": f"{statuses[1].position_degrees:.6f}" if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_feedback_speed_erpm": statuses[1].speed_erpm if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_feedback_current_amps": f"{statuses[1].current_amps:.6f}" if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_feedback_temperature_c": statuses[1].temperature_celsius if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_feedback_error_code": statuses[1].error_code if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_feedback_error_description": statuses[1].error_description if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_is_fresh_feedback": statuses[1].is_fresh if len(statuses) > 1 and statuses[1] is not None else "",
                    "sync_error_deg": "" if len(statuses) < 2 or statuses[0] is None or statuses[1] is None else f"{statuses[0].position_degrees - statuses[1].position_degrees:.2f}",
                }
            )

        time.sleep(move_period_s)

    hold_deadline = time.monotonic() + hold_seconds
    while time.monotonic() < hold_deadline:
        for motor in motors:
            motor.set_position(command_position_deg)

        statuses: list[MotorState | None] = []
        fresh_cycle = True
        for motor_index, motor in enumerate(motors):
            status = _read_status(motor, timeout=min(0.08, sample_period_s))
            feedback_ts = float(getattr(motor, "_last_feedback_monotonic", 0.0))
            fresh_feedback = status is not None and feedback_ts > last_feedback_ts[motor_index]
            if not fresh_feedback:
                missed[motor_index] += 1
                if missed[motor_index] > 10:
                    raise RuntimeError(
                        f"Motor 0x{motor_entries[motor_index][0]:02X} missed feedback while holding {command_position_deg:.2f} deg"
                    )
                fresh_cycle = False
                statuses.append(None)
                continue

            last_feedback_ts[motor_index] = feedback_ts
            missed[motor_index] = 0
            assert status is not None
            if status.error_code != 0:
                raise RuntimeError(
                    f"Motor 0x{motor_entries[motor_index][0]:02X} fault during hold at {command_position_deg:.2f} deg: "
                    f"{status.error_code} ({status.error_description})"
                )
            statuses.append(status)

        if not fresh_cycle:
            time.sleep(sample_period_s)
            continue

        sample_index += 1
        for motor_index, status in enumerate(statuses):
            if status is None:
                continue
            per_motor_errors[motor_index].append(float(status.position_degrees - cmd_deg))
            per_motor_positions[motor_index].append(status.position_degrees)
            per_motor_feedback_history[motor_index].append((time.monotonic(),status.position_degrees,))

        if sample_logger is not None:
            now_epoch = time.time()
            sample_logger(
                {
                    "wall_time_iso": datetime.fromtimestamp(now_epoch).isoformat(
                        timespec="milliseconds"
                    ),
                    "wall_time_epoch_s": f"{now_epoch:.6f}",
                    "elapsed_s": f"{(time.monotonic() - run_start):.6f}",
                    "phase_index": phase_index,
                    "phase_command_position_deg": f"{command_position_deg:.6f}",
                    "phase_duration_s": f"{phase_duration_s:.6f}",
                    "sample_index": sample_index,
                    "command_position_deg": f"{command_position_deg:.6f}",
                    "left_motor_id": f"0x{motor_entries[0][0]:02X}" if len(motor_entries) > 0 else "",
                    "left_motor_label": motor_entries[0][1] if len(motor_entries) > 0 else "",
                    "left_feedback_position_deg": f"{statuses[0].position_degrees:.6f}" if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_feedback_speed_erpm": statuses[0].speed_erpm if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_feedback_current_amps": f"{statuses[0].current_amps:.6f}" if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_feedback_temperature_c": statuses[0].temperature_celsius if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_feedback_error_code": statuses[0].error_code if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_feedback_error_description": statuses[0].error_description if len(statuses) > 0 and statuses[0] is not None else "",
                    "left_is_fresh_feedback": statuses[0].is_fresh if len(statuses) > 0 and statuses[0] is not None else "",
                    "right_motor_id": f"0x{motor_entries[1][0]:02X}" if len(motor_entries) > 1 else "",
                    "right_motor_label": motor_entries[1][1] if len(motor_entries) > 1 else "",
                    "right_feedback_position_deg": f"{statuses[1].position_degrees:.6f}" if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_feedback_speed_erpm": statuses[1].speed_erpm if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_feedback_current_amps": f"{statuses[1].current_amps:.6f}" if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_feedback_temperature_c": statuses[1].temperature_celsius if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_feedback_error_code": statuses[1].error_code if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_feedback_error_description": statuses[1].error_description if len(statuses) > 1 and statuses[1] is not None else "",
                    "right_is_fresh_feedback": statuses[1].is_fresh if len(statuses) > 1 and statuses[1] is not None else "",
                    "sync_error_deg": "" if len(statuses) < 2 or statuses[0] is None or statuses[1] is None else f"{statuses[0].position_degrees - statuses[1].position_degrees:.2f}",
                }
            )

        time.sleep(sample_period_s)

    summaries: list[PhaseSummary] = []
    for motor_index, (motor_id, label) in enumerate(motor_entries):
        positions = per_motor_positions[motor_index]
        errors = per_motor_errors[motor_index]
        mean_position = sum(positions) / len(positions) if positions else None
        summaries.append(
            PhaseSummary(
                command_position_deg=command_position_deg,
                samples_total=len(positions),
                mean_position_deg=mean_position,
                rmse_deg=_rmse(errors),
                sample_errors_deg=errors,
            )
        )
        print_phase_summary(motor_id, label, summaries[-1])

        # Print the average delay between commanded and measured position for each motor
        delay = calculate_average_position_delay(
                command_history,
                per_motor_feedback_history[motor_index],
            )
        if delay is not None:
                print(
                    f"Motor 0x{motor_id:02X} [{label}] | "
                    f"Average position delay = {delay*1000:.2f} ms"
                )
        else:
                print(
                    f"Motor 0x{motor_id:02X} [{label}] | "
                    "Average position delay unavailable"
                )

        if feedback_latencies[motor_index]:
            avg_latency = (
                sum(feedback_latencies[motor_index])
                /
                len(feedback_latencies[motor_index])
            )
            print(
                f"Motor 0x{motor_id:02X} [{label}] | "
                f"Average command-feedback latency = "
                f"{avg_latency*1000:.3f} ms"
            )
        else:
            print(
                f"Motor 0x{motor_id:02X} [{label}] | "
                "No latency samples"
            )

    return summaries


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments for the set_position evaluator."""
    parser = argparse.ArgumentParser(
        description="Evaluate CubeMars `set_position()` with single or dual motor support"
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
        "--position-deg",
        type=float,
        required=True,
        help="Target positive/negative position magnitude in degrees",
    )
    parser.add_argument(
        "--velocity-deg-s",
        type=float,
        default=20.0,
        help="Movement speed for each ping-pong segment in deg/s",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=1.0,
        help="Hold time at each endpoint in seconds",
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=CAN_DEFAULTS.motor_control_rate_hz,
        help="Position command update rate (default: 100.0 Hz)",
    )
    parser.add_argument(
        "--repeats",
        type=int,
        default=5,
        help="Number of ping-pong repetitions to evaluate (default: 5)",
    )
    parser.add_argument(
        "--sample-hz",
        type=float,
        default=CAN_DEFAULTS.motor_control_rate_hz,
        help="Feedback sampling rate in Hz (default: 100.0)",
    )
    parser.add_argument(
        "--csv-path",
        default=None,
        help="Output CSV path for command/feedback logging",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    """Validate input arguments for the position evaluator."""
    if args.bitrate <= 0:
        raise ValueError("--bitrate must be > 0")
    if abs(args.position_deg) < 1e-9:
        raise ValueError("--position-deg must be non-zero")
    if args.velocity_deg_s <= 0:
        raise ValueError("--velocity-deg-s must be > 0")
    if args.hold_seconds < 0:
        raise ValueError("--hold-seconds must be >= 0")
    if args.control_hz <= 0:
        raise ValueError("--control-hz must be > 0")
    if args.repeats < 1:
        raise ValueError("--repeats must be >= 1")
    if args.sample_hz <= 0:
        raise ValueError("--sample-hz must be > 0")


def print_phase_summary(motor_id: int, motor_label: str, phase_summary: PhaseSummary) -> None:
    """Print one concise phase summary line."""
    rmse = phase_summary.rmse_deg if phase_summary.rmse_deg is not None else float("nan")
    mean = phase_summary.mean_position_deg if phase_summary.mean_position_deg is not None else 0.0
    print(
        f"Motor 0x{motor_id:02X} [{motor_label}] | "
        f"cmd={phase_summary.command_position_deg:+7.2f} deg | "
        f"samples={phase_summary.samples_total:3d} | "
        f"mean={mean:8.2f} deg | "
        f"RMSE={rmse:8.2f} deg"
    )


def main() -> int:  # noqa: C901, PLR0912, PLR0915
    """Run the set_position evaluation for one or two motors."""
    args = parse_args()
    validate_args(args)
    csv_path = _resolve_csv_path(args.csv_path, prefix="evaluate_set_position")

    print(SEPARATOR)
    print("Evaluate set_position()")
    print(SEPARATOR)
    print(f"Interface         : {args.interface}")
    print(f"Bitrate           : {args.bitrate}")
    print(f"Motor model       : {args.motor_model}")
    print(f"Target amplitude  : {args.position_deg:.2f} deg")
    print(f"Velocity          : {args.velocity_deg_s:.2f} deg/s")
    print(f"Hold time         : {args.hold_seconds:.2f} s")
    print(f"Control Hz        : {args.control_hz:.1f}")
    print(f"Repeats           : {args.repeats}")
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
                motor.zero_position()
                if not motor.check_communication():
                    print(f"FAIL: communication check failed for motor 0x{motor_id:02X}")
                    return 1
                print(f"PASS: communication verified on motor 0x{motor_id:02X}")

            run_start = time.monotonic()
            all_errors: list[float] = []
            phase_count = 0
            next_start_position_deg = 0.0

            for cycle in range(1, args.repeats + 1):
                for direction in (+1, -1):
                    phase_count += 1
                    target_deg = direction * abs(args.position_deg)
                    print(f"\nCycle {cycle:02d} | Phase {phase_count:02d}: target={target_deg:+7.2f} deg")
                    phase_summaries = run_synchronized_phase(
                        motors,
                        motor_ids,
                        phase_index=phase_count,
                        command_position_deg=target_deg,
                        velocity_deg_s=args.velocity_deg_s,
                        hold_seconds=args.hold_seconds,
                        control_hz=args.control_hz,
                        sample_hz=args.sample_hz,
                        run_start=run_start,
                        start_position_deg = next_start_position_deg,
                        sample_logger=write_sample_row,
                    )
                    last_status = _read_status(motors[0], timeout=0.1)  # or average across motors if needed
                    next_start_position_deg = target_deg
                    for phase_summary in phase_summaries:
                        if phase_summary.sample_errors_deg:
                            all_errors.extend(phase_summary.sample_errors_deg)

            total_rmse = _rmse(all_errors)
            print(f"\n{SEPARATOR}")
            if total_rmse is None:
                print("FAIL: no feedback samples recorded")
                return 1

            print(f"Overall RMSE: {total_rmse:.3f} deg")
            try:
                plot_path = plot_position_feedback(csv_path)
                print(f"Plot saved to: {plot_path}")
            except Exception as exc:
                logger.warning(f"Position plot not generated: {exc}")

            print("PASS: set_position evaluation finished")
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
            try:
                timing_stats = motor.get_timing_stats()
                print_timing_stats(timing_stats, 0, SEPARATOR)
            except Exception as exc:
                logger.debug(f"Timing stats unavailable during cleanup: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
