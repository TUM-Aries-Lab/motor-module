#!/usr/bin/env python3
"""
Measure CAN refresh frequency while both motors run simultaneously.

This script mirrors the single-motor frequency sweep but commands two motors
in lockstep and records their timing statistics side by side. It is intended
for comparing the measured loop frequency when one motor is active versus when
both motors are active on the same CAN bus.

Run:
    sudo ./setup_can.sh
    .venv/bin/python scripts/verify_dual_frequency.py --left-id 0x03 --right-id 0x04 --left-motor-model AK80-6 --right-motor-model AK80-6
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

if __package__ in {None, ""}:
    repo_src = Path(__file__).resolve().parents[1] / "src"
    if str(repo_src) not in sys.path:
        sys.path.insert(0, str(repo_src))

from motor_python import create_can_motor
from motor_python.can_utils import get_can_state, reset_can_interface
from motor_python.cube_mars_motor_can import CubeMarsBaseCAN
from motor_python.definitions import CAN_DEFAULTS, MotorModel

CSV_FIELDNAMES = [
    "target_hz",
    "left_actual_hz",
    "left_frequency_error_hz",
    "left_loop_period_expected_s",
    "left_loop_period_mean_s",
    "left_loop_period_std_s",
    "left_loop_period_min_s",
    "left_loop_period_max_s",
    "left_loop_intervals_total",
    "left_loop_jitter_count",
    "left_loop_jitter_ratio",
    "left_cumulative_send_failures",
    "left_cumulative_missed_feedback",
    "left_actual_speed_erpm",
    "right_actual_hz",
    "right_frequency_error_hz",
    "right_loop_period_expected_s",
    "right_loop_period_mean_s",
    "right_loop_period_std_s",
    "right_loop_period_min_s",
    "right_loop_period_max_s",
    "right_loop_intervals_total",
    "right_loop_jitter_count",
    "right_loop_jitter_ratio",
    "right_cumulative_send_failures",
    "right_cumulative_missed_feedback",
    "right_actual_speed_erpm",
    "actual_hz_delta",
    "command_erpm",
    "timestamp_iso",
]

SEPARATOR = "=" * 78


@dataclass(frozen=True)
class MotorFrequencySnapshot:
    target_hz: float
    actual_hz: float
    loop_period_expected_s: float | None
    loop_period_mean_s: float | None
    loop_period_std_s: float | None
    loop_period_min_s: float | None
    loop_period_max_s: float | None
    loop_intervals_total: int
    loop_jitter_count: int
    loop_jitter_ratio: float | None
    cumulative_send_failures: int
    cumulative_missed_feedback: int
    actual_speed_erpm: int | None
    frequency_error_hz: float


@dataclass(frozen=True)
class DualFrequencyResult:
    target_hz: float
    left: MotorFrequencySnapshot
    right: MotorFrequencySnapshot
    actual_hz_delta: float
    command_erpm: int
    timestamp_iso: str


def _resolve_csv_path(csv_path_arg: str | None, *, prefix: str) -> Path:
    if csv_path_arg:
        return Path(csv_path_arg).expanduser().resolve()
    timestamp = datetime.now().strftime("%m%d_%H%M%S")
    return (Path("data/csv_logs") / f"{prefix}_{CAN_DEFAULTS.motor_control_rate_hz}hz_{timestamp}.csv").resolve()


def _resolve_plot_path(plot_path_arg: str | None, *, prefix: str) -> Path:
    if plot_path_arg:
        return Path(plot_path_arg).expanduser().resolve()
    timestamp = datetime.now().strftime("%m%d_%H%M%S")
    return (Path("data/csv_logs") / f"{prefix}_{CAN_DEFAULTS.motor_control_rate_hz}hz_{timestamp}.png").resolve()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sweep CAN refresh frequency while both motors run simultaneously."
    )
    parser.add_argument("--interface", default="can0", help="SocketCAN interface")
    parser.add_argument(
        "--bitrate",
        type=int,
        default=CAN_DEFAULTS.bitrate,
        help="CAN bitrate (default: 1000000)",
    )
    parser.add_argument(
        "--left-id",
        type=lambda value: int(value, 0),
        default=CAN_DEFAULTS.motor_can_id,
        help="Left motor CAN ID in decimal or hex (default: 0x03)",
    )
    parser.add_argument(
        "--right-id",
        type=lambda value: int(value, 0),
        default=CAN_DEFAULTS.motor_can_id_2,
        help="Right motor CAN ID in decimal or hex (default: 0x04)",
    )
    parser.add_argument(
        "--preflight-mode",
        choices=("strict", "auto", "skip"),
        default="auto",
        help=(
            "strict=fail if bus unhealthy, auto=try `sudo ./setup_can.sh`, "
            "skip=do not gate start (default: auto)"
        ),
    )
    parser.add_argument(
        "--left-motor-model",
        choices=list(MotorModel),
        default=MotorModel.AK60_6V3,
        help="Left motor model to instantiate (default: AK60-6)",
    )
    parser.add_argument(
        "--right-motor-model",
        choices=list(MotorModel),
        default=MotorModel.AK60_6V3,
        help="Right motor model to instantiate (default: AK60-6)",
    )
    parser.add_argument(
        "--start-hz",
        type=float,
        default=20.0,
        help="Starting target refresh frequency in Hz (default: 20)",
    )
    parser.add_argument(
        "--end-hz",
        type=float,
        default=500.0,
        help="Ending target refresh frequency in Hz (default: 500)",
    )
    parser.add_argument(
        "--step-hz",
        type=float,
        default=10.0,
        help="Step size between frequency points in Hz (default: 10)",
    )
    parser.add_argument(
        "--phase-seconds",
        type=float,
        default=5.0,
        help="Measurement duration for each frequency point in seconds (default: 5.0)",
    )
    parser.add_argument(
        "--warmup-seconds",
        type=float,
        default=0.5,
        help="Warmup time after changing frequency in seconds (default: 0.5)",
    )
    parser.add_argument(
        "--velocity-erpm",
        type=int,
        default=3000,
        help="Command velocity used during frequency testing in ERPM (default: 3000)",
    )
    parser.add_argument(
        "--tolerance-hz",
        type=float,
        default=5.0,
        help="Allowed absolute difference between target and actual frequency (default: 5 Hz)",
    )
    parser.add_argument(
        "--plot-path",
        default=None,
        help="Optional output PNG path for the dual-motor frequency plot.",
    )
    parser.add_argument(
        "--csv-path",
        default=None,
        help="Optional output CSV path for dual-motor frequency results.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.bitrate <= 0:
        raise ValueError("--bitrate must be > 0")
    if args.start_hz <= 0.0:
        raise ValueError("--start-hz must be > 0")
    if args.end_hz <= 0.0:
        raise ValueError("--end-hz must be > 0")
    if args.step_hz <= 0.0:
        raise ValueError("--step-hz must be > 0")
    if args.phase_seconds <= 0.0:
        raise ValueError("--phase-seconds must be > 0")
    if args.warmup_seconds < 0.0:
        raise ValueError("--warmup-seconds must be >= 0")
    if args.end_hz < args.start_hz:
        raise ValueError("--end-hz must be >= --start-hz")
    if args.left_id == args.right_id:
        raise ValueError("--left-id and --right-id must differ")


def _is_can_state_healthy(state: dict[str, int | str]) -> bool:
    return (
        state["state"] == "ERROR-ACTIVE"
        and int(state["tx_err"]) < 96
        and int(state["rx_err"]) < 64
    )


def ensure_can_ready(interface: str, bitrate: int, *, mode: str) -> None:
    state = get_can_state(interface)
    print(
        f"CAN preflight: state={state['state']} tx_err={state['tx_err']} rx_err={state['rx_err']}"
    )
    if _is_can_state_healthy(state):
        return
    if mode == "skip":
        print("CAN preflight skipped by request.")
        return
    if mode == "strict":
        raise RuntimeError(
            "CAN interface unhealthy. Run `sudo ./setup_can.sh` and retry."
        )

    print("CAN preflight: attempting automatic kernel-level CAN reset ...")
    if not reset_can_interface(interface=interface, bitrate=bitrate):
        raise RuntimeError(
            "Auto preflight reset failed. Run `sudo ./setup_can.sh` manually."
        )

    after = get_can_state(interface)
    print(
        f"CAN preflight after reset: state={after['state']} tx_err={after['tx_err']} rx_err={after['rx_err']}"
    )
    if not _is_can_state_healthy(after):
        raise RuntimeError(
            "CAN still unhealthy after reset. Check wiring/power/UART disconnect."
        )


def is_within_tolerance(target: float, actual: float, tol: float) -> bool:
    return abs(actual - target) <= tol


def _find_max_stable_frequency(
    results: list[DualFrequencyResult],
    *,
    side: str,
    tolerance_hz: float,
) -> float | None:
    if side not in {"left", "right"}:
        raise ValueError("side must be 'left' or 'right'")

    last_good_hz: float | None = None
    for result in results:
        actual_hz = result.left.actual_hz if side == "left" else result.right.actual_hz
        if abs(actual_hz - result.target_hz) <= tolerance_hz:
            last_good_hz = result.target_hz
    return last_good_hz


def _target_frequency_sequence(start_hz: float, end_hz: float, step_hz: float) -> list[float]:
    values: list[float] = []
    current = start_hz
    while current <= end_hz + 1e-9:
        values.append(float(current))
        current += step_hz
    if values and values[-1] < end_hz:
        values.append(end_hz)
    return values


def _snapshot_from_motor(
    motor: CubeMarsBaseCAN,
    target_hz: float,
    command_erpm: int,
) -> MotorFrequencySnapshot:
    stats = motor.get_timing_stats()
    effective_hz = float(stats.get("loop_effective_hz", 0.0))
    actual_speed_erpm = motor.get_speed()
    return MotorFrequencySnapshot(
        target_hz=target_hz,
        actual_hz=effective_hz,
        frequency_error_hz=effective_hz - target_hz,
        loop_period_expected_s=float(stats.get("loop_period_expected_s", 0.0)) if stats.get("available", False) else None,
        loop_period_mean_s=float(stats.get("loop_period_mean_s", 0.0)) if stats.get("available", False) else None,
        loop_period_std_s=float(stats.get("loop_period_std_s", 0.0)) if stats.get("available", False) else None,
        loop_period_min_s=float(stats.get("loop_period_min_s", 0.0)) if stats.get("available", False) else None,
        loop_period_max_s=float(stats.get("loop_period_max_s", 0.0)) if stats.get("available", False) else None,
        loop_intervals_total=int(stats.get("loop_intervals_total", 0)),
        loop_jitter_count=int(stats.get("loop_jitter_count", 0)),
        loop_jitter_ratio=float(stats.get("loop_jitter_ratio", 0.0)) if stats.get("available", False) else None,
        cumulative_send_failures=int(getattr(motor, "_cumulative_refresh_send_failures", 0)),
        cumulative_missed_feedback=int(getattr(motor, "_cumulative_refresh_no_feedback", 0)),
        actual_speed_erpm=actual_speed_erpm,
    )


def measure_dual_frequency(
    left_motor: CubeMarsBaseCAN,
    right_motor: CubeMarsBaseCAN,
    target_hz: float,
    command_erpm: int,
    warmup_seconds: float,
    phase_seconds: float,
) -> DualFrequencyResult:
    if target_hz <= 0.0:
        raise ValueError("target_hz must be > 0")

    left_motor.set_refresh_rate_hz(target_hz)
    right_motor.set_refresh_rate_hz(target_hz)
    left_motor.set_velocity(command_erpm)
    right_motor.set_velocity(command_erpm)
    time.sleep(warmup_seconds)

    for motor in (left_motor, right_motor):
        try:
            motor.reset_timing_stats()
        except Exception:
            if hasattr(motor, "_refresh_timestamps"):
                try:
                    motor._refresh_timestamps.clear()
                except Exception:
                    pass

    time.sleep(phase_seconds)

    left_snapshot = _snapshot_from_motor(left_motor, target_hz, command_erpm)
    right_snapshot = _snapshot_from_motor(right_motor, target_hz, command_erpm)

    return DualFrequencyResult(
        target_hz=target_hz,
        left=left_snapshot,
        right=right_snapshot,
        actual_hz_delta=left_snapshot.actual_hz - right_snapshot.actual_hz,
        command_erpm=command_erpm,
        timestamp_iso=datetime.now().isoformat(timespec="seconds"),
    )


def plot_results(
    results: list[DualFrequencyResult],
    path: Path,
    *,
    left_max_stable_hz: float | None = None,
    right_max_stable_hz: float | None = None,
    tolerance_hz: float | None = None,
) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise RuntimeError(
            "Matplotlib is required to generate the frequency plot. "
            "Install it via the analysis extras or pip install matplotlib."
        ) from exc

    path.parent.mkdir(parents=True, exist_ok=True)
    x = [result.target_hz for result in results]
    left_y = [result.left.actual_hz for result in results]
    right_y = [result.right.actual_hz for result in results]
    left_error = [result.left.frequency_error_hz for result in results]
    right_error = [result.right.frequency_error_hz for result in results]
    delta = [result.actual_hz_delta for result in results]
    left_missed_feedback = [result.left.cumulative_missed_feedback for result in results]
    right_missed_feedback = [result.right.cumulative_missed_feedback for result in results]
    left_jitter_ratio = [result.left.loop_jitter_ratio or 0.0 for result in results]
    right_jitter_ratio = [result.right.loop_jitter_ratio or 0.0 for result in results]
    left_actual_speed = [
        float(result.left.actual_speed_erpm) if result.left.actual_speed_erpm is not None else float("nan")
        for result in results
    ]
    right_actual_speed = [
        float(result.right.actual_speed_erpm) if result.right.actual_speed_erpm is not None else float("nan")
        for result in results
    ]
    commanded_speed = [float(result.command_erpm) for result in results]

    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(10, 17), sharex=True)
    subtitle = (
        f"Tolerance: ±{tolerance_hz:.1f} Hz"
        if tolerance_hz is not None
        else ""
    )
    fig.suptitle(
        "Dual-motor CAN refresh frequency comparison",
        fontsize=16,
        fontweight="bold",
    )
    if subtitle:
        fig.text(0.5, 0.965, subtitle, ha="center", fontsize=11)
    fig.subplots_adjust(top=0.92)

    ax1.plot(x, left_y, marker="o", linestyle="-", color="#1f77b4", label="left actual")
    ax1.plot(x, right_y, marker="s", linestyle="-", color="#d62728", label="right actual")
    ax1.plot(x, x, linestyle="--", color="#ff7f0e", label="ideal")
    if left_max_stable_hz is not None:
        ax1.axvline(left_max_stable_hz, color="#1f77b4", linestyle=":", linewidth=2)
        ax1.annotate(
            f"left max stable: {left_max_stable_hz:.1f} Hz",
            xy=(left_max_stable_hz, left_max_stable_hz),
            xytext=(left_max_stable_hz + 10, left_max_stable_hz - 40),
            textcoords="data",
            arrowprops={"arrowstyle": "->", "color": "#1f77b4"},
            color="#1f77b4",
        )
    if right_max_stable_hz is not None:
        ax1.axvline(right_max_stable_hz, color="#d62728", linestyle=":", linewidth=2)
        ax1.annotate(
            f"right max stable: {right_max_stable_hz:.1f} Hz",
            xy=(right_max_stable_hz, right_max_stable_hz),
            xytext=(right_max_stable_hz + 10, right_max_stable_hz - 25),
            textcoords="data",
            arrowprops={"arrowstyle": "->", "color": "#d62728"},
            color="#d62728",
        )
    ax1.set_ylabel("Actual loop frequency (Hz)")
    ax1.grid(alpha=0.3)
    ax1.legend(loc="upper left")

    ax2.plot(x, left_error, marker="o", linestyle="-", color="#1f77b4", label="left error")
    ax2.plot(x, right_error, marker="s", linestyle="-", color="#d62728", label="right error")
    ax2.axhline(0.0, color="black", linewidth=0.8, linestyle="--")
    ax2.set_ylabel("Error (Hz)")
    ax2.grid(alpha=0.3)
    ax2.legend(loc="upper left")

    ax3.plot(x, left_missed_feedback, marker="D", linestyle="-", color="#1f77b4", label="left missed feedback")
    ax3.plot(x, right_missed_feedback, marker="P", linestyle="-", color="#d62728", label="right missed feedback")
    ax3.set_ylabel("Cumulative missed feedback")
    ax3.grid(alpha=0.3)
    ax3.legend(loc="upper left")

    ax4.plot(x, left_jitter_ratio, marker="^", linestyle="-", color="#1f77b4", label="left jitter ratio")
    ax4.plot(x, right_jitter_ratio, marker="v", linestyle="-", color="#d62728", label="right jitter ratio")
    ax4.set_ylabel("Jitter ratio")
    ax4.grid(alpha=0.3)
    ax4.legend(loc="upper left")

    ax5.plot(x, left_actual_speed, marker="o", linestyle="-", color="#1f77b4", label="left actual speed")
    ax5.plot(x, right_actual_speed, marker="s", linestyle="-", color="#d62728", label="right actual speed")
    ax5.plot(x, commanded_speed, linestyle="--", color="#ff7f0e", label="commanded speed")
    ax5.set_xlabel("Target refresh frequency (Hz)")
    ax5.set_ylabel("Speed (ERPM)")
    ax5.grid(alpha=0.3)
    ax5.legend(loc="upper left")

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    validate_args(args)

    csv_path = _resolve_csv_path(args.csv_path, prefix="verify_dual_frequency")
    plot_path = _resolve_plot_path(args.plot_path, prefix="verify_dual_frequency")

    print(SEPARATOR)
    print("Dual-motor frequency sweep test")
    print(SEPARATOR)
    print(f"Interface     : {args.interface}")
    print(f"Bitrate       : {args.bitrate}")
    print(f"Left ID       : 0x{args.left_id:02X}")
    print(f"Right ID      : 0x{args.right_id:02X}")
    print(f"Left model    : {args.left_motor_model}")
    print(f"Right model   : {args.right_motor_model}")
    print(f"Velocity ERPM : {args.velocity_erpm}")
    print(f"Start Hz      : {args.start_hz:.1f}")
    print(f"End Hz        : {args.end_hz:.1f}")
    print(f"Step Hz       : {args.step_hz:.1f}")
    print(f"Phase sec     : {args.phase_seconds:.1f}")
    print(f"Warmup sec    : {args.warmup_seconds:.2f}")
    print(f"CSV output    : {csv_path}")
    print(f"Plot output   : {plot_path}")
    print(SEPARATOR)

    ensure_can_ready(args.interface, bitrate=args.bitrate, mode=args.preflight_mode)

    left_motor: CubeMarsBaseCAN | None = None
    right_motor: CubeMarsBaseCAN | None = None
    csv_file = None
    csv_writer = None
    results: list[DualFrequencyResult] = []

    try:
        left_motor = create_can_motor(
            motor_model=args.left_motor_model,
            motor_can_id=args.left_id,
            interface=args.interface,
            bitrate=args.bitrate,
        )
        right_motor = create_can_motor(
            motor_model=args.right_motor_model,
            motor_can_id=args.right_id,
            interface=args.interface,
            bitrate=args.bitrate,
        )

        if not left_motor.connected or not right_motor.connected:
            print("FAIL: could not connect to one or both CAN motor interfaces")
            return 1

        for label, motor in (("left", left_motor), ("right", right_motor)):
            try:
                motor.enable_motor()
                print(f"PASS: {label} motor enabled")
            except Exception as exc:
                print(f"FAIL: could not enable {label} motor: {exc}")
                return 1

            if not motor.check_communication():
                print(f"FAIL: {label} communication check failed (no feedback)")
                motor.disable_mit_mode()
                return 1

            motor.send_neutral_command()

        print("PASS: communication checks")
        print("Starting dual-motor frequency sweep...")

        csv_path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = csv_path.open("w", newline="", encoding="utf-8")
        csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
        csv_writer.writeheader()
        csv_file.flush()

        def write_csv_row(result: DualFrequencyResult) -> None:
            if csv_writer is None or csv_file is None:
                return
            row = {
                "target_hz": f"{result.target_hz:.3f}",
                "left_actual_hz": f"{result.left.actual_hz:.3f}",
                "left_frequency_error_hz": f"{result.left.frequency_error_hz:.3f}",
                "left_loop_period_expected_s": f"{result.left.loop_period_expected_s:.6f}" if result.left.loop_period_expected_s is not None else "",
                "left_loop_period_mean_s": f"{result.left.loop_period_mean_s:.6f}" if result.left.loop_period_mean_s is not None else "",
                "left_loop_period_std_s": f"{result.left.loop_period_std_s:.6f}" if result.left.loop_period_std_s is not None else "",
                "left_loop_period_min_s": f"{result.left.loop_period_min_s:.6f}" if result.left.loop_period_min_s is not None else "",
                "left_loop_period_max_s": f"{result.left.loop_period_max_s:.6f}" if result.left.loop_period_max_s is not None else "",
                "left_loop_intervals_total": result.left.loop_intervals_total,
                "left_loop_jitter_count": result.left.loop_jitter_count,
                "left_loop_jitter_ratio": f"{result.left.loop_jitter_ratio:.3f}" if result.left.loop_jitter_ratio is not None else "",
                "left_cumulative_send_failures": result.left.cumulative_send_failures,
                "left_cumulative_missed_feedback": result.left.cumulative_missed_feedback,
                "left_actual_speed_erpm": "" if result.left.actual_speed_erpm is None else f"{result.left.actual_speed_erpm:d}",
                "right_actual_hz": f"{result.right.actual_hz:.3f}",
                "right_frequency_error_hz": f"{result.right.frequency_error_hz:.3f}",
                "right_loop_period_expected_s": f"{result.right.loop_period_expected_s:.6f}" if result.right.loop_period_expected_s is not None else "",
                "right_loop_period_mean_s": f"{result.right.loop_period_mean_s:.6f}" if result.right.loop_period_mean_s is not None else "",
                "right_loop_period_std_s": f"{result.right.loop_period_std_s:.6f}" if result.right.loop_period_std_s is not None else "",
                "right_loop_period_min_s": f"{result.right.loop_period_min_s:.6f}" if result.right.loop_period_min_s is not None else "",
                "right_loop_period_max_s": f"{result.right.loop_period_max_s:.6f}" if result.right.loop_period_max_s is not None else "",
                "right_loop_intervals_total": result.right.loop_intervals_total,
                "right_loop_jitter_count": result.right.loop_jitter_count,
                "right_loop_jitter_ratio": f"{result.right.loop_jitter_ratio:.3f}" if result.right.loop_jitter_ratio is not None else "",
                "right_cumulative_send_failures": result.right.cumulative_send_failures,
                "right_cumulative_missed_feedback": result.right.cumulative_missed_feedback,
                "right_actual_speed_erpm": "" if result.right.actual_speed_erpm is None else f"{result.right.actual_speed_erpm:d}",
                "actual_hz_delta": f"{result.actual_hz_delta:.3f}",
                "command_erpm": result.command_erpm,
                "timestamp_iso": result.timestamp_iso,
            }
            csv_writer.writerow(row)
            csv_file.flush()

        sequence = _target_frequency_sequence(args.start_hz, args.end_hz, args.step_hz)
        for target_hz in sequence:
            print(f"\nTesting target={target_hz:.1f} Hz...")
            result = measure_dual_frequency(
                left_motor=left_motor,
                right_motor=right_motor,
                target_hz=target_hz,
                command_erpm=args.velocity_erpm,
                warmup_seconds=args.warmup_seconds,
                phase_seconds=args.phase_seconds,
            )
            results.append(result)
            write_csv_row(result)

            left_within_tol = is_within_tolerance(result.target_hz, result.left.actual_hz, args.tolerance_hz)
            right_within_tol = is_within_tolerance(result.target_hz, result.right.actual_hz, args.tolerance_hz)
            left_status = "OK" if left_within_tol else "EXCEEDS"
            right_status = "OK" if right_within_tol else "EXCEEDS"
            print(
                f"left target={result.target_hz:.1f} Hz actual={result.left.actual_hz:.1f} Hz "
                f"mean={result.left.loop_period_mean_s or 0:.5f}s status={left_status}"
            )
            print(
                f"right target={result.target_hz:.1f} Hz actual={result.right.actual_hz:.1f} Hz "
                f"mean={result.right.loop_period_mean_s or 0:.5f}s status={right_status}"
            )
            print(f"delta actual Hz = {result.actual_hz_delta:.3f}")

        left_max_stable_hz = _find_max_stable_frequency(
            results,
            side="left",
            tolerance_hz=args.tolerance_hz,
        )
        right_max_stable_hz = _find_max_stable_frequency(
            results,
            side="right",
            tolerance_hz=args.tolerance_hz,
        )

        print(f"\n{SEPARATOR}")
        print("Dual-motor frequency summary")
        print(SEPARATOR)
        if left_max_stable_hz is not None:
            print(f"Left max stable frequency (within tolerance): {left_max_stable_hz:.1f} Hz")
        else:
            print("Left max stable frequency: none within tolerance")
        if right_max_stable_hz is not None:
            print(f"Right max stable frequency (within tolerance): {right_max_stable_hz:.1f} Hz")
        else:
            print("Right max stable frequency: none within tolerance")
        print(f"CSV saved to: {csv_path}")

        try:
            plot_results(
                results,
                plot_path,
                left_max_stable_hz=left_max_stable_hz,
                right_max_stable_hz=right_max_stable_hz,
                tolerance_hz=args.tolerance_hz,
            )
            print(f"Plot saved to: {plot_path}")
        except RuntimeError as exc:
            print(f"WARN: Plot skipped: {exc}")

        return 0

    except KeyboardInterrupt:
        print("\nInterrupted by user, saving results and exiting...")
        return 130
    except Exception as exc:
        print(f"\nFAIL: {exc}")
        return 1
    finally:
        if csv_file is not None:
            try:
                csv_file.flush()
                csv_file.close()
            except Exception:
                pass
        for motor in (left_motor, right_motor):
            if motor is None:
                continue
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
