#!/usr/bin/env python3
"""
This script is intended to test how the CAN refresh/keepalive loop behaves
when the target refresh frequency is changed from a low value up to a high
value. It writes a CSV recording the commanded vs actual frequency and
produces a plot of set frequency vs actual frequency.

Run:
    sudo ./setup_can.sh
    .venv/bin/python scripts/verify_frequency.py
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from motor_python.can_utils import get_can_state, reset_can_interface
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN
from motor_python.definitions import CAN_DEFAULTS

CSV_FIELDNAMES = [
    "target_hz",
    "actual_hz",
    "loop_period_expected_s",
    "loop_period_mean_s",
    "loop_period_std_s",
    "loop_period_min_s",
    "loop_period_max_s",
    "loop_intervals_total",
    "loop_jitter_count",
    "loop_jitter_ratio",
    "cumulative_send_failures",
    "cumulative_missed_feedback",
    "command_erpm",
    "timestamp_iso",
]

SEPARATOR = "=" * 78


@dataclass(frozen=True)
class FrequencyResult:
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
    command_erpm: int
    timestamp_iso: str


def _resolve_csv_path(csv_path_arg: str | None, *, prefix: str) -> Path:
    if csv_path_arg:
        return Path(csv_path_arg).expanduser().resolve()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return (Path("data/csv_logs") / f"{prefix}_{timestamp}.csv").resolve()


def _resolve_plot_path(plot_path_arg: str | None, *, prefix: str) -> Path:
    if plot_path_arg:
        return Path(plot_path_arg).expanduser().resolve()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return (Path("data/csv_logs") / f"{prefix}_{timestamp}.png").resolve()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sweep CAN MIT refresh frequency and record actual loop frequency."
    )
    parser.add_argument("--interface", default="can0", help="SocketCAN interface")
    parser.add_argument(
        "--motor-id",
        type=lambda value: int(value, 0),
        default=CAN_DEFAULTS.motor_can_id,
        help="Motor CAN ID in decimal or hex (default: 0x03)",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=CAN_DEFAULTS.bitrate,
        help="CAN bitrate (default: 1000000)",
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
        help="Optional output PNG path for set frequency vs actual frequency plot.",
    )
    parser.add_argument(
        "--csv-path",
        default=None,
        help="Optional output CSV path for frequency results.",
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


def _target_frequency_sequence(start_hz: float, end_hz: float, step_hz: float) -> list[float]:
    values = []
    current = start_hz
    while current <= end_hz + 1e-9:
        values.append(float(current))
        current += step_hz
    if values and values[-1] < end_hz:
        values.append(end_hz)
    return values


def measure_frequency(
    motor: CubeMarsAK606v3CAN,
    target_hz: float,
    command_erpm: int,
    warmup_seconds: float,
    phase_seconds: float,
) -> FrequencyResult:
    if target_hz <= 0.0:
        raise ValueError("target_hz must be > 0")

    motor.set_refresh_rate_hz(target_hz)
    motor.set_velocity(command_erpm)
    time.sleep(warmup_seconds)

    # Prefer public API to reset timing diagnostics; fall back to private attr
    try:
        motor.reset_timing_stats()
    except Exception:
        if hasattr(motor, "_refresh_timestamps"):
            try:
                motor._refresh_timestamps.clear()
            except Exception:
                pass

    start = time.monotonic()
    time.sleep(phase_seconds)

    stats = motor.get_timing_stats()
    effective_hz = float(stats.get("loop_effective_hz", 0.0))

    return FrequencyResult(
        target_hz=target_hz,
        actual_hz=effective_hz,
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
        command_erpm=command_erpm,
        timestamp_iso=datetime.now().isoformat(timespec="seconds"),
    )


def plot_results(results: list[FrequencyResult], path: Path) -> None:
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
    y = [result.actual_hz for result in results]

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(x, y, marker="o", linestyle="-", color="#1f77b4", label="actual")
    ax.plot(x, x, linestyle="--", color="#ff7f0e", label="ideal")
    ax.set_xlabel("Target refresh frequency (Hz)")
    ax.set_ylabel("Actual loop frequency (Hz)")
    ax.set_title("Set frequency vs actual CAN refresh frequency")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main() -> int:
    limit_frequency = None
    last_good_frequency = None
    args = parse_args()
    validate_args(args)

    csv_file = None
    csv_writer = None

    csv_path = _resolve_csv_path(args.csv_path, prefix="verify_frequency")
    plot_path = _resolve_plot_path(args.plot_path, prefix="verify_frequency")

    print(SEPARATOR)
    print("Frequency sweep test")
    print(SEPARATOR)
    print(f"Interface   : {args.interface}")
    print(f"Bitrate     : {args.bitrate}")
    print(f"Motor ID    : 0x{args.motor_id:02X}")
    print(f"Velocity ERPM: {args.velocity_erpm}")
    print(f"Start Hz    : {args.start_hz:.1f}")
    print(f"End Hz      : {args.end_hz:.1f}")
    print(f"Step Hz     : {args.step_hz:.1f}")
    print(f"Phase sec   : {args.phase_seconds:.1f}")
    print(f"Warmup sec  : {args.warmup_seconds:.2f}")
    print(f"CSV output  : {csv_path}")
    print(f"Plot output : {plot_path}")
    print(SEPARATOR)

    ensure_can_ready(args.interface, bitrate=args.bitrate, mode=args.preflight_mode)

    motor: CubeMarsAK606v3CAN | None = None
    results: list[FrequencyResult] = []
    try:
        motor = CubeMarsAK606v3CAN(
            motor_can_id=args.motor_id,
            interface=args.interface,
            bitrate=args.bitrate,
        )
        if not motor.connected:
            print("FAIL: could not connect to CAN motor interface")
            return 1

        # Enable MIT mode FIRST before checking communication
        try:
            motor.enable_motor()
            print("PASS: motor enabled (MIT mode)")
        except Exception as exc:
            print(f"FAIL: could not enable motor: {exc}")
            return 1

        if not motor.check_communication():
            print("FAIL: communication check failed (no feedback)")
            motor.disable_mit_mode()
            return 1

        print("PASS: communication check")

        print("Starting frequency sweep...")

        # csv setup
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = csv_path.open("w", newline="", encoding="utf-8")
        csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
        csv_writer.writeheader()
        csv_file.flush()

        def write_csv_row(result: FrequencyResult) -> None:
            if csv_writer is None or csv_file is None:
                return

            row = {
                "target_hz": f"{result.target_hz:.3f}",
                "actual_hz": f"{result.actual_hz:.3f}",
                "loop_period_expected_s": f"{result.loop_period_expected_s:.6f}" if result.loop_period_expected_s is not None else "",
                "loop_period_mean_s": f"{result.loop_period_mean_s:.6f}" if result.loop_period_mean_s is not None else "",
                "loop_period_std_s": f"{result.loop_period_std_s:.6f}" if result.loop_period_std_s is not None else "",
                "loop_period_min_s": f"{result.loop_period_min_s:.6f}" if result.loop_period_min_s is not None else "",
                "loop_period_max_s": f"{result.loop_period_max_s:.6f}" if result.loop_period_max_s is not None else "",
                "loop_intervals_total": result.loop_intervals_total,
                "loop_jitter_count": result.loop_jitter_count,
                "loop_jitter_ratio": f"{result.loop_jitter_ratio:.3f}" if result.loop_jitter_ratio is not None else "",
                "cumulative_send_failures": result.cumulative_send_failures,
                "cumulative_missed_feedback": result.cumulative_missed_feedback,
                "command_erpm": result.command_erpm,
                "timestamp_iso": result.timestamp_iso,
            }

            csv_writer.writerow(row)
            csv_file.flush()

        sequence = _target_frequency_sequence(args.start_hz, args.end_hz, args.step_hz)
        for target_hz in sequence:
            print(f"\nTesting target={target_hz:.1f} Hz...")
            result = measure_frequency(
                motor=motor,
                target_hz=target_hz,
                command_erpm=args.velocity_erpm,
                warmup_seconds=args.warmup_seconds,
                phase_seconds=args.phase_seconds,
            )
            results.append(result)

            write_csv_row(result)

            within_tol = is_within_tolerance(
                result.target_hz, result.actual_hz, args.tolerance_hz
            )

            status = "OK" if within_tol else "EXCEEDS"

            print(
                f"target={result.target_hz:.1f} Hz "
                f"actual={result.actual_hz:.1f} Hz "
                f"mean={result.loop_period_mean_s or 0:.5f}s "
                f"jitter={result.loop_jitter_count}"
                f"status={status}"
            )

            if within_tol:
                last_good_frequency = result.target_hz
            elif limit_frequency is None:
                limit_frequency = result.target_hz

        # Summary
        print(f"\n{SEPARATOR}")
        print("Frequency Limit Analysis")
        print(SEPARATOR)

        if last_good_frequency is not None:
            print(f"Max stable frequency (within tolerance): {last_good_frequency:.1f} Hz")
        else:
            print("No frequency met tolerance criteria")

        if limit_frequency is not None:
            print(f"First unstable frequency: {limit_frequency:.1f} Hz")
        else:
            print("No instability detected within tested range")

        print(SEPARATOR)

        try:
            plot_results(results, plot_path)
            print(f"Plot saved to: {plot_path}")
        except RuntimeError as exc:
            print(f"WARN: Plot skipped: {exc}")

        print(f"CSV saved to: {csv_path}")
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
