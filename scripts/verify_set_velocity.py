#!/usr/bin/env python3
"""Focused verifier for CubeMars MIT `set_velocity()` behavior.

This script is intentionally simple and strict:
1. Bring CAN to a known-good state (optional auto reset).
2. Check communication.
3. Command +ERPM, then 0, then -ERPM (optional).
4. Verify measured feedback speed matches expected sign and magnitude.
5. Always stop/disable motor on exit.

Run:
    sudo ./setup_can.sh
    .venv/bin/python scripts/verify_set_velocity.py --motor-id 0x03
"""
# ruff: noqa: T201

from __future__ import annotations

import argparse
import csv
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable

from motor_python.base_motor import MotorState
from motor_python.can_utils import get_can_state, reset_can_interface
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN
from motor_python.definitions import CAN_DEFAULTS

SEPARATOR = "=" * 78
HEALTHY_TX_ERR_MAX = 96
HEALTHY_RX_ERR_MAX = 64
VERIFY_VELOCITY_MIN_ERPM = -5000
VERIFY_VELOCITY_MAX_ERPM = 5000

CSV_FIELDNAMES = [
    "wall_time_iso",
    "wall_time_epoch_s",
    "elapsed_s",
    "phase_index",
    "phase_command_erpm",
    "phase_duration_s",
    "sample_index",
    "command_erpm",
    "feedback_position_deg",
    "feedback_speed_erpm",
    "feedback_current_amps",
    "feedback_temperature_c",
    "feedback_error_code",
    "feedback_error_description",
]


def _resolve_csv_path(csv_path_arg: str | None, *, prefix: str) -> Path:
    """Resolve CSV path from CLI arg or generate a timestamped default path."""
    if csv_path_arg:
        return Path(csv_path_arg).expanduser().resolve()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return (Path("data/csv_logs") / f"{prefix}_{timestamp}.csv").resolve()


@dataclass(frozen=True)
class PhaseResult:
    """Collected metrics for one commanded-velocity phase."""

    command_erpm: int
    samples_total: int
    informative_samples: int
    sign_match_ratio: float | None
    mean_speed_erpm: float | None
    peak_abs_speed_erpm: int
    pass_phase: bool


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description="Verify CubeMarsAK606v3CAN.set_velocity() with live motor feedback"
    )
    parser.add_argument("--interface", default="can0", help="SocketCAN interface")
    parser.add_argument(
        "--motor-id",
        type=lambda value: int(value, 0),
        default=0x03,
        help="Motor CAN ID in decimal or hex (default: 0x03)",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=1_000_000,
        help="CAN bitrate (default: 1000000)",
    )
    parser.add_argument(
        "--helper-policy",
        choices=("strict", "fcfd", "legacy"),
        default="fcfd",
        help="MIT helper-frame policy (default: fcfd)",
    )
    parser.add_argument(
        "--allow-legacy-feedback-ids",
        action="store_true",
        help="Accept legacy non-canonical feedback IDs while diagnosing firmware variants.",
    )
    parser.add_argument(
        "--feedback-can-id",
        type=lambda value: int(value, 0),
        default=None,
        help="Optional explicit feedback CAN ID override (hex or decimal).",
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
        "--velocity-erpm",
        type=int,
        default=1000,
        help="Test start velocity in ERPM (range: -5000..5000, default: 1000)",
    )
    parser.add_argument(
        "--velocity-kd",
        type=float,
        default=CAN_DEFAULTS.mit_velocity_kd,
        help=f"MIT velocity damping KD used by set_velocity() (default: {CAN_DEFAULTS.mit_velocity_kd})",
    )
    parser.add_argument(
        "--phase-seconds",
        type=float,
        default=120.0,
        help="Duration for each velocity phase in seconds (default: 120.0)",
    )
    parser.add_argument(
        "--neutral-seconds",
        type=float,
        default=0.8,
        help="Duration for neutral phases between/after directions (default: 0.8)",
    )
    parser.add_argument(
        "--sample-hz",
        type=float,
        default=50.0,
        help="Feedback sampling rate in Hz (default: 50)",
    )
    parser.add_argument(
        "--min-informative-erpm",
        type=int,
        default=700,
        help=(
            "Speed threshold above which a sample is informative for sign checks "
            "(default: 700)"
        ),
    )
    parser.add_argument(
        "--min-sign-match-ratio",
        type=float,
        default=0.65,
        help="Required sign-match ratio for non-zero phases (default: 0.65)",
    )
    parser.add_argument(
        "--min-informative-samples",
        type=int,
        default=6,
        help="Required informative sample count for non-zero phases (default: 6)",
    )
    parser.add_argument(
        "--max-missed-feedback",
        type=int,
        default=10,
        help="Abort after this many consecutive missing feedback samples (default: 10)",
    )
    parser.add_argument(
        "--forward-only",
        action="store_true",
        help="Only verify the commanded start direction (--velocity-erpm sign), then neutral.",
    )
    parser.add_argument(
        "--csv-path",
        default=None,
        help=(
            "Output CSV path for feedback logging "
            "(default: data/csv_logs/verify_set_velocity_<timestamp>.csv)"
        ),
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:  # noqa: C901
    """Validate numeric argument ranges."""
    if args.bitrate <= 0:
        raise ValueError("--bitrate must be > 0")
    if args.phase_seconds <= 0:
        raise ValueError("--phase-seconds must be > 0")
    if args.neutral_seconds < 0:
        raise ValueError("--neutral-seconds must be >= 0")
    if args.sample_hz <= 0:
        raise ValueError("--sample-hz must be > 0")
    if args.min_informative_erpm <= 0:
        raise ValueError("--min-informative-erpm must be > 0")
    if not 0.0 <= args.min_sign_match_ratio <= 1.0:
        raise ValueError("--min-sign-match-ratio must be in [0, 1]")
    if args.min_informative_samples < 1:
        raise ValueError("--min-informative-samples must be >= 1")
    if args.max_missed_feedback < 1:
        raise ValueError("--max-missed-feedback must be >= 1")
    if args.velocity_kd < 0:
        raise ValueError("--velocity-kd must be >= 0")
    velocity_cmd = int(args.velocity_erpm)
    if velocity_cmd < VERIFY_VELOCITY_MIN_ERPM or velocity_cmd > VERIFY_VELOCITY_MAX_ERPM:
        raise ValueError(
            f"--velocity-erpm must be in [{VERIFY_VELOCITY_MIN_ERPM}, {VERIFY_VELOCITY_MAX_ERPM}]"
        )
    if velocity_cmd == 0:
        raise ValueError("--velocity-erpm must be non-zero")


def _is_can_state_healthy(state: dict[str, int | str]) -> bool:
    """Return True when CAN state is healthy enough for controlled motor tests."""
    return (
        state["state"] == "ERROR-ACTIVE"
        and int(state["tx_err"]) < HEALTHY_TX_ERR_MAX
        and int(state["rx_err"]) < HEALTHY_RX_ERR_MAX
    )


def ensure_can_ready(interface: str, bitrate: int, *, mode: str) -> None:
    """Verify or recover CAN state before commanding motion."""
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


def read_status(motor: CubeMarsAK606v3CAN, timeout: float) -> MotorState | None:
    """Read freshest available feedback without long blocking."""
    status = motor._receive_feedback(timeout=timeout)
    if status is not None:
        return status
    return motor._last_feedback


def _command_phase(motor: CubeMarsAK606v3CAN, command_erpm: int) -> None:
    """Send one phase command."""
    if command_erpm == 0:
        motor.set_mit_mode(
            pos_rad=0.0,
            vel_rad_s=0.0,
            kp=0.0,
            kd=2.0,
            torque_ff_nm=0.0,
        )
        return
    motor.set_velocity(command_erpm)


def _mean(values: list[float]) -> float | None:
    """Return mean or None for empty input."""
    if not values:
        return None
    return sum(values) / len(values)


def _sign(value: float) -> int:
    """Return sign as -1, 0, +1."""
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


def run_phase(  # noqa: PLR0913
    motor: CubeMarsAK606v3CAN,
    *,
    phase_index: int,
    command_erpm: int,
    duration_s: float,
    sample_hz: float,
    min_informative_erpm: int,
    min_sign_match_ratio: float,
    min_informative_samples: int,
    max_missed_feedback: int,
    run_start: float,
    sample_logger: Callable[[dict[str, str | int]], None] | None = None,
) -> PhaseResult:
    """Execute a velocity phase and evaluate speed feedback quality."""
    _command_phase(motor, command_erpm)

    period_s = 1.0 / sample_hz
    t_end = time.monotonic() + duration_s
    speeds: list[int] = []
    missed = 0
    last_feedback_ts = float(getattr(motor, "_last_feedback_monotonic", 0.0))
    sample_index = 0

    while time.monotonic() < t_end:
        transport_fault = getattr(motor, "_transport_fault", None)
        if transport_fault is not None:
            raise RuntimeError(f"Motor transport fault during cmd={command_erpm:+d}: {transport_fault}")

        status = read_status(motor, timeout=min(0.08, period_s))
        feedback_ts = float(getattr(motor, "_last_feedback_monotonic", 0.0))
        fresh_feedback = status is not None and feedback_ts > last_feedback_ts
        if not fresh_feedback:
            missed += 1
            if missed > max_missed_feedback:
                raise RuntimeError(
                    f"No fresh feedback for {missed} consecutive samples during cmd={command_erpm:+d}"
                )
            time.sleep(period_s)
            continue

        last_feedback_ts = feedback_ts
        missed = 0
        assert status is not None
        if status.error_code != 0:
            raise RuntimeError(
                f"Motor fault during cmd={command_erpm:+d}: "
                f"{status.error_code} ({status.error_description})"
            )

        speeds.append(status.speed_erpm)
        sample_index += 1
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
                    "phase_command_erpm": command_erpm,
                    "phase_duration_s": f"{duration_s:.6f}",
                    "sample_index": sample_index,
                    "command_erpm": command_erpm,
                    "feedback_position_deg": f"{status.position_degrees:.6f}",
                    "feedback_speed_erpm": status.speed_erpm,
                    "feedback_current_amps": f"{status.current_amps:.6f}",
                    "feedback_temperature_c": status.temperature_celsius,
                    "feedback_error_code": status.error_code,
                    "feedback_error_description": status.error_description,
                }
            )
        time.sleep(period_s)

    abs_threshold = abs(min_informative_erpm)
    informative = [value for value in speeds if abs(value) >= abs_threshold]
    target_sign = _sign(command_erpm)

    sign_match_ratio: float | None = None
    pass_phase = True
    if target_sign != 0:
        if not informative:
            pass_phase = False
        else:
            matches = sum(1 for value in informative if _sign(value) == target_sign)
            sign_match_ratio = matches / len(informative)
            pass_phase = (
                len(informative) >= min_informative_samples
                and sign_match_ratio >= min_sign_match_ratio
            )
    else:
        # Neutral phase: should not spin aggressively.
        pass_phase = all(abs(value) < 2500 for value in speeds)

    return PhaseResult(
        command_erpm=command_erpm,
        samples_total=len(speeds),
        informative_samples=len(informative),
        sign_match_ratio=sign_match_ratio,
        mean_speed_erpm=_mean([float(value) for value in speeds]),
        peak_abs_speed_erpm=max((abs(value) for value in speeds), default=0),
        pass_phase=pass_phase,
    )


def print_phase_result(result: PhaseResult) -> None:
    """Print one concise result line."""
    ratio = (
        f"{100.0 * result.sign_match_ratio:5.1f}%"
        if result.sign_match_ratio is not None
        else "-"
    )
    print(
        f"cmd={result.command_erpm:+7d} ERPM | "
        f"samples={result.samples_total:3d} | "
        f"informative={result.informative_samples:3d} | "
        f"sign_match={ratio:>6} | "
        f"mean={result.mean_speed_erpm if result.mean_speed_erpm is not None else 0:8.1f} ERPM | "
        f"peak|v|={result.peak_abs_speed_erpm:6d} ERPM | "
        f"{'PASS' if result.pass_phase else 'FAIL'}"
    )


def main() -> int:  # noqa: C901, PLR0912, PLR0915
    """Run set_velocity verification."""
    args = parse_args()
    validate_args(args)

    velocity_cmd = int(args.velocity_erpm)
    sequence: list[tuple[int, float]] = [(velocity_cmd, args.phase_seconds)]
    if args.neutral_seconds > 0:
        sequence.append((0, args.neutral_seconds))
    if not args.forward_only:
        sequence.append((-velocity_cmd, args.phase_seconds))
        if args.neutral_seconds > 0:
            sequence.append((0, args.neutral_seconds))

    csv_path = _resolve_csv_path(args.csv_path, prefix="verify_set_velocity")

    print(SEPARATOR)
    print("Verify set_velocity()")
    print(SEPARATOR)
    print(f"Interface      : {args.interface}")
    print(f"Bitrate        : {args.bitrate}")
    print(f"Motor ID       : 0x{args.motor_id:02X}")
    print(f"Helper policy  : {args.helper_policy}")
    print(f"Legacy IDs     : {args.allow_legacy_feedback_ids}")
    if args.feedback_can_id is not None:
        print(f"Feedback CAN ID: 0x{args.feedback_can_id:08X}")
    print(f"Velocity start : {velocity_cmd:+d} ERPM")
    print(f"Velocity range : [{VERIFY_VELOCITY_MIN_ERPM}, {VERIFY_VELOCITY_MAX_ERPM}] ERPM")
    print(f"Velocity KD    : {args.velocity_kd:.3f}")
    print(f"Forward only   : {args.forward_only}")
    print(f"Sequence       : {sequence}")
    print(f"Preflight mode : {args.preflight_mode}")
    print(f"CSV log        : {csv_path}")
    print(SEPARATOR)

    ensure_can_ready(args.interface, bitrate=args.bitrate, mode=args.preflight_mode)

    motor: CubeMarsAK606v3CAN | None = None
    csv_file = None
    csv_writer: csv.DictWriter | None = None
    run_start = 0.0
    results: list[PhaseResult] = []
    try:
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = csv_path.open("w", newline="", encoding="utf-8")
        csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
        csv_writer.writeheader()

        def write_sample_row(row: dict[str, str | int]) -> None:
            if csv_writer is None or csv_file is None:
                return
            csv_writer.writerow(row)
            csv_file.flush()

        motor = CubeMarsAK606v3CAN(
            motor_can_id=args.motor_id,
            interface=args.interface,
            bitrate=args.bitrate,
            mit_velocity_kd=args.velocity_kd,
            helper_policy=args.helper_policy,
            allow_legacy_feedback_ids=args.allow_legacy_feedback_ids,
            feedback_can_id=args.feedback_can_id,
        )
        if not motor.connected:
            print("FAIL: could not connect to CAN motor interface")
            return 1

        if not motor.check_communication():
            print("FAIL: communication check failed (no feedback)")
            return 1

        print("PASS: communication check")
        print("\nRunning verification phases...")
        run_start = time.monotonic()

        for phase_index, (command_erpm, duration_s) in enumerate(sequence, start=1):
            print(f"\nPhase: cmd={command_erpm:+d} ERPM for {duration_s:.2f} s")
            result = run_phase(
                motor,
                phase_index=phase_index,
                command_erpm=command_erpm,
                duration_s=duration_s,
                sample_hz=args.sample_hz,
                min_informative_erpm=args.min_informative_erpm,
                min_sign_match_ratio=args.min_sign_match_ratio,
                min_informative_samples=args.min_informative_samples,
                max_missed_feedback=args.max_missed_feedback,
                run_start=run_start,
                sample_logger=write_sample_row,
            )
            results.append(result)
            print_phase_result(result)

        failed = [result for result in results if not result.pass_phase]
        print(f"\n{SEPARATOR}")
        if failed:
            print("FAIL: set_velocity verification failed")
            for result in failed:
                print_phase_result(result)
            print(SEPARATOR)
            print(f"CSV saved to: {csv_path}")
            return 1

        print("PASS: set_velocity verification passed")
        print(SEPARATOR)
        print(f"CSV saved to: {csv_path}")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 130
    except Exception as exc:
        print(f"\nFAIL: {exc}")
        return 1
    finally:
        if csv_file is not None:
            try:
                csv_file.close()
            except Exception as exc:  # pragma: no cover - cleanup path
                print(f"WARN: CSV close failed during cleanup: {exc}")
        if motor is not None:
            try:
                motor.close()
            except Exception as exc:  # pragma: no cover - cleanup path
                print(f"WARN: close() failed during cleanup: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
