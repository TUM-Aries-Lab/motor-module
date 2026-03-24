#!/usr/bin/env python3
"""MIT velocity step test for the motion-capture rig.

This script validates the current MIT-backed CAN velocity helper by running a
repeatable step sequence and logging:

- commanded ERPM
- motor feedback (position, ERPM, current, temperature, fault)
- optional motion-capture angle / velocity samples

Out of the box the script runs with motor telemetry only. To add live mocap,
pass a factory with ``--mocap-factory module:function``. The factory may accept
zero arguments or one ``argparse.Namespace`` argument and must return an object
with a ``sample()`` method. Optional ``start()`` and ``stop()`` methods are
also supported.

Accepted mocap sample formats:

1. ``None``
2. ``MocapSample(angle_deg=..., velocity_deg_s=..., source_time_s=...)``
3. ``dict`` with keys:
   - ``angle_deg`` (required for angle-based logging)
   - ``velocity_deg_s`` (optional, script derives it if absent)
   - ``timestamp_s`` or ``source_time_s`` (optional)
   - ``quality`` (optional)

Example adapter factory:

    def build_adapter(args):
        class MyAdapter:
            def start(self):
                ...

            def sample(self):
                return {"angle_deg": read_angle_deg()}

            def stop(self):
                ...

        return MyAdapter()

Run:
    sudo ./setup_can.sh
    .venv/bin/python scripts/mocap_velocity_test.py --motor-id 0x03
"""
# ruff: noqa: T201

from __future__ import annotations

import argparse
import csv
import importlib
import inspect
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

from motor_python.base_motor import MotorState
from motor_python.can_utils import get_can_state, reset_can_interface
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN
from motor_python.definitions import CAN_DEFAULTS

SEPARATOR = "=" * 84
DEFAULT_SEQUENCE = "0:1.5,5000:2.5,7000:2.5,0:1.5,-5000:2.5,-7000:2.5,0:1.5"
DEFAULT_LOG_DIR = Path(__file__).parent.parent / "data" / "logs"
HEALTHY_TX_ERR_MAX = 96
HEALTHY_RX_ERR_MAX = 64


@dataclass(frozen=True)
class VelocityPhase:
    """One commanded-velocity phase."""

    index: int
    command_erpm: int
    hold_s: float
    auto_generated: bool = False

    @property
    def label(self) -> str:
        """Short CSV/summary label."""
        if self.auto_generated and self.command_erpm == 0:
            return f"phase_{self.index:02d}_auto_neutral"
        sign = "+" if self.command_erpm >= 0 else "-"
        return f"phase_{self.index:02d}_{sign}{abs(self.command_erpm):05d}erpm"


@dataclass
class MocapSample:
    """Optional motion-capture sample."""

    angle_deg: float | None = None
    velocity_deg_s: float | None = None
    source_time_s: float | None = None
    quality: float | None = None


@dataclass
class LoggedSample:
    """One combined motor + mocap row."""

    wall_time_iso: str
    run_time_s: float
    phase_index: int
    phase_label: str
    phase_elapsed_s: float
    command_erpm: int
    motor_position_deg: float | None
    motor_speed_erpm: int | None
    motor_mech_deg_s: float | None
    motor_current_amps: float | None
    motor_temp_c: int | None
    motor_error_code: int | None
    motor_error_description: str | None
    mocap_angle_deg: float | None
    mocap_velocity_deg_s: float | None
    mocap_source_time_s: float | None
    mocap_quality: float | None


@dataclass(frozen=True)
class TimingSample:
    """Timing values associated with one logged row."""

    wall_time_iso: str
    run_time_s: float
    phase_elapsed_s: float


@dataclass
class RunContext:
    """Mutable state shared across the execution helpers."""

    args: argparse.Namespace
    mocap_adapter: Any
    mocap_estimator: AngleVelocityEstimator
    writer: csv.DictWriter
    rows: list[LoggedSample]
    run_start_monotonic: float
    loop_period_s: float


class NullMocapAdapter:
    """Default adapter used when no live mocap source is configured."""

    def start(self) -> None:
        """No-op startup hook."""

    def sample(self) -> None:
        """Return no mocap data."""
        return None

    def stop(self) -> None:
        """No-op teardown hook."""


class AngleVelocityEstimator:
    """Finite-difference angle -> velocity estimator with light EMA smoothing."""

    def __init__(self, alpha: float) -> None:
        self.alpha = alpha
        self._last_angle_deg: float | None = None
        self._last_time_s: float | None = None
        self._last_velocity_deg_s: float | None = None

    def prime(self, angle_deg: float, time_s: float) -> None:
        """Update position/time state without computing a new velocity sample."""
        self._last_angle_deg = angle_deg
        self._last_time_s = time_s

    def update(self, angle_deg: float, time_s: float) -> float | None:
        """Return filtered angular velocity, or None on the first sample."""
        if self._last_angle_deg is None or self._last_time_s is None:
            self.prime(angle_deg, time_s)
            return None

        dt = time_s - self._last_time_s
        if dt <= 1e-6:
            self.prime(angle_deg, time_s)
            return self._last_velocity_deg_s

        raw_velocity = (angle_deg - self._last_angle_deg) / dt
        if self._last_velocity_deg_s is None:
            velocity = raw_velocity
        else:
            velocity = (
                self.alpha * raw_velocity
                + (1.0 - self.alpha) * self._last_velocity_deg_s
            )

        self._last_velocity_deg_s = velocity
        self.prime(angle_deg, time_s)
        return velocity


def parse_sequence(sequence: str) -> list[VelocityPhase]:
    """Parse ``command:seconds`` pairs into an ordered phase list."""
    phases: list[VelocityPhase] = []
    for index, raw_item in enumerate(sequence.split(","), start=1):
        item = raw_item.strip()
        if not item:
            continue
        try:
            command_text, hold_text = item.split(":", maxsplit=1)
            phase = VelocityPhase(
                index=index,
                command_erpm=int(command_text.strip()),
                hold_s=float(hold_text.strip()),
            )
        except ValueError as exc:
            raise ValueError(
                f"Invalid phase '{item}'. Expected 'command_erpm:hold_seconds'."
            ) from exc
        if phase.hold_s <= 0.0:
            raise ValueError(f"Phase '{item}' has non-positive hold time.")
        phases.append(phase)

    if not phases:
        raise ValueError("Velocity sequence is empty.")
    return phases


def expand_transition_phases(
    phases: list[VelocityPhase],
    *,
    enable_auto_neutral: bool,
    neutral_hold_s: float,
) -> list[VelocityPhase]:
    """Insert short neutral phases before sign flips to reduce jerk/spin risk."""
    if not enable_auto_neutral or neutral_hold_s <= 0.0:
        return [
            VelocityPhase(
                index=index,
                command_erpm=phase.command_erpm,
                hold_s=phase.hold_s,
                auto_generated=False,
            )
            for index, phase in enumerate(phases, start=1)
        ]

    expanded: list[VelocityPhase] = []
    next_index = 1
    previous_nonzero_command: int | None = None

    for phase in phases:
        command = phase.command_erpm
        if (
            previous_nonzero_command is not None
            and command != 0
            and ((previous_nonzero_command > 0 > command) or (previous_nonzero_command < 0 < command))
        ):
            expanded.append(
                VelocityPhase(
                    index=next_index,
                    command_erpm=0,
                    hold_s=neutral_hold_s,
                    auto_generated=True,
                )
            )
            next_index += 1

        expanded.append(
            VelocityPhase(
                index=next_index,
                command_erpm=command,
                hold_s=phase.hold_s,
                auto_generated=False,
            )
        )
        next_index += 1

        if command != 0:
            previous_nonzero_command = command

    return expanded


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description="MIT velocity step test with optional motion-capture logging"
    )
    parser.add_argument("--interface", default="can0", help="SocketCAN interface")
    parser.add_argument(
        "--motor-id",
        type=lambda value: int(value, 0),
        default=0x03,
        help="Motor CAN ID, decimal or hex (default: 0x03)",
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
        "--sequence",
        default=DEFAULT_SEQUENCE,
        help=(
            "Comma-separated velocity phases as 'command_erpm:hold_seconds'. "
            f"Default: {DEFAULT_SEQUENCE}"
        ),
    )
    parser.add_argument(
        "--auto-neutral-on-sign-flip",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Insert an automatic 0-ERPM transition phase before +ERPM/-ERPM "
            "direction flips (default: true)."
        ),
    )
    parser.add_argument(
        "--transition-neutral-s",
        type=float,
        default=0.35,
        help=(
            "Hold time in seconds for auto-inserted neutral transition phases "
            "(default: 0.35)."
        ),
    )
    parser.add_argument(
        "--zero-mode",
        choices=("mit-hold", "stop"),
        default="mit-hold",
        help=(
            "How to handle 0 ERPM phases: 'mit-hold' keeps MIT velocity damping "
            "alive, 'stop' disables the motor between steps."
        ),
    )
    parser.add_argument(
        "--velocity-kd",
        type=float,
        default=CAN_DEFAULTS.mit_velocity_kd,
        help=f"MIT velocity damping KD used by set_velocity() (default: {CAN_DEFAULTS.mit_velocity_kd})",
    )
    parser.add_argument(
        "--sample-hz",
        type=float,
        default=50.0,
        help="Log/sample rate in Hz (default: 50)",
    )
    parser.add_argument(
        "--countdown-s",
        type=float,
        default=3.0,
        help="Countdown before the first phase begins (default: 3.0)",
    )
    parser.add_argument(
        "--prompt-before-start",
        action="store_true",
        help="Pause for Enter so you can start external mocap recording first.",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=DEFAULT_LOG_DIR,
        help="Directory for the output CSV",
    )
    parser.add_argument(
        "--tag",
        default="",
        help="Optional filename tag appended to the CSV name",
    )
    parser.add_argument(
        "--summary-tail-s",
        type=float,
        default=1.0,
        help="Tail window used for steady-state summary metrics (default: 1.0)",
    )
    parser.add_argument(
        "--sign-threshold-erpm",
        type=int,
        default=1000,
        help="Ignore motor sign checks below this |ERPM| threshold (default: 1000)",
    )
    parser.add_argument(
        "--sign-threshold-mocap-deg-s",
        type=float,
        default=2.0,
        help=(
            "Ignore mocap sign checks below this |deg/s| threshold "
            "(default: 2.0)"
        ),
    )
    parser.add_argument(
        "--min-sign-match-ratio",
        type=float,
        default=0.65,
        help="Minimum sign-match ratio for non-zero phases (default: 0.65).",
    )
    parser.add_argument(
        "--fail-on-sign-mismatch",
        action="store_true",
        help="Return non-zero exit code when non-zero phases fail sign checks.",
    )
    parser.add_argument(
        "--max-missed-feedback",
        type=int,
        default=10,
        help="Abort after this many consecutive missing motor samples (default: 10)",
    )
    parser.add_argument(
        "--max-temp-c",
        type=int,
        default=85,
        help="Abort if motor temperature exceeds this value (default: 85)",
    )
    parser.add_argument(
        "--max-current-a",
        type=float,
        default=15.0,
        help="Abort if |current| exceeds this value (default: 15.0)",
    )
    parser.add_argument(
        "--max-abs-mocap-angle",
        type=float,
        default=None,
        help="Optional safety limit for |mocap angle| in degrees",
    )
    parser.add_argument(
        "--mocap-factory",
        default="",
        help=(
            "Optional mocap adapter factory as 'module:function'. "
            "The function returns an object with sample()/start()/stop()."
        ),
    )
    parser.add_argument(
        "--mocap-velocity-alpha",
        type=float,
        default=0.25,
        help=(
            "EMA smoothing factor used when deriving mocap velocity from angle "
            "(default: 0.25)"
        ),
    )
    parser.add_argument(
        "--preflight-mode",
        choices=("strict", "auto", "skip"),
        default="strict",
        help=(
            "CAN preflight strategy: strict=fail if unhealthy, "
            "auto=try `sudo ./setup_can.sh`, skip=do not gate run (default: strict)."
        ),
    )
    parser.add_argument(
        "--bus-health-check-seconds",
        type=float,
        default=0.75,
        help="Runtime period for `ip` CAN-state checks during phases (default: 0.75).",
    )
    parser.add_argument(
        "--tx-err-abort-threshold",
        type=int,
        default=128,
        help="Abort when CAN tx error counter reaches this value (default: 128).",
    )
    parser.add_argument(
        "--rx-err-abort-threshold",
        type=int,
        default=96,
        help="Abort when CAN rx error counter reaches this value (default: 96).",
    )
    return parser.parse_args()



def validate_args(args: argparse.Namespace) -> None:  # noqa: C901
    """Validate numeric runtime arguments."""
    if args.bitrate <= 0:
        raise ValueError("--bitrate must be > 0.")
    if args.sample_hz <= 0:
        raise ValueError("--sample-hz must be > 0.")
    if args.velocity_kd < 0:
        raise ValueError("--velocity-kd must be >= 0.")
    if args.countdown_s < 0:
        raise ValueError("--countdown-s must be >= 0.")
    if args.transition_neutral_s < 0:
        raise ValueError("--transition-neutral-s must be >= 0.")
    if args.bus_health_check_seconds <= 0:
        raise ValueError("--bus-health-check-seconds must be > 0.")
    if args.tx_err_abort_threshold < 0:
        raise ValueError("--tx-err-abort-threshold must be >= 0.")
    if args.rx_err_abort_threshold < 0:
        raise ValueError("--rx-err-abort-threshold must be >= 0.")
    if args.max_missed_feedback < 1:
        raise ValueError("--max-missed-feedback must be >= 1.")
    if not 0.0 <= args.min_sign_match_ratio <= 1.0:
        raise ValueError("--min-sign-match-ratio must be in [0, 1].")


def load_mocap_adapter(args: argparse.Namespace) -> Any:
    """Create the optional mocap adapter object."""
    if not args.mocap_factory:
        return NullMocapAdapter()

    module_name, sep, attr_name = args.mocap_factory.partition(":")
    if not sep or not module_name or not attr_name:
        raise ValueError(
            "--mocap-factory must look like 'module:function', "
            f"got '{args.mocap_factory}'."
        )

    module = importlib.import_module(module_name)
    factory = getattr(module, attr_name)
    signature = inspect.signature(factory)
    parameter_count = len(signature.parameters)

    if parameter_count == 0:
        adapter = factory()
    elif parameter_count == 1:
        adapter = factory(args)
    else:
        raise TypeError(
            f"Mocap factory '{args.mocap_factory}' must accept 0 or 1 arguments, "
            f"found {parameter_count}."
        )

    if not hasattr(adapter, "sample"):
        raise TypeError(
            f"Mocap adapter from '{args.mocap_factory}' has no sample() method."
        )
    return adapter


def normalize_mocap_sample(raw_sample: Any, default_time_s: float) -> MocapSample:
    """Convert adapter output into a ``MocapSample``."""
    if raw_sample is None:
        return MocapSample()

    if isinstance(raw_sample, MocapSample):
        sample = raw_sample
    elif isinstance(raw_sample, dict):
        source_time_s = raw_sample.get("source_time_s", raw_sample.get("timestamp_s"))
        sample = MocapSample(
            angle_deg=_coerce_optional_float(raw_sample.get("angle_deg")),
            velocity_deg_s=_coerce_optional_float(raw_sample.get("velocity_deg_s")),
            source_time_s=_coerce_optional_float(source_time_s),
            quality=_coerce_optional_float(raw_sample.get("quality")),
        )
    else:
        raise TypeError(
            "Mocap sample must be None, MocapSample, or dict; "
            f"got {type(raw_sample).__name__}."
        )

    if sample.source_time_s is None:
        sample.source_time_s = default_time_s
    return sample


def _coerce_optional_float(value: Any) -> float | None:
    """Return ``value`` as float, or None."""
    if value is None or value == "":
        return None
    return float(value)


def read_status(motor: CubeMarsAK606v3CAN, timeout: float) -> MotorState | None:
    """Read the freshest feedback sample without blocking for the full default timeout."""
    status = motor._receive_feedback(timeout=timeout)
    if status is not None:
        return status
    return motor._last_feedback


def command_velocity(
    motor: CubeMarsAK606v3CAN,
    phase: VelocityPhase,
    *,
    zero_mode: str,
) -> None:
    """Apply one phase command using the MIT velocity helper."""
    if phase.command_erpm == 0:
        use_stop_mode = zero_mode == "stop" and not phase.auto_generated
        if use_stop_mode:
            motor.stop()
        else:
            motor.set_mit_mode(
                pos_rad=0.0,
                vel_rad_s=0.0,
                kp=0.0,
                kd=CAN_DEFAULTS.mit_velocity_kd,
                torque_ff_nm=0.0,
            )
        return

    motor.set_velocity(phase.command_erpm)


def _is_can_state_healthy(state: dict[str, Any]) -> bool:
    """Return True when CAN state is healthy for controlled bench runs."""
    return (
        state["state"] == "ERROR-ACTIVE"
        and int(state["tx_err"]) < HEALTHY_TX_ERR_MAX
        and int(state["rx_err"]) < HEALTHY_RX_ERR_MAX
    )


def ensure_can_ready(interface: str, bitrate: int, *, preflight_mode: str) -> None:
    """Validate/repair CAN state according to the requested preflight policy."""
    state = get_can_state(interface)
    print(
        f"CAN preflight: state={state['state']} tx_err={state['tx_err']} rx_err={state['rx_err']}"
    )
    if _is_can_state_healthy(state):
        return
    if preflight_mode == "skip":
        print("CAN preflight: skipping health gate by request (--preflight-mode skip).")
        return
    if preflight_mode == "strict":
        raise RuntimeError(
            "CAN interface is not healthy enough for a controlled bench run. "
            "Run `sudo ./setup_can.sh` and verify ERROR-ACTIVE before retrying."
        )

    print("CAN preflight: attempting automatic kernel-level CAN reset ...")
    if not reset_can_interface(interface=interface, bitrate=bitrate):
        raise RuntimeError(
            "CAN preflight auto-reset failed. Run `sudo ./setup_can.sh` manually and retry."
        )

    after = get_can_state(interface)
    print(
        f"CAN preflight after reset: state={after['state']} tx_err={after['tx_err']} rx_err={after['rx_err']}"
    )
    if not _is_can_state_healthy(after):
        raise RuntimeError(
            "CAN still unhealthy after auto-reset. "
            "Check power, wiring/termination, and UART disconnect."
        )


def enforce_runtime_bus_health(args: argparse.Namespace, *, where: str) -> None:
    """Abort the run once the CAN controller enters an unhealthy state."""
    state = get_can_state(args.interface)
    tx_err = int(state["tx_err"])
    rx_err = int(state["rx_err"])
    unhealthy = (
        state["state"] != "ERROR-ACTIVE"
        or tx_err >= args.tx_err_abort_threshold
        or rx_err >= args.rx_err_abort_threshold
    )
    if unhealthy:
        raise RuntimeError(
            "CAN degraded during run "
            f"({where}): state={state['state']} tx_err={tx_err} rx_err={rx_err}. "
            "Stopping for safety. Reset CAN and verify motor ACK/power/UART before retry."
        )


def countdown(seconds: float) -> None:
    """Print a short countdown before motion starts."""
    total = max(0, round(seconds))
    if total <= 0:
        return

    for remaining in range(total, 0, -1):
        print(f"Starting in {remaining}...")
        time.sleep(1.0)


def sample_to_csv_row(sample: LoggedSample) -> dict[str, Any]:
    """Serialize one in-memory sample to a CSV row."""
    return {
        "wall_time_iso": sample.wall_time_iso,
        "run_time_s": f"{sample.run_time_s:.6f}",
        "phase_index": sample.phase_index,
        "phase_label": sample.phase_label,
        "phase_elapsed_s": f"{sample.phase_elapsed_s:.6f}",
        "command_erpm": sample.command_erpm,
        "motor_position_deg": _csv_value(sample.motor_position_deg),
        "motor_speed_erpm": _csv_value(sample.motor_speed_erpm),
        "motor_mech_deg_s": _csv_value(sample.motor_mech_deg_s),
        "motor_current_amps": _csv_value(sample.motor_current_amps),
        "motor_temp_c": _csv_value(sample.motor_temp_c),
        "motor_error_code": _csv_value(sample.motor_error_code),
        "motor_error_description": sample.motor_error_description or "",
        "mocap_angle_deg": _csv_value(sample.mocap_angle_deg),
        "mocap_velocity_deg_s": _csv_value(sample.mocap_velocity_deg_s),
        "mocap_source_time_s": _csv_value(sample.mocap_source_time_s),
        "mocap_quality": _csv_value(sample.mocap_quality),
    }


def _csv_value(value: Any) -> str:
    """Return a CSV-safe string value."""
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def check_safety(
    args: argparse.Namespace,
    status: MotorState | None,
    mocap: MocapSample,
) -> None:
    """Abort on unsafe motor or rig state."""
    if status is not None:
        if status.error_code != 0:
            raise RuntimeError(
                f"Motor fault code {status.error_code}: {status.error_description}"
            )
        if status.temperature_celsius > args.max_temp_c:
            raise RuntimeError(
                f"Motor temperature {status.temperature_celsius} C exceeded --max-temp-c={args.max_temp_c}"
            )
        if abs(status.current_amps) > args.max_current_a:
            raise RuntimeError(
                f"Motor current {status.current_amps:.2f} A exceeded --max-current-a={args.max_current_a:.2f}"
            )

    if (
        args.max_abs_mocap_angle is not None
        and mocap.angle_deg is not None
        and abs(mocap.angle_deg) > args.max_abs_mocap_angle
    ):
        raise RuntimeError(
            f"Mocap angle {mocap.angle_deg:.2f} deg exceeded --max-abs-mocap-angle={args.max_abs_mocap_angle:.2f}"
        )


def summarize_phase(
    phase: VelocityPhase,
    rows: list[LoggedSample],
    args: argparse.Namespace,
) -> str:
    """Return one printable summary line for a phase."""
    tail_start = max(0.0, phase.hold_s - args.summary_tail_s)
    tail_rows = [row for row in rows if row.phase_elapsed_s >= tail_start] or rows

    mean_motor = _mean(
        row.motor_speed_erpm for row in tail_rows if row.motor_speed_erpm is not None
    )
    mean_mocap = _mean(
        row.mocap_velocity_deg_s
        for row in tail_rows
        if row.mocap_velocity_deg_s is not None
    )
    peak_current = _peak_abs(
        row.motor_current_amps for row in rows if row.motor_current_amps is not None
    )
    peak_temp = _peak_abs(
        float(row.motor_temp_c) for row in rows if row.motor_temp_c is not None
    )
    motor_sign = _sign_match_ratio(
        target_sign=_sign(phase.command_erpm),
        values=(row.motor_speed_erpm for row in rows if row.motor_speed_erpm is not None),
        min_abs_value=float(args.sign_threshold_erpm),
    )
    mocap_sign = _sign_match_ratio(
        target_sign=_sign(phase.command_erpm),
        values=(
            row.mocap_velocity_deg_s
            for row in rows
            if row.mocap_velocity_deg_s is not None
        ),
        min_abs_value=args.sign_threshold_mocap_deg_s,
    )

    return (
        f"{phase.label:<22s}  cmd={phase.command_erpm:>7d}  "
        f"motor_tail={_fmt_float(mean_motor, 0):>8s} ERPM  "
        f"mocap_tail={_fmt_float(mean_mocap, 1):>8s} deg/s  "
        f"peak|I|={_fmt_float(peak_current, 2):>6s} A  "
        f"peakT={_fmt_float(peak_temp, 0):>4s} C  "
        f"motor_sign={_fmt_ratio(motor_sign):>6s}  "
        f"mocap_sign={_fmt_ratio(mocap_sign):>6s}"
    )


def _mean(values: Any) -> float | None:
    """Return mean(values) or None."""
    data = [float(value) for value in values]
    if not data:
        return None
    return sum(data) / len(data)


def _peak_abs(values: Any) -> float | None:
    """Return max(abs(values)) or None."""
    data = [abs(float(value)) for value in values]
    if not data:
        return None
    return max(data)


def _sign(value: float) -> int:
    """Return -1, 0, or +1."""
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


def _sign_match_ratio(
    *,
    target_sign: int,
    values: Any,
    min_abs_value: float,
) -> float | None:
    """Return fraction of informative samples that match the command sign."""
    if target_sign == 0:
        return None

    informative = [float(value) for value in values if abs(float(value)) >= min_abs_value]
    if not informative:
        return None

    matches = sum(1 for value in informative if _sign(value) == target_sign)
    return matches / len(informative)


def _fmt_float(value: float | None, digits: int) -> str:
    """Pretty-print an optional float."""
    if value is None:
        return "-"
    return f"{value:.{digits}f}"


def _fmt_ratio(value: float | None) -> str:
    """Pretty-print an optional percentage ratio."""
    if value is None:
        return "-"
    return f"{100.0 * value:5.1f}%"


def print_summary(
    phases: list[VelocityPhase],
    rows: list[LoggedSample],
    args: argparse.Namespace,
) -> None:
    """Print a concise post-run summary."""
    print(f"\n{SEPARATOR}")
    print("Velocity Test Summary")
    print(SEPARATOR)
    for phase in phases:
        phase_rows = [row for row in rows if row.phase_index == phase.index]
        if not phase_rows:
            print(f"{phase.label:<22s}  no samples captured")
            continue
        print(summarize_phase(phase, phase_rows, args))
    print(SEPARATOR)


def collect_sign_check_failures(
    phases: list[VelocityPhase],
    rows: list[LoggedSample],
    args: argparse.Namespace,
) -> list[str]:
    """Return user-facing sign-check failures for non-zero phases."""
    failures: list[str] = []
    for phase in phases:
        if phase.command_erpm == 0:
            continue
        phase_rows = [row for row in rows if row.phase_index == phase.index]
        if not phase_rows:
            failures.append(f"{phase.label}: no samples captured")
            continue
        ratio = _sign_match_ratio(
            target_sign=_sign(phase.command_erpm),
            values=(
                row.motor_speed_erpm
                for row in phase_rows
                if row.motor_speed_erpm is not None
            ),
            min_abs_value=float(args.sign_threshold_erpm),
        )
        if ratio is None or ratio < args.min_sign_match_ratio:
            failures.append(
                f"{phase.label}: motor sign-match={_fmt_ratio(ratio)} "
                f"(required >= {100.0 * args.min_sign_match_ratio:.1f}%)"
            )
    return failures


def build_log_path(log_dir: Path, tag: str) -> Path:
    """Return the CSV path for this run."""
    log_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    suffix = f"_{tag.strip()}" if tag.strip() else ""
    return log_dir / f"mocap_velocity_test_{timestamp}{suffix}.csv"


def print_run_header(
    args: argparse.Namespace,
    log_path: Path,
    phases: list[VelocityPhase],
) -> None:
    """Print the configuration summary before motion starts."""
    print(SEPARATOR)
    print("MIT Velocity Step Test")
    print(SEPARATOR)
    print(f"Interface      : {args.interface}")
    print(f"Motor ID       : 0x{args.motor_id:02X}")
    print(f"Bitrate        : {args.bitrate}")
    print(f"Helper policy  : {args.helper_policy}")
    print(f"Legacy IDs     : {args.allow_legacy_feedback_ids}")
    if args.feedback_can_id is not None:
        print(f"Feedback CAN ID: 0x{args.feedback_can_id:08X}")
    print(f"Sequence raw   : {args.sequence}")
    print(f"Phase count    : {len(phases)}")
    print(
        f"Auto-neutral   : {args.auto_neutral_on_sign_flip} "
        f"(hold {args.transition_neutral_s:.2f} s)"
    )
    print(f"Zero mode      : {args.zero_mode}")
    print(f"Sample rate    : {args.sample_hz:.1f} Hz")
    print(f"Velocity KD    : {args.velocity_kd:.3f}")
    print(f"Preflight mode : {args.preflight_mode}")
    print(
        f"Sign criteria  : threshold={args.sign_threshold_erpm} ERPM "
        f"ratio>={args.min_sign_match_ratio:.2f} fail={args.fail_on_sign_mismatch}"
    )
    print(
        "Bus guard      : "
        f"every {args.bus_health_check_seconds:.2f}s, "
        f"tx>={args.tx_err_abort_threshold}, rx>={args.rx_err_abort_threshold}"
    )
    print(f"Mocap factory  : {args.mocap_factory or 'none (motor telemetry only)'}")
    print(f"CSV            : {log_path}")
    print(SEPARATOR)


def print_phase_plan(phases: list[VelocityPhase]) -> None:
    """Print the fully expanded execution plan."""
    print("Expanded phase plan:")
    for phase in phases:
        auto_note = " [auto-neutral]" if phase.auto_generated else ""
        print(
            f"  - [{phase.index:02d}] {phase.command_erpm:+7d} ERPM "
            f"for {phase.hold_s:.2f} s{auto_note}"
        )
    print(SEPARATOR)


def maybe_wait_for_start(args: argparse.Namespace) -> None:
    """Pause for operator confirmation before the countdown when requested."""
    if args.prompt_before_start:
        input("Start your external motion-capture recording, then press Enter to continue...")
    else:
        print("Start external motion-capture recording now if you are logging it separately.")
    countdown(args.countdown_s)


def build_fieldnames() -> list[str]:
    """Return CSV fieldnames."""
    return [
        "wall_time_iso",
        "run_time_s",
        "phase_index",
        "phase_label",
        "phase_elapsed_s",
        "command_erpm",
        "motor_position_deg",
        "motor_speed_erpm",
        "motor_mech_deg_s",
        "motor_current_amps",
        "motor_temp_c",
        "motor_error_code",
        "motor_error_description",
        "mocap_angle_deg",
        "mocap_velocity_deg_s",
        "mocap_source_time_s",
        "mocap_quality",
    ]


def create_logged_sample(
    context: RunContext,
    phase: VelocityPhase,
    timing: TimingSample,
    status: MotorState | None,
    mocap: MocapSample,
) -> LoggedSample:
    """Build one combined log sample."""
    return LoggedSample(
        wall_time_iso=timing.wall_time_iso,
        run_time_s=timing.run_time_s,
        phase_index=phase.index,
        phase_label=phase.label,
        phase_elapsed_s=timing.phase_elapsed_s,
        command_erpm=phase.command_erpm,
        motor_position_deg=status.position_degrees if status else None,
        motor_speed_erpm=status.speed_erpm if status else None,
        motor_mech_deg_s=(
            (status.speed_erpm / CAN_DEFAULTS.motor_pole_pairs) * 6.0
            if status is not None
            else None
        ),
        motor_current_amps=status.current_amps if status else None,
        motor_temp_c=status.temperature_celsius if status else None,
        motor_error_code=status.error_code if status else None,
        motor_error_description=status.error_description if status is not None else None,
        mocap_angle_deg=mocap.angle_deg,
        mocap_velocity_deg_s=mocap.velocity_deg_s,
        mocap_source_time_s=mocap.source_time_s,
        mocap_quality=mocap.quality,
    )


def read_mocap_sample(context: RunContext, *, now: float) -> MocapSample:
    """Read one mocap sample and derive velocity when needed."""
    mocap_raw = context.mocap_adapter.sample()
    mocap = normalize_mocap_sample(
        mocap_raw, default_time_s=now - context.run_start_monotonic
    )
    if mocap.angle_deg is None:
        return mocap

    source_time_s = (
        mocap.source_time_s
        if mocap.source_time_s is not None
        else now - context.run_start_monotonic
    )
    if mocap.velocity_deg_s is None:
        mocap.velocity_deg_s = context.mocap_estimator.update(
            mocap.angle_deg, source_time_s
        )
    else:
        context.mocap_estimator.prime(mocap.angle_deg, source_time_s)
    return mocap


def run_phase(
    motor: CubeMarsAK606v3CAN,
    phase: VelocityPhase,
    context: RunContext,
) -> None:
    """Execute one velocity phase and append CSV rows."""
    auto_note = " [auto-neutral]" if phase.auto_generated else ""
    print(
        f"\n[{phase.index}] {phase.label}{auto_note}: "
        f"command {phase.command_erpm:+d} ERPM for {phase.hold_s:.2f} s"
    )
    command_velocity(
        motor,
        phase,
        zero_mode=context.args.zero_mode,
    )

    phase_start = time.monotonic()
    next_tick = phase_start
    next_bus_health_check = phase_start
    consecutive_missed_feedback = 0

    while True:
        now = time.monotonic()
        phase_elapsed_s = now - phase_start
        if phase_elapsed_s > phase.hold_s:
            return

        if now >= next_bus_health_check:
            enforce_runtime_bus_health(
                context.args,
                where=f"{phase.label} @ {phase_elapsed_s:.2f}s",
            )
            next_bus_health_check = now + context.args.bus_health_check_seconds

        transport_fault = getattr(motor, "_transport_fault", None)
        if transport_fault is not None:
            raise RuntimeError(f"Motor transport fault during {phase.label}: {transport_fault}")

        status = read_status(motor, timeout=min(0.08, context.loop_period_s))
        if status is None:
            consecutive_missed_feedback += 1
        else:
            consecutive_missed_feedback = 0

        if consecutive_missed_feedback > context.args.max_missed_feedback:
            raise RuntimeError(
                f"No motor feedback for {consecutive_missed_feedback} consecutive samples."
            )

        mocap = read_mocap_sample(context, now=now)
        check_safety(context.args, status, mocap)
        timing = TimingSample(
            wall_time_iso=datetime.now().isoformat(timespec="milliseconds"),
            run_time_s=now - context.run_start_monotonic,
            phase_elapsed_s=phase_elapsed_s,
        )

        row = create_logged_sample(
            context,
            phase,
            timing,
            status=status,
            mocap=mocap,
        )
        context.rows.append(row)
        context.writer.writerow(sample_to_csv_row(row))

        next_tick += context.loop_period_s
        wait_s = next_tick - time.monotonic()
        if wait_s > 0:
            time.sleep(wait_s)


def execute_run(
    args: argparse.Namespace,
    phases: list[VelocityPhase],
    log_path: Path,
    mocap_adapter: Any,
) -> list[LoggedSample]:
    """Run the full bench sequence and return captured rows."""
    rows: list[LoggedSample] = []
    motor: CubeMarsAK606v3CAN | None = None
    csv_file = None
    mocap_estimator = AngleVelocityEstimator(args.mocap_velocity_alpha)

    try:
        csv_file = open(log_path, "w", newline="", encoding="utf-8")
        writer = csv.DictWriter(csv_file, fieldnames=build_fieldnames())
        writer.writeheader()

        mocap_adapter.start()

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
            raise RuntimeError("CAN motor driver could not connect to the interface.")
        enforce_runtime_bus_health(args, where="post-connect")
        if not motor.check_communication():
            raise RuntimeError(
                "Motor did not respond to check_communication(). Check power, UART disconnect, and CAN ID."
            )
        enforce_runtime_bus_health(args, where="post-check_communication")

        motor.enable_mit_mode()
        context = RunContext(
            args=args,
            mocap_adapter=mocap_adapter,
            mocap_estimator=mocap_estimator,
            writer=writer,
            rows=rows,
            run_start_monotonic=time.monotonic(),
            loop_period_s=1.0 / max(args.sample_hz, 1.0),
        )

        for phase in phases:
            run_phase(motor, phase, context)

        print("\nTest sequence complete. Stopping motor...")
        return rows

    finally:
        try:
            mocap_adapter.stop()
        except Exception as exc:
            print(f"WARN: mocap_adapter.stop() failed: {exc}")
        if motor is not None:
            try:
                motor.close()
            except Exception as exc:
                print(f"WARN: motor.close() failed during cleanup: {exc}")
        if csv_file is not None:
            csv_file.close()


def main() -> int:
    """Run the velocity step test."""
    args = parse_args()
    validate_args(args)
    parsed_phases = parse_sequence(args.sequence)
    phases = expand_transition_phases(
        parsed_phases,
        enable_auto_neutral=args.auto_neutral_on_sign_flip,
        neutral_hold_s=args.transition_neutral_s,
    )
    ensure_can_ready(
        args.interface,
        bitrate=args.bitrate,
        preflight_mode=args.preflight_mode,
    )
    log_path = build_log_path(args.log_dir, args.tag)
    mocap_adapter = load_mocap_adapter(args)

    print_run_header(args, log_path, phases)
    print_phase_plan(phases)
    maybe_wait_for_start(args)

    try:
        rows = execute_run(args, phases, log_path, mocap_adapter)
    except KeyboardInterrupt:
        print("\nInterrupted by user. Stopping motor...")
        return 130
    except Exception as exc:
        print(f"\nFAIL: {exc}")
        return 1

    print_summary(phases, rows, args)
    if args.fail_on_sign_mismatch:
        failures = collect_sign_check_failures(phases, rows, args)
        if failures:
            print("FAIL: sign-check criteria not met")
            for item in failures:
                print(f"  - {item}")
            print(f"CSV saved to: {log_path}")
            return 1
    print(f"CSV saved to: {log_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
