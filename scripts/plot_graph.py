#!/usr/bin/env python3
# ruff: noqa: T201

from __future__ import annotations

import argparse
import csv
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

# Allow running as plain `python scripts/plot_ch4_validation.py` from repo root.
if __package__ in {None, ""}:
    repo_src = Path(__file__).resolve().parents[1] / "src"
    if str(repo_src) not in sys.path:
        sys.path.insert(0, str(repo_src))

FRAME_RATE_HZ = 200.0
AK60_6_MOTOR_POLE_PAIRS = 21
MECH_DEG_PER_SEC_PER_ERPM = 6.0 / AK60_6_MOTOR_POLE_PAIRS
POSITION_TARGETS = (30, 50, 90)
VELOCITY_SPEEDS = (1000, 3000, 5000)
POSITION_MOCAP_FILES = {
    30: "position30.csv",
    50: "position50.csv",
    90: "position90_new.csv",
}
VELOCITY_MOCAP_FILES = {
    1000: "velo1000.csv",
    3000: "velo3000.csv",
    5000: "velo5000.csv",
}
POSITION_MOTOR_PREFIX = "mit_position_steps"
VELOCITY_MOTOR_PREFIX = "verify_set_velocity"

COLOR_COMMAND = "#1f77b4"
COLOR_MOTOR = "#d62728"
COLOR_MOCAP = "#2ca02c"
COLOR_ERROR = "#7f7f7f"

POSITION_REQUIRED_COLUMNS = {
    "elapsed_s",
    "tick_index",
    "direction",
    "command_position_deg",
    "segment_target_deg",
    "feedback_received",
    "feedback_position_deg",
    "feedback_speed_erpm",
}
VELOCITY_REQUIRED_COLUMNS = {
    "elapsed_s",
    "phase_index",
    "phase_command_erpm",
    "command_erpm",
    "feedback_position_deg",
    "feedback_speed_erpm",
}


@dataclass(frozen=True)
class FilePair:
    """One motor/mocap input pair."""

    label: str
    motor_csv: Path
    mocap_csv: Path


@dataclass(frozen=True)
class MotorPositionData:
    """Parsed MIT position-step motor log."""

    elapsed_s: np.ndarray
    tick_index: np.ndarray
    direction: np.ndarray
    command_position_deg: np.ndarray
    segment_target_deg: np.ndarray
    feedback_received: np.ndarray
    feedback_position_deg: np.ndarray
    feedback_speed_erpm: np.ndarray


@dataclass(frozen=True)
class MotorVelocityData:
    """Parsed MIT velocity-validation motor log."""

    elapsed_s: np.ndarray
    phase_index: np.ndarray
    phase_command_erpm: np.ndarray
    command_erpm: np.ndarray
    feedback_position_deg: np.ndarray
    feedback_speed_erpm: np.ndarray
    motor_mech_deg_s: np.ndarray


@dataclass(frozen=True)
class RawMocapData:
    """Raw Vicon samples from RX/RY/RZ and TX/TY/TZ columns."""

    frame: np.ndarray
    sub_frame: np.ndarray
    rx_deg: np.ndarray
    ry_deg: np.ndarray
    rz_deg: np.ndarray
    tx_mm: np.ndarray
    ty_mm: np.ndarray
    tz_mm: np.ndarray


@dataclass(frozen=True)
class MocapSeries:
    """Processed mocap angle/velocity trace."""

    time_s: np.ndarray
    angle_deg: np.ndarray
    velocity_deg_s: np.ndarray
    frame: np.ndarray
    trim_frame: int
    selected_axis: str


@dataclass(frozen=True)
class PositionSegment:
    """One retained one-direction position window."""

    segment_id: int
    direction: int
    start_time_s: float
    end_time_s: float
    start_index: int
    end_index: int
    target_deg: float
    tick_start: int
    tick_end: int
    angle_span_deg: float


def _require_analysis_runtime() -> tuple[Any, Any]:
    """Return Rotation and pyplot, or raise a helpful runtime error."""
    try:
        from scipy.spatial.transform import Rotation  # noqa: PLC0415
    except Exception as exc:  # pragma: no cover - exercised in runtime only
        raise RuntimeError(
            "SciPy is required for raw mocap rotation handling. "
            "Install the analysis dependencies first, for example:\n"
            "  uv sync --group analysis"
        ) from exc

    try:
        import matplotlib  # noqa: PLC0415

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # noqa: PLC0415
    except Exception as exc:  # pragma: no cover - exercised in runtime only
        raise RuntimeError(
            "Matplotlib is required for Chapter 4 plot generation. "
            "Install the analysis dependencies first, for example:\n"
            "  uv sync --group analysis"
        ) from exc

    return Rotation, plt


def _parse_csv_with_required_columns(
    path: Path, required_columns: set[str]
) -> list[dict[str, str]]:
    """Load a CSV file and validate that the expected columns are present."""
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path} has no CSV header row.")
        missing = sorted(required_columns - set(reader.fieldnames))
        if missing:
            raise ValueError(f"{path} is missing required columns: {', '.join(missing)}")
        return list(reader)


def _to_float(value: str) -> float:
    """Convert CSV text to float."""
    return float(value.strip())


def _to_int(value: str) -> int:
    """Convert CSV text to integer."""
    return int(float(value.strip()))


def _moving_average(values: np.ndarray, window: int) -> np.ndarray:
    """Return a centered moving average with edge padding."""
    if window <= 1 or len(values) <= 2:
        return values.astype(float, copy=True)
    window = max(1, int(window))
    if window % 2 == 0:
        window += 1
    if len(values) < window:
        return values.astype(float, copy=True)
    pad_left = window // 2
    pad_right = window - 1 - pad_left
    padded = np.pad(values.astype(float), (pad_left, pad_right), mode="edge")
    kernel = np.ones(window, dtype=float) / float(window)
    return np.convolve(padded, kernel, mode="valid")


def _unwrap_degrees(angle_deg: np.ndarray) -> np.ndarray:
    """Unwrap degree-valued angles into a continuous trace."""
    return np.rad2deg(np.unwrap(np.deg2rad(angle_deg)))


def _parse_number_list(raw: str, *, kind: str) -> list[int]:
    """Parse a comma-separated integer list."""
    try:
        values = [int(part.strip()) for part in raw.split(",") if part.strip()]
    except ValueError as exc:
        raise ValueError(f"Invalid {kind} list: {raw!r}") from exc
    if not values:
        raise ValueError(f"No {kind} values were provided.")
    return values


def _time_shift_value(raw: str) -> str | float:
    """Parse --time-shift-s preserving the special 'auto' value."""
    if raw == "auto":
        return raw
    return float(raw)


def _trim_frame_value(raw: str) -> str | int:
    """Parse --trim-frame preserving the special 'auto' value."""
    if raw == "auto":
        return raw
    return int(raw)


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description="Generate Chapter 4 overlay plots from the curated CSV folder."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    def add_common(parser_obj: argparse.ArgumentParser) -> None:
        parser_obj.add_argument(
            "--data-root",
            type=Path,
            default=Path("CSV"),
            help="Root folder containing the curated Chapter 4 CSV files (default: CSV)",
        )
        parser_obj.add_argument(
            "--out-dir",
            type=Path,
            default=Path("data/plots/ch4"),
            help="Base output directory for thesis plots (default: data/plots/ch4)",
        )
        parser_obj.add_argument(
            "--trim-frame",
            type=_trim_frame_value,
            default="auto",
            help="First mocap frame to keep, or 'auto' (default: auto)",
        )
        parser_obj.add_argument(
            "--axis",
            choices=("auto", "rx", "ry", "rz"),
            default="auto",
            help="Aligned mocap axis used as the final angle trace (default: auto)",
        )
        parser_obj.add_argument(
            "--invert-mocap-sign",
            action="store_true",
            help="Invert the final mocap angle/velocity sign.",
        )
        parser_obj.add_argument(
            "--time-shift-s",
            type=_time_shift_value,
            default="auto",
            help="Shift applied to mocap time before alignment, or 'auto' (default: auto)",
        )

    position_parser = subparsers.add_parser(
        "position", help="Compare MIT position-step motor feedback against raw mocap."
    )
    add_common(position_parser)
    position_parser.add_argument(
        "--targets",
        default="30,50,90",
        help="Comma-separated position-step targets to process (default: 30,50,90)",
    )
    position_parser.add_argument(
        "--motor-csv",
        type=Path,
        default=None,
        help="Explicit motor CSV path for a single debug run.",
    )
    position_parser.add_argument(
        "--mocap-csv",
        type=Path,
        default=None,
        help="Explicit raw mocap CSV path for a single debug run.",
    )
    position_parser.add_argument(
        "--boundary-trim-s",
        type=float,
        default=0.25,
        help="Trim this much time from each side of a one-direction window (default: 0.25)",
    )
    position_parser.add_argument(
        "--min-segment-s",
        type=float,
        default=1.0,
        help="Drop retained position windows shorter than this (default: 1.0)",
    )
    position_parser.add_argument(
        "--min-angle-span-deg",
        type=float,
        default=5.0,
        help="Drop retained position windows with smaller feedback span (default: 5.0)",
    )

    velocity_parser = subparsers.add_parser(
        "velocity", help="Compare MIT set_velocity feedback against raw mocap."
    )
    add_common(velocity_parser)
    velocity_parser.add_argument(
        "--speeds",
        default="1000,3000,5000",
        help="Comma-separated commanded ERPM runs to process (default: 1000,3000,5000)",
    )
    velocity_parser.add_argument(
        "--motor-csv",
        type=Path,
        default=None,
        help="Explicit motor CSV path for a single debug run.",
    )
    velocity_parser.add_argument(
        "--mocap-csv",
        type=Path,
        default=None,
        help="Explicit raw mocap CSV path for a single debug run.",
    )
    velocity_parser.add_argument(
        "--settle-s",
        type=float,
        default=2.0,
        help="Discard this much time after each phase start before metrics (default: 2.0)",
    )
    velocity_parser.add_argument(
        "--phase-index",
        default="all",
        help="Comma-separated phase indices to analyze, or 'all' (default: all)",
    )

    all_parser = subparsers.add_parser(
        "all", help="Run both position and velocity analysis."
    )
    add_common(all_parser)
    all_parser.add_argument(
        "--targets",
        default="30,50,90",
        help="Comma-separated position-step targets to process (default: 30,50,90)",
    )
    all_parser.add_argument(
        "--speeds",
        default="1000,3000,5000",
        help="Comma-separated velocity runs to process (default: 1000,3000,5000)",
    )
    all_parser.add_argument(
        "--boundary-trim-s",
        type=float,
        default=0.25,
        help="Trim this much time from each side of a one-direction window (default: 0.25)",
    )
    all_parser.add_argument(
        "--min-segment-s",
        type=float,
        default=1.0,
        help="Drop retained position windows shorter than this (default: 1.0)",
    )
    all_parser.add_argument(
        "--min-angle-span-deg",
        type=float,
        default=5.0,
        help="Drop retained position windows with smaller feedback span (default: 5.0)",
    )
    all_parser.add_argument(
        "--settle-s",
        type=float,
        default=2.0,
        help="Discard this much time after each phase start before metrics (default: 2.0)",
    )
    all_parser.add_argument(
        "--phase-index",
        default="all",
        help="Comma-separated phase indices to analyze, or 'all' (default: all)",
    )

    args = parser.parse_args()
    if not hasattr(args, "motor_csv"):
        args.motor_csv = None
    if not hasattr(args, "mocap_csv"):
        args.mocap_csv = None
    if hasattr(args, "targets"):
        args.targets = _parse_number_list(args.targets, kind="target")
    if hasattr(args, "speeds"):
        args.speeds = _parse_number_list(args.speeds, kind="speed")
    if hasattr(args, "phase_index") and args.phase_index != "all":
        args.phase_index = _parse_number_list(args.phase_index, kind="phase-index")
    if hasattr(args, "boundary_trim_s") and args.boundary_trim_s < 0:
        raise ValueError("--boundary-trim-s must be >= 0")
    if hasattr(args, "min_segment_s") and args.min_segment_s <= 0:
        raise ValueError("--min-segment-s must be > 0")
    if hasattr(args, "min_angle_span_deg") and args.min_angle_span_deg < 0:
        raise ValueError("--min-angle-span-deg must be >= 0")
    if hasattr(args, "settle_s") and args.settle_s < 0:
        raise ValueError("--settle-s must be >= 0")
    return args


def resolve_position_pairs(
    data_root: Path,
    *,
    targets: Sequence[int] | None = None,
    motor_csv: Path | None = None,
    mocap_csv: Path | None = None,
) -> list[FilePair]:
    """Resolve position-run motor/mocap pairs from the curated folder."""
    if (motor_csv is None) != (mocap_csv is None):
        raise ValueError("--motor-csv and --mocap-csv must be provided together.")
    if motor_csv and mocap_csv:
        label = motor_csv.stem.removeprefix(f"{POSITION_MOTOR_PREFIX}_")
        return [FilePair(label=label, motor_csv=motor_csv, mocap_csv=mocap_csv)]

    requested = POSITION_TARGETS if targets is None else tuple(targets)
    pairs: list[FilePair] = []
    for target in requested:
        if target not in POSITION_MOCAP_FILES:
            raise ValueError(
                f"Unsupported position target {target}. "
                f"Supported targets: {', '.join(map(str, POSITION_TARGETS))}"
            )
        motor_path = data_root / f"{POSITION_MOTOR_PREFIX}_{target}.csv"
        mocap_path = data_root / "motion-capture-data" / POSITION_MOCAP_FILES[target]
        if not motor_path.exists():
            raise FileNotFoundError(f"Missing position motor CSV: {motor_path}")
        if not mocap_path.exists():
            raise FileNotFoundError(f"Missing position mocap CSV: {mocap_path}")
        pairs.append(FilePair(label=str(target), motor_csv=motor_path, mocap_csv=mocap_path))
    return pairs


def resolve_velocity_pairs(
    data_root: Path,
    *,
    speeds: Sequence[int] | None = None,
    motor_csv: Path | None = None,
    mocap_csv: Path | None = None,
) -> list[FilePair]:
    """Resolve velocity-run motor/mocap pairs from the curated folder."""
    if (motor_csv is None) != (mocap_csv is None):
        raise ValueError("--motor-csv and --mocap-csv must be provided together.")
    if motor_csv and mocap_csv:
        label = motor_csv.stem.removeprefix(f"{VELOCITY_MOTOR_PREFIX}_")
        return [FilePair(label=label, motor_csv=motor_csv, mocap_csv=mocap_csv)]

    requested = VELOCITY_SPEEDS if speeds is None else tuple(speeds)
    pairs: list[FilePair] = []
    for speed in requested:
        if speed not in VELOCITY_MOCAP_FILES:
            raise ValueError(
                f"Unsupported velocity run {speed}. "
                f"Supported speeds: {', '.join(map(str, VELOCITY_SPEEDS))}"
            )
        motor_path = data_root / f"{VELOCITY_MOTOR_PREFIX}_{speed}.csv"
        mocap_path = data_root / "motion-capture-data" / VELOCITY_MOCAP_FILES[speed]
        if not motor_path.exists():
            raise FileNotFoundError(f"Missing velocity motor CSV: {motor_path}")
        if not mocap_path.exists():
            raise FileNotFoundError(f"Missing velocity mocap CSV: {mocap_path}")
        pairs.append(FilePair(label=str(speed), motor_csv=motor_path, mocap_csv=mocap_path))
    return pairs


def load_motor_position_csv(path: Path) -> MotorPositionData:
    """Parse a MIT position-step motor CSV."""
    rows = _parse_csv_with_required_columns(path, POSITION_REQUIRED_COLUMNS)
    elapsed_s: list[float] = []
    tick_index: list[int] = []
    direction: list[int] = []
    command_position_deg: list[float] = []
    segment_target_deg: list[float] = []
    feedback_received: list[int] = []
    feedback_position_deg: list[float] = []
    feedback_speed_erpm: list[float] = []
    for row in rows:
        elapsed_s.append(_to_float(row["elapsed_s"]))
        tick_index.append(_to_int(row["tick_index"]))
        direction.append(_to_int(row["direction"]))
        command_position_deg.append(_to_float(row["command_position_deg"]))
        segment_target_deg.append(_to_float(row["segment_target_deg"]))
        feedback_received.append(_to_int(row["feedback_received"]))
        feedback_position_deg.append(_to_float(row["feedback_position_deg"]))
        feedback_speed_erpm.append(_to_float(row["feedback_speed_erpm"]))

    return MotorPositionData(
        elapsed_s=np.asarray(elapsed_s, dtype=float),
        tick_index=np.asarray(tick_index, dtype=int),
        direction=np.asarray(direction, dtype=int),
        command_position_deg=np.asarray(command_position_deg, dtype=float),
        segment_target_deg=np.asarray(segment_target_deg, dtype=float),
        feedback_received=np.asarray(feedback_received, dtype=int),
        feedback_position_deg=np.asarray(feedback_position_deg, dtype=float),
        feedback_speed_erpm=np.asarray(feedback_speed_erpm, dtype=float),
    )


def load_motor_velocity_csv(path: Path) -> MotorVelocityData:
    """Parse a MIT velocity-validation motor CSV."""
    rows = _parse_csv_with_required_columns(path, VELOCITY_REQUIRED_COLUMNS)
    elapsed_s: list[float] = []
    phase_index: list[int] = []
    phase_command_erpm: list[int] = []
    command_erpm: list[int] = []
    feedback_position_deg: list[float] = []
    feedback_speed_erpm: list[float] = []
    for row in rows:
        elapsed_s.append(_to_float(row["elapsed_s"]))
        phase_index.append(_to_int(row["phase_index"]))
        phase_command_erpm.append(_to_int(row["phase_command_erpm"]))
        command_erpm.append(_to_int(row["command_erpm"]))
        feedback_position_deg.append(_to_float(row["feedback_position_deg"]))
        feedback_speed_erpm.append(_to_float(row["feedback_speed_erpm"]))

    feedback_speed_array = np.asarray(feedback_speed_erpm, dtype=float)
    return MotorVelocityData(
        elapsed_s=np.asarray(elapsed_s, dtype=float),
        phase_index=np.asarray(phase_index, dtype=int),
        phase_command_erpm=np.asarray(phase_command_erpm, dtype=int),
        command_erpm=np.asarray(command_erpm, dtype=int),
        feedback_position_deg=np.asarray(feedback_position_deg, dtype=float),
        feedback_speed_erpm=feedback_speed_array,
        motor_mech_deg_s=feedback_speed_array * MECH_DEG_PER_SEC_PER_ERPM,
    )


def load_raw_mocap_csv(path: Path) -> RawMocapData:
    """Parse a raw Vicon-style mocap CSV with preamble and units rows."""
    frame: list[float] = []
    sub_frame: list[float] = []
    rx_deg: list[float] = []
    ry_deg: list[float] = []
    rz_deg: list[float] = []
    tx_mm: list[float] = []
    ty_mm: list[float] = []
    tz_mm: list[float] = []

    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.reader(handle)
        for row in reader:
            if len(row) < 8:
                continue
            try:
                frame_value = float(row[0])
                sub_frame_value = float(row[1] or 0)
                rx_value = float(row[2])
                ry_value = float(row[3])
                rz_value = float(row[4])
                tx_value = float(row[5])
                ty_value = float(row[6])
                tz_value = float(row[7])
            except ValueError:
                continue

            frame.append(frame_value)
            sub_frame.append(sub_frame_value)
            rx_deg.append(rx_value)
            ry_deg.append(ry_value)
            rz_deg.append(rz_value)
            tx_mm.append(tx_value)
            ty_mm.append(ty_value)
            tz_mm.append(tz_value)

    if not frame:
        raise ValueError(f"{path} did not contain numeric Vicon samples.")

    return RawMocapData(
        frame=np.asarray(frame, dtype=float),
        sub_frame=np.asarray(sub_frame, dtype=float),
        rx_deg=np.asarray(rx_deg, dtype=float),
        ry_deg=np.asarray(ry_deg, dtype=float),
        rz_deg=np.asarray(rz_deg, dtype=float),
        tx_mm=np.asarray(tx_mm, dtype=float),
        ty_mm=np.asarray(ty_mm, dtype=float),
        tz_mm=np.asarray(tz_mm, dtype=float),
    )


def _first_sustained_true(mask: np.ndarray, min_count: int) -> int | None:
    """Return the first index with at least min_count consecutive True values."""
    run_length = 0
    for index, value in enumerate(mask):
        if value:
            run_length += 1
            if run_length >= min_count:
                return index - min_count + 1
        else:
            run_length = 0
    return None


def detect_auto_trim_frame(raw: RawMocapData) -> int:
    """Detect the first mocap frame to keep using sustained motion onset."""
    Rotation, _ = _require_analysis_runtime()
    rot = Rotation.from_rotvec(
        np.deg2rad(np.column_stack([raw.rx_deg, raw.ry_deg, raw.rz_deg]))
    )
    if len(raw.frame) < 8:
        return round(raw.frame[0])

    incremental_deg = np.zeros(len(raw.frame), dtype=float)
    incremental_deg[1:] = np.rad2deg((rot[:-1].inv() * rot[1:]).magnitude())
    smoothed = _moving_average(incremental_deg, window=9)
    baseline_count = min(max(25, len(smoothed) // 20), len(smoothed))
    baseline_slice = smoothed[:baseline_count]
    baseline = float(np.median(baseline_slice))
    mad = float(np.median(np.abs(baseline_slice - baseline)))
    threshold = max(0.08, baseline + max(0.12, 6.0 * mad))
    onset_index = _first_sustained_true(smoothed > threshold, min_count=5)
    if onset_index is None:
        return round(raw.frame[0])
    return round(raw.frame[max(onset_index - 1, 0)])


def process_raw_mocap(
    raw: RawMocapData,
    *,
    trim_frame: str | int,
    axis: str,
    invert_sign: bool,
) -> MocapSeries:
    """Convert raw RX/RY/RZ Vicon data into one aligned angle + velocity trace."""
    Rotation, _ = _require_analysis_runtime()
    resolved_trim = (
        detect_auto_trim_frame(raw) if trim_frame == "auto" else int(trim_frame)
    )
    keep_mask = raw.frame >= float(resolved_trim)
    if not np.any(keep_mask):
        raise ValueError(f"No mocap samples remain after trim frame {resolved_trim}.")

    frame = raw.frame[keep_mask]
    rot = Rotation.from_rotvec(
        np.deg2rad(
            np.column_stack(
                [raw.rx_deg[keep_mask], raw.ry_deg[keep_mask], raw.rz_deg[keep_mask]]
            )
        )
    )
    aligned = rot[0].inv() * rot
    aligned_rotvec_deg = np.rad2deg(aligned.as_rotvec())
    axis_names = ("rx", "ry", "rz")
    if axis == "auto":
        selected_index = int(np.argmax(np.var(aligned_rotvec_deg, axis=0)))
        selected_axis = axis_names[selected_index]
    else:
        selected_axis = axis
        selected_index = axis_names.index(axis)

    angle_deg = aligned_rotvec_deg[:, selected_index]
    angle_deg = _unwrap_degrees(angle_deg - angle_deg[0])
    if invert_sign:
        angle_deg = -angle_deg
    time_s = (frame - frame[0]) / FRAME_RATE_HZ
    velocity_deg_s = np.gradient(angle_deg, time_s, edge_order=1)
    velocity_deg_s = _moving_average(velocity_deg_s, window=5)
    return MocapSeries(
        time_s=time_s,
        angle_deg=angle_deg,
        velocity_deg_s=velocity_deg_s,
        frame=frame,
        trim_frame=resolved_trim,
        selected_axis=selected_axis,
    )


def _detect_motion_onset_time(
    time_s: np.ndarray,
    angle_deg: np.ndarray,
    velocity_deg_s: np.ndarray,
    *,
    min_count: int = 5,
    angle_floor_deg: float = 2.0,
    velocity_floor_deg_s: float = 5.0,
) -> float:
    """Detect the first sustained motion onset time in a trace."""
    displacement = np.abs(angle_deg - angle_deg[0])
    speed = np.abs(velocity_deg_s)
    displacement_threshold = max(
        angle_floor_deg, float(np.percentile(displacement, 90)) * 0.05
    )
    speed_threshold = max(
        velocity_floor_deg_s, float(np.percentile(speed, 90)) * 0.05
    )
    onset_mask = (displacement >= displacement_threshold) | (speed >= speed_threshold)
    onset_index = _first_sustained_true(onset_mask, min_count=min_count)
    if onset_index is None:
        return float(time_s[0])
    return float(time_s[onset_index])


def _align_mocap_time(
    motor_time_s: np.ndarray,
    motor_angle_deg: np.ndarray,
    motor_velocity_deg_s: np.ndarray,
    mocap: MocapSeries,
    *,
    time_shift_s: str | float,
    velocity_floor_deg_s: float,
) -> tuple[np.ndarray, float]:
    """Return aligned mocap time and the resolved time shift."""
    if time_shift_s == "auto":
        motor_onset = _detect_motion_onset_time(
            motor_time_s,
            motor_angle_deg,
            motor_velocity_deg_s,
            velocity_floor_deg_s=velocity_floor_deg_s,
        )
        mocap_onset = _detect_motion_onset_time(
            mocap.time_s,
            mocap.angle_deg,
            mocap.velocity_deg_s,
            velocity_floor_deg_s=max(3.0, velocity_floor_deg_s * 0.5),
        )
        resolved_shift = motor_onset - mocap_onset
    else:
        resolved_shift = float(time_shift_s)
    return mocap.time_s + resolved_shift, float(resolved_shift)


def _overlap_mask(reference_time_s: np.ndarray, other_time_s: np.ndarray) -> np.ndarray:
    """Return mask for the shared time support of two series."""
    overlap_start = max(float(reference_time_s[0]), float(other_time_s[0]))
    overlap_end = min(float(reference_time_s[-1]), float(other_time_s[-1]))
    if overlap_end <= overlap_start:
        return np.zeros_like(reference_time_s, dtype=bool)
    return (reference_time_s >= overlap_start) & (reference_time_s <= overlap_end)


def _largest_true_run(mask: np.ndarray) -> tuple[int, int] | None:
    """Return the start/end indices of the longest contiguous True run."""
    true_indices = np.flatnonzero(mask)
    if true_indices.size == 0:
        return None
    split_points = np.where(np.diff(true_indices) > 1)[0]
    run_starts = np.concatenate(([true_indices[0]], true_indices[split_points + 1]))
    run_ends = np.concatenate((true_indices[split_points], [true_indices[-1]]))
    run_lengths = run_ends - run_starts + 1
    best_index = int(np.argmax(run_lengths))
    return int(run_starts[best_index]), int(run_ends[best_index])


def _shared_active_velocity_window_mask(
    time_s: np.ndarray,
    motor_speed_deg_s: np.ndarray,
    mocap_speed_deg_s: np.ndarray,
    *,
    floor_deg_s: float = 20.0,
    threshold_fraction: float = 0.15,
    smoothing_window: int = 11,
    pad_s: float = 0.25,
    edge_trim_samples: int = 8,
) -> np.ndarray:
    """Return a plotting mask for the time span where both velocity traces are active."""
    if len(time_s) < 2:
        return np.ones_like(time_s, dtype=bool)

    motor_threshold = max(
        floor_deg_s,
        float(np.percentile(np.abs(motor_speed_deg_s), 80)) * threshold_fraction,
    )
    mocap_threshold = max(
        floor_deg_s,
        float(np.percentile(np.abs(mocap_speed_deg_s), 80)) * threshold_fraction,
    )
    common_active = (np.abs(motor_speed_deg_s) >= motor_threshold) & (
        np.abs(mocap_speed_deg_s) >= mocap_threshold
    )
    if smoothing_window > 1:
        common_active = _moving_average(
            common_active.astype(float), window=smoothing_window
        ) >= 0.5
    active_run = _largest_true_run(common_active)
    if active_run is None:
        return np.ones_like(time_s, dtype=bool)

    start_index, end_index = active_run
    dt = float(np.median(np.diff(time_s)))
    pad_samples = max(0, int(round(pad_s / max(dt, 1e-9))))
    start_index = max(0, start_index - pad_samples)
    end_index = min(len(time_s) - 1, end_index + pad_samples)
    if end_index - start_index > 2 * edge_trim_samples:
        start_index += edge_trim_samples
        end_index -= edge_trim_samples
    window_mask = np.zeros_like(time_s, dtype=bool)
    window_mask[start_index : end_index + 1] = True
    return window_mask


def _interp_series(
    sample_time_s: np.ndarray, source_time_s: np.ndarray, source_value: np.ndarray
) -> np.ndarray:
    """Interpolate one source series onto another time base."""
    return np.interp(sample_time_s, source_time_s, source_value)


def find_position_segments(
    motor: MotorPositionData,
    *,
    boundary_trim_s: float,
    min_segment_s: float,
    min_angle_span_deg: float,
) -> list[PositionSegment]:
    """Find retained one-direction windows in the motor position trace."""
    keep_mask = motor.feedback_received == 1
    if not np.any(keep_mask):
        return []

    time_s = motor.elapsed_s[keep_mask]
    direction = motor.direction[keep_mask]
    command = motor.command_position_deg[keep_mask]
    feedback = motor.feedback_position_deg[keep_mask]
    target = motor.segment_target_deg[keep_mask]
    ticks = motor.tick_index[keep_mask]
    global_indices = np.flatnonzero(keep_mask)

    segments: list[PositionSegment] = []
    start = 0
    segment_id = 1

    def close_candidate(start_idx: int, end_idx: int) -> None:
        nonlocal segment_id
        local_time = time_s[start_idx : end_idx + 1]
        if len(local_time) < 2:
            return
        trimmed_start = float(local_time[0] + boundary_trim_s)
        trimmed_end = float(local_time[-1] - boundary_trim_s)
        if trimmed_end <= trimmed_start:
            return
        trimmed_mask = (local_time >= trimmed_start) & (local_time <= trimmed_end)
        if np.count_nonzero(trimmed_mask) < 2:
            return
        trimmed_time = local_time[trimmed_mask]
        duration = float(trimmed_time[-1] - trimmed_time[0])
        if duration < min_segment_s:
            return
        trimmed_feedback = feedback[start_idx : end_idx + 1][trimmed_mask]
        angle_span = float(np.ptp(trimmed_feedback))
        if angle_span < min_angle_span_deg:
            return
        local_indices = global_indices[start_idx : end_idx + 1][trimmed_mask]
        segments.append(
            PositionSegment(
                segment_id=segment_id,
                direction=int(direction[start_idx]),
                start_time_s=float(trimmed_time[0]),
                end_time_s=float(trimmed_time[-1]),
                start_index=int(local_indices[0]),
                end_index=int(local_indices[-1]),
                target_deg=float(target[start_idx : end_idx + 1][trimmed_mask][-1]),
                tick_start=int(ticks[start_idx]),
                tick_end=int(ticks[end_idx]),
                angle_span_deg=angle_span,
            )
        )
        segment_id += 1

    for index in range(1, len(time_s)):
        monotonic_step = (command[index] - command[index - 1]) * direction[index - 1]
        if direction[index] != direction[index - 1] or monotonic_step < -1e-9:
            close_candidate(start, index - 1)
            start = index

    close_candidate(start, len(time_s) - 1)
    return segments


def _write_summary_csv(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    """Write summary rows to CSV."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _save_figure(fig: Any, path_without_suffix: Path) -> None:
    """Save one figure as PNG and PDF."""
    path_without_suffix.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path_without_suffix.with_suffix(".png"), dpi=200, bbox_inches="tight")
    fig.savefig(path_without_suffix.with_suffix(".pdf"), bbox_inches="tight")


def _normalize_by_mean_abs(values: np.ndarray) -> np.ndarray:
    """Normalize a signal by its mean absolute value, guarding against zeros."""
    scale = max(float(np.mean(np.abs(values))), 1e-9)
    return values / scale


def _series_correlation(a: np.ndarray, b: np.ndarray) -> float:
    """Return a guarded Pearson correlation for two numeric series."""
    if len(a) < 3 or len(b) < 3:
        return float("nan")
    std_a = float(np.std(a))
    std_b = float(np.std(b))
    if std_a <= 1e-12 or std_b <= 1e-12:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def summarize_position_functional_rows(
    rows: Sequence[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Aggregate retained position-segment metrics into one summary row per run."""
    if not rows:
        return []

    by_run: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        by_run.setdefault(str(row["run_label"]), []).append(row)

    def _run_sort_key(run_label: str) -> tuple[int, str]:
        try:
            return (int(run_label), run_label)
        except ValueError:
            return (10**9, run_label)

    summary_rows: list[dict[str, Any]] = []
    for run_label in sorted(by_run, key=_run_sort_key):
        run_rows = by_run[run_label]
        command_corr = np.asarray(
            [float(row["command_relative_vs_mocap_corr"]) for row in run_rows],
            dtype=float,
        )
        feedback_corr = np.asarray(
            [float(row["feedback_relative_vs_mocap_corr"]) for row in run_rows],
            dtype=float,
        )
        mocap_ratio = np.asarray(
            [float(row["mocap_to_command_relative_span_ratio"]) for row in run_rows],
            dtype=float,
        )
        feedback_ratio = np.asarray(
            [float(row["feedback_to_command_relative_span_ratio"]) for row in run_rows],
            dtype=float,
        )
        durations = np.asarray(
            [float(row["segment_duration_s"]) for row in run_rows], dtype=float
        )
        summary_rows.append(
            {
                "run_label": run_label,
                "retained_segment_count": len(run_rows),
                "retained_duration_s": round(float(np.sum(durations)), 6),
                "command_relative_vs_mocap_corr_median": round(
                    float(np.nanmedian(command_corr)), 6
                ),
                "command_relative_vs_mocap_corr_min": round(
                    float(np.nanmin(command_corr)), 6
                ),
                "command_relative_vs_mocap_corr_max": round(
                    float(np.nanmax(command_corr)), 6
                ),
                "feedback_relative_vs_mocap_corr_median": round(
                    float(np.nanmedian(feedback_corr)), 6
                ),
                "feedback_relative_vs_mocap_corr_min": round(
                    float(np.nanmin(feedback_corr)), 6
                ),
                "feedback_relative_vs_mocap_corr_max": round(
                    float(np.nanmax(feedback_corr)), 6
                ),
                "mocap_to_command_relative_span_ratio_median": round(
                    float(np.nanmedian(mocap_ratio)), 6
                ),
                "mocap_to_command_relative_span_ratio_min": round(
                    float(np.nanmin(mocap_ratio)), 6
                ),
                "mocap_to_command_relative_span_ratio_max": round(
                    float(np.nanmax(mocap_ratio)), 6
                ),
                "feedback_to_command_relative_span_ratio_median": round(
                    float(np.nanmedian(feedback_ratio)), 6
                ),
                "feedback_to_command_relative_span_ratio_min": round(
                    float(np.nanmin(feedback_ratio)), 6
                ),
                "feedback_to_command_relative_span_ratio_max": round(
                    float(np.nanmax(feedback_ratio)), 6
                ),
            }
        )
    return summary_rows


def summarize_velocity_response_rows(
    rows: Sequence[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Aggregate velocity-phase metrics into one summary row per commanded ERPM."""
    if not rows:
        return []

    by_command: dict[int, list[dict[str, Any]]] = {}
    for row in rows:
        by_command.setdefault(int(row["command_erpm"]), []).append(row)

    summary_rows: list[dict[str, Any]] = []
    for command_erpm in sorted(by_command):
        command_rows = by_command[command_erpm]
        motor_mean = np.asarray(
            [float(row["mean_motor_speed_deg_s"]) for row in command_rows], dtype=float
        )
        mocap_mean = np.asarray(
            [float(row["mean_mocap_speed_deg_s"]) for row in command_rows], dtype=float
        )
        window_duration = np.asarray(
            [float(row["active_window_duration_s"]) for row in command_rows], dtype=float
        )
        ratio = np.asarray(
            [float(row["motor_to_mocap_speed_ratio"]) for row in command_rows],
            dtype=float,
        )
        normalized_rmse = np.asarray(
            [float(row["normalized_speed_rmse"]) for row in command_rows], dtype=float
        )
        summary_rows.append(
            {
                "command_erpm": command_erpm,
                "phase_count": len(command_rows),
                "mean_motor_speed_deg_s": round(float(np.mean(motor_mean)), 6),
                "mean_mocap_speed_deg_s": round(float(np.mean(mocap_mean)), 6),
                "active_window_duration_s": round(float(np.mean(window_duration)), 6),
                "motor_to_mocap_speed_ratio": round(float(np.mean(ratio)), 6),
                "normalized_speed_rmse": round(float(np.mean(normalized_rmse)), 6),
            }
        )
    return summary_rows


def analyze_position_pair(
    pair: FilePair,
    *,
    output_dir: Path,
    trim_frame: str | int,
    axis: str,
    invert_mocap_sign: bool,
    boundary_trim_s: float,
    min_segment_s: float,
    min_angle_span_deg: float,
    time_shift_s: str | float,
) -> list[dict[str, Any]]:
    """Analyze one position motor/mocap pair and write its figures."""
    _, plt = _require_analysis_runtime()
    motor = load_motor_position_csv(pair.motor_csv)
    raw_mocap = load_raw_mocap_csv(pair.mocap_csv)
    mocap = process_raw_mocap(
        raw_mocap,
        trim_frame=trim_frame,
        axis=axis,
        invert_sign=invert_mocap_sign,
    )
    motor_feedback_speed_deg_s = motor.feedback_speed_erpm * MECH_DEG_PER_SEC_PER_ERPM
    aligned_mocap_time_s, resolved_shift_s = _align_mocap_time(
        motor.elapsed_s,
        motor.feedback_position_deg,
        motor_feedback_speed_deg_s,
        mocap,
        time_shift_s=time_shift_s,
        velocity_floor_deg_s=10.0,
    )
    overlap_mask = _overlap_mask(motor.elapsed_s, aligned_mocap_time_s)
    if not np.any(overlap_mask):
        raise RuntimeError(f"No motor/mocap time overlap for position pair {pair.label}.")

    overlap_time_s = motor.elapsed_s[overlap_mask]
    mocap_interp_angle_deg = _interp_series(
        overlap_time_s, aligned_mocap_time_s, mocap.angle_deg
    )
    feedback_overlap = motor.feedback_position_deg[overlap_mask]
    command_overlap = motor.command_position_deg[overlap_mask]
    feedback_error_deg = feedback_overlap - mocap_interp_angle_deg
    command_error_deg = command_overlap - mocap_interp_angle_deg
    segments = find_position_segments(
        motor,
        boundary_trim_s=boundary_trim_s,
        min_segment_s=min_segment_s,
        min_angle_span_deg=min_angle_span_deg,
    )

    fig, axes = plt.subplots(
        2,
        1,
        sharex=True,
        figsize=(11, 6.5),
        constrained_layout=True,
    )
    ax_top, ax_bottom = axes
    for segment in segments:
        ax_top.axvspan(
            segment.start_time_s, segment.end_time_s, color="0.7", alpha=0.12
        )
        ax_bottom.axvspan(
            segment.start_time_s, segment.end_time_s, color="0.7", alpha=0.12
        )
    ax_top.plot(
        motor.elapsed_s,
        motor.command_position_deg,
        color=COLOR_COMMAND,
        label="Command position",
        linewidth=1.2,
    )
    ax_top.plot(
        motor.elapsed_s,
        motor.feedback_position_deg,
        color=COLOR_MOTOR,
        label="Motor feedback",
        linewidth=1.4,
    )
    ax_top.plot(
        aligned_mocap_time_s,
        mocap.angle_deg,
        color=COLOR_MOCAP,
        label="Motion capture",
        linewidth=1.3,
    )
    ax_top.set_title(f"Position validation: {pair.label} deg run")
    ax_top.set_ylabel("Angle [deg]")
    ax_top.legend(loc="best")
    ax_top.grid(alpha=0.3)

    ax_bottom.plot(
        overlap_time_s,
        feedback_error_deg,
        color=COLOR_ERROR,
        label="Motor - mocap",
        linewidth=1.2,
    )
    ax_bottom.plot(
        overlap_time_s,
        command_error_deg,
        color=COLOR_COMMAND,
        label="Command - mocap",
        linewidth=1.0,
    )
    ax_bottom.axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    ax_bottom.set_xlabel("Time [s]")
    ax_bottom.set_ylabel("Error [deg]")
    ax_bottom.legend(loc="best")
    ax_bottom.grid(alpha=0.3)
    _save_figure(fig, output_dir / f"position_overlay_{pair.label}")
    plt.close(fig)

    summary_rows: list[dict[str, Any]] = []
    for segment in segments:
        segment_mask = (
            (motor.elapsed_s >= segment.start_time_s)
            & (motor.elapsed_s <= segment.end_time_s)
            & overlap_mask
        )
        if np.count_nonzero(segment_mask) < 2:
            continue
        segment_time_s = motor.elapsed_s[segment_mask]
        segment_command_deg = motor.command_position_deg[segment_mask]
        segment_feedback_deg = motor.feedback_position_deg[segment_mask]
        segment_mocap_deg = _interp_series(
            segment_time_s, aligned_mocap_time_s, mocap.angle_deg
        )
        motor_error = segment_feedback_deg - segment_mocap_deg
        command_error = segment_command_deg - segment_mocap_deg
        rel_time_s = segment_time_s - segment_time_s[0]
        command_relative_deg = segment_command_deg - segment_command_deg[0]
        feedback_relative_deg = segment_feedback_deg - segment_feedback_deg[0]
        mocap_relative_deg = segment_mocap_deg - segment_mocap_deg[0]
        motor_relative_error = feedback_relative_deg - mocap_relative_deg
        command_relative_error = command_relative_deg - mocap_relative_deg
        command_relative_corr = _series_correlation(
            command_relative_deg, mocap_relative_deg
        )
        feedback_relative_corr = _series_correlation(
            feedback_relative_deg, mocap_relative_deg
        )
        command_relative_span = float(np.ptp(command_relative_deg))
        feedback_relative_span = float(np.ptp(feedback_relative_deg))
        mocap_relative_span = float(np.ptp(mocap_relative_deg))
        summary_rows.append(
            {
                "run_label": pair.label,
                "motor_csv": str(pair.motor_csv),
                "mocap_csv": str(pair.mocap_csv),
                "trim_frame": mocap.trim_frame,
                "selected_axis": mocap.selected_axis,
                "time_shift_s": round(resolved_shift_s, 6),
                "invert_mocap_sign": int(invert_mocap_sign),
                "segment_id": segment.segment_id,
                "direction": segment.direction,
                "segment_start_s": round(segment.start_time_s, 6),
                "segment_end_s": round(segment.end_time_s, 6),
                "segment_duration_s": round(segment.end_time_s - segment.start_time_s, 6),
                "target_deg": round(segment.target_deg, 6),
                "tick_start": segment.tick_start,
                "tick_end": segment.tick_end,
                "feedback_angle_span_deg": round(segment.angle_span_deg, 6),
                "mean_abs_motor_vs_mocap_deg": round(
                    float(np.mean(np.abs(motor_error))), 6
                ),
                "rmse_motor_vs_mocap_deg": round(
                    float(np.sqrt(np.mean(motor_error**2))), 6
                ),
                "max_abs_motor_vs_mocap_deg": round(
                    float(np.max(np.abs(motor_error))), 6
                ),
                "mean_abs_command_vs_mocap_deg": round(
                    float(np.mean(np.abs(command_error))), 6
                ),
                "final_mocap_error_to_target_deg": round(
                    float(segment_mocap_deg[-1] - segment.target_deg), 6
                ),
                "final_motor_vs_mocap_deg": round(
                    float(segment_feedback_deg[-1] - segment_mocap_deg[-1]), 6
                ),
                "command_relative_vs_mocap_corr": round(
                    float(command_relative_corr), 6
                ),
                "feedback_relative_vs_mocap_corr": round(
                    float(feedback_relative_corr), 6
                ),
                "mocap_to_command_relative_span_ratio": round(
                    float(mocap_relative_span / max(command_relative_span, 1e-9)), 6
                ),
                "feedback_to_command_relative_span_ratio": round(
                    float(feedback_relative_span / max(command_relative_span, 1e-9)), 6
                ),
            }
        )

        detail_fig, detail_axes = plt.subplots(
            2,
            1,
            sharex=True,
            figsize=(9.5, 5.8),
            constrained_layout=True,
        )
        detail_axes[0].plot(
            segment_time_s - segment_time_s[0],
            segment_command_deg,
            color=COLOR_COMMAND,
            label="Command position",
            linewidth=1.2,
        )
        detail_axes[0].plot(
            segment_time_s - segment_time_s[0],
            segment_feedback_deg,
            color=COLOR_MOTOR,
            label="Motor feedback",
            linewidth=1.4,
        )
        detail_axes[0].plot(
            segment_time_s - segment_time_s[0],
            segment_mocap_deg,
            color=COLOR_MOCAP,
            label="Motion capture",
            linewidth=1.3,
        )
        detail_axes[0].set_title(
            f"Position segment {segment.segment_id}: run {pair.label}, target {segment.target_deg:.1f} deg"
        )
        detail_axes[0].set_ylabel("Angle [deg]")
        detail_axes[0].legend(loc="best")
        detail_axes[0].grid(alpha=0.3)
        detail_axes[1].plot(
            segment_time_s - segment_time_s[0],
            motor_error,
            color=COLOR_ERROR,
            label="Motor - mocap",
            linewidth=1.2,
        )
        detail_axes[1].plot(
            segment_time_s - segment_time_s[0],
            command_error,
            color=COLOR_COMMAND,
            label="Command - mocap",
            linewidth=1.0,
        )
        detail_axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
        detail_axes[1].set_xlabel("Time from segment start [s]")
        detail_axes[1].set_ylabel("Error [deg]")
        detail_axes[1].legend(loc="best")
        detail_axes[1].grid(alpha=0.3)
        _save_figure(
            detail_fig,
            output_dir / f"position_segment_{pair.label}_{segment.segment_id:02d}",
        )
        plt.close(detail_fig)

        consistency_fig, consistency_axes = plt.subplots(
            2,
            1,
            sharex=True,
            figsize=(9.5, 5.8),
            constrained_layout=True,
        )
        consistency_axes[0].plot(
            rel_time_s,
            command_relative_deg,
            color=COLOR_COMMAND,
            label="Command relative angle",
            linewidth=1.2,
        )
        consistency_axes[0].plot(
            rel_time_s,
            feedback_relative_deg,
            color=COLOR_MOTOR,
            label="Motor feedback relative angle",
            linewidth=1.4,
        )
        consistency_axes[0].plot(
            rel_time_s,
            mocap_relative_deg,
            color=COLOR_MOCAP,
            label="Motion capture relative angle",
            linewidth=1.3,
        )
        consistency_axes[0].set_title(
            "Position consistency: "
            f"run {pair.label}, segment {segment.segment_id}, "
            f"direction {'+' if segment.direction > 0 else '-'}"
        )
        consistency_axes[0].set_ylabel("Relative angle [deg]")
        consistency_axes[0].legend(loc="best")
        consistency_axes[0].grid(alpha=0.3)
        consistency_axes[1].plot(
            rel_time_s,
            motor_relative_error,
            color=COLOR_ERROR,
            label="Motor relative - mocap relative",
            linewidth=1.2,
        )
        consistency_axes[1].plot(
            rel_time_s,
            command_relative_error,
            color=COLOR_COMMAND,
            label="Command relative - mocap relative",
            linewidth=1.0,
        )
        consistency_axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
        consistency_axes[1].set_xlabel("Time from segment start [s]")
        consistency_axes[1].set_ylabel("Relative error [deg]")
        consistency_axes[1].legend(loc="best")
        consistency_axes[1].grid(alpha=0.3)
        _save_figure(
            consistency_fig,
            output_dir / f"position_consistency_{pair.label}_{segment.segment_id:02d}",
        )
        plt.close(consistency_fig)

    return summary_rows


def analyze_velocity_pair(
    pair: FilePair,
    *,
    output_dir: Path,
    trim_frame: str | int,
    axis: str,
    invert_mocap_sign: bool,
    settle_s: float,
    time_shift_s: str | float,
    phase_selection: str | Sequence[int],
) -> tuple[list[dict[str, Any]], list[tuple[np.ndarray, np.ndarray, str]]]:
    """Analyze one velocity motor/mocap pair and write its figures."""
    _, plt = _require_analysis_runtime()
    motor = load_motor_velocity_csv(pair.motor_csv)
    raw_mocap = load_raw_mocap_csv(pair.mocap_csv)
    mocap = process_raw_mocap(
        raw_mocap,
        trim_frame=trim_frame,
        axis=axis,
        invert_sign=invert_mocap_sign,
    )
    aligned_mocap_time_s, resolved_shift_s = _align_mocap_time(
        motor.elapsed_s,
        motor.feedback_position_deg,
        motor.motor_mech_deg_s,
        mocap,
        time_shift_s=time_shift_s,
        velocity_floor_deg_s=15.0,
    )
    overlap_mask = _overlap_mask(motor.elapsed_s, aligned_mocap_time_s)
    if not np.any(overlap_mask):
        raise RuntimeError(f"No motor/mocap time overlap for velocity pair {pair.label}.")

    overlap_time_s = motor.elapsed_s[overlap_mask]
    mocap_interp_velocity = _interp_series(
        overlap_time_s, aligned_mocap_time_s, mocap.velocity_deg_s
    )
    motor_overlap_velocity = motor.motor_mech_deg_s[overlap_mask]
    full_velocity_error = motor_overlap_velocity - mocap_interp_velocity
    plot_window_mask = _shared_active_velocity_window_mask(
        overlap_time_s,
        motor_overlap_velocity,
        mocap_interp_velocity,
        pad_s=0.0,
    )
    plot_time_s = overlap_time_s[plot_window_mask]
    plot_time_relative_s = plot_time_s - plot_time_s[0]
    plot_motor_velocity = motor_overlap_velocity[plot_window_mask]
    plot_mocap_velocity = mocap_interp_velocity[plot_window_mask]
    plot_velocity_error = full_velocity_error[plot_window_mask]
    plot_start_s = float(plot_time_s[0])
    plot_end_s = float(plot_time_s[-1])

    phase_masks: list[tuple[int, np.ndarray, int]] = []
    phase_ids = np.unique(motor.phase_index)
    for phase_id in phase_ids:
        full_phase_mask = motor.phase_index == phase_id
        phase_command = int(np.median(motor.command_erpm[full_phase_mask]))
        if phase_command == 0:
            continue
        if phase_selection != "all" and phase_id not in phase_selection:
            continue
        phase_masks.append((int(phase_id), full_phase_mask, phase_command))

    fig, axes = plt.subplots(
        2,
        1,
        sharex=True,
        figsize=(11, 6.5),
        constrained_layout=True,
    )
    ax_top, ax_bottom = axes
    for _phase_id, full_phase_mask, phase_command in phase_masks:
        phase_time = motor.elapsed_s[full_phase_mask]
        phase_start_s = max(float(phase_time[0]), plot_start_s)
        phase_end_s = min(float(phase_time[-1]), plot_end_s)
        if phase_end_s <= phase_start_s:
            continue
        ax_top.axvspan(
            phase_start_s - plot_start_s,
            phase_end_s - plot_start_s,
            color="0.7",
            alpha=0.10,
        )
        ax_bottom.axvspan(
            phase_start_s - plot_start_s,
            phase_end_s - plot_start_s,
            color="0.7",
            alpha=0.10,
        )
        ax_top.text(
            phase_start_s - plot_start_s,
            float(np.max(plot_motor_velocity)) if len(plot_motor_velocity) else 0.0,
            f"{phase_command:+d} ERPM",
            fontsize=8,
            va="bottom",
        )
    ax_top.plot(
        plot_time_relative_s,
        plot_motor_velocity,
        color=COLOR_MOTOR,
        linewidth=1.3,
        label="Motor mechanical speed",
    )
    ax_top.plot(
        plot_time_relative_s,
        plot_mocap_velocity,
        color=COLOR_MOCAP,
        linewidth=1.2,
        label="Motion capture speed",
    )
    ax_top.set_title(f"Velocity validation: {pair.label} ERPM run")
    ax_top.set_ylabel("Angular speed [deg/s]")
    ax_top.legend(loc="best")
    ax_top.grid(alpha=0.3)
    ax_bottom.plot(
        plot_time_relative_s,
        plot_velocity_error,
        color=COLOR_ERROR,
        linewidth=1.2,
        label="Motor - mocap",
    )
    ax_bottom.axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    ax_bottom.set_xlabel("Shared-motion time [s]")
    ax_bottom.set_ylabel("Error [deg/s]")
    ax_bottom.legend(loc="best")
    ax_bottom.grid(alpha=0.3)
    _save_figure(fig, output_dir / f"velocity_overlay_{pair.label}")
    plt.close(fig)

    summary_rows: list[dict[str, Any]] = []
    scatter_points: list[tuple[np.ndarray, np.ndarray, str]] = []
    for phase_id, full_phase_mask, phase_command in phase_masks:
        phase_time = motor.elapsed_s[full_phase_mask]
        overlap_start = max(float(phase_time[0]), float(aligned_mocap_time_s[0]))
        overlap_end = min(float(phase_time[-1]), float(aligned_mocap_time_s[-1]))
        if overlap_end <= overlap_start:
            continue
        phase_mask = full_phase_mask & (motor.elapsed_s >= overlap_start) & (
            motor.elapsed_s <= overlap_end
        )
        if np.count_nonzero(phase_mask) < 2:
            continue
        phase_time = motor.elapsed_s[phase_mask]
        phase_motor_speed = motor.motor_mech_deg_s[phase_mask]
        phase_mocap_speed = _interp_series(
            phase_time, aligned_mocap_time_s, mocap.velocity_deg_s
        )
        settled_mask = phase_time >= (phase_time[0] + settle_s)
        if np.count_nonzero(settled_mask) < 2:
            settled_mask = np.ones_like(phase_time, dtype=bool)
        settled_time = phase_time[settled_mask]
        settled_motor_speed = phase_motor_speed[settled_mask]
        settled_mocap_speed = phase_mocap_speed[settled_mask]
        speed_error = settled_motor_speed - settled_mocap_speed
        denominator = max(float(np.mean(np.abs(settled_mocap_speed))), 1e-9)
        settled_motor_norm = _normalize_by_mean_abs(settled_motor_speed)
        settled_mocap_norm = _normalize_by_mean_abs(settled_mocap_speed)
        normalized_speed_rmse = float(
            np.sqrt(np.mean((settled_motor_norm - settled_mocap_norm) ** 2))
        )
        summary_rows.append(
            {
                "run_label": pair.label,
                "motor_csv": str(pair.motor_csv),
                "mocap_csv": str(pair.mocap_csv),
                "trim_frame": mocap.trim_frame,
                "selected_axis": mocap.selected_axis,
                "time_shift_s": round(resolved_shift_s, 6),
                "invert_mocap_sign": int(invert_mocap_sign),
                "phase_index": phase_id,
                "command_erpm": phase_command,
                "phase_start_s": round(float(phase_time[0]), 6),
                "phase_end_s": round(float(phase_time[-1]), 6),
                "settled_start_s": round(float(settled_time[0]), 6),
                "settled_end_s": round(float(settled_time[-1]), 6),
                "active_window_start_s": round(plot_start_s, 6),
                "active_window_end_s": round(plot_end_s, 6),
                "active_window_duration_s": round(plot_end_s - plot_start_s, 6),
                "mean_motor_speed_deg_s": round(float(np.mean(settled_motor_speed)), 6),
                "mean_mocap_speed_deg_s": round(float(np.mean(settled_mocap_speed)), 6),
                "mean_abs_speed_error_deg_s": round(
                    float(np.mean(np.abs(speed_error))), 6
                ),
                "rmse_speed_error_deg_s": round(
                    float(np.sqrt(np.mean(speed_error**2))), 6
                ),
                "peak_abs_speed_error_deg_s": round(
                    float(np.max(np.abs(speed_error))), 6
                ),
                "steady_state_bias_deg_s": round(float(np.mean(speed_error)), 6),
                "percent_error_of_mocap_mean": round(
                    float(np.mean(np.abs(speed_error)) / denominator * 100.0), 6
                ),
                "motor_to_mocap_speed_ratio": round(
                    float(
                        np.mean(np.abs(settled_motor_speed))
                        / max(np.mean(np.abs(settled_mocap_speed)), 1e-9)
                    ),
                    6,
                ),
                "normalized_speed_rmse": round(normalized_speed_rmse, 6),
            }
        )
        scatter_points.append((settled_mocap_speed, settled_motor_speed, pair.label))

    if summary_rows:
        motor_norm = _normalize_by_mean_abs(
            np.asarray(plot_motor_velocity, dtype=float)
        )
        mocap_norm = _normalize_by_mean_abs(np.asarray(plot_mocap_velocity, dtype=float))
        normalized_error = motor_norm - mocap_norm

        consistency_fig, consistency_axes = plt.subplots(
            2,
            1,
            sharex=True,
            figsize=(10.5, 5.8),
            constrained_layout=True,
        )
        consistency_axes[0].plot(
            plot_time_relative_s,
            motor_norm,
            color=COLOR_MOTOR,
            linewidth=1.3,
            label="Motor speed (normalized)",
        )
        consistency_axes[0].plot(
            plot_time_relative_s,
            mocap_norm,
            color=COLOR_MOCAP,
            linewidth=1.2,
            label="Motion capture speed (normalized)",
        )
        consistency_axes[0].set_title(
            f"Velocity consistency: {pair.label} ERPM run"
        )
        consistency_axes[0].set_ylabel("Normalized speed [-]")
        consistency_axes[0].legend(loc="best")
        consistency_axes[0].grid(alpha=0.3)
        consistency_axes[1].plot(
            plot_time_relative_s,
            normalized_error,
            color=COLOR_ERROR,
            linewidth=1.2,
            label="Normalized motor - mocap",
        )
        consistency_axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
        consistency_axes[1].set_xlabel("Shared-motion time [s]")
        consistency_axes[1].set_ylabel("Normalized error [-]")
        consistency_axes[1].legend(loc="best")
        consistency_axes[1].grid(alpha=0.3)
        _save_figure(
            consistency_fig,
            output_dir / f"velocity_consistency_{pair.label}",
        )
        plt.close(consistency_fig)

    return summary_rows, scatter_points


def build_position_target_summary(
    rows: Sequence[dict[str, Any]],
    *,
    output_dir: Path,
) -> None:
    """Create the aggregate final-angle summary plot across position runs."""
    _, plt = _require_analysis_runtime()
    if not rows:
        return
    fig, ax = plt.subplots(figsize=(7.5, 5.5), constrained_layout=True)
    targets = np.asarray([float(row["target_deg"]) for row in rows], dtype=float)
    finals = targets + np.asarray(
        [float(row["final_mocap_error_to_target_deg"]) for row in rows], dtype=float
    )
    run_labels = [str(row["run_label"]) for row in rows]
    for run_label in sorted(set(run_labels)):
        mask = np.asarray([label == run_label for label in run_labels], dtype=bool)
        ax.scatter(targets[mask], finals[mask], label=f"Run {run_label}", s=30)
    low = float(min(np.min(targets), np.min(finals)))
    high = float(max(np.max(targets), np.max(finals)))
    ax.plot([low, high], [low, high], color="black", linestyle="--", linewidth=1.0)
    ax.set_title("Final mocap angle vs commanded target")
    ax.set_xlabel("Commanded target [deg]")
    ax.set_ylabel("Final mocap angle [deg]")
    ax.legend(loc="best")
    ax.grid(alpha=0.3)
    _save_figure(fig, output_dir / "position_target_summary")
    plt.close(fig)


def build_position_functional_summary(
    rows: Sequence[dict[str, Any]],
    *,
    output_dir: Path,
) -> None:
    """Create a retained-segment summary geared toward Chapter 4 prose."""
    _, plt = _require_analysis_runtime()
    summary_rows = summarize_position_functional_rows(rows)
    _write_summary_csv(output_dir / "position_functional_summary.csv", summary_rows)
    if not summary_rows:
        return

    fig, axes = plt.subplots(
        2,
        1,
        sharex=True,
        figsize=(8.0, 6.2),
        constrained_layout=True,
    )
    ax_corr, ax_ratio = axes
    x = np.arange(len(summary_rows), dtype=float)
    labels = [str(row["run_label"]) for row in summary_rows]
    count_labels = [int(row["retained_segment_count"]) for row in summary_rows]

    corr_min = np.asarray(
        [float(row["command_relative_vs_mocap_corr_min"]) for row in summary_rows],
        dtype=float,
    )
    corr_max = np.asarray(
        [float(row["command_relative_vs_mocap_corr_max"]) for row in summary_rows],
        dtype=float,
    )
    corr_median = np.asarray(
        [float(row["command_relative_vs_mocap_corr_median"]) for row in summary_rows],
        dtype=float,
    )
    ratio_min = np.asarray(
        [
            float(row["mocap_to_command_relative_span_ratio_min"])
            for row in summary_rows
        ],
        dtype=float,
    )
    ratio_max = np.asarray(
        [
            float(row["mocap_to_command_relative_span_ratio_max"])
            for row in summary_rows
        ],
        dtype=float,
    )
    ratio_median = np.asarray(
        [
            float(row["mocap_to_command_relative_span_ratio_median"])
            for row in summary_rows
        ],
        dtype=float,
    )

    ax_corr.vlines(x, corr_min, corr_max, color=COLOR_COMMAND, linewidth=2.0, alpha=0.8)
    ax_corr.scatter(x, corr_median, color=COLOR_COMMAND, s=50, zorder=3)
    for x_value, y_value, count in zip(x, corr_median, count_labels, strict=True):
        ax_corr.text(
            x_value,
            min(1.02, y_value + 0.05 if np.isfinite(y_value) else 1.0),
            f"n={count}",
            ha="center",
            va="bottom",
            fontsize=8,
        )
    ax_corr.set_title("Position functional summary across retained segments")
    ax_corr.set_ylabel("Corr(command rel., mocap rel.) [-]")
    ax_corr.set_ylim(-1.05, 1.05)
    ax_corr.grid(alpha=0.3)

    ax_ratio.vlines(
        x, ratio_min, ratio_max, color=COLOR_MOCAP, linewidth=2.0, alpha=0.8
    )
    ax_ratio.scatter(x, ratio_median, color=COLOR_MOCAP, s=50, zorder=3)
    ax_ratio.axhline(1.0, color="black", linestyle="--", linewidth=1.0, alpha=0.6)
    ax_ratio.set_ylabel("Mocap / command span [-]")
    ax_ratio.set_xlabel("Position-step run label [deg]")
    ax_ratio.set_xticks(x, labels)
    ax_ratio.grid(alpha=0.3)

    _save_figure(fig, output_dir / "position_functional_summary")
    plt.close(fig)


def build_velocity_scatter(
    scatter_points: Sequence[tuple[np.ndarray, np.ndarray, str]],
    *,
    output_dir: Path,
) -> None:
    """Create the aggregate motor-vs-mocap speed scatter plot."""
    _, plt = _require_analysis_runtime()
    if not scatter_points:
        return
    fig, ax = plt.subplots(figsize=(7.5, 5.5), constrained_layout=True)
    all_x: list[np.ndarray] = []
    all_y: list[np.ndarray] = []
    for mocap_speed, motor_speed, label in scatter_points:
        ax.scatter(mocap_speed, motor_speed, label=f"Run {label}", s=10, alpha=0.6)
        all_x.append(mocap_speed)
        all_y.append(motor_speed)
    combined_x = np.concatenate(all_x)
    combined_y = np.concatenate(all_y)
    low = float(min(np.min(combined_x), np.min(combined_y)))
    high = float(max(np.max(combined_x), np.max(combined_y)))
    ax.plot([low, high], [low, high], color="black", linestyle="--", linewidth=1.0)
    ax.set_title("Motor speed vs motion-capture speed")
    ax.set_xlabel("Motion-capture speed [deg/s]")
    ax.set_ylabel("Motor mechanical speed [deg/s]")
    ax.legend(loc="best")
    ax.grid(alpha=0.3)
    _save_figure(fig, output_dir / "velocity_scatter")
    plt.close(fig)


def build_velocity_command_response(
    rows: Sequence[dict[str, Any]],
    *,
    output_dir: Path,
) -> None:
    """Create a velocity command-response summary for Chapter 4."""
    _, plt = _require_analysis_runtime()
    summary_rows = summarize_velocity_response_rows(rows)
    _write_summary_csv(output_dir / "velocity_command_response.csv", summary_rows)
    if not summary_rows:
        return

    command_erpm = np.asarray(
        [int(row["command_erpm"]) for row in summary_rows], dtype=float
    )
    motor_speed = np.asarray(
        [float(row["mean_motor_speed_deg_s"]) for row in summary_rows], dtype=float
    )
    mocap_speed = np.asarray(
        [float(row["mean_mocap_speed_deg_s"]) for row in summary_rows], dtype=float
    )
    duration_s = np.asarray(
        [float(row["active_window_duration_s"]) for row in summary_rows], dtype=float
    )

    fig, axes = plt.subplots(
        2,
        1,
        sharex=True,
        figsize=(8.0, 6.2),
        constrained_layout=True,
    )
    ax_speed, ax_duration = axes
    ax_speed.plot(
        command_erpm,
        motor_speed,
        marker="o",
        color=COLOR_MOTOR,
        linewidth=1.6,
        label="Motor-derived mean speed",
    )
    ax_speed.plot(
        command_erpm,
        mocap_speed,
        marker="o",
        color=COLOR_MOCAP,
        linewidth=1.6,
        label="Motion-capture mean speed",
    )
    ax_speed.set_title("Velocity command response across saved runs")
    ax_speed.set_ylabel("Mean speed [deg/s]")
    ax_speed.legend(loc="best")
    ax_speed.grid(alpha=0.3)

    ax_duration.bar(command_erpm, duration_s, width=500.0, color="0.7")
    ax_duration.set_ylabel("Shared-motion window [s]")
    ax_duration.set_xlabel("Commanded speed [ERPM]")
    ax_duration.grid(alpha=0.3)

    _save_figure(fig, output_dir / "velocity_command_response")
    plt.close(fig)


def run_position_mode(args: argparse.Namespace) -> list[dict[str, Any]]:
    """Process all requested position pairs."""
    output_dir = args.out_dir / "position"
    pairs = resolve_position_pairs(
        args.data_root,
        targets=args.targets,
        motor_csv=args.motor_csv,
        mocap_csv=args.mocap_csv,
    )
    all_rows: list[dict[str, Any]] = []
    for pair in pairs:
        print(
            "Processing position pair "
            f"{pair.label}: {pair.motor_csv.name} vs {pair.mocap_csv.name}"
        )
        all_rows.extend(
            analyze_position_pair(
                pair,
                output_dir=output_dir,
                trim_frame=args.trim_frame,
                axis=args.axis,
                invert_mocap_sign=args.invert_mocap_sign,
                boundary_trim_s=args.boundary_trim_s,
                min_segment_s=args.min_segment_s,
                min_angle_span_deg=args.min_angle_span_deg,
                time_shift_s=args.time_shift_s,
            )
        )
    _write_summary_csv(output_dir / "position_summary.csv", all_rows)
    build_position_target_summary(all_rows, output_dir=output_dir)
    build_position_functional_summary(all_rows, output_dir=output_dir)
    return all_rows


def run_velocity_mode(args: argparse.Namespace) -> list[dict[str, Any]]:
    """Process all requested velocity pairs."""
    output_dir = args.out_dir / "velocity"
    pairs = resolve_velocity_pairs(
        args.data_root,
        speeds=args.speeds,
        motor_csv=args.motor_csv,
        mocap_csv=args.mocap_csv,
    )
    all_rows: list[dict[str, Any]] = []
    scatter_points: list[tuple[np.ndarray, np.ndarray, str]] = []
    for pair in pairs:
        print(
            "Processing velocity pair "
            f"{pair.label}: {pair.motor_csv.name} vs {pair.mocap_csv.name}"
        )
        rows, pair_scatter = analyze_velocity_pair(
            pair,
            output_dir=output_dir,
            trim_frame=args.trim_frame,
            axis=args.axis,
            invert_mocap_sign=args.invert_mocap_sign,
            settle_s=args.settle_s,
            time_shift_s=args.time_shift_s,
            phase_selection=args.phase_index,
        )
        all_rows.extend(rows)
        scatter_points.extend(pair_scatter)
    _write_summary_csv(output_dir / "velocity_summary.csv", all_rows)
    build_velocity_scatter(scatter_points, output_dir=output_dir)
    build_velocity_command_response(all_rows, output_dir=output_dir)
    return all_rows


def main() -> int:
    """CLI entry point."""
    args = parse_args()
    if args.command == "position":
        run_position_mode(args)
    elif args.command == "velocity":
        run_velocity_mode(args)
    else:
        run_position_mode(args)
        run_velocity_mode(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
