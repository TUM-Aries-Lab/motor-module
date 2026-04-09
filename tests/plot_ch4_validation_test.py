"""Tests for the Chapter 4 validation plotter."""

from __future__ import annotations

import csv
import importlib.util
import sys
from pathlib import Path

import numpy as np

SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "plot_graph.py"


def _load_script_module():
    """Import the plotter script as a module for testing."""
    spec = importlib.util.spec_from_file_location("plot_graph", SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_csv(path: Path, header: list[str], rows: list[list[object]]) -> None:
    """Write a small CSV fixture file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)


def _write_position_motor_csv(path: Path) -> None:
    """Create a minimal MIT position-step motor CSV."""
    _write_csv(
        path,
        [
            "elapsed_s",
            "tick_index",
            "direction",
            "command_position_deg",
            "segment_target_deg",
            "feedback_received",
            "feedback_position_deg",
            "feedback_speed_erpm",
        ],
        [
            [0.0, 0, 1, 22.0, 30.0, 1, 22.0, 120.0],
            [0.5, 1, 1, 28.0, 30.0, 1, 27.5, 130.0],
            [1.0, 2, 1, 34.0, 30.0, 1, 33.0, 140.0],
        ],
    )


def _write_velocity_motor_csv(path: Path) -> None:
    """Create a minimal MIT velocity-validation motor CSV."""
    _write_csv(
        path,
        [
            "elapsed_s",
            "phase_index",
            "phase_command_erpm",
            "command_erpm",
            "feedback_position_deg",
            "feedback_speed_erpm",
        ],
        [
            [0.0, 0, 1000, 1000, 10.0, 1000.0],
            [0.5, 0, 1000, 1000, 20.0, 1270.0],
            [1.0, 0, 1000, 1000, 30.0, 1400.0],
        ],
    )


def _write_mocap_csv(path: Path) -> None:
    """Create a minimal Vicon-style raw mocap CSV with a small preamble."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        handle.write("Trajectory export\n")
        handle.write("Frame,Sub Frame,RX,RY,RZ,TX,TY,TZ\n")
        handle.write("units,,deg,deg,deg,mm,mm,mm\n")
        writer = csv.writer(handle)
        writer.writerow([1, 0, 0.0, 0.0, 90.0071, 1.0, 2.0, 3.0])
        writer.writerow([2, 0, 0.2, 0.1, 90.2071, 1.5, 2.5, 3.5])
        writer.writerow([3, 0, 0.4, 0.1, 90.4071, 2.0, 3.0, 4.0])


def _make_curated_data_root(tmp_path: Path) -> Path:
    """Create a tiny curated data root used by resolve_* tests."""
    data_root = tmp_path / "CSV"
    motion_root = data_root / "motion-capture-data"
    for name in (
        "mit_position_steps_30.csv",
        "mit_position_steps_50.csv",
        "mit_position_steps_90.csv",
    ):
        _write_position_motor_csv(data_root / name)
    for name in (
        "verify_set_velocity_1000.csv",
        "verify_set_velocity_3000.csv",
        "verify_set_velocity_5000.csv",
    ):
        _write_velocity_motor_csv(data_root / name)
    for name in (
        "position30.csv",
        "position50.csv",
        "position90_new.csv",
        "velo1000.csv",
        "velo3000.csv",
        "velo5000.csv",
    ):
        _write_mocap_csv(motion_root / name)
    return data_root


def test_resolve_position_pairs_maps_90_to_position90_new(tmp_path: Path) -> None:
    """The strict curated mapping should use the *_new file for 90 degrees."""
    module = _load_script_module()
    data_root = _make_curated_data_root(tmp_path)
    pairs = module.resolve_position_pairs(data_root, targets=[90])
    assert len(pairs) == 1
    assert pairs[0].motor_csv.name == "mit_position_steps_90.csv"
    assert pairs[0].mocap_csv.name == "position90_new.csv"


def test_resolve_velocity_pairs_maps_known_speed_files(tmp_path: Path) -> None:
    """Velocity pairing should use the curated verify/velo filenames."""
    module = _load_script_module()
    data_root = _make_curated_data_root(tmp_path)
    pairs = module.resolve_velocity_pairs(data_root, speeds=[1000, 5000])
    assert [pair.motor_csv.name for pair in pairs] == [
        "verify_set_velocity_1000.csv",
        "verify_set_velocity_5000.csv",
    ]
    assert [pair.mocap_csv.name for pair in pairs] == [
        "velo1000.csv",
        "velo5000.csv",
    ]


def test_load_motor_position_csv_reads_expected_columns(tmp_path: Path) -> None:
    """Motor position CSV parsing should expose the MIT position columns."""
    module = _load_script_module()
    csv_path = tmp_path / "mit_position_steps_30.csv"
    _write_position_motor_csv(csv_path)
    data = module.load_motor_position_csv(csv_path)
    assert len(data.elapsed_s) == 3
    assert data.feedback_received.dtype == np.int64
    assert data.elapsed_s[0] == 0.0
    assert data.command_position_deg[0] == 22.0
    assert data.feedback_position_deg[0] == 22.0


def test_load_motor_velocity_csv_computes_mechanical_speed(tmp_path: Path) -> None:
    """Motor velocity parsing should convert ERPM to mechanical deg/s."""
    module = _load_script_module()
    csv_path = tmp_path / "verify_set_velocity_1000.csv"
    _write_velocity_motor_csv(csv_path)
    data = module.load_motor_velocity_csv(csv_path)
    assert len(data.elapsed_s) == 3
    assert data.feedback_speed_erpm[1] == 1270.0
    expected = data.feedback_speed_erpm[1] * module.MECH_DEG_PER_SEC_PER_ERPM
    assert np.isclose(data.motor_mech_deg_s[1], expected)


def test_load_raw_mocap_csv_skips_preamble_and_units_rows(tmp_path: Path) -> None:
    """Raw Vicon parsing should start at the first numeric frame row."""
    module = _load_script_module()
    mocap_path = tmp_path / "motion-capture-data" / "position30.csv"
    _write_mocap_csv(mocap_path)
    mocap = module.load_raw_mocap_csv(mocap_path)
    assert len(mocap.frame) == 3
    assert mocap.frame[0] == 1.0
    assert mocap.sub_frame[0] == 0.0
    assert np.isclose(mocap.rz_deg[0], 90.0071)


def test_find_position_segments_keeps_one_direction_windows(tmp_path: Path) -> None:
    """Retained segments should be trimmed one-direction windows with span."""
    module = _load_script_module()
    csv_path = tmp_path / "mit_position_steps_segments.csv"
    rows = []
    for index in range(9):
        rows.append(
            [index * 0.25, index, 1, float(index), 10.0, 1, float(index), 100.0]
        )
    _write_csv(
        csv_path,
        [
            "elapsed_s",
            "tick_index",
            "direction",
            "command_position_deg",
            "segment_target_deg",
            "feedback_received",
            "feedback_position_deg",
            "feedback_speed_erpm",
        ],
        rows,
    )
    motor = module.load_motor_position_csv(csv_path)
    segments = module.find_position_segments(
        motor,
        boundary_trim_s=0.25,
        min_segment_s=1.0,
        min_angle_span_deg=5.0,
    )
    assert segments
    assert all(segment.end_time_s > segment.start_time_s for segment in segments)
    assert all(segment.angle_span_deg >= 5.0 for segment in segments)
    assert {segment.direction for segment in segments}.issubset({-1, 1})


def test_shared_active_velocity_window_mask_crops_to_common_motion() -> None:
    """Velocity plotting should trim to the shared active-motion window."""
    module = _load_script_module()
    time_s = np.arange(8, dtype=float)
    motor_speed = np.array([0.0, 0.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0])
    mocap_speed = np.array([0.0, 0.0, 60.0, 60.0, 60.0, 0.0, 0.0, 0.0])
    mask = module._shared_active_velocity_window_mask(
        time_s,
        motor_speed,
        mocap_speed,
        floor_deg_s=10.0,
        threshold_fraction=0.1,
        smoothing_window=1,
        pad_s=0.0,
        edge_trim_samples=0,
    )
    assert mask.tolist() == [False, False, True, True, True, False, False, False]


def test_shared_active_velocity_window_mask_uses_common_motion_interval() -> None:
    """Velocity plot masking should keep only the interval where both traces are active."""
    module = _load_script_module()
    time_s = np.arange(10, dtype=float)
    motor_speed = np.array(
        [0.0, 0.0, 120.0, 120.0, 120.0, 120.0, 120.0, 120.0, 120.0, 120.0]
    )
    mocap_speed = np.array([0.0, 0.0, 60.0, 60.0, 60.0, 60.0, 0.0, 0.0, 0.0, 0.0])
    mask = module._shared_active_velocity_window_mask(
        time_s,
        motor_speed,
        mocap_speed,
        floor_deg_s=20.0,
        threshold_fraction=0.15,
        smoothing_window=1,
        pad_s=0.0,
        edge_trim_samples=0,
    )
    kept = np.flatnonzero(mask)
    assert kept.tolist() == [2, 3, 4, 5]


def test_summarize_position_functional_rows_groups_by_run() -> None:
    """Run-level position summaries should aggregate retained-segment metrics."""
    module = _load_script_module()
    rows = [
        {
            "run_label": "90",
            "segment_duration_s": 10.0,
            "command_relative_vs_mocap_corr": 0.99,
            "feedback_relative_vs_mocap_corr": 0.80,
            "mocap_to_command_relative_span_ratio": 1.02,
            "feedback_to_command_relative_span_ratio": 0.45,
        },
        {
            "run_label": "90",
            "segment_duration_s": 12.0,
            "command_relative_vs_mocap_corr": 0.97,
            "feedback_relative_vs_mocap_corr": 0.82,
            "mocap_to_command_relative_span_ratio": 0.98,
            "feedback_to_command_relative_span_ratio": 0.47,
        },
        {
            "run_label": "50",
            "segment_duration_s": 8.0,
            "command_relative_vs_mocap_corr": 0.80,
            "feedback_relative_vs_mocap_corr": 0.70,
            "mocap_to_command_relative_span_ratio": 0.90,
            "feedback_to_command_relative_span_ratio": 0.60,
        },
    ]
    summary = module.summarize_position_functional_rows(rows)
    assert [row["run_label"] for row in summary] == ["50", "90"]
    assert summary[0]["retained_segment_count"] == 1
    assert np.isclose(summary[1]["retained_duration_s"], 22.0)
    assert np.isclose(summary[1]["command_relative_vs_mocap_corr_median"], 0.98)
    assert np.isclose(summary[1]["mocap_to_command_relative_span_ratio_median"], 1.0)


def test_summarize_velocity_response_rows_groups_by_command() -> None:
    """Velocity summaries should aggregate one or more rows per commanded ERPM."""
    module = _load_script_module()
    rows = [
        {
            "command_erpm": 1000,
            "mean_motor_speed_deg_s": 900.0,
            "mean_mocap_speed_deg_s": 220.0,
            "active_window_duration_s": 70.0,
            "motor_to_mocap_speed_ratio": 4.1,
            "normalized_speed_rmse": 0.06,
        },
        {
            "command_erpm": 1000,
            "mean_motor_speed_deg_s": 920.0,
            "mean_mocap_speed_deg_s": 230.0,
            "active_window_duration_s": 74.0,
            "motor_to_mocap_speed_ratio": 4.0,
            "normalized_speed_rmse": 0.05,
        },
        {
            "command_erpm": 3000,
            "mean_motor_speed_deg_s": 3300.0,
            "mean_mocap_speed_deg_s": 840.0,
            "active_window_duration_s": 90.0,
            "motor_to_mocap_speed_ratio": 3.93,
            "normalized_speed_rmse": 0.02,
        },
    ]
    summary = module.summarize_velocity_response_rows(rows)
    assert [row["command_erpm"] for row in summary] == [1000, 3000]
    assert summary[0]["phase_count"] == 2
    assert np.isclose(summary[0]["mean_motor_speed_deg_s"], 910.0)
    assert np.isclose(summary[0]["active_window_duration_s"], 72.0)
    assert np.isclose(summary[0]["normalized_speed_rmse"], 0.055)
