"""
NOT YET TESTED with both motors

Example:
    sudo ./setup_can.sh

    # Single motor, AK60-6
    .venv/bin/python scripts/gait_tracking_test.py --motor-id 0x01 --motor-model AK60-6 --cycles 5

    # Single motor, AK80-6, wider amplitude
    sudo ./setup_can.sh
    .venv/bin/python scripts/gait_tracking_test.py --motor-id 0x03 --motor-model AK80-6 --amplitude-deg 35 --cycles 10 --control-hz 100 --gait-freq-hz 0.2

    # Dual motor (left + right hip)
    .venv/bin/python scripts/gait_tracking_test.py --motor-id 0x01 --motor-model AK60-6 --right-id 0x03 --right-motor-model AK80-6 --amplitude-deg 30 --cycles 5

    IMPORTANT: Maybe increase kp for a better performance?
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

from motor_python.base_motor import MotorState, print_timing_stats
from motor_python.definitions import CAN_DEFAULTS
from motor_python import create_can_motor
from motor_python.can_utils import get_can_state, reset_can_interface
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN, CubeMarsAK806v2CAN

SEPARATOR = "=" * 72
HEALTHY_TX_ERR_MAX = 96
HEALTHY_RX_ERR_MAX = 64
MIT_POSITION_LIMIT_DEG = math.degrees(12.56)  # ±719.7°

# Default pass/fail thresholds
DEFAULT_RMS_THRESHOLD_DEG  = 3.0   # degrees RMS tracking error
DEFAULT_PEAK_THRESHOLD_DEG = 8.0   # degrees peak tracking error

# Print to console every N samples (keeps terminal readable at 100 Hz)
PRINT_EVERY_N_SAMPLES = 10


# ---------------------------------------------------------------------------
# Biomechanically realistic hip gait profile (Winter 2009, normalised) - Source from using Claude
#
# 19 keypoints spanning 0–100% of the gait cycle.
# Values are normalised to unit amplitude; scaled by --amplitude-deg at runtime.
# Shape: flexion peak ~+1.0 at 10% GC, extension peak ~-0.33 at 50% GC.
# ---------------------------------------------------------------------------
_GAIT_KEYPOINTS_PCT: list[tuple[float, float]] = [
    (  0.0,  0.00),   # heel strike — neutral
    (  5.0,  0.35),   # loading response
    ( 10.0,  1.00),   # peak flexion (swing carry-over)
    ( 15.0,  0.80),
    ( 20.0,  0.50),   # mid-stance descent
    ( 25.0,  0.20),
    ( 30.0,  0.00),   # neutral crossing
    ( 35.0, -0.10),
    ( 40.0, -0.25),
    ( 50.0, -0.33),   # peak extension (terminal stance / push-off)
    ( 55.0, -0.25),
    ( 60.0, -0.10),   # toe-off — start of swing
    ( 65.0,  0.10),
    ( 70.0,  0.40),   # early swing acceleration
    ( 75.0,  0.70),
    ( 80.0,  0.90),
    ( 85.0,  1.00),   # peak flexion (mid-swing)
    ( 90.0,  0.80),
    ( 95.0,  0.40),
    (100.0,  0.00),   # next heel strike
]


def _build_gait_profile(
    amplitude_deg: float,
    control_hz: float,
    gait_freq_hz: float,
) -> list[float]:
    """Return a single-cycle position array (degrees) at control_hz resolution.

    Uses cosine-interpolation between keypoints for smooth transitions.
    """
    n_samples = max(2, round(control_hz / gait_freq_hz))
    profile: list[float] = []

    kp = _GAIT_KEYPOINTS_PCT  # shorthand

    for i in range(n_samples):
        pct = 100.0 * i / n_samples  # 0 … <100

        # Find surrounding keypoints
        lo_idx = 0
        for j in range(len(kp) - 1):
            if kp[j][0] <= pct < kp[j + 1][0]:
                lo_idx = j
                break

        p0, v0 = kp[lo_idx]
        p1, v1 = kp[lo_idx + 1]

        # Cosine interpolation for smooth transitions
        t = (pct - p0) / (p1 - p0) if (p1 - p0) > 1e-9 else 0.0
        smooth_t = (1.0 - math.cos(math.pi * t)) / 2.0
        normalised = v0 + (v1 - v0) * smooth_t

        profile.append(amplitude_deg * normalised)

    return profile


# ---------------------------------------------------------------------------
# Sample container
# ---------------------------------------------------------------------------

class GaitSample(NamedTuple):
    """One control-loop sample (single or dual motor)."""
    commanded_deg: float
    gait_cycle_pct: float          # 0–100 % within current stride
    cycle_index: int
    left: MotorState | None        # always the primary / only motor
    right: MotorState | None       # None in single-motor mode

    @property
    def left_tracking_error_deg(self) -> float | None:
        if self.left is None:
            return None
        return self.left.position_degrees - self.commanded_deg

    @property
    def right_tracking_error_deg(self) -> float | None:
        if self.right is None:
            return None
        return self.right.position_degrees - self.commanded_deg

    @property
    def sync_error_deg(self) -> float | None:
        """Left − Right position difference (dual-motor mode only)."""
        if self.left is None or self.right is None:
            return None
        return self.left.position_degrees - self.right.position_degrees


# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------

CSV_FIELDNAMES = [
    "wall_time_iso",
    "wall_time_epoch_s",
    "elapsed_s",
    "sample_index",
    "cycle_index",
    "gait_cycle_pct",
    "commanded_position_deg",
    # Left / primary motor
    "left_feedback_received",
    "left_position_deg",
    "left_tracking_error_deg",
    "left_speed_erpm",
    "left_current_amps",
    "left_temperature_c",
    "left_error_code",
    "left_error_description",
    # Right motor (empty in single-motor mode)
    "right_feedback_received",
    "right_position_deg",
    "right_tracking_error_deg",
    "right_speed_erpm",
    "right_current_amps",
    "right_temperature_c",
    "right_error_code",
    "right_error_description",
    # Derived
    "sync_error_deg",
]

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

def read_status(
    motor: CubeMarsAK606v3CAN | CubeMarsAK806v2CAN,
    timeout_s: float = 0.1,
) -> MotorState | None:
    """Read the motor status with a timeout."""
    status = motor._receive_feedback(timeout=timeout_s)
    if status is not None:
            return status
    return motor.get_status()  # fallback to last known status if no new feedback

def _clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp a value to a range."""
    return max(min_value, min(max_value, value))

def _check_fault_code(sample: GaitSample) -> None:
    """Check for fault codes in the sample and raise immediately."""
    for label, status in (("LEFT", sample.left), ("RIGHT", sample.right)):
        if status is None:
            continue
        if status.error_code != 0:
            raise RuntimeError(
                f"{label} motor fault: code={status.error_code} desc={status.error_description}"
            )

def _write_csv_row(
    writer: csv.DictWriter,
    csv_file,
    *,
    run_start_time: float,
    sample_index: int,
    sample: GaitSample,
) -> None:
    now = time.time()
    left, right = sample.left, sample.right
    row = {
        "wall_time_iso": datetime.fromtimestamp(now).isoformat(),
        "wall_time_epoch_s":  f"{now:.6f}",
        "elapsed_s": f"{time.monotonic() - run_start_time:.6f}",
        "sample_index": sample_index,
        "cycle_index":    sample.cycle_index,
        "gait_cycle_pct": f"{sample.gait_cycle_pct:.2f}",
        "commanded_position_deg": f"{sample.commanded_deg:.6f}",
        #Left motor
        "left_feedback_received": int(left is not None),
        "left_position_deg":      "" if left is None else f"{left.position_degrees:.4f}",
        "left_speed_erpm":        "" if left is None else left.speed_erpm,
        "left_current_amps":      "" if left is None else f"{left.current_amps:.4f}",
        "left_temperature_c":     "" if left is None else left.temperature_celsius,
        "left_error_code":        "" if left is None else left.error_code,
        "left_error_description": "" if left is None else left.error_description,
        # Right
        "right_feedback_received": int(right is not None),
        "right_position_deg":      "" if right is None else f"{right.position_degrees:.4f}",
        "right_speed_erpm":        "" if right is None else right.speed_erpm,
        "right_current_amps":      "" if right is None else f"{right.current_amps:.4f}",
        "right_temperature_c":     "" if right is None else right.temperature_celsius,
        "right_error_code":        "" if right is None else right.error_code,
        "right_error_description": "" if right is None else right.error_description,
        # Derived
        "sync_error_deg":         "" if sample.sync_error_deg is None else f"{sample.sync_error_deg:.4f}",
    }
    writer.writerow(row)
    csv_file.flush()

def _print_sample_line(elapsed_s: float, sample_index: int, sample: GaitSample, dual: bool) -> None:
    """Print a single line of sample data to the console."""
    left, right = sample.left, sample.right

    left_error = sample.left_tracking_error_deg
    right_error = sample.right_tracking_error_deg

    left_err_str  = f"{left_error:+.2f}°" if left_error is not None else "  n/a  "
    left_pos_str  = f"{left.position_degrees:+8.2f}°" if left is not None else "   n/a   "

    base = (
        f"t={elapsed_s:6.2f}s  cyc={sample.cycle_index:03d}  "
        f"gc={sample.gait_cycle_pct:5.1f}%  "
        f"cmd={sample.commanded_deg:+7.2f}°  "
        f"L_pos={left_pos_str}  L_err={left_err_str}"
    )

    if dual and right is not None:
        right_err_str = f"{right_error:+.2f}°" if right_error is not None else "  n/a  "
        right_pos_str = f"{right.position_degrees:+8.2f}°"
        sync_str  = f"{sample.sync_error_deg:+.2f}°" if sample.sync_error_deg is not None else "n/a"
        base += f"  R_pos={right_pos_str}  R_err={right_err_str}  sync={sync_str}"

    print(base)


# ---------------------------------------------------------------------------
# Statistics helpers
# ---------------------------------------------------------------------------

def _rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def _print_tracking_summary(
    label: str,
    errors: list[float],
    rms_threshold: float,
    peak_threshold: float,
) -> bool:
    """Print per-motor tracking summary. Returns True if PASS."""
    if not errors:
        print(f"  {label}: no feedback received — cannot evaluate")
        return False

    rms_err  = _rms(errors)
    peak_err = max(abs(e) for e in errors)
    mean_err = sum(errors) / len(errors)

    rms_pass  = rms_err  < rms_threshold
    peak_pass = peak_err < peak_threshold
    passed    = rms_pass and peak_pass

    status = "PASS ✓" if passed else "FAIL ✗"
    print(f"  {label} [{status}]")
    print(f"    RMS error  : {rms_err:.3f}°  (threshold {rms_threshold:.1f}°)  {'OK' if rms_pass  else 'EXCEEDED'}")
    print(f"    Peak error : {peak_err:.3f}°  (threshold {peak_threshold:.1f}°)  {'OK' if peak_pass else 'EXCEEDED'}")
    print(f"    Mean error : {mean_err:+.3f}°  (signed bias)")
    print(f"    Samples    : {len(errors)}")
    return passed


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Gait-profile position tracking test for exosuit hip motor(s). "
            "Replays Winter normative hip trajectory and measures tracking error."
        )
    )
    p.add_argument("--interface", default="can0")
    p.add_argument("--bitrate", type=int, default=CAN_DEFAULTS.bitrate)

    # Primary (left / single) motor
    p.add_argument(
        "--motor-id",
        type=lambda value: int(value, 0),
        default=0x03,
        help="Motor CAN ID in decimal or hex (default: 0x03)",
    )
    p.add_argument(
        "--motor-model",
        choices=["AK60-6", "AK80-6"],
        default="AK60-6",
    )

    # Optional right motor (dual-motor mode)
    p.add_argument(
        "--right-id",
        type=lambda v: int(v, 0),
        default=None,
        help="Right motor CAN ID — omit for single-motor mode",
    )
    p.add_argument(
        "--right-motor-model",
        choices=["AK60-6", "AK80-6"],
        default="AK60-6",
    )

    # Trajectory parameters
    p.add_argument(
        "--amplitude-deg",
        type=float,
        default=30.0,
        help="Peak hip flexion in degrees (scales the gait profile, default: 30°)",
    )
    p.add_argument(
        "--gait-freq-hz",
        type=float,
        default=1.0,
        help="Stride frequency in Hz (default: 1.0 Hz ≈ normal walk)",
    )
    p.add_argument(
        "--cycles",
        type=int,
        default=5,
        help="Number of complete gait cycles to run (default: 5)",
    )
    p.add_argument(
        "--control-hz",
        type=float,
        default=CAN_DEFAULTS.motor_control_rate_hz,
        help=f"Control loop rate in Hz (default: {CAN_DEFAULTS.motor_control_rate_hz})",
    )

    # Pass/fail thresholds
    p.add_argument(
        "--rms-threshold-deg",
        type=float,
        default=DEFAULT_RMS_THRESHOLD_DEG,
        help=f"RMS tracking error pass threshold in degrees (default: {DEFAULT_RMS_THRESHOLD_DEG}°)",
    )
    p.add_argument(
        "--peak-threshold-deg",
        type=float,
        default=DEFAULT_PEAK_THRESHOLD_DEG,
        help=f"Peak tracking error pass threshold in degrees (default: {DEFAULT_PEAK_THRESHOLD_DEG}°)",
    )

    p.add_argument(
        "--helper-policy",
        choices=["strict", "fcfd", "legacy"],
        default="fcfd",
    )
    p.add_argument("--skip-preflight", action="store_true")
    p.add_argument(
        "--csv-path",
        default=None,
        help="Output CSV path (default: data/csv_logs/gait_tracking_<timestamp>.csv)",
    )
    return p.parse_args()

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = parse_args()

    dual_mode = args.right_id is not None

    if args.amplitude_deg <= 0:
        print("--amplitude-deg must be > 0")
        return 1
    if args.amplitude_deg > MIT_POSITION_LIMIT_DEG:
        print(f"FAIL: --amplitude-deg {args.amplitude_deg:.1f}° exceeds MIT limit {MIT_POSITION_LIMIT_DEG:.1f}°")
        return 1
    if args.gait_freq_hz <= 0 or args.control_hz <= 0:
        print("FAIL: --gait-freq-hz and --control-hz must be > 0")
        return 1
    if args.cycles < 1:
        print("FAIL: --cycles must be >= 1")
        return 1
    if dual_mode and args.right_id == args.motor_id:
        print("FAIL: --right-id must differ from --motor-id")
        return 1

    samples_per_cycle = round(args.control_hz / args.gait_freq_hz)
    if samples_per_cycle < 10:
        print(
            f"FAIL: only {samples_per_cycle} samples/cycle at "
            f"{args.control_hz:.0f} Hz / {args.gait_freq_hz:.2f} Hz — "
            "increase control-hz or reduce gait-freq-hz"
        )
        return 1

    # Building gait profile once (one full stride at control_hz resolution)
    gait_profile = _build_gait_profile(
        amplitude_deg=args.amplitude_deg,
        control_hz=args.control_hz,
        gait_freq_hz=args.gait_freq_hz,
    )
    period_s = 1.0/args.control_hz
    total_samples_planned = len(gait_profile) * args.cycles
    estimated_duration_s = total_samples_planned * period_s

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = (
        Path(args.csv_path)
        if args.csv_path is not None
        else Path("data/csv_logs") / f"gait_tracking_{timestamp}.csv"
    )

    mode_str = (
        f"Dual mode Left={args.motor_model} ID=0x{args.motor_id:02X} "
        f"Right={args.right_motor_model} ID=0x{args.right_id:02X} "
        if dual_mode else
        f"Single mode Left={args.motor_model} ID=0x{args.motor_id:02X} "
    )

    print(SEPARATOR)
    print(f"Gait-Profile Tracking Test — {mode_str}")
    print(SEPARATOR)
    print(f"Interface         : {args.interface}")
    print(f"Amplitude         : ±{args.amplitude_deg:.1f}° (peak flexion)")
    print(f"Gait frequency    : {args.gait_freq_hz:.2f} Hz  (stride period {1/args.gait_freq_hz:.2f} s)")
    print(f"Cycles            : {args.cycles}")
    print(f"Samples/cycle     : {len(gait_profile)}")
    print(f"Control rate      : {args.control_hz:.1f} Hz")
    print(f"Estimated duration: ~{estimated_duration_s:.1f} s")
    print(f"RMS  threshold    : {args.rms_threshold_deg:.1f}°")
    print(f"Peak threshold    : {args.peak_threshold_deg:.1f}°")
    print(f"CSV log           : {csv_path}")
    print("Safety            : keep load clear; be ready to cut power")
    print(SEPARATOR)

    motor_left: CubeMarsAK606v3CAN | CubeMarsAK806v2CAN | None = None
    motor_right: CubeMarsAK606v3CAN | CubeMarsAK806v2CAN | None = None
    csv_file = None
    csv_writer: csv.DictWriter | None = None
    run_start = 0.0
    total_samples = 0

    left_errors: list[float] = []
    right_errors: list[float] = []

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
            args.motor_model,
            motor_can_id=args.motor_id,
            interface=args.interface,
            bitrate=args.bitrate,
            helper_policy=args.helper_policy,
        )
        if dual_mode:
            print("Initializing right motor …")
            motor_right = create_can_motor(
                args.right_motor_model,
                motor_can_id=args.right_id,
                interface=args.interface,
                bitrate=args.bitrate,
                helper_policy=args.helper_policy,
            )
        if not motor_left.connected:
            print("FAIL: LEFT motor not connected")
            return 1
        if dual_mode and not motor_right.connected:
            print("FAIL: RIGHT motor not connected")
            return 1

        motor_left.send_neutral_command()
        if dual_mode:
            motor_right.send_neutral_command()

        # Checking communication with both motors
        print("Check Communication - Left Motor")
        if not motor_left.check_communication():
            print("Error: Left motor communication check failed.")
            return 1
        print("Left motor communication check passed.")
        motor_left.zero_position()
        if dual_mode:
            print("Check Communication - Right Motor")
            if not motor_right.check_communication():
                print("Error: Right motor communication check failed.")
                return 1
            print("Right motor communication check passed.")
            motor_right.zero_position()

        # Read start position
        left_start_status = read_status(motor_left, timeout_s=0.5)
        right_start_status = read_status(motor_right, timeout_s=0.5) if dual_mode else None
        if left_start_status is not None:
            print(f"Left motor start position: {left_start_status.position_degrees:.2f}°")
        if right_start_status is not None:
            print(f"Right motor start position: {right_start_status.position_degrees:.2f}°")

        # -------------------------------------------------------------------
        # Main control loop — cycle through gait_profile args.cycles times
        # -------------------------------------------------------------------
        print(f"\nStarting gait profile replay  ({args.cycles} cycles x {len(gait_profile)} samples) …\n")

        run_start = time.monotonic()
        sample_index = 0
        next_tick = run_start

        for cycle_index in range(args.cycles):
            print(f"Cycle {cycle_index + 1}/{args.cycles} …")

            for step_index, target_deg in enumerate(gait_profile):
                target_degree = _clamp(target_deg, -MIT_POSITION_LIMIT_DEG, MIT_POSITION_LIMIT_DEG)
                gait_cycle_pct = 100.0 * step_index / len(gait_profile)

                #Send command to motors
                motor_left.set_position(target_degree)
                if dual_mode:
                    motor_right.set_position(target_degree)

                # Read feedback
                left_status = read_status(motor_left, timeout_s=min(0.06, period_s))
                right_status = read_status(motor_right, timeout_s=min(0.06, period_s)) if dual_mode else None

                sample = GaitSample(
                    commanded_deg=target_degree,
                    gait_cycle_pct=gait_cycle_pct,
                    cycle_index=cycle_index,
                    left=left_status,
                    right=right_status,
                )

                # Check for faults - raise immediately if any
                _check_fault_code(sample)


                # Log to CSV
                if sample.left_tracking_error_deg is not None:
                    left_errors.append(sample.left_tracking_error_deg)
                if sample.right_tracking_error_deg is not None:
                    right_errors.append(sample.right_tracking_error_deg)

                total_samples += int(left_status is not None) + int(right_status is not None)

                if csv_writer is not None:
                    _write_csv_row(
                        writer=csv_writer,
                        csv_file=csv_file,
                        run_start_time=run_start,
                        sample_index=sample_index,
                        sample=sample,
                    )

                if sample_index % PRINT_EVERY_N_SAMPLES == 0:
                    elapsed = time.monotonic() - run_start
                    _print_sample_line(elapsed, sample_index, sample, dual=dual_mode)

                sample_index += 1

                # Hold until next tick
                next_tick += period_s
                sleep_time = next_tick - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)

        # -------------------------------------------------------------------
        # Test complete — print summary
        # -------------------------------------------------------------------
        elapsed_total = time.monotonic() - run_start
        print(SEPARATOR)
        print(f"Test complete: {total_samples} samples in {elapsed_total:.2f}s (~{total_samples/elapsed_total:.1f} Hz)")
        print(SEPARATOR)
        print("Tracking error summary:")

        left_passed = _print_tracking_summary(
            label="LEFT",
            errors=left_errors,
            rms_threshold=args.rms_threshold_deg,
            peak_threshold=args.peak_threshold_deg,
        )
        right_passed = True
        if dual_mode:
            right_passed = _print_tracking_summary(
                label="RIGHT",
                errors=right_errors,
                rms_threshold=args.rms_threshold_deg,
                peak_threshold=args.peak_threshold_deg,
            )

        overall_passed = left_passed and right_passed
        print(SEPARATOR)
        print(f"Overall result: {'PASS ✓' if overall_passed else 'FAIL ✗'}")
        print(f"CSV log saved to: {csv_path}")
        return 0 if overall_passed else 1

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 130
    except Exception as exc:
        print(f"\nFAIL: {exc}")
        return 1
    finally:
        # Emergency stop — always send neutral before closing
        for motor in (motor_left, motor_right):
            if motor is not None:
                try:
                    motor.send_neutral_command()
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
