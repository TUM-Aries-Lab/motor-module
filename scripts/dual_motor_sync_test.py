"""
NOT YET TESTED

Example:
    sudo ./setup_can.sh
    .venv/bin/python scripts/dual_motor_sync_test.py --left-id 0x03 --right-id 0x04 --left-motor-model AK80-6 --right-motor-model AK80-6 --amplitude-deg 60 --freq-hz 0.2 --duration 30

    # AK80-6 with wider amplitude
    .venv/bin/python scripts/dual_motor_sync_test.py
        --left-id 0x01 --right-id 0x02
        --left-motor-model AK80-6 --right-motor-model AK80-6 --amplitude-deg 45 --freq-hz 0.8 --duration 60

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
from motor_python.cube_mars_motor_can import CubeMarsBaseCAN
from motor_python.definitions import MotorModel

SEPARATOR = "=" * 72
HEALTHY_TX_ERR_MAX = 96
HEALTHY_RX_ERR_MAX = 64

MIT_POSITION_LIMIT_DEG = math.degrees(12.5) # TODO: do this in main
SYNC_DIFFERENCE_THRESHOLD_DEG = 5.0  # degrees...also move this to main


CSV_FIELDNAMES = [
    "wall_time_iso",
    "wall_time_epoch_s",
    "elapsed_s",
    "sample_index",
    "commanded_position_deg",
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
    "sync_error_deg",           # left_pos - right_pos
    "current_asymmetry_amps",   # left_current - right_current
]

# ---------------------------------------------------------------------------
# Result container for one dual-sample
# ---------------------------------------------------------------------------
class DualMotorSample(NamedTuple):
    """Container for one dual-motor sample."""

    commanded_position_deg: float
    left: MotorState | None
    right: MotorState | None

    @property
    def sync_error_deg(self) -> float | None:
        """Return the difference between left and right motor positions."""
        if self.left is None or self.right is None:
            return None
        return self.left.position_degrees - self.right.position_degrees

    @property
    def current_asymmetry_amps(self) -> float | None:
        """Return the difference between left and right motor currents."""
        if self.left is None or self.right is None:
            return None
        return self.left.current_amps - self.right.current_amps


# ---------------------------------------------------------------------------
# Helper functions for dual-motor sync test
# ---------------------------------------------------------------------------


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
    motor: CubeMarsBaseCAN,
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

def _check_fault_code(sample: DualMotorSample) -> None:
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
    sample: DualMotorSample,
) -> None:
    now = time.time()
    left, right = sample.left, sample.right
    row = {
        "wall_time_iso": datetime.fromtimestamp(now).isoformat(),
        "wall_time_epoch_s":  f"{now:.6f}",
        "elapsed_s": f"{time.monotonic() - run_start_time:.6f}",
        "sample_index": sample_index,
        "commanded_position_deg": f"{sample.commanded_position_deg:.6f}",
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
        "current_asymmetry_amps": "" if sample.current_asymmetry_amps is None else f"{sample.current_asymmetry_amps:.4f}",
    }
    writer.writerow(row)
    csv_file.flush()

def _print_sample_line(elapsed_s: float, sample_index: int, sample: DualMotorSample) -> None:
    """Print a single line of sample data to the console."""
    left, right = sample.left, sample.right
    sync_str = (
        f"{sample.sync_error_deg:+.2f}°" if sample.sync_error_deg is not None else "n/a"
    )
    warn = " ⚠ SYNC" if (
        sample.sync_error_deg is not None
        and abs(sample.sync_error_deg) > SYNC_DIFFERENCE_THRESHOLD_DEG
    ) else ""
    l_pos = f"{left.position_degrees:+8.2f}°" if left is not None else "    n/a  "
    r_pos = f"{right.position_degrees:+8.2f}°" if right is not None else "    n/a  "
    print(
        f"t={elapsed_s:6.2f}s  i={sample_index:05d}  "
        f"cmd={sample.commanded_position_deg:+8.2f}°  "
        f"L={l_pos}  R={r_pos}  "
        f"sync={sync_str}{warn}"
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Dual-motor synchronization test for CAN motors."
    )
    parser.add_argument(
        "--interface",
        type=str,
        default=CAN_DEFAULTS.interface,
        help=f"CAN interface to use (default: {CAN_DEFAULTS.interface})",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=CAN_DEFAULTS.bitrate,
        help=f"CAN bitrate in bps (default: {CAN_DEFAULTS.bitrate})",
    )
    parser.add_argument(
        "--left-id",
        type=lambda v: int(v, 0),
        default=CAN_DEFAULTS.motor_can_id,
        help=f"Left motor CAN ID (default: {CAN_DEFAULTS.motor_can_id})",
    )
    parser.add_argument(
        "--right-id",
        type=lambda v: int(v, 0),
        default=CAN_DEFAULTS.motor_can_id_2,
        help=f"Right motor CAN ID (default: {CAN_DEFAULTS.motor_can_id_2})",
    )
    parser.add_argument(
        "--left-motor-model",
        choices=list(MotorModel),
        default=MotorModel.AK60_6V3,
        help=f"Left motor model (default: {MotorModel.AK60_6V3})",
    )
    parser.add_argument(
        "--right-motor-model",
        choices=list(MotorModel),
        default=MotorModel.AK60_6V3,
        help=f"Right motor model (default: {MotorModel.AK60_6V3})",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=20.0,
        help="Duration of the test in seconds (default: 20.0)",
    )
    parser.add_argument(
        "--amplitude-deg",
        type=float,
        default=30.0,
        help="Sinusoid amplitude in degrees — peak hip angle (default: 30°)",
    )
    parser.add_argument(
        "--freq-hz",
        type=float,
        default=0.5,
        help="Gait frequency in Hz (default: 0.5 Hz ≈ slow walk)",
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=CAN_DEFAULTS.motor_control_rate_hz,
        help=f"Position command update rate (default: {CAN_DEFAULTS.motor_control_rate_hz} Hz)",
    )
    parser.add_argument(
        "--helper-policy",
        choices=["strict", "fcfd", "legacy"],
        default="fcfd",
    )
    parser.add_argument("--skip-preflight", action="store_true")
    parser.add_argument(
        "--csv-path",
        default=None,
        help="Output CSV path (default: data/csv_logs/dual_sync_<timestamp>.csv)",
    )
    return parser.parse_args()

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = parse_args()

    if args.left_id == args.right_id:
        print("Error: Left and right motor IDs must be different.")
        return 1
    if args.amplitude_deg <= 0:
        print("Error: Amplitude must be positive.")
        return 1
    if args.amplitude_deg > MIT_POSITION_LIMIT_DEG:
        print(f"Error: Amplitude must be <= {MIT_POSITION_LIMIT_DEG:.1f}° to avoid hitting hard stops.")
        return 1
    if args.freq_hz <= 0 or args.control_hz <= 0 or args.duration <= 0:
        print("Error: Frequency, control rate, and duration must be positive.")
        return 1

    period_s = 1.0 / args.control_hz

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = (
        Path(args.csv_path)
        if args.csv_path is not None
        else Path("data/csv_logs") / f"dual_sync_{timestamp}.csv"
    )

    print(SEPARATOR)
    print(f"Dual-Motor Sync Test — {args.left_motor_model} / {args.right_motor_model}  (L=0x{args.left_id:02X}  R=0x{args.right_id:02X})")
    print(SEPARATOR)
    print(f"Interface    : {args.interface}")
    print(f"Amplitude    : ±{args.amplitude_deg:.1f}°")
    print(f"Frequency    : {args.freq_hz:.2f} Hz  (~{1/args.freq_hz:.1f} s/cycle)")
    print(f"Duration     : {args.duration:.1f} s  (~{args.duration * args.freq_hz:.1f} cycles)")
    print(f"Control rate : {args.control_hz:.1f} Hz")
    print(f"Sync warning : >{SYNC_DIFFERENCE_THRESHOLD_DEG}° error flagged live")
    print(f"CSV log      : {csv_path}")
    print("Safety       : keep load clear; be ready to cut power")
    print(SEPARATOR)

    motor_left: CubeMarsBaseCAN | None
    motor_right: CubeMarsBaseCAN | None
    csv_file = None
    csv_writer: csv.DictWriter | None = None
    run_start = 0.0
    total_samples = 0

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

        motor_left.send_neutral_command()
        motor_right.send_neutral_command()

        # Checking communication with both motors
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

        motor_left.zero_position()
        motor_right.zero_position()

        # Read start position
        left_start_status = read_status(motor_left, timeout_s=0.5)
        right_start_status = read_status(motor_right, timeout_s=0.5)
        if left_start_status is not None:
            print(f"Left motor start position: {left_start_status.position_degrees:.2f}°")
        if right_start_status is not None:
            print(f"Right motor start position: {right_start_status.position_degrees:.2f}°")

        # ---------------------------------------------------------------
        # Main control loop — sinusoidal trajectory, same command to both
        # ---------------------------------------------------------------
        print(f"\nStarting synchronised sinusoidal sweep …")
        print(f"  cmd(t) = {args.amplitude_deg:.1f}° · sin(2π · {args.freq_hz:.2f}Hz · t)\n")

        run_start = time.monotonic()
        deadline = run_start + args.duration
        sample_index = 0
        sync_errors: list[float] = []

        next_tick = run_start
        while time.monotonic() < deadline:
            elapsed_s = time.monotonic() - run_start
            commanded_deg = args.amplitude_deg * math.sin(2 * math.pi * args.freq_hz * elapsed_s)

            target_position_deg = _clamp(commanded_deg, -MIT_POSITION_LIMIT_DEG, MIT_POSITION_LIMIT_DEG)

            print(f"t={elapsed_s:.2f}s  cmd={target_position_deg:+.2f}°  sending command …")

            motor_left.set_position(target_position_deg)
            motor_right.set_position(target_position_deg)

            # Read feedback from both motors
            left_status = read_status(motor_left, timeout_s=min(0.5, period_s))
            right_status = read_status(motor_right, timeout_s=min(0.5, period_s))
            sample = DualMotorSample(
                commanded_position_deg=target_position_deg,
                left=left_status,
                right=right_status,
            )

            # Check for faults - raise immediately if found
            _check_fault_code(sample)

            if csv_writer is not None:
                _write_csv_row(
                    writer=csv_writer,
                    csv_file=csv_file,
                    run_start_time=run_start,
                    sample_index=sample_index,
                    sample=sample,
                )

            if sample.sync_error_deg is not None:
                sync_errors.append(abs(sample.sync_error_deg))

            total_samples += int(sample.left is not None) + int(sample.right is not None)

            # Print a sample line every 10 samples
            if sample_index % 10 == 0:
                _print_sample_line(elapsed_s, sample_index, sample)

            sample_index += 1

            # Wait until the next control tick
            next_tick += period_s
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

        # Summary statistics
        print(SEPARATOR)
        print("Test complete.")
        print(f"Total samples collected: {total_samples}")
        if sync_errors:
            print(f"Sync error: min={min(sync_errors):.2f}°  max={max(sync_errors):.2f}°  mean={sum(sync_errors)/len(sync_errors):.2f}°")
            over_threshold = sum(1 for e in sync_errors if e > SYNC_DIFFERENCE_THRESHOLD_DEG)
            print(f"Samples over sync threshold ({SYNC_DIFFERENCE_THRESHOLD_DEG}°): {over_threshold} ({100*over_threshold/len(sync_errors):.1f}%)")
        print(F"CSV log saved to: {csv_path}")
        return 0


    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 130
    except Exception as exc:
        print(f"\nFAIL: {exc}")
        return 1
    finally:
        # Emergency stop — zero both motors regardless of how we exit
        for label, motor in (("LEFT", motor_left), ("RIGHT", motor_right)):
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
