"""Motion capture validation script — arm sweep with IMU.

Moves an arm attached to the motor through a defined angle sequence while
logging motor feedback at 20 Hz to a CSV file.  The CSV can be compared
against the motion capture / IMU data to verify angle and velocity accuracy.

Arm sequence (motor zero = arm hanging straight down):
  0°  → 90°   smooth rise to horizontal          (parallel to ground)
  90° → 120°  gentle 30° lift above horizontal
  120° → 80°  smooth 40° drop below last peak
  80°  → 0°   return to home

Run:
    sudo ./setup_can.sh          # bring up can0 if not already up
    source .venv/bin/activate
    python motion_capture_test.py

Output CSV is saved to data/logs/mocap_<timestamp>.csv
"""

import csv
import sys
import time
from datetime import datetime
from pathlib import Path

# Allow running from the project root without installing the package
sys.path.insert(0, str(Path(__file__).parent / "src"))

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

# ---------------------------------------------------------------------------
# Motion parameters — adjust to taste
# ---------------------------------------------------------------------------

# Speed for all moves.  Slow enough for smooth IMU readings, fast enough
# to be useful.  5 000 ERPM ≈ comfortable human joint speed for this setup.
MOVE_SPEED_ERPM = 5_000

# Acceleration limit — low value = soft ramp-in/ramp-out (gentler on IMU)
ACCEL_ERPM_PER_SEC = 3_000

# How long to hold each waypoint before moving to the next one (seconds)
HOLD_TIME = 2.0

# Feedback sampling rate during moves (Hz)
SAMPLE_HZ = 20
SAMPLE_INTERVAL = 1.0 / SAMPLE_HZ

# Output directory (same folder the existing log files use)
LOG_DIR = Path(__file__).parent / "data" / "logs"

# ---------------------------------------------------------------------------
# Waypoint sequence
# ---------------------------------------------------------------------------

WAYPOINTS = [
    (  0.0, "home — arm hanging straight down"),
    ( 90.0, "arm horizontal (parallel to ground)"),
    (120.0, "30° above horizontal"),
    ( 80.0, "40° drop from peak"),
    (  0.0, "return home"),
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def move_and_sample(
    motor: CubeMarsAK606v3CAN,
    target_degrees: float,
    label: str,
    writer: csv.DictWriter,
    t0: float,
) -> None:
    """Command a smooth move to target_degrees and log feedback until settled.

    :param motor: CAN motor instance (already enabled).
    :param target_degrees: Target angle in degrees.
    :param label: Human-readable label for this waypoint.
    :param writer: CSV writer to write feedback rows to.
    :param t0: Script start time (for relative timestamp column).
    """
    print(f"\n  ▶  Moving to {target_degrees:+.1f}°  —  {label}")

    # Command the move with controlled speed and gentle acceleration
    motor.set_position_velocity_accel(
        position_degrees=target_degrees,
        velocity_erpm=MOVE_SPEED_ERPM,
        accel_erpm_per_sec=ACCEL_ERPM_PER_SEC,
    )

    # Sample feedback for HOLD_TIME seconds after commanding the move.
    # The refresh thread (started by set_position_velocity_accel) owns bus.recv()
    # and keeps motor._last_feedback up to date at ~50 Hz.  We read that shared
    # field here at SAMPLE_HZ — no bus.recv() competition.
    last_logged_ts: float = -1.0
    deadline = time.monotonic() + HOLD_TIME
    while time.monotonic() < deadline:
        time.sleep(SAMPLE_INTERVAL)
        elapsed = time.monotonic() - t0
        fb = motor._last_feedback

        if fb is not None and elapsed != last_logged_ts:
            last_logged_ts = elapsed
            writer.writerow(
                {
                    "time_s":        f"{elapsed:.3f}",
                    "commanded_deg": f"{target_degrees:.1f}",
                    "actual_deg":    f"{fb.position_degrees:.2f}",
                    "speed_erpm":    fb.speed_erpm,
                    "current_amps":  f"{fb.current_amps:.3f}",
                    "temp_c":        fb.temperature_celsius,
                    "error_code":    fb.error_code,
                    "label":         label,
                }
            )
            print(
                f"    t={elapsed:6.2f}s  "
                f"cmd={target_degrees:+6.1f}°  "
                f"pos={fb.position_degrees:+7.2f}°  "
                f"spd={fb.speed_erpm:+7d} ERPM  "
                f"cur={fb.current_amps:+5.2f} A",
                end="\r",
            )

    print()  # newline after the \r progress line


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    """Run the full motion capture validation sequence."""
    # Create output file
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_path = LOG_DIR / f"mocap_{timestamp}.csv"

    fieldnames = [
        "time_s",
        "commanded_deg",
        "actual_deg",
        "speed_erpm",
        "current_amps",
        "temp_c",
        "error_code",
        "label",
    ]

    print("=" * 60)
    print("  Motion Capture Validation — Arm Sweep")
    print("=" * 60)
    print(f"  Log file : {csv_path}")
    print(f"  Speed    : {MOVE_SPEED_ERPM} ERPM")
    print(f"  Accel    : {ACCEL_ERPM_PER_SEC} ERPM/s")
    print(f"  Hold     : {HOLD_TIME} s per waypoint")
    print(f"  Sample   : {SAMPLE_HZ} Hz")
    print("=" * 60)

    with open(csv_path, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        with CubeMarsAK606v3CAN() as motor:
            if not motor.connected:
                print(
                    "ERROR: CAN bus not available.\n"
                    "  Run: sudo ip link set can0 up type can bitrate 1000000"
                )
                return

            print("\n  Enabling motor...")
            motor.enable_motor()
            time.sleep(0.3)

            if not motor.check_communication():
                print(
                    "ERROR: Motor not responding.\n"
                    "  Check power, CANH/CANL wiring, and CAN ID.\n"
                    "  Make sure the UART cable is disconnected."
                )
                return

            # Zero the encoder at the current physical position (arm down = 0°)
            print("  Setting current position as home (0°)...")
            motor.set_origin(permanent=False)
            time.sleep(0.3)

            # ── Verify the motor actually accepted the set_origin command ──
            # If the UART cable is still connected, the motor ignores all CAN
            # control commands but continues to broadcast 50 Hz feedback.
            # After a successful set_origin the motor should report ~0°.
            origin_check = motor._receive_feedback(timeout=0.5)
            if origin_check is None:
                print(
                    "ERROR: No feedback after set_origin.\n"
                    "  Run: sudo ./setup_can.sh  (resets CAN bus error state)"
                )
                return
            if abs(origin_check.position_degrees) > 5.0:
                print(
                    f"ERROR: Position is {origin_check.position_degrees:.1f}° after set_origin"
                    " — expected ~0°.\n"
                    "  The motor is broadcasting feedback but ignoring control commands.\n"
                    "  Most likely cause: UART cable is still connected.\n"
                    "  Fix:\n"
                    "    1. Disconnect the UART cable from the motor.\n"
                    "    2. sudo ./setup_can.sh   (resets CAN bus state)\n"
                    "    3. Re-run this script."
                )
                return
            print(f"  Origin set — motor now at {origin_check.position_degrees:.2f}° ✓")

            print("\n  Starting sequence — press Ctrl+C to abort safely\n")
            t0 = time.monotonic()

            try:
                for target_deg, label in WAYPOINTS:
                    move_and_sample(motor, target_deg, label, writer, t0)

            except KeyboardInterrupt:
                print("\n\n  Aborted by user — returning to home (0°)...")
                motor.set_position_velocity_accel(
                    position_degrees=0.0,
                    velocity_erpm=MOVE_SPEED_ERPM,
                    accel_erpm_per_sec=ACCEL_ERPM_PER_SEC,
                )
                time.sleep(HOLD_TIME)

            finally:
                motor.stop()

    print("\n" + "=" * 60)
    print(f"  Done.  Data saved to:\n  {csv_path}")
    print("=" * 60)


if __name__ == "__main__":
    main()
