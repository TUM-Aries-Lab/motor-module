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
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "src"))

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

# ---------------------------------------------------------------------------
# Motion parameters — adjust to taste
# ---------------------------------------------------------------------------

MOVE_SPEED_ERPM = 5_000    # max travel speed (ERPM)
HOLD_TIME       = 4.0      # max seconds per waypoint before moving on
SAMPLE_HZ       = 20       # CSV log rate
LOG_DIR = Path(__file__).parent / "data" / "logs"

# P-controller gains (velocity mode is the only confirmed-working mode)
KP        = 150    # ERPM per degree of error
TOLERANCE = 1.5    # degrees — zero velocity once within this band
HOLD_AFTER = 1.5   # seconds to hold at target before next waypoint

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
    """Drive to target_degrees using velocity + P-controller and log feedback.

    Position mode (0x04) and PVA mode (0x06) are not responding on this
    firmware — see docs/CAN_TROUBLESHOOTING.md.  Velocity mode (0x03) is
    confirmed working, so we close the loop in Python.
    """
    print(f"\n  ▶  Moving to {target_degrees:+.1f}°  —  {label}")

    last_csv_t = -1.0
    csv_interval = 1.0 / SAMPLE_HZ
    settled_at = None
    deadline = time.monotonic() + HOLD_TIME

    while True:
        now = time.monotonic()

        # _last_feedback is updated by the refresh thread at ~50 Hz.
        # Read it directly — no bus.recv() competition.
        fb = motor._last_feedback
        if fb is not None:
            pos   = fb.position_degrees
            error = target_degrees - pos
            vel   = int(max(-MOVE_SPEED_ERPM, min(MOVE_SPEED_ERPM, KP * error)))

            if abs(error) < TOLERANCE:
                vel = 0
                if settled_at is None:
                    settled_at = now
            else:
                settled_at = None  # reset if we drift out of band

            motor.set_velocity(vel, allow_low_speed=True)

            elapsed = now - t0
            if elapsed - last_csv_t >= csv_interval:
                last_csv_t = elapsed
                writer.writerow({
                    "time_s":        f"{elapsed:.3f}",
                    "commanded_deg": f"{target_degrees:.1f}",
                    "actual_deg":    f"{pos:.2f}",
                    "speed_erpm":    fb.speed_erpm,
                    "current_amps":  f"{fb.current_amps:.3f}",
                    "temp_c":        fb.temperature_celsius,
                    "error_code":    fb.error_code,
                    "label":         label,
                })
            print(
                f"    t={elapsed:6.2f}s  "
                f"cmd={target_degrees:+6.1f}°  "
                f"pos={pos:+7.2f}°  "
                f"err={error:+6.1f}°  "
                f"vel={vel:+6d}",
                end="\r",
            )
        else:
            time.sleep(0.02)
            continue

        if settled_at is not None and (now - settled_at) >= HOLD_AFTER:
            break
        if now >= deadline:
            break

        time.sleep(0.02)

    motor.set_velocity(0, allow_low_speed=True)
    print()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_path = LOG_DIR / f"mocap_{timestamp}.csv"

    fieldnames = ["time_s", "commanded_deg", "actual_deg",
                  "speed_erpm", "current_amps", "temp_c", "error_code", "label"]

    print("=" * 60)
    print("  Motion Capture Validation — Arm Sweep")
    print("=" * 60)
    print(f"  Log file : {csv_path}")
    print(f"  Speed    : {MOVE_SPEED_ERPM} ERPM  |  KP={KP}")
    print(f"  Tolerance: {TOLERANCE}°  |  Hold: {HOLD_TIME}s")
    print(f"  Sample   : {SAMPLE_HZ} Hz")
    print("=" * 60)

    with open(csv_path, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        with CubeMarsAK606v3CAN() as motor:
            if not motor.connected:
                print("ERROR: CAN bus not available.  Run: sudo ./setup_can.sh")
                return

            print("\n  Enabling motor...")
            motor.enable_motor()
            time.sleep(0.3)

            if not motor.check_communication():
                print("ERROR: Motor not responding.  Check wiring and CAN ID.")
                return

            print("  Setting current position as home (0°)...")
            motor.set_origin(permanent=False)
            time.sleep(0.3)

            print("\n  Starting sequence — press Ctrl+C to abort safely\n")
            t0 = time.monotonic()

            try:
                for target_deg, label in WAYPOINTS:
                    move_and_sample(motor, target_deg, label, writer, t0)

            except KeyboardInterrupt:
                print("\n\n  Aborted — returning to home (0°)...")
                move_and_sample(motor, 0.0, "abort-return-home", writer, t0)

            finally:
                motor.stop()

    print("\n" + "=" * 60)
    print(f"  Done.  Data saved to:\n  {csv_path}")
    print("=" * 60)


if __name__ == "__main__":
    main()
