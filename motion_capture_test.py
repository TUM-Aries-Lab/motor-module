"""Motion capture validation script — arm sweep with IMU.

Moves an arm attached to the motor through a defined angle sequence while
logging motor feedback at 20 Hz to a CSV file.  The CSV can be compared
against the motion capture / IMU data to verify angle and velocity accuracy.

Uses DUTY CYCLE mode (arb_id = motor_id = 0x03) was tested first, but the
confirmed working mode is POSITION-VELOCITY LOOP (0x0603 for motor_id=0x03).
Position loop (0x0403) and position-velocity loop (0x0603) were previously
noted as not working, but that was before the UART cable was disconnected.
See docs/CAN_IMPLEMENTATION_JOURNEY.md for full details.

Leg sequence (motor zero = leg hanging straight down, right-side only):
  0°  → 45°   swing leg forward to mid-range
  45° → 65°   push to peak forward angle
  65° → 30°   partial return through mid
  30° →  0°   return to home

  The arm NEVER goes below 0° (left side).  If the PID overshoots below
  MIN_ANGLE a corrective duty is applied immediately to push it back.

Run:
    sudo ./setup_can.sh          # bring up can0 if not already up
    source .venv/bin/activate
    python motion_capture_test.py

Output CSV is saved to data/logs/mocap_<timestamp>.csv
"""

import csv
import time
from datetime import datetime
from pathlib import Path

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

try:
    import matplotlib
    matplotlib.use("Agg")          # headless — no display needed
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# ---------------------------------------------------------------------------
# CAN / motor constants
# ---------------------------------------------------------------------------

MOTOR_ID    = 0x03   # CAN ID of the motor
INTERFACE   = "can0"

# ---------------------------------------------------------------------------
# Motion parameters — adjust to taste
# ---------------------------------------------------------------------------

HOLD_TIME   = 15.0       # max seconds per waypoint before moving on
SAMPLE_HZ   = 20         # CSV log rate
LOG_DIR     = Path(__file__).parent / "data" / "logs"

# Motor direction: +1 if positive duty → positive angle, -1 if reversed.
# Set to -1 when physical wiring/gearbox causes positive duty to drive
# the leg toward negative firmware angles (observed empirically).
MOTOR_DIRECTION = -1

# ---------------------------------------------------------------------------
# Position-Velocity loop parameters
# ---------------------------------------------------------------------------
# Uses CAN mode 0x0603 (position-velocity loop): one frame sets target angle,
# max speed, and acceleration.  The motor's own trapezoidal position controller
# handles everything — no software PID needed.
#
# Firmware target = initial_pos + (physical_target × MOTOR_DIRECTION)
# Physical frame : 0° = start, positive = forward (always, regardless of wiring)

MOVE_SPEED_ERPM = 5000   # max travel speed (ERPM) — motor decelerates into target
MOVE_ACCEL      = 2000   # acceleration profile (ERPM/s) — 0 = firmware default
TOLERANCE  = 3.0          # degrees — considered arrived within this band
HOLD_AFTER = 2.0          # seconds to confirm settled before next waypoint
LOOP_HZ    = 10           # position-poll rate (motor handles motion internally)

# Safety clamp — single-turn mode, half-rotation limit
# Firmware setup (in CubeMars software):
#   - Single rotation mode enabled (encoder range 0°..360°)
#   - Arm hanging straight down = 360° (home position)
#   - Forward swing decreases firmware angle: 360° → 315° → 180°
#   - Hard stop at 180° firmware = 180° physical (arm fully forward)
#   - NEVER drive below 180° firmware — that crosses to the wrong side
# Physical frame (what PID sees): 0° = home, +° = forward, max = 180°
MIN_ANGLE  = 0.0      # degrees — never go behind home position
MAX_ANGLE  = 180.0    # degrees — hard limit: half rotation forward only

# ---------------------------------------------------------------------------
# Waypoint sequence  (PHYSICAL degrees from startup position)
# ---------------------------------------------------------------------------
# Physical frame: 0° = arm at bottom (firmware ≈360°), positive = forward swing.
# Firmware counts DOWN as arm swings forward (MOTOR_DIRECTION = -1).
# All waypoints must be within [0° .. 180°] physical (half-rotation limit).

BASE_WAYPOINTS = [
    (  0.0, "home — arm hanging straight down (360° firmware)"),
    ( 45.0, "mid-swing forward (315° firmware)"),
    ( 65.0, "peak forward swing (295° firmware)"),
    ( 30.0, "partial return through mid (330° firmware)"),
    (  0.0, "return home (360° firmware)"),
]




# ---------------------------------------------------------------------------
# Core move-and-sample loop
# ---------------------------------------------------------------------------

def move_and_sample(
    motor: CubeMarsAK606v3CAN,
    target_degrees: float,
    label: str,
    writer: csv.DictWriter,
    t0: float,
    initial_pos: float,
    *,
    phys_min: float = -5.0,
    phys_max: float = MAX_ANGLE,
) -> None:
    """Move to target_degrees (PHYSICAL degrees from startup) via position-velocity loop.

    Converts physical target → firmware degrees, sends one
    set_position_velocity_accel() command, then polls feedback until the
    motor settles within TOLERANCE for HOLD_AFTER seconds.  The motor's own
    trapezoidal controller handles speed and deceleration — no software PID.
    """
    # Convert physical target → absolute firmware degrees.
    # phys = (firmware - initial_pos) * MOTOR_DIRECTION  →  firmware = initial_pos + phys * MOTOR_DIRECTION
    # Normalise into [0°, 360°) — single-turn mode firmware rejects negative values.
    firmware_target = (initial_pos + target_degrees * MOTOR_DIRECTION) % 360.0
    print(f"\n  ▶  Moving to {target_degrees:+.1f}° physical  "
          f"(firmware = {firmware_target:.1f}°)  —  {label}")

    # Safety: reject targets outside physical bounds before sending anything.
    if target_degrees < phys_min or target_degrees > phys_max:
        print(f"  ⚠  Target {target_degrees:.1f}° outside bounds "
              f"[{phys_min:.0f}°..{phys_max:.0f}°] — skipping")
        return

    # Issue the command once — the refresh thread repeats it at 50 Hz.
    motor.set_position_velocity_accel(
        position_degrees=firmware_target,
        velocity_erpm=MOVE_SPEED_ERPM,
        accel_erpm_per_sec=MOVE_ACCEL,
    )

    last_csv_t = -1.0
    csv_interval = 1.0 / SAMPLE_HZ
    settled_at = None
    deadline   = time.monotonic() + HOLD_TIME
    loop_dt    = 1.0 / LOOP_HZ
    prev_phys  = None
    prev_time  = None

    while True:
        tick_start = time.monotonic()

        fb = motor._last_feedback
        if fb is None:
            fb = motor.get_status()
            if fb is None:
                print("  ⚠  feedback lost", end="\r")
                time.sleep(loop_dt)
                continue

        firmware_pos = fb.position_degrees
        spd          = fb.speed_erpm
        cur          = fb.current_amps
        tmp          = fb.temperature_celsius
        err_code     = fb.error_code
        now          = time.monotonic()

        phys_pos = (firmware_pos - initial_pos) * MOTOR_DIRECTION
        error    = target_degrees - phys_pos

        d_error = 0.0
        if prev_phys is not None and prev_time is not None:
            dt = now - prev_time
            if dt > 0.001:
                d_error = (phys_pos - prev_phys) / dt
        prev_phys = phys_pos
        prev_time = now

        if abs(error) < TOLERANCE:
            if settled_at is None:
                settled_at = now
        else:
            settled_at = None

        elapsed = now - t0
        if elapsed - last_csv_t >= csv_interval:
            last_csv_t = elapsed
            writer.writerow({
                "time_s":         f"{elapsed:.3f}",
                "commanded_deg":  f"{target_degrees:.1f}",
                "actual_deg":     f"{phys_pos:.2f}",
                "velocity_deg_s": f"{d_error:.2f}",
                "speed_erpm":     spd,
                "current_amps":   f"{cur:.3f}",
                "temp_c":         tmp,
                "error_code":     err_code,
                "duty":           f"{firmware_target:.1f}",
                "loop":           label.split("]")[0].lstrip("[") if "]" in label else "",
                "label":          label,
            })
        print(
            f"    t={elapsed:6.2f}s  "
            f"cmd={target_degrees:+6.1f}°  "
            f"phys={phys_pos:+7.2f}°  "
            f"err={error:+6.1f}°  "
            f"erpm={spd:+6.0f}",
            end="\r",
        )

        if settled_at is not None and (now - settled_at) >= HOLD_AFTER:
            break
        if now >= deadline:
            print(f"\n  ⚠  Waypoint timeout after {HOLD_TIME:.0f}s", end="")
            break

        elapsed_tick = time.monotonic() - tick_start
        if elapsed_tick < loop_dt:
            time.sleep(loop_dt - elapsed_tick)

    print()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_path = LOG_DIR / f"mocap_{timestamp}.csv"

    fieldnames = ["time_s", "commanded_deg", "actual_deg", "velocity_deg_s",
                  "speed_erpm", "current_amps", "temp_c", "error_code",
                  "duty", "loop", "label"]

    print("=" * 60)
    print("  Motion Capture Validation — Arm Sweep  (position-velocity loop)")
    print("=" * 60)
    print(f"  Log file      : {csv_path}")
    print(f"  Speed         : {MOVE_SPEED_ERPM} ERPM  |  Accel: {MOVE_ACCEL} ERPM/s")
    print(f"  Tolerance     : {TOLERANCE}°  |  Hold: {HOLD_AFTER}s  |  Timeout: {HOLD_TIME}s")
    print(f"  Poll rate     : {LOOP_HZ} Hz  |  Sample: {SAMPLE_HZ} Hz")
    print("=" * 60)

    with CubeMarsAK606v3CAN(motor_can_id=MOTOR_ID, interface=INTERFACE) as motor:
        with open(csv_path, "w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

            # ── Enable motor and wait for first feedback ────────────────
            # check_communication() has a tight 50ms capture window that can
            # race against the motor's 20ms broadcast period.  Instead, send
            # enable and then poll get_status() — which uses a 500ms timeout —
            # until we get a valid response (up to ~5 s total).
            print("\n  Enabling motor...")
            motor.enable_motor()
            time.sleep(0.3)  # let motor enter servo mode

            fb = None
            for attempt in range(10):
                fb = motor.get_status()
                if fb is not None:
                    break
                print(f"  Waiting for motor feedback (attempt {attempt + 1}/10)...", end="\r")
                motor.enable_motor()   # re-send enable in case motor missed first
                time.sleep(0.5)

            if fb is None:
                print("\nERROR: Motor not responding after 10 attempts.  "
                      "Check wiring, power, and CAN ID.")
                return
            initial_pos = fb.position_degrees
            print(f"  Motor alive — firmware pos={initial_pos:.1f}°  "
                  f"temp={fb.temperature_celsius}°C  cur={fb.current_amps:.2f}A ✓")
            print(f"  Physical frame: 0° = current position, +° = forward swing")
            print(f"  Safety bounds : {-5.0:.0f}° .. {MAX_ANGLE:.0f}° (physical)")

            # Waypoints are already physical offsets — no translation needed.
            # Physical: 0=start, positive=forward regardless of MOTOR_DIRECTION.
            # MOTOR_DIRECTION flips the ERPM sign inside move_and_sample.
            print(f"  Waypoints (physical): " +
                  ", ".join(f"{deg:+.1f}°" for deg, _ in BASE_WAYPOINTS))

            # ── Waypoint sequence ───────────────────────────────────────
            NUM_LOOPS = 1
            print(f"\n  Starting sequence ({NUM_LOOPS} loops) — press Ctrl+C to abort safely\n")
            t0 = time.monotonic()

            try:
                for loop_num in range(1, NUM_LOOPS + 1):
                    print(f"\n  ━━━ Loop {loop_num}/{NUM_LOOPS} ━━━")
                    for target_deg, label in BASE_WAYPOINTS:
                        move_and_sample(motor, target_deg,
                                        f"[{loop_num}/{NUM_LOOPS}] {label}",
                                        writer, t0, initial_pos)

                # ── Return home (physical 0° = startup position) ────────
                print("\n  All loops complete — returning to physical 0° (startup)...")
                move_and_sample(motor, 0.0, "final-return-home",
                                writer, t0, initial_pos)

            except KeyboardInterrupt:
                print("\n\n  Aborted — returning to physical 0° (startup)...")
                move_and_sample(motor, 0.0, "abort-return-home",
                                writer, t0, initial_pos)

            finally:
                motor.stop()
                time.sleep(0.05)
                motor.disable_motor()

    # ── Post-run analysis ────────────────────────────────────────
    print_summary(csv_path)
    if HAS_MPL:
        plot_path = csv_path.with_suffix(".png")
        generate_plots(csv_path, plot_path)
    else:
        print("  (matplotlib not installed — skipping plot generation)")

    print("\n" + "=" * 60)
    print(f"  Done.  Data saved to:\n  {csv_path}")
    if HAS_MPL:
        print(f"  Plot saved to:\n  {plot_path}")
    print("=" * 60)


# ---------------------------------------------------------------------------
# Post-run summary
# ---------------------------------------------------------------------------

def print_summary(csv_path: Path) -> None:
    """Read the CSV and print per-waypoint statistics."""
    rows = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            rows.append(row)
    if not rows:
        print("  No data recorded.")
        return

    # Group by (commanded_deg, loop) pairs to get per-waypoint stats
    from collections import OrderedDict
    waypoints: dict[str, list] = OrderedDict()
    for r in rows:
        key = f"{r['commanded_deg']}°"
        waypoints.setdefault(key, []).append(r)

    print("\n" + "=" * 72)
    print("  POST-RUN SUMMARY")
    print("=" * 72)
    print(f"  {'Waypoint':>10s}  {'Samples':>7s}  {'Mean Err':>8s}  "
          f"{'Max Err':>7s}  {'Mean Vel':>8s}  {'Peak Vel':>8s}")
    print("  " + "-" * 68)

    total_samples = 0
    all_errors = []
    all_velocities = []

    for wp, wp_rows in waypoints.items():
        errors = [abs(float(r["commanded_deg"]) - float(r["actual_deg"]))
                  for r in wp_rows]
        velocities = [abs(float(r["velocity_deg_s"])) for r in wp_rows
                      if r.get("velocity_deg_s")]
        n = len(wp_rows)
        mean_err = sum(errors) / n if n else 0
        max_err  = max(errors) if errors else 0
        mean_vel = sum(velocities) / len(velocities) if velocities else 0
        peak_vel = max(velocities) if velocities else 0

        print(f"  {wp:>10s}  {n:>7d}  {mean_err:>7.2f}°  "
              f"{max_err:>6.2f}°  {mean_vel:>7.1f}°/s  {peak_vel:>7.1f}°/s")

        total_samples += n
        all_errors.extend(errors)
        all_velocities.extend(velocities)

    print("  " + "-" * 68)
    overall_mean_err = sum(all_errors) / len(all_errors) if all_errors else 0
    overall_max_err  = max(all_errors) if all_errors else 0
    overall_peak_vel = max(all_velocities) if all_velocities else 0
    duration = float(rows[-1]["time_s"]) - float(rows[0]["time_s"])
    print(f"  {'TOTAL':>10s}  {total_samples:>7d}  {overall_mean_err:>7.2f}°  "
          f"{overall_max_err:>6.2f}°  {'':>8s}  {overall_peak_vel:>7.1f}°/s")
    print(f"  Duration: {duration:.1f}s  |  Samples: {total_samples}")
    print("=" * 72)


# ---------------------------------------------------------------------------
# Plot generation
# ---------------------------------------------------------------------------

def generate_plots(csv_path: Path, plot_path: Path) -> None:
    """Create a 2-panel plot: angle tracking + velocity over time."""
    times, cmd, actual, velocity = [], [], [], []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            times.append(float(row["time_s"]))
            cmd.append(float(row["commanded_deg"]))
            actual.append(float(row["actual_deg"]))
            velocity.append(float(row["velocity_deg_s"]))

    if not times:
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("Motion Capture Validation — Motor Arm Sweep", fontsize=14)

    # ── Panel 1: Angle ──────────────────────────────────────────────────
    ax1.plot(times, cmd, "--", color="#888888", linewidth=1.2, label="Commanded")
    ax1.plot(times, actual, "-", color="#2196F3", linewidth=1.0, label="Actual (motor)")
    ax1.set_ylabel("Angle (°)")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)
    ax1.set_title("Angle Tracking")

    # ── Panel 2: Velocity ───────────────────────────────────────────────
    ax2.plot(times, velocity, "-", color="#FF5722", linewidth=0.8, label="Motor velocity")
    ax2.axhline(0, color="#888888", linewidth=0.5)
    ax2.set_ylabel("Velocity (°/s)")
    ax2.set_xlabel("Time (s)")
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)
    ax2.set_title("Angular Velocity")

    plt.tight_layout()
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    print(f"\n  Plot saved → {plot_path}")


if __name__ == "__main__":
    main()
