"""Motion capture validation script — arm sweep with IMU.

Moves an arm attached to the motor through a defined angle sequence while
logging motor feedback at 20 Hz to a CSV file.  The CSV can be compared
against the motion capture / IMU data to verify angle and velocity accuracy.

Uses DUTY CYCLE mode (arb_id = motor_id = 0x03), which is the only control
mode confirmed working on this CubeMars AK60-6 firmware.  Velocity mode
(0x0303), position mode (0x0403), and PVA mode (0x0603) all ACK but do NOT
produce shaft rotation.  See docs/CAN_TROUBLESHOOTING.md for full details.

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
import time
from datetime import datetime
from pathlib import Path

import can

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

MOTOR_ID    = 0x03            # CAN ID of the motor (also the duty-mode arb_id)
INTERFACE   = "can0"
FEEDBACK_ID = 0x2903          # motor always replies with this extended ID

ENABLE  = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
DISABLE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])

# ---------------------------------------------------------------------------
# Motion parameters — adjust to taste
# ---------------------------------------------------------------------------

MAX_DUTY    = 0.15       # max duty cycle fraction (0..1) — limits peak voltage
HOLD_TIME   = 10.0       # max seconds per waypoint before moving on
SAMPLE_HZ   = 20         # CSV log rate
LOG_DIR     = Path(__file__).parent / "data" / "logs"

# PID-controller gains  (duty cycle per degree of error / per deg/s)
KP         = 0.004    # duty/deg — 0.004 × 40° error = 0.16 duty = 16%
KD         = 0.0003   # duty/(deg/s) — damps oscillation
KI         = 0.0008   # duty/(deg·s) — eliminates steady-state friction offset
KI_MAX     = 0.08     # max integral contribution (prevents windup)
MIN_DUTY   = 0.015    # minimum duty to apply when error > TOLERANCE (overcomes static friction)
TOLERANCE  = 2.0      # degrees — zero output once within this band
HOLD_AFTER = 1.5      # seconds to hold at target before next waypoint
LOOP_HZ    = 50       # control loop rate (send one command per tick)

# Safety clamp — arm must stay on the RIGHT side of the rig
MIN_ANGLE  = 0.0      # degrees — never drive the arm below this (prevents crossing to left side)
MAX_ANGLE  = 130.0    # degrees — hard limit above horizontal

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
# Low-level helpers  (same pattern as spin_test.py)
# ---------------------------------------------------------------------------


def duty_cmd(duty_frac: float) -> bytes:
    """Pack a duty cycle command: duty fraction × 100 000 → int32 big-endian."""
    return struct.pack(">i", int(duty_frac * 100_000)) + bytes(4)


def tx(bus: can.BusABC, arb_id: int, data: bytes) -> None:
    """Send an extended-ID CAN frame, retrying on TX buffer full."""
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=True)
    for attempt in range(3):
        try:
            bus.send(msg)
            return
        except can.CanOperationError:
            time.sleep(0.02 * (attempt + 1))


def get_feedback(bus: can.BusABC, timeout: float = 0.05):
    """Read one frame → (pos_deg, speed_erpm, cur_a, temp_c, err) or None."""
    msg = bus.recv(timeout=timeout)
    if msg and msg.arbitration_id == FEEDBACK_ID and len(msg.data) == 8:
        d = msg.data
        pos = struct.unpack(">h", d[0:2])[0] * 0.1
        spd = struct.unpack(">h", d[2:4])[0] * 10
        cur = struct.unpack(">h", d[4:6])[0] * 0.01
        tmp = struct.unpack("b",  bytes([d[6]]))[0]
        err = d[7]
        return pos, spd, cur, tmp, err
    return None


def stop_motor(bus: can.BusABC) -> None:
    """Send zero-duty frames to coast to a stop."""
    for _ in range(5):
        tx(bus, MOTOR_ID, duty_cmd(0))
        time.sleep(0.02)


# ---------------------------------------------------------------------------
# Core move-and-sample loop
# ---------------------------------------------------------------------------

def move_and_sample(
    bus: can.BusABC,
    target_degrees: float,
    label: str,
    writer: csv.DictWriter,
    t0: float,
    home_offset: float = 0.0,
) -> None:
    """Drive to (target_degrees + home_offset) using duty-cycle PD-controller.

    home_offset is the motor's absolute position when the script started,
    so target_degrees is always relative to the physical home position.

    Exactly ONE tx + ONE recv per loop tick at LOOP_HZ.  No double-sends.
    """
    abs_target = target_degrees + home_offset
    print(f"\n  ▶  Moving to {target_degrees:+.1f}°  —  {label}")

    last_csv_t   = -1.0
    csv_interval = 1.0 / SAMPLE_HZ
    settled_at   = None
    deadline     = time.monotonic() + HOLD_TIME
    loop_dt      = 1.0 / LOOP_HZ

    prev_pos     = None
    prev_time    = None
    duty         = 0.0
    integral     = 0.0

    # Flush stale rx frames
    while bus.recv(timeout=0.0) is not None:
        pass

    while True:
        tick_start = time.monotonic()

        # ── Single send → single recv (spin_test pattern) ───────────────
        tx(bus, MOTOR_ID, duty_cmd(duty))
        fb = get_feedback(bus, timeout=0.05)

        if fb is not None:
            pos, spd, cur, tmp, err_code = fb
            now = time.monotonic()
            error = abs_target - pos

            # Derivative: degrees/second from successive feedback
            d_error = 0.0
            dt = 1.0 / LOOP_HZ
            if prev_pos is not None and prev_time is not None:
                dt = now - prev_time
                if dt > 0.001:
                    d_error = (pos - prev_pos) / dt   # positive = moving forward
            prev_pos  = pos
            prev_time = now

            # Integral: accumulates to push through static friction
            integral += error * dt
            integral = max(-KI_MAX / max(KI, 1e-9), min(KI_MAX / max(KI, 1e-9), integral))

            # PID output
            duty = KP * error + KI * integral - KD * d_error
            duty = max(-MAX_DUTY, min(MAX_DUTY, duty))

            # Ensure minimum duty when outside tolerance (overcome static friction)
            if abs(error) >= TOLERANCE and 0 < abs(duty) < MIN_DUTY:
                duty = MIN_DUTY if error > 0 else -MIN_DUTY

            # ── Safety clamp: keep arm on RIGHT side only ───────────
            rel_pos = pos - home_offset
            if rel_pos <= MIN_ANGLE and duty < 0:
                # Arm is at or past the left boundary — refuse to go further left
                duty = 0.0
            if rel_pos >= MAX_ANGLE and duty > 0:
                # Arm is at or past the right/upper boundary — refuse to go further
                duty = 0.0

            if abs(error) < TOLERANCE:
                duty = 0.0
                integral = 0.0   # reset windup when settled
                if settled_at is None:
                    settled_at = now
            else:
                settled_at = None

            elapsed = now - t0
            if elapsed - last_csv_t >= csv_interval:
                last_csv_t = elapsed
                writer.writerow({
                    "time_s":        f"{elapsed:.3f}",
                    "commanded_deg": f"{target_degrees:.1f}",
                    "actual_deg":    f"{pos - home_offset:.2f}",
                    "velocity_deg_s": f"{d_error:.2f}",
                    "speed_erpm":    spd,
                    "current_amps":  f"{cur:.3f}",
                    "temp_c":        tmp,
                    "error_code":    err_code,
                    "duty":          f"{duty:.4f}",
                    "loop":          label.split("]")[0].lstrip("[") if "]" in label else "",
                    "label":         label,
                })
            print(
                f"    t={elapsed:6.2f}s  "
                f"cmd={target_degrees:+6.1f}°  "
                f"pos={pos - home_offset:+7.2f}°  "
                f"err={error:+6.1f}°  "
                f"duty={duty:+.3f}",
                end="\r",
            )

        now = time.monotonic()
        if settled_at is not None and (now - settled_at) >= HOLD_AFTER:
            break
        if now >= deadline:
            print(f"\n  ⚠  Waypoint timeout after {HOLD_TIME:.0f}s", end="")
            break

        # Pace to LOOP_HZ — avoid flooding the bus
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
    print("  Motion Capture Validation — Arm Sweep  (duty-cycle mode)")
    print("=" * 60)
    print(f"  Log file  : {csv_path}")
    print(f"  Max duty  : {MAX_DUTY*100:.0f}%  |  KP={KP}  KD={KD}  KI={KI}")
    print(f"  Tolerance : {TOLERANCE}°  |  Hold: {HOLD_AFTER}s  |  Timeout: {HOLD_TIME}s")
    print(f"  Loop      : {LOOP_HZ} Hz  |  Sample: {SAMPLE_HZ} Hz")
    print("=" * 60)

    # CAN filter: only receive 0x2903 feedback — blocks the 0x0088 flood
    can_filters = [{"can_id": FEEDBACK_ID, "can_mask": 0x1FFFFFFF, "extended": True}]
    bus = can.interface.Bus(channel=INTERFACE, interface="socketcan",
                            can_filters=can_filters)

    try:
        with open(csv_path, "w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

            # ── Enable motor ────────────────────────────────────────────
            print("\n  Enabling motor...")
            tx(bus, MOTOR_ID, ENABLE)
            time.sleep(0.15)

            # Verify alive & capture home position
            tx(bus, MOTOR_ID, duty_cmd(0))
            fb = get_feedback(bus, timeout=0.5)
            if fb is None:
                print("ERROR: Motor not responding.  Check wiring and CAN ID.")
                return
            home_offset = fb[0]
            print(f"  Motor alive — abs_pos={home_offset:.1f}°  "
                  f"temp={fb[3]}°C  cur={fb[2]:.2f}A ✓")
            print(f"  Home offset: all waypoints relative to {home_offset:.1f}°")

            # ── Waypoint sequence ───────────────────────────────────────
            NUM_LOOPS = 5
            print(f"\n  Starting sequence ({NUM_LOOPS} loops) — press Ctrl+C to abort safely\n")
            t0 = time.monotonic()

            try:
                for loop_num in range(1, NUM_LOOPS + 1):
                    print(f"\n  ━━━ Loop {loop_num}/{NUM_LOOPS} ━━━")
                    for target_deg, label in WAYPOINTS:
                        move_and_sample(bus, target_deg, f"[{loop_num}/{NUM_LOOPS}] {label}",
                                        writer, t0, home_offset)

            except KeyboardInterrupt:
                print("\n\n  Aborted — returning to home (0°)...")
                move_and_sample(bus, 0.0, "abort-return-home", writer, t0, home_offset)

            finally:
                stop_motor(bus)
                time.sleep(0.05)
                tx(bus, MOTOR_ID, DISABLE)

    finally:
        bus.shutdown()

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
