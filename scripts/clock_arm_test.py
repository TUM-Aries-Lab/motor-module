"""Clock-arm motor accuracy test.

Attaches to the motor via CAN and moves the arm through clock-like positions
using only the CubeMarsAK606v3CAN class methods — no raw CAN structs, no
manual PID.

Sequence
--------
  Phase 1 — 12 steps x 30°   (every hour on a clock face)
  Phase 2 —  6 steps x 60°   (every other hour)
  Phase 3 —  3 steps x 120°  (thirds of a full circle)

For each stop: PD control drives to the target, the arm is actively held there
for HOLD_TIME (PD loop stays running — no free coast), then position is recorded.
Every control loop iteration is printed to the terminal and saved to CSV.

Run
---
    source .venv/bin/activate
    sudo ./setup_can.sh
    python scripts/clock_arm_test.py
"""

import csv
import sys
import time
from datetime import datetime
from itertools import groupby
from pathlib import Path

from loguru import logger

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

# Suppress INFO/DEBUG logs from the motor class so terminal output stays clean.
# Change to "DEBUG" to see every CAN frame sent/received.
logger.remove()
logger.add(sys.stderr, level="WARNING")

# ─────────────────────── tunables ─────────────────────────────────────────────
# Duty-cycle PD controller — only duty-cycle mode reliably moves the motor on
# the current AK60-6 v3 firmware.  Tune KP / MAX_DUTY if the arm overshoots.
KP           = 0.005     # duty per degree of error
KD           = 0.000015  # duty per ERPM (damping term)
MAX_DUTY     = 0.30      # ±30% duty ceiling
DEADBAND     = 2.0       # degrees — no correction below this error
SETTLE_DEG   = 3.0       # degrees — arm considered at target
SETTLE_ERPM  = 200       # ERPM    — arm considered stopped
SETTLE_TIME  = 0.5       # seconds — must hold settled for this long
MOVE_TIMEOUT = 25.0      # seconds — abort step if no settle by then
HOLD_TIME    = 2.0       # seconds — actively hold at target (PD loop on)
LOOP_HZ      = 10        # control loop rate (Hz)
LOG_DIR      = Path(__file__).parent.parent / "data" / "logs"
# ──────────────────────────────────────────────────────────────────────────────

PHASES = [
    (12, 30,  "Phase 1 — 12 × 30°"),
    ( 6, 60,  "Phase 2 —  6 × 60°"),
    ( 3, 120, "Phase 3 —  3 × 120°"),
]

FIELDNAMES = [
    "time_s", "phase", "step", "state",
    "target_deg", "actual_deg", "error_deg",
    "duty", "speed_erpm", "current_amps",
]


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def enable_with_retry(motor: CubeMarsAK606v3CAN, attempts: int = 6) -> bool:
    """Send enable and retry until the motor ACKs, or give up after *attempts*."""
    for _ in range(attempts):
        motor.enable_motor()
        if motor._last_feedback is not None:
            return True
        time.sleep(0.5)
    return False


def pd_duty(error: float, speed: int) -> float:
    """Compute PD duty from position error (degrees) and motor speed (ERPM)."""
    if abs(error) < DEADBAND:
        return 0.0
    duty = KP * error - KD * speed
    return max(-MAX_DUTY, min(MAX_DUTY, duty))


def move_to(
    motor: CubeMarsAK606v3CAN,
    target_deg: float,
    label: str,
    step: int | str,
    writer: csv.DictWriter,
    t0: float,
) -> tuple[float | None, bool]:
    """PD control to target_deg, then actively hold for HOLD_TIME.

    The PD loop runs continuously — no duty=0 sleep after settling.  This
    prevents the arm from drifting (or completing a full extra revolution)
    during the hold period.  Every iteration is logged to CSV and printed.

    Returns (final_position_degrees, settled_ok).
    """
    loop_dt    = 1.0 / LOOP_HZ
    deadline   = time.monotonic() + MOVE_TIMEOUT
    settled_at: float | None = None
    hold_end:   float | None = None

    while time.monotonic() < deadline:
        tick = time.monotonic()

        # Read all feedback from the latest CAN frame in one shot.
        fb = motor._last_feedback
        if fb is None:
            motor.set_duty_cycle(0.0)
            time.sleep(loop_dt)
            continue

        pos     = fb.position_degrees
        speed   = fb.speed_erpm
        current = fb.current_amps
        elapsed = tick - t0
        error   = target_deg - pos
        state   = "HOLD" if hold_end is not None else "MOVE"

        # PD duty — applied during both MOVE and HOLD so arm is never free-coasting.
        duty = pd_duty(error, speed)
        motor.set_duty_cycle(duty)

        # ── CSV ──────────────────────────────────────────────────────────
        writer.writerow({
            "time_s":      f"{elapsed:.3f}",
            "phase":       label,
            "step":        step,
            "state":       state,
            "target_deg":  f"{target_deg:.1f}",
            "actual_deg":  f"{pos:.2f}",
            "error_deg":   f"{error:.2f}",
            "duty":        f"{duty:.5f}",
            "speed_erpm":  speed,
            "current_amps": f"{current:.3f}",
        })

        # ── Terminal ─────────────────────────────────────────────────────
        print(
            f"    [{state}]  pos={pos:+7.2f}°  target={target_deg:+7.1f}°  "
            f"err={error:+6.1f}°  duty={duty:+.4f}  erpm={speed:+6d}  cur={current:+.2f}A",
            end="\r",
            flush=True,
        )

        # ── State machine ────────────────────────────────────────────────
        if hold_end is not None:
            # HOLD phase: keep PD running; return when hold_end passes.
            if tick >= hold_end:
                motor.set_duty_cycle(0.0)
                print()  # newline after \r
                return pos, True
        # MOVE phase: check for settle.
        elif abs(error) < SETTLE_DEG and abs(speed) < SETTLE_ERPM:
            if settled_at is None:
                settled_at = tick
            elif tick - settled_at >= SETTLE_TIME:
                hold_end = tick + HOLD_TIME
                print(f"\n    ✓  Settled at {pos:+.2f}°  (err={error:+.2f}°)  — holding {HOLD_TIME}s")
        else:
            settled_at = None

        time.sleep(max(loop_dt - (time.monotonic() - tick), 0.005))

    # Timeout — coast and return last known position.
    motor.set_duty_cycle(0.0)
    print()
    return pos if fb is not None else motor.get_position(), False


def run_phase(
    motor: CubeMarsAK606v3CAN,
    steps: int,
    step_deg: float,
    label: str,
    writer: csv.DictWriter,
    t0: float,
) -> list[dict]:
    """Move the arm through *steps* equal clock positions and record accuracy."""
    print(f"\n{'=' * 60}")
    print(f"  {label}   ({steps} stops × {step_deg}°)")
    print(f"{'=' * 60}")

    results = []
    for i in range(1, steps + 1):
        target = i * step_deg
        print(f"\n  [{i:2d}/{steps}]  →  {target:6.1f}°", flush=True)

        actual, settled = move_to(motor, target, label, i, writer, t0)

        if actual is not None:
            error = actual - target
            flag  = "✓" if settled else "TIMEOUT"
            print(f"\n  actual = {actual:+7.2f}°   err = {error:+6.2f}°   [{flag}]")
        else:
            error = None
            print("\n  (no feedback)")

        results.append({
            "phase":      label,
            "step":       i,
            "target_deg": target,
            "actual_deg": actual,
            "error_deg":  error,
            "settled":    settled,
        })

    # Return to origin after each phase.
    print("\n  Returning to 0°...", flush=True)
    move_to(motor, 0.0, f"{label}:return", "ret", writer, t0)
    pos_after = motor.get_position()
    if pos_after is not None:
        print(f"\n  Back at origin  →  {pos_after:+.2f}°")

    return results


def print_summary(all_results: list[dict], csv_path: Path) -> None:
    print(f"\n{'=' * 60}")
    print("  ACCURACY SUMMARY")
    print(f"{'=' * 60}")
    print(f"  {'Phase':<28s}  {'N':>3}  {'Mean |err|':>10}  {'Max |err|':>9}")
    print(f"  {'-' * 53}")
    for phase_label, rows in groupby(all_results, key=lambda r: r["phase"]):
        rows = list(rows)
        errors = [abs(r["error_deg"]) for r in rows if r["error_deg"] is not None]
        if not errors:
            print(f"  {phase_label:<28s}   —  no data")
            continue
        mean_err = sum(errors) / len(errors)
        max_err  = max(errors)
        print(f"  {phase_label:<28s}  {len(rows):>3}  {mean_err:>9.2f}°  {max_err:>8.2f}°")
    print(f"{'=' * 60}")
    print(f"  CSV → {csv_path}")


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main() -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts       = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_path = LOG_DIR / f"clock_arm_{ts}.csv"

    print("=" * 60)
    print("  Clock-Arm Motor Accuracy Test")
    print(f"  {datetime.now().strftime('%Y-%m-%d  %H:%M:%S')}")
    print(f"  KP={KP}  KD={KD}  MAX_DUTY={MAX_DUTY}  DEADBAND={DEADBAND}°")
    print(f"  Settle: ±{SETTLE_DEG}°  <{SETTLE_ERPM} ERPM  for {SETTLE_TIME}s  |  Hold: {HOLD_TIME}s")
    print(f"  Log → {csv_path.name}")
    print("=" * 60)

    with open(csv_path, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=FIELDNAMES)
        writer.writeheader()

        with CubeMarsAK606v3CAN() as motor:
            if not motor.connected:
                print("\n  ✗  CAN bus not available — run: sudo ./setup_can.sh")
                return

            # ── Enable ────────────────────────────────────────────────────────
            print("\n  Enabling motor...")
            if not enable_with_retry(motor):
                print("  ✗  No motor feedback — check power & wiring")
                return

            fb = motor.get_status()
            if fb is not None:
                print(
                    f"  Motor alive  "
                    f"pos={fb.position_degrees:.1f}°  "
                    f"temp={fb.temperature_celsius}°C  "
                    f"err={fb.error_code} ({fb.error_description})"
                )
            else:
                print("  Motor alive (no telemetry yet)")

            # ── Zero the origin ───────────────────────────────────────────────
            print("\n  Setting origin (current position = 0°)...")
            motor.set_origin()
            # Let firmware process the origin command, then send a zero-duty frame
            # to get a fresh feedback frame (set_origin has no ACK on this unit).
            time.sleep(0.5)
            motor.set_duty_cycle(0.0)
            time.sleep(1.0)
            pos = motor.get_position()
            print(f"  Origin set — position now reads: {pos:.2f}°" if pos is not None else "  Origin set")

            print("\n  Press Ctrl+C at any time to stop safely.\n")

            # ── Run phases ────────────────────────────────────────────────────
            all_results: list[dict] = []
            t0 = time.monotonic()
            try:
                for steps, step_deg, label in PHASES:
                    results = run_phase(motor, steps, step_deg, label, writer, t0)
                    all_results.extend(results)

            except KeyboardInterrupt:
                print("\n\n  Ctrl+C — stopping safely...")

            finally:
                motor.stop()

            # ── Summary ───────────────────────────────────────────────────────
            if all_results:
                print_summary(all_results, csv_path)

    print(f"\n  CSV saved → {csv_path}")
    print("\n  Done.\n")


if __name__ == "__main__":
    main()
