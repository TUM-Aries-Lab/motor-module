"""Motion capture validation script — arm sweep with IMU.

Moves an arm attached to the motor through a defined angle sequence while
logging motor feedback at 20 Hz to a CSV file.  The CSV can be compared
against the motion capture / IMU data to verify angle and velocity accuracy.

Uses raw python-can (same approach as spin_test.py) — sends the command in
the main thread and reads feedback immediately after each send.

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

# ---------------------------------------------------------------------------
# Hardware config
# ---------------------------------------------------------------------------

MOTOR_ID  = 0x03
INTERFACE = "can0"

# CAN arbitration IDs (extended 29-bit): (mode << 8) | motor_id
_ID_ENABLE       = MOTOR_ID                    # 0x03  — enable/disable sent direct
_ID_SET_ORIGIN   = (0x05 << 8) | MOTOR_ID     # 0x0503
_ID_VELOCITY     = (0x03 << 8) | MOTOR_ID     # 0x0303  — velocity control (confirmed working)
_ID_FEEDBACK     = 0x2903                      # motor broadcast ID

ENABLE_DATA  = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
DISABLE_DATA = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
# Set-origin byte: 0x01 = temporary (until power cycle)
ORIGIN_DATA  = bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

# ---------------------------------------------------------------------------
# Motion parameters — adjust to taste
# ---------------------------------------------------------------------------

MOVE_SPEED_ERPM    = 5_000    # max travel speed (ERPM)
ACCEL_ERPM_PER_SEC = 3_000   # acceleration ramp (ERPM/s)
HOLD_TIME          = 2.0     # seconds to hold each waypoint
SEND_HZ            = 50      # command repeat rate
SEND_INTERVAL      = 1.0 / SEND_HZ
SAMPLE_HZ          = 20      # CSV log rate
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

def _tx(bus: can.BusABC, arb_id: int, data: bytes) -> None:
    try:
        bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=True))
    except can.CanOperationError:
        time.sleep(0.02)  # back off if TX buffer is full


def _get_feedback(bus: can.BusABC):
    """Read one frame; return (pos_deg, spd_erpm, cur_a, tmp_c, err) or None."""
    msg = bus.recv(timeout=0.02)
    if msg and msg.arbitration_id == _ID_FEEDBACK and len(msg.data) == 8:
        d = msg.data
        pos = struct.unpack(">h", d[0:2])[0] * 0.1
        spd = struct.unpack(">h", d[2:4])[0] * 10
        cur = struct.unpack(">h", d[4:6])[0] * 0.01
        tmp = struct.unpack("b", bytes([d[6]]))[0]
        err = d[7]
        return pos, spd, cur, tmp, err
    return None



def move_and_sample(
    bus: can.BusABC,
    target_degrees: float,
    label: str,
    writer: csv.DictWriter,
    t0: float,
) -> None:
    """Drive to target_degrees using velocity mode + proportional controller.

    Velocity mode is used because it is confirmed to work on this hardware.
    A proportional controller converts the angle error into an ERPM command.
    """
    print(f"\n  ▶  Moving to {target_degrees:+.1f}°  —  {label}")

    KP          = 150        # ERPM per degree of error
    TOLERANCE   = 1.5        # degrees — stop commanding once within this band
    HOLD_AFTER  = 1.5        # seconds to stay at target before next waypoint

    last_csv_t  = -1.0
    csv_interval = 1.0 / SAMPLE_HZ

    settled_at   = None      # time when we first entered the tolerance band
    deadline     = time.monotonic() + HOLD_TIME

    while True:
        now = time.monotonic()

        # Send velocity command proportional to error
        fb = _get_feedback(bus)
        if fb is not None:
            pos, spd, cur, tmp, err = fb
            error = target_degrees - pos
            vel   = int(max(-MOVE_SPEED_ERPM, min(MOVE_SPEED_ERPM, KP * error)))

            if abs(error) < TOLERANCE:
                vel = 0
                if settled_at is None:
                    settled_at = now

            vel_cmd = struct.pack(">i", vel) + bytes(4)
            _tx(bus, (0x03 << 8) | MOTOR_ID, vel_cmd)

            elapsed = now - t0
            if elapsed - last_csv_t >= csv_interval:
                last_csv_t = elapsed
                writer.writerow({
                    "time_s":        f"{elapsed:.3f}",
                    "commanded_deg": f"{target_degrees:.1f}",
                    "actual_deg":    f"{pos:.2f}",
                    "speed_erpm":    spd,
                    "current_amps":  f"{cur:.3f}",
                    "temp_c":        tmp,
                    "error_code":    err,
                    "label":         label,
                })
            print(
                f"    t={now-t0:6.2f}s  "
                f"cmd={target_degrees:+6.1f}°  "
                f"pos={pos:+7.2f}°  "
                f"err={error:+6.1f}°  "
                f"vel={vel:+6d}",
                end="\r",
            )
        else:
            _tx(bus, (0x03 << 8) | MOTOR_ID, struct.pack(">i", 0) + bytes(4))
            time.sleep(SEND_INTERVAL)
            continue

        # Exit: either settled long enough OR overall deadline reached
        if settled_at is not None and (now - settled_at) >= HOLD_AFTER:
            break
        if now >= deadline:
            break

        time.sleep(SEND_INTERVAL)

    # Hold position after reaching waypoint
    _tx(bus, (0x03 << 8) | MOTOR_ID, struct.pack(">i", 0) + bytes(4))
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
    print(f"  Speed    : {MOVE_SPEED_ERPM} ERPM")
    print(f"  Accel    : {ACCEL_ERPM_PER_SEC} ERPM/s")
    print(f"  Hold     : {HOLD_TIME} s per waypoint")
    print(f"  Sample   : {SAMPLE_HZ} Hz")
    print("=" * 60)

    try:
        bus = can.interface.Bus(
            channel=INTERFACE,
            interface="socketcan",
            # Only pass motor feedback frames — blocks the 0x0088 noise device
            # that floods the receive queue at ~30 kHz and swamps bus.recv().
            can_filters=[{"can_id": _ID_FEEDBACK, "can_mask": 0x1FFFFFFF, "extended": True}],
        )
    except Exception as e:
        print(f"ERROR: Cannot open {INTERFACE}: {e}\n  Run: sudo ./setup_can.sh")
        return

    with open(csv_path, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        try:
            # ── Enable ──────────────────────────────────────────────────────
            print("\n  Enabling motor...")
            _tx(bus, _ID_ENABLE, ENABLE_DATA)
            time.sleep(0.15)

            # Confirm motor is alive
            fb = _get_feedback(bus)
            if fb is None:
                print(
                    "ERROR: No feedback after enable.\n"
                    "  Check power, CANH/CANL wiring, CAN ID, and CAN bus state.\n"
                    "  Run: sudo ./setup_can.sh"
                )
                return
            print(f"  Motor alive — pos={fb[0]:.1f}°  temp={fb[3]}°C ✓")

            # ── Set origin ──────────────────────────────────────────────────
            print("  Setting current position as home (0°)...")
            _tx(bus, _ID_SET_ORIGIN, ORIGIN_DATA)
            time.sleep(0.3)   # let motor process; next feedback will show 0°

            # Read fresh position after origin is applied
            fb = _get_feedback(bus)
            if fb is not None:
                print(f"  Origin set — motor now reports {fb[0]:.2f}°")
            else:
                print("  (origin set — no confirmation feedback)")

            # ── Sweep ───────────────────────────────────────────────────────
            print("\n  Starting sequence — press Ctrl+C to abort safely\n")
            t0 = time.monotonic()

            for target_deg, label in WAYPOINTS:
                move_and_sample(bus, target_deg, label, writer, t0)

        except KeyboardInterrupt:
            print("\n\n  Aborted — returning to home (0°)...")
            move_and_sample(bus, 0.0, "abort-return-home", writer, t0)

        finally:
            # Zero current then disable
            _tx(bus, _ID_ENABLE, ENABLE_DATA)   # re-enable in case of error
            time.sleep(0.05)
            _tx(bus, _ID_ENABLE, DISABLE_DATA)
            bus.shutdown()

    print("\n" + "=" * 60)
    print(f"  Done.  Data saved to:\n  {csv_path}")
    print("=" * 60)


if __name__ == "__main__":
    main()
