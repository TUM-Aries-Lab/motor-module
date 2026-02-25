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
_ID_POS_VEL_ACCEL = (0x06 << 8) | MOTOR_ID   # 0x0603
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
    bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=True))


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


def _make_pos_cmd(position_deg: float, velocity_erpm: int, accel_erpm_per_sec: int) -> bytes:
    """Pack a Position+Velocity+Accel command (mode 0x06) payload."""
    pos_int  = int(max(-32_000_000, min(32_000_000, position_deg * 10_000)))
    vel_int  = int(max(-32767, min(32767, velocity_erpm // 10)))
    acc_int  = int(max(-32767, min(32767, accel_erpm_per_sec // 10)))
    return struct.pack(">ihh", pos_int, vel_int, acc_int)


def move_and_sample(
    bus: can.BusABC,
    target_degrees: float,
    label: str,
    writer: csv.DictWriter,
    t0: float,
) -> None:
    """Send position command at 50 Hz for HOLD_TIME seconds and log feedback."""
    print(f"\n  ▶  Moving to {target_degrees:+.1f}°  —  {label}")
    cmd = _make_pos_cmd(target_degrees, MOVE_SPEED_ERPM, ACCEL_ERPM_PER_SEC)

    last_csv_t = -1.0
    csv_interval = 1.0 / SAMPLE_HZ
    deadline = time.monotonic() + HOLD_TIME

    while time.monotonic() < deadline:
        _tx(bus, _ID_POS_VEL_ACCEL, cmd)
        fb = _get_feedback(bus)
        elapsed = time.monotonic() - t0

        if fb is not None:
            pos, spd, cur, tmp, err = fb
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
                f"    t={elapsed:6.2f}s  "
                f"cmd={target_degrees:+6.1f}°  "
                f"pos={pos:+7.2f}°  "
                f"spd={spd:+7d} ERPM  "
                f"cur={cur:+5.2f} A",
                end="\r",
            )
        else:
            time.sleep(SEND_INTERVAL)

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
        bus = can.interface.Bus(channel=INTERFACE, interface="socketcan")
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
            _tx(bus, _ID_POS_VEL_ACCEL,
                _make_pos_cmd(0.0, MOVE_SPEED_ERPM, ACCEL_ERPM_PER_SEC))
            fb = _get_feedback(bus)
            if fb is not None:
                print(f"  Origin set — motor now reports {fb[0]:.2f}° ✓")
            else:
                print("  (origin set — no confirmation feedback)")

            # ── Sweep ───────────────────────────────────────────────────────
            print("\n  Starting sequence — press Ctrl+C to abort safely\n")
            t0 = time.monotonic()

            for target_deg, label in WAYPOINTS:
                move_and_sample(bus, target_deg, label, writer, t0)

        except KeyboardInterrupt:
            print("\n\n  Aborted — returning to home (0°)...")
            cmd = _make_pos_cmd(0.0, MOVE_SPEED_ERPM, ACCEL_ERPM_PER_SEC)
            deadline = time.monotonic() + HOLD_TIME
            while time.monotonic() < deadline:
                _tx(bus, _ID_POS_VEL_ACCEL, cmd)
                _get_feedback(bus)

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
