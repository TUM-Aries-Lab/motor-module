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
import time
from datetime import datetime
from pathlib import Path

import can

# ---------------------------------------------------------------------------
# CAN / motor constants
# ---------------------------------------------------------------------------

MOTOR_ID    = 0x03
INTERFACE   = "can0"
FEEDBACK_ID = 0x2903          # motor always replies with this extended ID

ENABLE  = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
DISABLE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])

# CAN control-mode IDs  (upper byte of the 29-bit arbitration ID)
MODE_VELOCITY = 0x03          # velocity-loop command
MODE_CURRENT  = 0x00          # raw current (used for stop / zero-current)

def vel_cmd(erpm: int) -> bytes:
    return struct.pack(">i", erpm) + bytes(4)

def cur_cmd(ma: int) -> bytes:
    return struct.pack(">i", ma) + bytes(4)

STOP_CMD = cur_cmd(0)

# ---------------------------------------------------------------------------
# Motion parameters — adjust to taste
# ---------------------------------------------------------------------------

MOVE_SPEED_ERPM = 5_000    # max travel speed (ERPM)
HOLD_TIME       = 4.0      # max seconds per waypoint before moving on
SAMPLE_HZ       = 20       # CSV log rate
LOG_DIR = Path(__file__).parent / "data" / "logs"

# P-controller gains (velocity mode is the only confirmed-working mode;
# position/PVA modes do not respond — see docs/CAN_TROUBLESHOOTING.md)
KP         = 150    # ERPM per degree of error
TOLERANCE  = 1.5    # degrees — zero velocity once within this band
HOLD_AFTER = 1.5    # seconds to hold at target before next waypoint

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

def tx(bus: can.BusABC, mode: int, data: bytes) -> None:
    """Send an extended-ID CAN frame with the given mode and motor ID."""
    arb_id = (mode << 8) | MOTOR_ID
    bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=True))


def get_feedback(bus: can.BusABC, timeout: float = 0.02):
    """Read one frame and return (pos_deg, speed_erpm, cur_a) or None."""
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
    for _ in range(3):
        tx(bus, MODE_CURRENT, STOP_CMD)
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
) -> None:
    """Drive to target_degrees using velocity + P-controller and log feedback."""
    print(f"\n  ▶  Moving to {target_degrees:+.1f}°  —  {label}")

    last_csv_t  = -1.0
    csv_interval = 1.0 / SAMPLE_HZ
    settled_at  = None
    deadline    = time.monotonic() + HOLD_TIME

    while True:
        now = time.monotonic()

        # Send a keep-alive / command frame, then immediately read the reply —
        # same single-threaded send→recv pattern as spin_test.py.
        fb = get_feedback(bus, timeout=0.02)

        if fb is not None:
            pos, spd, cur, tmp, err_code = fb
            error = target_degrees - pos
            vel   = int(max(-MOVE_SPEED_ERPM, min(MOVE_SPEED_ERPM, KP * error)))

            if abs(error) < TOLERANCE:
                vel = 0
                if settled_at is None:
                    settled_at = now
            else:
                settled_at = None

            # Send velocity command; motor replies with another status frame
            # (that recv will be captured on the NEXT loop iteration — good enough
            #  at 50 Hz; we don't need to double-recv here).
            tx(bus, MODE_VELOCITY, vel_cmd(vel))

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
                    "error_code":    err_code,
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
            # No feedback — still send a keep-alive command so motor doesn't time out
            tx(bus, MODE_VELOCITY, vel_cmd(0))

        if settled_at is not None and (now - settled_at) >= HOLD_AFTER:
            break
        if now >= deadline:
            break

    stop_motor(bus)
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

    # CAN filter: only receive 0x2903 feedback frames — blocks the 0x0088 flood
    # (~30 kHz unknown device) that would otherwise starve every bus.recv() call.
    can_filters = [{"can_id": FEEDBACK_ID, "can_mask": 0x1FFFFFFF, "extended": True}]
    bus = can.interface.Bus(channel=INTERFACE, interface="socketcan",
                            can_filters=can_filters)

    try:
        with open(csv_path, "w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

            print("\n  Enabling motor...")
            tx(bus, MODE_CURRENT, ENABLE)
            time.sleep(0.15)

            # Verify motor is alive
            tx(bus, MODE_VELOCITY, vel_cmd(0))
            fb = get_feedback(bus, timeout=0.5)
            if fb is None:
                print("ERROR: Motor not responding.  Check wiring and CAN ID.")
                return
            print(f"  Motor alive — pos={fb[0]:.1f}°  temp={fb[3]}°C ✓")

            print("\n  Starting sequence — press Ctrl+C to abort safely\n")
            t0 = time.monotonic()

            try:
                for target_deg, label in WAYPOINTS:
                    move_and_sample(bus, target_deg, label, writer, t0)

            except KeyboardInterrupt:
                print("\n\n  Aborted — returning to home (0°)...")
                move_and_sample(bus, 0.0, "abort-return-home", writer, t0)

            finally:
                stop_motor(bus)
                tx(bus, MODE_CURRENT, DISABLE)

    finally:
        bus.shutdown()

    print("\n" + "=" * 60)
    print(f"  Done.  Data saved to:\n  {csv_path}")
    print("=" * 60)


if __name__ == "__main__":
    main()
