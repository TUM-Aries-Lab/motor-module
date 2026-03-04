"""Motion capture validation — duty-cycle P-controller sine sweep.

Motor: CubeMars AK60-6 v3, CAN ID 0x03, CAN bus (can0, 1 Mbps).

The 100 ms CAN watchdog only resets from frames on arb_id = MOTOR_ID (0x03).
Position-mode (0x0403) and velocity-mode (0x0303) do NOT feed the watchdog
and the motor goes silent after ~100 ms.  Therefore we use **duty-cycle
control** on arb_id 0x03 with a software proportional controller.

The motor's 0x0088 standard-frame flood is handled by draining the receive
buffer until a valid feedback frame is found within a 15 ms deadline.

Physical angle convention (cable-drive joint):
  phys = 0 deg  → arm hanging straight down (gravity rest)
  phys increases → arm swings forward / up
  fw_target = fw_home + phys * _fw_dir

Motion sequence
---------------
  1.  0 → 120 deg   raise arm directly to 120° (no intermediate stop)
  2.  Sine wave      oscillate 120 ↔ 60 deg for SINE_DURATION s
  3.  → 0 deg        coast back to gravity hang

Run:
    sudo ./setup_can.sh
    source .venv/bin/activate
    python motion_capture_test.py
"""

import csv
import math
import struct
import time
from datetime import datetime
from pathlib import Path

import can


class BusDownError(RuntimeError):
    """Raised when the CAN socket reports the interface has gone down."""


# ──────────────────────────────────────────────────────────────────────────
# Hardware
# ──────────────────────────────────────────────────────────────────────────
MOTOR_ID  = 0x03
INTERFACE = "can0"
DUTY_ARB  = MOTOR_ID          # 0x03 — duty-cycle frames (keeps watchdog alive)

# ──────────────────────────────────────────────────────────────────────────
# CAN payloads
# ──────────────────────────────────────────────────────────────────────────
ENABLE_CMD  = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
DISABLE_CMD = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
STOP_CMD    = struct.pack(">i", 0) + bytes(4)   # duty = 0%

# Feedback arbitration IDs (motor may use any of these variants)
FEEDBACK_IDS = {
    MOTOR_ID,
    MOTOR_ID + 1,            # 0x04  — firmware variant
    0x2900 | MOTOR_ID,       # 0x2903
    0x2900,                   # enable-response
    0x0080 | MOTOR_ID,       # 0x0083
}

# ──────────────────────────────────────────────────────────────────────────
# P-controller
# ──────────────────────────────────────────────────────────────────────────
KP        = 0.005        # duty per degree of error (proportional gain)
KD        = 0.000015     # duty per ERPM (damping — reduces overshoot)
MAX_DUTY  = 0.30         # 30% absolute max — needed to fight gravity at 90°+
DEADBAND  = 1.0          # degrees — no correction below this error

# ──────────────────────────────────────────────────────────────────────────
# Motion parameters
# ──────────────────────────────────────────────────────────────────────────
WAYPOINT_SETTLE_DEG  = 5.0
WAYPOINT_SETTLE_ERPM = 500
WAYPOINT_SETTLE_TIME = 0.3
WAYPOINT_TIMEOUT     = 20.0

SINE_CENTER   = 90.0    # degrees
SINE_AMP      = 30.0    # degrees
SINE_PERIOD   = 6.0     # seconds
SINE_DURATION = 90.0    # seconds (~15 full cycles at 6s period)

ROT_GUARD_NEG = 25.0
ROT_GUARD_POS = 175.0
PHYS_MIN      = -20.0
PHYS_MAX      = 190.0
MAX_SAFE_ERPM = 6000    # coast to a stop if speed exceeds this during sine sweep
                        # NOTE: normal sine peaks at ~2560 ERPM; 6000 gives 2× margin
BRAKE_ERPM    = 1500    # resume normal control once speed drops below this

LOOP_HZ   = 50
SAMPLE_HZ = 20

LOG_DIR = Path(__file__).parent / "data" / "logs"

# ──────────────────────────────────────────────────────────────────────────
# Direction state  (set by detect_fw_dir)
# ──────────────────────────────────────────────────────────────────────────
_fw_dir: int = +1   # +1 means fw increases as phys increases (normal cable)


def phys_to_fw(phys: float, fw_home: float) -> float:
    return fw_home + phys * _fw_dir


def fw_to_phys(fw: float, fw_home: float) -> float:
    return (fw - fw_home) * _fw_dir


# ──────────────────────────────────────────────────────────────────────────
# CAN helpers
# ──────────────────────────────────────────────────────────────────────────

def tx(bus: can.BusABC, data: bytes, arb_id: int = DUTY_ARB) -> None:
    try:
        bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=True))
    except (can.CanOperationError, OSError):
        pass


def duty_frame(duty: float) -> bytes:
    """Encode duty [-1.0 .. +1.0] as 8-byte CAN payload."""
    duty = max(-1.0, min(1.0, duty))
    raw = int(duty * 100_000)
    return struct.pack(">i", raw) + bytes(4)


def tx_duty(bus: can.BusABC, duty: float) -> None:
    """Send a duty-cycle command on arb_id=MOTOR_ID (keeps watchdog alive)."""
    tx(bus, duty_frame(duty), arb_id=DUTY_ARB)


def read_feedback(bus: can.BusABC, deadline_ms: float = 15.0):
    """Drain 0x0088 flood and return the first valid feedback frame.

    Loops through received CAN frames for up to *deadline_ms* milliseconds,
    skipping non-feedback frames (e.g. the 0x0088 flood).

    Returns (pos_deg, erpm, cur_A, temp_C, err_code) or None.
    Raises BusDownError if the CAN interface went BUS-OFF.
    """
    deadline = time.monotonic() + deadline_ms / 1000.0
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        try:
            msg = bus.recv(timeout=remaining)
        except (can.CanOperationError, OSError) as exc:
            raise BusDownError(str(exc)) from exc
        if msg is None:
            return None
        if msg.arbitration_id not in FEEDBACK_IDS:
            continue          # skip 0x0088 and other noise
        d = msg.data
        if len(d) < 8:
            continue
        pos_deg = struct.unpack(">h", d[0:2])[0] * 0.1
        erpm    = struct.unpack(">h", d[2:4])[0] * 10
        cur_a   = struct.unpack(">h", d[4:6])[0] * 0.01
        # Byte 6 is signed int8 (-20..127 °C).  The enable-ACK frame sometimes
        # carries uninitialised data here; clamp to a plausible range.
        temp_raw = struct.unpack("b", bytes([d[6]]))[0]
        temp_c   = temp_raw if -20 <= temp_raw <= 127 else 0
        err      = d[7]
        return pos_deg, erpm, cur_a, temp_c, err


# ──────────────────────────────────────────────────────────────────────────
# P-controller
# ──────────────────────────────────────────────────────────────────────────

def compute_duty(error_phys: float, speed_erpm: int) -> float:
    """Proportional-derivative controller → duty command.

    error_phys: target_phys - actual_phys  (positive = need to swing forward)
    speed_erpm: current motor speed in ERPM (feedback)

    Returns duty in [-MAX_DUTY, +MAX_DUTY], in the fw direction.
    """
    if abs(error_phys) < DEADBAND:
        return 0.0
    p = KP * error_phys
    d = KD * speed_erpm * _fw_dir   # damping: opposes motion in phys-space
    duty_phys = p - d
    # Convert to fw-space duty
    duty_fw = duty_phys * _fw_dir
    return max(-MAX_DUTY, min(MAX_DUTY, duty_fw))


# ──────────────────────────────────────────────────────────────────────────
# Init
# ──────────────────────────────────────────────────────────────────────────

def find_home(bus: can.BusABC) -> float:
    """Coast arm to gravity hang and return fw_home (= physical 0°)."""
    STILL_ERPM = 200
    STILL_TIME = 1.5
    TIMEOUT    = 15.0

    print("  Waiting for arm to settle at gravity home...", end="", flush=True)
    t0       = time.monotonic()
    still_at = None
    fw_home  = None
    last_fw  = None

    while time.monotonic() - t0 < TIMEOUT:
        tx(bus, STOP_CMD)
        fb = read_feedback(bus)
        if fb is None:
            time.sleep(0.01)
            continue
        fw_pos, spd, *_ = fb
        last_fw = fw_pos
        now = time.monotonic()
        if abs(spd) < STILL_ERPM:
            if still_at is None:
                still_at = now
            elif now - still_at >= STILL_TIME:
                fw_home = fw_pos
                break
        else:
            still_at = None
        time.sleep(0.01)

    fw_home = fw_home if fw_home is not None else (last_fw or 0.0)
    print(f" settled  fw_home={fw_home:.1f} deg")
    return fw_home


def detect_fw_dir(bus: can.BusABC, fw_home: float) -> int:
    """Determine sign of fw_pos vs physical angle with a brief duty pulse.

    Sends a small positive duty for 1 s, observes which way fw_pos moves.
    """
    global _fw_dir

    TEST_DUTY = 0.02     # 2% — gentle nudge
    TEST_SEC  = 0.5
    DEADZONE  = 0.5      # degrees

    print("  Detecting fw direction (1 s duty pulse)...", end="", flush=True)

    # Flush stale frames
    while bus.recv(timeout=0.01):
        pass

    fw_start = None
    fw_end   = fw_home
    t0 = time.monotonic()
    while time.monotonic() - t0 < TEST_SEC:
        tx_duty(bus, TEST_DUTY)
        fb = read_feedback(bus)
        if fb:
            if fw_start is None:
                fw_start = fb[0]
            fw_end = fb[0]
        time.sleep(0.01)

    if fw_start is None:
        fw_start = fw_home

    # Coast back
    for _ in range(40):
        tx(bus, STOP_CMD)
        time.sleep(0.015)

    delta = fw_end - fw_start
    if abs(delta) < DEADZONE:
        print(f" no movement ({delta:+.1f} deg) — keeping _fw_dir={_fw_dir:+d}")
    elif delta > 0:
        _fw_dir = +1
        print(f" fw moved {delta:+.1f} deg → _fw_dir=+1  (positive duty = forward)")
    else:
        _fw_dir = -1
        print(f" fw moved {delta:+.1f} deg → _fw_dir=-1  [INVERTED]")

    return _fw_dir


# ──────────────────────────────────────────────────────────────────────────
# Safety
# ──────────────────────────────────────────────────────────────────────────

def check_safety(fw_pos: float, fw_home: float) -> str | None:
    fw_delta = fw_pos - fw_home
    phys     = fw_to_phys(fw_pos, fw_home)
    if fw_delta < -ROT_GUARD_NEG:
        return f"ROT GUARD NEG: fw_delta={fw_delta:+.1f}"
    if fw_delta > ROT_GUARD_POS:
        return f"ROT GUARD POS: fw_delta={fw_delta:+.1f}"
    if phys < PHYS_MIN:
        return f"PHYS MIN: phys={phys:.1f}"
    if phys > PHYS_MAX:
        return f"PHYS MAX: phys={phys:.1f}"
    return None


# ──────────────────────────────────────────────────────────────────────────
# Motion primitives
# ──────────────────────────────────────────────────────────────────────────

def move_to(
    bus: can.BusABC,
    phys_tgt: float,
    fw_home: float,
    label: str,
    writer: csv.DictWriter,
    t0: float,
) -> bool:
    """Drive arm to phys_tgt using P-controller on duty-cycle.

    Returns True on success, False on safety abort.
    """
    loop_dt    = 1.0 / LOOP_HZ
    csv_dt     = 1.0 / SAMPLE_HZ
    last_csv   = -1.0
    settled_at = None
    deadline   = time.monotonic() + WAYPOINT_TIMEOUT
    no_fb      = 0

    print(f"\n  >> MOVE  target={phys_tgt:+.1f} deg  [{label}]")

    while True:
        tick = time.monotonic()

        # Will compute duty after reading feedback
        try:
            fb = read_feedback(bus)
        except BusDownError as exc:
            print(f"\n  ✗ CAN bus down during move_to: {exc}")
            return False
        if fb is None:
            # Keep watchdog alive even without feedback
            tx(bus, STOP_CMD)
            no_fb += 1
            if no_fb % 50 == 0:
                print(f"  ⚠ no feedback ({no_fb} ticks)", end="\r")
            elapsed = tick - tick  # can't compute
            time.sleep(max(loop_dt - (time.monotonic() - tick), 0.005))
            continue
        no_fb = 0

        fw_pos, spd, cur, tmp, err_code = fb
        now    = time.monotonic()
        phys   = fw_to_phys(fw_pos, fw_home)
        err    = phys_tgt - phys
        elapsed = now - t0

        # Safety
        fault = check_safety(fw_pos, fw_home)
        if fault:
            tx(bus, STOP_CMD)
            print(f"\n  !! {fault} — aborting")
            return False

        # P-controller
        duty = compute_duty(err, spd)
        tx_duty(bus, duty)

        # CSV logging
        if elapsed - last_csv >= csv_dt:
            last_csv = elapsed
            writer.writerow({
                "time_s":        f"{elapsed:.3f}",
                "label":         label,
                "commanded_deg": f"{phys_tgt:.1f}",
                "actual_deg":    f"{phys:.2f}",
                "error_deg":     f"{err:.2f}",
                "duty":          f"{duty:.5f}",
                "speed_erpm":    spd,
                "current_amps":  f"{cur:.3f}",
                "temp_c":        tmp,
                "error_code":    err_code,
            })

        print(
            f"    t={elapsed:6.2f}s  phys={phys:+7.2f}  err={err:+6.1f}  "
            f"duty={duty:+.4f}  erpm={spd:+6d}  cur={cur:+5.2f}A",
            end="\r",
        )

        # Settle check
        if abs(err) < WAYPOINT_SETTLE_DEG and abs(spd) < WAYPOINT_SETTLE_ERPM:
            if settled_at is None:
                settled_at = now
        else:
            settled_at = None

        if settled_at is not None and (now - settled_at) >= WAYPOINT_SETTLE_TIME:
            print(f"\n  ✓ Settled at phys={phys:+.2f} (err={err:+.2f})")
            return True

        if now >= deadline:
            print(f"\n  ⏱ Timeout — phys={phys:+.2f} err={err:+.2f}")
            return True

        time.sleep(max(loop_dt - (time.monotonic() - tick), 0.001))


def sine_sweep(
    bus: can.BusABC,
    fw_home: float,
    writer: csv.DictWriter,
    t0: float,
) -> bool:
    """Oscillate arm sinusoidally: phys(t) = 90 + 30·cos(2πt/6) for 10 s.

    Starts at 120° (cos(0)=1) and ends near 120° after ~1.67 full cycles.
    """
    loop_dt  = 1.0 / LOOP_HZ
    csv_dt   = 1.0 / SAMPLE_HZ
    last_csv = -1.0
    no_fb    = 0
    label    = "sine-sweep"

    t_start = time.monotonic()
    t_end   = t_start + SINE_DURATION

    print(
        f"\n  >> SINE  {SINE_CENTER}±{SINE_AMP} deg  "
        f"period={SINE_PERIOD}s  duration={SINE_DURATION}s"
    )

    while True:
        tick = time.monotonic()
        t    = tick - t_start
        if tick >= t_end:
            print(f"\n  ✓ Sine sweep complete ({SINE_DURATION:.0f} s)")
            break

        phys_tgt = SINE_CENTER + SINE_AMP * math.cos(2 * math.pi * t / SINE_PERIOD)

        try:
            fb = read_feedback(bus)
        except BusDownError as exc:
            print(f"\n  ✗ CAN bus down during sine_sweep: {exc}")
            return False
        if fb is None:
            tx(bus, STOP_CMD)
            no_fb += 1
            time.sleep(max(loop_dt - (time.monotonic() - tick), 0.005))
            continue
        no_fb = 0

        fw_pos, spd, cur, tmp, err_code = fb
        now    = time.monotonic()
        phys   = fw_to_phys(fw_pos, fw_home)
        err    = phys_tgt - phys
        elapsed = now - t0

        fault = check_safety(fw_pos, fw_home)
        if fault:
            tx(bus, STOP_CMD)
            print(f"\n  !! {fault} — aborting sine")
            return False

        # Speed brake: if motor is moving too fast, coast until it slows down
        if abs(spd) > MAX_SAFE_ERPM:
            print(f"\n  ⚡ Speed brake: {spd:+d} ERPM > {MAX_SAFE_ERPM} — coasting...",
                  end="", flush=True)
            while abs(spd) > BRAKE_ERPM:
                tx(bus, STOP_CMD)
                try:
                    fb2 = read_feedback(bus)
                except BusDownError as exc:
                    print(f"\n  ✗ CAN bus down during speed brake: {exc}")
                    return False
                if fb2:
                    spd = fb2[1]
                    # Update phys and log while braking
                    phys = fw_to_phys(fb2[0], fw_home)
                    now2 = time.monotonic()
                    elapsed2 = now2 - t0
                    if elapsed2 - last_csv >= csv_dt:
                        last_csv = elapsed2
                        writer.writerow({
                            "time_s":        f"{elapsed2:.3f}",
                            "label":         "brake",
                            "commanded_deg": f"{phys_tgt:.2f}",
                            "actual_deg":    f"{phys:.2f}",
                            "error_deg":     f"{phys_tgt - phys:.2f}",
                            "duty":          "0.00000",
                            "speed_erpm":    spd,
                            "current_amps":  f"{fb2[2]:.3f}",
                            "temp_c":        fb2[3],
                            "error_code":    fb2[4],
                        })
                    # Safety check inside brake loop — abort if position goes OOB
                    brake_fault = check_safety(fb2[0], fw_home)
                    if brake_fault:
                        tx(bus, STOP_CMD)
                        print(f"\n  !! {brake_fault} during speed brake — aborting")
                        return False
                time.sleep(0.01)
            print(f" resumed at {spd:+d} ERPM")

        duty = compute_duty(err, spd)
        tx_duty(bus, duty)

        if elapsed - last_csv >= csv_dt:
            last_csv = elapsed
            writer.writerow({
                "time_s":        f"{elapsed:.3f}",
                "label":         label,
                "commanded_deg": f"{phys_tgt:.2f}",
                "actual_deg":    f"{phys:.2f}",
                "error_deg":     f"{err:.2f}",
                "duty":          f"{duty:.5f}",
                "speed_erpm":    spd,
                "current_amps":  f"{cur:.3f}",
                "temp_c":        tmp,
                "error_code":    err_code,
            })

        print(
            f"    t={t:5.2f}/{SINE_DURATION:.0f}s  "
            f"cmd={phys_tgt:+7.2f}  phys={phys:+7.2f}  "
            f"err={err:+5.1f}  duty={duty:+.4f}  erpm={spd:+6d}",
            end="\r",
        )

        time.sleep(max(loop_dt - (time.monotonic() - tick), 0.001))

    return True


def coast_to_hang(
    bus: can.BusABC,
    fw_home: float,
    writer: csv.DictWriter,
    t0: float,
) -> None:
    """Send duty=0 and let gravity lower the arm."""
    STILL_ERPM = 300
    STILL_TIME = 2.0
    label      = "hang-return"
    loop_dt    = 1.0 / LOOP_HZ
    csv_dt     = 1.0 / SAMPLE_HZ
    last_csv   = -1.0
    still_at   = None
    deadline   = time.monotonic() + WAYPOINT_TIMEOUT

    print("\n  >> COAST  lowering arm to gravity hang (duty=0)...")

    while time.monotonic() < deadline:
        tick = time.monotonic()
        tx(bus, STOP_CMD)
        try:
            fb = read_feedback(bus)
        except BusDownError as exc:
            print(f"\n  ✗ CAN bus down during coast_to_hang: {exc}")
            break
        if fb is None:
            time.sleep(max(loop_dt - (time.monotonic() - tick), 0.005))
            continue

        fw_pos, spd, cur, tmp, err_code = fb
        now     = time.monotonic()
        phys    = fw_to_phys(fw_pos, fw_home)
        elapsed = now - t0

        if elapsed - last_csv >= csv_dt:
            last_csv = elapsed
            writer.writerow({
                "time_s":        f"{elapsed:.3f}",
                "label":         label,
                "commanded_deg": "0.0",
                "actual_deg":    f"{phys:.2f}",
                "error_deg":     f"{phys:.2f}",
                "duty":          "0.00000",
                "speed_erpm":    spd,
                "current_amps":  f"{cur:.3f}",
                "temp_c":        tmp,
                "error_code":    err_code,
            })

        print(
            f"    t={elapsed:6.2f}s  phys={phys:+7.2f}  erpm={spd:+6d}",
            end="\r",
        )

        if abs(spd) < STILL_ERPM:
            if still_at is None:
                still_at = now
        else:
            still_at = None

        if still_at is not None and (now - still_at) >= STILL_TIME:
            print(f"\n  ✓ Arm at rest  phys={phys:+.2f}")
            break

        time.sleep(max(loop_dt - (time.monotonic() - tick), 0.001))


# ──────────────────────────────────────────────────────────────────────────
# Summary
# ──────────────────────────────────────────────────────────────────────────

def print_summary(csv_path: Path) -> None:
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print("  No data recorded.")
        return

    from collections import OrderedDict
    by_label: dict[str, list] = OrderedDict()
    for r in rows:
        by_label.setdefault(r["label"], []).append(r)

    print("\n" + "=" * 72)
    print("  POST-RUN SUMMARY")
    print("=" * 72)
    print(f"  {'Label':<18s}  {'N':>5s}  {'MeanErr':>8s}  {'MaxErr':>7s}  "
          f"{'PeakERPM':>9s}  {'PeakDuty':>9s}")
    print("  " + "-" * 66)
    for lbl, lrows in by_label.items():
        errs   = [abs(float(r["error_deg"])) for r in lrows]
        erpms  = [abs(int(r["speed_erpm"])) for r in lrows]
        duties = [abs(float(r["duty"])) for r in lrows]
        n = len(lrows)
        print(
            f"  {lbl:<18s}  {n:>5d}  {sum(errs)/n:>7.2f}°  "
            f"{max(errs):>6.2f}°  {max(erpms):>9d}  {max(duties):>8.4f}"
        )
    print("=" * 72)
    dur = float(rows[-1]["time_s"]) - float(rows[0]["time_s"])
    print(f"  Duration: {dur:.1f} s  |  Samples: {len(rows)}")
    print(f"  CSV: {csv_path}")


# ──────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────

def main() -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts       = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_path = LOG_DIR / f"mocap_{ts}.csv"

    fieldnames = [
        "time_s", "label", "commanded_deg", "actual_deg", "error_deg",
        "duty", "speed_erpm", "current_amps", "temp_c", "error_code",
    ]

    print("=" * 64)
    print("  Motion Capture — Duty-Cycle P-Controller Sine Sweep")
    print(f"  Motor ID: 0x{MOTOR_ID:02X}   Interface: {INTERFACE}")
    print(f"  KP={KP}  KD={KD}  MAX_DUTY={MAX_DUTY}  DEADBAND={DEADBAND}")
    print(f"  Sine: {SINE_CENTER}±{SINE_AMP} deg  period={SINE_PERIOD}s  "
          f"duration={SINE_DURATION}s")
    print(f"  Loop: {LOOP_HZ} Hz   CSV: {SAMPLE_HZ} Hz")
    print(f"  Log: {csv_path}")
    print("=" * 64)

    # ── Pre-flight: confirm bus is live before opening filtered socket ─────
    # Open a raw unfiltered socket for 1.5 s.  If we see zero frames the motor
    # is almost certainly powered off — sending ENABLE on a dead bus causes TX
    # errors → BUS-OFF → socket dies.  Print a clear message and exit early.
    print("  Pre-flight: checking for CAN bus traffic...")
    _raw_bus = can.interface.Bus(channel=INTERFACE, interface="socketcan")
    _t0_sniff = time.monotonic()
    _got_frame = False
    while time.monotonic() - _t0_sniff < 1.5:
        try:
            _m = _raw_bus.recv(timeout=0.1)
        except (can.CanOperationError, OSError):
            break
        if _m is not None:
            _got_frame = True
            break
    _raw_bus.shutdown()
    if not _got_frame:
        print(
            "  ✗ No CAN frames detected in 1.5 s — motor appears to be OFF.\n"
            "  Power on the motor, then re-run:  python motion_capture_test.py"
        )
        return
    print("  ✓ Bus is live — opening filtered socket")

    # Apply kernel-level CAN receive filter so the 0x0088 standard-frame
    # flood (~30 kfps) is dropped before it fills the socket buffer and
    # starves Python of motor feedback frames.
    _can_filters = [
        {"can_id": fid, "can_mask": 0x1FFFFFFF, "extended": True}
        for fid in FEEDBACK_IDS
    ]
    bus = can.interface.Bus(
        channel=INTERFACE,
        interface="socketcan",
        can_filters=_can_filters,
    )

    try:
        with open(csv_path, "w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

            # ── Enable ────────────────────────────────────────────────────
            print("\n  Enabling motor...")
            tx(bus, ENABLE_CMD)
            time.sleep(0.3)

            fw_alive = None
            for attempt in range(30):
                try:
                    fb = read_feedback(bus)
                except BusDownError as exc:
                    print(f"\n  ✗ CAN bus went down during enable loop: {exc}")
                    print("  Run: sudo ./setup_can.sh   then try again.")
                    return
                if fb is not None:
                    fw_alive = fb[0]
                    print(
                        f"  Motor alive  fw={fw_alive:.1f}°  temp={fb[3]}°C  "
                        f"cur={fb[2]:.2f}A  err={fb[4]}  (attempt {attempt + 1})"
                    )
                    break
                tx(bus, ENABLE_CMD)
                time.sleep(0.1)

            if fw_alive is None:
                print("  ✗ No motor feedback — check power & wiring")
                return

            # ── Home ──────────────────────────────────────────────────────
            fw_home = find_home(bus)
            print(f"  Physical 0° = fw_pos {fw_home:.1f}°")
            print(f"  Safety: fw_delta [{-ROT_GUARD_NEG}, +{ROT_GUARD_POS}]°   "
                  f"phys [{PHYS_MIN}, {PHYS_MAX}]°")

            # ── Direction detection ───────────────────────────────────────
            detect_fw_dir(bus, fw_home)
            sign = "+" if _fw_dir > 0 else "-"
            print(f"  _fw_dir={_fw_dir:+d}   duty_fw = duty_phys × ({sign}1)")

            # Re-home after direction test
            fw_home = find_home(bus)
            print(f"  Re-homed: fw_home={fw_home:.1f}°")
            print("\n  Press Ctrl+C to abort at any time\n")

            t0 = time.monotonic()

            try:
                # Step 1: 0 → 120° directly (no pause at 90°)
                ok = move_to(bus, 120.0, fw_home, "swing-up", writer, t0)
                if not ok:
                    raise RuntimeError("move to 120° aborted by safety")

                # Step 2: Sine sweep 120 ↔ 60°
                ok = sine_sweep(bus, fw_home, writer, t0)
                if not ok:
                    raise RuntimeError("sine sweep aborted by safety")

                # Step 3: Coast back to 0° (gravity hang)
                coast_to_hang(bus, fw_home, writer, t0)

            except KeyboardInterrupt:
                print("\n\n  Ctrl+C — stopping safely...")
            except (RuntimeError, BusDownError) as e:
                print(f"\n  Sequence aborted: {e}")

            # ── Stop & disable ────────────────────────────────────────────
            print("\n  Sending stop + disable...")
            for _ in range(15):
                tx(bus, STOP_CMD)
                time.sleep(0.02)
            tx(bus, DISABLE_CMD)
            time.sleep(0.2)

    finally:
        bus.shutdown()

    print_summary(csv_path)
    print(f"\n  Done.  Data → {csv_path}")
    print("=" * 64)


if __name__ == "__main__":
    main()
