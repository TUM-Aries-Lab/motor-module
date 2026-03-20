"""Test CubeMars AK60-6 v3 native velocity loop (0x0303).

This script bypasses the high-level library and sends raw CAN frames so every
step is visible and auditable.

Why the previous test failed
-----------------------------
In motion_capture_vel.py we sent bare 0x0303 frames with no watchdog keepalive.
The motor's 100 ms CAN watchdog fires when it stops seeing frames on the *base*
arb_id (0x03), so the motor silently disabled itself.

What this test does
--------------------
Correct protocol sequence (matches what CubeMarsAK606v3CAN._refresh_loop does):

  PHASE 0 — Verify comms:
    Send  enable (0x03, FF FF FF FF FF FF FF FC)
    Wait  for 0x2903 feedback ACK
    Print position/ERPM in reply

  PHASE 1 — Velocity loop, 5 s at +1500 ERPM (~85 physical deg/s):
    At 50 Hz:
      Send  keepalive  arb_id=0x03  data=FF FF FF FF FF FF FF FC   (feeds 100 ms watchdog)
      Send  velocity   arb_id=0x0303  data=int32_be(erpm) + 0x00000000
    Collect every 0x2903 reply and print pos / ERPM / current / temp

  PHASE 2 — Velocity loop, 5 s at -1500 ERPM (reverse):
    Same as Phase 1, opposite sign.

  PHASE 3 — Stop:
    Send  current=0   arb_id=0x0103  data=int32_be(0) + 0x00000000
    Send  disable     arb_id=0x03   data=FF FF FF FF FF FF FF FD

Wiring / hardware assumptions
-------------------------------
  Motor CAN ID : 0x03
  Interface    : can0
  Bitrate      : 1 Mbps
  ERPM_PER_PHYS_DEG_S = 18.0  (calibrated 2026-03-10)

Usage
------
  .venv/bin/python scripts/test_velocity_loop.py           # default ±5000 ERPM
  .venv/bin/python scripts/test_velocity_loop.py 3000      # custom ERPM magnitude

CAN not working? (motor silent / TX buffer full)
-------------------------------------------------
  Running this script with 0x0303 fills the TX queue because the motor does not
  ACK those frames, pushing the Jetson into ERROR-PASSIVE. To recover:

  1. Unplug UART/R-Link cable from motor.
  2. Power-cycle the motor (unplug power, wait 3 s, reconnect).
  3. Reload the mttcan kernel module to reset hardware error counters:

       sudo ip link set can0 down && sudo modprobe -r mttcan && sleep 0.5 && \\
       sudo modprobe mttcan && sleep 0.2 && \\
       sudo ip link set can0 up type can bitrate 1000000 berr-reporting on restart-ms 100 && \\
       sudo ip link set can0 txqueuelen 1000

  4. Verify recovery: ip -details link show can0
     Expected: state ERROR-ACTIVE (berr-counter tx 0 rx 0)
  5. Verify motor is on bus: timeout 3 candump can0  (should see 0x2903 frames)
"""

from __future__ import annotations

import struct
import sys
import time
from typing import NamedTuple

import can

# ── Config ──────────────────────────────────────────────────────────────────
MOTOR_ID    = 0x03
INTERFACE   = "can0"
BITRATE     = 1_000_000

# arb_ids (extended 29-bit)
ARB_ENABLE   = MOTOR_ID                      # 0x0003  base ID — enable/disable/watchdog
ARB_CURRENT  = (0x01 << 8) | MOTOR_ID       # 0x0103  current loop  (used for clean stop)
ARB_VELOCITY = (0x03 << 8) | MOTOR_ID       # 0x0303  velocity loop  ← THE CANDIDATE
ARB_FEEDBACK = 0x2900 | MOTOR_ID            # 0x2903  status replies

# CAN spec payloads (base id)
ENABLE_PAYLOAD  = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
DISABLE_PAYLOAD = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])

# Test parameters
DEFAULT_ERPM  = 5000          # library blocks < 5000 ERPM (firmware low-speed PID issue)
PHASE_SECS    = 5.0           # seconds per direction
LOOP_HZ       = 50            # command rate
RECV_TIMEOUT  = 0.025         # 25 ms recv window — keepalive ACK + velocity reply

# Safety: never exceed this |ERPM| regardless of argument
MAX_SAFE_ERPM = 5000

# Feedback decoding (CubeMars 0x2903 frame, 8 bytes big-endian)
#   [0:2]  int16  pos      × 0.1  → degrees
#   [2:4]  int16  speed    × 10   → ERPM
#   [4:6]  int16  current  × 0.01 → amps
#   [6]    uint8  temp             → °C
#   [7]    uint8  error
class Feedback(NamedTuple):
    pos_deg:  float
    erpm:     int
    amps:     float
    temp_c:   int
    error:    int


def decode_feedback(data: bytes) -> Feedback | None:
    if len(data) < 8:
        return None
    pos   = struct.unpack_from(">h", data, 0)[0] * 0.1
    spd   = struct.unpack_from(">h", data, 2)[0] * 10
    cur   = struct.unpack_from(">h", data, 4)[0] * 0.01
    tmp   = data[6]
    err   = data[7]
    return Feedback(pos, spd, cur, tmp, err)


def send_raw(bus: can.BusABC, arb_id: int, data: bytes) -> bool:
    """Returns False if the TX buffer is full (caller should abort the phase)."""
    try:
        bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=True))
        return True
    except can.CanOperationError as e:
        if "buffer" in str(e).lower():
            return False  # TX buffer full — caller will break
        raise


def recv_feedback(bus: can.BusABC, deadline_s: float) -> list[tuple[int, Feedback | None]]:
    """Collect ALL extended frames until deadline_s.  Returns (arb_id, Feedback|None).
    Feedback is decoded when arb_id == ARB_FEEDBACK; otherwise None with arb_id logged.
    """
    replies: list[tuple[int, Feedback | None]] = []
    while True:
        remaining = deadline_s - time.monotonic()
        if remaining <= 0:
            break
        msg = bus.recv(timeout=remaining)
        if msg is None:
            break
        if msg.is_error_frame or not msg.is_extended_id:
            continue
        arb_id = msg.arbitration_id
        if arb_id == ARB_FEEDBACK:
            fb = decode_feedback(bytes(msg.data))
            replies.append((arb_id, fb))
        else:
            # Unexpected ID — log it so we know if replies arrive on a different arb_id
            replies.append((arb_id, None))
    return replies


def run_phase(bus: can.BusABC, erpm: int, secs: float, label: str) -> None:
    """Run the velocity loop for `secs` seconds at `erpm`, printing feedback."""
    print(f"\n{'─'*60}")
    print(f"  {label}  ({erpm:+d} ERPM, {secs:.0f} s)")
    print(f"  Sending: keepalive on 0x{ARB_ENABLE:04X}  then  velocity on 0x{ARB_VELOCITY:04X}")
    print(f"{'─'*60}")
    print(f"  {'t(s)':>6}  {'cmd_erpm':>9}  {'fb_erpm':>9}  {'pos°':>8}  {'A':>6}  {'°C':>4}  {'err':>4}")
    print(f"  {'------':>6}  {'---------':>9}  {'---------':>9}  {'--------':>8}  {'------':>6}  {'----':>4}  {'----':>4}")

    vel_data = struct.pack(">i", erpm) + bytes(4)
    interval = 1.0 / LOOP_HZ
    t_start  = time.monotonic()
    t_end    = t_start + secs
    last_print = t_start - interval  # print every tick
    any_motion = False

    while True:
        tick_start = time.monotonic()
        if tick_start >= t_end:
            break

        # ── Watchdog keepalive (required for non-duty-cycle modes) ─────────
        if not send_raw(bus, ARB_ENABLE, ENABLE_PAYLOAD):
            print("  TX BUFFER FULL on keepalive — aborting phase")
            break

        # ── Velocity command ───────────────────────────────────────────────
        if not send_raw(bus, ARB_VELOCITY, vel_data):
            print("  TX BUFFER FULL on velocity cmd — aborting phase")
            break

        # ── Collect replies (up to 25 ms) ──────────────────────────────────
        deadline = tick_start + RECV_TIMEOUT
        replies = recv_feedback(bus, deadline)

        t_now = time.monotonic() - t_start
        if replies:
            for arb_id, fb in replies:
                if fb is not None:          # known 0x2903 feedback
                    if abs(fb.erpm) > 50:
                        any_motion = True
                    print(f"  {t_now:6.2f}  {erpm:+9d}  {fb.erpm:+9d}  {fb.pos_deg:8.2f}  "
                          f"{fb.amps:6.2f}  {fb.temp_c:4d}  {fb.error:4d}")
                else:                       # unexpected arb_id — print it
                    print(f"  {t_now:6.2f}  {erpm:+9d}  {'??':>9}  {'???':>8}  "
                          f"{'???':>6}  {'??':>4}  {'??':>4}   [arb_id=0x{arb_id:08X}]")
        else:
            print(f"  {t_now:6.2f}  {erpm:+9d}  {'NO REPLY':>9}  {'---':>8}  "
                  f"{'---':>6}  {'--':>4}  {'--':>4}")

        # Sleep remainder of tick
        elapsed = time.monotonic() - tick_start
        sleep_t = max(0.0, interval - elapsed - RECV_TIMEOUT)
        if sleep_t > 0:
            time.sleep(sleep_t)

    if any_motion:
        print(f"\n  ✓ MOTION DETECTED during {label}")
    else:
        print(f"\n  ✗ NO MOTION detected during {label}  (feedback ERPM stayed near 0)")


def main() -> None:
    erpm_mag = DEFAULT_ERPM
    if len(sys.argv) > 1:
        try:
            erpm_mag = int(sys.argv[1])
        except ValueError:
            print(f"Usage: {sys.argv[0]} [erpm_magnitude]", file=sys.stderr)
            sys.exit(1)
    erpm_mag = min(abs(erpm_mag), MAX_SAFE_ERPM)

    print("=" * 60)
    print("  CubeMars AK60-6 v3 — velocity loop (0x0303) test")
    print(f"  Motor 0x{MOTOR_ID:02X}  interface={INTERFACE}  test_erpm=±{erpm_mag}")
    print("=" * 60)
    print("\n  arb_id map:")
    print(f"    keepalive / enable : 0x{ARB_ENABLE:04X}")
    print(f"    velocity command   : 0x{ARB_VELOCITY:04X}  ← under test")
    print(f"    feedback replies   : 0x{ARB_FEEDBACK:04X}")

    # Accept ALL extended frames — we want to see every arb_id the motor sends
    # (in case velocity-mode replies arrive on a different ID than 0x2903).
    bus = can.interface.Bus(
        channel=INTERFACE,
        interface="socketcan",
        bitrate=BITRATE,
    )

    try:
        # ── Phase 0: enable + check comms ──────────────────────────────────
        print("\n── Phase 0: enable motor & verify comms ──")
        send_raw(bus, ARB_ENABLE, ENABLE_PAYLOAD)
        time.sleep(0.05)
        replies = recv_feedback(bus, time.monotonic() + 0.1)
        if replies:
            fb = replies[-1]
            print(f"  ACK received  pos={fb.pos_deg:.2f}°  ERPM={fb.erpm:+d}  "
                  f"cur={fb.amps:.2f}A  temp={fb.temp_c}°C  err={fb.error}")
        else:
            print("  WARNING: no 0x2903 reply to enable frame — motor may not be on bus")
            print("  Continuing anyway (some firmware versions don't ACK enable)...")

        # ── Phase 1: forward ────────────────────────────────────────────────
        run_phase(bus, +erpm_mag, PHASE_SECS, "Phase 1 — FORWARD")

        # ── Phase 2: reverse ────────────────────────────────────────────────
        run_phase(bus, -erpm_mag, PHASE_SECS, "Phase 2 — REVERSE")

        # ── Phase 3: stop cleanly ───────────────────────────────────────────
        print("\n── Phase 3: stop (current=0, then disable) ──")
        stop_data = struct.pack(">i", 0) + bytes(4)
        for _ in range(10):
            send_raw(bus, ARB_ENABLE, ENABLE_PAYLOAD)         # keep watchdog alive
            send_raw(bus, ARB_CURRENT, stop_data)              # current=0 A (releases windings)
            time.sleep(0.02)
        send_raw(bus, ARB_ENABLE, DISABLE_PAYLOAD)
        print("  Disable frame sent.")

    finally:
        bus.shutdown()
        print("\nDone.")


if __name__ == "__main__":
    main()
