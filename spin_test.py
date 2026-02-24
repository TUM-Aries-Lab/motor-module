# ruff: noqa: T201
"""Spin the motor forward for 2 s, pause, then spin backward for 2 s.

Demonstrates CAN communication in both directions.
Run:  sudo ./setup_can.sh && python spin_test.py
"""
import struct
import time

import can

MOTOR_ID  = 0x03
DUTY      = 0.20      # 20% — adjust if you want more/less speed
SPIN_SECS = 2.0
PAUSE_SECS = 0.5

ENABLE  = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
DISABLE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
FWD_CMD  = struct.pack(">i", int( DUTY * 100_000)) + bytes(4)
REV_CMD  = struct.pack(">i", int(-DUTY * 100_000)) + bytes(4)
STOP_CMD = struct.pack(">i", 0) + bytes(4)

print("=" * 54)
print("  CAN Motor Demo — Forward / Reverse")
print(f"  Motor ID : {MOTOR_ID}  |  Duty: ±{DUTY*100:.0f}%  |  {SPIN_SECS}s each")
print("  Interface: can0  |  Bitrate: 1 Mbps")
print("=" * 54)

bus = can.interface.Bus(channel="can0", interface="socketcan")


def tx(arb_id, data, ext=True):
    bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=ext))


def get_feedback():
    m = bus.recv(timeout=0.02)
    if m and m.arbitration_id == 0x2903 and len(m.data) == 8:
        d = m.data
        pos = struct.unpack(">h", d[0:2])[0] * 0.1
        spd = struct.unpack(">h", d[2:4])[0] * 10
        cur = struct.unpack(">h", d[4:6])[0] * 0.01
        return pos, spd, cur
    return None


def spin(label: str, cmd: bytes, duration: float) -> None:
    """Send *cmd* at 50 Hz for *duration* seconds and print live feedback."""
    print(f"\n[{label}]  ({duration}s @ {DUTY*100:.0f}% duty)")
    t0 = time.time()
    last_print = -1.0
    while time.time() - t0 < duration:
        tx(MOTOR_ID, cmd)
        fb = get_feedback()
        now = time.time() - t0
        if fb and now - last_print >= 0.25:
            pos, spd, cur = fb
            print(f"  t={now:.1f}s   pos={pos:8.1f} deg   spd={spd:+7d} ERPM   cur={cur:5.2f} A")
            last_print = now
        time.sleep(0.02)


def send_stop() -> None:
    """Send zero-current stop frames."""
    for _ in range(3):
        tx(MOTOR_ID, STOP_CMD)
        time.sleep(0.02)


# ── 1. Enable ──────────────────────────────────────────────────────────────
print("\nEnabling motor...")
tx(MOTOR_ID, ENABLE)
time.sleep(0.15)

# ── 2. Forward ─────────────────────────────────────────────────────────────
spin("FORWARD", FWD_CMD, SPIN_SECS)

# ── 3. Brief stop between directions ───────────────────────────────────────
print(f"\n[PAUSE]  ({PAUSE_SECS}s)")
send_stop()
time.sleep(PAUSE_SECS)

# ── 4. Reverse ─────────────────────────────────────────────────────────────
spin("REVERSE", REV_CMD, SPIN_SECS)

# ── 5. Stop and disable ─────────────────────────────────────────────────────
print("\n[STOP]")
send_stop()
time.sleep(0.1)
tx(MOTOR_ID, DISABLE)

fb = get_feedback()
if fb:
    print(f"  Final:  pos={fb[0]:8.1f} deg   spd={fb[1]:+7d} ERPM   cur={fb[2]:5.2f} A")

bus.shutdown()
print("\nDone.\n")
