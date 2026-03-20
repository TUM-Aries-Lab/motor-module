#!/usr/bin/env python3
"""Comprehensive motor drive test - try various duty formats, levels, and modes."""
import struct
import sys
import time

import can

sys.stdout.reconfigure(line_buffering=True)

MOTOR_ID = 0x03
FEEDBACK_ID = 0x2903

bus = can.Bus(channel="can0", interface="socketcan")

def drain(max_iter=100):
    for _ in range(max_iter):
        if bus.recv(timeout=0.01) is None:
            break

def parse_feedback(data):
    """Parse 0x2903 feedback using CubeMars format."""
    pos = struct.unpack(">h", data[0:2])[0] * 0.1
    spd = struct.unpack(">h", data[2:4])[0] * 10
    cur = struct.unpack(">h", data[4:6])[0] * 0.01
    tmp = struct.unpack("b", bytes([data[6]]))[0]
    err = data[7]
    return pos, spd, cur, tmp, err

def tx_and_read(arb_id, data, ext=True, label="", duration=0.15):
    """Send frame and collect motor feedback for duration seconds."""
    drain()
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=ext)
    bus.send(msg)

    feedbacks = []
    deadline = time.time() + duration
    while time.time() < deadline:
        r = bus.recv(timeout=0.02)
        if r and r.arbitration_id == FEEDBACK_ID:
            feedbacks.append(r)

    if feedbacks:
        last = feedbacks[-1]
        pos, spd, cur, tmp, err = parse_feedback(last.data)
        print(f"  {label:<35s} -> {len(feedbacks)} fb | pos={pos:7.1f}° spd={spd:+7d} cur={cur:5.2f}A tmp={tmp}°C err={err} raw={last.data.hex()}")
    else:
        print(f"  {label:<35s} -> NO feedback")
    return feedbacks

ENABLE  = bytes([0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC])
DISABLE = bytes([0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD])

print("=" * 80)
print("COMPREHENSIVE MOTOR DRIVE TEST")
print("=" * 80)

# ── 1. Baseline ──
print("\n[1] BASELINE (no enable)")
tx_and_read(MOTOR_ID, bytes(8), ext=True, label="Null frame")

# ── 2. Enable ──
print("\n[2] ENABLE")
tx_and_read(MOTOR_ID, ENABLE, ext=True, label="Enable (ext)")

# ── 3. Duty Cycle - 4 bytes (proper VESC format) ──
print("\n[3] DUTY CYCLE - 4 bytes (VESC format)")
for duty_pct in [10, 30, 50, 80]:
    duty_val = int(duty_pct / 100.0 * 100000)
    data4 = struct.pack(">i", duty_val)
    tx_and_read(MOTOR_ID, data4, ext=True, label=f"4-byte duty {duty_pct}%")
    time.sleep(0.02)

# ── 4. Duty Cycle - 8 bytes (our current format) ──
print("\n[4] DUTY CYCLE - 8 bytes (padded)")
for duty_pct in [10, 30, 50, 80]:
    duty_val = int(duty_pct / 100.0 * 100000)
    data8 = struct.pack(">i", duty_val) + bytes(4)
    tx_and_read(MOTOR_ID, data8, ext=True, label=f"8-byte duty {duty_pct}%")
    time.sleep(0.02)

# ── 5. Sustained duty at 50Hz for 2 seconds ──
print("\n[5] SUSTAINED 80% DUTY for 2s at 50Hz")
drain()
duty_80 = struct.pack(">i", 80000) + bytes(4)
t0 = time.time()
fb_count = 0
last_print = 0
while time.time() - t0 < 2.0:
    bus.send(can.Message(arbitration_id=MOTOR_ID, data=duty_80, is_extended_id=True))
    deadline = time.time() + 0.02
    while time.time() < deadline:
        r = bus.recv(timeout=0.005)
        if r and r.arbitration_id == FEEDBACK_ID:
            fb_count += 1
            elapsed = time.time() - t0
            if elapsed - last_print >= 0.5:
                pos, spd, cur, tmp, err = parse_feedback(r.data)
                print(f"  t={elapsed:.1f}s pos={pos:7.1f}° spd={spd:+7d} cur={cur:5.2f}A tmp={tmp}°C err={err}")
                last_print = elapsed
print(f"  Total feedbacks: {fb_count}")

# ── 6. Current mode ──
print("\n[6] CURRENT MODE (2A)")
CURRENT_ARB = (0x01 << 8) | MOTOR_ID  # 0x0103
cur_data = struct.pack(">i", 2000)  # 2A * 1000
tx_and_read(CURRENT_ARB, cur_data, ext=True, label="Current 2A (4 byte)", duration=0.3)
tx_and_read(CURRENT_ARB, cur_data + bytes(4), ext=True, label="Current 2A (8 byte)", duration=0.3)

# ── 7. Velocity mode ──
print("\n[7] VELOCITY MODE (1000 ERPM)")
VEL_ARB = (0x03 << 8) | MOTOR_ID  # 0x0303
vel_data = struct.pack(">i", 1000)  # 1000 ERPM
tx_and_read(VEL_ARB, vel_data, ext=True, label="Velocity 1000 ERPM (4 byte)", duration=0.3)
tx_and_read(VEL_ARB, vel_data + bytes(4), ext=True, label="Velocity 1000 ERPM (8 byte)", duration=0.3)

# ── 8. Disable ──
print("\n[8] DISABLE")
tx_and_read(MOTOR_ID, DISABLE, ext=True, label="Disable")

bus.shutdown()
print("\nDONE")
