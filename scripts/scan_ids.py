#!/usr/bin/env python3
"""Scan CAN IDs 1-10 to find which motor ID the AK60-6 responds on."""
import sys
import time

import can

sys.stdout.reconfigure(line_buffering=True)  # flush each line

bus = can.Bus(channel="can0", interface="socketcan")

# Flush stale (limited iterations to avoid hang)
flushed = 0
for _ in range(50):
    r = bus.recv(timeout=0.02)
    if r is None:
        break
    flushed += 1
print(f"Flushed {flushed} stale frames")

ENABLE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
DISABLE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])

def drain(bus, max_iter=50):
    for _ in range(max_iter):
        if bus.recv(timeout=0.02) is None:
            break

print("\n=== EXTENDED FRAME SCAN (motor IDs 1-10) ===")
for motor_id in range(1, 11):
    drain(bus)

    msg = can.Message(arbitration_id=motor_id, data=ENABLE, is_extended_id=True)
    bus.send(msg)

    replies = []
    deadline = time.time() + 0.15
    while time.time() < deadline:
        r = bus.recv(timeout=0.03)
        if r:
            replies.append(r)

    # Send disable too
    msg = can.Message(arbitration_id=motor_id, data=DISABLE, is_extended_id=True)
    bus.send(msg)
    drain(bus)

    if replies:
        motor_replies = [r for r in replies if r.arbitration_id == motor_id + 1]
        other_replies = [r for r in replies if r.arbitration_id != motor_id + 1]
        print(f"  ID {motor_id}: {len(motor_replies)} motor replies, {len(other_replies)} other")
        for r in replies:
            print(f"    arb=0x{r.arbitration_id:04X} ext={r.is_extended_id} data={r.data.hex()}")
    else:
        print(f"  ID {motor_id}: NO replies")

print("\n=== STANDARD FRAME SCAN (motor IDs 1-10) ===")
for motor_id in range(1, 11):
    drain(bus)

    msg = can.Message(arbitration_id=motor_id, data=ENABLE, is_extended_id=False)
    bus.send(msg)

    replies = []
    deadline = time.time() + 0.15
    while time.time() < deadline:
        r = bus.recv(timeout=0.03)
        if r:
            replies.append(r)

    msg = can.Message(arbitration_id=motor_id, data=DISABLE, is_extended_id=False)
    bus.send(msg)
    drain(bus)

    if replies:
        print(f"  ID {motor_id} (STD): {len(replies)} replies")
        for r in replies:
            print(f"    arb=0x{r.arbitration_id:04X} ext={r.is_extended_id} data={r.data.hex()}")
    else:
        print(f"  ID {motor_id} (STD): NO replies")

bus.shutdown()
print("\nSCAN COMPLETE")
