# ruff: noqa: T201  (print is intentional in this diagnostic script)
"""Quick test to check motor feedback.

Run this after: sudo ./setup_can.sh && .venv/bin/python scripts/quick_test.py
Make sure the UART cable is DISCONNECTED from the motor first.
"""

import argparse
import struct
import time

import can

from motor_python import create_can_motor


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Quick CAN feedback diagnostic")
    parser.add_argument(
        "--interface",
        default="can0",
        help="SocketCAN interface",
    )
    parser.add_argument(
        "--motor-id",
        type=lambda value: int(value, 0),
        default=0x03,
        help="Motor CAN ID in decimal or hex (default: 0x03)",
    )
    parser.add_argument(
        "--motor-model",
        choices=("AK60-6", "AK80-6"),
        default="AK60-6",
        help="Motor model to instantiate (default: AK60-6)",
    )
    return parser.parse_args()


args = parse_args()

print("=" * 60)
print("CAN Motor Feedback Diagnostic")
print(f"Motor model: {args.motor_model}")
print(f"Motor CAN ID: 0x{args.motor_id:02X}  |  Interface: {args.interface}")
print("UART cable must be DISCONNECTED for CAN control")
print("=" * 60)

# ── Step 1: enable motor then sniff ALL 8-byte frames for 3 s ──
print("\n[1] Sending enable command, then listening (3 s) for ALL 8-byte frames...")
bus = can.interface.Bus(channel=args.interface, interface="socketcan")

# AK80-6 uses standard 11-bit frames; AK60-6 uses extended 29-bit frames
is_extended_id = True if args.motor_model != "AK80-6" else False

enable_msg = can.Message(
    arbitration_id=0x03,
    data=bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]),
    is_extended_id=is_extended_id,
)
bus.send(enable_msg)
print(f"  TX enable → 0x{enable_msg.arbitration_id:08X}  FF FF FF FF FF FF FF FC")

seen: dict[int, dict] = {}
deadline = time.time() + 3.0
while time.time() < deadline:
    msg = bus.recv(timeout=0.05)
    if msg and len(msg.data) == 8:
        aid = msg.arbitration_id
        if aid not in seen:
            seen[aid] = {"count": 0, "data": bytes(msg.data), "ext": msg.is_extended_id}
        seen[aid]["count"] += 1
        seen[aid]["data"] = bytes(msg.data)

# Confirmed background noise IDs — NEVER motor feedback (Session 2 verified)
BACKGROUND_IDS = {
    0x0004: "IMU/sensor (not motor)",
    0x0040: "heartbeat (not motor)",
    0x0088: "background device — proven NOT motor (data never changes with commands)",
    0x0100: "heartbeat (not motor)",
}
# Confirmed motor feedback ID (Session 2 verified, EXT 29-bit frame)
MOTOR_FEEDBACK_ID = 0x2900 | args.motor_id

print()
if seen:
    print("  Frames captured (8-byte only):")
    for aid, info in sorted(seen.items()):
        d = info["data"]
        data_hex = " ".join(f"{b:02X}" for b in d)
        ext = "EXT" if info["ext"] else "STD"
        rate_note = f"~{info['count']/3:.0f} Hz"
        if aid == MOTOR_FEEDBACK_ID:
            pos = struct.unpack(">h", d[0:2])[0] * 0.1
            spd = struct.unpack(">h", d[2:4])[0] * 10
            cur = struct.unpack(">h", d[4:6])[0] * 0.01
            tmp = struct.unpack("b", bytes([d[6]]))[0]
            err = d[7]
            print(f"    {ext} 0x{aid:08X} [{data_hex}]  x{info['count']} ({rate_note})  *** MOTOR FEEDBACK ***")
            print(f"       pos={pos:.1f}deg  spd={spd} ERPM  cur={cur:.2f}A  tmp={tmp}C  err={err}")
        elif aid in BACKGROUND_IDS:
            print(f"    {ext} 0x{aid:08X} [{data_hex}]  x{info['count']} ({rate_note})  [BACKGROUND: {BACKGROUND_IDS[aid]}]")
        else:
            pos = struct.unpack(">h", d[0:2])[0] * 0.1
            spd = struct.unpack(">h", d[2:4])[0] * 10
            cur = struct.unpack(">h", d[4:6])[0] * 0.01
            tmp = struct.unpack("b", bytes([d[6]]))[0]
            err = d[7]
            print(f"    {ext} 0x{aid:08X} [{data_hex}]  x{info['count']} ({rate_note})  [UNKNOWN]")
            print(f"       if-feedback: pos={pos:.1f}deg spd={spd} ERPM cur={cur:.2f}A tmp={tmp}C err={err}")
else:
    print("  *** No frames received — motor may be off or UART still connected ***")

bus.shutdown()

# ── Step 2: try with our CAN class (send command, check cached response) ──
print(f"\n[2] Testing with {args.motor_model} class (ID=0x{args.motor_id:02X})...")
print("    Motor is response-only: feedback comes as reply to each command.")
motor = create_can_motor(
    args.motor_model,
    motor_can_id=args.motor_id,
    interface=args.interface,
)
motor.enable_motor()          # sends enable → auto-captures 0x2903 response

got = 0
# _pending_feedback is already populated by enable_motor's _capture_response.
# Calling _receive_feedback() returns it immediately from cache.
for i in range(5):
    feedback = motor._receive_feedback(timeout=0.1)
    if feedback:
        got += 1
        if got == 1:
            print("  ✓ Feedback received!")
            print(f"    Position: {feedback.position_degrees:.2f}°")
            print(f"    Speed:    {feedback.speed_erpm} ERPM")
            print(f"    Current:  {feedback.current_amps:.2f} A")
            print(f"    Temp:     {feedback.temperature_celsius}°C")
            print(f"    Error:    {feedback.error_code}")
    else:
        # Re-trigger with explicit MIT keepalive.
        motor.send_keepalive()
        print(f"  Attempt {i+1}/5: sent MIT keepalive, waiting for response...")

if got == 0:
    print("\n  *** DIAGNOSIS: no feedback received ***")
    print("  Motor is in response-only mode — it replies to commands, not broadcasts.")
    print("  Possible causes:")
    print("    1. Interface not reset since last run — try: sudo ./setup_can.sh")
    print("    2. CAN wiring issue (CANH/CANL or termination)")
    print("    3. Motor lost power")

motor.close()
print("\nDone.")
