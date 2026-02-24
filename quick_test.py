# ruff: noqa: T201  (print is intentional in this diagnostic script)
"""Quick test to check motor feedback.

Run this after: sudo ./setup_can.sh
Make sure the UART cable is DISCONNECTED from the motor first.
"""

import struct
import time

import can

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

print("=" * 60)
print("CAN Motor Feedback Diagnostic")
print("Motor CAN ID: 0x03  |  Interface: can0")
print("UART cable must be DISCONNECTED for CAN control")
print("=" * 60)

# ── Step 1: enable motor then sniff ALL 8-byte frames for 3 s ──
print("\n[1] Sending enable command, then listening (3 s) for ALL 8-byte frames...")
bus = can.interface.Bus(channel="can0", interface="socketcan")

enable_msg = can.Message(
    arbitration_id=0x03,
    data=bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]),
    is_extended_id=True,
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
MOTOR_FEEDBACK_ID = 0x2903

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
print("\n[2] Testing with CubeMarsAK606v3CAN (ID=0x03)...")
print("    Motor is response-only: feedback comes as reply to each command.")
motor = CubeMarsAK606v3CAN(motor_can_id=0x03)
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
        # Re-trigger: send a zero-current command to get another response
        motor.set_current(0.0)
        print(f"  Attempt {i+1}/5: sent heartbeat command, waiting for response...")

if got == 0:
    print("\n  *** DIAGNOSIS: no feedback received ***")
    print("  Motor is in response-only mode — it replies to commands, not broadcasts.")
    print("  Possible causes:")
    print("    1. Interface not reset since last run — try: sudo ./setup_can.sh")
    print("    2. CAN wiring issue (CANH/CANL or termination)")
    print("    3. Motor lost power")

motor.close()
print("\nDone.")
