#!/usr/bin/env python3
"""CAN reliability diagnostic — pinpoints why motor connection is intermittent.

Tests each phase of the connection sequence independently:
  1. CAN bus state (ERROR-ACTIVE vs PASSIVE vs BUS-OFF)
  2. Raw recv with NO filters (do we see 0x2903 frames at all?)
  3. Raw recv WITH our exact filters (do filters block the frames?)
  4. Enable command → response (does motor ACK and reply?)
  5. Multiple rapid enable/get_status cycles (is it a timing race?)

Run:
    python diagnose_can.py
"""

import subprocess
import sys
import time

import can

# ── Config ──────────────────────────────────────────────────────────────
MOTOR_ID  = 0x03
INTERFACE = "can0"
ENABLE_FRAME = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
FEEDBACK_IDS = {0x03, 0x04, 0x2903, 0x0083, 0x2900}


def get_can_state() -> dict:
    """Parse CAN interface state from ip command."""
    result = subprocess.run(
        ["ip", "-details", "-statistics", "link", "show", INTERFACE],
        capture_output=True, text=True, timeout=5,
    )
    output = result.stdout
    state = "UNKNOWN"
    tx_err = rx_err = 0
    for line in output.splitlines():
        if "state" in line and "berr-counter" in line:
            for part in line.split():
                if part in ("ERROR-ACTIVE", "ERROR-PASSIVE", "ERROR-WARNING", "BUS-OFF"):
                    state = part
            if "tx" in line:
                idx = line.index("tx")
                tx_err = int(line[idx:].split()[1])
            if "rx" in line:
                idx = line.index("rx")
                rx_err = int(line[idx:].split()[1].rstrip(")"))
    return {"state": state, "tx_err": tx_err, "rx_err": rx_err}


def test_raw_recv_no_filter(duration: float = 2.0) -> dict:
    """Open a raw CAN socket with NO hardware filters — see everything on the bus."""
    print(f"\n{'─'*60}")
    print(f"Test 1: Raw recv, NO filters ({duration}s)")
    print(f"{'─'*60}")
    bus = can.interface.Bus(channel=INTERFACE, interface="socketcan")
    frames = {}  # {arb_id: count}
    first_motor_at = None
    t0 = time.monotonic()
    deadline = t0 + duration
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg:
            frames[msg.arbitration_id] = frames.get(msg.arbitration_id, 0) + 1
            if msg.arbitration_id in FEEDBACK_IDS and first_motor_at is None:
                first_motor_at = time.monotonic() - t0
    bus.shutdown()

    total = sum(frames.values())
    motor_frames = sum(cnt for aid, cnt in frames.items() if aid in FEEDBACK_IDS)
    flood_0088 = frames.get(0x88, 0)
    print(f"  Total frames received : {total}")
    print(f"  Motor feedback frames : {motor_frames}  (IDs: {[hex(k) for k in sorted(frames) if k in FEEDBACK_IDS]})")
    print(f"  0x0088 flood frames   : {flood_0088}")
    print(f"  Other frame IDs       : {[hex(k) for k in sorted(frames) if k not in FEEDBACK_IDS and k != 0x88]}")
    if first_motor_at is not None:
        print(f"  First motor frame at  : {first_motor_at*1000:.0f} ms")
    else:
        print(f"  ⚠  NO motor frames received in {duration}s!")
    return {"total": total, "motor": motor_frames, "flood_0088": flood_0088,
            "first_motor_ms": first_motor_at * 1000 if first_motor_at else None}


def test_raw_recv_with_filter(duration: float = 2.0) -> dict:
    """Open CAN socket with the UPDATED kernel filters matching the driver."""
    print(f"\n{'─'*60}")
    print(f"Test 2: Raw recv, WITH driver filters ({duration}s)")
    print(f"{'─'*60}")
    # These must mirror CubeMarsAK606v3CAN._connect() exactly:
    can_filters = [
        {"can_id": 0x2903, "can_mask": 0x1FFFFFFF, "extended": True},
        {"can_id": 0x2900, "can_mask": 0x1FFFFFFF, "extended": True},
        {"can_id": MOTOR_ID + 1, "can_mask": 0x1FFFFFFF, "extended": True},
        {"can_id": MOTOR_ID, "can_mask": 0x1FFFFFFF, "extended": True},
        {"can_id": 0x0080 | MOTOR_ID, "can_mask": 0x1FFFFFFF, "extended": True},
        {"can_id": MOTOR_ID + 1, "can_mask": 0x7FF, "extended": False},
    ]
    bus = can.interface.Bus(channel=INTERFACE, interface="socketcan",
                            can_filters=can_filters)
    frames = {}
    first_motor_at = None
    t0 = time.monotonic()
    deadline = t0 + duration
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg:
            frames[msg.arbitration_id] = frames.get(msg.arbitration_id, 0) + 1
            if first_motor_at is None:
                first_motor_at = time.monotonic() - t0
    bus.shutdown()

    total = sum(frames.values())
    print(f"  Total frames received : {total}")
    print(f"  Frame IDs             : {[(hex(k), v) for k, v in sorted(frames.items())]}")
    if first_motor_at is not None:
        print(f"  First frame at        : {first_motor_at*1000:.0f} ms")
    else:
        print(f"  ⚠  NO frames passed the filter in {duration}s!")
    return {"total": total, "first_ms": first_motor_at * 1000 if first_motor_at else None}


def test_enable_and_response(attempts: int = 5) -> dict:
    """Send enable commands and measure response timing."""
    print(f"\n{'─'*60}")
    print(f"Test 3: Enable → response timing ({attempts} attempts)")
    print(f"{'─'*60}")
    # Use updated driver filters:
    can_filters = [
        {"can_id": 0x2903, "can_mask": 0x1FFFFFFF, "extended": True},
        {"can_id": 0x2900, "can_mask": 0x1FFFFFFF, "extended": True},
        {"can_id": MOTOR_ID + 1, "can_mask": 0x1FFFFFFF, "extended": True},
        {"can_id": MOTOR_ID, "can_mask": 0x1FFFFFFF, "extended": True},
        {"can_id": 0x0080 | MOTOR_ID, "can_mask": 0x1FFFFFFF, "extended": True},
        {"can_id": MOTOR_ID + 1, "can_mask": 0x7FF, "extended": False},
    ]
    bus = can.interface.Bus(channel=INTERFACE, interface="socketcan",
                            can_filters=can_filters)
    results = []
    for i in range(attempts):
        # Drain any pending frames
        while bus.recv(timeout=0.01):
            pass

        msg = can.Message(arbitration_id=MOTOR_ID, data=ENABLE_FRAME, is_extended_id=True)
        t_send = time.monotonic()
        bus.send(msg)

        # Wait for response
        reply = None
        deadline = t_send + 1.0  # 1s timeout
        while time.monotonic() < deadline:
            m = bus.recv(timeout=0.1)
            if m and m.arbitration_id in FEEDBACK_IDS:
                reply = m
                break

        if reply:
            latency_ms = (time.monotonic() - t_send) * 1000
            pos = int.from_bytes(reply.data[0:2], 'big', signed=True) * 0.1
            results.append({"ok": True, "latency_ms": latency_ms, "pos": pos})
            print(f"  [{i+1}] ✓  Response in {latency_ms:.1f} ms  pos={pos:.1f}°  "
                  f"id=0x{reply.arbitration_id:04X}")
        else:
            results.append({"ok": False, "latency_ms": None, "pos": None})
            print(f"  [{i+1}] ✗  No response within 1s")
        time.sleep(0.3)

    bus.shutdown()
    ok_count = sum(1 for r in results if r["ok"])
    print(f"\n  Success rate: {ok_count}/{attempts}")
    if ok_count > 0:
        latencies = [r["latency_ms"] for r in results if r["ok"]]
        print(f"  Latency: min={min(latencies):.1f}ms  max={max(latencies):.1f}ms  "
              f"avg={sum(latencies)/len(latencies):.1f}ms")
    return {"success_rate": ok_count / attempts, "results": results}


def test_get_status_loop(count: int = 10) -> dict:
    """Emulate the motion_capture_test enable+get_status loop."""
    print(f"\n{'─'*60}")
    print(f"Test 4: Full driver enable + get_status ({count} attempts)")
    print(f"{'─'*60}")
    sys.path.insert(0, "src")
    from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

    results = []
    for i in range(count):
        try:
            motor = CubeMarsAK606v3CAN(motor_can_id=MOTOR_ID, interface=INTERFACE)
            motor.enable_motor()
            time.sleep(0.3)

            fb = motor.get_status()
            if fb:
                results.append({"ok": True, "pos": fb.position_degrees})
                print(f"  [{i+1}] ✓  pos={fb.position_degrees:.1f}°  "
                      f"temp={fb.temperature_celsius}°C")
            else:
                results.append({"ok": False, "pos": None})
                print(f"  [{i+1}] ✗  get_status() returned None")
            motor.disable_motor()
            motor.close()
        except Exception as e:
            results.append({"ok": False, "pos": None})
            print(f"  [{i+1}] ✗  Exception: {e}")
        time.sleep(0.5)

    ok_count = sum(1 for r in results if r["ok"])
    print(f"\n  Success rate: {ok_count}/{count}")
    return {"success_rate": ok_count / count}


def main():
    print("=" * 60)
    print("  CAN Reliability Diagnostic")
    print("=" * 60)

    # Phase 0: CAN bus state
    state = get_can_state()
    print(f"\n  CAN state: {state['state']}  (tx_err={state['tx_err']} rx_err={state['rx_err']})")
    if state["state"] != "ERROR-ACTIVE":
        print(f"  ⚠  Bus is NOT ERROR-ACTIVE — resetting...")
        subprocess.run(["sudo", "ip", "link", "set", INTERFACE, "down"], timeout=5)
        subprocess.run([
            "sudo", "ip", "link", "set", INTERFACE, "up",
            "type", "can", "bitrate", "1000000",
            "berr-reporting", "on", "restart-ms", "100",
        ], timeout=5)
        time.sleep(0.5)
        state = get_can_state()
        print(f"  After reset: {state['state']}  (tx_err={state['tx_err']} rx_err={state['rx_err']})")

    # Phase 1: Raw recv
    test1 = test_raw_recv_no_filter()

    # Phase 2: Filtered recv
    test2 = test_raw_recv_with_filter()

    # Phase 3: Enable + response
    test3 = test_enable_and_response()

    # Phase 4: Full driver cycle
    test4 = test_get_status_loop()

    # Summary
    print(f"\n{'=' * 60}")
    print("  SUMMARY")
    print(f"{'=' * 60}")
    print(f"  Bus state             : {state['state']}")
    print(f"  Raw motor frames      : {test1['motor']} in 2s  "
          f"({'broadcasting' if test1['motor'] > 0 else 'response-only (normal)'})")
    print(f"  Filtered frames       : {test2['total']} in 2s  "
          f"({'broadcasting' if test2['total'] > 0 else 'response-only (normal)'})")
    print(f"  Enable→response       : {test3['success_rate']*100:.0f}%")
    print(f"  Full driver get_status: {test4['success_rate']*100:.0f}%")

    # Diagnosis logic — the motor is response-only so passive tests (1 & 2)
    # returning 0 frames is EXPECTED.  The real reliability indicators are
    # Tests 3 and 4.
    if test3['success_rate'] == 0:
        print("\n  ROOT CAUSE: Motor not responding to commands at all.")
        print("  → Check: motor powered? UART cable connected? (UART blocks CAN)")
    elif test3['success_rate'] < 1.0:
        print("\n  ROOT CAUSE: Enable command sent but motor response is intermittent.")
        print("  → Timing race in _capture_response() or bus error state.")
    elif test4['success_rate'] < 1.0:
        print("\n  ROOT CAUSE: Driver code issue in enable → get_status path.")
        print("  → Check _capture_response timeout, _pending_feedback logic.")
    else:
        print("\n  ✓  All tests passed — CAN is working reliably right now.")
        if test1['motor'] == 0:
            print("  ℹ  Motor is in response-only mode (no autonomous broadcast).")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
