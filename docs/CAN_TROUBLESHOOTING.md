# CAN Motor Troubleshooting Log

Chronological record of what was tried, what failed, and **why**, so we don't repeat it.

> **Important (current state):** This file contains historical troubleshooting
> from earlier servo/duty experiments. The active Python CAN implementation is
> now MIT-only (`mode_id=0x08`). For current bench steps, use:
> - `docs/CAN_TESTING.md`
> - `docs/CAN_TELEMETRY_API.md`
> - `scripts/mit_mode_test.py`

---

## Current Production Baseline (MIT-only)

Use this as the authoritative command baseline for current software:

| Action | Arb ID | Payload |
|---|---|---|
| Enable MIT | `motor_id` (for ID 3: `0x000003`) | `FF FF FF FF FF FF FF FF` |
| MIT control command | `(0x08 << 8) | motor_id` (for ID 3: `0x000803`) | 8-byte MIT-packed payload (`KP/KD/Pos/Vel/Tau`) |
| Disable MIT | `motor_id` (for ID 3: `0x000003`) | `FF FF FF FF FF FF FF FE` |
| Recommended bench script | N/A | `.venv/bin/python scripts/mit_mode_test.py --motor-id 0x03` |

---

## ✅ Historical Results (pre-MIT switch)

| Feature | Arb ID | Payload | Notes |
|---|---|---|---|
| Enable motor | `0x000003` | `FF FF FF FF FF FF FF FC` | Standard enable |
| Disable motor | `0x000003` | `FF FF FF FF FF FF FF FD` | Standard disable |
| Duty cycle | `0x000003` | `struct.pack('>i', int(duty*100_000)) + bytes(4)` | `spin_test.py` uses this |
| Velocity | `0x000303` | `struct.pack('>i', erpm) + bytes(4)` | ⚠️ **No feedback received** (see Problem 6) |
| Current | `0x000103` | `struct.pack('>i', mA) + bytes(4)` | ⚠️ **No feedback received** (see Problem 6) |
| Feedback reception | listen on `0x2903` | 8 bytes: pos/spd/cur/tmp/err | Works **only with CAN filter** (see below) |

---

## ❌ Tried and NOT Working

### 1. Position mode (0x04) — arb `0x000403`
- Payload: `struct.pack('>i', int(deg * 10000)) + bytes(4)`
- Motor acknowledges (sends feedback), but **position does not change**
- Tried absolute targets: -90°, -80°, 20° — no movement
- Status: likely requires firmware setting in CubeMars PC software — **not investigated yet**

### 2. Position+Velocity+Accel mode (0x06) — arb `0x000603`
- Payload: `struct.pack('>ihh', pos_int, vel_int, acc_int)` (int32 + int16 + int16)
- Motor acknowledges, **position does not change**
- Same symptom as mode 0x04
- Status: **not working**, root cause unknown

### 3. Set origin (0x05) — arb `0x000503`
- Payload: `bytes([0x01, 0, 0, 0, 0, 0, 0, 0])` (0x01 = temporary)
- Motor sends feedback after receiving it, but **position in feedback does not reset to 0**
- The -168.7° value persists in all subsequent feedback frames
- Status: **not working**, or maybe only takes effect on the next power cycle

---

## 🔍 Root Causes Discovered

### Problem 1 — `0x0088` flood swamps `bus.recv()`
**Symptom:** `_get_feedback()` always returns `None` even though the motor broadcasts 50 Hz feedback.

**Root cause:** An unknown device on the CAN bus (possibly the IMU) transmits standard 11-bit frames on ID `0x0088` at **~30,000 frames/second**. Every call to `bus.recv(timeout=...)` returns a `0x0088` frame before the motor's `0x2903` feedback arrives.

**Fix applied (confirmed working):**
```python
bus = can.interface.Bus(
    channel="can0",
    interface="socketcan",
    can_filters=[{"can_id": 0x2903, "can_mask": 0x1FFFFFFF, "extended": True}],
)
```
This drops `0x0088` in the kernel before it reaches userspace.

**Do NOT:**
- Open the bus without the filter
- Try to ignore `0x0088` in Python — the kernel socket buffer fills before you can drain it

---

### Problem 2 — UART cable connected silences CAN control
**Symptom:** Motor broadcasts 50 Hz feedback normally. All commands are ACK'd (no bus errors). But speed = 0, position never changes regardless of what is sent.

**Root cause:** The CubeMars motor has two interfaces: UART and CAN. **UART has higher priority.** When a UART cable is physically connected to the motor, it ignores ALL incoming CAN control commands while continuing to broadcast its status.

**How to detect:**
```bash
ip -details link show can0 | grep berr
# берр-counter tx 128+ and ERROR-PASSIVE = UART is connected (commands not ACK'd)
# berr-counter tx 0 and ERROR-ACTIVE = bus is clean
```

Also: after `set_origin`, if position does **not** reset to ~0° in feedback, UART is likely still plugged in.

**Fix:** Physically disconnect the UART cable from the motor, then:
```bash
sudo ./setup_can.sh   # resets ERROR-PASSIVE state
```

---

### Problem 3 — ERROR-PASSIVE state from accumulated TX errors
**Symptom:** `berr-counter tx 144` (or similar), `state ERROR-PASSIVE`. Motor feedback still arrives but no commands work.

**Root cause:** While UART was connected, every CAN command sent got no ACK from the motor, incrementing the TX error counter. At 128 the interface enters ERROR-PASSIVE (still transmits but motor ignores it).

**Fix:**
```bash
sudo ./setup_can.sh
# or manually:
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 berr-reporting on restart-ms 100
```

---

### Problem 4 — TX buffer overflow (`[Errno 105] No buffer space available`)
**Symptom:** `can.CanOperationError: Failed to transmit: No buffer space available`

**Root cause:** When feedback arrives fast (motor running), the inner loop sends commands faster than the kernel CAN socket can drain the TX queue.

**Fix applied:** Catch `CanOperationError` in `_tx()` and back off:
```python
def _tx(bus, arb_id, data):
    try:
        bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=True))
    except can.CanOperationError:
        time.sleep(0.02)
```

---

## 📋 Current Approach (motion_capture_test.py)

Since position modes (0x04, 0x06) do not work, the script uses:

**Velocity mode (0x03) + proportional controller:**
```
error = target_degrees - actual_degrees
velocity_erpm = clamp(KP * error, -MAX_SPEED, +MAX_SPEED)
```
- `KP = 150` ERPM/degree
- `TOLERANCE = 1.5°` — commands zero velocity once within this band
- Sends at ~50 Hz (one command per feedback frame received)
- CAN filter (`0x2903` only) applied at bus open

This is confirmed to make the motor move toward the target.

---

### Problem 5 — PSU under-voltage lockout (motor communicates but won't spin)

**Date discovered:** 9 March 2026

**Symptom:** Motor ACKs every CAN command, feedback reports `error_code=0`, but `Speed=0`, `Current=0` in ALL feedback frames regardless of control mode or duty level. PSU ammeter shows only ~35 mA quiescent draw (logic power only, no motor drive current).

**Root cause:** The PSU was set to **18.5V** instead of the required **24V**. The AK60-6 motor controller has a split power architecture:
- The MCU + CAN transceiver can operate at lower voltages (~12V+), so CAN communication appears completely normal.
- The MOSFET H-bridge that drives the windings has an **under-voltage lockout** that silently prevents motor drive below ~20V.
- The motor firmware does **NOT** report this as an error — `error_code` remains 0, and feedback frames look identical to a working motor at rest.

**How to detect:**
1. Check PSU voltage: must be **24V** (nominal range 24–48V for AK60-6).
2. Check PSU current draw: should be >0.3A when motor is enabled and commanded. If it stays at ~35 mA, the H-bridge is not driving.
3. Send an MIT command (`set_mit_mode` / `set_velocity`) and watch `speed_erpm` — if it stays at 0 after 0.5s, suspect under-voltage.

**Fix:**
1. Set PSU to **24V** (the AK60-6 nominal voltage).
2. **Power-cycle the motor** (turn PSU off, wait 3s, turn back on). The motor controller latches the under-voltage fault and does NOT auto-recover when voltage is raised — you must power-cycle.
3. Re-run `sudo ./setup_can.sh` to clear any accumulated CAN errors.
4. Verify with `.venv/bin/python scripts/mit_mode_test.py --motor-id 0x03`.

**Why this is insidious:**
- The motor communicates perfectly on CAN — all diagnostics look clean.
- `error_code=0` ("No fault") is reported in every feedback frame.
- The only clues are: (a) PSU ammeter showing ~35 mA instead of >300 mA, and (b) speed/current always stuck at 0.
- Software debugging is a dead-end because the protocol layer is working correctly.

**Pre-flight checklist added:** Always verify PSU voltage (24V) and current draw (>0.3A under command) before debugging CAN software.

---

### Problem 6 — Current mode and Velocity mode produce no feedback

**Date discovered:** 9 March 2026

**Symptom:** Sending commands on arb IDs `0x0103` (current mode) or `0x0303` (velocity mode) produces **zero feedback frames** on `0x2903`. Duty cycle mode (`0x0003`) works perfectly.

**Root cause:** Unknown — possibly a firmware configuration issue or the motor firmware version only supports duty-cycle CAN control. The motor may need additional configuration via the CubeMars PC software to enable current/velocity loop modes over CAN.

**Historical workaround (at discovery time):** duty-cycle mode (`0x0003`).

**Current status:** superseded by MIT-only Python implementation (`mode_id=0x08`).

---

## 🔜 Still To Investigate

1. **Why position mode (0x04) and PVA mode (0x06) don't respond** — may need a setting in CubeMars PC software to unlock position control (e.g. encoder calibration, control mode selection). Check the CubeMars configuration app.

2. **Why set_origin (0x05) doesn't reset the feedback position** — might only take effect after a power cycle, or might need to be sent in a specific motor state.

3. **P-controller overshoot / oscillation** — `KP=150` may cause oscillation on long moves. Tune or add a derivative term if needed.

4. **Why current mode (0x01) and velocity mode (0x03) produce no CAN feedback** — may need CubeMars PC software configuration.
