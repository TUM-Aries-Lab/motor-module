# CAN Motor Troubleshooting Log

Chronological record of what was tried, what failed, and **why**, so we don't repeat it.

---

## ✅ Confirmed Working

| Feature | Arb ID | Payload | Notes |
|---|---|---|---|
| Enable motor | `0x000003` | `FF FF FF FF FF FF FF FC` | Standard enable |
| Disable motor | `0x000003` | `FF FF FF FF FF FF FF FD` | Standard disable |
| Duty cycle | `0x000003` | `struct.pack('>i', int(duty*100_000)) + bytes(4)` | `spin_test.py` uses this |
| Velocity | `0x000303` | `struct.pack('>i', erpm) + bytes(4)` | Works **only with CAN filter** (see below) |
| Feedback reception | listen on `0x2903` | 8 bytes: pos/spd/cur/tmp/err | Works **only with CAN filter** |

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

## 🔜 Still To Investigate

1. **Why position mode (0x04) and PVA mode (0x06) don't respond** — may need a setting in CubeMars PC software to unlock position control (e.g. encoder calibration, control mode selection). Check the CubeMars configuration app.

2. **Why set_origin (0x05) doesn't reset the feedback position** — might only take effect after a power cycle, or might need to be sent in a specific motor state.

3. **P-controller overshoot / oscillation** — `KP=150` may cause oscillation on long moves. Tune or add a derivative term if needed.
