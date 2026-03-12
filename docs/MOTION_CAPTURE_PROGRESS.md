# Motion Capture Validation — Progress Log

**File:** `motion_capture_test.py`  
**Last updated:** 2026-03-03

---

## Goal

Move a cable-driven exosuit arm through waypoints **[0° → 45° → 65° → 30° → 0°]**, log motor feedback to CSV at 20 Hz, then compare against motion capture / IMU data for angle and velocity validation.

**Safety constraints:** No full rotation, single-side swing only, slow and smooth motion to protect cables and sensor attached to the arm.

---

## Hardware

| Item | Value |
|------|-------|
| Motor | CubeMars AK60-6 v3 |
| CAN ID | `0x03` |
| Interface | `can0`, 1 Mbps |
| Platform | Jetson Orin Nano (`aries-orin-1`), Python 3.12, `.venv` |
| Joint type | Cable-drive (tendon winding) |

---

## Key Technical Discoveries

### 1. CAN Watchdog (SOLVED)
The motor has a **1-second CAN watchdog** that disables servo mode if no frame is received on `arb_id=0x03`.

- Velocity mode (`0x0303`), position mode (`0x0403`), pos-vel (`0x0603`) use **different arb_ids** → watchdog fires → motor stops.
- **Fix:** Use raw duty-cycle frames on `arb_id=MOTOR_ID=0x03` only, at 50 Hz. This is what `spin_test.py` does and it keeps the motor alive.

### 2. Duty-Cycle Encoding
```python
raw = int(duty * 100_000)           # duty in [-1.0, 1.0]
payload = struct.pack(">i", raw) + bytes(4)   # 8 bytes, big-endian
```

### 3. Feedback Decoding
Motor broadcasts at `arb_id=0x2903` (and variants), 50 Hz:
```python
pos_deg = struct.unpack(">h", d[0:2])[0] * 0.1
erpm    = struct.unpack(">h", d[2:4])[0] * 10
cur_a   = struct.unpack(">h", d[4:6])[0] * 0.01
temp_c  = d[6]
err     = d[7]
```

### 4. Motor Direction (CONFIRMED, HARDCODED)
**Negative duty = forward (arm swings up), positive duty = backward (gravity wins).**

Evidence from live runs:
- `duty=-0.18`, `ERPM=+2940`, `fw: 51→116°` → arm went forward (phys 0→65°) ✓
- `duty=+0.18`, `ERPM=-3480`, `fw: 99→85°` → arm went backward ✓

**Why auto-calibration failed:**  
A brief +duty nudge gives `ERPM>0` because the motor spins freely before the cable goes taut. Once taut, sustained positive duty unwinds (lets gravity win). Sign is opposite in free-spin vs. cable-loaded regime → nudge-based calibration always reports the wrong direction.

**Convention in code:**
```python
MOTOR_DIRECTION = +1   # fw_pos increase = positive physical angle (forward)
_duty_sign      = -1   # positive PID output → negative duty → cable winds → forward
```
`phys_pos = (fw_pos - initial_fw_pos) * MOTOR_DIRECTION`  
`duty_cmd = clamp(pid_out, -MAX_DUTY, MAX_DUTY) * _duty_sign`

### 5. TX Buffer Overflow (SOLVED)
Sending STOP_CMD in a tight loop without rate-limiting fills the CAN TX queue → `CanOperationError`.  
Fix: `try/except CanOperationError` in `tx()` + minimum 5 ms sleep in no-feedback path.

---

## Current Configuration

```python
# Controller gains
KP            = 0.003   # duty per degree
KI            = 0.004   # duty per (deg·s)  — gravity compensation
KD            = 0.009   # duty per (deg/s)  — braking; limits speed to ~20 deg/s
MAX_DUTY      = 0.18    # enough torque to fight gravity; KD limits actual speed
MAX_INTEGRAL  = 0.12    # anti-windup

# Safety
ROT_GUARD_NEG = 15.0    # fw_delta below home → emergency stop
ROT_GUARD_POS = 200.0   # fw_delta above home → emergency stop
DEADBAND      = 1.5     # deg — zero duty + integral reset
TOLERANCE     = 2.0     # deg — "settled" band
HOLD_AFTER    = 1.5     # s — time inside TOLERANCE before advancing waypoint
HOLD_TIME     = 25.0    # s — max time per waypoint before skip
```

Speed theory: `v_steady ≈ MAX_DUTY / KD = 0.18 / 0.009 = 20 deg/s`

---

## Startup Sequence

1. `ENABLE_CMD` sent → wait for feedback
2. `calibrate_direction()` — prints hardcoded convention, no nudge
3. `find_home()` — sends `STOP_CMD` and waits for `|ERPM| < 200` for 1.5 s → records `initial_fw_pos`
4. Run waypoints via `move_and_sample()`

---

## Run History

| Run | KP | KI | KD | MAX_DUTY | Result |
|-----|----|----|----|---------|----|
| 1 | 0.001 | — | — | 0.05 | Stalled @24°, gravity too strong |
| 2 | 0.005 | — | — | 0.15 | Backward (wrong direction sign) |
| 3 | 0.010 | 0.002 | — | 0.55 | Forward but overshot 172° in <1 s |
| 4 | 0.010 | 0.002 | 0.003 | 0.30 | TX buffer crash |
| 5 | 0.003 | 0.0005 | 0.002 | 0.08 | Stalled @24° (not enough torque) |
| 6 | 0.005 | 0.003 | 0.002 | 0.18 | Backward immediately, safety stop |
| 7 | 0.003 | 0.004 | 0.002 | 0.18 | Forward ✓ but ~250 deg/s, ROT_GUARD abort |
| 8 | 0.003 | 0.004 | 0.009 | 0.18 | **Forward ✓, speed limited** — still going backward on some runs due to `_duty_sign` flip |
| **Current** | 0.003 | 0.004 | 0.009 | 0.18 | `_duty_sign=-1` hardcoded, cable-state check added — aborts cleanly when fw out of range, runs correctly when fw ∈ [30,140°] |

---

## Cable State — Critical Constraint

The duty→direction mapping is **only valid** when `fw_pos ∈ [30, 140]°`.

| fw_pos range | Cable state | Duty behaviour |
|---|---|---|
| 30–140° | Normal (taut, correct wrap) | negative = forward ✓ |
| < 30° | Slack / reverse-wound | direction undefined / reversed |
| > 140° | Over-wound | direction reversed |

**Root cause:** The cable spool changes effective wind direction as it crosses wrap boundaries. There is no safe auto-correct — the script now aborts with clear instructions if fw is out of range.

**Manual reset procedure (required before each new session):**
1. Power off the motor
2. Rotate shaft by hand until arm hangs at natural gravity rest
3. Power on — `fw_pos` should read 30–140°
4. Script will confirm and proceed, or print a cable state error

## Remaining Issues

- [ ] After many test runs cables wind up and fw drifts out of operating range — need manual reset between sessions.  
- [ ] ROT_GUARD_NEG=20° may still abort on return strokes if arm undershoots slightly. Consider loosening further if returns are consistently hitting it.
- [ ] Validate CSV data against motion capture / IMU angles once a full clean run is achieved.
- [ ] FW_HOME_MIN/MAX (30–140°) were set empirically from ~4 working runs — refine once more data collected.

---

## File Locations

| File | Purpose |
|------|---------|
| `motion_capture_test.py` | Main test script |
| `spin_test.py` | Reference: confirmed working 40% duty sweep |
| `data/logs/mocap_*.csv` | Output logs |
| `src/motor_python/cube_mars_motor_can.py` | Driver class (keepalive removed — not used by test) |
| `docs/MOTION_CAPTURE_PROGRESS.md` | This file |

---

## Run Command

```bash
cd /home/aries-orin-1/lower_exosuit/motor/motor-module
sudo ./setup_can.sh
.venv/bin/python motion_capture_test.py
```
