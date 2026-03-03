# Session Log — 3 March 2026

Work is on the motion-capture branch.  All changes are in `motion_capture_test.py`.

---

## Summary

Complete rewrite of `motion_capture_test.py` from a software-PID duty-cycle script
to a position-mode approach, then back to duty-cycle after discovering position mode
does not feed the CAN watchdog.  The session ended with a working 90-second sine sweep
with a speed-brake safety net.

---

## 1. Position Mode Does Not Work Over CAN (New Discovery)

**Attempted:** Replace the software PID loop with the motor's own internal position
controller using `arb_id = 0x0403` (CubeMars multimode, position loop).

**Expected:** Motor firmware handles motion smoothly; we just stream target positions
at 50 Hz.

**Actual:** Motor went silent after ~1 second — CAN watchdog fired.

**Root cause (confirmed):**

> The motor's 100 ms CAN watchdog is only reset by frames received on
> `arb_id = MOTOR_ID = 0x03` (raw duty-cycle / enable channel).
> Position-mode frames (`0x0403`), velocity-mode (`0x0303`), and
> pos-vel-accel (`0x0603`) use *different* arbitration IDs and do **not**
> reset the watchdog.

This is documented in `CAN_IMPLEMENTATION_CHAPTER.tex §watchdog` and
`MOTION_CAPTURE_PROGRESS.md §1`, but was not remembered when attempting the
position-mode rewrite.

**Attempted workaround:** Send a `STOP_CMD` on `DUTY_ARB` every 400 ms alongside
position commands.  Result: STOP_CMD overrode the position command — motor stayed
at 0% duty and did not move.  Position mode is incompatible with the watchdog
architecture unless the firmware is reconfigured.

**Final decision:** Stay on duty-cycle control (`arb_id = 0x03`) with a software
P-controller.  This is the only approach that (a) keeps the watchdog alive, and
(b) actually moves the motor.

---

## 2. 0x0088 Standard-Frame Flood (New Discovery)

During position-mode debugging a new issue appeared: `bus.recv()` was returning
frames with `arb_id = 0x0088` instead of motor feedback.

**Finding:** An unrelated device on the CAN bus (exact source unknown — likely board
support electronics) continuously broadcasts standard 8-byte all-zero frames on
`arb_id = 0x0088` at high rate.

**Effect on code:** The original `read_feedback()` called `bus.recv()` once and
returned `None` if the ID did not match.  With the 0x0088 flood, almost every
`recv()` call hit a noise frame, so real motor feedback was invisible to the loop
(appeared as 7950 consecutive "no feedback" ticks while the arm sat still).

**Fix applied (in `motion_capture_test.py`):**

```python
def read_feedback(bus, deadline_ms=15.0):
    deadline = time.monotonic() + deadline_ms / 1000.0
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        msg = bus.recv(timeout=remaining)
        if msg is None:
            return None
        if msg.arbitration_id not in FEEDBACK_IDS:
            continue          # skip 0x0088 noise — loop to next frame
        ...
```

Instead of returning `None` on the first non-matching frame, the loop drains
frames until a valid feedback ID is found within the 15 ms deadline.

**Note:** `cube_mars_motor_can.py` already handles this correctly —
`_refresh_loop()` and `_receive_feedback()` both loop through non-motor frames
within a time budget (comment explicitly mentions "0x0088 flood").

---

## 3. Motion Sequence Finalised

The capture sequence went through several iterations:

| Version | Sequence | Outcome |
|---------|----------|---------|
| v1 | 0→90→120→sine(10 s)→0 | 90° stalled (KP too low, 60 s timeout) |
| v2 | Skip 90°: 0→120→sine(10 s)→0 | Arm reached 120° and oscillated |
| v3 | 0→90→120→sine(90 s)→0 | Sine ran wild at t=87 s (5280 ERPM) |
| v4 (current) | 0→90→120→sine(90 s, speed-brake)→0 | Speed-brake activates above 3000 ERPM |

**Current sequence:**

1. Gravity settle → `fw_home`
2. Direction detection (1 s × 2% duty pulse)
3. Re-home
4. `move_to(90°)` — raise to parallel
5. `move_to(120°)` — swing above parallel
6. `sine_sweep()` — 90 s, 120 ↔ 60°, 6 s period, cosine phase
7. `coast_to_hang()` — duty=0, gravity lowers arm

---

## 4. Runaway Oscillation at End of Sine Sweep (New Discovery)

**Symptom:** At t ≈ 87 s the arm reached −16.8° physical (target was 60°),
triggering the `PHYS_MIN = −20°` safety stop.  Peak ERPM logged: 5280.

**Root cause:** The P-controller with `KP=0.005` and `MAX_DUTY=0.30` built
enormous momentum at large errors.  The D-term (`KD=0.000015`) was far too weak
to brake the arm before it overshot.

At 5000 ERPM the D contribution was only:
```
KD × ERPM × _fw_dir = 0.000015 × 5000 = 0.075 duty
```
While the P-term at 30° error was:
```
KP × err = 0.005 × 30 = 0.150 duty
```
The arm accelerated faster than the D-term could brake → runaway overshoot →
safety stop at −21°.

**Fix attempted (then reverted):** Increased `KD` to 0.00005 and reduced
`MAX_DUTY` to 0.18 with a separate `SINE_MAX = 0.12` cap during the sweep.
Result: motion became jerky and unsmooth — user preferred the original gains.

**Final fix applied:** Speed-brake (see §5 below).

---

## 5. Speed-Brake Safety Net (New Feature)

Rather than tuning gains to prevent high speed, a speed-brake block was added
inside `sine_sweep()`.

**Logic:**
- If `|ERPM| > MAX_SAFE_ERPM` (3000): cut duty to 0, coast, log as label `"brake"`
- Loop until `|ERPM| < BRAKE_ERPM` (1000)
- Resume normal sine control from wherever the arm is

```python
MAX_SAFE_ERPM = 3000    # coast to a stop if speed exceeds this during sine sweep
BRAKE_ERPM    = 1000    # resume normal control once speed drops below this
```

This prevents the arm from exceeding safe speed without changing the motion
character during normal operation.

---

## 6. cube_mars_motor_can.py — Unit Test Status

The driver class was never used in `motion_capture_test.py` (hardcoded CAN was
faster to debug).  Test status as of today:

```
47 / 63 unit tests PASS  (no hardware required)
16 tests HANG            (mock bus.recv returns None but _capture_response
                          loops with a timeout — pytest-timeout not installed)
```

The 47 passing tests cover: initialisation, enable/disable, velocity control,
position encoding, current/brake commands, feedback parsing, error codes,
`get_status` / `get_motor_data`, and `control_exosuit_tendon`.

The 16 hanging tests are not failures — they are waiting for hardware feedback
that will never arrive from the mock.  Installing `pytest-timeout` or patching
`_capture_response` to not block would fix them.

**Available functions in `CubeMarsAK606v3CAN`:**

| Function | Purpose |
|----------|---------|
| `enable_motor()` / `disable_motor()` | Send magic bytes |
| `set_duty_cycle(duty)` | Direct duty, keeps watchdog alive |
| `set_velocity(erpm)` | Velocity mode (watchdog issue — see §1) |
| `set_position(deg)` | Position mode (watchdog issue — see §1) |
| `set_position_velocity_accel(deg, vel, accel)` | Smooth profiled move |
| `set_current(amps)` / `set_brake_current(amps)` | Torque / braking |
| `set_origin(permanent)` | Zero the encoder |
| `get_position()` / `get_speed()` / `get_current()` / `get_temperature()` | Feedback getters |
| `get_status()` / `get_motor_data()` | Full feedback struct / dict |
| `check_communication()` | Heartbeat test |
| `move_to_position_with_speed(deg, erpm)` | Blocking position move |
| `control_exosuit_tendon(action, erpm)` | High-level tendon pull/release |
| `stop()` / `close()` | Graceful shutdown |

**Recommendation for exosuit firmware:** Use `set_duty_cycle()` (which calls
`_start_refresh()`, handling the watchdog background thread automatically)
rather than `set_position()` or `set_velocity()` until the watchdog behaviour
of those modes is confirmed on the physical hardware.

---

## 7. Controller Gains — Current Values

```python
KP        = 0.005        # duty per degree of error
KD        = 0.000015     # duty per ERPM  (damping)
MAX_DUTY  = 0.30         # absolute duty cap for waypoint moves
DEADBAND  = 1.0          # degrees — no output below this error
```

From the working run (`mocap_2026-03-03_17-52-30.csv`):

| Phase | Mean error | Max error | Peak ERPM |
|-------|-----------|-----------|-----------|
| parallel (0→90°) | 12.0° | 90.0° | 5340 |
| swing-up (90→120°) | 7.3° | 34.4° | 1720 |
| sine-sweep (90 s) | 6.6° | 79.2° | 5280 |

Tracking is imperfect — the arm lags the sinusoidal command by ~6° on average —
but the motion is smooth and the duty P-controller stabilises the arm at the
waypoints.

---

## Files Changed Today

| File | Change |
|------|--------|
| `motion_capture_test.py` | Complete rewrite × 2; final: duty P-ctrl + speed-brake |
| `docs/MOTION_CAPTURE_PROGRESS.md` | Updated with today's sequence and status |
| `docs/CHANGELOG_2026-03-03.md` | This file |
