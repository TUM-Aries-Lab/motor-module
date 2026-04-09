# CAN Testing Scripts

Ad-hoc scripts for testing, diagnosing, and exercising the CubeMars AK60-6 motor over CAN.
These are **not** part of the main library or automated test suite — they are meant to be run
manually on the bench with the motor physically connected.

> **Current software status (2026-03-18): MIT-only CAN implementation.**
> For routine validation use `scripts/mit_mode_test.py` first.
> Duty/servo scripts in this folder are retained as historical diagnostics.

## Pre-conditions (all scripts)

```bash
# 1. Set PSU to 24 V and power-cycle the motor
# 2. Disconnect the UART / R-Link cable from the motor
# 3. Set up the CAN interface (run once per reboot)
sudo ./setup_can.sh

# 4. Activate the venv  — OR — prefix every script with .venv/bin/python scripts/<name>.py
source .venv/bin/activate
```

---

## Scripts

### `spin_test.py` (legacy)
Raw CAN demo — spins the motor forward for 2 s, pauses, then reverses for 2 s using
duty-cycle mode (arb_id 0x03). Prints live position/speed/current.

```bash
.venv/bin/python scripts/spin_test.py
```

---

### `motion_capture_test.py` (legacy)
Full motion-capture validation run — duty-cycle P-controller sine sweep (90 ± 30° at 6 s
period, 90 s total).  Logs telemetry to `data/logs/mocap_<timestamp>.csv`.

```bash
.venv/bin/python scripts/motion_capture_test.py
```

---

### `can_demo.py` (legacy exploration)
Exercises **every public method** of `CubeMarsAK606v3CAN` and prints PASS / FAIL / SKIP.
Use this to verify which modes work on the current firmware build.

```bash
.venv/bin/python scripts/can_demo.py
```

---

### `mit_mode_test.py`
Single-motor MIT protocol validation (CAN ID `0x03` by default). Exercises:
`enable_mit_mode`, `set_mit_mode`, `disable_mit_mode`, plus MIT-backed helpers
`set_position`, `set_velocity`, `set_current`, `stop`, and alias calls
`enable_motor` / `disable_motor`.

```bash
.venv/bin/python scripts/mit_mode_test.py
# If can0 is already configured and you don't want auto sudo reset:
.venv/bin/python scripts/mit_mode_test.py --skip-preflight
# Include velocity/current API sections (can spin the motor):
.venv/bin/python scripts/mit_mode_test.py --include-spin-tests
```

---

### `verify_set_velocity.py`
Focused validation for the `set_velocity()` API only.
It runs `start_ERPM -> 0 -> opposite_ERPM -> 0` (or start-direction only),
checks feedback sign/magnitude against the command, prints strict PASS/FAIL,
and logs each fresh feedback sample to CSV for mocap overlay.

```bash
.venv/bin/python scripts/verify_set_velocity.py --motor-id 0x03
# Signed range: -5000..5000 (starts in given direction, then reverses unless --forward-only)
.venv/bin/python scripts/verify_set_velocity.py --motor-id 0x03 --velocity-erpm -5000
# Skip automatic CAN reset if your interface is already configured:
.venv/bin/python scripts/verify_set_velocity.py --motor-id 0x03 --preflight-mode skip
# Forward-only check:
.venv/bin/python scripts/verify_set_velocity.py --motor-id 0x03 --forward-only
# Firmware-compatibility debug knobs:
.venv/bin/python scripts/verify_set_velocity.py --motor-id 0x03 --helper-policy fcfd --allow-legacy-feedback-ids
.venv/bin/python scripts/verify_set_velocity.py --motor-id 0x03 --feedback-can-id 0x2903
# Reduce velocity-loop aggressiveness if the bus drops under load:
.venv/bin/python scripts/verify_set_velocity.py --motor-id 0x03 --velocity-kd 0.12
# explicit CSV output path:
.venv/bin/python scripts/verify_set_velocity.py --motor-id 0x03 --csv-path data/csv_logs/verify_overlay.csv
```

---

### `mit_position_steps.py`
Long-run MIT position stepping script (ping-pong inside a safe angle window) so it can run for motion capture without hitting MIT position limits.
Primary knobs are still `--angle-deg`, `--duration`, and `--velocity-deg-s`.
Each command/feedback sample is also logged to CSV for direct comparison with mocap data.

```bash
# 3-minute mocap run (default behavior):
.venv/bin/python scripts/mit_position_steps.py --motor-id 0x03 --duration 180 --angle-deg 30 --velocity-deg-s 20

# faster stepping:
.venv/bin/python scripts/mit_position_steps.py --motor-id 0x03 --duration 180 --angle-deg 45 --velocity-deg-s 30

# custom sweep window (stays away from MIT hard limits):
.venv/bin/python scripts/mit_position_steps.py --motor-id 0x03 --duration 180 --angle-deg 30 --velocity-deg-s 20 --min-deg -600 --max-deg 600

# If can0 is already configured and you don't want auto reset:
.venv/bin/python scripts/mit_position_steps.py --motor-id 0x03 --duration 180 --angle-deg 30 --velocity-deg-s 20 --skip-preflight
# explicit CSV output path:
.venv/bin/python scripts/mit_position_steps.py --motor-id 0x03 --duration 180 --angle-deg 30 --velocity-deg-s 20 --csv-path data/csv_logs/mit_overlay.csv
```

> `mit_position_steps.py` uses the same kernel-level reset path as `setup_can.sh`
> for auto preflight (`berr-reporting on`, `restart-ms 100`).

---

### `reset_degree.py`
MIT-only practical reset helper: smoothly recenters the motor to a target degree
(default `0`) in MIT command space.

```bash
.venv/bin/python scripts/reset_degree.py --motor-id 0x03 --target-deg 0
# slower and gentler reset:
.venv/bin/python scripts/reset_degree.py --motor-id 0x03 --target-deg 0 --velocity-deg-s 10 --control-hz 15
# if can0 is already configured:
.venv/bin/python scripts/reset_degree.py --motor-id 0x03 --target-deg 0 --skip-preflight
```

---

### `quick_test.py`
Sends an enable command, listens for 3 s, and prints every unique CAN ID captured.
Useful for confirming the motor is on the bus and identifying the feedback ID.

```bash
.venv/bin/python scripts/quick_test.py
```

---

### `stress_test_can.py`
Runs 30 rounds of connect → enable → get_status → disable → close to verify the driver
is reliable under rapid reconnection and heavy use.

```bash
.venv/bin/python scripts/stress_test_can.py
```

---

### `diagnose_can.py`
Step-by-step diagnostic that tests each phase of the CAN connection independently:
CAN bus state → raw listen → filtered listen → enable/response → rapid cycles.
The script now reports **error-frame volume** explicitly, so `0x88` noise is
distinguished between real data frames and CAN error frames.

```bash
.venv/bin/python scripts/diagnose_can.py
```

---

### `scan_ids.py`
Scans CAN IDs 1–10 to discover which ID the motor is configured to use.
Run this if you don't know the motor's CAN ID.

```bash
.venv/bin/python scripts/scan_ids.py
```

---

### `drive_test.py`
Tries multiple duty formats, levels, and control modes (duty, current, velocity)
and records the feedback to characterise which modes the firmware supports.

```bash
.venv/bin/python scripts/drive_test.py
```

---

### `simple_test.py`
Minimal velocity spin helper for one motor: command one ERPM for one duration, then stop.
Use this when you only want to change speed and spin time quickly.

```bash
# default quick run
.venv/bin/python scripts/simple_test.py

# set only speed and duration
.venv/bin/python scripts/simple_test.py --velocity-erpm 4500 --duration 1.8

# reverse direction
.venv/bin/python scripts/simple_test.py --velocity-erpm -3000 --duration 1.2
```

---

### `tests/manual_our_implementation_test.py`
Manual integration script for the `CubeMarsAK606v3CAN` class API: enable,
feedback, velocity over 30 iterations. Good for iterating on the driver code.

```bash
.venv/bin/python tests/manual_our_implementation_test.py
```

---

### `tests/manual_servo_mode_test.py`
Legacy test using the third-party `TMotorCANControl` library's servo mode.
**Not for regular use** — kept for historical reference only.

```bash
.venv/bin/python tests/manual_servo_mode_test.py
```

---

## Two-Motor Setup (Second motor arriving Wednesday)

When the second motor arrives:
1. Set its CAN ID to **0x04** using CubeMars PC software.
2. The driver automatically assigns feedback ID `0x2904` for it.
3. Run the dual-motor demo:

```bash
.venv/bin/python -m motor_python --dual --motor-id-left 0x03 --motor-id-right 0x04
```

4. Hardware test (auto-skips until motor2 is connected):

```bash
make test-hardware-can   # includes TestCANDualMotor — skips if only one motor present
```

---

## Known Firmware Behaviour

| Mode | Works? | Notes |
|------|--------|-------|
| MIT (0x0803) | ✅ | Current production path in Python API |
| Duty/current/velocity/position servo frames | legacy | historical experiments only |

> Root cause of Speed=0 feedback: PSU must be **24 V**. At 18.5 V the H-bridge quietly
> locks out (error_code remains 0). Treat undervoltage and power integrity as first-line
> diagnostics when CAN feedback is present but commanded motion does not occur.
