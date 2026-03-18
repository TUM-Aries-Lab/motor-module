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
Minimal smoke-test: enable → set_velocity(5000) → disable.
Useful as a first-boot sanity check.

```bash
.venv/bin/python scripts/simple_test.py
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
> locks out (error_code remains 0). See `docs/CAN_TROUBLESHOOTING.md` Problem 5.
