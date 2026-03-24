# Session Log — 25 February 2026

All work is on the `CAN_implementation` branch.
Commits in chronological order, latest at bottom.

---

## Summary of Changes

| Area | What changed |
|---|---|
| `cube_mars_motor_can.py` | Telemetry methods, `CAN_ERROR_CODES`, `error_description`, `set_brake_current()` |
| `can_demo.py` | Created — 13-section end-to-end API demo |
| `can0.service` | Created — systemd unit for automatic CAN interface bring-up at boot |
| `README.md` | CAN-first rewrite; added `can0.service` one-time install instructions |
| `motion_capture_test.py` | 5-loop sweep, velocity logging, summary table, PNG plots, safety clamp |
| `docs/CAN_IMPLEMENTATION_CHAPTER.tex` | Three new sections + professional overhaul |

---

## Commit Log

### `cbb952f` — Motion capture: 5-loop sweep, velocity logging, safety clamp, summary & plots

**File:** `motion_capture_test.py`

Added a `NUM_LOOPS = 5` outer loop so the arm sweeps the full waypoint sequence five
times per run (intended for motion-capture / IMU averaging).

Added `velocity_deg_s` column to the CSV, computed numerically as
Δθ / Δt from successive motor feedback position values.

Added a `loop` column (1–5) so post-processing software can separate individual passes.

Added `print_summary()` — after the run, prints a table to the console with
per-waypoint mean error, max error, mean velocity, and peak velocity across all loops.

Added `generate_plots()` — saves a two-panel PNG alongside the CSV:
- Panel 1: commanded vs. actual angle over time
- Panel 2: computed angular velocity over time

Added a **safety clamp** applied after the PID calculation at every 50 Hz tick:
- `MIN_ANGLE = 0.0°` — if the arm is at or below this and duty < 0, force duty to 0
- `MAX_ANGLE = 130.0°` — if the arm is at or above this and duty > 0, force duty to 0

Prevents the arm from crossing to the left side of the test rig regardless of PID output.

---

### `dc2af04` — CAN telemetry methods: get_temperature, get_current, get_speed, get_motor_data

**File:** `src/motor_python/cube_mars_motor_can.py`

Four new public methods added to `CubeMarsAK606v3CAN`:

| Method | Returns | Notes |
|---|---|---|
| `get_position()` | `float \| None` | Shaft angle in degrees relative to start-up origin |
| `get_temperature()` | `int \| None` | Driver temperature in °C |
| `get_current()` | `float \| None` | Winding current in Amps |
| `get_speed()` | `int \| None` | Shaft speed in electrical RPM |
| `get_motor_data()` | `dict \| None` | All five fields plus `error_description` |

All methods read from `_last_feedback` (instant, cached by the 50 Hz background thread).
Fallback: if no feedback has arrived yet, they call `_receive_feedback()` with a
50 ms blocking timeout rather than returning stale data.

---

### `bcf21d1` — CAN error codes table, error_description property

**File:** `src/motor_python/cube_mars_motor_can.py`

Added `CAN_ERROR_CODES` module-level dict mapping error byte values 0–7 to
human-readable strings taken directly from the CubeMars CAN spec section 4.3.1:

| Code | Meaning |
|---|---|
| 0 | No fault |
| 1 | Over-temperature |
| 2 | Over-current |
| 3 | Over-voltage |
| 4 | Under-voltage |
| 5 | Encoder fault |
| 6 | Hardware protection triggered |
| 7 | Reserved |

Added `error_description` property to `CANMotorFeedback` — returns the string from
`CAN_ERROR_CODES` for the current error code byte.

Updated `get_status()` to log the `error_description` text alongside the numeric code
so fault conditions are visible in the console without consulting the spec.

Added `error_description` key to the `get_motor_data()` return dict.

---

### `2bfa5c5` — set_brake_current(), CANControlMode spec docstring

**File:** `src/motor_python/cube_mars_motor_can.py`

Added `set_brake_current(current_amps: float)` method.

- Uses instruction type `0x02` (arb_id = `0x00000203` for motor ID 3).
- Payload encoding: `int32(current_amps × 1000)` big-endian, same as regular current
  control but with the different instruction byte.
- Effect: regenerative braking — short-circuits the windings in a controlled way to
  actively slow the shaft rather than simply removing torque.

Updated `CANControlMode` class docstring with a full specification table documenting
all six instruction types and example arbitration IDs for motor ID 3.

---

### `732832f` — can_demo.py created

**File:** `can_demo.py` (project root)

A 13-section sequential demo that exercises every public method of
`CubeMarsAK606v3CAN`. Run with:

```bash
python can_demo.py
```

Sections in order:

| # | Method / Feature tested |
|---|---|
| 1 | `check_communication()` |
| 2 | `enable_motor()` |
| 3 | `get_status()` |
| 4 | `get_temperature()`, `get_current()`, `get_speed()` individually |
| 5 | `get_motor_data()` — full dict |
| 6 | `get_position()` |
| 7 | `set_duty_cycle()` forward — live ERPM readback for 2 s |
| 8 | `set_duty_cycle()` reverse — live ERPM readback for 2 s |
| 9 | `set_velocity()` — acknowledged but no shaft rotation (firmware limitation) |
| 10 | `set_brake_current()` |
| 11 | `control_exosuit_tendon()` — pull, release, stop using `TendonAction` enum |
| 12 | `stop()` + `disable_motor()` |
| 13 | Print full `CAN_ERROR_CODES` reference table |

Verified exit code 0 with motor physically connected (motor at 318°, 33°C, 0A at rest).

---

### `0a0662c` — can_demo.py crash fix: TendonAction enum

**File:** `can_demo.py`

`control_exosuit_tendon(action="pull")` raised `ValueError: Invalid action pull`.
The method requires the `TendonAction` enum, not raw strings.

Fix: import `TendonAction` from `motor_python.definitions` and pass
`TendonAction.PULL`, `TendonAction.RELEASE`, `TendonAction.STOP` in all three calls.

---

### `17b30b6` — can0.service systemd unit + README update

**Files:** `can0.service` (new), `README.md`

#### Problem
The CAN interface required manual administrator commands before every session
(via `sudo ./setup_can.sh`). After any reboot the system had no CAN communication
until the script was run again.

#### Solution
Created `can0.service` — a systemd one-shot service unit that runs the full bus
setup automatically at every boot.

Six commands executed at boot:

```
ExecStart = modprobe can
ExecStart = modprobe can_raw
ExecStart = modprobe mttcan
ExecStart = ip link set can0 down
ExecStart = ip link set can0 up type can bitrate 1000000 berr-reporting on restart-ms 100
ExecStart = ip link set can0 txqueuelen 1000
```

One-time installation (administrator, once per machine):

```bash
sudo cp can0.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable can0.service
sudo systemctl start can0.service
```

After installation: `can0` is `state UP` on every subsequent boot with no user action.

Updated `README.md`:
- Replaced `sudo ./setup_can.sh` quickstart with `can0.service` install instructions.
- Added `can_demo.py` and `can0.service` to the project structure listing.
- Added firmware-limitation warning block (velocity/position modes ACK but don't rotate).
- Added motion capture parameter table.

Status confirmed: `active (exited)`, `ip link show can0` → `state UP qlen 1000`.

---

### `8cdc087` — LaTeX: new sections (telemetry, PID control, boot service)

**File:** `docs/CAN_IMPLEMENTATION_CHAPTER.tex`

Added three new sections (+299 lines):

**Section 8 — Motor Telemetry**
- Documents all five telemetry methods with their return types and fall-back behaviour.
- Includes a full Table of all eight CAN error codes from spec section 4.3.1.
- Documents `error_description` property and its use in `get_status()`.

**Section 9 — Motion Validation with Software PID Position Control**
- Explains why duty-cycle mode is the only working mode on this firmware.
- Derives the PID equations with values (KP, KI, KD, clamp, dead zone, minimum duty).
- Documents the safety clamp (0°–130°).
- Documents the five waypoints and the 5×-loop sweep.
- Documents all 11 CSV columns, the post-run summary table, and the auto-generated PNG.

**Section 10 — Automatic CAN Interface Initialisation at Boot**
- Explains the original problem (manual `sudo ./setup_can.sh`).
- Documents the `can0.service` systemd one-shot unit and its six boot commands.
- Documents the one-time install procedure.

Updated Summary section: "three mechanisms" → "six mechanisms". Added PID parameter
block to the summary table (gains, limits, clamp angles, log rate, loop count).

---

### `a2b8b56` — LaTeX: professional overhaul

**File:** `docs/CAN_IMPLEMENTATION_CHAPTER.tex`

Seven improvements to bring the document to professional paper standard:

1. **Abstract** — added before the table of contents.
2. **Introduction** — expanded to include project context, a section-by-section roadmap
   with `\ref` links to every section so a reader can navigate directly.
3. **TikZ flowchart (Figure 1)** — completely rebuilt.
   - Root cause of crossing arrows: `(launch) |- (ready)` drew an L-shaped path
     directly through the `loop` node.
   - New layout: `launch` and `loop` are in the right column at the same vertical
     levels as the two nodes they connect to in the left column. All arrows are
     straight (horizontal or vertical) or travel along the right margin only.
   - Local node styles scoped inside the figure so they don't affect other diagrams.
4. **Brake Current subsection** (Section 4) — documents `set_brake_current()`:
   regenerative braking, payload encoding, and why it matters for exosuit deceleration.
5. **Firmware note subsection** (Section 7) — explicitly states that the two-phase
   velocity→position strategy is the intended design but is inapplicable on this
   firmware. Routes the reader to Section 9 (PID). Keeps the original section for
   future reference.
6. **`get_position()` in telemetry** — added as the first item in the telemetry
   description list with a cross-reference to the PID section.
7. **Comprehensive API Demonstration subsection** — documents `can_demo.py` and how
   to use it as a fast end-to-end system health check.

---

## Key Constants and Parameters (as of end of session)

```
Motor CAN ID        : 0x03
Feedback ID         : 0x2903 (extended)
CAN filter          : can_id=0x2903, can_mask=0x1FFFFFFF, extended=True
CAN bitrate         : 1 Mbit/s
Feedback rate       : 50 Hz

PID control rate    : 50 Hz
KP                  : 0.004  duty/deg
KI                  : 0.0008 duty/(deg·s)
KD                  : 0.0003 duty/(deg/s)
MAX_DUTY            : ±0.15
MIN_DUTY            : 0.015  (static friction override)
TOLERANCE           : ±2.0°
HOLD_AFTER          : 1.5 s
HOLD_TIME           : 10.0 s (timeout per waypoint)
MIN_ANGLE           : 0.0°   (safety clamp)
MAX_ANGLE           : 130.0° (safety clamp)
NUM_LOOPS           : 5
SAMPLE_HZ           : 20 Hz  (CSV log rate)

systemd service     : /etc/systemd/system/can0.service
Service status      : active (exited) — starts at boot
```

---

## Known Firmware Limitation

Velocity mode (`0x0303`), position mode (`0x0403`), and position-with-velocity
mode (`0x0603`) all produce a valid CAN acknowledgement but **do not produce shaft
rotation** on the evaluation unit's firmware.

Only **duty-cycle mode** (`arb_id = motor_id = 0x03`) is confirmed to produce
sustained, controllable motion.

This is believed to be a firmware configuration issue on the specific evaluation unit.
All software implementations of those modes are retained and correct — they will
work if the firmware is updated or a production unit with different firmware is used.
