# motor-python

Motor communication package for the ARIES Lab (IBRS, TUM). Handles UART serial communication with CubeMars AK60-6 motors for the hip flexion exosuit, including frame building, command sending, response parsing, and safety clamping.

**PyPI:** motor-python | **GitHub:** TUM-Aries-Lab/motor-module | **Current version:** 0.0.6 | **Python:** 3.11+

**Target motor:** CubeMars AK60-6 V3 KV80 | **Communication:** UART serial (not CAN)

## Project Structure

```
src/motor_python/
├── __init__.py
├── __main__.py                  # Standalone testing (runs all control modes)
├── definitions.py               # Constants, configs, scale factors, fault codes, CRC table
├── cube_mars_motor.py           # CubeMarsAK606v3 — main motor controller class
├── motor_status_parser.py       # MotorStatusParser — parses feedback frames
├── examples.py                  # Example control loops (position, velocity, duty, current)
└── utils.py                     # Logging setup, timestamped files
```

Uses src-layout. Package metadata and dependencies in `pyproject.toml`.
Tests in `tests/` at root level.

## Communication Protocol

UART serial over `/dev/ttyTHS1` (Jetson Orin Nano) at 921600 baud.

Frame structure: `| 0xAA | DataLength | CMD | Payload... | CRC_H | CRC_L | 0xBB |`

- All multi-byte values are **big-endian** (`struct.pack(">i", ...)`)
- CRC16-CCITT checksum via 256-entry lookup table in `definitions.py`
- Response wait: 100ms after each send

## Data Flow

```
PID output (rad/s)  [from exosuit-python]
    │
    ├─► convert_rad_per_sec_to_rpm()     ← eRPM = (rad/s × 60) / (2π)
    │
    └─► CubeMarsAK606v3.set_velocity(eRPM)
            ├─► clamp to ±100,000 eRPM
            ├─► struct.pack(">i", eRPM)
            ├─► _build_frame(CMD_SET_SPEED, payload)
            │       ├─► prepend command byte
            │       ├─► compute CRC16
            │       └─► wrap in 0xAA...0xBB frame
            └─► _send_frame(frame)
                    ├─► serial.write() → wait 100ms → read response
                    └─► parse response via MotorStatusParser
```

## Motor Commands

| Command | Code | Method | Units |
|---------|------|--------|-------|
| Get status | 0x45 | `get_status()` | — |
| Set velocity | 0x49 | `set_velocity()` | eRPM (int) |
| Set position | 0x4A | `set_position()` | degrees (float, ×1,000,000) |
| Set current | 0x47 | `set_current()` | amps (float, ×1000) |
| Set duty | 0x46 | `set_duty_cycle()` | fraction -1.0 to 1.0 (×100,000) |
| Get position | 0x4C | `get_position()` | lightweight, updates every 10ms |

## Safety Limits (MotorLimits)

All commands are **clamped** before sending — never raises exceptions:
- Duty cycle: ±0.95 (95%)
- Current: ±60 A
- Velocity: ±100,000 eRPM
- Position: ±2147.48° (~6 rotations)
- Movement time cap: 5.0 s

## Status Parsing

`MotorStatusParser` extracts structured data from status responses into frozen dataclasses:
- `MotorTemperatures` — MOS and motor winding temps (°C)
- `MotorCurrents` — output, input, id, iq currents (A)
- `MotorDutySpeedVoltage` — duty, speed (eRPM), bus voltage (V)
- `MotorStatusPosition` — fault code, position (°), motor ID
- `MotorEncoders` — internal and external encoder angles (°)
- `MotorStatus` — composite of all above

Scale factors convert raw integers to SI: temperature ÷10, current ÷100, duty ÷1000, voltage ÷10, position ÷1,000,000.

## Key Design Decisions

- **UART, not CAN** — V3 motor uses UART serial, unlike V1.1 which used CAN via MCP2515. Different from the Simulink implementation.
- **Safety clamping** — All commands clamped to safe limits before sending, preventing damage without crashing the control loop.
- **Communication monitoring** — After 3 consecutive failed status queries, motor is marked non-communicating.
- **Context manager** — Supports `with` statement for automatic stop + close on exit/crash.

## How to Run

```bash
pip install -e .
python -m motor_python          # Standalone test (all control modes)
pytest
ruff check .
```

## This Package in Context

Consumed by `exosuit-python`, which calls `set_velocity()` each control loop iteration with the PID output converted to eRPM. Sibling packages:
- `hip-controller` — Gait phase estimation and motor command generation
- `imu-python` — BNO055 IMU reading and orientation estimation
- `exosuit-python` — Top-level integration

## Conventions

See `.claude/skills/code-review.md` for the full coding conventions checklist.

## Restrictions

- **Never modify motor safety limits** (`MotorLimits`) without explicit discussion
- **Never change the default serial port or baudrate** without hardware verification
- **Never remove safety clamping** on any control method
- **Never bypass the context manager cleanup** — motor must always stop on exit
- **Never hardcode hardware-specific paths**
- **Never push directly to main** — always use pull requests
- Always run `ruff check .` and `pytest` before considering a task complete
