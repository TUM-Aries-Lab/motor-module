# motor-python

Motor communication package for the ARIES Lab (IBRS, TUM). Drives CubeMars AK-series motors for the hip flexion exosuit over CAN using Force Control Mode (MIT), including frame packing, feedback parsing, bus recovery and safety clamping.

**PyPI:** motor-python | **GitHub:** TUM-Aries-Lab/motor-module | **Current version:** 0.0.10 | **Python:** 3.11+

**Primary interface:** SocketCAN, MIT force-control mode. A legacy UART serial implementation is still shipped but is not the active path.

**Supported motors:** AK60-6 V3 (default), AK80-6 V2, AK60-6 V1.1

## Project Structure

```
src/motor_python/
├── __init__.py                     # create_can_motor() factory, Motor alias, __all__
├── __main__.py                     # CLI entry point (MotorManager demos)
├── base_motor.py                   # BaseMotor interface, MotorState dataclass
├── definitions.py                  # Constants, MotorSpec profiles, MIT limits, CRC table
├── can_protocol.py                 # CANControlMode — extended-ID mode constants
├── can_utils.py                    # Bus state query and interface reset helpers
├── cube_mars_motor_can.py          # CAN/MIT drivers — the active implementation
├── examples_can.py                 # Example CAN control loops
├── motor_manager.py                # MotorManager — several motors by CAN ID
├── pid_controller.py               # PID used by motor_control_using_pid
├── motor_control_using_pid.py      # Closed-loop position control on top of the driver
├── second_order_low_pass_filter.py # Filter used in the PID path
├── cube_mars_motor.py              # LEGACY UART driver (CubeMarsAK606v3/AK806v2)
├── motor_status_parser.py          # LEGACY UART status frame parser
├── examples.py                     # LEGACY UART example loops
└── utils.py                        # Logging, timestamped files, unit + MIT bit helpers
```

Uses src-layout. Package metadata and dependencies in `pyproject.toml`.
Tests in `tests/` at root level; bench and diagnostic scripts in `scripts/`.

## CAN Driver Class Hierarchy

`cube_mars_motor_can.py` holds one base class and three thin subclasses:

- `CubeMarsBaseCAN` — the shared MIT implementation, used directly by the AK80-6 V2 and AK60-6 V1.1
- `CubeMarsAK606v3CAN` — **overrides** `pack_mit_frame`, `_parse_feedback_msg` and `_connect`, because the V3 differs from the other two
- `CubeMarsAK806v2CAN`, `CubeMarsAK606v1CAN` — model defaults only, no behaviour of their own

Build motors through `create_can_motor(model, ...)` or `MotorManager`, which pass the right `MotorSpec`. Constructing a class directly without `motor_spec=` falls back to the currently selected global profile, which may not be that class's motor.

## Communication Protocol

SocketCAN over `can0` (Jetson Orin Nano) at 1 Mbps, via an SN65HVD230 transceiver with 120 Ω termination. Bring the interface up with `setup_can.sh` (or `make setup-can`) before use.

Frame format differs by motor, which is the single most important distinction in this package:

| | AK60-6 V3 | AK80-6 V2 / AK60-6 V1.1 |
|---|---|---|
| Command frames | **Extended** ID `(0x08 << 8) \| motor_id` | **Standard** ID `motor_id` |
| Command payload | `KP, KD, Position, Speed, Torque` | `Position, Speed, KP, KD, Torque` |
| Feedback | `pos(int16×0.1°), speed(int16×10 ERPM), current(int16×0.01 A), temp(int8), err(uint8)` | MIT reply: `id, pos(16b), vel(12b), current(12b), temp−40, err` |
| MIT reset on connect | no | yes (`0xFD` frame) |

The split is driven by `EXTENDED_FORMAT_MOTOR_MODELS` in `definitions.py` — add a model there rather than testing model names inline.

## MIT Encoding Ranges

Physical values are packed into fixed-width integers against per-motor ranges (`MITModeLimits`). Getting these wrong silently mis-scales every command, because the motor firmware decodes with its own fixed ranges.

| Motor | position | velocity | torque | kp | kd |
|---|---|---|---|---|---|
| AK60-6 V3 | ±12.56 rad | ±60 rad/s | ±12 N·m | 0–500 | 0–5 |
| AK80-6 V2 | ±12.56 rad | ±76 rad/s | ±12 N·m | 0–500 | 0–5 |
| AK60-6 V1.1 | ±12.5 rad | ±45 rad/s | ±15 N·m | 0–500 | 0–5 |

These are **encoding ranges, not commanded values** — do not confuse them with the gains below.

## Commanded Gains

MIT mode computes `tau = kp*(p_des - p) + kd*(v_des - v) + tau_ff` on the motor.

- Every velocity path commands `kp = 0`, so `MotorSpec.mit_velocity_kd` **is the entire gain of the inner velocity loop** (N·m per rad/s of tracking error). Changing it changes assist torque directly.
- `set_position()` is the only method that commands a non-zero kp (`MotorSpec.mit_position_kp`).
- `CANDefaults.mit_velocity_kd` (0.2) is a separate, more conservative default used only as a CLI default by some `scripts/`. It is not what the library uses.

Values carrying `TO BE CHECKED` comments in `definitions.py` have not been confirmed on hardware — read those comments before trusting or changing them.

## Data Flow

```
velocity setpoint (ERPM)  [from exosuit-python / hip-controller]
    │
    └─► set_velocity(erpm)
            ├─► _erpm_to_rad_s()
            └─► set_mit_mode(pos=0, vel, kp=0, kd=mit_velocity_kd, tau_ff=0)
                    ├─► clamp velocity to the MotorSpec ERPM range
                    ├─► pack_mit_frame()  → float_to_uint against MITModeLimits
                    └─► _send_mit_payload() → extended or standard frame per model
                            └─► CanBusDispatcher → _parse_feedback_msg() → MotorState
```

A background refresh thread re-sends the MIT command to satisfy the motor's watchdog; `CanBusDispatcher` is the only thing allowed to call `bus.recv()`.

## Safety and Recovery

- Commands are **clamped**, never raised on — a bad setpoint must not crash the control loop
- `MotorLimits` (`definitions.py`) caps velocity and movement time; `MotorSpec` caps per-model velocity
- After 3 consecutive failed status queries the motor is marked non-communicating
- Bus faults trigger recovery via `can_utils.reset_can_interface`; kernel-level reset is opt-in (`aggressive_bus_reset`)
- Context manager support — `with` guarantees stop + close on exit or crash

## How to Run

```bash
make init                       # uv sync + pre-commit install
make setup-can                  # bring up can0 (needs sudo)
python -m motor_python --motor-ids 0x03      # single motor demo
python -m motor_python --dual                # two-motor demo
python -m motor_python --discover            # scan for motor IDs

make test                       # pytest, excludes hardware markers, 70% coverage gate
make lint                       # ruff format + ruff check --fix
make typecheck                  # pyright
make test-hardware-can          # CAN hardware tests (motor on can0 required)
```

`scripts/` holds bench tools — `mit_mode_test.py`, `verify_set_velocity.py`, `mit_position_steps.py`, `scan_ids.py`, `diagnose_can.py` and others.

## This Package in Context

Sibling packages:
- `hip-controller` — gait phase estimation and motor command generation
- `imu-python` — BNO055 IMU reading and orientation estimation
- `exosuit-python` — top-level integration (currently uses the legacy UART class)

Note that `LocomotionMode_IMUbased` does **not** use this package; it carries its own `motor_control.py` CAN driver.

## Conventions

See `.claude/skills/code-review-nathalie.md` for the full coding conventions checklist.

## Restrictions

- **Never modify motor safety limits** (`MotorLimits`, `MotorSpec` velocity caps) without explicit discussion
- **Never change commanded MIT gains** (`mit_velocity_kd`, `mit_position_kp`) without hardware confirmation — they set torque per unit of error
- **Never change the MIT encoding ranges** (`MITModeLimits`) — they must match the motor firmware exactly
- **Never change the default CAN interface or bitrate** without hardware verification
- **Never remove safety clamping** on any control method
- **Never bypass the context manager cleanup** — motor must always stop on exit
- **Never hardcode hardware-specific paths**
- **Never push directly to main** — always use pull requests
- Always run `make lint`, `make typecheck` and `make test` before considering a task complete
