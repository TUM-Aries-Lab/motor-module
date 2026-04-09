# Motor Control Software for Soft Exoskeleton
[![Coverage Status](https://coveralls.io/repos/github/TUM-Aries-Lab/motor-module/badge.svg?branch=main)](https://coveralls.io/github/TUM-Aries-Lab/motor-module?branch=main)
![Docker Image CI](https://github.com/TUM-Aries-Lab/motor-module/actions/workflows/ci.yml/badge.svg)

Motor control for the CubeMars AK60-6 actuator module used in Aries soft-exosuit development.

## Current Status

- Primary interface: CAN on Jetson Orin Nano via Linux SocketCAN
- Current maintained CAN path: MIT-only control
- Supported high-level helpers: `set_position()`, `set_velocity()`, `set_current()`, `stop()`
- UART support remains available for compatibility and earlier bring-up workflows

This repository is the motor-module codebase, not a full exosuit application. Its current focus is a modular actuator-control layer plus the bench-side scripts used to validate it.

## Key Features

- Transport-independent motor abstraction through `BaseMotor`
- Maintained SocketCAN backend for the CubeMars AK60-6
- MIT-only CAN control path with one consistent command model
- Background command refresh, feedback decoding, cached state handling, and staged recovery
- Bench-validation scripts for MIT mode, position stepping, velocity verification, and plotting

## Installation

For normal development, testing, and analysis:

```bash
uv sync --all-groups
source .venv/bin/activate
```

If you only want to install the package itself:

```bash
uv pip install -e .
```

## CAN Setup on Jetson Orin Nano

### Recommended: systemd startup

Install the tracked `can0.service` once so `can0` comes up automatically on boot:

```bash
sudo cp can0.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable can0.service
sudo systemctl start can0.service
```

This brings `can0` up at 1 Mbps with bus-off auto-recovery.

Useful checks:

```bash
systemctl status can0.service
sudo systemctl restart can0.service
ip -details link show can0
```

### Manual setup

If you do not want to use the service, configure the interface manually:

```bash
sudo ./setup_can.sh
```

## Important Hardware Note

If the R-Link / UART cable is connected to the motor, CAN commands may be silently ignored even though feedback frames are still visible on the bus. For CAN operation, disconnect the UART cable and then reset the CAN interface.

## Recommended Workflow

### 1. Confirm MIT/CAN communication

```bash
sudo ./setup_can.sh
.venv/bin/python scripts/mit_mode_test.py --motor-id 0x03
```

### 2. Validate sustained velocity behavior

```bash
.venv/bin/python scripts/verify_set_velocity.py --motor-id 0x03 --velocity-erpm 3000
```

### 3. Run repeated MIT position stepping

```bash
.venv/bin/python scripts/mit_position_steps.py --motor-id 0x03 --duration 180 --angle-deg 50 --velocity-deg-s 100
```

### 4. Generate comparison plots from recorded CSV files

```bash
.venv/bin/python scripts/plot_graph.py velocity --data-root CSV --out-dir CSV/plots
.venv/bin/python scripts/plot_graph.py position --data-root CSV --out-dir CSV/plots
```

## Python API

The default convenience alias is the CAN motor class:

```python
from motor_python import Motor

with Motor(motor_can_id=0x03) as motor:
    motor.enable_mit_mode()
    motor.set_velocity(3000)
    motor.set_position(90.0)
    motor.stop()
```

For direct access to the specific implementations:

```python
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN
from motor_python.cube_mars_motor import CubeMarsAK606v3
```

### Main CAN lifecycle methods

| Method | Purpose |
| --- | --- |
| `enable_mit_mode()` | Enter MIT mode |
| `disable_mit_mode()` | Exit MIT mode |
| `check_communication()` | Verify that the motor responds on the bus |
| `get_status()` | Return the latest cached `MotorState` |
| `close()` | Stop and close transport resources |

### Main CAN control methods

| Method | Purpose |
| --- | --- |
| `set_position(position_degrees)` | MIT-backed position command |
| `set_velocity(velocity_erpm, allow_low_speed=False)` | MIT-backed velocity command |
| `set_current(current_amps)` | Interpreted as MIT torque feedforward |
| `set_mit_mode(pos_rad, vel_rad_s, kp, kd, torque_ff_nm)` | Direct MIT command |
| `stop()` | Neutral MIT command and disable |

## Useful Bench Scripts

These are the maintained scripts that matter most for the current implementation:

| Script | Purpose |
| --- | --- |
| `scripts/mit_mode_test.py` | Focused MIT/CAN protocol validation |
| `scripts/verify_set_velocity.py` | Bench verification of `set_velocity()` |
| `scripts/mit_position_steps.py` | Long-run MIT position stepping for bench tests |
| `scripts/plot_graph.py` | Chapter-4-style overlay and summary plots from recorded CSV files |
| `scripts/diagnose_can.py` | CAN bus diagnostics and recovery support |
| `scripts/scan_ids.py` | CAN ID discovery |
| `scripts/reset_degree.py` | Practical MIT recentering helper |

Additional exploratory and legacy scripts remain under `scripts/`, but they are not the primary maintained validation path.

## Testing

Run unit tests:

```bash
make test
```

Run CAN hardware tests:

```bash
make test-hardware-can
```

Run all hardware tests:

```bash
make test-hardware-all
```

## Project Structure

```text
can0.service                    systemd unit for automatic can0 startup
setup_can.sh                    manual CAN setup and reset helper
src/motor_python/               package source
scripts/                        bench and validation scripts
tests/                          unit and hardware-oriented tests
data/csv_logs/.gitkeep          placeholder for locally generated CSV logs
Test Rig CAD files/             CAD assets for the bench rig
```

## Notes on Scope

- The current CAN implementation is MIT-only.
- Legacy servo transport modes are intentionally not part of the maintained CAN path.
- The repository includes bench-validation tooling, but not thesis-only notes, temporary files, or local analysis artifacts.

## Run the Package Entry Point

```bash
uv run python -m motor_python
```
