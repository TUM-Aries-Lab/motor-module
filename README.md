# Motor Control Software for Soft Exoskeleton
[![Coverage Status](https://coveralls.io/repos/github/TUM-Aries-Lab/motor-module/badge.svg?branch=main)](https://coveralls.io/github/TUM-Aries-Lab/motor-module?branch=main)
![Docker Image CI](https://github.com/TUM-Aries-Lab/motor-module/actions/workflows/ci.yml/badge.svg)

Motor control for exosuit tendon systems using a CubeMars AK60-6 motor.

**Communication Interfaces:**
- **UART:** Serial via RS-485 or TTL
- **CAN:** CAN bus via SN65HVD230 transceiver on Jetson Orin Nano

## Install

```bash
uv install motor_python
```

## CAN Setup (Jetson Orin Nano)

### One-time install (run once, then never again)

```bash
sudo cp can0.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable can0.service
sudo systemctl start can0.service
```

After this, `can0` comes up automatically at every boot at 1 Mbps with BUS-OFF
auto-recovery. No more `sudo ./setup_can.sh` before every run.

```bash
# Just run your script directly:
python motion_capture_test.py
python can_demo.py
```

To check status: `systemctl status can0.service`
If you ever need to reset manually: `sudo systemctl restart can0.service`

See [docs/CAN_SETUP_GUIDE.md](docs/CAN_SETUP_GUIDE.md) for hardware wiring details.

> **Important:** Only **duty cycle mode** (`arb_id = motor_id`) works on the current
> CubeMars AK60-6 firmware. Velocity (0x0303), position (0x0403), and PVA (0x0603)
> modes ACK but do **not** produce shaft rotation.
> See [docs/CAN_TROUBLESHOOTING.md](docs/CAN_TROUBLESHOOTING.md) for details.

## Quick Start — CAN

### Spin Test (verify motor is alive)

The fastest way to confirm CAN communication. Spins the motor forward
for 2 s, pauses, then reverses for 2 s using raw duty-cycle commands.

```bash
sudo ./setup_can.sh
python spin_test.py
```

### Motion Capture Test (arm sweep with logging)

Moves an attached arm through a waypoint sequence (0° → 90° → 120° → 80° → 0°)
five times using a PID controller in duty-cycle mode. Logs position, velocity,
current, and temperature to CSV at 20 Hz and generates angle/velocity plots.

```bash
sudo ./setup_can.sh
python motion_capture_test.py
```

Output: `data/logs/mocap_<timestamp>.csv` + `data/logs/mocap_<timestamp>.png`

Key parameters (adjustable at the top of the script):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAX_DUTY` | 0.15 | Peak duty cycle (15%) |
| `KP / KD / KI` | 0.004 / 0.0003 / 0.0008 | PID gains |
| `TOLERANCE` | 2.0° | Settle tolerance |
| `NUM_LOOPS` | 5 | Number of full sweep repetitions |
| `MIN_ANGLE / MAX_ANGLE` | 0° / 130° | Safety clamp (arm stays on one side) |

### Quick Start — UART

```python
from motor_python.cube_mars_motor import CubeMarsAK606v3

motor = CubeMarsAK606v3()
motor.set_velocity(velocity_erpm=10000)   # Pull tendon
motor.stop()
```

### Quick Start — CAN (Python class)

```python
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

motor = CubeMarsAK606v3CAN(motor_can_id=0x03, interface="can0", bitrate=1000000)
motor.set_velocity(velocity_erpm=10000)
motor.stop()
```

## API

| Method | Description |
|--------|-------------|
| `set_velocity(velocity_erpm, allow_low_speed=False)` | Set speed in ERPM |
| `set_position(position_degrees)` | Set target position in degrees (±360) |
| `get_position()` | Read current position from motor |
| `move_to_position_with_speed(target_degrees, motor_speed_erpm)` | Move to position via velocity then hold |
| `control_exosuit_tendon(action, velocity_erpm)` | Helper for pull / release / stop |
| `stop()` | Stop the motor (current = 0, releases windings) |
| `get_status()` | Query all motor parameters |
| `check_communication()` | Verify motor is responding |

## Testing

```bash
make test            # Unit tests only (no hardware, uses mocks)
make test-hardware   # All tests (unit + hardware, requires motor connected)
```

## Project Structure

```
spin_test.py                       # CAN smoke test (duty cycle fwd/rev)
motion_capture_test.py             # Arm sweep with PID, CSV + plot output
can_demo.py                        # Exercises all CubeMarsAK606v3CAN methods
setup_can.sh                       # Manual CAN setup (use can0.service instead)
can0.service                       # Systemd service — auto-starts can0 at boot
src/motor_python/
    cube_mars_motor.py             # UART motor controller class
    cube_mars_motor_can.py         # CAN motor controller class
    definitions.py                 # Constants and limits
    motor_status_parser.py         # UART telemetry parser
    utils.py
tests/
    cube_mars_motor_test.py        # Unit tests (mocked serial)
    hardware_test.py               # Integration tests (real motor)
docs/
    CAN_SETUP_GUIDE.md             # CAN hardware wiring guide
    CAN_TROUBLESHOOTING.md         # Known CAN issues and fixes
```
