# Motor Control Software for Soft Exoskeleton
[![Coverage Status](https://coveralls.io/repos/github/TUM-Aries-Lab/motor-module/badge.svg?branch=main)](https://coveralls.io/github/TUM-Aries-Lab/motor-module?branch=main)
![Docker Image CI](https://github.com/TUM-Aries-Lab/motor-module/actions/workflows/ci.yml/badge.svg)

Motor control for exosuit tendon systems using a CubeMars AK60-6 motor.

**Communication Interfaces:**
- **UART:** Serial via RS-485 or TTL
- **CAN:** CAN bus via SN65HVD230 transceiver on Jetson Orin Nano

## Install

```bash
uv pip install -e .
```

Both motor classes inherit from `BaseMotor` which provides a unified API for
both UART and CAN control.

### Basic Setup (Jetson Orin Nano)

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
python scripts/motion_capture_test.py
python scripts/can_demo.py
```

To check status: `systemctl status can0.service`
If you ever need to reset manually: `sudo systemctl restart can0.service`
The tracked `can0.service` file in this repo is copied once to
`/etc/systemd/system/can0.service` during setup and then used by systemd at boot.

See [docs/CAN_SETUP_GUIDE.md](docs/CAN_SETUP_GUIDE.md) for hardware wiring details.

> **Critical — UART blocks CAN:** If the R-Link / UART cable is connected to the motor,
> CAN commands are silently ignored. The motor still broadcasts 50 Hz feedback over CAN, but
> it will not ACK any command frame — this fills the Jetson's TX error counter and pushes
> the interface into BUS-OFF state (116 M+ errors observed in one session).
> **Always disconnect the UART/R-Link cable before using CAN control.**
> After connecting or disconnecting any cable, reset the interface:
> ```bash
> sudo systemctl restart can0.service   # or: sudo ./setup_can.sh
> ```

## CAN Not Working? Step-by-step Recovery

If the motor is silent (no `candump can0` output) or the script says
"No CAN frames in 1.5 s", work through these steps in order:

**Step 1 — Check the obvious**
- Is the motor powered on? (LED on the motor should be lit)
- Is the UART/R-Link cable unplugged? (it silently blocks all CAN commands)
- Is the CAN cable plugged into the correct port on the Jetson?

**Step 2 — Check the interface state**
```bash
ip -details link show can0
```
Look for `state ERROR-ACTIVE (berr-counter tx 0 rx 0)` — that's healthy.
If you see `ERROR-PASSIVE` or `BUS-OFF` go to Step 3.

**Step 3 — Reset the CAN interface**

A simple `ip link set down/up` does NOT reset the hardware error counters on
the Jetson Orin Nano mttcan controller. You must reload the kernel module:

```bash
sudo ip link set can0 down
sudo modprobe -r mttcan
sleep 0.5
sudo modprobe mttcan
sleep 0.2
sudo ip link set can0 up type can bitrate 1000000 berr-reporting on restart-ms 100
sudo ip link set can0 txqueuelen 1000
```

Or in one line:
```bash
sudo ip link set can0 down && sudo modprobe -r mttcan && sleep 0.5 && sudo modprobe mttcan && sleep 0.2 && sudo ip link set can0 up type can bitrate 1000000 berr-reporting on restart-ms 100 && sudo ip link set can0 txqueuelen 1000
```

**Step 4 — Power-cycle the motor**

If the interface reset didn't help, the motor itself may have entered a fault
state (common after `test_velocity_loop.py` fills the TX queue):
1. Unplug motor power
2. Wait 3 seconds
3. Reconnect power
4. Re-run Step 3

**Step 5 — Verify traffic**
```bash
timeout 3 candump can0
```
You should see `0x2903` frames scrolling at ~50 Hz. If yes, the motor is ready.

> **Current CAN implementation is MIT-only (Force Control Mode, `mode_id=0x08`).**
> Legacy servo transport modes (`duty/current/brake/profile`) are intentionally disabled.
> See [docs/CAN_TELEMETRY_API.md](docs/CAN_TELEMETRY_API.md) for the full API reference.

## Quick Start — CAN

### MIT Bench Test (recommended)

Use the MIT-only validation script for current CAN implementation:

```bash
sudo ./setup_can.sh
.venv/bin/python scripts/mit_mode_test.py --motor-id 0x03
```

### Legacy Spin Test (archived duty workflow)

Historical quick check from pre-MIT bring-up. Spins the motor forward
for 2 s, pauses, then reverses for 2 s using raw duty-cycle commands.

> Legacy bench script from the pre-MIT-only phase.

```bash
sudo ./setup_can.sh
.venv/bin/python scripts/spin_test.py
```

### Motion Capture Test (legacy duty-cycle workflow)

Historical script from the earlier duty-cycle control phase.
Current production CAN control is MIT-only; keep this section for archived runs.

```bash
sudo ./setup_can.sh
.venv/bin/python scripts/motion_capture_test.py
```

Output: `data/logs/mocap_<timestamp>.csv` + `data/logs/mocap_<timestamp>.png`

Key parameters (adjustable at the top of the script):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAX_DUTY` | 0.30 | Peak duty cycle (30%) |
| `KP / KD` | 0.005 / 0.000015 | PD gains |
| `SINE_CENTER / SINE_AMP` | 90° / 30° | Sine sweep centre and amplitude |
| `SINE_PERIOD / SINE_DURATION` | 6 s / 90 s | Oscillation period and total run time |
| `WAYPOINT_SETTLE_TIME` | 0.3 s | Settle dwell at each waypoint |

### Quick Start — CAN (primary interface)

```python
from motor_python import Motor  # CAN by default
from motor_python.definitions import TendonAction

with Motor(motor_can_id=0x03) as motor:
    motor.enable_motor()
    motor.set_velocity(velocity_erpm=8000)               # velocity loop via MIT parameters
    motor.control_exosuit_tendon(TendonAction.PULL)      # high-level: pull tendon
    motor.control_exosuit_tendon(TendonAction.STOP)
    motor.set_position(90.0)                             # position loop via MIT parameters
    motor.set_mit_mode(pos_rad=1.57, kp=100, kd=2)       # impedance control
```

### Quick Start — UART (testing only)

```python
from motor_python import CubeMarsAK606v3 as Motor  # swap this one line

with Motor() as motor:
    motor.set_velocity(velocity_erpm=10000)
    motor.stop()
```

Both classes share the same public API — `set_velocity`, `set_position`, `stop`,
`control_exosuit_tendon`, `get_position`, `get_status`, `check_communication`, etc.

## Publishing

It's super easy to publish your own packages on PyPi. To build and publish this package run:
1. Update the version number in pyproject.toml and motor_python/__init__.py
2. Commit your changes and add a git tag "<new.version.number>"
3. Push the tag `git push --tag`

## API

### Lifecycle

| Method | Description |
|--------|-------------|
| `enable_motor()` | Compatibility alias for `enable_mit_mode()` |
| `disable_motor()` | Compatibility alias for `disable_mit_mode()` |
| `check_communication()` | Verify motor is responding over CAN |
| `close()` | Stop + release CAN bus (also called automatically by `with` statement) |

### Control (MIT-only CAN implementation)

| Method | Description |
|--------|-------------|
| `set_velocity(velocity_erpm, allow_low_speed=False)` | Velocity loop using MIT fields |
| `set_position(position_degrees)` | Position loop using default MIT gains |
| `set_current(current_amps)` | Interpreted as MIT torque feedforward (Nm) |
| `set_brake_current(current_amps)` | Not implemented (raises) |
| `set_duty_cycle(duty)` | Not implemented (raises) |
| `set_origin(permanent=False)` | Not implemented (raises) |
| `set_position_velocity_accel(pos, vel, accel)` | Not implemented (raises) |
| `move_to_position_with_speed(target, speed_erpm)` | Velocity-drive to target then hold |
| `control_exosuit_tendon(action, velocity_erpm)` | High-level tendon helper via `set_velocity()` |
| `stop()` | Neutral MIT command + MIT disable |

### Control (MIT impedance mode — CAN only)

| Method | Description |
|--------|-------------|
| `enable_mit_mode()` | Enter MIT mode (replaces `enable_motor()`) |
| `set_mit_mode(pos_rad, vel_rad_s, kp, kd, torque_ff_nm)` | τ = kp·Δpos + kd·Δvel + tau_ff |
| `disable_mit_mode()` | Exit MIT mode |

### Telemetry

| Method | Returns |
|--------|---------|
| `get_position()` | Current angle in degrees (float) |
| `get_speed()` | Current speed in ERPM (int) |
| `get_current()` | Phase current in Amps (float) |
| `get_temperature()` | Driver board temperature in °C (int) |
| `get_status()` | Full state as a `MotorState` object |
| `get_motor_data()` | All telemetry fields as a `dict` |

*Note: All getters rely on the same underlying `MotorState` structure.*

## Known Firmware Limitations

The Python CAN driver now intentionally uses **MIT mode only** (`mode_id=0x08`).

- Legacy servo transport calls are disabled:
  - `set_duty_cycle`
  - `set_brake_current`
  - `set_origin`
  - `set_position_velocity_accel`
- `enable_motor()` / `disable_motor()` are compatibility aliases to
  `enable_mit_mode()` / `disable_mit_mode()`.

## Velocity Safety

`set_velocity()` is implemented through MIT helper mapping.

- Base safety still blocks very low non-zero ERPM by default.
- Use `allow_low_speed=True` only when needed for controlled bench tuning.

```python
motor.set_velocity(6000, allow_low_speed=True)
motor.stop()
```

## Position Control

`set_position(degrees)` is MIT-backed and uses default position gains from
`CAN_DEFAULTS` (`mit_position_kp`, `mit_position_kd`).

## Exosuit Integration

For tendon control, use MIT-backed velocity helper or direct `set_mit_mode`.

### Minimal example

```python
from motor_python import Motor

with Motor(motor_can_id=0x03) as motor:
    motor.enable_mit_mode()
    while True:
        joint_vel_dps = read_imu()  # your IMU read function
        desired_erpm = int(joint_vel_dps * 18.0)
        motor.set_velocity(desired_erpm, allow_low_speed=True)
```

### Key notes for exosuit use

| Topic | Detail |
|-------|--------|
| Protocol | MIT force control (`mode_id=0x08`) |
| Velocity helper | `set_velocity()` maps ERPM to MIT `vel_rad_s` |
| Position helper | `set_position()` maps degrees to MIT `pos_rad` |
| Torque helper | `set_current(x)` currently interpreted as `torque_ff_nm=x` |
| Stop behavior | `stop()` sends neutral MIT command and disables MIT mode |
| Feedback | Use `get_status()` / getters (pos, ERPM, current, temperature, fault) |

## Run

```bash
uv run python -m motor_python
```


## Testing

```bash
make test            # Unit tests only (no hardware, uses mocks)
make test-hardware   # All tests (unit + hardware, requires motor connected)
```

## Project Structure

```
scripts/spin_test.py               # Legacy CAN smoke test (duty cycle fwd/rev)
scripts/motion_capture_test.py     # Legacy arm sweep script (duty PID, CSV + plot)
scripts/can_demo.py                # Legacy API exploration script
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
    manual_our_implementation_test.py
    manual_servo_mode_test.py
docs/
    CAN_SETUP_GUIDE.md             # CAN hardware wiring guide
    CAN_TROUBLESHOOTING.md         # Known CAN issues and fixes
```
