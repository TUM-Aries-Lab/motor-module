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

> **Confirmed working control modes:** duty cycle, position, position+velocity+acceleration,
> current, brake current, and MIT impedance mode.
> **Velocity loop (0x0303) is not functional on this unit — see [Known Firmware Limitations](#known-firmware-limitations) below.**
> See [docs/CAN_TELEMETRY_API.md](docs/CAN_TELEMETRY_API.md) for the full API reference.

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
    motor.set_duty_cycle(0.15)                           # 15% duty ≈ 1800 ERPM (works on all firmware)
    motor.control_exosuit_tendon(TendonAction.PULL)      # high-level: pull tendon
    motor.control_exosuit_tendon(TendonAction.STOP)
    motor.set_position_velocity_accel(90.0, 8000, 4000)  # smooth move to 90°
    motor.set_mit_mode(pos_rad=1.57, kp=100, kd=2)       # impedance control
```

> **Note:** `set_velocity()` sends `0x0303` frames which are not acknowledged by the motor
> firmware on this unit. Use `set_duty_cycle()` for speed control instead — see
> [Known Firmware Limitations](#known-firmware-limitations).

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
| `enable_motor()` | Power on — enter Servo mode (required before any command) |
| `disable_motor()` | Power off — motor coasts |
| `check_communication()` | Verify motor is responding over CAN |
| `close()` | Stop + release CAN bus (also called automatically by `with` statement) |

### Control (Servo mode)

| Method | Description |
|--------|-------------|
| `set_velocity(velocity_erpm, allow_low_speed=False)` | Continuous spin at given ERPM |
| `set_position(position_degrees)` | Move to angle and hold |
| `set_position_velocity_accel(pos, vel, accel)` | Smooth trapezoidal profile move |
| `set_current(current_amps)` | Direct torque via IQ current |
| `set_brake_current(current_amps)` | Regenerative hold against external load |
| `set_duty_cycle(duty)` | Raw PWM fraction −1.0 … +1.0 |
| `set_origin(permanent=False)` | Define current angle as new zero |
| `move_to_position_with_speed(target, speed_erpm)` | Velocity-drive to target then hold |
| `control_exosuit_tendon(action, velocity_erpm)` | High-level: `TendonAction.PULL / RELEASE / STOP` |
| `stop()` | Zero current, coast to stop |

### Control (MIT impedance mode — CAN only)

| Method | Description |
|--------|-------------|
| `enable_mit_mode()` | Enter MIT mode (replaces `enable_motor()`) |
| `set_mit_mode(pos_rad, vel_rad_s, kp, kd, torque_ff_nm)` | τ = kp·Δpos + kd·Δvel + tau_ff |
| `disable_mit_mode()` | Exit MIT mode |

### Telemetry

| Method | Returns |
|--------|---------|
| `get_position()` | Current angle in degrees |
| `get_speed()` | Current speed in ERPM |
| `get_current()` | Phase current in Amps |
| `get_temperature()` | Driver board temperature in °C |
| `get_status()` | Full state — logs position / speed / current / temp / error |
| `get_motor_data()` | All fields as a `dict` |

## Known Firmware Limitations

### Velocity loop (0x0303) — not functional on this unit

The CubeMars protocol spec documents a velocity loop mode (arb_id `0x0303`,
payload `int32_be(ERPM)`). On this AK60-6 v3 unit the motor's CAN controller
**does not acknowledge `0x0303` frames at the bus level** — no ACK bit is
pulled, so the Jetson retries indefinitely, fills its TX queue, and enters
ERROR-PASSIVE.

This was tested exhaustively on 2026-03-11 (`scripts/test_velocity_loop.py`):
- Raw CAN, no library, exact spec encoding ✓
- Enable keepalive on `0x03` before each `0x0303` command ✓
- Tested at 1 500 ERPM and 5 000 ERPM ✓
- Unfiltered socket open to catch replies on any arb_id ✓
- **Result: zero ACKs, zero motion, bus enters ERROR-PASSIVE after ~1.7 s**

**Workaround:** use `set_duty_cycle()` for open-loop speed control:

```python
ERPM_PER_PHYS_DEG_S = 18.0   # calibrated 2026-03-10
MAX_VEL_ERPM       = 3000
MAX_DUTY           = 0.25    # 25% max PWM

def erpm_to_duty(erpm: int) -> float:
    duty = erpm / MAX_VEL_ERPM * MAX_DUTY
    return max(-MAX_DUTY, min(MAX_DUTY, duty))

motor.set_duty_cycle(erpm_to_duty(desired_erpm))
```

Feedback (position, ERPM, current, temperature) is still available via the
`0x2903` reply that each duty-cycle frame triggers.

## Velocity Safety

**`set_velocity()` is non-functional on this unit** (see above). For
`set_duty_cycle()` speed control keep duty below `0.25` (25%).

```python
motor.set_duty_cycle(0.15)    # ~1800 ERPM equivalent
motor.set_duty_cycle(-0.15)   # reverse
motor.stop()                  # always works
```

## Position Control

**Range: Unlimited** — motor can rotate continuously for spool-based cable systems.
No artificial position limits. Suitable for applications requiring multiple rotations to wind cable.

## Exosuit Integration

This section describes how to drive the tendon spool motor from an IMU or other
external sensor for use in the lower exosuit.

### Signal chain

```
IMU joint angular velocity (deg/s)
    × ERPM_PER_PHYS_DEG_S          →  desired ERPM
    ÷ MAX_VEL_ERPM × MAX_DUTY      →  duty fraction
    → set_duty_cycle(duty)         →  CAN frame on arb_id=0x03
    ← 0x2903 feedback reply        →  pos / ERPM / current / temp
```

### Minimal example

```python
from motor_python import Motor

ERPM_PER_PHYS_DEG_S = 18.0   # calibrated: 5000 ERPM ≈ 278 deg/s (2026-03-10)
MAX_VEL_ERPM       = 3000    # ERPM ceiling for duty mapping
MAX_DUTY           = 0.25    # hard limit — do not exceed

def joint_vel_to_duty(joint_vel_dps: float) -> float:
    """Convert IMU joint angular velocity (deg/s) to motor duty cycle."""
    erpm = joint_vel_dps * ERPM_PER_PHYS_DEG_S
    duty = erpm / MAX_VEL_ERPM * MAX_DUTY
    return max(-MAX_DUTY, min(MAX_DUTY, duty))

with Motor(motor_can_id=0x03) as motor:
    motor.enable_motor()
    while True:
        joint_vel_dps = read_imu()   # your IMU read function (deg/s)
        motor.set_duty_cycle(joint_vel_to_duty(joint_vel_dps))
```

### Key notes for exosuit use

| Topic | Detail |
|-------|--------|
| **Do NOT use `set_velocity()`** | Sends `0x0303` — not acknowledged by this firmware |
| **Use `set_duty_cycle()`** | Open-loop speed control, works on all firmware versions |
| **Calibration** | `ERPM_PER_PHYS_DEG_S = 18.0` — re-calibrate if motor or cable changes |
| **Direction** | Positive duty = tendon pull. Verify before first run. |
| **Watchdog** | Each `set_duty_cycle()` call feeds the 100 ms CAN watchdog. Loop must run at ≥ 10 Hz. |
| **Feedback** | Every duty frame triggers a `0x2903` reply — use for spool position and safety checks. |
| **Safety guard** | Clamp duty to ±0.25; abort if feedback ERPM exceeds expected maximum. |

### Optional: Jetson-side closed-loop speed correction

If open-loop duty is not accurate enough, add a lightweight P correction
using the ERPM feedback from each `0x2903` reply:

```python
KP = 0.00003   # tune empirically — start small

desired_erpm = joint_vel_dps * ERPM_PER_PHYS_DEG_S
actual_erpm  = motor.get_speed()              # from last 0x2903 reply
error        = desired_erpm - actual_erpm
base_duty    = desired_erpm / MAX_VEL_ERPM * MAX_DUTY
duty         = max(-MAX_DUTY, min(MAX_DUTY, base_duty + error * KP))
motor.set_duty_cycle(duty)
```

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
