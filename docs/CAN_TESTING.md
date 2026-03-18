# CAN Communication Testing Guide (MIT-Only)

This guide is the **current** bench procedure for `CubeMarsAK606v3CAN`.

- Hardware: Jetson Orin Nano + SN65HVD230 + CubeMars AK60-6
- Transport: SocketCAN (`can0`, 1 Mbps)
- Control protocol: **Force Control Mode (MIT)** only (`mode_id=0x08`)

---

## Prerequisites

### 1) Hardware

- [ ] SN65HVD230 wired correctly to Jetson CAN pins
- [ ] CANH/CANL connected to motor (not swapped)
- [ ] 120 Ω bus termination verified (~60 Ω across CANH/CANL when powered off)
- [ ] Motor powered from 24 V supply
- [ ] Common ground between Jetson, transceiver, and motor
- [ ] UART/R-Link cable physically disconnected from motor

### 2) Motor Configuration (CubeMars Tool)

- [ ] CAN Mode: Periodic Feedback
- [ ] CAN Bitrate: 1 Mbps
- [ ] CAN ID: `0x03`
- [ ] Feedback Rate: 50 Hz
- [ ] Config saved to motor

### 3) Jetson CAN Interface

```bash
sudo ./setup_can.sh
ip -details link show can0
```

Target state should include `ERROR-ACTIVE`.

---

## Test Methods

### Method 1: Bus Sanity With can-utils

```bash
candump can0
```

You should see feedback frames from the motor once commands are sent.

Optional frame injection test:

```bash
cansend can0 123#DEADBEEF
```

---

### Method 2: MIT API Smoke Test (recommended)

Run the one-motor MIT bench script:

```bash
.venv/bin/python scripts/mit_mode_test.py --motor-id 0x03
```

This script exercises:

1. `check_communication()`
2. `enable_mit_mode()`
3. Direct `set_mit_mode(...)` calls (passive, position, velocity, torque)
4. MIT-backed helpers: `set_position()`, `set_velocity()`, `set_current()`
5. `stop()`
6. Alias checks: `enable_motor()` / `disable_motor()`
7. `disable_mit_mode()`

---

### Method 3: Minimal Interactive Check

```python
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN
import time

motor = CubeMarsAK606v3CAN(motor_can_id=0x03, interface="can0", bitrate=1_000_000)
assert motor.connected
assert motor.check_communication()

motor.enable_mit_mode()
motor.set_mit_mode(pos_rad=0.0, vel_rad_s=2.0, kp=0.0, kd=2.0, torque_ff_nm=0.0)
time.sleep(1.0)
print(motor.get_status())

motor.stop()
motor.close()
```

---

## Current API Behavior (important)

MIT-only over CAN:

- Supported:
  - `enable_mit_mode`, `set_mit_mode`, `disable_mit_mode`
  - `set_position`, `set_velocity`, `set_current` (MIT-backed helpers)
  - `stop`, `check_communication`, telemetry getters
- Not implemented (raises `NotImplementedError`):
  - `set_duty_cycle`
  - `set_brake_current`
  - `set_origin`
  - `set_position_velocity_accel`

---

## Troubleshooting Quick Checks

### 1) No CAN interface

- Run `ip link show can0`
- Re-run `sudo ./setup_can.sh`

### 2) No motor response

- Confirm PSU is 24 V
- Confirm UART cable is unplugged
- Confirm CAN ID in motor tool is `0x03`
- Confirm can0 bitrate is 1 Mbps

### 3) Frames seen, but no movement

- Re-check PSU voltage/current under command (under-voltage lockout can look like healthy CAN)
- Re-run MIT script with low torque/velocity values first

### 4) BUS-OFF / ERROR-PASSIVE

```bash
sudo ./setup_can.sh
ip -details -statistics link show can0
```

---

## Expected Outcome

A successful MIT bench run should show:

- Stable feedback reception
- No CAN transport exceptions
- Motor response to MIT commands
- Clean stop (`stop()` + `disable_mit_mode()`)
