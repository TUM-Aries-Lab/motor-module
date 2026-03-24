# CAN Telemetry & Control API Reference

Class: `CubeMarsAK606v3CAN`  
Module: `src/motor_python/cube_mars_motor_can.py`

> **Current implementation status (2026-03-18): MIT-only over CAN.**
> Legacy servo transport commands (`set_duty_cycle`, `set_brake_current`,
> `set_origin`, `set_position_velocity_accel`) are intentionally disabled and
> raise `NotImplementedError`.

All methods below are safe to call while the motor is running.
Telemetry getters return cached feedback when available and otherwise wait for
new feedback up to the method timeout (`~0.2 s` for `get_position()`,
`~0.5 s` for `get_status()`).

---

## Telemetry Methods

### `get_position() → float | None`

Returns the current shaft angle in **degrees** from the motor feedback frame.
The absolute reference depends on the motor-side origin configuration.

- Positive values: rotation in the forward direction
- Negative values: rotation in the reverse direction
- Returns `None` if no feedback has been received

```python
pos = motor.get_position()
if pos is not None:
    print(f"Shaft is at {pos:.2f}°")
```

---

### `get_temperature() → int | None`

Returns the motor driver board temperature in **°C** as an integer.

Typical values: 25–45 °C at rest, up to ~80 °C under heavy sustained load.
The motor firmware triggers an over-temperature fault (error code 1) if the
limit is exceeded.

```python
temp = motor.get_temperature()
if temp is not None:
    print(f"Driver temp: {temp} °C")
```

---

### `get_current() → float | None`

Returns the winding current in **Amperes** (floating-point).

Current is proportional to torque.  Positive = forward torque, negative = reverse.
At neutral MIT command (`kp=kd=tau=0`) this reads approximately 0 A.

```python
cur = motor.get_current()
if cur is not None:
    print(f"Current: {cur:.3f} A")
```

---

### `get_speed() → int | None`

Returns the shaft rotational speed in **electrical RPM** (ERPM) as an integer.

To convert ERPM to mechanical RPM or mechanical angular speed:

```
mechanical RPM  = ERPM / pole_pairs
deg/s           = (ERPM / pole_pairs) × 6
```

The AK60-6 has **21 pole pairs**, so:

```
mechanical RPM  = ERPM / 21
```

```python
speed = motor.get_speed()
if speed is not None:
    print(f"Speed: {speed} ERPM  ({speed / 21:.0f} mechanical RPM)")
```

---

### `get_motor_data() → dict | None`

Returns all five feedback fields in a single dictionary, plus a human-readable
error description.

| Key | Type | Unit |
|---|---|---|
| `position_degrees` | `float` | degrees |
| `speed_erpm` | `int` | electrical RPM |
| `current_amps` | `float` | Amperes |
| `temperature_celsius` | `int` | °C |
| `error_code` | `int` | 0 = no fault |
| `error_description` | `str` | human-readable fault string |

```python
data = motor.get_motor_data()
if data:
    print(data)
    # {'position_degrees': 42.3, 'speed_erpm': 0, 'current_amps': 0.0,
    #  'temperature_celsius': 33, 'error_code': 0, 'error_description': 'No fault'}
```

---

## Control Methods

### `set_current(current_amps: float)`

MIT-only compatibility helper.

The argument is currently interpreted as **torque feedforward in Nm** and routed
to `set_mit_mode(..., kp=0, kd=0, torque_ff_nm=current_amps)`.

---

### `set_brake_current(current_amps: float)`

Not implemented in MIT-only transport. Raises `NotImplementedError`.

---

### `set_velocity(velocity_erpm: int)`

MIT-backed helper. Converts ERPM to rad/s and calls:

`set_mit_mode(pos_rad=0, vel_rad_s=..., kp=0, kd=default_velocity_kd, torque_ff_nm=0)`

The BaseMotor safety check still blocks very low non-zero ERPM unless
`allow_low_speed=True` is passed.

```python
motor.set_velocity(10000)                      # 10 000 ERPM ≈ 476 mechanical RPM
motor.set_velocity(-8000)                      # reverse
motor.set_velocity(2000, allow_low_speed=True) # slow speed (noisy, use with caution)
```

---

### `set_position(position_degrees: float)`

MIT-backed helper. Converts degrees → radians and calls:

`set_mit_mode(pos_rad=..., vel_rad_s=0, kp=default_position_kp, kd=default_position_kd, torque_ff_nm=0)`

```python
motor.set_position(90.0)   # move to 90 degrees and hold
```

---

### `set_position_velocity_accel(position_degrees, velocity_erpm, accel_erpm_per_sec)`

Not implemented in MIT-only transport. Raises `NotImplementedError`.

---

### `set_duty_cycle(duty: float)`

Not implemented in MIT-only transport. Raises `NotImplementedError`.

---

### `set_mit_mode(pos_rad, vel_rad_s, kp, kd, torque_ff_nm)`

Impedance control via the MIT actuator protocol.  The output torque is:

```
τ = kp × (pos_target − pos_actual) + kd × (vel_target − vel_actual) + tau_ff
```

`set_mit_mode()` auto-enables MIT mode if needed. Calling `enable_mit_mode()`
explicitly is still recommended for clear startup sequencing.

Use cases:

| Goal | kp | kd | tau_ff | vel |
|---|---|---|---|---|
| Spring position hold | > 0 | > 0 | 0 | 0 |
| Velocity damping | 0 | > 0 | 0 | desired |
| Pure torque | 0 | 0 | > 0 | 0 |
| Passive float | 0 | 0 | 0 | 0 |

AK60-6 physical limits (automatically clamped):

Manual nuance: the sample C snippet in section 4.2 is labeled
`AK10-9 as an example` and uses AK10-9 constants there. This driver uses the
AK60-6 range table values below.

| Parameter | Range | Unit |
|---|---|---|
| `pos_rad` | −12.56 … +12.56 | rad |
| `vel_rad_s` | −60 … +60 | rad/s |
| `kp` | 0 … 500 | Nm/rad |
| `kd` | 0 … 5 | Nms/rad |
| `torque_ff_nm` | −12 … +12 | Nm |

```python
motor.enable_mit_mode()
motor.set_mit_mode(pos_rad=1.57, kp=100, kd=2)          # spring to 90°
motor.set_mit_mode(pos_rad=0, vel_rad_s=3.0, kd=1.5)    # velocity damping
motor.set_mit_mode(pos_rad=0, torque_ff_nm=5.0)          # torque only
motor.disable_mit_mode()
```

---

### `enable_mit_mode()` / `disable_mit_mode()`

Driver compatibility helpers used by this repository before/after MIT commands.

Manual note: section 4.2 defines Force Control command frames on
`(0x08 << 8) | motor_id` with the 8-byte MIT payload, but does not explicitly
list a dedicated enable/disable frame sequence.

| Driver helper action | Byte sequence | Arb ID |
|---|---|---|
| `enable_mit_mode()` | `FF FF FF FF FF FF FF FC` | `motor_id` |
| `disable_mit_mode()` | `FF FF FF FF FF FF FF FD` | `motor_id` |

---

### `stop()`

Stops the background refresh thread, sends a neutral MIT command
(`kp=kd=tau=0`), then sends MIT disable.

---

### `enable_motor()` / `disable_motor()`

Compatibility aliases in this MIT-only implementation:

- `enable_motor()` calls `enable_mit_mode()`
- `disable_motor()` calls `disable_mit_mode()`

---

## Error Codes (`CAN_ERROR_CODES`)

Defined in `src/motor_python/base_motor.py` as `CAN_ERROR_CODES`.

| Code | Description |
|---|---|
| `0` | No fault — normal operation |
| `1` | Over-temperature: driver temperature exceeds safe limit |
| `2` | Over-current: winding current exceeds protection threshold |
| `3` | Over-voltage: supply voltage exceeds maximum |
| `4` | Under-voltage: supply voltage below minimum |
| `5` | Encoder fault: position sensor error or loss of signal |
| `6` | Hardware protection triggered |
| `7` | Reserved |

Source: CubeMars CAN protocol specification, section 4.3.1.

---

## Legacy Servo Modes

The current CAN implementation intentionally disables legacy servo transport
modes (`duty/current/brake/position-velocity profile`) and only transmits
Force Control Mode (`mode_id = 0x08`) frames.

---

## Feedback Byte Layout (from motor spec, section 4.3.1)

```
Byte offset  Length  Type    Scale    Unit
───────────  ──────  ──────  ───────  ─────────────
0–1          2       int16   × 0.1    degrees
2–3          2       int16   × 10     electrical RPM
4–5          2       int16   × 0.01   Amperes
6            1       int8    × 1      °C
7            1       uint8   —        error code
```

All values are big-endian (most significant byte first).

---

## Control Payload Layout (from motor spec, section 4.4.1)

This implementation is MIT-only (Force Control Mode, mode ID `0x08`).

| Mode | Payload | Arbitration ID (motor 3) |
|---|---|---|
| MIT force control | bit-packed (manual layout below) | `0x00000803` |

**MIT 8-byte payload layout used here (CubeMars manual):**

| Byte | Bits | Field |
|---|---|---|
| `DATA[0]` | `7..0` | `kp[11:4]` |
| `DATA[1]` | `7..4` | `kp[3:0]` |
| `DATA[1]` | `3..0` | `kd[11:8]` |
| `DATA[2]` | `7..0` | `kd[7:0]` |
| `DATA[3]` | `7..0` | `pos[15:8]` |
| `DATA[4]` | `7..0` | `pos[7:0]` |
| `DATA[5]` | `7..0` | `vel[11:4]` |
| `DATA[6]` | `7..4` | `vel[3:0]` |
| `DATA[6]` | `3..0` | `tau[11:8]` |
| `DATA[7]` | `7..0` | `tau[7:0]` |
