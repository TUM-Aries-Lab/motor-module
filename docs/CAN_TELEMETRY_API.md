# CAN Telemetry & Control API Reference

Class: `CubeMarsAK606v3CAN`  
Module: `src/motor_python/cube_mars_motor_can.py`

All methods below are safe to call while the motor is running.
Telemetry reads are non-blocking instant reads from the feedback cached by the
50 Hz background thread.  If no feedback has arrived yet (e.g. immediately after
`enable_motor()`), a single blocking receive with a 50 ms timeout is used as
a fallback.

---

## Telemetry Methods

### `get_position() → float | None`

Returns the current shaft angle in **degrees**, relative to the position recorded
at the time the motor was enabled (the "home" origin).

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
At rest (duty = 0) this reads approximately 0 A.

```python
cur = motor.get_current()
if cur is not None:
    print(f"Current: {cur:.3f} A")
```

---

### `get_speed() → int | None`

Returns the shaft rotational speed in **electrical RPM** (ERPM) as an integer.

To convert ERPM to mechanical RPM or angular degrees/second:

```
mechanical RPM  = ERPM / pole_pairs
deg/s           = (ERPM / 60) × 360
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

### `set_duty_cycle(duty: float)`

Apply a duty cycle between `-1.0` (full reverse) and `+1.0` (full forward).
This is the **only confirmed working control mode** on the evaluation firmware.

Practical safe limits: `±0.15` (15 % of supply voltage) for controlled arm motion.

```python
motor.set_duty_cycle(0.10)   # gentle forward
time.sleep(2.0)
motor.set_duty_cycle(0.0)    # coast
```

---

### `set_current(current_amps: float)`

Apply a torque-producing current in Amperes.  Positive = forward torque.

Used internally for the **soft-start** pre-spin (3 A for 150 ms) before switching
to velocity mode.

---

### `set_brake_current(current_amps: float)`

Apply **regenerative braking** at the specified current level.

Unlike `set_current()`, which produces torque in a chosen direction, brake current
short-circuits the windings in a controlled manner that converts kinetic energy into
heat and actively slows the shaft.

Instruction type: `0x02` — arbitration ID for motor 3 = `0x00000203`.

Payload encoding: `int32(current_amps × 1000)` big-endian — identical to
regular current control.

```python
motor.set_brake_current(5.0)   # braking force equivalent to 5 A
```

---

### `set_velocity(velocity_erpm: int)`

Command a shaft speed in electrical RPM.

**Note:** On the evaluation unit's firmware, this mode ACKs correctly over CAN but
does **not** produce shaft rotation.  The method is implemented correctly and will
work on units with updated or production firmware.

```python
motor.set_velocity(10000)   # 10 000 ERPM ≈ 476 mechanical RPM
```

---

### `set_position(degrees: float, speed_erpm: int, accel_erpm_s: int)`

Command a target absolute angle with a trapezoidal speed profile.

Position is encoded as `int32(degrees × 10000)`.
Speed and acceleration limits are each divided by 10 before being encoded as
`int16`, as required by the CubeMars CAN spec (section 4.4.1).

Same firmware caveat as `set_velocity()`: ACKs but no shaft rotation on
evaluation unit.

---

### `stop()`

Terminates the background refresh thread and sends a zero-duty command,
causing the motor to coast to a stop.

---

### `enable_motor()` / `disable_motor()`

Special 8-byte command frames that activate / deactivate the motor drive output.
The motor will not respond to any motion command until `enable_motor()` has been
sent.  `disable_motor()` returns the motor to a free-spinning, unpowered state.

---

## Error Codes (`CAN_ERROR_CODES`)

Defined in `cube_mars_motor_can.py` as a module-level dict.

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

## Brake Current Mode vs. Current Control

| | `set_current()` | `set_brake_current()` |
|---|---|---|
| Instruction type byte | `0x01` | `0x02` |
| Arbitration ID (motor 3) | `0x00000103` | `0x00000203` |
| Effect | Applies torque (accelerates or decelerates) | Regenerative braking (always decelerates) |
| Payload encoding | `int32(amps × 1000)` | `int32(amps × 1000)` |
| Sign of value | Positive = forward torque | Magnitude only — always brakes |
| Use case | Soft-start, torque control | Active deceleration under load |

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

| Mode | Byte 0–3 | Byte 4–7 | Arbitration ID (motor 3) |
|---|---|---|---|
| Duty cycle | `int32(duty × 100 000)` | `0x00000000` | `0x00000003` |
| Current | `int32(amps × 1000)` | `0x00000000` | `0x00000103` |
| Brake current | `int32(amps × 1000)` | `0x00000000` | `0x00000203` |
| Velocity | `int32(ERPM)` | `0x00000000` | `0x00000303` |
| Position | `int32(deg × 10000)` | `0x00000000` | `0x00000403` |
| Position+Vel | `int32(deg × 10000)` | `int16(speed÷10)` + `int16(accel÷10)` | `0x00000603` |

All payloads are padded to exactly 8 bytes, big-endian.
