# CAN Implementation Journey — CubeMars AK60-6 on Jetson Orin Nano

## Hardware

| Component | Details |
|---|---|
| Controller | Jetson Orin Nano |
| CAN Transceiver | SN65HVD230 |
| Motor | CubeMars AK60-6 (HW V3.0) |
| CAN Interface | `can0` via `mttcan` driver |

**Wiring:**
```
Jetson CAN_TX  →  SN65HVD230 D  (pin 1)
Jetson CAN_RX  →  SN65HVD230 R  (pin 4)
SN65HVD230 CANH (pin 7)  →  Motor CANH
SN65HVD230 CANL (pin 6)  →  Motor CANL
Common GND
3.3V  →  SN65HVD230 Vcc (pin 3)
120Ω termination resistor across CANH/CANL
```

---

## Motor Configuration (CubeMars Software)

| Setting | Value |
|---|---|
| CAN Bitrate | 1 Mbps |
| CAN ID | 3 |
| Feedback Mode | Periodic Feedback |
| Feedback Rate | 50 Hz |

---

## CAN Protocol

### Frame Format

**Control commands** use extended 29-bit CAN IDs:

```
Arbitration ID = (mode << 8) | motor_id
```

For motor ID `0x03`:

| Mode | Mode Byte | Arbitration ID | Description |
|---|---|---|---|
| Duty Cycle | `0x00` | `0x00000003` | Duty cycle (-1 to 1) |
| Current Loop | `0x01` | `0x00000103` | IQ current (torque) |
| Current Brake | `0x02` | `0x00000203` | Hold position with current |
| Velocity | `0x03` | `0x00000303` | ERPM |
| Position | `0x04` | `0x00000403` | Degrees |
| Set Origin | `0x05` | `0x00000503` | Zero position |
| Position+Velocity | `0x06` | `0x00000603` | Trapezoidal profile |

**Special commands** sent directly to motor ID (no mode byte):

| Command | Data | Description |
|---|---|---|
| Enable motor | `FF FF FF FF FF FF FF FC` | Enter Servo mode control |
| Disable motor | `FF FF FF FF FF FF FF FD` | Power off motor |

### Feedback Format

The motor broadcasts feedback at 50 Hz with arbitration ID `0x00002903` (extended).

```
Bytes [0:2]  →  Position  (int16, × 0.1 = degrees)
Bytes [2:4]  →  Speed     (int16, × 10 = ERPM)
Bytes [4:6]  →  Current   (int16, × 0.01 = Amps)
Byte  [6]    →  Temperature (°C)
Byte  [7]    →  Error code
```

### Payload Encoding

| Control Mode | Encoding |
|---|---|
| Velocity | `int32(erpm)` in bytes 0–3 |
| Current | `int32(amps × 1000)` in bytes 0–3 |
| Position | `int32(degrees × 10000)` in bytes 0–3 |
| Position+Vel+Accel | `int32(degrees × 10000)` + `int16(erpm)` + `int16(accel)` |

---

## CAN Interface Setup (Jetson)

### Load Kernel Modules

```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
```

### Bring Up Interface

```bash
sudo ip link set can0 up type can bitrate 1000000 berr-reporting on restart-ms 100
```

> **Critical:** Always use `type can bitrate ... berr-reporting on restart-ms 100`.
> Plain `sudo ip link set can0 up` leaves the interface without error recovery,
> causing it to lock up in ERROR-PASSIVE state after any failed transmission.

### Verify

```bash
ip -details link show can0
```

Healthy output shows `state ERROR-ACTIVE (berr-counter tx 0 rx 0)`.

---

## Key Discoveries

### 1. UART Blocks CAN

The motor has **two interfaces: UART and CAN**. UART has higher priority.

- When a UART cable is connected, the motor **ignores all CAN control commands**.
- The motor continues to broadcast its 50 Hz feedback via CAN even while UART is active.
- CAN commands are transmitted but receive no ACK, causing the Jetson's CAN interface to accumulate TX errors and enter ERROR-PASSIVE state (`berr-counter tx 128+`).

**Fix:** Disconnect UART before using CAN control.

### 2. Extended vs Standard CAN IDs

The motor uses **extended 29-bit CAN IDs** (`is_extended_id=True`), not standard 11-bit IDs.

- Feedback arrives on `0x00002903` (not `0x03`)
- Commands must be sent with `is_extended_id=True`

### 3. ERROR-PASSIVE Recovery

When the interface enters ERROR-PASSIVE due to unACK'd frames, use:

```bash
# Option 1: cangen with polling (NVIDIA recommended)
timeout 5 cangen -L 8 can0 -p 1000

# Option 2: full restart
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 berr-reporting on restart-ms 100
```

With `restart-ms 100`, the interface automatically restarts after 100 ms in BUS-OFF state.

### 4. Two CAN Control Modes

The motor supports two CAN protocols:

| Mode | Enable/Disable | Control |
|---|---|---|
| **Servo mode** | `0xFFFFFFFFFFFFFFFC` / `0xFFFFFFFFFFFFFFFD` | Velocity, position, current, duty cycle |
| **MIT mode** | `0xFFFFFFFFFFFFFFFF` / `0xFFFFFFFFFFFFFFFE` | Impedance control (KP, KD, torque feedforward) |

The motor was configured for Servo mode in CubeMars software.

### 5. candump Echo Confirms TX Works

When monitoring with `candump can0`, you can see your own outgoing frames echoed back (e.g., `00000003 [8] FF FF FF FF FF FF FF FD`). This confirms the Jetson is transmitting — the issue is whether the motor ACKs them.

---

## Files Created

| File | Purpose |
|---|---|
| `src/motor_python/cube_mars_motor_can.py` | Main CAN implementation (`CubeMarsAK606v3CAN` class) |
| `src/motor_python/definitions.py` | Added `CANDefaults` dataclass |
| `src/motor_python/examples_can.py` | Usage examples |
| `src/motor_python/test_can.py` | Unit test suite (5 tests) |
| `setup_can.sh` | Automated CAN interface setup script |
| `quick_test.py` | Feedback reception test |
| `simple_test.py` | Minimal enable/velocity/disable test |
| `test_our_implementation.py` | Full integration test |
| `test_servo_mode.py` | TMotorCANControl library servo mode test |
| `docs/CAN_SETUP_GUIDE.md` | Full hardware + software setup guide |
| `docs/CAN_QUICK_REFERENCE.md` | Quick reference card |
| `docs/CAN_TESTING.md` | Testing methodology |

---

## Current Status

| Item | Status |
|---|---|
| CAN interface configured (1 Mbps, can0) | ✅ |
| Motor feedback receiving (50 Hz, ID 0x2903) | ✅ |
| Commands transmitting (confirmed via candump echo) | ✅ |
| enable/disable commands implemented | ✅ |
| Motor responding to control commands | ⏳ Pending UART disconnect test |
| Code pushed to `CAN_implementation` branch | ✅ |

---

## Next Steps

1. **Disconnect UART cable** from motor
2. Reset interface: `sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 1000000 berr-reporting on restart-ms 100`
3. Clear error state: `timeout 5 cangen -L 8 can0 -p 1000`
4. Run test: `python simple_test.py`
5. If motor moves → merge `CAN_implementation` branch
6. Remove test scripts from repo root before merging PR
