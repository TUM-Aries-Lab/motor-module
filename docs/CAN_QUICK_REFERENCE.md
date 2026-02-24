# Quick Reference: CAN Setup on Jetson Orin Nano

## Hardware Wiring (SN65HVD230 Transceiver)

```
Jetson Pin    →  SN65HVD230 Pin   →  Motor
══════════       ══════════════       ═════
Pin 22 (CAN0_TX) → Pin 1 (D)
Pin 24 (CAN0_RX) ← Pin 4 (R)
Pin 17 (3.3V)    → Pin 3 (Vcc)
Pin 20 (GND)     → Pin 2 (GND)
                   Pin 8 (Rs) → GND
                   Pin 7 (CANH) → Motor CANH
                   Pin 6 (CANL) → Motor CANL
```

**Don't forget:** 120Ω resistor across CANH and CANL!

## Enable CAN Interface (One-Time Setup)

```bash
# CRITICAL: berr-reporting + restart-ms 100 prevents the interface from locking
# up in ERROR-PASSIVE/BUS-OFF state when TX frames go unACK'd.
sudo ip link set can0 up type can bitrate 1000000 berr-reporting on restart-ms 100
```

## Verify CAN is Working

```bash
# Check interface status
ip -details link show can0

# Monitor CAN traffic
candump can0

# Send test frame (in another terminal)
cansend can0 003#DEADBEEF
```

## Python Code

```python
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

# Single motor
motor = CubeMarsAK606v3CAN(motor_can_id=0x03)
motor.set_velocity(velocity_erpm=10000)
motor.stop()
motor.close()

# Multiple motors (unique CAN IDs)
motor1 = CubeMarsAK606v3CAN(motor_can_id=0x03)
motor2 = CubeMarsAK606v3CAN(motor_can_id=0x04)
```

## Motor Configuration (CubeMars Software)

- **CAN Bitrate:** 1 Mbps
- **CAN ID:** 3 (or unique ID per motor, 0x03-0xFF)
- **CAN Mode:** Periodic Feedback
- **CAN Fdb Rate:** 50 Hz (1-500 Hz range)

## CAN Protocol Format

**Extended CAN IDs (29-bit)**: `00 00 0M XX` where M=mode, XX=motor ID
- Mode 01: Current control
- Mode 03: Velocity control  
- Mode 04: Position control

**Command Payloads** (8 bytes, big-endian):
- Current: int32 milliamps (A × 1000)
- Velocity: int32 ERPM (direct value)
- Position: int32 (degrees × 10000)

**Feedback** (8 bytes, auto-sent at configured rate):
- Position: int16 (×0.1°), Speed: int16 (×10 ERPM)
- Current: int16 (×0.01 A), Temp: int8 (°C), Error: uint8

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `No such device can0` | Run `sudo ip link set can0 up` |
| Motor not responding | Check power, CANH/CANL wiring, termination resistor |
| Bus-off errors | Check termination, verify bitrate matches (1 Mbps) |
| Permission denied | Add user to dialout group or use sudo |

## Pin Reference (Jetson Orin Nano 40-pin Header)

```
     3V3  [1] [2]  5V
   GPIO2  [3] [4]  5V
   GPIO3  [5] [6]  GND
   GPIO4  [7] [8]  GPIO14
     GND  [9] [10] GPIO15
  GPIO17 [11] [12] GPIO18
  GPIO27 [13] [14] GND
  GPIO22 [15] [16] GPIO23
    3V3  [17] [18] GPIO24
  GPIO10 [19] [20] GND ← GND for transceiver
   GPIO9 [21] [22] GPIO25 ← CAN0_TX
  GPIO11 [23] [24] GPIO8  ← CAN0_RX
     GND [25] [26] GPIO7
   GPIO0 [27] [28] GPIO1
   GPIO5 [29] [30] GND
   GPIO6 [31] [32] GPIO12
  GPIO13 [33] [34] GND
CAN1_RX [35] [36] GPIO16
CAN1_TX [37] [38] GPIO20
     GND [39] [40] GPIO21
```

Legend:
- Pin 17: 3.3V power
- Pin 20: GND
- Pin 22: CAN0_TX
- Pin 24: CAN0_RX
- Pin 35/37: CAN1 (alternative interface)
