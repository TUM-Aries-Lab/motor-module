# CAN Protocol Quick Reference

## Message Format

### Control Message (Host → Motor)
```
Byte:  [  0  ][  1  ][ 2  ][ 3  ][ 4  ][ 5  ][ 6  ][ 7  ]
Data:  [ 0x00 ][ 0x00 ][ CMD ][ ID  ][      DATA      ]
```

### Feedback Message (Motor → Host)
```
Byte:  [  0-1   ][  2-3   ][  4-5   ][  6  ][  7  ]
Data:  [ POS_INT ][ SPD_INT ][ CUR_INT ][ TEMP ][ ERR ]
       (int16-be) (int16-be) (int16-be) (int8) (uint8)
```

## Control Commands

| Mode | CMD | Data Format | Example Command |
|------|-----|-------------|-----------------|
| **Duty Cycle** | 0x00 | int32 duty×100000 | `00 00 00 68 00 00 4E 20` (0.2 = 20%) |
| **Current Loop** | 0x01 | int32 current÷0.01 | `00 00 01 68 FF FF F0 60` (-4A) |
| **Brake Current** | 0x02 | int32 current÷0.01 | `00 00 02 68 00 00 0F A0` (4A) |
| **Velocity** | 0x03 | int32 speed÷10 | `00 00 03 68 00 00 13 88` (5000 RPM) |
| **Position** | 0x04 | int32 pos÷0.1 | `00 00 04 68 00 5B 8D 80` (600°) |
| **Pos+Vel** | 0x06 | int32 pos + int16 vel + int16 acc | `00 00 06 68 09 98 96 80 03 E8 03 E8` |
| **MIT Velocity** | 0x08 | 0x06 + Kd×100 + vel÷10 | `00 00 08 68 00 06 66 7F FF 8F 57 FF` |
| **MIT Position** | 0x08 | 0x06 + Kp×100 + Kd×100 + pos÷0.1 | `00 00 08 68 01 06 66 BD 70 7F F7 FF` |
| **MIT Torque** | 0x08 | 0x00 + current÷0.01 | `00 00 08 68 00 00 00 7F FF 7F F8 3F` |

## Data Conversion

### Position
```
Real (degrees) = int16_value × 0.1
int16_value = Real (degrees) ÷ 0.1

Range: -3200° to +3200° (-32000 to +32000 int16)
Resolution: 0.1°
```

### Speed
```
Real (RPM) = int16_value × 10.0
int16_value = Real (RPM) ÷ 10.0

Range: -320000 RPM to +320000 RPM (-32000 to +32000 int16)
Resolution: 10 RPM
```

### Current
```
Real (Amps) = int16_value × 0.01
int16_value = Real (Amps) ÷ 0.01

Range: -60A to +60A (-6000 to +6000 int16)
Resolution: 0.01A
```

### Temperature
```
Real (°C) = int8_value × 1.0

Range: -20°C to +127°C (-20 to +127 int8)
Resolution: 1°C
```

## Error Codes

| Code | Meaning |
|------|---------|
| 0 | No fault |
| 1 | Motor over-temperature |
| 2 | Over-current |
| 3 | Over-voltage |
| 4 | Under-voltage |
| 5 | Encoder fault |
| 6 | MOSFET over-temperature |
| 7 | Motor lock-up |

## Example Calculations

### Example 1: Position Command (180°)
```python
position_degrees = 180.0
position_int = int(180.0 / 0.1)  # = 1800
data = struct.pack('>i', 1800)  # Big-endian int32
# Result: 0x00 0x00 0x07 0x08
message = [0x00, 0x00, 0x04, 0x68, 0x00, 0x00, 0x07, 0x08]
```

### Example 2: Velocity Command (1000 RPM)
```python
velocity_rpm = 1000.0
velocity_int = int(1000.0 / 10.0)  # = 100
data = struct.pack('>i', 100)  # Big-endian int32
# Result: 0x00 0x00 0x00 0x64
message = [0x00, 0x00, 0x03, 0x68, 0x00, 0x00, 0x00, 0x64]
```

### Example 3: Current Command (2.5A)
```python
current_amps = 2.5
current_int = int(2.5 / 0.01)  # = 250
data = struct.pack('>i', 250)  # Big-endian int32
# Result: 0x00 0x00 0x00 0xFA
message = [0x00, 0x00, 0x01, 0x68, 0x00, 0x00, 0x00, 0xFA]
```

### Example 4: Parse Feedback
```python
feedback_data = bytes([
    0x03, 0xE8,  # Position: 1000 → 100.0°
    0x00, 0x64,  # Speed: 100 → 1000 RPM
    0x00, 0xFA,  # Current: 250 → 2.5A
    0x19,        # Temperature: 25°C
    0x00         # Error: No fault
])

position = struct.unpack('>h', feedback_data[0:2])[0] * 0.1  # = 100.0°
speed = struct.unpack('>h', feedback_data[2:4])[0] * 10.0    # = 1000 RPM
current = struct.unpack('>h', feedback_data[4:6])[0] * 0.01  # = 2.5A
temp = struct.unpack('>b', feedback_data[6:7])[0]            # = 25
error = feedback_data[7]                                      # = 0
```

## Python Code Snippets

### Send Position Command
```python
import struct
from motor_python import CubeMarsAK606CAN

motor = CubeMarsAK606CAN(motor_id=0x68)
motor.set_position(90.0)  # Move to 90 degrees
```

### Receive and Parse Feedback
```python
status = motor.receive_feedback(timeout=0.5)
if status:
    print(f"Position: {status.position:.2f}°")
    print(f"Speed: {status.speed:.0f} RPM")
    print(f"Current: {status.current:.2f} A")
    print(f"Temperature: {status.temperature}°C")
    print(f"Error: {status.error_code}")
```

### Continuous Monitoring
```python
with CubeMarsAK606CAN(motor_id=0x68) as motor:
    for _ in range(100):
        status = motor.receive_feedback(timeout=0.1)
        if status:
            print(f"Pos: {status.position:7.2f}° | "
                  f"Spd: {status.speed:7.0f} RPM")
```

## CAN Bus Commands (Linux)

### Configure Interface
```bash
# Set bitrate and bring up
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Verify status
ip link show can0
```

### Monitor Traffic
```bash
# Show all CAN messages
candump can0

# Filter by ID
candump can0,068:7FF

# Show with timestamp
candump -t a can0
```

### Send Test Message
```bash
# Format: cansend <interface> <id>#<data>
# Example: Set motor 0x68 to 0% duty cycle
cansend can0 068#0000006800000000
```

### Troubleshooting
```bash
# Check for errors
ip -details -statistics link show can0

# Reset interface
sudo ip link set can0 down
sudo ip link set can0 up

# Check kernel modules
lsmod | grep can
```

## Hardware Setup Checklist

- [ ] SPI enabled on Jetson (`ls /dev/spidev*`)
- [ ] CAN modules loaded (`lsmod | grep can`)
- [ ] CAN interface configured (`ip link show can0`)
- [ ] CAN interface up (`UP` status)
- [ ] 120Ω termination at both ends of bus
- [ ] CAN_H and CAN_L properly wired
- [ ] Common ground between devices
- [ ] Motor powered on
- [ ] python-can installed (`pip list | grep can`)
- [ ] Correct motor ID configured

## Common Issues & Solutions

**Issue**: `RTNETLINK answers: Operation not supported`
- **Solution**: Load CAN modules: `sudo modprobe can && sudo modprobe can_raw`

**Issue**: No feedback from motor
- **Solution**: Check wiring, power, termination resistors

**Issue**: `ImportError: No module named 'can'`
- **Solution**: Install python-can: `pip install python-can`

**Issue**: Permission denied on /dev/spidev
- **Solution**: `sudo chmod 666 /dev/spidev*` or add user to spi group

**Issue**: CAN bus error frames
- **Solution**: Verify baudrate (500kbps), check termination, inspect wiring
