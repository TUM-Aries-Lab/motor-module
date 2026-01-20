# CAN Communication Setup for AK60-6 Motor

This guide explains how to use the CAN interface for controlling the AK60-6 motor via the Waveshare RS485 CAN expansion board.

## Hardware Requirements

- Jetson Nano/Orin (or compatible)
- Waveshare RS485 CAN expansion board (with MCP2515 CAN controller)
- AK60-6 Motor with CAN interface
- Proper CAN wiring (CAN_H, CAN_L, GND)

## Hardware Setup

### 1. Enable SPI on Jetson

The MCP2515 CAN controller uses SPI interface. Enable SPI using jetson-io:

```bash
# Add spidev to modules
sudo nano /etc/modules-load.d/modules.conf
# Add this line:
spidev
# Save and exit (Ctrl+X, Y, Enter)

# Run jetson-io to configure pins
sudo /opt/nvidia/jetson-io/jetson-io.py
```

In jetson-io:
1. Select "Configure 40-pin expansion header"
2. Select and enable "SPI1"
3. Save and reboot

### 2. Verify SPI Device

After reboot, verify SPI device is available:

```bash
ls /dev/spidev*
# Should show: /dev/spidev0.0 /dev/spidev0.1
```

## Software Setup

### 1. Install Required Libraries

```bash
# Install spidev Python library
pip install spidev

# OR install motor_python with CAN support
pip install motor_python[can]
```

### 2. Download Waveshare Sample Code (Optional)

```bash
# Download Waveshare sample code for testing
cd ~
wget https://files.waveshare.com/upload/7/77/RS485_CAN_for_JetsonNano_Code.zip
unzip RS485_CAN_for_JetsonNano_Code.zip -d RS485_CAN_for_JetsonNano_Code/
```

### 3. Test CAN with Waveshare Demo

```bash
cd ~/RS485_CAN_for_JetsonNano_Code
sudo python cantest.py
```

## Python Usage

### Basic Example

```python
from motor_python import CubeMarsAK606CAN
from loguru import logger

# Initialize motor with CAN ID 0x68
motor = CubeMarsAK606CAN(motor_id=0x68)

# Check connection
if not motor.connected:
    logger.error("Failed to connect to CAN bus")
    exit(1)

# Receive motor feedback
status = motor.receive_feedback(timeout=1.0)
if status:
    logger.info(f"Position: {status.position:.2f}°")
    logger.info(f"Speed: {status.speed:.0f} RPM")
    logger.info(f"Current: {status.current:.2f} A")
    logger.info(f"Temperature: {status.temperature}°C")

# Control motor position
motor.set_position(90.0)  # Move to 90 degrees

# Control motor velocity
motor.set_velocity(1000.0)  # 1000 RPM

# Control motor current
motor.set_current(2.0)  # 2 Amps

# Stop motor
motor.set_current(0.0)

# Close connection
motor.close()
```

### Context Manager Usage

```python
from motor_python import CubeMarsAK606CAN

# Using context manager (recommended)
with CubeMarsAK606CAN(motor_id=0x68) as motor:
    if motor.connected:
        motor.set_position(180.0)
        status = motor.receive_feedback()
        # Connection automatically closed when exiting context
```

### Run Built-in Test

```bash
# Run with CAN interface (default motor ID 0x68)
python -m motor_python --interface can

# Specify custom motor ID
python -m motor_python --interface can --motor-id 0x69

# Run example script
python -m motor_python.examples_can
```

## CAN Protocol Details

### Message Format

The motor uses **Extended CAN frames** with 8 bytes of data.

**Control Message Format** (Host → Motor):
```
[00 00 CMD ID DATA[0] DATA[1] DATA[2] DATA[3]]
```

**Feedback Message Format** (Motor → Host):
```
Data[0-1]: Position (int16, big-endian) → multiply by 0.1 = degrees
Data[2-3]: Speed (int16, big-endian) → multiply by 10.0 = RPM
Data[4-5]: Current (int16, big-endian) → multiply by 0.01 = Amps
Data[6]:   Temperature (int8) = °C
Data[7]:   Error code (uint8)
```

### Control Commands

| Command | ID | Description |
|---------|-----|-------------|
| Duty Cycle | 0x00 | Set motor duty cycle (-95% to 95%) |
| Current Loop | 0x01 | Set IQ current (-60A to 60A) |
| Brake Current | 0x02 | Set brake current |
| Velocity Loop | 0x03 | Set velocity (-100000 to 100000 RPM) |
| Position Loop | 0x04 | Set position (-3200° to 3200°) |
| Position-Velocity | 0x06 | Set position with velocity/accel limits |
| MIT Velocity | 0x08 | Set velocity with MIT gains |
| MIT Position | 0x08 | Set position with MIT gains |
| MIT Torque | 0x08 | Set torque with MIT gains |

### Data Scaling

| Parameter | Type | Range (Raw) | Range (Real) | Scale Factor |
|-----------|------|-------------|--------------|--------------|
| Position | int16 | -32000 to 32000 | -3200° to 3200° | × 0.1 |
| Speed | int16 | -32000 to 32000 | -320000 to 320000 RPM | × 10.0 |
| Current | int16 | -6000 to 6000 | -60 to 60 A | × 0.01 |
| Temperature | int8 | -20 to 127 | -20°C to 127°C | × 1.0 |

### Error Codes

| Code | Description |
|------|-------------|
| 0 | No fault |
| 1 | Motor over-temperature |
| 2 | Over-current |
| 3 | Over-voltage |
| 4 | Under-voltage |
| 5 | Encoder fault |
| 6 | MOSFET over-temperature |
| 7 | Motor lock-up |

## Troubleshooting

### CAN interface not found

```bash
# Check if SPI is enabled
ls /dev/spidev*

# Check CAN modules
lsmod | grep can

# Load CAN modules if needed
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mcp251x
```

### No motor feedback

1. Check CAN wiring (CAN_H, CAN_L, GND)
2. Verify motor is powered on
3. Check CAN termination resistors (120Ω at each end)
4. Monitor CAN bus traffic: `candump can0`
5. Verify motor CAN ID matches your configuration

### Permission denied on /dev/spidev

```bash
# Add user to spi group
sudo usermod -a -G spi $USER
sudo usermod -a -G gpio $USER

# Set permissions
sudo chmod 666 /dev/spidev*

# Reboot or re-login for group changes to take effect
```

### Python-can not found

```bash
pip install python-can
```

## API Reference

### CubeMarsAK606CAN Class

#### Initialization

```python
motor = CubeMarsAK606CAN(
    motor_id=0x68,              # Motor CAN ID
    spi_device="/dev/spidev0.0", # SPI device path
    can_baudrate=500000,         # CAN baudrate in bps
    oscillator_freq=8000000      # MCP2515 oscillator frequency
)
```

#### Methods

**Control Methods:**
- `set_duty_cycle(duty: float)` - Set duty cycle (-0.95 to 0.95)
- `set_current(current_amps: float)` - Set IQ current (-60 to 60 A)
- `set_brake_current(current_amps: float)` - Set brake current
- `set_velocity(velocity_rpm: float)` - Set velocity (-100000 to 100000 RPM)
- `set_position(position_degrees: float)` - Set position (-3200 to 3200°)
- `set_position_velocity(pos, vel, accel)` - Position with velocity/accel limits
- `set_mit_position(pos, kp, kd)` - MIT position control
- `set_mit_velocity(vel, kd)` - MIT velocity control
- `set_mit_torque(current)` - MIT torque control

**Feedback Methods:**
- `receive_feedback(timeout=0.1)` - Receive motor status (returns MotorCANStatus)
- `check_communication()` - Verify motor is responding

**Connection Methods:**
- `close()` - Close CAN connection
- Context manager support: `with CubeMarsAK606CAN(...) as motor:`

#### MotorCANStatus

```python
@dataclass
class MotorCANStatus:
    position: float      # Position in degrees
    speed: float         # Speed in RPM (electrical)
    current: float       # Current in amps
    temperature: int     # Temperature in °C
    error_code: int      # Error code (0 = no fault)
```

## Performance Notes

- **Maximum CAN baudrate:** 500 Kbps (hardware limitation)
- **Motor feedback rate:** Configurable 1-500 Hz (set on motor)
- **Typical feedback rate:** 100-200 Hz
- **Latency:** ~5-10 ms typical

## References

- [Waveshare RS485 CAN for Jetson Nano Wiki](https://www.waveshare.com/wiki/RS485_CAN_for_Jetson_Nano)
- [MCP2515 CAN Controller Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf)
- [Python-CAN Documentation](https://python-can.readthedocs.io/)
- [CAN Bus Specification](https://en.wikipedia.org/wiki/CAN_bus)
