# CAN Bus Setup Guide for Jetson Orin Nano

This guide explains how to set up CAN communication between the Jetson Orin Nano and the CubeMars AK60-6 motor using the SN65HVD230 CAN transceiver.

## Hardware Requirements

1. **Jetson Orin Nano Developer Kit**
2. **SN65HVD230 CAN Transceiver Module** (3.3V compatible)
3. **CubeMars AK60-6 Motor** (configured for CAN communication)
4. **120Ω Termination Resistor** (across CANH and CANL)
5. **Jumper wires** and breadboard/PCB for connections

## Hardware Connections

### SN65HVD230 Pin Configuration

The SN65HVD230 is an 8-pin CAN transceiver. Pin assignments:

| Pin | Name | Function |
|-----|------|----------|
| 1   | D    | CAN TX (Data input from microcontroller) |
| 2   | GND  | Ground |
| 3   | Vcc  | Power supply (3.3V) |
| 4   | R    | CAN RX (Data output to microcontroller) |
| 5   | Vref | Reference voltage (leave floating or 3.3V) |
| 6   | CANL | CAN Low signal |
| 7   | CANH | CAN High signal |
| 8   | Rs   | Slope control (connect to GND for high speed) |

### Jetson Orin Nano to SN65HVD230 Wiring

Connect the following pins on the Jetson Orin Nano 40-pin header:

```
Jetson Orin Nano          SN65HVD230 Transceiver
================          ======================
Pin 22 (CAN0_TX)    -->   Pin 1 (D - TX Input)
Pin 24 (CAN0_RX)    <--   Pin 4 (R - RX Output)
Pin 17 (3.3V)       -->   Pin 3 (Vcc)
Pin 20 (GND)        -->   Pin 2 (GND)
                          Pin 8 (Rs) --> GND (for high-speed mode)
```

**Alternative CAN Interface:**
- For CAN1, use Pin 37 (CAN1_TX) and Pin 35 (CAN1_RX)

### SN65HVD230 to Motor Wiring

```
SN65HVD230              Motor CAN Connector
==========              ===================
Pin 7 (CANH)      -->   CANH
Pin 6 (CANL)      -->   CANL
Pin 2 (GND)       -->   GND (common ground)
```

### Termination Resistor

**IMPORTANT:** Install a 120Ω resistor across CANH and CANL at each end of the CAN bus.

```
     CANH ----[120Ω resistor]---- CANL
```

For a single motor setup, place the termination resistor either:
- At the motor end (across motor CANH/CANL)
- At the transceiver end (across SN65HVD230 pins 6 and 7)

## Software Setup

### 1. Enable CAN Interface on Jetson

The Jetson Orin Nano has two CAN controllers (can0 and can1). Enable them using device tree overlays or manually configure SocketCAN.

#### Option A: Configure at Boot (Persistent)

Edit `/etc/network/interfaces` and add:

```bash
auto can0
iface can0 inet manual
    pre-up ip link set can0 type can bitrate 1000000
    up ifconfig can0 up
    down ifconfig can0 down
```

#### Option B: Manual Configuration (Temporary)

Run these commands after each boot:

```bash
# Bring down the interface (if already up)
sudo ip link set can0 down

# Configure CAN bitrate to 1 Mbps (matches motor config)
sudo ip link set can0 type can bitrate 1000000

# Bring up the interface
sudo ip link set can0 up

# Verify configuration
ip -details -statistics link show can0
```

### 2. Install Required Python Packages

The `python-can` library is already included in the dependencies:

```bash
# Activate your virtual environment
source .venv/bin/activate

# Install/update dependencies
uv pip install -e .
```

### 3. Verify CAN Bus Operation

#### Using `cansend` and `candump` (Testing)

Install CAN utilities:

```bash
sudo apt-get update
sudo apt-get install can-utils
```

Test the CAN interface:

```bash
# Terminal 1: Monitor CAN traffic
candump can0

# Terminal 2: Send a test frame
cansend can0 123#DEADBEEF
```

You should see the frame appear in the candump output.

## Motor CAN Configuration

Configure the motor using the CubeMars configuration software (Windows):

1. **CAN Mode:** Periodic Feedback
2. **CAN Bitrate:** 1 Mbps (1000000 bps)
3. **CAN ID:** 3 (or your preferred ID, 0x03-0xFF)
4. **CAN Feedback Rate:** 50 Hz (1-500 Hz supported)
5. **Save configuration to motor**

### CAN Protocol Details

The motor uses **native CAN protocol** (not UART-over-CAN):
- **Extended 29-bit CAN IDs** encode control mode: `00 00 0M XX`
  - `M` = control mode (01=current, 03=velocity, 04=position, etc.)
  - `XX` = motor CAN ID (e.g., 0x03)
- **8-byte payloads** contain command data (int32 values)
- **Periodic feedback** auto-transmitted at configured rate:
  - Byte 0-1: Position (int16, ×0.1° scale, -3200° to 3200°)
  - Byte 2-3: Speed (int16, ×10 ERPM scale, -320k to 320k)
  - Byte 4-5: Current (int16, ×0.01 A scale, -60 to 60 A)
  - Byte 6: Temperature (int8, °C, -20 to 127)
  - Byte 7: Error code (uint8)

### Command Data Formats

| Command | CAN ID (for motor 0x68) | Data Format | Example |
|---------|------------------------|-------------|----------|
| Current | 0x00000168 | int32 (mA) | -4.4 A = 0xFFFFEED8 |
| Velocity | 0x00000368 | int32 (ERPM) | 5000 = 0x00001388 |
| Position | 0x00000468 | int32 (deg×10000) | 600° = 0x005B8D80 |

## Python Usage

### Basic CAN Motor Control

```python
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

# Initialize motor with CAN ID 3 (from motor config)
motor = CubeMarsAK606v3CAN(motor_can_id=0x03, interface="can0", bitrate=1000000)

# Check communication
if motor.check_communication():
    print("Motor is responding!")

    # Velocity control
    motor.set_velocity(velocity_erpm=10000)   # Pull tendon
    motor.set_velocity(velocity_erpm=-8000)   # Release tendon
    motor.stop()                              # Stop

    # Position control
    motor.set_position(position_degrees=90.0)
    current_pos = motor.get_position()

    # Tendon control helper
    motor.control_exosuit_tendon(action="pull", velocity_erpm=10000)
    motor.control_exosuit_tendon(action="stop")

# Clean shutdown
motor.close()
```

### Using Context Manager

```python
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

with CubeMarsAK606v3CAN(motor_can_id=0x03) as motor:
    motor.set_velocity(velocity_erpm=10000)
    time.sleep(2)
    motor.stop()
# Motor automatically closed when exiting context
```

### Multiple Motors

```python
# Each motor has a unique CAN ID configured in CubeMars software
motor_left = CubeMarsAK606v3CAN(motor_can_id=0x03, interface="can0")
motor_right = CubeMarsAK606v3CAN(motor_can_id=0x04, interface="can0")

# Control independently
motor_left.set_velocity(velocity_erpm=10000)
motor_right.set_velocity(velocity_erpm=10000)
```

## Troubleshooting

### CAN Interface Not Found

**Error:** `Failed to connect to CAN interface can0`

**Solutions:**
1. Verify CAN is enabled in device tree
2. Check interface status: `ip link show can0`
3. Bring up interface manually (see Software Setup above)

### No Response from Motor

**Error:** `Motor not responding after X attempts`

**Check:**
1. **Power:** Motor has 24-48V power supply connected
2. **Wiring:** Verify CANH/CANL connections (not swapped)
3. **Termination:** 120Ω resistor is installed across CANH/CANL
4. **CAN ID:** Motor CAN ID matches code (default: 0x03)
5. **Bitrate:** CAN bitrate matches motor config (1 Mbps)
6. **Motor Config:** Motor is configured for CAN mode (not UART)

### Bus-Off State

**Error:** CAN interface goes into bus-off state

**Causes:**
- Missing or incorrect termination resistor
- CANH and CANL swapped
- Bitrate mismatch
- Electrical noise or bad connections

**Solution:**
```bash
# Reset CAN interface
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
```

### Permission Denied

**Error:** `Operation not permitted`

**Solution:**
```bash
# Add user to dialout group for CAN access
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

Or run with sudo (not recommended for production):
```bash
sudo python your_script.py
```

## Hardware Validation Checklist

- [ ] SN65HVD230 powered with 3.3V
- [ ] CAN_TX connected to D (pin 1)
- [ ] CAN_RX connected to R (pin 4)
- [ ] Rs (pin 8) connected to GND for high-speed mode
- [ ] CANH connects transceiver pin 7 to motor CANH
- [ ] CANL connects transceiver pin 6 to motor CANL
- [ ] 120Ω termination resistor across CANH/CANL
- [ ] Common GND between Jetson, transceiver, and motor
- [ ] Motor powered on (24-48V)
- [ ] can0 interface configured and up at 1 Mbps
- [ ] Motor configured for CAN mode with matching bitrate

## Performance Notes

- **CAN Bitrate:** 1 Mbps provides sufficient bandwidth for motor control
- **Periodic Feedback:** Motor sends status at 50 Hz automatically
- **Latency:** Typical CAN frame transmission is <1ms
- **Multi-motor:** CAN bus supports up to ~30 motors on single bus (limited by IDs and bandwidth)

## References

- [SN65HVD230 Datasheet](https://www.ti.com/lit/ds/symlink/sn65hvd230.pdf)
- [Jetson Orin Nano Developer Kit User Guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit)
- [SocketCAN Documentation](https://www.kernel.org/doc/html/latest/networking/can.html)
- [python-can Documentation](https://python-can.readthedocs.io/)
