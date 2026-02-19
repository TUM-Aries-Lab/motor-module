# CAN Communication Testing Guide

## Prerequisites

### 1. Hardware Setup

Ensure all hardware is connected:
- [ ] SN65HVD230 transceiver wired to Jetson Orin Nano
- [ ] CANH/CANL connected from transceiver to motor
- [ ] 120Ω termination resistor across CANH/CANL
- [ ] Motor powered on (24-48V)
- [ ] Common ground between all devices

### 2. Motor Configuration

Configure motor using CubeMars software (Windows):
- [ ] **CAN Mode:** Periodic Feedback
- [ ] **CAN Bitrate:** 1 Mbps
- [ ] **CAN ID:** 3 (0x03)
- [ ] **CAN Feedback Rate:** 50 Hz
- [ ] Configuration saved to motor

### 3. Jetson CAN Interface

Enable the CAN interface on the Jetson:

```bash
# Configure CAN interface
sudo ip link set can0 type can bitrate 1000000

# Bring up the interface
sudo ip link set can0 up

# Verify it's running
ip -details link show can0
```

Expected output:
```
3: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can
    can state ERROR-ACTIVE restart-ms 0
          bitrate 1000000 sample-point 0.750
```

## Testing Methods

### Method 1: Using can-utils (Low-Level Test)

Install can-utils if not already installed:
```bash
sudo apt-get update
sudo apt-get install can-utils
```

#### Listen for Motor Feedback

In one terminal, monitor CAN traffic:
```bash
candump can0
```

If the motor is powered and configured correctly, you should see periodic messages from CAN ID 0x03:
```
can0  003   [8]  00 00 00 00 00 00 1D 00
can0  003   [8]  00 01 00 05 00 02 1D 00
can0  003   [8]  00 02 00 0A 00 04 1D 00
```

#### Send Test Commands

Send a velocity command (5000 ERPM) to motor ID 0x03:
```bash
# Velocity command: CAN ID 0x00000368, data = 5000 ERPM = 0x00001388
cansend can0 00000368#0000138800000000
```

Send stop command (0 current):
```bash
# Current command: CAN ID 0x00000168, data = 0 mA
cansend can0 00000168#0000000000000000
```

### Method 2: Python Test Script

Run the automated test suite:

```bash
# Activate virtual environment
source .venv/bin/activate

# Run test script
python src/motor_python/test_can.py
```

The test script will:
1. ✓ Test CAN interface connection
2. ✓ Listen for motor feedback (passive test)
3. ✓ Verify bidirectional communication
4. ⚠ Send velocity command (requires confirmation)
5. ⚠ Send position command (requires confirmation)

### Method 3: Interactive Python Testing

```bash
# Activate virtual environment
source .venv/bin/activate

# Start Python REPL
python
```

```python
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN
import time

# Initialize motor
motor = CubeMarsAK606v3CAN(motor_can_id=0x03)

# Test 1: Check if CAN interface is working
print(f"Connected: {motor.connected}")

# Test 2: Listen for feedback (motor must be powered on)
feedback = motor._receive_feedback(timeout=2.0)
if feedback:
    print(f"Position: {feedback.position_degrees}°")
    print(f"Speed: {feedback.speed_erpm} ERPM")
    print(f"Current: {feedback.current_amps} A")
    print(f"Temperature: {feedback.temperature_celsius}°C")
else:
    print("No feedback received - check motor power and wiring")

# Test 3: Communication check
if motor.check_communication():
    print("Motor responding!")
else:
    print("Motor not responding")

# Test 4: Send velocity command (MOTOR WILL ROTATE!)
# Uncomment only when ready:
# motor.set_velocity(velocity_erpm=5000)
# time.sleep(2)
# motor.stop()

# Test 5: Get status
status = motor.get_status()

# Close connection
motor.close()
```

### Method 4: Example Scripts

Run the pre-built examples:

```bash
# Basic CAN example
python src/motor_python/examples_can.py
```

## Troubleshooting

### No CAN Interface Found

**Error:** `Failed to connect to CAN interface can0`

**Solutions:**
```bash
# Check if interface exists
ip link show can0

# If not found, check kernel modules
lsmod | grep can

# Load SocketCAN modules if needed
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

# Configure and bring up
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### No Feedback Messages

**Problem:** candump shows no messages from motor

**Checklist:**
1. Motor powered on? Check 24-48V supply
2. CANH/CANL wired correctly? (not swapped)
3. 120Ω termination installed?
4. Motor configured for CAN mode?
5. Motor CAN ID matches (0x03)?
6. Bitrate matches (1 Mbps)?

**Test transceiver:**
```bash
# Send loopback test (requires RS pin modification)
# Or send a message and verify with oscilloscope on CANH/CANL
cansend can0 123#DEADBEEF
```

### Bus-Off State

**Error:** CAN interface goes into ERROR-PASSIVE or BUS-OFF

**Causes:**
- Missing termination resistor
- CANH/CANL swapped
- Bitrate mismatch
- Electrical noise

**Solution:**
```bash
# Reset interface
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up

# Check bus state
ip -details -statistics link show can0
```

### Motor Not Responding to Commands

**Problem:** Feedback received but commands don't work

**Debug:**
```bash
# Monitor with candump while sending command
candump can0 &
cansend can0 00000368#0000138800000000

# Check if message appears on bus
# Verify extended ID format is correct
```

**Verify command format:**
- Extended ID should be 29-bit: `00 00 0M XX`
- Data should be 8 bytes (padded with zeros)
- Values should be big-endian (most significant byte first)

### Permission Issues

**Error:** `Operation not permitted`

**Solution:**
```bash
# Add user to netdev group
sudo usermod -a -G netdev $USER

# Or run with sudo (not recommended for production)
sudo python src/motor_python/test_can.py
```

## Expected Test Results

### Successful Test Output

```
TEST 1: CAN Interface Connection
✓ CAN interface 'can0' initialized successfully

TEST 2: Motor Periodic Feedback
✓ Received motor feedback!
  Position: 12.5°
  Speed: 0 ERPM
  Current: 0.02 A
  Temperature: 29°C
  Error Code: 0

TEST 3: Communication Check
✓ Motor communication verified

TEST 4: Velocity Command
Motor speed: 5123 ERPM
✓ Motor is rotating
✓ Velocity command test completed

TEST 5: Position Command
Current position: 12.5°
Moving to position: 102.5°...
New position: 101.8°
✓ Position reached (error: 0.70°)

TEST SUMMARY
INTERFACE          : ✓ PASS
FEEDBACK           : ✓ PASS
COMMUNICATION      : ✓ PASS
VELOCITY           : ✓ PASS
POSITION           : ✓ PASS
```

## Safety Notes

⚠️ **Before running motor tests:**
- Disconnect tendon/load if possible
- Ensure motor can rotate freely
- Keep clear of moving parts
- Have emergency stop ready (power switch)
- Start with low velocities (5000 ERPM)

## Next Steps

Once basic communication is verified:
1. Test with your exosuit control application
2. Tune feedback rate if needed (1-500 Hz)
3. Test with multiple motors (different CAN IDs)
4. Implement error handling for motor fault codes
5. Add CAN bus monitoring to your logging system
