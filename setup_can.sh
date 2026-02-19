#!/bin/bash
# CAN interface setup script for Jetson Orin Nano
# Usage: sudo ./setup_can.sh

set -e  # Exit on error

echo "================================================="
echo "CAN Interface Setup for Motor Communication"
echo "================================================="

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: Please run with sudo"
    echo "Usage: sudo ./setup_can.sh"
    exit 1
fi

# Load CAN kernel modules
echo ""
echo "Step 1: Loading CAN kernel modules..."
modprobe can 2>/dev/null || echo "  - can module already loaded or built-in"
modprobe can_raw 2>/dev/null || echo "  - can_raw module already loaded or built-in"
modprobe mttcan 2>/dev/null || echo "  - mttcan module already loaded or built-in"
echo "  ✓ Kernel modules loaded"

# Bring down interface (if already up)
echo ""
echo "Step 2: Bringing down can0 interface..."
ip link set can0 down 2>/dev/null || echo "  - Interface already down"
echo "  ✓ Interface down"

# Configure bitrate (1 Mbps to match motor configuration)
echo ""
echo "Step 3: Configuring CAN bitrate to 1 Mbps..."
ip link set can0 type can bitrate 1000000
echo "  ✓ Bitrate configured"

# Bring up interface
echo ""
echo "Step 4: Bringing up can0 interface..."
ip link set can0 up
echo "  ✓ Interface up"

# Verify configuration
echo ""
echo "Step 5: Verifying configuration..."
echo "------------------------------------------------"
ip -details link show can0
echo "------------------------------------------------"

# Check state
STATE=$(ip -details link show can0 | grep -o "state [A-Z-]*" | awk '{print $2}')
BITRATE=$(ip -details link show can0 | grep -o "bitrate [0-9]*" | awk '{print $2}')

echo ""
if [ "$STATE" = "UP" ] && [ "$BITRATE" = "1000000" ]; then
    echo "✓ SUCCESS: CAN interface configured correctly!"
    echo ""
    echo "Interface: can0"
    echo "State: $STATE"
    echo "Bitrate: $BITRATE bps (1 Mbps)"
    echo ""
    echo "Next steps:"
    echo "  1. Power on your motor (24-48V)"
    echo "  2. Run: candump can0"
    echo "     (You should see periodic messages from CAN ID 003)"
    echo "  3. Or run Python test: python src/motor_python/test_can.py"
else
    echo "⚠ WARNING: Configuration may not be correct"
    echo "State: $STATE (expected: UP)"
    echo "Bitrate: $BITRATE (expected: 1000000)"
fi

echo ""
echo "================================================="
