#!/usr/bin/env bash
# Hard reset the CAN interface by unloading and reloading the mttcan driver, then bringing the interface back up.
# Run this on one terminal: ./scripts/can_hard_reset.sh

set -euo pipefail

INTERFACE="can0"
BITRATE=1000000

echo "Performing hard CAN reset for $INTERFACE"

sudo ip link set "$INTERFACE" down
sudo modprobe -r mttcan || true
sleep 0.5
sudo modprobe mttcan
sudo ip link set "$INTERFACE" up type can bitrate "$BITRATE" berr-reporting on restart-ms 100
sudo ip link set "$INTERFACE" txqueuelen 1000
sudo ip link set "$INTERFACE" up
sleep 1

echo "CAN hard reset complete"
ip -details -statistics link show "$INTERFACE"
