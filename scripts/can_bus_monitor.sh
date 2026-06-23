#!/usr/bin/env bash
# This is a can bus monitoring script that logs the state of the CAN interface and dumps CAN messages to a log file.
# Run this on one terminal: ./scripts/can_bus_monitor.sh

#Run the motor script on second terminal: uv run python -m motor_python --log-level DEBUG

set -euo pipefail

LOGDIR=~/can_debug_$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOGDIR"
STATEFILE="$LOGDIR/state.txt"

echo "=== BEFORE ===" > "$STATEFILE"
ip -details -statistics link show can0 >> "$STATEFILE" 2>&1

echo "Logging CAN bus state to: $LOGDIR"

echo "Starting CAN state logger..."
(
    while true; do
        echo "=== $(date +%T.%N) ==="
        ip -details -statistics link show can0 2>&1 | grep -E "state|berr-counter|can|tx|rx|error"
        sleep 1
    done
) >> "$STATEFILE" &
STATE_PID=$!

echo "Started CAN state logger: PID=$STATE_PID"

echo "Starting candump to $LOGDIR/candump.log..."
candump can0,0:0,#FFFFFFFF -te -ta -L > "$LOGDIR/candump.log" &
DUMP_PID=$!

echo "Started candump: PID=$DUMP_PID"

echo ""
echo "To stop logging, press Ctrl-C or run: kill $STATE_PID $DUMP_PID"
echo "State log file : $STATEFILE"
echo "CAN dump file  : $LOGDIR/candump.log"

cleanup() {
    echo ""
    echo "Stopping CAN logging..."
    echo "=== AFTER FAILURE ===" >> "$STATEFILE"   # If it fails, immediately snapshot the bus state in Terminal 1
    ip -details -statistics link show can0 >> "$STATEFILE" 2>&1
    kill "$STATE_PID" "$DUMP_PID" 2>/dev/null || true
    wait "$STATE_PID" 2>/dev/null || true
    wait "$DUMP_PID" 2>/dev/null || true
    echo "=== FINAL ===" >> "$STATEFILE"
    ip -details -statistics link show can0 >> "$STATEFILE" 2>&1
    echo "Logging stopped. Review $LOGDIR"
}

trap cleanup INT TERM EXIT

# Keep the script running until interrupted.
while true; do
    sleep 1
done
