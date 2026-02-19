#!/usr/bin/env python3
"""Ultra simple test to see CAN frames."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN
import time

motor = CubeMarsAK606v3CAN(motor_can_id=0x03)
print("Sending enable command...")
motor.enable_motor()
time.sleep(1)
print("Sending velocity command...")
motor.set_velocity(5000, allow_low_speed=False)
time.sleep(1)
print("Sending disable command...")
motor.disable_motor()
print("Done. Check candump output.")
motor.bus.shutdown()
