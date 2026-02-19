#!/usr/bin/env python3
"""Test our CAN implementation with enable/disable commands."""

import sys
import time
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

print("=== Testing Our CAN Implementation ===")
print("Motor ID: 3")
print("Make sure UART cable is disconnected!")
print()

try:
    motor = CubeMarsAK606v3CAN(
        motor_can_id=0x03,
        interface="can0",
        bitrate=1000000,
    )

    print("✓ Motor object created")

    # Enable the motor
    print("\nEnabling motor...")
    motor.enable_motor()
    time.sleep(0.5)

    # Get feedback
    feedback = motor._receive_feedback(timeout=1.0)
    if feedback:
        print(f"✓ Motor responding!")
        print(f"  Position: {feedback.position_degrees:.1f}°")
        print(f"  Velocity: {feedback.speed_erpm} ERPM")
        print(f"  Current: {feedback.current_amps:.2f}A")
    else:
        print("✗ No feedback received")

    # Try velocity control
    print("\nTesting velocity control (5000 ERPM = ~50 RPM)...")
    for i in range(30):
        motor.set_velocity(5000, allow_low_speed=False)
        feedback = motor._receive_feedback(timeout=0.1)
        time.sleep(0.05)

        if feedback and i % 10 == 0:
            print(f"  pos={feedback.position_degrees:.1f}°, vel={feedback.speed_erpm} ERPM")

    # Stop motor
    print("\nStopping motor...")
    motor.set_velocity(0, allow_low_speed=True)
    time.sleep(0.5)

    # Disable motor
    print("Disabling motor...")
    motor.disable_motor()

    print("✓ Test complete!")

except KeyboardInterrupt:
    print("\n\nTest interrupted by user")
    if 'motor' in locals() and motor.bus is not None:
        motor.disable_motor()
except Exception as e:
    print(f"\n✗ Error: {e}")
    import traceback
    traceback.print_exc()
    if 'motor' in locals() and motor.bus is not None:
        motor.disable_motor()
finally:
    if 'motor' in locals() and motor.bus is not None:
        motor.bus.shutdown()
    print("✓ Done")
