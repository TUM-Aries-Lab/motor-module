"""Quick test to check motor feedback."""

import time
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

print("Connecting to motor...")
motor = CubeMarsAK606v3CAN(motor_can_id=0x03)

print("Sending stop command to activate motor...")
motor.set_current(0.0)
time.sleep(0.1)

print("Listening for feedback...")
for i in range(10):
    feedback = motor._receive_feedback(timeout=0.5)
    if feedback:
        print(f"✓ Feedback received!")
        print(f"  Position: {feedback.position_degrees:.2f}°")
        print(f"  Speed: {feedback.speed_erpm} ERPM")
        print(f"  Current: {feedback.current_amps:.2f} A")
        print(f"  Temperature: {feedback.temperature_celsius}°C")
        break
    else:
        print(f"  Attempt {i+1}/10: No feedback")
        time.sleep(0.2)

motor.close()
