"""Quick test of TMotorCANControl Servo mode with AK60-6."""

import time
from TMotorCANControl.servo_can import TMotorManager_servo_can

print("=== TMotorCANControl Servo Mode Test ===")
print("Motor ID: 3 (AK60-6)")
print("Make sure UART cable is disconnected!")
print()

try:
    # Using AK80-9 parameters (closest available to AK60-6)
    with TMotorManager_servo_can(motor_type='AK80-9', motor_ID=3, CSV_file=None) as motor:
        print("✓ Motor manager created")

        # Get initial state
        motor.update()
        print(f"✓ Motor responding!")
        print(f"  Position: {motor.position:.3f} rad ({motor.position * 57.3:.1f}°)")
        print(f"  Velocity: {motor.velocity:.3f} rad/s")
        print(f"  Current: {motor.current_qaxis:.3f} A")
        print()

        # Try velocity control
        print("Testing velocity control (0.5 rad/s)...")
        for i in range(30):
            motor.set_output_velocity_radians_per_second(0.5)
            motor.update()
            time.sleep(0.05)

            if i % 10 == 0:
                print(f"  pos={motor.position:.3f} rad, vel={motor.velocity:.3f} rad/s")

        # Stop motor
        print("\nStopping motor...")
        motor.set_output_velocity_radians_per_second(0)
        motor.update()
        time.sleep(0.5)

        print("✓ Test complete!")

except KeyboardInterrupt:
    print("\nTest interrupted by user")
except Exception as e:
    print(f"\n✗ Error: {e}")
    import traceback
    traceback.print_exc()

print("✓ Done")
