#!/usr/bin/env python3
"""Manual bench script to test our CAN implementation end-to-end."""

import argparse
import sys
import time
import traceback
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


def parse_int(value: str) -> int:
    """Parse decimal or hex motor ID values."""
    return int(value, 0)


def parse_args() -> argparse.Namespace:
    """Parse command line inputs for this manual bench script."""
    parser = argparse.ArgumentParser(description="Manual CAN implementation bench test")
    parser.add_argument(
        "--motor-id",
        type=parse_int,
        default=0x03,
        help="Motor CAN ID (decimal or hex, default: 0x03).",
    )
    parser.add_argument(
        "--interface",
        default="can0",
        help="SocketCAN interface (default: can0).",
    )
    parser.add_argument(
        "--velocity-erpm",
        type=int,
        default=5000,
        help="Velocity command used in the loop (default: 5000).",
    )
    return parser.parse_args()


def main() -> None:
    """Run the manual CAN bench sequence."""
    from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

    args = parse_args()
    motor = None

    print("=== Testing Our CAN Implementation ===")
    print(f"Motor ID: {args.motor_id} (0x{args.motor_id:02X})")
    print(f"Interface: {args.interface}")
    print("Make sure UART cable is disconnected!")
    print()

    try:
        motor = CubeMarsAK606v3CAN(
            motor_can_id=args.motor_id,
            interface=args.interface,
            bitrate=1000000,
        )
        print("Motor object created")

        print("\nEnabling motor...")
        motor.enable_motor()
        time.sleep(0.5)

        feedback = motor._receive_feedback(timeout=1.0)
        if feedback:
            print("Motor responding")
            print(f"  Position: {feedback.position_degrees:.1f} deg")
            print(f"  Velocity: {feedback.speed_erpm} ERPM")
            print(f"  Current: {feedback.current_amps:.2f} A")
        else:
            print("No feedback received")

        print(f"\nTesting velocity control ({args.velocity_erpm} ERPM)...")
        for i in range(30):
            motor.set_velocity(args.velocity_erpm, allow_low_speed=False)
            feedback = motor._receive_feedback(timeout=0.1)
            time.sleep(0.05)

            if feedback and i % 10 == 0:
                print(
                    "  pos="
                    f"{feedback.position_degrees:.1f} deg, "
                    f"vel={feedback.speed_erpm} ERPM"
                )

        print("\nStopping motor...")
        motor.set_velocity(0, allow_low_speed=True)
        time.sleep(0.5)

        print("Disabling motor...")
        motor.disable_motor()
        print("Test complete")

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        if motor is not None and motor.bus is not None:
            motor.disable_motor()
    except Exception as exc:
        print(f"\nError: {exc}")
        traceback.print_exc()
        if motor is not None and motor.bus is not None:
            motor.disable_motor()
    finally:
        if motor is not None and motor.bus is not None:
            motor.bus.shutdown()
        print("Done")


if __name__ == "__main__":
    main()
