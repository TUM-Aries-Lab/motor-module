#!/usr/bin/env python3
"""Manual bench script for third-party TMotorCANControl servo mode."""

import argparse
import time
import traceback


def parse_int(value: str) -> int:
    """Parse decimal or hex motor ID values."""
    return int(value, 0)


def parse_args() -> argparse.Namespace:
    """Parse command line arguments for this manual servo script."""
    parser = argparse.ArgumentParser(
        description="Manual TMotorCANControl servo bench test"
    )
    parser.add_argument(
        "--motor-id",
        type=parse_int,
        default=0x03,
        help="Motor CAN ID (decimal or hex, default: 0x03).",
    )
    parser.add_argument(
        "--motor-type",
        default="AK80-9",
        help="TMotorCANControl motor type string (default: AK80-9).",
    )
    parser.add_argument(
        "--velocity-rad-s",
        type=float,
        default=0.5,
        help="Velocity command in rad/s used in test loop (default: 0.5).",
    )
    return parser.parse_args()


def main() -> None:
    """Run manual servo-mode checks with TMotorCANControl."""
    from TMotorCANControl.servo_can import TMotorManager_servo_can

    args = parse_args()

    print("=== TMotorCANControl Servo Mode Test ===")
    print(f"Motor ID: {args.motor_id} (0x{args.motor_id:02X})")
    print(f"Motor type: {args.motor_type}")
    print("Make sure UART cable is disconnected!")
    print()

    try:
        with TMotorManager_servo_can(
            motor_type=args.motor_type,
            motor_ID=args.motor_id,
            CSV_file=None,
        ) as motor:
            print("Motor manager created")

            motor.update()
            print("Motor responding")
            print(
                f"  Position: {motor.position:.3f} rad ({motor.position * 57.3:.1f} deg)"
            )
            print(f"  Velocity: {motor.velocity:.3f} rad/s")
            print(f"  Current: {motor.current_qaxis:.3f} A")
            print()

            print(f"Testing velocity control ({args.velocity_rad_s} rad/s)...")
            for i in range(30):
                motor.set_output_velocity_radians_per_second(args.velocity_rad_s)
                motor.update()
                time.sleep(0.05)

                if i % 10 == 0:
                    print(
                        f"  pos={motor.position:.3f} rad, vel={motor.velocity:.3f} rad/s"
                    )

            print("\nStopping motor...")
            motor.set_output_velocity_radians_per_second(0.0)
            motor.update()
            time.sleep(0.5)

            print("Test complete")

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as exc:
        print(f"\nError: {exc}")
        traceback.print_exc()

    print("Done")


if __name__ == "__main__":
    main()
