#!/usr/bin/env python3
"""CAN reliability stress test — verifies the driver works under harsh conditions.

Runs multiple rounds of connect → enable → get_status → disable → close,
simulating real usage patterns including:
  - Rapid reconnection (script crash → restart)
  - check_communication() (the historically unreliable path)
  - Back-to-back commands at full speed

Run:
    sudo ./setup_can.sh
    .venv/bin/python scripts/stress_test_can.py
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))
from motor_python import create_can_motor

MOTOR_ID = 0x03
MOTOR_MODEL = "AK60-6"
INTERFACE = "can0"
BITRATE = 1_000_000
ROUNDS = 30  # Total test iterations


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="CAN reliability stress test")
    parser.add_argument("--interface", default="can0", help="SocketCAN interface")
    parser.add_argument(
        "--motor-id",
        type=lambda value: int(value, 0),
        default=0x03,
        help="Motor CAN ID in decimal or hex (default: 0x03)",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=1_000_000,
        help="CAN bitrate (default: 1000000)",
    )
    parser.add_argument(
        "--motor-model",
        choices=("AK60-6", "AK80-6"),
        default="AK60-6",
        help="Motor model to instantiate (default: AK60-6)",
    )
    return parser.parse_args()


def test_basic_cycle(round_num: int) -> bool:
    """Connect → enable → get_status → disable → close."""
    try:
        motor = create_can_motor(
            MOTOR_MODEL,
            motor_can_id=MOTOR_ID,
            interface=INTERFACE,
            bitrate=BITRATE,
        )
        motor.enable_motor()
        fb = motor.get_status()
        motor.disable_motor()
        motor.close()
        if fb:
            print(f"  [{round_num:2d}] ✓  basic_cycle        pos={fb.position_degrees:.1f}°  temp={fb.temperature_celsius}°C")
            return True
        else:
            print(f"  [{round_num:2d}] ✗  basic_cycle        get_status returned None")
            return False
    except Exception as e:
        print(f"  [{round_num:2d}] ✗  basic_cycle        Exception: {e}")
        return False


def test_check_communication(round_num: int) -> bool:
    """Use the historically unreliable check_communication() path."""
    try:
        motor = create_can_motor(
            MOTOR_MODEL,
            motor_can_id=MOTOR_ID,
            interface=INTERFACE,
            bitrate=BITRATE,
        )
        ok = motor.check_communication()
        motor.disable_motor()
        motor.close()
        if ok:
            print(f"  [{round_num:2d}] ✓  check_communication")
            return True
        else:
            print(f"  [{round_num:2d}] ✗  check_communication returned False")
            return False
    except Exception as e:
        print(f"  [{round_num:2d}] ✗  check_communication Exception: {e}")
        return False


def test_rapid_commands(round_num: int) -> bool:
    """Send 20 rapid get_status calls to stress the feedback path."""
    try:
        motor = create_can_motor(
            MOTOR_MODEL,
            motor_can_id=MOTOR_ID,
            interface=INTERFACE,
            bitrate=BITRATE,
        )
        motor.enable_motor()
        time.sleep(0.1)

        successes = 0
        for _ in range(20):
            fb = motor.get_status()
            if fb:
                successes += 1

        motor.disable_motor()
        motor.close()
        if successes == 20:
            print(f"  [{round_num:2d}] ✓  rapid_commands     {successes}/20 get_status OK")
            return True
        else:
            print(f"  [{round_num:2d}] ✗  rapid_commands     {successes}/20 get_status OK")
            return False
    except Exception as e:
        print(f"  [{round_num:2d}] ✗  rapid_commands     Exception: {e}")
        return False


def test_no_close_reconnect(round_num: int) -> bool:
    """Simulate crash: connect without closing previous connection."""
    try:
        # First connection — deliberately NOT closed
        motor1 = create_can_motor(
            MOTOR_MODEL,
            motor_can_id=MOTOR_ID,
            interface=INTERFACE,
            bitrate=BITRATE,
        )
        motor1.enable_motor()
        # "crash" — lose reference without close()

        # Second connection — should recover
        motor2 = create_can_motor(
            MOTOR_MODEL,
            motor_can_id=MOTOR_ID,
            interface=INTERFACE,
            bitrate=BITRATE,
        )
        motor2.enable_motor()
        fb = motor2.get_status()
        motor2.disable_motor()
        motor2.close()

        # Clean up motor1 (GC might not have run yet)
        try:
            motor1.close()
        except Exception:
            pass

        if fb:
            print(f"  [{round_num:2d}] ✓  crash_reconnect    pos={fb.position_degrees:.1f}°")
            return True
        else:
            print(f"  [{round_num:2d}] ✗  crash_reconnect    get_status returned None")
            return False
    except Exception as e:
        print(f"  [{round_num:2d}] ✗  crash_reconnect    Exception: {e}")
        return False


def main():
    args = parse_args()
    global MOTOR_ID, INTERFACE, BITRATE, MOTOR_MODEL
    MOTOR_ID = args.motor_id
    INTERFACE = args.interface
    BITRATE = args.bitrate
    MOTOR_MODEL = args.motor_model

    print("=" * 60)
    print("  CAN Reliability Stress Test")
    print(f"  Motor model: {MOTOR_MODEL}")
    print(f"  Motor ID: 0x{MOTOR_ID:02X}  |  Interface: {INTERFACE}  |  Bitrate: {BITRATE}")
    print(f"  {ROUNDS} rounds × 4 test types = {ROUNDS * 4} operations")
    print("=" * 60)

    results = {"basic_cycle": 0, "check_comm": 0, "rapid_cmd": 0, "crash_reconnect": 0}
    total_tests = 0
    total_pass = 0

    for i in range(1, ROUNDS + 1):
        print(f"\n── Round {i}/{ROUNDS} ──")

        ok = test_basic_cycle(i)
        results["basic_cycle"] += ok
        total_tests += 1
        total_pass += ok

        ok = test_check_communication(i)
        results["check_comm"] += ok
        total_tests += 1
        total_pass += ok

        ok = test_rapid_commands(i)
        results["rapid_cmd"] += ok
        total_tests += 1
        total_pass += ok

        if i % 5 == 0:  # Only run crash reconnect every 5 rounds
            ok = test_no_close_reconnect(i)
            results["crash_reconnect"] += ok
            total_tests += 1
            total_pass += ok

        # Small delay between rounds
        time.sleep(0.2)

    print(f"\n{'=' * 60}")
    print("  RESULTS")
    print(f"{'=' * 60}")
    print(f"  basic_cycle       : {results['basic_cycle']}/{ROUNDS}")
    print(f"  check_communication: {results['check_comm']}/{ROUNDS}")
    print(f"  rapid_commands    : {results['rapid_cmd']}/{ROUNDS}")
    crash_rounds = ROUNDS // 5
    print(f"  crash_reconnect   : {results['crash_reconnect']}/{crash_rounds}")
    print(f"  {'─' * 40}")
    print(f"  TOTAL             : {total_pass}/{total_tests}  "
          f"({total_pass/total_tests*100:.1f}%)")

    if total_pass == total_tests:
        print("\n  ✓  ALL TESTS PASSED — CAN is rock-solid!")
    else:
        failures = total_tests - total_pass
        print(f"\n  ✗  {failures} FAILURES detected — investigate above logs")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
