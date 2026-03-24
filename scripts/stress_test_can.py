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

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

MOTOR_ID = 0x03
INTERFACE = "can0"
ROUNDS = 30  # Total test iterations


def test_basic_cycle(round_num: int) -> bool:
    """Connect → enable → get_status → disable → close."""
    try:
        motor = CubeMarsAK606v3CAN(motor_can_id=MOTOR_ID, interface=INTERFACE)
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
        motor = CubeMarsAK606v3CAN(motor_can_id=MOTOR_ID, interface=INTERFACE)
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
        motor = CubeMarsAK606v3CAN(motor_can_id=MOTOR_ID, interface=INTERFACE)
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
        motor1 = CubeMarsAK606v3CAN(motor_can_id=MOTOR_ID, interface=INTERFACE)
        motor1.enable_motor()
        # "crash" — lose reference without close()

        # Second connection — should recover
        motor2 = CubeMarsAK606v3CAN(motor_can_id=MOTOR_ID, interface=INTERFACE)
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
    print("=" * 60)
    print("  CAN Reliability Stress Test")
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
