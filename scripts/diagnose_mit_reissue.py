"""Find which startup step makes the motor actually produce torque.

verify_set_velocity.py turns the shaft. A minimal script that connects, calls
enable_mit_mode() and then set_mit_mode() does not -- the motor reports live
position and a single unchanging torque, which is also what exosuit-python's
assist shows. Re-issuing the command at control rate was ruled out: issuing it
once behaves identically.

Comparing the two logs, verify_set_velocity.py does more before commanding:

    send_neutral_command()      -- a neutral MIT frame
    check_communication()       -- neutral frame plus an enable handshake
    set_velocity(erpm)          -- rather than set_mit_mode() directly

Each stage below adds one of those, so a single run says which one matters.
The first stage to move the shaft is the answer.

Gentle: 0.83 rad/s (1000 ERPM) for 2 s is about a quarter turn, four times.
"""
# ruff: noqa: T201

import time

from motor_python import create_can_motor

HZ = 90
SECONDS = 2.0
ERPM = 1000
VEL = 0.83  # rad/s, the same command expressed for set_mit_mode


def read(motor):
    """Return (position_deg, torque) from the transport's cached feedback."""
    feedback = motor._last_feedback
    if feedback is None:
        return None, None
    return feedback.position_degrees, feedback.current_amps


def hold(motor, seconds):
    """Watch the reported torque for a window and return the distinct values."""
    torques = set()
    started = time.monotonic()
    while time.monotonic() - started < seconds:
        _, torque = read(motor)
        if torque is not None:
            torques.add(round(torque, 4))
        time.sleep(1.0 / HZ)
    return torques


def stage(motor, label, prepare, command):
    """Run one startup variant and report whether the shaft moved."""
    motor.enable_mit_mode()
    prepare(motor)
    position_start, torque_start = read(motor)
    command(motor)
    torques = hold(motor, SECONDS)
    position_end, torque_end = read(motor)

    print()
    print(label)
    if position_start is None or position_end is None:
        print("  position moved   : unknown, no feedback")
    else:
        moved = position_end - position_start
        verdict = "MOVED" if abs(moved) > 1.0 else "stalled"
        print(f"  position moved   : {moved:+.1f} deg   <-- {verdict}")
    print(f"  torque start/end : {torque_start} -> {torque_end}")
    print(f"  distinct torques : {len(torques)}  {sorted(torques)[:6]}")

    motor.stop()
    time.sleep(1.0)


def nothing(motor):
    """Add no preparation."""


def neutral_then_check(motor):
    """Do what verify_set_velocity.py does before it commands."""
    motor.send_neutral_command()
    motor.check_communication()


def main() -> None:
    """Walk the startup variants against motor 0x04."""
    motor = create_can_motor("AK80-6", motor_can_id=4)
    try:
        stage(
            motor,
            "1: set_mit_mode direct, no preparation",
            nothing,
            lambda m: m.set_mit_mode(
                pos_rad=0.0, vel_rad_s=VEL, kp=0.0, kd=1.0, torque_ff_nm=0.0
            ),
        )
        stage(
            motor,
            "2: set_velocity, no preparation",
            nothing,
            lambda m: m.set_velocity(velocity_erpm=ERPM),
        )
        stage(
            motor,
            "3: set_velocity, after check_communication",
            lambda m: m.check_communication(),
            lambda m: m.set_velocity(velocity_erpm=ERPM),
        )
        stage(
            motor,
            "4: set_velocity, after neutral + check  <-- verify_set_velocity",
            neutral_then_check,
            lambda m: m.set_velocity(velocity_erpm=ERPM),
        )
    finally:
        motor.stop()
        motor.close()


if __name__ == "__main__":
    main()
