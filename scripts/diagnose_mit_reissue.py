"""Check whether re-issuing set_mit_mode every tick stops the motor.

exosuit-python calls set_mit_mode about ninety times a second, once per
control tick, and the motor produces no torque. verify_set_velocity.py calls
it once and lets the keep-alive thread re-send the payload, and the motor
turns. Every set_mit_mode call runs _start_refresh(), which resets
_refresh_payload, _pending_feedback and _refresh_feedback.

Three cases, identical but for how often the command is issued:

    C  issued once, keep-alive holds it   -- the control
    A  re-issued every tick, constant     -- isolates the re-issue
    B  re-issued every tick, changing     -- re-issue plus a moving target

Watch the distinct-torque count. A failing exosuit run showed one value across
fifteen seconds where pre-tensioning showed 77 in six.

Gentle: 0.83 rad/s for 2 s is about a quarter turn of the spool, three times.
"""
# ruff: noqa: T201

import time

from motor_python import create_can_motor

HZ = 90
SECONDS = 2.0
VEL = 0.83  # rad/s, the same as 1000 ERPM


def read(motor):
    """Return (position_deg, torque) from the transport's cached feedback."""
    feedback = motor._last_feedback
    if feedback is None:
        return None, None
    return feedback.position_degrees, feedback.current_amps


def report(label, calls, start, end, torques):
    """Print what the shaft did during one case."""
    position_start, torque_start = start
    position_end, torque_end = end
    print()
    print(label)
    print(f"  calls sent       : {calls}")
    if position_start is None or position_end is None:
        print("  position moved   : unknown, no feedback")
    else:
        print(f"  position moved   : {position_end - position_start:+.1f} deg")
    print(f"  torque start/end : {torque_start} -> {torque_end}")
    print(f"  distinct torques : {len(torques)}  {sorted(torques)[:6]}")


def watch(motor, seconds):
    """Collect the distinct torques reported over a window."""
    torques = set()
    started = time.monotonic()
    while time.monotonic() - started < seconds:
        _, torque = read(motor)
        if torque is not None:
            torques.add(round(torque, 4))
        time.sleep(1.0 / HZ)
    return torques


def issued_once(motor, label):
    """Send the command once and let the keep-alive thread hold it.

    The control case. Everything else matches the bursts, so if this turns the
    shaft and they do not, the re-issue is the whole difference rather than
    something about how this script builds the motor or picks its gains.
    """
    start = read(motor)
    motor.set_mit_mode(pos_rad=0.0, vel_rad_s=VEL, kp=0.0, kd=1.0, torque_ff_nm=0.0)
    torques = watch(motor, SECONDS)
    report(label, 1, start, read(motor), torques)


def burst(motor, label, velocity_of):
    """Re-issue the command at HZ, the way a control loop does."""
    start = read(motor)
    began = time.monotonic()
    calls = 0
    torques = set()
    while True:
        elapsed = time.monotonic() - began
        if elapsed >= SECONDS:
            break
        motor.set_mit_mode(
            pos_rad=0.0,
            vel_rad_s=velocity_of(elapsed),
            kp=0.0,
            kd=1.0,
            torque_ff_nm=0.0,
        )
        calls += 1
        _, torque = read(motor)
        if torque is not None:
            torques.add(round(torque, 4))
        time.sleep(1.0 / HZ)
    report(label, calls, start, read(motor), torques)


def main() -> None:
    """Run the control case and both bursts against motor 0x04."""
    motor = create_can_motor("AK80-6", motor_can_id=4)
    try:
        motor.enable_mit_mode()
        issued_once(motor, "C: issued ONCE, keep-alive holds it   <-- control")
        motor.stop()
        time.sleep(1.0)

        motor.enable_mit_mode()
        burst(motor, "A: re-issued every tick, CONSTANT value", lambda _: VEL)
        motor.stop()
        time.sleep(1.0)

        motor.enable_mit_mode()
        # A triangle wave, so the value changes every tick as the assist does.
        burst(
            motor,
            "B: re-issued every tick, CHANGING value",
            lambda elapsed: VEL * abs(((elapsed % 1.0) - 0.5) * 2.0),
        )
    finally:
        motor.stop()
        motor.close()


if __name__ == "__main__":
    main()
