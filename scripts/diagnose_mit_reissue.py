"""Check whether re-issuing set_mit_mode every tick stops the motor.

exosuit-python calls set_mit_mode ~90 times a second, once per control tick.
verify_set_velocity.py calls it once and lets the keep-alive thread re-send.
The first does not move the motor; the second does. Every set_mit_mode call
runs _start_refresh(), which resets the refresh state, so this separates two
candidate causes:

    A  repeated calls, constant value   -> isolates the re-issue itself
    B  repeated calls, changing value   -> re-issue plus a moving target

Gentle: 0.83 rad/s for 2 s is about a quarter turn of the spool.
"""
# ruff: noqa: T201

import time

from motor_python import create_can_motor

HZ = 90
SECONDS = 2.0
VEL = 0.83  # rad/s, same as 1000 ERPM


def read(motor):
    """Return (position_deg, torque) from the transport's cached feedback."""
    fb = motor._last_feedback
    if fb is None:
        return None, None
    return fb.position_degrees, fb.current_amps


def burst(motor, label, velocity_of):
    """Re-issue a command at HZ and report what the shaft did."""
    p0, t0_tq = read(motor)
    started = time.monotonic()
    calls = 0
    torques = set()
    while True:
        elapsed = time.monotonic() - started
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
        _, tq = read(motor)
        if tq is not None:
            torques.add(round(tq, 4))
        time.sleep(1.0 / HZ)

    p1, t1_tq = read(motor)
    moved = None if (p0 is None or p1 is None) else p1 - p0
    print(f"\n{label}")
    print(f"  calls sent        : {calls} in {SECONDS}s")
    print(f"  position moved    : {moved:+.1f} deg" if moved is not None else "  position: unknown")
    print(f"  torque start/end  : {t0_tq} -> {t1_tq}")
    print(f"  distinct torques  : {len(torques)}  {sorted(torques)[:6]}")


def main() -> None:
    """Run both bursts against motor 0x04."""
    motor = create_can_motor("AK80-6", motor_can_id=4)
    try:
        motor.enable_mit_mode()
        burst(motor, "A: re-issued every tick, CONSTANT value", lambda _: VEL)
        motor.stop()
        time.sleep(1.0)

        motor.enable_mit_mode()
        # A triangle wave, so the value changes every tick as the assist does.
        burst(
            motor,
            "B: re-issued every tick, CHANGING value",
            lambda e: VEL * abs(((e % 1.0) - 0.5) * 2.0),
        )
    finally:
        motor.stop()
        motor.close()


if __name__ == "__main__":
    main()
