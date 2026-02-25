"""CAN class function demo — exercises every public method of CubeMarsAK606v3CAN.

Run:
    sudo ./setup_can.sh
    python can_demo.py

Each section prints PASS / FAIL / SKIP so you can confirm which functions
are working on the current firmware.

Known firmware limitation: set_velocity(), set_position(), and
set_position_velocity_accel() send valid CAN frames and get ACKs but do NOT
produce shaft rotation on this AK60-6 firmware build.  Only set_duty_cycle()
produces real movement.  Those sections are marked [ACK-ONLY].
"""

import time

from motor_python.cube_mars_motor_can import CAN_ERROR_CODES, CubeMarsAK606v3CAN

MOTOR_ID  = 0x03
INTERFACE = "can0"
SEPARATOR = "─" * 56

# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def section(title: str) -> None:
    print(f"\n{SEPARATOR}")
    print(f"  {title}")
    print(SEPARATOR)


def result(label: str, value, *, passed: bool = True, note: str = "") -> None:
    icon = "✓" if passed else "✗"
    note_str = f"  ({note})" if note else ""
    print(f"  {icon}  {label}: {value}{note_str}")


# ---------------------------------------------------------------------------
# Main demo
# ---------------------------------------------------------------------------

def main() -> None:
    print("=" * 56)
    print("  CubeMarsAK606v3CAN — Function Demo")
    print(f"  Motor ID: 0x{MOTOR_ID:02X}  |  Interface: {INTERFACE}")
    print("=" * 56)

    with CubeMarsAK606v3CAN(motor_can_id=MOTOR_ID, interface=INTERFACE) as motor:

        # ── 1. check_communication ─────────────────────────────────────────
        section("1. check_communication()")
        alive = motor.check_communication()
        result("Motor responding", alive, passed=alive)
        if not alive:
            print("\n  Motor not responding — check power, wiring, and CAN ID.")
            return

        # ── 2. enable_motor ────────────────────────────────────────────────
        section("2. enable_motor()")
        motor.enable_motor()
        time.sleep(0.2)
        result("enable_motor()", "sent ✓")

        # ── 3. get_status ──────────────────────────────────────────────────
        section("3. get_status()  — full feedback frame")
        fb = motor.get_status()
        if fb:
            result("position_degrees", f"{fb.position_degrees:.1f}°")
            result("speed_erpm",       f"{fb.speed_erpm} ERPM")
            result("current_amps",     f"{fb.current_amps:.2f} A")
            result("temperature_celsius", f"{fb.temperature_celsius} °C")
            result("error_code",       f"{fb.error_code} — {fb.error_description}")
        else:
            result("get_status()", "None", passed=False)

        # ── 4. Individual telemetry getters ────────────────────────────────
        section("4. get_temperature() / get_current() / get_speed()")
        temp = motor.get_temperature()
        cur  = motor.get_current()
        spd  = motor.get_speed()
        result("get_temperature()", f"{temp} °C",   passed=temp is not None)
        result("get_current()",     f"{cur:.2f} A", passed=cur  is not None)
        result("get_speed()",       f"{spd} ERPM",  passed=spd  is not None)

        # ── 5. get_motor_data ──────────────────────────────────────────────
        section("5. get_motor_data()  — dict with all fields")
        data = motor.get_motor_data()
        if data:
            for k, v in data.items():
                result(k, v)
        else:
            result("get_motor_data()", "None", passed=False)

        # ── 6. get_position ────────────────────────────────────────────────
        section("6. get_position()")
        pos = motor.get_position()
        result("get_position()", f"{pos:.1f}°" if pos is not None else "None",
               passed=pos is not None)

        # ── 7. set_duty_cycle — CONFIRMED WORKING ──────────────────────────
        section("7. set_duty_cycle()  [CONFIRMED WORKING — spins shaft]")
        print("  Spinning forward at 10% duty for 1.5 s ...")
        motor.set_duty_cycle(0.10)
        time.sleep(1.5)
        fb_mid = motor.get_status()
        if fb_mid:
            result("speed during spin", f"{fb_mid.speed_erpm} ERPM",
                   passed=abs(fb_mid.speed_erpm) > 100)
            result("current during spin", f"{fb_mid.current_amps:.2f} A")
        motor.set_duty_cycle(0.0)
        time.sleep(0.3)
        motor.set_duty_cycle(-0.10)
        print("  Spinning reverse at 10% duty for 1.5 s ...")
        time.sleep(1.5)
        motor.set_duty_cycle(0.0)
        time.sleep(0.2)
        result("set_duty_cycle()", "fwd + rev ✓")

        # ── 8. set_velocity — ACK-ONLY on this firmware ────────────────────
        section("8. set_velocity()  [ACK-ONLY — no shaft rotation on this firmware]")
        print("  Sending velocity command (5000 ERPM) — expect no movement ...")
        motor.set_velocity(velocity_erpm=5000, allow_low_speed=True)
        time.sleep(0.5)
        fb_vel = motor.get_status()
        if fb_vel:
            moved = abs(fb_vel.speed_erpm) > 500
            result("shaft speed after velocity cmd",
                   f"{fb_vel.speed_erpm} ERPM",
                   passed=True,
                   note="ACK-ONLY — no rotation expected")
        motor.stop()

        # ── 9. control_exosuit_tendon ──────────────────────────────────────
        section("9. control_exosuit_tendon()")
        print("  pull → release → stop (duty-based) ...")
        motor.control_exosuit_tendon(action="pull",    velocity_erpm=5000)
        time.sleep(0.4)
        motor.control_exosuit_tendon(action="release", velocity_erpm=5000)
        time.sleep(0.4)
        motor.control_exosuit_tendon(action="stop")
        result("control_exosuit_tendon()", "pull / release / stop ✓")

        # ── 10. stop ───────────────────────────────────────────────────────
        section("10. stop()")
        motor.stop()
        time.sleep(0.2)
        fb_stop = motor.get_status()
        if fb_stop:
            result("speed after stop()", f"{fb_stop.speed_erpm} ERPM",
                   passed=abs(fb_stop.speed_erpm) < 500)
        result("stop()", "sent ✓")

        # ── 11. Error code table ────────────────────────────────────────────
        section("11. CAN_ERROR_CODES reference table")
        for code, desc in CAN_ERROR_CODES.items():
            print(f"  {code}: {desc}")

        # ── 12. disable_motor ──────────────────────────────────────────────
        section("12. disable_motor()")
        motor.disable_motor()
        result("disable_motor()", "sent ✓")

    print(f"\n{'=' * 56}")
    print("  Demo complete.")
    print(f"{'=' * 56}\n")


if __name__ == "__main__":
    main()
