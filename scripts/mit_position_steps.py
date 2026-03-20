#!/usr/bin/env python3
"""MIT position-step exercise for one AK60-6 motor.

Default behavior:
- Move in 30-degree position steps
- Run for 120 seconds
- Bounce between -90 and +90 degrees

Run:
    sudo ./setup_can.sh
    .venv/bin/python scripts/mit_position_steps.py --motor-id 0x03
"""
# ruff: noqa: T201, S110

from __future__ import annotations

import argparse
import math
import sys
import time
from itertools import cycle

from motor_python.base_motor import MotorState
from motor_python.can_utils import get_can_state, reset_can_interface
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

SEPARATOR = "=" * 72
HEALTHY_TX_ERR_MAX = 96
HEALTHY_RX_ERR_MAX = 64


def _is_can_state_healthy(state: dict[str, int | str]) -> bool:
    """Return True when CAN state is healthy enough for controlled runs."""
    return (
        state["state"] == "ERROR-ACTIVE"
        and int(state["tx_err"]) < HEALTHY_TX_ERR_MAX
        and int(state["rx_err"]) < HEALTHY_RX_ERR_MAX
    )


def _ensure_can_ready(interface: str, bitrate: int) -> None:
    """Attempt automatic recovery when CAN interface is not ERROR-ACTIVE."""
    state = get_can_state(interface)
    if _is_can_state_healthy(state):
        return

    print(
        f"CAN preflight: state={state['state']} tx_err={state['tx_err']} rx_err={state['rx_err']}"
    )
    print("CAN preflight: attempting automatic kernel-level CAN reset ...")

    if not reset_can_interface(interface=interface, bitrate=bitrate):
        raise RuntimeError(
            "CAN preflight reset failed. Run `sudo ./setup_can.sh` manually and retry."
        )

    after = get_can_state(interface)
    print(
        f"CAN preflight after reset: state={after['state']} tx_err={after['tx_err']} rx_err={after['rx_err']}"
    )

    if not _is_can_state_healthy(after):
        raise RuntimeError(
            "CAN bus still unhealthy after reset (error frames dominate). "
            "Check wiring/termination/power/UART disconnect before retrying."
        )


def _build_targets(
    min_deg: float, max_deg: float, step_deg: float, start_deg: float
) -> list[float]:
    """Build a bouncing target list (min -> max -> min) with fixed steps."""
    if step_deg <= 0:
        raise ValueError("--step-deg must be > 0")
    if max_deg <= min_deg:
        raise ValueError("--max-deg must be greater than --min-deg")

    positions: list[float] = []
    pos = min_deg
    while pos <= max_deg + 1e-9:
        positions.append(round(pos, 6))
        pos += step_deg

    if len(positions) < 2:
        raise ValueError("Need at least two targets; adjust min/max/step")

    # Bounce back without duplicating endpoints.
    targets = positions + positions[-2:0:-1]

    # Rotate so sequence starts close to the desired start angle.
    start_index = min(range(len(targets)), key=lambda idx: abs(targets[idx] - start_deg))
    return targets[start_index:] + targets[:start_index]


def _read_status(motor: CubeMarsAK606v3CAN, timeout: float = 0.25) -> MotorState | None:
    """Get the freshest available motor status sample."""
    status = motor._receive_feedback(timeout=timeout)
    if status is None:
        status = motor.get_status()
    return status


def _print_step_line(
    elapsed_s: float, step_index: int, target_deg: float, status: MotorState | None
) -> None:
    """Print one concise step result line."""
    if status is None:
        print(
            f"t={elapsed_s:6.1f}s  step={step_index:04d}  target={target_deg:7.2f} deg  feedback=none"
        )
        return

    print(
        f"t={elapsed_s:6.1f}s  step={step_index:04d}  target={target_deg:7.2f} deg  "
        f"pos={status.position_degrees:7.2f} deg  vel={status.speed_erpm:7d} ERPM  "
        f"cur={status.current_amps:6.2f} A  err={status.error_code}"
    )


def _compute_transition_seconds(
    delta_deg: float,
    min_transition_s: float,
    max_speed_deg_s: float,
    max_accel_deg_s2: float,
) -> float:
    """Compute smoothstep transition time from speed/acceleration constraints."""
    delta_abs = abs(delta_deg)
    if delta_abs <= 1e-9:
        return 0.0

    # For smoothstep s(u)=3u^2-2u^3:
    # max|velocity| ~= 1.5 * delta / T
    # max|accel|    ~= 6.0 * delta / T^2
    t_speed = (1.5 * delta_abs / max_speed_deg_s) if max_speed_deg_s > 0 else 0.0
    t_accel = (
        (6.0 * delta_abs / max_accel_deg_s2) ** 0.5 if max_accel_deg_s2 > 0 else 0.0
    )
    return max(min_transition_s, t_speed, t_accel)


def _move_smooth(  # noqa: PLR0913
    motor: CubeMarsAK606v3CAN,
    start_deg: float,
    target_deg: float,
    transition_s: float,
    control_hz: float,
    deadline: float,
) -> tuple[float, MotorState | None]:
    """Move from start_deg to target_deg with smoothstep interpolation."""
    if transition_s <= 0:
        motor.set_position(target_deg)
        return target_deg, _read_status(motor, timeout=0.15)

    period_s = 1.0 / max(control_hz, 1.0)
    steps = max(1, round(transition_s * control_hz))
    last_status: MotorState | None = None
    cmd_pos = start_deg
    next_tick = time.monotonic()

    for i in range(1, steps + 1):
        if time.monotonic() >= deadline:
            return cmd_pos, last_status

        u = i / steps
        smooth_u = (3.0 * u * u) - (2.0 * u * u * u)
        cmd_pos = start_deg + ((target_deg - start_deg) * smooth_u)
        motor.set_position(cmd_pos)

        last_status = _read_status(motor, timeout=min(0.08, period_s))
        if last_status is not None and last_status.error_code != 0:
            raise RuntimeError(
                f"Motor fault code {last_status.error_code}: {last_status.error_description}"
            )

        next_tick += period_s
        wait_s = next_tick - time.monotonic()
        if wait_s > 0:
            time.sleep(wait_s)

    return target_deg, last_status


def _move_snappy(  # noqa: PLR0913
    motor: CubeMarsAK606v3CAN,
    target_deg: float,
    kp: float,
    kd: float,
    settle_timeout_s: float,
    pos_tolerance_deg: float,
    deadline: float,
) -> tuple[float, MotorState | None]:
    """Move quickly to target_deg using direct MIT position impedance command."""
    motor.set_mit_mode(
        pos_rad=math.radians(target_deg),
        vel_rad_s=0.0,
        kp=kp,
        kd=kd,
        torque_ff_nm=0.0,
    )

    step_start = time.monotonic()
    last_status: MotorState | None = None
    while time.monotonic() - step_start < settle_timeout_s:
        if time.monotonic() >= deadline:
            break

        last_status = _read_status(motor, timeout=0.08)
        if last_status is not None:
            if last_status.error_code != 0:
                raise RuntimeError(
                    f"Motor fault code {last_status.error_code}: {last_status.error_description}"
                )

            pos_ok = abs(last_status.position_degrees - target_deg) <= pos_tolerance_deg
            speed_ok = abs(last_status.speed_erpm) <= 1400
            if pos_ok and speed_ok:
                break
        time.sleep(0.02)

    return target_deg, last_status


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description="MIT position-step runner (default: 30 deg steps for 120 s)"
    )
    parser.add_argument(
        "--interface",
        default="can0",
        help="SocketCAN interface (default: can0)",
    )
    parser.add_argument(
        "--motor-id",
        type=lambda x: int(x, 0),
        default=0x03,
        help="Motor CAN ID (default: 0x03)",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=1_000_000,
        help="CAN bitrate (default: 1000000)",
    )
    parser.add_argument(
        "--duration-seconds",
        type=float,
        default=120.0,
        help="Total runtime (default: 120)",
    )
    parser.add_argument(
        "--step-deg",
        type=float,
        default=30.0,
        help="Position step size in degrees (default: 30)",
    )
    parser.add_argument(
        "--dwell-seconds",
        type=float,
        default=0.5,
        help="Time to hold each target in seconds (default: 0.5)",
    )
    parser.add_argument(
        "--min-deg",
        type=float,
        default=-90.0,
        help="Minimum target angle in degrees (default: -90)",
    )
    parser.add_argument(
        "--max-deg",
        type=float,
        default=90.0,
        help="Maximum target angle in degrees (default: 90)",
    )
    parser.add_argument(
        "--start-deg",
        type=float,
        default=None,
        help="Start sequence near this angle (default: auto from live motor position)",
    )
    parser.add_argument(
        "--mode",
        choices=["helper", "snappy", "smooth"],
        default="helper",
        help="Motion mode: helper (stable), snappy, or smooth (default: helper)",
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=70.0,
        help="MIT position stiffness for snappy mode (default: 70)",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=3.0,
        help="MIT damping for snappy mode (default: 3.0)",
    )
    parser.add_argument(
        "--far-error-deg",
        type=float,
        default=25.0,
        help="If target error exceeds this, apply far-error gain cap (default: 25)",
    )
    parser.add_argument(
        "--kp-far-cap",
        type=float,
        default=38.0,
        help="Max kp to use on large-error snappy steps (default: 38)",
    )
    parser.add_argument(
        "--settle-timeout-seconds",
        type=float,
        default=0.45,
        help="Max wait time per snappy step for settling (default: 0.45)",
    )
    parser.add_argument(
        "--position-tolerance-deg",
        type=float,
        default=2.0,
        help="Target tolerance for snappy settling check in degrees (default: 2.0)",
    )
    parser.add_argument(
        "--max-speed-deg-s",
        type=float,
        default=40.0,
        help="Max commanded transition speed in deg/s (default: 40)",
    )
    parser.add_argument(
        "--max-accel-deg-s2",
        type=float,
        default=120.0,
        help="Max commanded transition acceleration in deg/s^2 (default: 120)",
    )
    parser.add_argument(
        "--min-transition-seconds",
        type=float,
        default=0.6,
        help="Minimum per-step transition duration in seconds (default: 0.6)",
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=50.0,
        help="Position command update rate during transitions (default: 50)",
    )
    parser.add_argument(
        "--helper-policy",
        choices=["strict", "fcfd", "legacy"],
        default="fcfd",
        help="MIT helper-frame policy (default: fcfd)",
    )
    parser.add_argument(
        "--allow-legacy-feedback-ids",
        action="store_true",
        help="Accept legacy non-canonical feedback IDs",
    )
    parser.add_argument(
        "--skip-preflight",
        action="store_true",
        help="Skip automatic `sudo ./setup_can.sh` preflight reset.",
    )
    return parser.parse_args()


def main() -> int:  # noqa: C901, PLR0911, PLR0912, PLR0915
    """Run the MIT position-step sequence."""
    args = parse_args()
    if args.duration_seconds <= 0:
        print("FAIL: --duration-seconds must be > 0")
        return 1
    if args.dwell_seconds <= 0:
        print("FAIL: --dwell-seconds must be > 0")
        return 1
    if args.max_speed_deg_s <= 0:
        print("FAIL: --max-speed-deg-s must be > 0")
        return 1
    if args.max_accel_deg_s2 <= 0:
        print("FAIL: --max-accel-deg-s2 must be > 0")
        return 1
    if args.min_transition_seconds < 0:
        print("FAIL: --min-transition-seconds must be >= 0")
        return 1
    if args.control_hz <= 0:
        print("FAIL: --control-hz must be > 0")
        return 1
    if args.kp < 0:
        print("FAIL: --kp must be >= 0")
        return 1
    if args.kd < 0:
        print("FAIL: --kd must be >= 0")
        return 1
    if args.far_error_deg <= 0:
        print("FAIL: --far-error-deg must be > 0")
        return 1
    if args.kp_far_cap < 0:
        print("FAIL: --kp-far-cap must be >= 0")
        return 1
    if args.settle_timeout_seconds <= 0:
        print("FAIL: --settle-timeout-seconds must be > 0")
        return 1
    if args.position_tolerance_deg <= 0:
        print("FAIL: --position-tolerance-deg must be > 0")
        return 1

    print(SEPARATOR)
    print("AK60-6 MIT Position-Step Script")
    print(SEPARATOR)
    print(f"Interface            : {args.interface}")
    print(f"Motor ID             : 0x{args.motor_id:02X}")
    print(f"Bitrate              : {args.bitrate}")
    print(f"Duration             : {args.duration_seconds:.1f} s")
    print(f"Step                 : {args.step_deg:.1f} deg")
    print(f"Dwell                : {args.dwell_seconds:.2f} s")
    print(f"Mode                 : {args.mode}")
    if args.mode == "smooth":
        print(f"Max speed            : {args.max_speed_deg_s:.1f} deg/s")
        print(f"Max accel            : {args.max_accel_deg_s2:.1f} deg/s^2")
        print(f"Min transition       : {args.min_transition_seconds:.2f} s")
        print(f"Control rate         : {args.control_hz:.1f} Hz")
    elif args.mode == "snappy":
        print(f"Kp / Kd              : {args.kp:.1f} / {args.kd:.2f}")
        print(f"Settle timeout       : {args.settle_timeout_seconds:.2f} s")
        print(f"Position tolerance   : {args.position_tolerance_deg:.2f} deg")
    else:
        print("Profile              : original set_position() helper behavior")
    print(f"Range                : [{args.min_deg:.1f}, {args.max_deg:.1f}] deg")
    if args.start_deg is None:
        print("Start near           : auto (live feedback)")
    else:
        print(f"Start near           : {args.start_deg:.1f} deg")
    print(f"Helper policy        : {args.helper_policy}")
    print(f"Legacy feedback IDs  : {args.allow_legacy_feedback_ids}")
    print(f"Preflight            : {'skip' if args.skip_preflight else 'auto-reset if needed'}")
    print("Targets cycle        : pending (computed after live feedback)")
    print("Safety               : keep load clear; be ready to cut power")
    if args.skip_preflight:
        state = get_can_state(args.interface)
        print(
            "CAN preflight skipped: "
            f"state={state['state']} tx_err={state['tx_err']} rx_err={state['rx_err']}"
        )
    else:
        _ensure_can_ready(args.interface, args.bitrate)

    motor: CubeMarsAK606v3CAN | None = None
    try:
        motor = CubeMarsAK606v3CAN(
            motor_can_id=args.motor_id,
            interface=args.interface,
            bitrate=args.bitrate,
            helper_policy=args.helper_policy,
            allow_legacy_feedback_ids=args.allow_legacy_feedback_ids,
        )
        if not motor.connected:
            print("\nFAIL: CAN interface not connected")
            return 1

        print("\nChecking communication...")
        if not motor.check_communication():
            print("FAIL: motor did not respond")
            return 1
        print("PASS: communication OK")

        status0 = _read_status(motor, timeout=0.3)
        live_start = status0.position_degrees if status0 is not None else 0.0
        start_near = args.start_deg if args.start_deg is not None else live_start
        targets = _build_targets(
            min_deg=args.min_deg,
            max_deg=args.max_deg,
            step_deg=args.step_deg,
            start_deg=start_near,
        )
        print(
            f"Initial feedback pos : "
            f"{(status0.position_degrees if status0 is not None else float('nan')):.2f} deg"
        )
        print(f"Start near (resolved): {start_near:.2f} deg")
        print(f"Targets cycle        : {targets}")

        print("\nStarting stepped motion...")
        start = time.monotonic()
        deadline = start + args.duration_seconds
        step_index = 0
        cmd_pos_deg = start_near
        last_status = status0

        for target in cycle(targets):
            now = time.monotonic()
            if now >= deadline:
                break

            step_index += 1
            if args.mode == "smooth":
                transition_s = _compute_transition_seconds(
                    delta_deg=target - cmd_pos_deg,
                    min_transition_s=args.min_transition_seconds,
                    max_speed_deg_s=args.max_speed_deg_s,
                    max_accel_deg_s2=args.max_accel_deg_s2,
                )
                cmd_pos_deg, status = _move_smooth(
                    motor=motor,
                    start_deg=cmd_pos_deg,
                    target_deg=target,
                    transition_s=transition_s,
                    control_hz=args.control_hz,
                    deadline=deadline,
                )
            elif args.mode == "snappy":
                transition_s = 0.0
                current_est = (
                    last_status.position_degrees
                    if last_status is not None
                    else cmd_pos_deg
                )
                err_deg = abs(target - current_est)
                kp_cmd = (
                    min(args.kp, args.kp_far_cap)
                    if err_deg > args.far_error_deg
                    else args.kp
                )
                cmd_pos_deg, status = _move_snappy(
                    motor=motor,
                    target_deg=target,
                    kp=kp_cmd,
                    kd=args.kd,
                    settle_timeout_s=args.settle_timeout_seconds,
                    pos_tolerance_deg=args.position_tolerance_deg,
                    deadline=deadline,
                )
            else:
                transition_s = 0.0
                motor.set_position(target)
                cmd_pos_deg = target
                status = _read_status(motor, timeout=min(0.3, args.dwell_seconds))
            elapsed = now - start
            _print_step_line(elapsed, step_index, target, status)
            if transition_s > 0.0:
                print(f"  transition={transition_s:.2f}s")
            elif args.mode == "snappy":
                print(
                    f"  snappy: kp={kp_cmd:.1f} kd={args.kd:.2f} "
                    f"(err_est={err_deg:.1f} deg)"
                )

            if status is not None and status.error_code != 0:
                raise RuntimeError(
                    f"Motor fault code {status.error_code}: {status.error_description}"
                )
            last_status = status if status is not None else last_status

            sleep_until = min(deadline, time.monotonic() + args.dwell_seconds)
            while time.monotonic() < sleep_until:
                time.sleep(min(0.1, sleep_until - time.monotonic()))

        print("\nPASS: position-step run completed")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 130
    except Exception as exc:
        print(f"\nFAIL: {exc}")
        return 1
    finally:
        if motor is not None:
            try:
                motor.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
