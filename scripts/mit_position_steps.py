#!/usr/bin/env python3
"""MIT position step script for long motion-capture runs.

This script avoids MIT position-limit lockups by stepping back-and-forth
(ping-pong) inside a safe command window.

Primary controls:
- --angle-deg       : step size per tick
- --duration        : total runtime
- --velocity-deg-s  : movement speed used for each segment

Example:
    .venv/bin/python scripts/mit_position_steps.py --motor-id 0x03 --angle-deg 30 --duration 180 --velocity-deg-s 20
"""
# ruff: noqa: T201

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

# Allow running as plain `python scripts/mit_position_steps.py` from repo root.
if __package__ in {None, ""}:
    repo_src = Path(__file__).resolve().parents[1] / "src"
    if str(repo_src) not in sys.path:
        sys.path.insert(0, str(repo_src))

from motor_python.base_motor import MotorState
from motor_python.can_utils import get_can_state, reset_can_interface
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

SEPARATOR = "=" * 72
HEALTHY_TX_ERR_MAX = 96
HEALTHY_RX_ERR_MAX = 64
MIT_POSITION_LIMIT_DEG = math.degrees(12.56)
MIN_TRAVEL_EPS_DEG = 0.5

# ---------------------------------------------------------------------------
# Quick settings (edit here if you prefer code-based tuning)
# ---------------------------------------------------------------------------
DEFAULT_ANGLE_DEG = 90.0
DEFAULT_DURATION_S = 180.0
DEFAULT_VELOCITY_DEG_S = 100.0
DEFAULT_TICK_PAUSE_S = 0.02
DEFAULT_CONTROL_HZ = 20.0
DEFAULT_SWEEP_MIN_DEG = -650.0
DEFAULT_SWEEP_MAX_DEG = 650.0


def _clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp value into [min_value, max_value]."""
    return max(min_value, min(max_value, value))


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
            "CAN bus still unhealthy after reset. "
            "Check wiring/termination/power/UART disconnect before retrying."
        )


def _read_status(motor: CubeMarsAK606v3CAN, timeout: float = 0.10) -> MotorState | None:
    """Get freshest available motor status sample."""
    status = motor._receive_feedback(timeout=timeout)
    if status is None:
        status = motor.get_status()
    return status


def _print_tick_line(
    elapsed_s: float,
    tick_index: int,
    cmd_target_deg: float,
    direction: int,
    status: MotorState | None,
) -> None:
    """Print one concise tick summary line."""
    arrow = ">" if direction > 0 else "<"
    if status is None:
        print(
            f"t={elapsed_s:6.2f}s  tick={tick_index:04d}  dir={arrow}  "
            f"target={cmd_target_deg:8.2f} deg  feedback=none"
        )
        return

    print(
        f"t={elapsed_s:6.2f}s  tick={tick_index:04d}  dir={arrow}  "
        f"target={cmd_target_deg:8.2f} deg  "
        f"pos={status.position_degrees:8.2f} deg  "
        f"vel={status.speed_erpm:7d} ERPM  "
        f"cur={status.current_amps:6.2f} A  err={status.error_code}"
    )


def _move_segment(  # noqa: PLR0913
    motor: CubeMarsAK606v3CAN,
    *,
    start_deg: float,
    delta_deg: float,
    velocity_deg_s: float,
    control_hz: float,
    deadline: float,
) -> tuple[float, MotorState | None]:
    """Move one segment with smooth interpolation and bounded velocity."""
    segment_time = abs(delta_deg) / max(velocity_deg_s, 1e-9)
    if segment_time <= 1e-9:
        return start_deg, _read_status(motor, timeout=0.05)

    steps = max(1, round(segment_time * control_hz))
    period_s = segment_time / steps

    next_tick = time.monotonic()
    cmd_deg = start_deg
    last_status: MotorState | None = None

    for step in range(1, steps + 1):
        if time.monotonic() >= deadline:
            break

        u = step / steps
        smooth_u = (3.0 * u * u) - (2.0 * u * u * u)
        cmd_deg = start_deg + (delta_deg * smooth_u)

        motor.set_position(cmd_deg)
        last_status = _read_status(motor, timeout=min(0.08, period_s))
        if last_status is not None and last_status.error_code != 0:
            raise RuntimeError(
                f"Motor fault code {last_status.error_code}: {last_status.error_description}"
            )

        next_tick += period_s
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)

    return cmd_deg, last_status


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Long-run MIT position stepping (ping-pong in safe window). "
            "Tune angle, duration, velocity."
        )
    )
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
        "--angle-deg",
        type=float,
        default=DEFAULT_ANGLE_DEG,
        help=(
            "Tick angle in degrees. Sign selects initial direction "
            f"(default: {DEFAULT_ANGLE_DEG})"
        ),
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help=f"Total run duration in seconds (default: {DEFAULT_DURATION_S})",
    )
    parser.add_argument(
        "--velocity-deg-s",
        type=float,
        default=DEFAULT_VELOCITY_DEG_S,
        help=(
            "Movement speed for each tick segment in deg/s "
            f"(default: {DEFAULT_VELOCITY_DEG_S})"
        ),
    )
    parser.add_argument(
        "--tick-pause",
        type=float,
        default=DEFAULT_TICK_PAUSE_S,
        help=(
            "Pause after each tick in seconds (set 0 for continuous stepping, "
            f"default: {DEFAULT_TICK_PAUSE_S})"
        ),
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=DEFAULT_CONTROL_HZ,
        help=f"Position command update rate (default: {DEFAULT_CONTROL_HZ})",
    )
    parser.add_argument(
        "--min-deg",
        type=float,
        default=DEFAULT_SWEEP_MIN_DEG,
        help=f"Lower bound of ping-pong window (default: {DEFAULT_SWEEP_MIN_DEG})",
    )
    parser.add_argument(
        "--max-deg",
        type=float,
        default=DEFAULT_SWEEP_MAX_DEG,
        help=f"Upper bound of ping-pong window (default: {DEFAULT_SWEEP_MAX_DEG})",
    )
    parser.add_argument(
        "--start-deg",
        type=float,
        default=None,
        help=(
            "Optional start command angle in degrees. "
            "If omitted, feedback is clamped into [min-deg, max-deg]."
        ),
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
        help="Skip automatic preflight reset.",
    )
    return parser.parse_args()


def main() -> int:
    """Run long ping-pong MIT stepping sequence."""
    args = parse_args()

    if abs(args.angle_deg) < 1e-9:
        print("FAIL: --angle-deg must be non-zero")
        return 1
    if args.duration <= 0:
        print("FAIL: --duration must be > 0")
        return 1
    if args.velocity_deg_s <= 0:
        print("FAIL: --velocity-deg-s must be > 0")
        return 1
    if args.control_hz <= 0:
        print("FAIL: --control-hz must be > 0")
        return 1
    if args.tick_pause < 0:
        print("FAIL: --tick-pause must be >= 0")
        return 1

    safe_min = _clamp(args.min_deg, -MIT_POSITION_LIMIT_DEG, MIT_POSITION_LIMIT_DEG)
    safe_max = _clamp(args.max_deg, -MIT_POSITION_LIMIT_DEG, MIT_POSITION_LIMIT_DEG)
    if safe_max <= safe_min:
        print("FAIL: --max-deg must be greater than --min-deg after MIT clamping")
        return 1

    print(SEPARATOR)
    print("AK60-6 MIT Position Steps (Long Ping-Pong)")
    print(SEPARATOR)
    print(f"Interface            : {args.interface}")
    print(f"Motor ID             : 0x{args.motor_id:02X}")
    print(f"Bitrate              : {args.bitrate}")
    print(f"Step angle           : {args.angle_deg:.2f} deg")
    print(f"Total duration       : {args.duration:.2f} s")
    print(f"Step velocity        : {args.velocity_deg_s:.2f} deg/s")
    print(f"Tick pause           : {args.tick_pause:.2f} s")
    print(f"Control rate         : {args.control_hz:.1f} Hz")
    print(f"Sweep window         : [{safe_min:.2f}, {safe_max:.2f}] deg")
    print(f"Helper policy        : {args.helper_policy}")
    print(f"Legacy feedback IDs  : {args.allow_legacy_feedback_ids}")
    print(f"Preflight            : {'skip' if args.skip_preflight else 'auto-reset if needed'}")
    print("Safety               : keep load clear; be ready to cut power")

    if abs(safe_min - args.min_deg) > 0.1 or abs(safe_max - args.max_deg) > 0.1:
        print(
            "WARN: sweep window clamped to MIT limits "
            f"(requested [{args.min_deg:.2f}, {args.max_deg:.2f}] deg)."
        )

    motor: CubeMarsAK606v3CAN | None = None
    try:
        if args.skip_preflight:
            state = get_can_state(args.interface)
            print(
                "CAN preflight skipped: "
                f"state={state['state']} tx_err={state['tx_err']} rx_err={state['rx_err']}"
            )
        else:
            _ensure_can_ready(args.interface, args.bitrate)

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
        feedback_start_deg = status0.position_degrees if status0 is not None else 0.0

        if args.start_deg is None:
            current_cmd_deg = _clamp(feedback_start_deg, safe_min, safe_max)
            if abs(current_cmd_deg - feedback_start_deg) > 0.1:
                print(
                    "WARN: feedback start angle is outside sweep window; "
                    f"using clamped start {current_cmd_deg:.2f} deg "
                    f"(feedback={feedback_start_deg:.2f} deg)."
                )
        else:
            current_cmd_deg = _clamp(args.start_deg, safe_min, safe_max)
            if abs(current_cmd_deg - args.start_deg) > 0.1:
                print(
                    f"WARN: --start-deg={args.start_deg:.2f} exceeds sweep window; "
                    f"clamped to {current_cmd_deg:.2f} deg."
                )

        direction = 1 if args.angle_deg > 0 else -1
        tick_magnitude_deg = abs(args.angle_deg)

        print(f"Start position (cmd) : {current_cmd_deg:.2f} deg")
        if status0 is not None:
            print(f"Initial feedback pos : {status0.position_degrees:.2f} deg")
        print(f"Initial direction    : {'positive' if direction > 0 else 'negative'}")

        print("\nStarting ping-pong stepping...")
        run_start = time.monotonic()
        deadline = run_start + args.duration
        tick_index = 0
        turnarounds = 0
        last_status = status0

        while time.monotonic() < deadline:
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0:
                break

            max_delta_remaining = args.velocity_deg_s * remaining_s
            if max_delta_remaining <= MIN_TRAVEL_EPS_DEG:
                break

            room_in_direction = (
                safe_max - current_cmd_deg if direction > 0 else current_cmd_deg - safe_min
            )
            if room_in_direction < MIN_TRAVEL_EPS_DEG:
                direction *= -1
                turnarounds += 1
                time.sleep(0.01)
                continue

            delta_mag = min(tick_magnitude_deg, max_delta_remaining, room_in_direction)
            if delta_mag < MIN_TRAVEL_EPS_DEG:
                break

            delta_deg = direction * delta_mag
            next_target_deg = current_cmd_deg + delta_deg

            tick_index += 1
            current_cmd_deg, status = _move_segment(
                motor,
                start_deg=current_cmd_deg,
                delta_deg=delta_deg,
                velocity_deg_s=args.velocity_deg_s,
                control_hz=args.control_hz,
                deadline=deadline,
            )

            if status is not None and status.error_code != 0:
                raise RuntimeError(
                    f"Motor fault code {status.error_code}: {status.error_description}"
                )
            if status is not None:
                last_status = status

            _print_tick_line(
                elapsed_s=time.monotonic() - run_start,
                tick_index=tick_index,
                cmd_target_deg=next_target_deg,
                direction=direction,
                status=last_status,
            )

            pause_deadline = min(deadline, time.monotonic() + args.tick_pause)
            while time.monotonic() < pause_deadline:
                time.sleep(min(0.05, pause_deadline - time.monotonic()))

        if tick_index == 0:
            print(
                "FAIL: no ticks were sent. "
                "Try smaller --angle-deg, lower --velocity-deg-s, or wider sweep window."
            )
            return 1

        print("\nPASS: ping-pong run completed")
        print(f"Ticks sent            : {tick_index}")
        print(f"Turnarounds           : {turnarounds}")
        print(f"Final command pos     : {current_cmd_deg:.2f} deg")
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
                motor.stop()
            except Exception:
                pass
            try:
                motor.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
