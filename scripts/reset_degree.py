#!/usr/bin/env python3
"""Reset motor position to a chosen degree in MIT command space.

Important:
- MIT-only firmware path does not support true encoder-origin reset (`set_origin`).
- This script performs a practical reset by smoothly commanding position to a
  target (default 0 deg), so subsequent MIT position scripts start from a
  known, centered command angle.

Example:
    .venv/bin/python scripts/reset_degree.py --motor-id 0x03 --target-deg 0
"""
# ruff: noqa: T201

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

if __package__ in {None, ""}:
    repo_src = Path(__file__).resolve().parents[1] / "src"
    if str(repo_src) not in sys.path:
        sys.path.insert(0, str(repo_src))

from motor_python.base_motor import MotorState
from motor_python.can_utils import get_can_state, reset_can_interface
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

SEPARATOR = "=" * 72
MIT_POSITION_LIMIT_DEG = math.degrees(12.56)
HEALTHY_TX_ERR_MAX = 96
HEALTHY_RX_ERR_MAX = 64


def _clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def _is_can_state_healthy(state: dict[str, int | str]) -> bool:
    return (
        state["state"] == "ERROR-ACTIVE"
        and int(state["tx_err"]) < HEALTHY_TX_ERR_MAX
        and int(state["rx_err"]) < HEALTHY_RX_ERR_MAX
    )


def _ensure_can_ready(interface: str, bitrate: int) -> None:
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
        raise RuntimeError("CAN bus still unhealthy after reset.")


def _read_status(motor: CubeMarsAK606v3CAN, timeout: float = 0.10) -> MotorState | None:
    status = motor._receive_feedback(timeout=timeout)
    if status is None:
        status = motor.get_status()
    return status


def _move_to_target(  # noqa: PLR0913
    motor: CubeMarsAK606v3CAN,
    *,
    start_deg: float,
    target_deg: float,
    velocity_deg_s: float,
    control_hz: float,
    tolerance_deg: float,
    timeout_s: float,
) -> MotorState | None:
    distance = abs(target_deg - start_deg)
    if distance <= tolerance_deg:
        return _read_status(motor)

    move_time = max(distance / max(velocity_deg_s, 1e-6), 0.3)
    move_time = min(move_time, timeout_s)
    steps = max(1, round(move_time * control_hz))
    period_s = move_time / steps

    next_tick = time.monotonic()
    last_status: MotorState | None = None

    for step in range(1, steps + 1):
        u = step / steps
        smooth_u = (3.0 * u * u) - (2.0 * u * u * u)
        cmd_deg = start_deg + ((target_deg - start_deg) * smooth_u)

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

    return last_status


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Reset/recenter position to target degree in MIT command space"
    )
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--motor-id", type=lambda v: int(v, 0), default=0x03)
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument(
        "--target-deg",
        type=float,
        default=0.0,
        help="Target reset degree in MIT command space (default: 0)",
    )
    parser.add_argument("--velocity-deg-s", type=float, default=60.0)
    parser.add_argument("--control-hz", type=float, default=40.0)
    parser.add_argument("--tolerance-deg", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument(
        "--helper-policy",
        choices=["strict", "fcfd", "legacy"],
        default="fcfd",
    )
    parser.add_argument("--allow-legacy-feedback-ids", action="store_true")
    parser.add_argument("--skip-preflight", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.velocity_deg_s <= 0:
        print("FAIL: --velocity-deg-s must be > 0")
        return 1
    if args.control_hz <= 0:
        print("FAIL: --control-hz must be > 0")
        return 1
    if args.tolerance_deg <= 0:
        print("FAIL: --tolerance-deg must be > 0")
        return 1
    if args.timeout <= 0:
        print("FAIL: --timeout must be > 0")
        return 1

    target_deg = _clamp(args.target_deg, -MIT_POSITION_LIMIT_DEG, MIT_POSITION_LIMIT_DEG)
    if abs(target_deg - args.target_deg) > 0.1:
        print(
            f"WARN: target {args.target_deg:.2f} deg exceeds MIT range; clamped to {target_deg:.2f} deg"
        )

    print(SEPARATOR)
    print("AK60-6 MIT Degree Reset Script")
    print(SEPARATOR)
    print(f"Interface            : {args.interface}")
    print(f"Motor ID             : 0x{args.motor_id:02X}")
    print(f"Target degree        : {target_deg:.2f}")
    print(f"Velocity             : {args.velocity_deg_s:.2f} deg/s")
    print(f"Control rate         : {args.control_hz:.1f} Hz")
    print(f"Tolerance            : {args.tolerance_deg:.2f} deg")
    print(f"Timeout              : {args.timeout:.1f} s")
    print(f"Preflight            : {'skip' if args.skip_preflight else 'auto-reset if needed'}")

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
            print("FAIL: CAN interface not connected")
            return 1

        if not motor.check_communication():
            print("FAIL: motor did not respond")
            return 1

        status0 = _read_status(motor, timeout=0.3)
        feedback_deg = status0.position_degrees if status0 is not None else 0.0
        start_cmd_deg = _clamp(feedback_deg, -MIT_POSITION_LIMIT_DEG, MIT_POSITION_LIMIT_DEG)

        print(f"Feedback start deg   : {feedback_deg:.2f}")
        print(f"Command start deg    : {start_cmd_deg:.2f}")

        status_end = _move_to_target(
            motor,
            start_deg=start_cmd_deg,
            target_deg=target_deg,
            velocity_deg_s=args.velocity_deg_s,
            control_hz=args.control_hz,
            tolerance_deg=args.tolerance_deg,
            timeout_s=args.timeout,
        )

        final_feedback = status_end.position_degrees if status_end is not None else None
        if final_feedback is None:
            final_feedback = motor.get_position()

        print(f"Feedback final deg   : {final_feedback}")
        print("PASS: reset move completed")
        return 0

    except KeyboardInterrupt:
        print("Interrupted by user")
        return 130
    except Exception as exc:
        print(f"FAIL: {exc}")
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
