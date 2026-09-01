#!/usr/bin/env python3
"""
conversions.py

Command-line converter between electrical RPM (ERPM) and mechanical
output-shaft speed (rad/s or deg/s) for CubeMars AK-series motors.

Mirrors the conversion math used in motor_python's motor class:
    _erpm_to_rad_s(erpm)  -> rad/s
    _rad_s_to_erpm(rad_s) -> erpm

Usage
-----
Convert ERPM -> deg/s (and rad/s):
    python scripts/conversions.py --motor ak80-6 --erpm 3000

Convert rad/s -> ERPM:
    python scripts/conversions.py --motor ak60-6_V3 --rad-s 60

 Convert deg/s -> ERPM:
     python scripts/conversions.py --motor ak80-6 --deg-s -385.507

Convert rad -> deg:
    python scripts/conversions.py --motor ak60-6_v3 --rad 12.56

Override pole pairs / gear ratio directly (skips --motor presets):
    python scripts/conversions.py --pole-pairs 21 --gear-ratio 6 --erpm 3000

You can pass --erpm, --rad-s, or --deg-s in the same call; each one
present gets converted and printed.
"""

import argparse
import math
from dataclasses import dataclass
from motor_python.definitions import MOTOR_SPECS


@dataclass(frozen=True)
class MotorSpec:
    pole_pairs: float
    gear_ratio: float


MOTOR_PRESETS = {
    "ak60-6_v3": MOTOR_SPECS["AK60-6_V3.0"],
    "ak80-6": MOTOR_SPECS["AK80-6"],
}


def erpm_to_rad_s(erpm: float, spec: MotorSpec) -> float:
    """Electrical RPM -> output-shaft mechanical rad/s."""
    return (
        float(erpm)
        * (2.0 * math.pi)
        / (60.0 * float(spec.pole_pairs) * float(spec.gear_ratio))
    )


def rad_s_to_erpm(rad_s: float, spec: MotorSpec) -> int:
    """Output-shaft mechanical rad/s -> electrical RPM."""
    return round(
        rad_s
        * (60.0 * float(spec.pole_pairs) * float(spec.gear_ratio))
        / (2.0 * math.pi)
    )


def rad_s_to_deg_s(rad_s: float) -> float:
    return rad_s * (180.0 / math.pi)


def deg_s_to_rad_s(deg_s: float) -> float:
    return deg_s * (math.pi / 180.0)

def rad_to_deg(rad: float) -> float:
    return rad * (180.0 / math.pi)


def build_spec(args: argparse.Namespace) -> MotorSpec:
    if args.pole_pairs is not None and args.gear_ratio is not None:
        return MotorSpec(pole_pairs=args.pole_pairs, gear_ratio=args.gear_ratio)
    if args.motor is None:
        raise SystemExit(
            "Specify either --motor {ak60-6,ak80-6} or both "
            "--pole-pairs and --gear-ratio."
        )
    return MOTOR_PRESETS[args.motor]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert between ERPM and rad/s or deg/s for AK-series motors."
    )
    parser.add_argument(
        "--motor",
        choices=sorted(MOTOR_PRESETS.keys()),
        help="Motor preset to use for pole_pairs/gear_ratio.",
    )
    parser.add_argument("--pole-pairs", type=float, help="Override pole pairs.")
    parser.add_argument("--gear-ratio", type=float, help="Override gear ratio.")

    parser.add_argument("--erpm", type=float, help="Input value in ERPM.")
    parser.add_argument("--rad-s", type=float, help="Input value in rad/s.")
    parser.add_argument("--deg-s", type=float, help="Input value in deg/s.")
    parser.add_argument("--rad", type=float, help="Input value in rad.")

    args = parser.parse_args()

    if args.erpm is None and args.rad_s is None and args.deg_s is None and args.rad is None:
        parser.error("Provide at least one of --erpm, --rad-s, --deg-s, or --rad.")

    spec = build_spec(args)
    print(f"Using motor spec: pole_pairs={spec.pole_pairs}, gear_ratio={spec.gear_ratio}\n")

    if args.erpm is not None:
        rad_s = erpm_to_rad_s(args.erpm, spec)
        deg_s = rad_s_to_deg_s(rad_s)
        print(f"{args.erpm:g} ERPM  ->  {rad_s:.6f} rad/s  ->  {deg_s:.4f} deg/s")

    if args.rad_s is not None:
        erpm = rad_s_to_erpm(args.rad_s, spec)
        print(f"{args.rad_s:g} rad/s  ->  {erpm} ERPM")

    if args.deg_s is not None:
        rad_s_val = deg_s_to_rad_s(args.deg_s)
        erpm = rad_s_to_erpm(rad_s_val, spec)
        print(f"{args.deg_s:g} deg/s  ->  {rad_s_val:.6f} rad/s  ->  {erpm} ERPM")

    if args.rad is not None:
        deg_val = rad_to_deg(args.rad)
        print(f"{args.rad:g} rad  ->  {deg_val:.6f} deg")


if __name__ == "__main__":
    main()
