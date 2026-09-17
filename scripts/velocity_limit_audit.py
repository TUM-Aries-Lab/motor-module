#!/usr/bin/env python3
"""Bench audit of the MIT velocity scale and the commandable velocity ceiling.

Answers two open questions about ``MotorSpec.max_velocity_electrical_rpm``:

1. **Is the MIT velocity field output-shaft or motor-shaft rad/s?**
   ``_rad_s_to_erpm()`` multiplies by ``pole_pairs * gear_ratio``, i.e. it treats
   the commanded rad/s as output-shaft. If that is right, the measured electrical
   RPM per commanded rad/s equals ``60 * pole_pairs * gear_ratio / 2pi``; if the
   field is really motor-shaft, it equals ``60 * pole_pairs / 2pi`` instead --
   a factor of ``gear_ratio`` apart, so the two are easy to tell apart.

   This only works on the **AK60-6 V3**, whose feedback is the servo-style status
   frame reporting electrical RPM -- a quantity independent of what was commanded.
   The AK80-6 and AK60-6 V1.1 reply with an MIT frame whose velocity is decoded on
   the same scale it was commanded on, so they cannot answer question 1 alone.

2. **Where does velocity actually stop tracking the command?**
   ``set_mit_mode()`` clamps to the spec ERPM range, which currently binds far
   below each motor's MIT encoding range. Pass ``--raise-limit`` to lift that
   clamp for the duration of the run (in memory only -- nothing on disk changes)
   and find the real ceiling.

SAFETY -- read before running:
    This spins the motor under velocity control. Run it on the bench with the
    TENDON DISCONNECTED and nothing attached to the output shaft that can wind
    up. Start low, keep phases short, and keep the power switch within reach.

Run:
    sudo ./setup_can.sh

    # Question 1 -- stays inside the existing clamp, low speed:
    .venv/bin/python scripts/velocity_limit_audit.py --motor-model AK60-6 \
        --motor-id 0x03 --max-rad 6 --step-rad 1

    # Question 2 -- lifts the clamp, goes to the encoding range:
    .venv/bin/python scripts/velocity_limit_audit.py --motor-model AK60-6 \
        --motor-id 0x03 --max-rad 30 --step-rad 3 --raise-limit
"""
# ruff: noqa: T201

from __future__ import annotations

import argparse
import math
import statistics
import time
from dataclasses import dataclass, replace
from itertools import pairwise
from pathlib import Path

from motor_python import create_can_motor
from motor_python.definitions import CAN_DEFAULTS, MOTOR_SPECS, MotorSpec
from motor_python.utils import CsvStreamWriter, create_timestamped_filepath

SEPARATOR = "=" * 78

#: A step counts as "still tracking" if measured ERPM grew by at least this
#: fraction of the growth the commanded step should have produced.
TRACKING_FRACTION = 0.5

CSV_FIELDS = (
    "commanded_rad_s",
    "mean_speed_erpm",
    "erpm_per_rad_s",
    "samples",
    "missed",
    "mean_current",
    "max_temp_c",
    "error_code",
)


@dataclass(frozen=True)
class SweepConfig:
    """Everything one sweep needs, so helpers stay under the argument limit."""

    start_rad: float
    max_rad: float
    step_rad: float
    kd: float
    phase_seconds: float
    settle_seconds: float
    sample_hz: float


@dataclass(frozen=True)
class StepResult:
    """Outcome of holding one commanded velocity."""

    commanded_rad_s: float
    mean_speed_erpm: float | None
    samples: int
    missed: int
    mean_current: float | None
    max_temp_c: float | None
    error_code: int

    @property
    def erpm_per_rad_s(self) -> float | None:
        """Measured electrical RPM per commanded rad/s, or None if unusable."""
        if self.mean_speed_erpm is None or self.commanded_rad_s == 0.0:
            return None
        return self.mean_speed_erpm / self.commanded_rad_s


def parse_args() -> argparse.Namespace:
    """Build the command line."""
    parser = argparse.ArgumentParser(
        description="Audit the MIT velocity scale and the velocity ceiling.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--motor-model", default="AK60-6", help="Motor model name")
    parser.add_argument(
        "--motor-id",
        type=lambda value: int(value, 0),
        default=CAN_DEFAULTS.motor_can_id,
        help="Motor CAN ID",
    )
    parser.add_argument("--interface", default=CAN_DEFAULTS.interface)
    parser.add_argument("--bitrate", type=int, default=CAN_DEFAULTS.bitrate)
    parser.add_argument("--start-rad", type=float, default=1.0)
    parser.add_argument("--max-rad", type=float, default=6.0)
    parser.add_argument("--step-rad", type=float, default=1.0)
    parser.add_argument("--kd", type=float, default=1.0, help="MIT velocity damping")
    parser.add_argument("--phase-seconds", type=float, default=2.0)
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=0.5,
        help="Discarded at the start of each step while the motor accelerates",
    )
    parser.add_argument("--sample-hz", type=float, default=50.0)
    parser.add_argument(
        "--raise-limit",
        action="store_true",
        help=(
            "Lift the spec ERPM clamp in memory for this run so the sweep can "
            "reach the MIT encoding range. Nothing on disk is modified."
        ),
    )
    parser.add_argument(
        "--csv-path",
        type=Path,
        default=None,
        help="Where to write per-step results (default: timestamped file in CSV/)",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the safety confirmation prompt",
    )
    return parser.parse_args()


def resolve_spec(model_name: str) -> MotorSpec:
    """Look up the MotorSpec the factory will use for this model name."""
    for spec in MOTOR_SPECS.values():
        if spec.model_name.upper().replace("_", "-") in model_name.upper().replace(
            "_", "-"
        ) or model_name.upper().replace("_", "-") in spec.model_name.upper().replace(
            "_", "-"
        ):
            return spec
    raise SystemExit(f"Could not resolve a MotorSpec for model '{model_name}'")


def confirm(spec: MotorSpec, cfg: SweepConfig, raise_limit: bool) -> None:
    """Print the safety banner and require an explicit go-ahead."""
    print(SEPARATOR)
    print("VELOCITY SWEEP -- the output shaft will spin.")
    print("  Tendon disconnected?  Nothing attached that can wind up?")
    print(
        f"  {spec.model_name}: sweeping {cfg.start_rad} -> {cfg.max_rad} rad/s "
        f"in steps of {cfg.step_rad}, kd={cfg.kd}"
    )
    if raise_limit:
        print("  --raise-limit is ON: the spec ERPM clamp will NOT protect you.")
    print(SEPARATOR)
    answer = input("Type 'yes' to start: ").strip().lower()
    if answer != "yes":
        raise SystemExit("Aborted.")


def hold_velocity(motor, velocity_rad_s: float, cfg: SweepConfig) -> StepResult:
    """Command one velocity, then sample feedback for the phase duration."""
    speeds: list[float] = []
    currents: list[float] = []
    temps: list[float] = []
    missed = 0
    error_code = 0

    period = 1.0 / cfg.sample_hz
    deadline = time.monotonic() + cfg.settle_seconds + cfg.phase_seconds
    sample_from = time.monotonic() + cfg.settle_seconds

    while time.monotonic() < deadline:
        motor.set_mit_mode(
            pos_rad=0.0,
            vel_rad_s=velocity_rad_s,
            kp=0.0,
            kd=cfg.kd,
            torque_ff_nm=0.0,
        )
        status = motor.get_status()
        if time.monotonic() >= sample_from:
            if status is None:
                missed += 1
            else:
                speeds.append(float(status.speed_erpm))
                currents.append(float(status.current_amps))
                temps.append(float(status.temperature_celsius))
                error_code = error_code or int(status.error_code)
        time.sleep(period)

    return StepResult(
        commanded_rad_s=velocity_rad_s,
        mean_speed_erpm=statistics.fmean(speeds) if speeds else None,
        samples=len(speeds),
        missed=missed,
        mean_current=statistics.fmean(currents) if currents else None,
        max_temp_c=max(temps) if temps else None,
        error_code=error_code,
    )


def steps(cfg: SweepConfig) -> list[float]:
    """Commanded velocities for the sweep, inclusive of max where it lands."""
    out: list[float] = []
    value = cfg.start_rad
    while value <= cfg.max_rad + 1e-9:
        out.append(round(value, 4))
        value += cfg.step_rad
    return out


def report(spec: MotorSpec, results: list[StepResult]) -> None:
    """Print the ratio table and the verdict on both open questions."""
    if_output = 60.0 * spec.pole_pairs * spec.gear_ratio / (2.0 * math.pi)
    if_motor = 60.0 * spec.pole_pairs / (2.0 * math.pi)

    print()
    print(SEPARATOR)
    print(
        f"{'cmd rad/s':>10s} {'mean ERPM':>12s} {'ERPM/rad/s':>12s} "
        f"{'n':>4s} {'miss':>5s} {'err':>4s}"
    )
    for r in results:
        ratio = r.erpm_per_rad_s
        print(
            f"{r.commanded_rad_s:10.2f} "
            f"{(r.mean_speed_erpm if r.mean_speed_erpm is not None else float('nan')):12.1f} "
            f"{(ratio if ratio is not None else float('nan')):12.1f} "
            f"{r.samples:4d} {r.missed:5d} {r.error_code:4d}"
        )

    print(SEPARATOR)
    print(f"Expected ERPM per rad/s if MIT velocity is OUTPUT-shaft: {if_output:8.1f}")
    print(f"Expected ERPM per rad/s if MIT velocity is MOTOR-shaft : {if_motor:8.1f}")

    usable = [r for r in results if r.erpm_per_rad_s is not None and r.samples > 0]
    if not usable:
        print("No usable samples -- is this the AK60-6 V3? Only it reports ERPM.")
        return

    # Use the lowest commanded steps, which are least likely to be clamped.
    early = [r.erpm_per_rad_s for r in usable[: max(1, len(usable) // 2)]]
    measured = statistics.fmean([value for value in early if value is not None])
    print(f"Measured (mean over the lower half of the sweep)       : {measured:8.1f}")

    if abs(measured - if_output) < abs(measured - if_motor):
        print("=> OUTPUT-shaft. The conversion is right and the ERPM limits are")
        print(
            "   missing the gear ratio; they should be multiplied by "
            f"{spec.gear_ratio}."
        )
    else:
        print("=> MOTOR-shaft. The ERPM limits are right and _rad_s_to_erpm()")
        print("   should NOT multiply by gear_ratio.")

    # The plateau value is the precise, noise-robust statement of the ceiling;
    # the bracketing loop below only says which step crossed it.
    peak = max(
        (r.mean_speed_erpm for r in usable if r.mean_speed_erpm is not None),
        default=None,
    )
    if peak is not None and measured > 0:
        clamp = spec.max_velocity_electrical_rpm
        print(
            f"Peak measured speed  : {peak:.0f} ERPM = {peak / measured:.2f} rad/s "
            f"(spec clamp {clamp} ERPM)"
        )
        if peak > clamp * 1.05:
            print("   Above the spec clamp, so --raise-limit was in effect.")
        elif peak >= clamp * 0.95:
            print("   The spec ERPM clamp is what bound the sweep.")
        else:
            print(
                "   Below the spec clamp -- something else limited it "
                "(supply, load, or the motor itself)."
            )

    # Ceiling: the first step whose measured speed failed to follow the command.
    for previous, current in pairwise(usable):
        if previous.mean_speed_erpm is None or current.mean_speed_erpm is None:
            continue
        commanded_growth = current.commanded_rad_s - previous.commanded_rad_s
        expected_growth = commanded_growth * measured
        actual_growth = current.mean_speed_erpm - previous.mean_speed_erpm
        if actual_growth < TRACKING_FRACTION * expected_growth:
            print(
                f"Velocity stopped tracking between {previous.commanded_rad_s:.2f} "
                f"and {current.commanded_rad_s:.2f} rad/s "
                f"(~{previous.mean_speed_erpm:.0f} ERPM)."
            )
            break
    else:
        print("Velocity tracked the command across the whole sweep -- no ceiling hit.")


def main() -> int:
    """Run the sweep and report."""
    args = parse_args()
    cfg = SweepConfig(
        start_rad=args.start_rad,
        max_rad=args.max_rad,
        step_rad=args.step_rad,
        kd=args.kd,
        phase_seconds=args.phase_seconds,
        settle_seconds=args.settle_seconds,
        sample_hz=args.sample_hz,
    )
    spec = resolve_spec(args.motor_model)

    if not args.yes:
        confirm(spec, cfg, args.raise_limit)

    if args.raise_limit:
        headroom = int(spec.max_output_speed_rpm * spec.gear_ratio * spec.pole_pairs)
        spec = replace(
            spec,
            max_velocity_electrical_rpm=headroom,
            min_velocity_electrical_rpm=-headroom,
        )
        print(f"ERPM clamp lifted for this run: +/-{headroom} (in memory only)")

    csv_path = args.csv_path or create_timestamped_filepath(
        suffix="csv", output_dir=Path("CSV"), prefix="velocity_limit_audit"
    )
    writer = CsvStreamWriter(csv_path, CSV_FIELDS)

    motor = create_can_motor(
        args.motor_model,
        motor_can_id=args.motor_id,
        interface=args.interface,
        bitrate=args.bitrate,
        motor_spec=spec,
    )
    results: list[StepResult] = []
    try:
        if not motor.check_communication():
            print("Motor not responding -- check power, CAN wiring and termination.")
            return 1
        motor.enable_mit_mode()

        for velocity in steps(cfg):
            result = hold_velocity(motor, velocity, cfg)
            results.append(result)
            ratio = result.erpm_per_rad_s
            print(
                f"  {velocity:6.2f} rad/s -> "
                f"{(result.mean_speed_erpm or float('nan')):9.1f} ERPM "
                f"(ratio {(ratio if ratio is not None else float('nan')):7.1f})"
            )
            writer.writerow(
                {
                    "commanded_rad_s": velocity,
                    "mean_speed_erpm": result.mean_speed_erpm,
                    "erpm_per_rad_s": ratio,
                    "samples": result.samples,
                    "missed": result.missed,
                    "mean_current": result.mean_current,
                    "max_temp_c": result.max_temp_c,
                    "error_code": result.error_code,
                }
            )
            if result.error_code:
                print(f"  motor reported error code {result.error_code} -- stopping")
                break
    finally:
        try:
            motor.stop()
            motor.disable_mit_mode()
        finally:
            motor.close()
            writer.close()

    report(spec, results)
    print(f"CSV saved to: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
