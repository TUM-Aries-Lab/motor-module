"""Motion capture validation -- pure velocity feedforward sine sweep.

Motor: CubeMars AK60-6 v3, CAN ID 0x03, CAN bus (can0, 1 Mbps).

Control strategy -- no Jetson-side PID
---------------------------------------
There is NO outer position loop running on the Jetson.  The Jetson only
computes and streams velocity setpoints to the motor.  The motor's own
firmware velocity PID tracks those setpoints.

Pendulum sine (primary motion):
    phys(t) = SINE_AMP * sin(2*pi*t / SINE_PERIOD)
    Starts at 0 deg (hang), swings to +40 deg, back through 0, then -40 deg.
    Pure feedforward velocity:
      v(t) = SINE_AMP * (2*pi/SINE_PERIOD) * cos(2*pi*t/SINE_PERIOD)    [deg/s]
      vel_cmd_erpm = v(t) * ERPM_PER_PHYS_DEG_S * _fw_dir
    Feedback is read every tick for LOGGING and SAFETY only -- NOT for control.

Waypoint moves (swing-up and return):
    Distance-based velocity scheduling (no PID):
      dist = abs(target_deg - actual_deg)
      speed = CRUISE_VEL_ERPM * clamp(dist / DECEL_ZONE_DEG, 0, 1)
      vel_cmd = speed * sign(error) * _fw_dir
    Velocity scales linearly to zero as the arm approaches the target.
    No integral, no derivative term.

Calibration: ERPM_PER_PHYS_DEG_S
    Relates motor ERPM to physical arm angular velocity (deg/s).
    Initial value 18.0 estimated from 2026-03-10 run:
      ~5000 ERPM -> ~280 physical deg/s -> 5000/280 ~ 18 ERPM per deg/s.
    To re-calibrate: command a constant velocity, divide commanded ERPM by
    the resulting physical deg/s visible in the CSV.

Watchdog keepalive
------------------
Velocity commands (arb_id=0x0303) do NOT feed the 100 ms CAN watchdog.
A duty=0 probe frame on arb_id=0x03 is sent every KEEPALIVE_TICKS iterations
(~16 Hz) to keep the motor enabled.  This also triggers one 0x2903 feedback
reply, which is the only way to get position/speed telemetry while in velocity
loop mode on this firmware.

Physical angle convention (cable-drive joint):
  phys = 0 deg  -> arm hanging straight down (gravity rest)
  phys > 0 deg  -> arm swings forward / up

Motion sequence
---------------
  1.  Pendulum sine  90 s pure feedforward oscillation +60 <-> 0 <-> -60 deg
  2.  -> 0 deg       velocity-scheduled return to gravity hang

Run:
    sudo ./setup_can.sh
    .venv/bin/python scripts/motion_capture_vel.py
"""

import csv
import math
import struct
import time
from datetime import datetime
from pathlib import Path

import can


class BusDownError(RuntimeError):
    """Raised when the CAN socket reports the interface has gone down."""


# ---------------------------------------------------------------------------
# Hardware
# ---------------------------------------------------------------------------
MOTOR_ID = 0x03
INTERFACE = "can0"
DUTY_ARB      = MOTOR_ID                # 0x03   -- duty commands: feeds watchdog + triggers 0x2903 reply
VELOCITY_ARB  = (0x03 << 8) | MOTOR_ID # 0x0303 -- true velocity loop (UNTESTED: no feedback on this firmware)
# NOTE: 0x0303 velocity-loop does not trigger 0x2903 feedback on this firmware AND
# appears to produce no motion on this unit.  All control is therefore sent as
# proportional duty on DUTY_ARB: duty = erpm / MAX_VEL_ERPM * MAX_DUTY.
# Each duty frame simultaneously feeds the 100ms watchdog AND triggers one 0x2903 reply.

# ---------------------------------------------------------------------------
# CAN payloads
# ---------------------------------------------------------------------------
ENABLE_CMD  = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
DISABLE_CMD = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
STOP_CMD    = struct.pack(">i", 0) + bytes(4)   # duty = 0% (used during home / coast)

FEEDBACK_IDS = {
    MOTOR_ID,
    MOTOR_ID + 1,           # 0x04  -- firmware variant
    0x2900 | MOTOR_ID,      # 0x2903 -- primary status frame
    0x2900,                  # enable-response
    0x0080 | MOTOR_ID,      # 0x0083
}

# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------
ERPM_PER_PHYS_DEG_S = 18.0
# Converts physical arm angular velocity (deg/s) to motor ERPM.
# Estimated from 2026-03-10 run data.
# To re-calibrate: command a known ERPM, divide by physical deg/s in CSV.

# ---------------------------------------------------------------------------
# Velocity scheduling -- waypoint moves (no PID)
# ---------------------------------------------------------------------------
CRUISE_VEL_ERPM = 1500    # max ERPM used during move_to waypoints
DECEL_ZONE_DEG  = 20.0    # linear slowdown starts this many degrees from target
DEADBAND_DEG    = 2.0     # zero velocity command within this error band

# ---------------------------------------------------------------------------
# Sine trajectory
# ---------------------------------------------------------------------------
SINE_CENTER   = 0.0     # degrees -- pendulum rest (hanging down)
SINE_AMP      = 40.0    # degrees -- amplitude  (+40 deg <-> 0 <-> -40 deg)
SINE_PERIOD   = 4.0     # seconds -- one full cycle (faster swing)
SINE_DURATION = 180.0   # seconds -- total sweep (~45 cycles, 3 minutes)

# Peak feedforward velocity (for reference):
#   SINE_AMP * (2*pi/SINE_PERIOD) * ERPM_PER_PHYS_DEG_S ~ 1696 ERPM

MAX_VEL_ERPM = 3000     # ceiling used for ERPM->duty scaling
MAX_DUTY     = 0.25     # absolute max duty sent to motor (25%)
# duty = erpm / MAX_VEL_ERPM * MAX_DUTY

# ---------------------------------------------------------------------------
# Safety limits
# ---------------------------------------------------------------------------
ROT_GUARD_NEG = 100.0   # 60 deg margin beyond -40 deg swing (arm overshoots ~30-45 deg at peak)
ROT_GUARD_POS = 175.0
PHYS_MIN      = -100.0  # 60 deg margin beyond -40 deg swing
PHYS_MAX      = 190.0
MAX_SAFE_ERPM = 5000    # emergency brake threshold

SAFETY_WARN_MARGIN = 20.0  # print a WARN line when arm is within this many deg of any limit

# ---------------------------------------------------------------------------
# Loop / logging
# ---------------------------------------------------------------------------
LOOP_HZ         = 50
SAMPLE_HZ       = 20
KEEPALIVE_TICKS = 3     # send ENABLE_CMD every N ticks (~16 Hz)

WAYPOINT_SETTLE_DEG  = 5.0
WAYPOINT_SETTLE_ERPM = 300
WAYPOINT_SETTLE_TIME = 0.5
WAYPOINT_TIMEOUT     = 20.0

LOG_DIR = Path(__file__).parent.parent / "data" / "logs"

# ---------------------------------------------------------------------------
# Direction state  (set once by detect_fw_dir at startup)
# ---------------------------------------------------------------------------
_fw_dir: int = +1   # +1: fw_pos increases as arm swings forward
                    # -1: fw_pos decreases as arm swings forward (inverted cable)


def phys_to_fw(phys: float, fw_home: float) -> float:
    return fw_home + phys * _fw_dir


def fw_to_phys(fw: float, fw_home: float) -> float:
    return (fw - fw_home) * _fw_dir


# ---------------------------------------------------------------------------
# CAN helpers
# ---------------------------------------------------------------------------

def tx(bus: can.BusABC, data: bytes, arb_id: int = DUTY_ARB) -> None:
    """Send a CAN frame; silently ignore TX-full / OS errors."""
    try:
        bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=True))
    except (can.CanOperationError, OSError):
        pass


def tx_velocity(bus: can.BusABC, erpm: int) -> None:
    """Send a velocity-proportional duty command on DUTY_ARB (0x03).

    0x0303 (true velocity loop) produces no motion on this firmware and
    does not generate 0x2903 feedback.  Duty commands on 0x03 do both:
    each frame feeds the 100ms watchdog AND triggers one 0x2903 status reply.

    Conversion: duty = clamp(erpm / MAX_VEL_ERPM * MAX_DUTY, +/-MAX_DUTY)
    """
    duty = int(erpm) * MAX_DUTY / MAX_VEL_ERPM
    duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
    tx(bus, struct.pack(">i", int(duty * 100_000)) + bytes(4), arb_id=DUTY_ARB)


def tx_keepalive(bus: can.BusABC) -> None:
    """No-op: every duty frame on DUTY_ARB already feeds the 100ms watchdog."""
    pass


def read_feedback(bus: can.BusABC, deadline_ms: float = 15.0):
    """Return the first valid feedback tuple within deadline_ms, or None.

    Returns (pos_deg, erpm, cur_A, temp_C, err_code) or None.
    Raises BusDownError if the CAN interface goes down.
    """
    deadline = time.monotonic() + deadline_ms / 1000.0
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        try:
            msg = bus.recv(timeout=remaining)
        except (can.CanOperationError, OSError) as exc:
            raise BusDownError(str(exc)) from exc
        if msg is None:
            return None
        if msg.arbitration_id not in FEEDBACK_IDS:
            continue
        d = msg.data
        if len(d) < 8:
            continue
        pos_deg  = struct.unpack(">h", d[0:2])[0] * 0.1
        erpm_val = struct.unpack(">h", d[2:4])[0] * 10
        cur_a    = struct.unpack(">h", d[4:6])[0] * 0.01
        temp_raw = struct.unpack("b", bytes([d[6]]))[0]
        temp_c   = temp_raw if -20 <= temp_raw <= 127 else 0
        err      = d[7]
        return pos_deg, erpm_val, cur_a, temp_c, err


# ---------------------------------------------------------------------------
# Velocity commands
# ---------------------------------------------------------------------------

def compute_waypoint_vel(error_phys: float) -> int:
    """Velocity command for waypoint moves -- distance scheduling, no PID.

    error_phys: target_deg - actual_deg

    Velocity scales linearly from CRUISE_VEL_ERPM down to zero as the arm
    closes in on the target.  No integral or derivative terms.

    Returns integer ERPM in firmware (fw) direction.
    """
    dist = abs(error_phys)
    if dist < DEADBAND_DEG:
        return 0
    direction = math.copysign(1.0, error_phys) * _fw_dir
    speed = CRUISE_VEL_ERPM * min(1.0, dist / DECEL_ZONE_DEG)
    speed = max(speed, 150)   # floor: avoid commanding < 150 ERPM (motor may stall)
    return int(direction * speed)


def sine_vel_ff(t: float) -> int:
    """Pure velocity feedforward for the pendulum sine trajectory.

    Computes the exact time-derivative of the commanded sine position:
      phys(t)  = SINE_AMP * sin(2*pi*t / SINE_PERIOD)
      v_ff(t)  = SINE_AMP * (2*pi/SINE_PERIOD) * cos(2*pi*t/SINE_PERIOD)   [deg/s]

    Starts at 0 (hang), swings to +SINE_AMP, back through 0, then -SINE_AMP.
    Converted to fw-space ERPM and clamped to MAX_VEL_ERPM.
    """
    omega        = 2.0 * math.pi / SINE_PERIOD
    vel_phys_dps = SINE_AMP * omega * math.cos(omega * t)
    vel_erpm_fw  = vel_phys_dps * ERPM_PER_PHYS_DEG_S * _fw_dir
    return int(max(-MAX_VEL_ERPM, min(MAX_VEL_ERPM, vel_erpm_fw)))


# ---------------------------------------------------------------------------
# Startup helpers
# ---------------------------------------------------------------------------

def find_home(bus: can.BusABC) -> float:
    """Coast arm to gravity hang; return fw_home (fw_pos at physical 0 deg)."""
    STILL_ERPM = 200
    STILL_TIME = 1.5
    TIMEOUT    = 15.0

    print("  Waiting for arm to settle at gravity home...", end="", flush=True)
    t0       = time.monotonic()
    still_at = None
    fw_home  = None
    last_fw  = None

    while time.monotonic() - t0 < TIMEOUT:
        tx(bus, STOP_CMD)
        fb = read_feedback(bus)
        if fb is None:
            time.sleep(0.01)
            continue
        fw_pos, spd, *_ = fb
        last_fw = fw_pos
        now = time.monotonic()
        if abs(spd) < STILL_ERPM:
            if still_at is None:
                still_at = now
            elif now - still_at >= STILL_TIME:
                fw_home = fw_pos
                break
        else:
            still_at = None
        time.sleep(0.01)

    fw_home = fw_home if fw_home is not None else (last_fw or 0.0)
    print(f" settled  fw_home={fw_home:.1f} deg")
    return fw_home


def detect_fw_dir(bus: can.BusABC, fw_home: float) -> int:
    """Determine cable-drive direction with a brief duty-cycle nudge.

    Uses a 2% duty pulse on arb_id=0x03.  Direction is a physical property
    of the cable routing that must be known before issuing velocity commands.
    """
    global _fw_dir

    TEST_DUTY = 0.02
    TEST_SEC  = 0.5
    DEADZONE  = 0.5

    print("  Detecting fw direction (2% duty nudge)...", end="", flush=True)

    while bus.recv(timeout=0.01):
        pass

    fw_start = None
    fw_end   = fw_home
    t0 = time.monotonic()
    while time.monotonic() - t0 < TEST_SEC:
        tx(bus, struct.pack(">i", int(TEST_DUTY * 100_000)) + bytes(4), arb_id=DUTY_ARB)
        fb = read_feedback(bus)
        if fb:
            if fw_start is None:
                fw_start = fb[0]
            fw_end = fb[0]
        time.sleep(0.01)

    if fw_start is None:
        fw_start = fw_home

    for _ in range(40):
        tx(bus, STOP_CMD)
        time.sleep(0.015)

    delta = fw_end - fw_start
    if abs(delta) < DEADZONE:
        print(f" no movement ({delta:+.1f} deg) -- keeping _fw_dir={_fw_dir:+d}")
    elif delta > 0:
        _fw_dir = +1
        print(f" fw moved {delta:+.1f} deg -> _fw_dir=+1")
    else:
        _fw_dir = -1
        print(f" fw moved {delta:+.1f} deg -> _fw_dir=-1  [INVERTED]")

    return _fw_dir


# ---------------------------------------------------------------------------
# Safety
# ---------------------------------------------------------------------------

def check_safety(fw_pos: float, fw_home: float) -> str | None:
    """Return fault description if any safety limit is exceeded, else None."""
    fw_delta = fw_pos - fw_home
    phys     = fw_to_phys(fw_pos, fw_home)
    if fw_delta < -ROT_GUARD_NEG:
        print(f"\n  [SAFETY] ROT_GUARD_NEG hit: fw_delta={fw_delta:+.1f}  phys={phys:+.1f}  limit=-{ROT_GUARD_NEG}", flush=True)
        return f"ROT GUARD NEG: fw_delta={fw_delta:+.1f}  phys={phys:+.1f}"
    if fw_delta > ROT_GUARD_POS:
        print(f"\n  [SAFETY] ROT_GUARD_POS hit: fw_delta={fw_delta:+.1f}  phys={phys:+.1f}  limit=+{ROT_GUARD_POS}", flush=True)
        return f"ROT GUARD POS: fw_delta={fw_delta:+.1f}  phys={phys:+.1f}"
    if phys < PHYS_MIN:
        print(f"\n  [SAFETY] PHYS_MIN hit: phys={phys:+.1f}  fw_delta={fw_delta:+.1f}  limit={PHYS_MIN}", flush=True)
        return f"PHYS MIN: phys={phys:.1f}"
    if phys > PHYS_MAX:
        print(f"\n  [SAFETY] PHYS_MAX hit: phys={phys:+.1f}  fw_delta={fw_delta:+.1f}  limit={PHYS_MAX}", flush=True)
        return f"PHYS MAX: phys={phys:.1f}"
    return None


_warn_last_t: dict[str, float] = {}   # rate-limit: key → last print time
_WARN_INTERVAL = 1.0                  # seconds between repeated warnings for same limit


def _check_safety_warn(fw_delta: float, phys: float) -> None:
    """Print a non-fatal warning when within SAFETY_WARN_MARGIN of any limit.
    Rate-limited to _WARN_INTERVAL seconds per limit to avoid log floods."""
    now = time.monotonic()

    def _emit(key: str, msg: str) -> None:
        if now - _warn_last_t.get(key, -999.0) >= _WARN_INTERVAL:
            print(f"\n{msg}", flush=True)
            _warn_last_t[key] = now

    if fw_delta < -(ROT_GUARD_NEG - SAFETY_WARN_MARGIN):
        _emit(
            "neg",
            f"  [WARN] fw_delta={fw_delta:+.1f} approaching ROT_GUARD_NEG (-{ROT_GUARD_NEG})"
            f"  margin={fw_delta + ROT_GUARD_NEG:+.1f} deg",
        )
    if fw_delta > (ROT_GUARD_POS - SAFETY_WARN_MARGIN):
        _emit(
            "pos",
            f"  [WARN] fw_delta={fw_delta:+.1f} approaching ROT_GUARD_POS (+{ROT_GUARD_POS})"
            f"  margin={ROT_GUARD_POS - fw_delta:+.1f} deg",
        )
    if phys < PHYS_MIN + SAFETY_WARN_MARGIN:
        _emit(
            "min",
            f"  [WARN] phys={phys:+.1f} approaching PHYS_MIN ({PHYS_MIN})"
            f"  margin={phys - PHYS_MIN:+.1f} deg",
        )
    if phys > PHYS_MAX - SAFETY_WARN_MARGIN:
        _emit(
            "max",
            f"  [WARN] phys={phys:+.1f} approaching PHYS_MAX ({PHYS_MAX})"
            f"  margin={PHYS_MAX - phys:+.1f} deg",
        )


# ---------------------------------------------------------------------------
# Motion primitives
# ---------------------------------------------------------------------------

def move_to(
    bus: can.BusABC,
    phys_tgt: float,
    fw_home: float,
    label: str,
    writer: csv.DictWriter,
    t0: float,
) -> bool:
    """Drive arm to phys_tgt using distance-based velocity scheduling (no PID).

    Returns True on success or timeout, False on safety abort.
    """
    loop_dt    = 1.0 / LOOP_HZ
    csv_dt     = 1.0 / SAMPLE_HZ
    last_csv   = -1.0
    settled_at = None
    deadline   = time.monotonic() + WAYPOINT_TIMEOUT
    no_fb      = 0
    tick_count = 0

    print(f"\n  >> MOVE  target={phys_tgt:+.1f} deg  [{label}]  (velocity scheduling)")

    last_err = phys_tgt     # initial guess: arm at home, full error
    while True:
        tick = time.monotonic()
        tick_count += 1

        try:
            fb = read_feedback(bus, deadline_ms=22.0)
        except BusDownError as exc:
            print(f"\n  X CAN bus down during move_to: {exc}")
            return False

        if fb is None:
            if tick_count % KEEPALIVE_TICKS == 0:
                tx_keepalive(bus)
            tx_velocity(bus, compute_waypoint_vel(last_err))
            no_fb += 1
            if no_fb % 50 == 0:
                print(f"  ! no feedback ({no_fb} ticks)", end="\r")
            time.sleep(max(loop_dt - (time.monotonic() - tick), 0.005))
            continue
        no_fb = 0

        fw_pos, spd, cur, tmp, err_code = fb
        now     = time.monotonic()
        phys    = fw_to_phys(fw_pos, fw_home)
        err     = phys_tgt - phys
        last_err = err
        elapsed = now - t0

        fault = check_safety(fw_pos, fw_home)
        if fault:
            tx_velocity(bus, 0)
            print(f"\n  !! {fault} -- aborting")
            return False

        vel_cmd = compute_waypoint_vel(err)

        if tick_count % KEEPALIVE_TICKS == 0:
            tx_keepalive(bus)
        tx_velocity(bus, vel_cmd)

        if elapsed - last_csv >= csv_dt:
            last_csv = elapsed
            writer.writerow({
                "time_s":        f"{elapsed:.3f}",
                "label":         label,
                "commanded_deg": f"{phys_tgt:.1f}",
                "actual_deg":    f"{phys:.2f}",
                "error_deg":     f"{err:.2f}",
                "vel_cmd_erpm":  vel_cmd,
                "speed_erpm":    spd,
                "current_amps":  f"{cur:.3f}",
                "temp_c":        tmp,
                "error_code":    err_code,
            })

        print(
            f"    t={elapsed:6.2f}s  phys={phys:+7.2f} deg  err={err:+6.1f} deg  "
            f"vel_cmd={vel_cmd:+5d}  erpm={spd:+6d}  cur={cur:+5.2f}A",
            end="\r",
        )

        if abs(err) < WAYPOINT_SETTLE_DEG and abs(spd) < WAYPOINT_SETTLE_ERPM:
            if settled_at is None:
                settled_at = now
        else:
            settled_at = None

        if settled_at is not None and (now - settled_at) >= WAYPOINT_SETTLE_TIME:
            print(f"\n  OK Settled at phys={phys:+.2f} deg (err={err:+.2f} deg)")
            return True

        if now >= deadline:
            print(f"\n  TIMEOUT phys={phys:+.2f} deg  err={err:+.2f} deg")
            return True

        time.sleep(max(loop_dt - (time.monotonic() - tick), 0.001))


def sine_sweep_ff(
    bus: can.BusABC,
    fw_home: float,
    writer: csv.DictWriter,
    t0: float,
) -> bool:
    """Sine sweep using pure velocity feedforward -- NO position error, NO PID.

    Every tick:
      1. Compute v_ff(t) = exact derivative of sine trajectory -> ERPM.
      2. Convert to duty and send on arb_id=0x03 (duty proportional to v_ff).
      3. Read feedback for LOGGING and SAFETY only.

    Feedback does NOT influence the velocity command in any way.
    The motor firmware velocity PID tracks v_ff(t) directly.

    Returns True on completion, False on safety abort.
    """
    loop_dt    = 1.0 / LOOP_HZ
    csv_dt     = 1.0 / SAMPLE_HZ
    last_csv   = -1.0
    no_fb      = 0
    label      = "sine-sweep"
    tick_count = 0

    # diagnostics state
    _last_vel_sign  = 0      # track sign of vel_cmd to detect reversals
    _last_full_print = -1.0  # last time we printed a full (non-\r) status line
    _peak_logged    = False  # suppress repeated peak prints within same half-cycle

    t_start   = time.monotonic()
    t_end     = t_start + SINE_DURATION
    peak_vel  = int(SINE_AMP * (2.0 * math.pi / SINE_PERIOD) * ERPM_PER_PHYS_DEG_S)

    print(
        f"\n  >> PENDULUM  +/-{SINE_AMP} deg around 0 deg  "
        f"period={SINE_PERIOD}s  duration={SINE_DURATION}s"
    )
    print(f"     Mode: pure feedforward -- NO position loop")
    print(f"     Peak v_ff = {peak_vel} ERPM  (ERPM_PER_PHYS_DEG_S={ERPM_PER_PHYS_DEG_S})")

    while True:
        tick = time.monotonic()
        tick_count += 1
        t = tick - t_start

        if tick >= t_end:
            print(f"\n  OK Sine sweep complete ({SINE_DURATION:.0f} s)")
            break

        # ---- Control: pure feedforward, feedback NOT used here ----
        vel_cmd     = sine_vel_ff(t)
        phys_tgt_ff = SINE_CENTER + SINE_AMP * math.sin(2.0 * math.pi * t / SINE_PERIOD)

        # --- direction-reversal diagnostic ---
        cur_sign = 1 if vel_cmd > 0 else (-1 if vel_cmd < 0 else 0)
        if cur_sign != 0 and _last_vel_sign != 0 and cur_sign != _last_vel_sign:
            phase = "+swing" if cur_sign > 0 else "-swing"
            print(
                f"\n  [REVERSAL t={t:.2f}s] now heading {phase}  "
                f"v_ff={vel_cmd:+d} ERPM",
                flush=True,
            )
            _peak_logged = False
        _last_vel_sign = cur_sign
        # -------------------------------------

        if tick_count % KEEPALIVE_TICKS == 0:
            tx_keepalive(bus)
        tx_velocity(bus, vel_cmd)
        # -----------------------------------------------------------

        try:
            fb = read_feedback(bus, deadline_ms=22.0)
        except BusDownError as exc:
            print(f"\n  X CAN bus down during sine_sweep: {exc}")
            return False

        if fb is None:
            no_fb += 1
            time.sleep(max(loop_dt - (time.monotonic() - tick), 0.005))
            continue
        no_fb = 0

        fw_pos, spd, cur, tmp, err_code = fb
        now          = time.monotonic()
        phys         = fw_to_phys(fw_pos, fw_home)
        fw_delta     = fw_pos - fw_home
        elapsed      = now - t0
        tracking_err = phys_tgt_ff - phys   # for logging only

        # --- safety proximity warning ---
        _check_safety_warn(fw_delta, phys)

        # --- peak excursion diagnostic ---
        if abs(phys) >= SINE_AMP * 0.85 and not _peak_logged:
            direction = "+" if phys > 0 else "-"
            print(
                f"\n  [PEAK  t={t:.2f}s] phys={phys:+.1f}°  cmd={phys_tgt_ff:+.1f}°  "
                f"overshoot={phys - phys_tgt_ff:+.1f}°  fw_delta={fw_delta:+.1f}°  "
                f"erpm={spd:+d}",
                flush=True,
            )
            _peak_logged = True
        elif abs(phys) < SINE_AMP * 0.5:
            _peak_logged = False

        fault = check_safety(fw_pos, fw_home)
        if fault:
            tx_velocity(bus, 0)
            print(f"\n  !! {fault} -- aborting sine")
            return False

        # Emergency brake: vel=0 is an active brake command in velocity mode.
        if abs(spd) > MAX_SAFE_ERPM:
            print(
                f"\n  BRAKE: {spd:+d} ERPM > {MAX_SAFE_ERPM} -- "
                "commanding vel=0...", end="", flush=True,
            )
            while abs(spd) > MAX_SAFE_ERPM // 2:
                tick_count += 1
                if tick_count % KEEPALIVE_TICKS == 0:
                    tx_keepalive(bus)
                tx_velocity(bus, 0)
                try:
                    fb2 = read_feedback(bus)
                except BusDownError as exc:
                    print(f"\n  X CAN bus down during brake: {exc}")
                    return False
                if fb2:
                    spd  = fb2[1]
                    phys = fw_to_phys(fb2[0], fw_home)
                    now2     = time.monotonic()
                    elapsed2 = now2 - t0
                    if elapsed2 - last_csv >= csv_dt:
                        last_csv = elapsed2
                        writer.writerow({
                            "time_s":        f"{elapsed2:.3f}",
                            "label":         "brake",
                            "commanded_deg": f"{phys_tgt_ff:.2f}",
                            "actual_deg":    f"{phys:.2f}",
                            "error_deg":     f"{phys_tgt_ff - phys:.2f}",
                            "vel_cmd_erpm":  0,
                            "speed_erpm":    spd,
                            "current_amps":  f"{fb2[2]:.3f}",
                            "temp_c":        fb2[3],
                            "error_code":    fb2[4],
                        })
                    brake_fault = check_safety(fb2[0], fw_home)
                    if brake_fault:
                        tx_velocity(bus, 0)
                        print(f"\n  !! {brake_fault} during brake -- aborting")
                        return False
                time.sleep(0.01)
            print(f" resumed at {spd:+d} ERPM")

        if elapsed - last_csv >= csv_dt:
            last_csv = elapsed
            writer.writerow({
                "time_s":        f"{elapsed:.3f}",
                "label":         label,
                "commanded_deg": f"{phys_tgt_ff:.2f}",
                "actual_deg":    f"{phys:.2f}",
                "error_deg":     f"{tracking_err:.2f}",
                "vel_cmd_erpm":  vel_cmd,
                "speed_erpm":    spd,
                "current_amps":  f"{cur:.3f}",
                "temp_c":        tmp,
                "error_code":    err_code,
            })

        # periodic full-line print (every 1 second) so output history is visible
        if elapsed - _last_full_print >= 1.0:
            _last_full_print = elapsed
            cycle   = int(t / SINE_PERIOD) + 1
            half    = "+half" if math.sin(2 * math.pi * t / SINE_PERIOD) >= 0 else "-half"
            print(
                f"  [t={t:6.1f}s | cyc={cycle} {half}]  "
                f"cmd={phys_tgt_ff:+6.1f}°  phys={phys:+6.1f}°  "
                f"lag={tracking_err:+5.1f}°  fw_delta={fw_delta:+6.1f}°  "
                f"v_ff={vel_cmd:+5d}  erpm={spd:+6d}  cur={cur:+.2f}A  no_fb={no_fb}",
                flush=True,
            )

        print(
            f"    t={t:5.2f}/{SINE_DURATION:.0f}s  "
            f"cmd={phys_tgt_ff:+7.2f}°  phys={phys:+7.2f}°  "
            f"fw_delta={fw_delta:+7.1f}°  lag={tracking_err:+5.1f}°  "
            f"v_ff={vel_cmd:+5d}  erpm={spd:+6d}",
            end="\r",
        )

        time.sleep(max(loop_dt - (time.monotonic() - tick), 0.001))

    return True


def return_to_home(
    bus: can.BusABC,
    fw_home: float,
    writer: csv.DictWriter,
    t0: float,
) -> None:
    """Return arm to gravity hang (phys=0) using velocity scheduling."""
    print("\n  >> RETURN  lowering arm to 0 deg (velocity scheduling)...")
    move_to(bus, 0.0, fw_home, "hang-return", writer, t0)


# ---------------------------------------------------------------------------
# Post-run summary
# ---------------------------------------------------------------------------

def print_summary(csv_path: Path) -> None:
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print("  No data recorded.")
        return

    from collections import OrderedDict
    by_label: dict = OrderedDict()
    for r in rows:
        by_label.setdefault(r["label"], []).append(r)

    print("\n" + "=" * 76)
    print("  POST-RUN SUMMARY  (pure velocity feedforward)")
    print("=" * 76)
    hdr = "  {:<18s}  {:>5s}  {:>9s}  {:>8s}  {:>9s}  {:>10s}".format(
        "Label", "N", "MeanLag", "MaxLag", "PeakERPM", "PeakVelCmd"
    )
    print(hdr)
    print("  " + "-" * 68)
    for lbl, lrows in by_label.items():
        errs  = [abs(float(r["error_deg"])) for r in lrows]
        erpms = [abs(int(r["speed_erpm"])) for r in lrows]
        vels  = [abs(int(r["vel_cmd_erpm"])) for r in lrows]
        n = len(lrows)
        print(
            "  {:<18s}  {:>5d}  {:>7.2f} deg  {:>6.2f} deg  {:>9d}  {:>10d}".format(
                lbl, n, sum(errs) / n, max(errs), max(erpms), max(vels)
            )
        )
    print("=" * 76)
    dur = float(rows[-1]["time_s"]) - float(rows[0]["time_s"])
    print(f"  Duration: {dur:.1f} s  |  Samples: {len(rows)}")
    print(f"  CSV: {csv_path}")
    print()
    print("  Calibration / tuning hints:")
    print(f"    ERPM_PER_PHYS_DEG_S = {ERPM_PER_PHYS_DEG_S}")
    print("      'lag' column shows how much the arm trails the feedforward command.")
    print("      Arm consistently lags  -> increase ERPM_PER_PHYS_DEG_S")
    print("      Arm leads / overshoots -> decrease ERPM_PER_PHYS_DEG_S")
    print(f"    CRUISE_VEL_ERPM = {CRUISE_VEL_ERPM}  -- raise if waypoint moves are too slow")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts       = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_path = LOG_DIR / f"mocap_vel_ff_{ts}.csv"

    fieldnames = [
        "time_s", "label", "commanded_deg", "actual_deg", "error_deg",
        "vel_cmd_erpm", "speed_erpm", "current_amps", "temp_c", "error_code",
    ]

    peak_vel = int(SINE_AMP * (2.0 * math.pi / SINE_PERIOD) * ERPM_PER_PHYS_DEG_S)

    print("=" * 70)
    print("  Motion Capture -- Pure Velocity Feedforward Sine Sweep")
    print("  (No Jetson-side PID -- motor firmware velocity PID only)")
    print(f"  Motor ID: 0x{MOTOR_ID:02X}   Interface: {INTERFACE}")
    print(
        f"  Control: duty on 0x{DUTY_ARB:02X} proportional to ERPM  "
        f"(0x{VELOCITY_ARB:03X} velocity loop not functional on this firmware)   "
        f"Keepalive: built-in (each duty frame)"
    )
    print(
        f"  ERPM_PER_PHYS_DEG_S={ERPM_PER_PHYS_DEG_S}   "
        f"Peak v_ff={peak_vel} ERPM   MAX_VEL={MAX_VEL_ERPM} ERPM"
    )
    print(
        f"  Pendulum: +/-{SINE_AMP} deg around 0 deg  "
        f"period={SINE_PERIOD}s  duration={SINE_DURATION}s"
    )
    print(f"  Loop: {LOOP_HZ} Hz   CSV: {SAMPLE_HZ} Hz")
    print(f"  Log: {csv_path}")
    print("=" * 70)

    print("  Pre-flight: checking for CAN bus traffic...")
    _raw_bus   = can.interface.Bus(channel=INTERFACE, interface="socketcan")
    _t0_sniff  = time.monotonic()
    _got_frame = False
    while time.monotonic() - _t0_sniff < 1.5:
        try:
            _m = _raw_bus.recv(timeout=0.1)
        except (can.CanOperationError, OSError):
            break
        if _m is not None:
            _got_frame = True
            break
    _raw_bus.shutdown()
    if not _got_frame:
        print(
            "  X No CAN frames in 1.5 s -- motor appears to be OFF.\n"
            "  Power on, then re-run:  .venv/bin/python scripts/motion_capture_vel.py"
        )
        return
    print("  OK Bus is live -- opening filtered socket")
    print(
        f"  Control: duty ∝ ERPM on arb_id=0x{DUTY_ARB:02X}  "
        f"(0x{VELOCITY_ARB:03X} velocity loop not functional on this firmware)\n"
        f"  duty = erpm / {MAX_VEL_ERPM} * {MAX_DUTY}  "
        f"(peak sine ~{int(SINE_AMP*(2*3.14159/SINE_PERIOD)*ERPM_PER_PHYS_DEG_S/MAX_VEL_ERPM*MAX_DUTY*100)}%  "
        f"cruise ~{int(1500/MAX_VEL_ERPM*MAX_DUTY*100)}%)"
    )

    _can_filters = [
        {"can_id": fid, "can_mask": 0x1FFFFFFF, "extended": True}
        for fid in FEEDBACK_IDS
    ]
    bus = can.interface.Bus(
        channel=INTERFACE,
        interface="socketcan",
        can_filters=_can_filters,
    )

    try:
        with open(csv_path, "w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

            print("\n  Enabling motor (servo mode)...")
            tx(bus, ENABLE_CMD)
            time.sleep(0.3)

            fw_alive = None
            for attempt in range(30):
                try:
                    fb = read_feedback(bus)
                except BusDownError as exc:
                    print(f"\n  X CAN bus went down during enable: {exc}")
                    print("  Run: sudo ./setup_can.sh  then try again.")
                    return
                if fb is not None:
                    fw_alive = fb[0]
                    print(
                        f"  Motor alive  fw={fw_alive:.1f} deg  temp={fb[3]} C  "
                        f"cur={fb[2]:.2f}A  err={fb[4]}  (attempt {attempt + 1})"
                    )
                    break
                tx(bus, ENABLE_CMD)
                time.sleep(0.1)

            if fw_alive is None:
                print("  X No motor feedback -- check power & wiring")
                return

            fw_home = find_home(bus)
            print(f"  Physical 0 deg = fw_pos {fw_home:.1f} deg")
            print(
                f"  Safety: fw_delta [{-ROT_GUARD_NEG:+.0f}, +{ROT_GUARD_POS:.0f}] deg  "
                f"phys [{PHYS_MIN:.0f}, {PHYS_MAX:.0f}] deg"
            )

            detect_fw_dir(bus, fw_home)
            sign = "+" if _fw_dir > 0 else "-"
            print(f"  _fw_dir={_fw_dir:+d}   vel_fw = vel_phys x ({sign}1)")

            fw_home = find_home(bus)
            print(f"  Re-homed: fw_home={fw_home:.1f} deg")
            print("\n  Press Ctrl+C to abort at any time\n")

            t0 = time.monotonic()

            try:
                # Arm is already at 0 deg (gravity hang); sine starts at 0 so
                # no initial move needed -- launch pendulum swing directly.
                ok = sine_sweep_ff(bus, fw_home, writer, t0)
                if not ok:
                    raise RuntimeError("sine sweep aborted by safety")

                return_to_home(bus, fw_home, writer, t0)

            except KeyboardInterrupt:
                print("\n\n  Ctrl+C -- stopping safely...")
            except (RuntimeError, BusDownError) as e:
                print(f"\n  Sequence aborted: {e}")

            print("\n  Sending vel=0 + disable...")
            for _ in range(15):
                tx_velocity(bus, 0)
                time.sleep(0.02)
            tx(bus, DISABLE_CMD)
            time.sleep(0.2)

    finally:
        bus.shutdown()

    print_summary(csv_path)
    print(f"\n  Done.  Data -> {csv_path}")
    print("=" * 70)


if __name__ == "__main__":
    main()
