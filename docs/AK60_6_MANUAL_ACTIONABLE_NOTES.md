# AK60-6 Manual: Actionable Notes for This Repo

This note distills the full `AK60-6 Manual.pdf` into what is practical for the
current Jetson Orin Nano + AK60-6 CAN implementation.

## 1) Hardware/Power Limits To Treat As Hard Constraints

- Driver nominal voltage: `48V`
- Allowable working voltage: `18-52V`
- CAN bitrate: `1 Mbps`
- Board max allowable temperature: `100C`
- Environment temperature: `-20C to 65C`
- Standby power is low (`<=50mA`), so low current draw alone does not prove drive is active.

Practical implication for bench work:
- Keep PSU at `24V` minimum for reliable drive behavior.
- If CAN looks alive but there is no torque/motion, verify PSU and power-cycle first.

## 2) CAN Configuration In CubeMarsTool Matters

The manual states application settings include:
- CAN mode: `periodic feedback` or `query-reply`
- CAN bus rate
- CAN feedback rate
- CAN ID

Recommended for this repo:
- CAN mode: `periodic feedback`
- CAN feedback rate: `50 Hz`
- CAN bitrate: `1 Mbps`
- Motor CAN ID: `0x03` (single motor target)

## 3) Force Control (MIT) Protocol We Should Follow

- Extended CAN ID format: `(mode_id << 8) | motor_id`
- MIT mode_id: `0x08`
- Frame type: extended data frame, DLC 8
- Payload layout:
  - `DATA[0]`: `kp[11:4]`
  - `DATA[1]`: `kp[3:0] | kd[11:8]`
  - `DATA[2]`: `kd[7:0]`
  - `DATA[3]`: `pos[15:8]`
  - `DATA[4]`: `pos[7:0]`
  - `DATA[5]`: `vel[11:4]`
  - `DATA[6]`: `vel[3:0] | tau[11:8]`
  - `DATA[7]`: `tau[7:0]`

AK60-6 limits from the manual parameter table:
- position: `-12.56 .. 12.56 rad`
- speed: `-60 .. 60 rad/s`
- torque: `-12 .. 12`
- kp: `0 .. 500`
- kd: `0 .. 5`

## 4) Feedback Decode (Must Match Exactly)

CAN upload message is 8 bytes:
- position: int16, scale `*0.1 deg`
- speed: int16, scale `*10 ERPM`
- current: int16, scale `*0.01 A`
- temperature: int8 (`-20 .. 127 C`)
- error code: uint8

## 5) Fault Codes Relevant To CAN Telemetry

For CAN upload error byte:
- `0`: no fault
- `1`: motor over-temp
- `2`: over-current
- `3`: over-voltage
- `4`: under-voltage
- `5`: encoder fault
- `6`: MOSFET over-temp
- `7`: motor lock-up

## 6) Calibration/Firmware Operations That Can Affect Behavior

Manual says recalibration is needed after:
- driver board re-installation
- motor phase wiring sequence changes
- firmware update

Calibration sequence in tool is no-load and can heat the motor if repeated.

## 7) Manual Inconsistencies To Be Aware Of

Do not blindly copy every sample byte/code line from the PDF:

- The MIT sample code block is explicitly "AK10-9 as an example", not AK60-6.
- In that sample code, some clamp lines are typographical errors (using `0` instead of the variable).
- Some numeric example frames do not perfectly match the AK60-6 table ranges.
- Manual section 4.2 clearly defines MIT command frame format, but does not explicitly define dedicated MIT enable/disable frames.

Use the manual's structure and AK60-6 range table as source of truth, then
validate behavior on hardware.

## 8) What This Means For The Current Repo

Current implementation direction is correct:
- MIT-only command path (`mode_id=0x08`)
- Extended IDs
- MIT bit-packing order
- AK60-6 limits
- Correct feedback scaling and fault map for CAN telemetry

Open caution:
- Treat the `FF..FF` / `FF..FE` helper sequence as implementation compatibility behavior rather than a clearly documented manual requirement.
