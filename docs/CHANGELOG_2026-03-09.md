# Session Log — 9 March 2026

Work is on the `CAN_implementation` branch.

---

## Summary

Resolved a multi-day motor debugging issue. The motor was communicating on CAN
(ACKing commands, feedback with error_code=0) but physically refused to spin.
Root cause was **PSU under-voltage (18.5V instead of 24V)** causing the MOSFET
H-bridge to silently lock out motor drive while the MCU + CAN transceiver
continued operating normally.

---

## Root Cause: PSU Under-Voltage Lockout

The AK60-6 motor controller has a split power architecture:

| Subsystem | Min Voltage | Behaviour below minimum |
|---|---|---|
| MCU + CAN transceiver | ~12V | CAN communication works normally |
| MOSFET H-bridge (motor drive) | ~20V | **Silently disabled** — no error reported |

### What happened

1. PSU was set to **18.5V** (unknown when/how it was changed).
2. Motor controller logic powered up normally — CAN bus was healthy.
3. All commands were ACKed, feedback reported `error_code=0`, temperature was valid (31–33°C).
4. H-bridge was in under-voltage lockout: `Speed=0`, `Current=0`, PSU draws only 35 mA.
5. **Multiple days of CAN protocol debugging** found nothing wrong because the protocol layer was working correctly.

### How it was diagnosed

1. CAN ID scan confirmed motor IS at ID 0x03 (responds on 0x2903).
2. PSU ammeter showed only **0.035A** — pure logic quiescent power, not motor drive.
3. User raised PSU to 24V and power-cycled the motor.
4. Motor immediately started spinning — `spin_test.py` showed ~7,200 ERPM at 40% duty.

### Why it was hard to find

- `error_code` was always 0 ("No fault").
- CAN bus was ERROR-ACTIVE (clean).
- Motor responded to every command with feedback frames.
- Software diagnostics all looked perfectly healthy.
- The **only** clue was the PSU ammeter reading.

---

## Additional Discovery: Only Duty-Cycle Mode Works

While testing all control modes with the motor running, found:

| Mode | arb_id | Result |
|---|---|---|
| Duty cycle (0x00) | 0x0003 | ✅ Motor spins, feedback shows speed/current |
| Current (0x01) | 0x0103 | ❌ Zero feedback frames after command |
| Velocity (0x03) | 0x0303 | ❌ Zero feedback frames after command |
| Position (0x04) | 0x0403 | ❌ ACKs but doesn't move (known issue) |

Only duty-cycle mode reliably drives the motor and produces feedback.
Current and velocity modes don't even generate feedback responses.
This may require CubeMars PC software configuration.

---

## Motor Performance at 24V (confirmed working)

| Duty | Speed (ERPM) | Current (A) | Position tracking |
|---|---|---|---|
| 10% | 2,170 | 0.56 | ✅ |
| 30% | 7,100 | 0.35 | ✅ |
| 50% | 12,130 | 0.44 | ✅ |
| 80% | 19,590 | 0.44 | ✅ |

Sustained 80% duty for 2 seconds showed continuous rotation at ~19,600 ERPM.
Forward and reverse directions both work.

---

## Documentation Updated

Added the under-voltage lockout issue to all relevant docs so this problem
never wastes debugging time again:

- **CAN_TROUBLESHOOTING.md** — Added Problem 5 (under-voltage lockout) and Problem 6 (current/velocity mode no feedback)
- **CAN_SETUP_GUIDE.md** — Added PSU voltage to hardware checklist with detailed explanation; added "Motor Communicates but Won't Spin" troubleshooting section
- **CAN_QUICK_REFERENCE.md** — Added Speed=0 troubleshooting row; added pre-flight checklist
- **CAN_IMPLEMENTATION_JOURNEY.md** — Added Key Discoveries #6 (under-voltage) and #7 (only duty-cycle works); updated status table

---

## Pre-Flight Checklist (added to docs)

Before debugging any CAN software issue:

1. ⚡ PSU voltage reads **24V** on the display
2. 🔌 UART / R-Link cable is **physically disconnected** from motor
3. 🖥️ Run `sudo ./setup_can.sh`
4. ✅ `ip -details link show can0` → `ERROR-ACTIVE (berr-counter tx 0 rx 0)`
5. 🔄 Run `python spin_test.py` → motor spins, speed reaches ~7000 ERPM
6. ⚡ PSU ammeter shows **>0.3A** when motor is commanded (not 35 mA)

---

## Files Modified

| File | Change |
|---|---|
| `docs/CAN_TROUBLESHOOTING.md` | Added Problems 5 & 6, updated confirmed-working table |
| `docs/CAN_SETUP_GUIDE.md` | PSU checklist, under-voltage section, new troubleshooting entry |
| `docs/CAN_QUICK_REFERENCE.md` | Speed=0 row in troubleshooting, pre-flight checklist |
| `docs/CAN_IMPLEMENTATION_JOURNEY.md` | Key Discoveries #6 & #7, updated status table |
| `docs/CHANGELOG_2026-03-09.md` | This file |
