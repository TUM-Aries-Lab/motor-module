# CAN Debugging Log — AK60-6 on Jetson Orin Nano

Live notes from each debugging session.  Update this file every session.

---

## Session 1 — 2026-02-24

### Setup state
| Item | Value |
|---|---|
| Interface | `can0`, UP, `ERROR-ACTIVE`, 1 Mbps, `restart-ms 100` |
| Motor CAN ID (CubeMars software) | **3 (0x03)** (confirmed by user) |
| Motor model | AK60-6 HW V3.0 |
| UART cable | **Unknown — not confirmed disconnected** |

### What we did

1. Brought `can0` up with `berr-reporting on restart-ms 100`.
2. Ran `quick_test.py` — passive feedback listener.
3. Ran Python sniffer (5 s passive) → **0 frames received**.
4. Ran Python sniffer with TX (enable + velocity commands) → frames appeared.

### Raw frames observed (during TX)

| ID | Type | Length | Sample data | Rate | Likely source |
|---|---|---|---|---|---|
| `0x00000088` | STD | 8 | `00 00 08 00 00 00 00 00` | ~10 Hz | **Unknown — NOT motor** (see below) |
| `0x00000004` | STD | 8 | `00 20 00 00 00 00 A0 00` | ~1–2 Hz | Unknown (IMU?) |
| `0x00000040` | STD | 8 | `00 00 00 00 00 00 00 00` | ~1 Hz | Unknown heartbeat |
| `0x00000100` | STD | 8 | `00 00 00 00 00 00 00 00` | ~1 Hz | Unknown heartbeat |

### TX berr-counter
`can state ERROR-ACTIVE (berr-counter tx 0 rx 0)` — **TX frames are being ACKed**.
The motor IS on the bus and is receiving our command frames.

### Why 0x88 is NOT the motor feedback

Motor feedback is configured at **50 Hz** (20 ms period).
`0x88` arrived at ~**10 Hz** → wrong rate. Almost certainly an IMU or other sensor.

If it were the motor from CAN ID 8 (hypothesis `0x80 | 8 = 0x88`):
- Parsed speed = `0x0800` × 10 = 20 480 ERPM at idle → impossible
- Parsed temp = 0 °C → implausible for powered motor

### Why the motor is not sending feedback

Two mutually exclusive hypotheses:

**Hypothesis A — UART cable still connected (most likely)**
Per the journey doc: when UART is connected, the motor ACKs CAN frames at the physical
layer but ignores all CAN control commands.
The docs also say it "continues to broadcast 50 Hz feedback via CAN even while UART is
active" — but some firmware versions suppress feedback while UART is active.
Action: **physically disconnect the UART cable**, then retest.

**Hypothesis B — Motor needs enable command first**
Some firmware versions only begin sending feedback after receiving the enable frame
`0xFFFFFFFFFFFFFFFC` on the motor's CAN ID.
Action: send enable, then listen; done in session 1 but UART may have blocked it.

### TX commands used (all with is_extended_id=True)

| Frame | Arbitration ID | Data |
|---|---|---|
| Enable (servo mode) | `0x00000003` | `FF FF FF FF FF FF FF FC` |
| Current = 0 A | `0x00000103` | `00 00 00 00 00 00 00 00` |
| Velocity 5000 ERPM | `0x00000303` | `00 00 13 88 00 00 00 00` |

All command encodings have been verified against the CubeMars CAN protocol spec
(in `CAN_IMPLEMENTATION_JOURNEY.md`).

---

## Next steps (priority order)

1. **Disconnect UART cable** — highest priority.  If motor feedback appears after this,
   all command encodings are correct and we're done.
2. Re-run sniffer passively (`candump -ta can0`) — look for 50 Hz stream.
3. If still no feedback: verify motor is in **Periodic Feedback** mode in CubeMars
   software (not MIT mode or response-only mode).
4. If feedback appears on an unexpected ID: update `CANDefaults.feedback_can_id` to
   that value and retest parsing.
5. Check motor power LED / status indicator to confirm it is powered.

---

## Code changes made to support debugging

- `_receive_feedback` now logs the actual received CAN ID at DEBUG level so every
  ignored frame is visible.
- `CANDefaults.feedback_can_id` field added — override to force a specific feedback ID.
- `_receive_feedback` now accepts any of these dynamically:
  - `motor_id` (direct match)
  - `0x2900 | motor_id` (original extended ID hypothesis)
  - `0x80 | motor_id` (standard-frame hypothesis)
  - The value of `feedback_can_id` if explicitly set

---

---

## Session 2 — 2026-02-24 (continued)

### Setup state
| Item | Value |
|---|---|
| Interface | `can0`, UP, `ERROR-ACTIVE`, 1 Mbps, `restart-ms 100` |
| Motor CAN ID | **3 (0x03)** (configured in CubeMars software — may differ from UART ID) |
| UART cable | **Still connected (suspected)** |

### Tests performed

#### Test A — Correlate 0x88 with enable command
Procedure: measure 0x88 count *before* enable vs *after* enable.

| Phase | 0x88 count (1 s) |
|---|---|
| Baseline (no command sent) | **0** |
| After enable to 0x0003 (std) | 8+ |

Initial interpretation: **0x88 is motor responding** (conditional appearance).

#### Test B — Definitive: vary command target to distinguish motor vs background
Procedure: send enable to ID=3, ID=8, extended, standard — all combinations.
Measure 0x88 count and data in each case.

| Command target | Frame type | 0x88 count | Unique data |
|---|---|---|---|
| ID=3 std | standard | 28 | `0000080000000000` only |
| ID=8 std | standard | 23 | `0000080000000000` only |
| ID=3 ext | extended | 14 | `0000080000000000` only |
| ID=8 ext | extended | 18 | `0000080000000000` only |
| Vel 2000 ERPM to 0x0303 ext | extended | 19 | `0000080000000000` only |
| Vel 2000 ERPM to 0x0308 ext | extended | 17 | `0000080000000000` only |

**Conclusion: `0x88` is definitively background noise.**
- Data is **always** `0000080000000000` regardless of any command
- Count is roughly constant regardless of which motor ID we target
- No data change after velocity command — real motor feedback would show changing speed
- Session 1 "correlation" was a timing artifact (baseline sample happened during a quiet gap)

### Current bus contents

| ID | Source hypothesis | Note |
|---|---|---|
| `0x0004` | IMU or other sensor | data varies slightly (~accel) |
| `0x0040` | Unknown heartbeat | all-zeros |
| `0x0088` | Background device (CAN node 8?) | fixed `0000080000000000` forever |
| `0x0100` | Unknown heartbeat | all-zeros |

**Motor AK60-6 at ID=3 is confirmed on bus (TX berr=0) but sends zero feedback frames.**

### Root cause conclusion

UART cable being connected is the **only remaining explanation**.
Until physically disconnected, the motor will not transmit CAN feedback.

> Physical action required: ***disconnect the UART cable from the motor***.

### After UART disconnect — expected results

Run `python quick_test.py`. Expected if UART was the issue:
```
STD/EXT 0x00000003 or 0x00002903  [00 00 00 00 00 00 XX 00] × many (~50 Hz)
```
where `XX` is temperature (likely 20–40 °C).

---

---

## Session 3 — 2026-02-24 (RESOLVED)

### Setup state
| Item | Value |
|---|---|
| Interface | `can0`, UP, `ERROR-ACTIVE`, 1 Mbps, `restart-ms 100` |
| Motor CAN ID | **3 (0x03)** confirmed |
| **UART cable** | **DISCONNECTED** ✅ |

### What changed
User physically disconnected the UART cable from the motor.

### Results
Motor now broadcasts **50 Hz periodic feedback** on `EXT 0x00002903`:
```
quick_test.py Phase 1:
  EXT 0x00002903 [00 4B 00 00 00 00 21 00]  x151 (~50 Hz)  *** MOTOR FEEDBACK ***
     pos=7.5deg  spd=0 ERPM  cur=0.00A  tmp=33C  err=0
```

`quick_test.py` Phase 2 (class test):
```
✓ Feedback received!
    Position: 7.50°
    Speed:    0 ERPM
    Current:  0.00 A
    Temp:     33°C
    Error:    0
```

### Lesson learned
When a UART cable is physically connected to the AK60-6, the motor **completely suppresses
CAN feedback** — it will ACK CAN frames at the physical layer (so berr-counter stays 0) but
it will not transmit any feedback frames. Removing the UART cable immediately restores normal
50 Hz CAN broadcast.

### Confirmed protocol details
| Item | Value |
|---|---|
| Feedback CAN ID | `0x00002903` (extended 29-bit) |
| Feedback rate | **50 Hz** (when UART disconnected + motor enabled) |
| Also sends | One immediate reply frame per received command |
| Data format | Bytes 0-1: pos int16 x 0.1deg / 2-3: spd int16 x 10 ERPM / 4-5: cur int16 x 0.01A / 6: tmp int8 degC / 7: err uint8 |
| Enable command | `FFFFFFFFFFFFFFFC` to motor CAN ID (extended), then motor starts broadcasting |

### Code changes made
- Added `_parse_feedback_msg()` helper to `CubeMarsAK606v3CAN`
- Added `_capture_response(timeout=0.05)` — read immediate reply after each command
- Added `self._pending_feedback` cache — consumed by next `_receive_feedback()` call
- `enable_motor()`, `disable_motor()`, `_send_can_command()` all call `_capture_response()`
- `_receive_feedback()` checks `_pending_feedback` cache before blocking on bus.recv()
- Updated `quick_test.py` to label 0x2903 as `*** MOTOR FEEDBACK ***` and show parsed values

### Status: **FULLY WORKING** ✅

---

## CAN ID encoding reference (CubeMars native protocol)

```
Command arbitration ID = (mode << 8) | motor_can_id      [extended, 29-bit]

Mode 0x00  Duty cycle    → 0x00000003 for motor_id=3
Mode 0x01  Current       → 0x00000103
Mode 0x02  Current brake → 0x00000203
Mode 0x03  Velocity      → 0x00000303
Mode 0x04  Position      → 0x00000403
Mode 0x05  Set origin    → 0x00000503
Mode 0x06  Pos+Vel+Accel → 0x00000603

Special (also extended):
Enable  motor_id only  FF FF FF FF FF FF FF FC  → 0x00000003
Disable motor_id only  FF FF FF FF FF FF FF FD  → 0x00000003

Feedback (observed in journey doc): 0x00002903 extended
Feedback (NOT yet confirmed live):  still unknown — see hypotheses above
```
