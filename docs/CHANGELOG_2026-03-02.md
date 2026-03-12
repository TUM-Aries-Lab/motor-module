# Session Log — 2 March 2026

All work is on the `CAN_implementation` branch.
No code commits today — session was entirely research, analysis, and documentation.

---

## Topics Covered

### 1. Kvaser CAN Protocol Tutorial — Full Code Review

**Source:** https://kvaser.com/can-protocol-tutorial/

Read the complete Kvaser CAN Bus Protocol Tutorial and compared every concept
against our implementation in `cube_mars_motor_can.py`, `setup_can.sh`, and
`can0.service`.

#### Confirmed correct

| CAN concept | Where in our code |
|---|---|
| Extended CAN 2.0B (29-bit IDs) | `is_extended_id=True` on every `can.Message` |
| Contents addressing via arb ID | `(mode << 8) \| motor_id` formula in `_build_extended_id()` |
| Max payload 8 bytes, big-endian | `struct.pack(">i", ...)` + 8-byte pad in `_send_can_command()` |
| Hardware frame filter | `can_filters = [{"can_id": 0x2903, "can_mask": 0x1FFFFFFF, "extended": True}]` |
| 1 Mbit/s bus speed | `bitrate 1000000` in `setup_can.sh` and `can0.service` |
| Bus-off auto-recovery | `berr-reporting on restart-ms 100` |
| TX queue depth | `txqueuelen 1000` |
| CRC / bit stuffing / ACK handling | Transparent at hardware layer — no application code needed |

#### Now fully understood: ERROR-PASSIVE state (TEC = 144)

The comment in `setup_can.sh` referred to a TX error counter reaching 144.
The Kvaser tutorial explains the exact mechanism:

- Every unacknowledged TX frame increments the **Transmit Error Counter (TEC) by 8**.
- After TEC > 127 (16 failed transmissions: 16 × 8 = 128), the node enters
  **Error-Passive** state — it stops destroying bus traffic with active error frames
  but keeps retrying.
- By a special CAN rule, once error-passive, the TEC no longer increases for ACK
  errors — so the node retransmits forever.
- After TEC > 255, the node goes **Bus-Off** and stops transmitting entirely.

In our case: TEC reached 144 because we were transmitting with either the motor
off (no ACK coming back from anyone) or with the UART cable still plugged in
(motor ignoring CAN). `restart-ms 100` forces the kernel to auto-exit bus-off
every 100 ms. Manual `setup_can.sh` resets the interface to Error-Active (TEC = 0).

#### Gap identified: no error frame subscription at application level

The Linux kernel can surface CAN error frames (stuff errors, form errors,
ACK errors, bus-off events) as special receive messages to user space. Our code
does not subscribe to these. We handle `can.CanError` exceptions and track
`_consecutive_no_response`, but the application has no proactive visibility into
whether the interface is currently error-passive before bus-off occurs.

`restart-ms 100` at the kernel level is the operational safety net. Subscribing
to error frames would allow logging a warning when TEC climbs, but is not critical
for the current application.

---

### 2. CAN Bus Termination — Deep Dive

#### What the 120 Ω resistor does (physical explanation)

CAN uses two wires (CANH and CANL) carrying opposite-polarity differential signals.
An electrical signal (wave) travels down both wires at roughly 2/3 the speed of
light. When the wave reaches the **physical end of the cable with nothing connected**,
it has nowhere to go and reflects back — like an echo. The reflected wave arrives
at receiving nodes slightly delayed and superimposes on the next bit being
transmitted, corrupting it.

A 120 Ω resistor placed directly across CANH and CANL at each cable end absorbs
the wave. 120 Ω is chosen because it matches the **characteristic impedance** of
the twisted-pair cable specified by ISO 11898 (nominally 120 Ω, allowed range
108–132 Ω). When the termination impedance matches the cable impedance there is
no reflection — the wave is absorbed and consumed as heat in the resistor.

Two terminators in a bus (one at each physical end) appear in parallel to any
device measuring across CANH/CANL: 120 Ω ∥ 120 Ω = **60 Ω**. This 60 Ω load
is also important for the DC operating point of the transceiver chips.

At 1 Mbit/s (our bus speed), one bit is 1 µs. At 2/3c a signal travels ~200 m
per µs. A cable reflection from 1 m away returns in ~10 ns — well within the same
bit period. Without termination this would corrupt communication even on a
bench cable. The Kvaser oscilloscope pictures demonstrate this explicitly.

#### Our hardware — what needs physical checking

**End 1 — AK60-6 motor:**
The CubeMars AK60-6 has a **factory-installed internal 120 Ω termination resistor**.
Nothing to do.

**End 2 — SN65HVD230 transceiver board:**
The SN65HVD230 chip itself has no built-in terminator. Nearly all breakout boards
include a 120 Ω resistor on the PCB with one of:
- A **2-pin jumper header** (labelled `120R`, `TERM`, or `RT`) — close with a
  jumper cap to enable. No soldering.
- A **solder bridge** — two adjacent pads to short with solder.
- Pre-installed with no option (less common) — already enabled.

**Action required:** Visually inspect the SN65HVD230 board. Check for a jumper
header or solder bridge and enable it if not already done.

#### Verification procedure (multimeter method)

**Everything must be powered off before measuring.**
(Meter in resistance mode passes its own current; active bus voltages corrupt the
reading and can damage the meter.)

Power-off sequence:
1. Motor: cut 24V supply
2. Jetson: `sudo shutdown now`, then unplug USB-C power
3. Wait ~10 s for capacitors to drain

Measurement:
- Set multimeter to Ω (resistance) mode
- Touch probes to CANH and CANL at the transceiver board (polarity doesn't matter)

| Reading | Meaning | Action |
|---|---|---|
| ~60 Ω | ✅ Both terminators present and working | None |
| ~120 Ω | ⚠️ Only the motor's terminator active | Enable jumper/solder bridge on SN65HVD230 board |
| Open / very high | ❌ Neither terminator active | Check motor connector; enable board jumper |
| ~40 Ω | Third 120 Ω somewhere — overconstrained | Find and remove the extra one |

#### Verification procedure (candump method)

If `candump can0` shows a clean 50 Hz stream of 8-byte messages from ID `0x2903`
with no `ERRORFRAME` lines — termination is adequate.

Error frames (`ERRORFRAME`) in the candump output indicate electrical layer
problems, which missing or incorrect termination is the most common cause of.

#### Why the reflections matter more at 1 Mbit/s

The Kvaser tutorial's oscilloscope picture of a bus at 125 kbps without
termination already shows stuff errors that kill communication. Our bus runs at
1 Mbit/s — 8× shorter bit periods. Reflections that might be tolerated at lower
speeds corrupt frames reliably at 1 Mbit/s. Correct termination is not optional
at this speed.

#### Key numbers

| Parameter | Value |
|---|---|
| Cable characteristic impedance (ISO 11898) | 120 Ω (range 108–132 Ω) |
| Required terminator value | 120 Ω at each end |
| Parallel resistance of both terminators | 60 Ω |
| Maximum cable length at 1 Mbit/s | ~40 m |
| Our cable length | ~1 m (lab bench) |
| AK60-6 internal terminator | Yes (factory fitted) |
| SN65HVD230 board terminator | Depends on board revision — must check jumper |

---

## Knowledge from Kvaser Tutorial — Reference Notes

### Four CAN frame types

| Type | What it is | Do we use it? |
|---|---|---|
| Data Frame | Carries 0–8 bytes of payload | Yes — all commands and feedback |
| Remote Frame | Requests data from another node (no payload) | No — not used |
| Error Frame | 6 identical bits (violates stuffing) — broadcast when error detected | Hardware only — invisible to application |
| Overload Frame | Node requests a pause — obsolete | No |

### Error detection (hardware, fully transparent to our code)

| Mechanism | What it checks |
|---|---|
| Bit Monitoring | Transmitter reads back its own transmission — mismatch = Bit Error |
| Bit Stuffing | After 5 consecutive identical bits, a complement bit is inserted; violation = Stuff Error |
| Frame Check | Fixed-format fields (CRC delimiter, ACK delimiter, EOF) must have exact values |
| ACK Check | At least one node must send a dominant ACK bit; absence = ACK Error |
| CRC | 15-bit checksum over the frame; mismatch = CRC Error |

### Error confinement state machine

```
TEC/REC = 0 on power-up
   │
   ▼
Error Active   (TEC ≤ 127, REC ≤ 127)
   │  TEC > 127 or REC > 127
   ▼
Error Passive  (active error flags → passive; TEC no longer rises for ACK errors)
   │  TEC > 255
   ▼
Bus-Off        (no transmissions; restart-ms 100 forces recovery after 100 ms)
   │  auto-recovery (restart-ms) or manual reset
   ▼
Error Active
```

### Bus arbitration priority

- Lower arb ID = higher priority (dominant 0 wins wired-AND bus)
- Our duty-cycle command arb ID: `0x00000003` (very high priority)
- Motor feedback arb ID: `0x00002903`
- The 0x0088 flood device arb ID: `0x00000088` (higher than us — it won
  arbitration at 30 kHz, which is why the filter was essential)

### ACK and the single-node problem

The Kvaser tutorial explains: when a node is alone on the bus (no other node
to send an ACK), every frame gets an ACK error. TEC increments by 8 per frame.
After 16 frames the node goes error-passive and retransmits forever (TEC frozen).

This explains the behaviour we saw repeatedly: transmitting with the motor off
or UART cable plugged in → no ACK → TEC climbs → error-passive → `setup_can.sh`
needed to reset. The `restart-ms 100` recovers from bus-off but doesn't prevent
reaching error-passive state when there is genuinely no other node ACKing.
