# ADR-002: Bidirectional DShot Implementation

**Status:** Deferred — superseded by the 2026-08 implementation work below (RX
capture + eRPM decode verified on hardware, 100% CRC-valid across two
independent confirmation sweeps). Formal flip to Accepted is pending Phase
4/5/6 (driver integration, docs) per the project plan; not yet done. The
"Implementation Update (2026-08-23)" section and everything below it hold
the real investigation history, including dead ends - each subsection's own
heading/status line says whether it's verified or superseded, so read those
markers rather than assuming everything under "Implementation Update" is
final. Sections above "Implementation Update" are this ADR's original
pre-implementation analysis and contain some estimates/assumptions later
found inaccurate (flagged inline where relevant).
**Date:** 2026-02-01 (original analysis); implementation findings added 2026-08-23/24
**Context:** Exploring ESC telemetry via bidirectional DShot for the test bench

## Context

The test bench would benefit from real-time motor telemetry:
- **eRPM feedback** for closed-loop speed control
- **Motor health monitoring** (temperature, voltage, current)
- **RPM filtering** for vibration analysis

Bidirectional DShot enables this by allowing ESCs to send telemetry back to the flight controller on the same signal wire used for throttle commands.

## Current Hardware

| Component | Model | Bidirectional Support |
|-----------|-------|----------------------|
| Controller | Raspberry Pi Pico 2 (RP2350) | Capable (with PIO) |
| ESC | JHEMCU Dual 40A 2-in-1 | **No** - requires firmware update |
| ESC Firmware | BLHeli_S G-H-30 V16.7 | **No** - stock BLHeli_S lacks support |
| Motors | BetaFPV Lava 1104 7200KV | N/A (motors don't affect protocol) |

*Superseded 2026-08-23: a Skystar KM55A2 (4-in-1, AM32 firmware) is now also
on the bench and is the ESC actually used for the implementation and hardware
results below - it supports bidirectional DShot natively, resolving the
firmware blocker this table's "No" column describes. See "Implementation
Update" below.*

## Firmware Compatibility Analysis

Stock BLHeli_S firmware does not support bidirectional DShot. Alternative firmware options:

| Firmware | Bidirectional | Cost | Open Source | Requirements |
|----------|---------------|------|-------------|--------------|
| **Bluejay** | Yes | Free | Yes | ESC programmer or BLHeli Configurator passthrough |
| **JESC** | Yes | ~$5 total | Partial | ESC programmer, paid license |
| **BLHeli_32** | Yes (native) | N/A | No | Different ESC hardware (not compatible) |
| **AM32** | Yes | Free | Yes | Different ESC hardware (ARM-based) |

### Recommendation: Bluejay

Bluejay is the recommended path for the current ESCs:
- Free and open source
- Active development community
- Compatible with EFM8BB MCU in JHEMCU ESCs
- Supports bidirectional DShot, EDT (Extended Telemetry)

**Blocker:** Flashing requires either:
1. ESC programmer (e.g., Arduino-based BLHeli programmer)
2. Flight controller with BLHeli passthrough (not available on Pico)

## Protocol Analysis

### Bidirectional DShot Timing

```
Standard DShot (current):
┌────────────────────────────────────────────────────────┐
│  FC TX (16 bits)                                       │
└────────────────────────────────────────────────────────┘

Bidirectional DShot:
┌──────────────────┐     ┌───────────────────────────────┐
│  FC TX (16 bits) │ gap │  ESC RX (21 bits GCR)         │
└──────────────────┘     └───────────────────────────────┘
                    │◄──►│
                     30µs
                  (turnaround)
```

### Signal Differences

| Aspect | Standard DShot | Bidirectional DShot |
|--------|---------------|---------------------|
| Signal polarity | Normal (HIGH=1) | Inverted (HIGH=0) |
| CRC calculation | `(value ^ (value >> 4) ^ (value >> 8)) & 0x0F` | Inverted: `~crc & 0x0F` |
| Communication | Unidirectional (FC→ESC) | Half-duplex (FC↔ESC) |
| Update rate | Full speed | ~50% (wait for response) |

### GCR Encoding

ESC response uses GCR (Group Code Recording) for noise immunity:

| Stage | Bits | Description |
|-------|------|-------------|
| Raw eRPM data | 12 | 3-bit exponent + 9-bit mantissa |
| + CRC | 16 | 4-bit CRC appended |
| GCR encoded | 20 | 4-bit nibbles → 5-bit symbols |
| + Marker bit | 21 | Final transmission size (marker is always 0 - functions as a frame marker, not a UART-style start bit) |

**GCR Symbol Table** (corrected 2026-08-23 against AM32 firmware source,
`Src/dshot.c`'s `gcr_encode_table[16]` - the table originally here agreed
with AM32's real table on only 7 of 16 entries and diverged on the rest; see
"Implementation Update" below for the source-verification method and for a
separate, related finding that `specification/DSHOT_PROTOCOL.md`'s own GCR
table is also wrong against this same source):

| Nibble | GCR | Nibble | GCR |
|--------|-----|--------|-----|
| 0x0 | 0x19 | 0x8 | 0x1A |
| 0x1 | 0x1B | 0x9 | 0x09 |
| 0x2 | 0x12 | 0xA | 0x0A |
| 0x3 | 0x13 | 0xB | 0x0B |
| 0x4 | 0x1D | 0xC | 0x1E |
| 0x5 | 0x15 | 0xD | 0x0D |
| 0x6 | 0x16 | 0xE | 0x0E |
| 0x7 | 0x17 | 0xF | 0x0F |

### Bitrate Calculation

GCR response is transmitted at 5/4× the DShot bitrate:

| DShot Variant | TX Bitrate | RX Bitrate (GCR) | Bit Period |
|---------------|------------|------------------|------------|
| DShot300 | 300 kbit/s | 375 kbit/s | 2.67µs |
| DShot600 | 600 kbit/s | 750 kbit/s | 1.33µs |
| DShot1200 | 1200 kbit/s | 1500 kbit/s | 0.67µs |

Actual hardware measurements on DShot300 (the variant channel 1 uses) put
the real bit period around 2.5-2.6µs, a few percent off the 2.67µs nominal
figure above. This is not something the implementation depends on either
way: `scripts/decode_bidir_capture.py`'s decoder estimates the real bit
period from each individual capture's edge timing and hardcodes neither
this nor any other fixed value.

### eRPM Decoding

**Corrected/completed 2026-08-23** - the original version of this pseudocode
skipped the GCR table lookup entirely (treating the raw differentially-
decoded value as if it were already the reassembled 16-bit number), which
was in practice the single most bug-prone step of this whole project (wrong
spec table, wrong bit width, wrong frame length - see "Implementation
Update"). The lookup step is not optional and must not be skipped:

```python
# 1. Differential/Gray decode across the 20 real data bits, packed MSB-first.
#    Equivalent to per-bit decoded_bit[i] = raw_bit[i] ^ raw_bit[i-1], with
#    the chain seeded from the marker bit's own value (always 0) - NOT a
#    separate seed bit (see "Implementation Update" for why an early "marker
#    + seed + 20 data" 22-bit frame model was tried and disproven).
decoded20 = gcr_value ^ (gcr_value >> 1)

# 2. Split into four 5-bit GCR symbols and reverse-map each through the
#    symbol table above (symbol -> nibble). Do not skip this step.
nibbles = [GCR_DECODE_TABLE[(decoded20 >> shift) & 0x1F] for shift in (15, 10, 5, 0)]
dshot_full_number = (nibbles[0] << 12) | (nibbles[1] << 8) | (nibbles[2] << 4) | nibbles[3]

# 3. Extract CRC + data fields from the reassembled 16-bit number
crc = dshot_full_number & 0x0F
data12 = (dshot_full_number >> 4) & 0xFFF
mantissa = data12 & 0x1FF          # 9 bits
exponent = (data12 >> 9) & 0x07    # 3 bits

# 4. Calculate period in microseconds, then eRPM
period_us = mantissa << exponent
if period_us > 0:
    erpm = 60_000_000 / period_us

# Convert to mechanical RPM (motor poles / 2). eRPM above is the verified
# quantity; this conversion is not: `driver`/`scripts` code uses
# MOTOR_POLES = 14 (AM32's EEPROM default), unverified for this specific
# motor/ESC pairing. This table originally guessed 12 (typical for small
# FPV motors) - the two disagree, and neither has been confirmed against
# the actual BetaFPV Lava 1104 hardware. Every RPM figure quoted anywhere
# in this ADR carries this uncertainty as a constant scale factor; eRPM,
# CRC-validity, and monotonicity/repeatability conclusions do not.
rpm = erpm / (motor_poles / 2)
```

## PIO Implementation Analysis

### Resource Requirements

**Corrected 2026-08-23:** this section originally assumed 8 total state
machines (the original Pico/RP2040's 2 PIO blocks × 4 SMs). The Pico 2
(RP2350) actually on the bench (see "Current Hardware" above) has **3 PIO
blocks × 4 SMs = 12 total** - the numbers below are corrected accordingly.
This undercount didn't end up constraining the actual design (Phase 1
scoped to channel 1 only, using 2 of 12), but is worth fixing for accuracy.
One real constraint the "remaining" column doesn't capture: a GPIO's
function-select routes to only one PIO block at a time, so a bidirectional
motor's TX+RX pair must share a block (max 4 SMs) even though 12 are
available chip-wide - see "Implementation Update" for how this actually
constrained channel 1 vs channels 2-4's PIO block placement.

**Current (TX only):**
- 1 state machine per motor
- 2 motors = 2 SMs
- 10 SMs available (12 total - 2 used)

**Bidirectional options:**

| Approach | SMs per Motor | Total (2 motors) | Remaining |
|----------|---------------|------------------|-----------|
| **A. Mode switching** | 1 | 2 | 10 |
| **B. Dual SM (TX+RX)** | 2 | 4 | 8 |
| **C. Shared RX** | 1.5 | 3 | 9 |

### Recommended: Option B (Dual SM per Motor)

*The dual-SM-per-motor direction was validated: the implemented design
("Option A'", see "Implementation Update") also uses one TX SM + one RX SM
per bidirectional motor. The internal design differs substantially from the
sketch below, though (IRQ handshake between the two SMs, not independent
free-running programs) - see `driver/dshot_pio.py`'s `dshot_bidir_tx`/
`dshot_bidir_rx` for what actually shipped.*

```
Motor 0:                              Motor 1:
┌─────────┐   GPIO 4   ┌─────────┐   ┌─────────┐   GPIO 5   ┌─────────┐
│  SM 0   │◄─────────► │   ESC   │   │  SM 2   │ ◄─────────►│   ESC   │
│  (TX)   │            │    0    │   │  (TX)   │            │    1    │
└─────────┘            └─────────┘   └─────────┘            └─────────┘
┌─────────┐                          ┌─────────┐
│  SM 1   │◄────────── (shared pin)  │  SM 3   │◄────────── (shared pin)
│  (RX)   │                          │  (RX)   │
└─────────┘                          └─────────┘
```

### PIO Program Sketches

*Superseded - these are the original speculative sketches, not the final
implementation. The actual verified programs (`dshot_bidir_tx`,
`dshot_bidir_rx` in `driver/dshot_pio.py`) differ substantially: TX uses a
manual `pull()` + bit counter (not autopull) so it can release the pin at a
known point and raise a PIO IRQ; RX waits on that IRQ, then a fixed hardware
delay, then densely oversamples the whole reply (128 raw samples,
`autopush=True, push_thresh=32`) rather than point-sampling 21 slots at a
guessed bitrate - see "Implementation Update" for the full history of why
the point-sampling approach below didn't work.*

**TX Program (modified for inverted signal):**
```python
@asm_pio(sideset_init=PIO.OUT_HIGH, out_shiftdir=PIO.SHIFT_LEFT)
def dshot_bidir_tx():
    # Inverted: side(1) = LOW output, side(0) = HIGH output
    wrap_target()
    out(x, 1)           .side(1)    [1]  # Start LOW (inverted HIGH)
    jmp(not_x, "zero")  .side(0)    [2]  # LOW = 1 in inverted logic
    jmp("start")        .side(0)    [2]  # Stay "low" (inverted high)
    label("zero")
    jmp("start")        .side(1)    [2]  # Go "high" (inverted low)
    wrap()
```

**RX Program (GCR capture):**
```python
@asm_pio(in_shiftdir=PIO.SHIFT_LEFT, autopush=True, push_thresh=21)
def dshot_bidir_rx():
    # Wait for start bit (falling edge in inverted mode)
    wait(0, pin, 0)

    # Sample 21 bits at 5/4× bitrate
    set(y, 20)                      # 21 bits to capture
    label("bitloop")
    in_(pins, 1)            [N]     # Sample and delay (N based on bitrate)
    jmp(y_dec, "bitloop")

    # Data pushed automatically via autopush
```

### Timing Coordination

```
┌─────────────────────────────────────────────────────────────────┐
│                    One Bidirectional Cycle                      │
├──────────────┬────────┬──────────────────────┬──────────────────┤
│   TX Frame   │  Gap   │      RX Frame        │     Idle         │
│   26.7µs     │  30µs  │       28µs           │    ~15µs         │
│  (16 bits)   │        │    (21 bits GCR)     │                  │
├──────────────┴────────┴──────────────────────┴──────────────────┤
│              Total cycle: ~100µs (10kHz max)                    │
└─────────────────────────────────────────────────────────────────┘
```

At DShot600:
- TX: 16 bits × 1.67µs = 26.7µs
- Gap: 30µs (line turnaround)
- RX: 21 bits × 1.33µs = 28µs
- **Total: ~85µs per motor** (effective update rate ~11.7kHz per motor) -
  this excludes the diagram's ~15µs idle box above (26.7+30+28+15 ≈ 100µs,
  matching the diagram's own ~10kHz figure); the two totals aren't
  inconsistent, they just include/exclude idle time differently.

**Both of the above are DShot600 pre-implementation estimates and do not
describe what was actually built or measured.** Channel 1 (the only
bidirectional channel implemented) runs DShot300, not DShot600. The 30µs
turnaround figure in particular was a generic estimate that hardware
measurement later replaced: the real fixed delay before RX starts listening
is ~4.7µs (see "Implementation Update"'s RX redesign section), not 30µs. The
actual per-cycle timing is in any case dominated by the application's own
update-loop cadence (tests hold each throttle step for seconds), not by this
protocol-level minimum - these numbers were never load-bearing for anything
built.

## Implementation Phases

If pursuing bidirectional DShot in the future:

### Phase 1: Firmware Update
1. Acquire ESC programmer or compatible flight controller
2. Backup current ESC settings
3. Flash Bluejay firmware
4. Verify standard DShot still works

### Phase 2: Inverted TX
1. Modify `DShotPIO` to support inverted signal mode
2. Use inverted CRC calculation
3. Verify ESC recognizes bidirectional mode

### Phase 3: RX Implementation
1. Create RX PIO program for GCR capture
2. Add GPIO direction switching
3. Implement timing coordination between TX and RX

### Phase 4: Telemetry Processing
1. GCR decoding in Python
2. eRPM calculation
3. Integration with `MotorThrottleGroup` facade

## Decision

*Superseded 2026-08-23 - the blocker below no longer applies (an AM32 ESC is
now on the bench with no ESC-programmer/Bluejay-flash requirement) and
implementation has since happened and been verified on hardware; see
"Implementation Update" below. This section is kept for historical context
on why bidirectional DShot was originally shelved. A formal status flip to
Accepted is still pending completion of Phase 4/5/6 (driver integration,
docs) - see the project plan.*

**Deferred** - Bidirectional DShot implementation is postponed due to:

1. **Firmware blocker**: Current BLHeli_S firmware doesn't support bidirectional DShot
2. **Hardware requirement**: Need ESC programmer to flash Bluejay firmware
3. **Alternative path**: Could acquire newer ESCs with BLHeli_32 or AM32 pre-installed

### Prerequisites for Future Implementation

- [ ] ESC programmer (Arduino-based or dedicated)
- [ ] OR: ESCs with bidirectional-capable firmware (BLHeli_32, AM32)
- [ ] Bluejay firmware flashed and verified
- [ ] Standard DShot confirmed working post-flash

## Consequences

*Superseded 2026-08-23 - see "Implementation Update" below for the current
state (RX capture + eRPM decode verified on hardware, not yet wired into the
driver's public API). The "Current State" below describes the pre-AM32-ESC
bench and is out of date.*

### Current State
- Standard (unidirectional) DShot works reliably
- No eRPM telemetry available
- Motor speed estimation would require external sensor (optical/magnetic encoder)

### When Implemented
- Real-time eRPM feedback (~11.7kHz update rate)
- Closed-loop speed control capability
- Extended telemetry (temperature, voltage, current) with EDT
- ~50% reduction in effective command rate (acceptable trade-off)

## Implementation Update (2026-08-23)

The blocker above is gone: a Skystar KM55A2 (4-in-1, AM32 firmware) is now on the
bench and already proven for unidirectional DShot300 (see `tests/test_slow_spin.py`
and README's "Verified Parameters"). AM32 supports bidirectional DShot natively -
no ESC programmer or Bluejay flash needed. Implementation is underway on
`feature/bidirectional-dshot`; this section records findings and open design
candidates so they survive context resets, not a final decision.

### Confirmed from AM32 firmware source (`am32-firmware/AM32`, `Src/dshot.c` /
`Src/main.c` / `Src/signal.c` on GitHub)

- Bidirectional mode is auto-detected per motor from **idle-line polarity**
  (idle HIGH vs the normal idle LOW), checked only while `!armed` - the ESC
  never re-checks once its own internal arm state is set. This means the
  inverted signal must be present for the **entire** arm sequence, not
  switched in afterward. No ESC-side configuration exists for this (the
  EEPROM `bi_direction` flag / DShot commands 9-10 turn out to control BEMF
  startup tuning, not telemetry - a red herring from an AI-generated wiki
  page, corrected against source).
- Once bidirectional mode is latched, AM32 replies with a GCR telemetry frame
  after **every** received command frame, unconditionally - not gated on the
  command's telemetry-request bit (`Src/signal.c`'s `transfercomplete()`
  simply alternates receive/reply on every DMA completion once
  `armed && dshot_telemetry`).
- AM32's real `gcr_encode_table[16]` (`Src/dshot.c`) does **not** match the
  GCR symbol table in `specification/DSHOT_PROTOCOL.md` - they agree on 6 of
  16 entries and diverge after that. That spec table is wrong (or at least
  not what this firmware implements) and needs fixing before it's trusted
  for a decoder. Confirmed independently: it matches betaflight's own
  `gcrs[]` reverse-lookup table exactly (see below), so the AM32-derived
  table is the one to build a decoder against.

### Hardware findings from Phase 2/3 bring-up

- Phase 2 (inverted TX waveform for the whole arm sequence, no RX) verified
  on hardware: arms normally, spins normally. Confirms the theoretical
  finding above empirically.
- Phase 3's first RX attempt (point-sample once per bit, `wait(0,pin,0)`
  immediately after `rx_resync()`) produced consistent all-zero captures.
  Root cause: the released line floats rather than idling HIGH, so RX
  false-triggers on the float itself. Fixed with the Pico's own internal
  pull-up (`Pin.IN, Pin.PULL_UP`) on the shared GPIO - AM32's
  `setInputPullUp()` (found in source, `Src/main.c`) apparently doesn't
  cover this pin/mode strongly enough to rely on alone.
- After the pull-up fix, captures looked structurally plausible by eye
  (varied, partially repeating) but **statistical validation proved this
  was noise**: decoding against AM32's real GCR table and CRC gave a 4/84
  (~4.8%) hit rate for even getting valid GCR symbols - indistinguishable
  from the ~6.25% expected by pure chance (P(4 random 5-bit groups all
  valid) = 0.5^4). Lesson: eyeballing hex for "looks structured" is not a
  substitute for actually running the decode + CRC check: it can't
  distinguish real capture from coincidence. Do this arithmetic before
  declaring an RX capture path verified, not after.
- `MotorThrottleGroup`/`DShotPIO`'s `rx_resync()` (`StateMachine.restart()`)
  was independently confirmed correct at the PC level - MicroPython's own
  docs state `restart()` "restarts the state machine and jumps to the
  beginning of the program," equivalent to the Pico C-SDK's
  `pio_sm_restart` *plus* a manual `pio_sm_exec_wait_blocking(jmp)` that
  betaflight's driver needs as two separate calls. It does **not** clear
  the RX/TX FIFOs (separate hardware) - that still needs an explicit drain,
  which the current capture code does.

### Reference implementation: betaflight's RP2350 port

Betaflight added Pico/Pico 2 support with a working bidirectional DShot PIO
implementation:
[PR #14618](https://github.com/betaflight/betaflight/pull/14618/files),
`src/platform/PICO/dshot.pio` and `dshot_bidir_pico.c` /
`dshot_pico.c`. Key design points that our first RX attempt got wrong:

- The ESC's reply turnaround is a **fixed ~25-30µs wall-clock delay**,
  documented as independent of DShot speed - not the ~30µs-from-a-generic-
  article figure this ADR originally quoted, and not something our RX
  program accounted for at all (it started `wait(0,pin,0)` immediately after
  resync, with no fixed delay). This is very likely why our capture read
  noise: it was very likely triggering off frame-release transients rather
  than the real reply.
- Each GCR bit is **oversampled 3x** (roughly 1/6, 1/2, 5/6 through the bit
  cell) rather than point-sampled once, with the bit value decided by
  majority vote in software (C, off-chip) rather than in PIO. Far more
  tolerant of small phase/clock-mismatch error than a single precisely-timed
  sample - directly validates the "just capture more raw bits and analyze"
  approach discussed live rather than trying to nail exact PIO-level timing
  blind.
- The framing/marker bit (the GCR-encoded value's implicit leading bit) is
  **0**, not 1 as this ADR's implementation work initially assumed from a
  literal reading of AM32's encode source.
- On the C-driver side (`dshot_pico.c`'s `dshotUpdateComplete()`), the SM is
  fully stopped, restarted (recovering a stalled `wait` mid-receive, which
  has no timeout), FIFOs cleared, refilled, and restarted **on every single
  frame** when telemetry is enabled - not left to free-run continuously.
  This is what makes an untimed `wait(0,pin,0)` safe even before
  bidirectional mode is detected during arming.
- Their program does TX and RX in **one PIO program on one state machine**,
  using manual `pull()`/`push(noblock)` rather than autopull/autopush - this
  sidesteps the constructor-time autopull/autopush threshold conflict
  (pull_thresh=16 vs push_thresh=21) that was this ADR's original reason for
  preferring two state machines per motor. That reason no longer holds; two
  SMs is now a preference (keeps the Phase-2-verified TX program untouched),
  not a technical necessity.

### Design candidates for RX synchronization (not yet decided)

**Option A' - keep dual-SM, add a PIO-to-PIO IRQ handshake.** TX program
(unchanged from the Phase-2-verified waveform) raises a PIO IRQ right after
releasing the pin each frame; RX program waits on that IRQ, then a fixed
~25-30µs delay, then listens with 3x oversampling per bit. Both state
machines already sit on the same PIO block (required since they share a
GPIO), which PIO IRQ signalling needs anyway.
- Pro: the exact TX waveform already arm-verified on this ESC never
  changes - zero new risk to the one thing repeatedly proven fragile here
  (500ms arm duration wasn't enough, needed 3000ms; anything less than
  back-to-back framing failed to arm at all).
- Con: PIO inter-SM IRQ handshaking is new ground for this codebase; two
  programs to keep in sync; not a direct port of a working reference.

**Option B - single-SM, close port of betaflight's `dshot_600_bidir`.**
One PIO program handles TX then RX, using betaflight's per-frame
stop/restart/clear-FIFO/refill/re-enable pattern (MicroPython's
`StateMachine.restart()` already provides the "jump to program start" half
of what betaflight needs two C calls for).
- Pro: closest to a known-working artifact; naturally captures every
  frame's reply (useful for Phase 5's eventual per-frame telemetry
  integration anyway); frees a state machine.
- Con: replaces the exact TX program Phase 2 verified, so arm/spin needs
  re-verification from scratch on real hardware; introduces a full
  stop/restart/refill/re-enable cycle on **every frame**, which is exactly
  the kind of per-frame overhead/irregularity this specific ESC has already
  shown unusual sensitivity to (it's the one ESC where even a clean,
  jitter-free 250µs-paced loop failed to arm - only true back-to-back
  framing worked). Betaflight's own testing fleet may simply not include an
  ESC this fussy about timing.

**Leaning A' as primary**, with B as a fallback only if A' turns out
unworkable and the team is willing to spend a hardware round re-verifying
arming from scratch. The single largest realized risk in this project to
date has been this ESC's arming fragility, and A' is the only candidate that
leaves the arm-proven code path completely untouched.

### RX capture diagnosis, Option A' implementation (2026-08-23, continued) - open, NOT verified (superseded - see "RX redesign: unslotted dense oversampling" below)

Option A' (PIO-to-PIO IRQ handshake) was implemented and iterated on hardware
through several real bugs. **Phase 3 is not yet verified** - do not treat
anything below as a working checkpoint. Recorded so the next session doesn't
re-derive it.

**Bugs found and fixed, in order:**
1. A hand-written two-word RX loop (manual `push()`/`jmp()` every 10 bits)
   added 3 uncompensated cycles at the word boundary, corrupting the second
   word's sample phase. Fixed by switching to `autopush` at a flat
   `push_thresh=30`, one 20-bit loop, no word seam.
2. First hardware run captured pure noise (4/84 valid GCR symbols, ~ the
   6.25% chance baseline) because the original design point-sampled once
   per bit with no fixed post-release delay, and used the wrong table (see
   above) - both fixed per the betaflight reference findings already
   recorded above.
3. A diagnostic capture-window widening used `set(y, 39)` - illegal, since
   PIO `set`'s immediate is a 5-bit field (max 31). This caused two
   confusing hardware hangs that were *not* TX/RX pin contention (that
   theory was raised and should be discarded) - just a misassembled
   program. Corrected to `set(y, 29)` (30 slots = 90 samples = exactly 3
   words/frame at `push_thresh=30`, comfortably under the 4-word FIFO
   depth).
4. The real bug behind the "noise-but-structured, 0/17 CRC-valid, extent
   ~14/20 slots" symptom that followed: **the fixed post-release predelay
   (~28µs, copied from a generic figure) massively overshot AM32's actual
   reply turnaround** on this ESC. Offline run-length analysis of raw
   sample transitions (histogram of run widths in cycles, read as sample
   counts rather than raw cycles) confirmed the RX clock and the 8-cycles/
   bit design were both correct all along - `rx_speed`'s 5/4 multiplier is
   right and should not be touched. The predelay was shortened to ~4.7µs
   (`set(x, 1)` instead of `set(x, 11)`), and a related 1-cycle bug in the
   marker-skip (`wait(0, pin, 0)` consumes a cycle on trigger that the
   following `nop()[7]` didn't account for, making the skip 9 cycles instead
   of 8) was fixed to `nop()[6]`.

**Result after the predelay fix (current state, still open):**
- Extent (idle onset) jumped from ~14/20 slots to ~17-18/30 slots, exactly
  as predicted for a corrected predelay - alignment is no longer the
  primary suspect.
- A consistent window start (start=0, i.e. the bitloop's first sample is
  the real first data bit) produces syntactically valid GCR symbols in
  7-8 of 17 captures - well above the ~6.25% chance baseline for 4
  simultaneously-valid 5-bit groups, so this is real signal, not noise.
- **Only one capture (of 17) also passed the CRC check**, and one hit is
  chance-level (P(CRC match | valid symbols) = 2/16 = 12.5% by chance) -
  **do not treat that single eRPM value (mantissa=373, exponent=3, ~2872
  RPM) as a real reading.** No eRPM value from this session should be
  trusted or quoted until the hit rate clears "majority of captures," per
  the working-bidirectional-DShot expectation of >90% valid frames on a
  clean wire.
- A per-sample-position phase probe (decoding with only the first, middle,
  or last of the 3x-oversampled raw samples, instead of majority vote) did
  **not** point to a sampling-phase fix: no single sample position reaches
  CRC validity either (sample[0]: 8 symbol-valid/0 CRC; majority: 7/1;
  sample[2]: 1/0). If phase alone explained the residual error, the best
  position would show it - it doesn't.

**Nibble ordering and CRC polarity are now proven correct against AM32
source - do not re-investigate them.** `crc_inverted` matches AM32's actual
`make_dshot_package` checksum computation exactly (brute-forced over all
4096 possible 12-bit values, 0 mismatches), and this codebase's nibble
reassembly order matches AM32's `dshot_full_number` GCR-symbol assembly
exactly (2000/2000 round-trip match in a from-scratch simulation of AM32's
real `gcr[]` array construction). An "off-by-one, skip one more bit for the
fixed seed" variant was also tested against the real captures and made
things *worse* (0/17 vs the original 7/17 symbol-valid) - the empirical
alignment (window start=0, prev=0) really is correct.

**Correction (do not trust the "seed=1" claim below - see "RX redesign:
unslotted dense oversampling" further down):** the 2000/2000 simulation
above also assumed the frame had two fixed leading bits before the 20 real
data bits - marker=0, then a separate fixed seed=1 - i.e. 22 bits total.
That frame-length assumption was itself wrong, disproven on real hardware
later the same day: the correct, hardware-verified model is 21 bits (marker
+ 20 differentially-encoded data bits, chain seeded from the marker's own
value 0, no separate seed bit). The 2000/2000 round-trip only proves the
simulation was internally consistent with its own (wrong) assumption, not
that the assumption was correct - a from-scratch simulator sharing the same
wrong assumption as the code under test can't catch that class of bug (see
"Two lessons worth keeping" further down). The nibble-order and CRC-polarity
findings in the paragraph above remain correct and verified; only the
22-bit/seed-bit frame model is disowned.

**The sharpest unexplained fact, found by localizing failures per GCR
symbol position:** symbol 1 (decoded bits 0-4, the data's top nibble) is
valid in **17 of 17** captures and its nibble is perfectly deterministic
per throttle group (always 7 at throttle=100, always 5 at throttle=200,
always 3 at throttle=300) - not just valid, stable. Failures then rise
monotonically deeper into the frame: symbols 2/3/4 invalid in 0/2/4/4 of 17
captures respectively (symbol 4 is the CRC nibble). A within-throttle-group
bit-disagreement check (diffing the majority-vote bit string across
captures sharing a throttle, where real eRPM should be near-identical)
corroborates: zero disagreement in the first 2-3 captured bit positions,
rising toward the middle/back. This pattern (clean front, degrading back)
rules out both a leading-edge framing seam (would corrupt symbol 1 first -
it doesn't) and uniformly-scattered sampling noise (would hit the front and
back equally - it doesn't).

**Extent-based period re-estimate (still open, not confirmed):** if the
real 20-bit frame's duration is measured as `extent_slots * 8 / 20` cycles,
the 17 captures give a tight 6.8-7.2 cycle range (mean ~7.0) - suggesting
the real GCR bit period might be ~7.0 cycles rather than the assumed 8,
i.e. a ~10/7 rate ratio to the command rate instead of 5/4. This would be
architecturally plausible (AM32's reply period is `96 * (prescaler+1) /
CPU_FREQUENCY_MHZ` - a fixed 96-count timer ARR against a target-specific,
possibly non-round clock, so a rate a few percent off 5/4 is a plausible
hardware artifact, not a protocol violation). **However, resampling the
existing raw captures onto a 7.0-cycle grid (nearest-sample from the
existing 3 samples/slot) does not validate** - CRC hits went to 0/17 at
7.0 (from 1/17 at the current 8-cycle assumption), and 6.8/7.2 also gave
0/17. This is weak evidence (only 3 unevenly-spaced samples/slot to
resample from) but it's the only check available offline, and it does not
support spending a hardware round on `rx_speed = dshot_speed * 10 // 7` -
**do not make that change**. The rate question is open, not settled either
way, given the resampling test's weakness.

**Conclusion: this dataset (17 captures at 3 samples/slot) is exhausted.**
Every offline check that could be run against it has been - decode logic,
alignment, nibble order, CRC, per-sample-position phase, extent-based rate
re-estimate. None isolates the remaining defect. Alignment, GCR table,
nibble order, and CRC are all now proven correct and should not be
revisited.

### RX redesign: unslotted dense oversampling (2026-08-23, continued) - VERIFIED ON HARDWARE

**Phase 3 is verified.** 17/17 real captures decode with valid CRC; eRPM
rises monotonically across throttle steps 100/200/300 (~21.6k -> ~48.8k
-> ~76.1k eRPM, tightly clustered within each throttle group, exponent
stepping 3->2->1 exactly as a shrinking commutation period should - not
something chance produces). See "Hardware result" below for the full
account, including a second frame-model bug found only once real hardware
data was available.

`dshot_bidir_rx` was rewritten from scratch rather than further refining
the slotted design. Instead of assuming a bit period (8 PIO cycles, 3
samples at fixed offsets) and inferring backward from a coarse vote when
that assumption turned out to be slightly wrong, it now makes **no**
assumption about bit period: it samples the pin uniformly and continuously
(128 samples, marker bit included this time rather than skipped) at a
clock chosen only to safely oversample the plausible range of real bit
periods. `scripts/decode_bidir_capture.py` now does what the PIO program
deliberately no longer does - finds the marker edge, estimates the real bit
period directly from edge-to-edge gaps in the raw waveform (sweeping
fractional candidate periods, not just integer sample-gap counts - the
true period is generally not a whole number of cycles), then resamples and
decodes.

**Two real bugs caught before this ever touched hardware**, both via
building a from-scratch simulator (encodes a random 16-bit value exactly
as AM32's `make_dshot_package`/`gcr[]` does, samples it the way the PIO
program would, feeds it through the decode pipeline, and checks the
round-trip):
1. `set(y, 127)` for a flat 128-sample loop is illegal - `set`'s immediate
   is a 5-bit field, max 31. Same bug class as the `set(y, 39)` hang
   earlier in this investigation. Fixed with a nested loop (4 outer passes
   of 32 inner samples each), which costs 2 extra PIO cycles at each of the
   3 pass boundaries - fully deterministic, so the analysis script computes
   each sample's exact absolute cycle position (`sample_cycle()`) rather
   than assuming uniform spacing.
2. The resample window was off by one full bit period: `marker_end` (where
   the marker bit ends) is also where AM32's fixed seed bit begins, not
   where real data begins - the window needs to skip the seed bit too.
   Caught because the simulator's round-trip failed 0/200 before this fix
   and passed 187-200/200 after.

**Validation (simulation, no hardware):** round-tripping 200 random 16-bit
values at each of 8 candidate real bit periods (2.1-2.8us, covering the
full range this project's measurements have suggested so far) with random
sub-cycle phase jitter, sampled at the exact non-uniform cycle positions
the real PIO program produces: 187-200/200 correctly decoded at every
period tested. Of the captures that failed to decode correctly, all but
one were caught by CRC and rejected outright (no plausible-looking wrong
answer); the single exception was a CRC coincidence (1/2400 trials overall
- consistent with a 4-bit CRC's inherent ~1/16 chance of matching wrong
data, not a design flaw).

**Hardware result:** the first hardware run of this design produced a
precise, highly consistent period measurement (10.1-10.4 PIO cycles, ~2.5-
2.6us, essentially identical `marker_end` cycle position across all 17
captures) - a real improvement over the old slotted design's measurements.
But decoding via `resample_bits` (fixed-offset resampling, same method the
simulation validated) still failed almost completely: 2/17 symbol-valid,
0/17 CRC-valid, and a full phase sweep across +/-5 cycles found *zero*
CRC hits anywhere - a flat result, worse than the old slotted design's
chance-level 1/17. This mattered because it meant the failure was
structural (wrong number of bits assumed), not a timing/phase problem no
sweep could fix.

**Root cause: the simulator's 22-bit frame model (marker + separate fixed
"seed" bit + 20 data bits) was wrong.** Reconstructing the actual bit
sequence via run-length decoding (each run's duration / period, rounded to
the nearest integer - see `reconstruct_bits()`) rather than resampling
gave a directly countable total: 19-20 bits before idle, consistently -
never enough for a 22-bit model, and short even of the simpler 21-bit
(marker + 20 data, no separate seed) model by 1-2 bits. The shortfall
turned out to be trailing real data bits whose value (1) matches idle's
level merging invisibly into the idle run, with no edge to mark where they
end - undercounting is expected, not a sign the frame itself is shorter
than 21 bits.

Testing both candidate frame models against the run-length reconstruction
(padded with idle-value bits up to each model's expected total, to recover
the merged trailing bits) settled it immediately: the 21-bit model (marker
+ 20 differentially-encoded data bits, differential chain seeded from the
marker's own value 0 - **no separate seed bit**) gave **17/17 CRC-valid**;
the 22-bit model gave 2/17 symbol-valid, 0/17 CRC-valid, matching the
original resampling failure exactly. This also retroactively explains the
old slotted design's finding from earlier tonight: seeding from `prev=0`
at window start=0 was the only combination that ever showed real signal
because it was the *correct* frame model all along - the slotted design's
resampler was just too coarse (3 samples/slot, fixed offsets) to deliver
clean bits from it.

**Two lessons worth keeping:**
- **Run-length reconstruction beat resampling because it's immune to
  accumulated phase error.** A resampler's fixed-offset window walks out
  of phase with the real signal bit-by-bit deeper into the frame (exactly
  the "clean front, degrading back" signature seen throughout this ADR);
  run-length reconstruction only needs each individual run's duration
  relative to the period, so small period-estimate error doesn't compound
  across 20 bits. This is now `scripts/decode_bidir_capture.py`'s
  production decode method - `resample_bits` was removed, not kept as an
  alternative.
- **The from-scratch simulator was still worth running despite encoding
  the wrong frame model** - it caught two real bugs before hardware (an
  illegal `set(y, 127)` immediate, and later shown to be moot once the
  frame model was corrected: an "off-by-one" fix to the resample window
  that was actually compensating for the wrong assumption, not a real
  timing bug). Simulation validates internal consistency of a pipeline
  against its own assumptions; it cannot catch a wrong assumption shared
  by both the simulator and the code under test. Don't cite the 187-
  200/200 simulation result as evidence for a 22-bit frame - it's
  disproven.

**Status: Phase 3 verified. Not yet integrated into `MotorThrottleGroup` or
`DShotPIO`'s public API** (Phase 4/5 per the original plan) - the decode
pipeline currently lives only in the offline `scripts/decode_bidir_capture.py`
tool. The 128-sample/4-word capture width and `MAX_SNAPSHOT_WORDS` in
`tests/test_bidir_rx_raw.py` are still sized for investigation (generous
margin for finding period/alignment), not necessarily final production
values - revisit if/when integrating into the driver proper.

### Confirmation sweep: full throttle range, gradual ramp (2026-08-23)

A longer, wider-coverage run (`tests/test_bidir_rx_sweep.py`) to confirm
the 17/17 result generalizes beyond the original short test, not just a
fluke of one throttle range. 12 throttle levels, 50-600 in steps of 50
(this ESC's power protection trips on sharp increases, not gradual ones -
50-unit steps avoid that; 600 is this run's deliberate ceiling), 5 seconds
held at each level, ~5 RX snapshots per level. Channel 1 only (motor +
prop mounted); channels 2-4 idle at 0 throughout, as in every RX test so
far.

**Result: 59/59 captures CRC-valid (100%).** RPM rises monotonically and
close to linearly across the entire range. (eRPM is the verified quantity;
the RPM figures below divide by `MOTOR_POLES = 14`, unverified for this
motor/ESC - see "eRPM Decoding" above. That divisor is a constant scale
factor, so it doesn't affect monotonicity, per-level spread, or any other
shape-of-the-data conclusion drawn here - only the absolute RPM labels carry
the uncertainty.)

| Throttle | Mean RPM (settled, excl. first snapshot per level) | Range |
|---------:|----------------------------------------------------:|:------|
| 50  | 1,395  | 761-2,402 (still spinning up from a cold start - see note below) |
| 100 | 3,092  | 3,088-3,097 |
| 150 | 4,895  | 4,881-4,915 |
| 200 | 6,940  | 6,912-6,957 |
| 250 | 8,684  | 8,676-8,693 |
| 300 | 10,857 | 10,850-10,877 |
| 350 | 12,727 | 12,680-12,793 |
| 400 | 14,591 | 14,479-14,778 |
| 450 | 16,644 | 16,547-16,676 |
| 500 | 18,493 | 18,354-18,593 |
| 550 | 20,481 | 20,408-20,555 |
| 600 | 22,468 | 22,380-22,616 |

Every level from 100 upward is tightly clustered (well under 2% spread),
consistent with a real, stable motor speed at equilibrium - not noise.
Throttle 50's wide range (761-2,402 RPM even after dropping the first
snapshot) is expected, not a decode problem: it's the very first level
from a cold start (arm -> 0 -> 50 directly), so the motor was still
spinning up through the entire 5s window rather than sitting at a settled
speed; later levels only step by 50 from an already-spinning state and
settle within the window. mantissa/exponent also behaved exactly as
expected throughout: exponent decreases as commutation period shrinks
with rising RPM, consistent with the eRPM encoding's floating-point-like
`mantissa << exponent` scheme.

This confirms Phase 3's result holds across the ESC's full usable throttle
range, not just the original short 100/200/300 test.

### Confirmation sweep: up/down ramp with hysteresis check (2026-08-24)

A further run addressing two things the previous sweep didn't cover:
throttle 50 was too low to be a usable base (audibly rough/unstable spin,
confirmed by ear, not just the wide RPM range above), and the previous
sweep never tested ramping back down. Same script
(`tests/test_bidir_rx_sweep.py`), updated profile: 60 as a brief 3s
post-arm settle throttle (avoids the throttle-50 roughness), then
measured steps starting at 100, up in 50-unit increments to 600, then back
down in 100-unit increments to 100 (larger steps are safe on the way down
- this ESC's power protection reacts to sharp *increases*, not decreases).
Each measured step held 10s+ (up from 5s), ~9-10 snapshots/level after
dropping the first (settling-transient) snapshot per level. Channel 1
only, channels 2-4 idle at 0, run twice - an earlier run of this profile
was discarded unused when the bench was power-cycled mid-session, and
rerun fresh afterward (results below are from that rerun).

**Result: 162/162 captures CRC-valid (100%).** Up-ramp and down-ramp
values at the same throttle level agree to within ~1%, i.e. no meaningful
hysteresis. (Same `MOTOR_POLES = 14` caveat as the previous sweep applies to
every RPM figure below - a constant scale factor that doesn't affect the
CRC-valid rate, monotonicity, or the up/down agreement being reported here.)

| Throttle | Up RPM | Down RPM | Diff |
|---------:|-------:|---------:|-----:|
| 100 | 3,092  | 3,083  | 0.3% |
| 200 | 6,965  | 6,980  | 0.2% |
| 300 | 10,881 | 10,875 | 0.1% |
| 400 | 14,790 | 14,847 | 0.4% |
| 500 | 18,647 | 18,674 | 0.1% |

Full up-ramp (the 50-unit up-ramp step reaches levels - 150, 250, 350, 450,
550 - that the 100-unit down-ramp never revisits, so they only have one
direction's data):

| Throttle | Mean RPM (settled) | Range |
|---------:|--------------------:|:------|
| 60  | 1,223  | single sample, transitional settle throttle only |
| 100 | 3,092  | 3,088-3,097 |
| 150 | 4,910  | 4,904-4,915 |
| 200 | 6,965  | 6,935-7,003 |
| 250 | 8,717  | 8,658-8,764 |
| 300 | 10,881 | 10,850-10,933 |
| 350 | 12,823 | 12,755-12,909 |
| 400 | 14,790 | 14,728-14,881 |
| 450 | 16,738 | 16,676-16,807 |
| 500 | 18,647 | 18,553-18,715 |
| 550 | 20,704 | 20,604-20,855 |
| 600 | 22,576 | 22,438-22,676 |

This is consistent with the previous sweep's numbers (e.g. throttle 300:
10,857 there vs 10,881/10,875 here), confirming the decode pipeline and
the ESC's throttle->RPM behavior are both repeatable across separate runs
and separate days, not a one-off result. Base throttle 60 spun smoothly
(no more roughness complaint), replacing 50 as the recommended post-arm
settle value for future test scripts.

### Unpaced continuous send/drain characterization (2026-09-06)

A new characterization harness drives one bidirectional channel (channel 1,
DShot300) directly, bypassing every layer above the raw driver: after the
usual 3-second back-to-back arm, it holds throttle 60 (this document's
confirmed smooth post-arm value) and sends commands with no
application-level pacing between them - each `send_throttle_command()` call
blocks only when the 4-word TX FIFO itself is full, and the RX FIFO is
drained completely after every send.

10,000 frames were sent this way over about 4.9 seconds, an achieved rate of
roughly 2,000 frames/second - well under DShot300's own ~18.75kHz wire-rate
ceiling (53.3us/frame). This run's own per-word RX draining in Python is
what limits it to 2kHz, not FIFO backpressure or PIO timing; a genuinely
close-to-wire-rate figure comes from the starvation run below instead. Every
one of the 10,000 frames produced a structurally well-formed 4-word capture
(the reply's leading marker bit correctly read as 0): zero misaligned or
partial groups, and the RX FIFO's occupancy never stalled.

A 298-capture sample of these was decoded offline with the full GCR/CRC
pipeline: 240/298 (80.5%) were CRC-valid - markedly lower than every
previous measurement in this document, all of which held a settled throttle
at a much slower, application-paced rate (on the order of 500 records/second
or less). The 58 failures in that sample were not spread evenly across the
run: 40 of them fall in the first ~60ms immediately after the transition
from arming to held throttle, consistent with the kind of brief
motor-settling transient this document has already seen at other throttle
transitions. A second, separate cluster of about 18 failures appears later,
around the 3.5-4.3 second mark of an otherwise clean run, with no
corresponding throttle change and no unusual decoded eRPM (still
7,570-7,620, consistent with the surrounding, fully valid windows). Every
other sampled window in the run was 100% CRC-valid. This second cluster's
cause was not identified by this run.

The headline result: passing every structural check this driver currently
performs on a capture is not the same as that capture carrying a real,
correctly-decodable reply. Around a fifth of the captures in this run looked
complete and correctly marked and still failed CRC, concentrated in short
windows rather than spread uniformly through the run.

### RX-starvation and recovery characterization (2026-09-06)

The same harness, same physical setup, but with the RX FIFO deliberately
left undrained for 5ms every 200ms while throttle commands kept sending the
whole time (only the receiving side paused) - 22 such cycles over one run.

Every one of the 22 cycles reached this driver's own recovery signal (three
consecutive structurally well-formed captures after resuming draining), and
did so in exactly 4 frames every single time - zero variance, and no
state-machine restart was ever required. But a sample of the first captures
taken immediately after each of the 22 resumes (88 captures total) was
decoded offline: 0/88 (0%) were CRC-valid. Every one had real signal
transitions, not a dead line, so this is not simply "no reply arrived" - the
reply that comes back right after a stall reliably fails CRC in this sample,
despite this driver's own recovery detector reporting success on every
single cycle. The fact that the recovery time was exactly 4 frames on all 22
cycles regardless of where in the frame cycle the stall happened to land is
itself telling: an invariant like that says the detector is measuring
something mechanical about how quickly the receiving state machine
resynchronizes structurally, not whether the content it captures can be
trusted.

Frames provably lost while undrained - bounded by the 4-word FIFO's
capacity to hold at most one capture's worth during a stall - totalled
1,269 across the 22 cycles, an average of roughly 58 per 5ms window. That
implies the driver and ESC together reach on the order of 11-12kHz once the
receiving side's own Python-level polling overhead is taken out of the
loop, well above the ~2,000 frames/second the unpaced run above otherwise
achieved. The overall CRC-valid rate sampled during this run's held-throttle
phase (31.0% on 271 samples) was markedly worse than the undisturbed run
above (80.5%), but the two aren't a clean comparison: this run's samples
were taken throughout a period carrying 22 separate 5ms interruptions
spaced every 200ms, not one undisturbed hold.

Taken together with the run above, the conclusion for this driver is the
same either way: any telemetry-validity signal exposed upward from this
layer needs to be gated on a real CRC check, not a structural one. A
capture that is complete, correctly marked, and even repeatedly "recovered"
by the driver's own detector is not, on its own, sufficient evidence that
the reply it carries is genuine.

### Implications for the RX-synchronization decision (2026-09-06)

The two characterization runs above change what the open synchronization
decision actually needs to solve.

The "Design candidates for RX synchronization" section above was written
against a specific failure model: TX racing ahead of RX and the driver
losing track of which reply belongs to which command. Both runs above show
that, at least on this ESC and at the rates exercised, that specific
failure mode is largely absent already - the unpaced run produced a
structurally correct, correctly-paired capture for all 10,000 frames sent,
and even deliberately starving the RX side for 5ms at a time never produced
a misaligned or partial capture once draining resumed. The relative-IRQ
addressing and clear-before-wait behavior already in this driver appear to
be doing their job: association between a command and its reply's capture
is not, on this evidence, the primary open risk any more.

What both runs show instead is a *content* problem that neither of the
listed design candidates was written to address: a capture can be complete,
correctly framed, and paired with the right command, and still carry a
reply that fails CRC - reliably so in the frames immediately following any
disruption to steady-state timing (a throttle transition, or a resumed
RX drain after a stall), and occasionally elsewhere for reasons this data
doesn't explain. Keeping TX from getting ahead of RX (the lockstep
candidate) or stamping captures with a sequence number (the epoch-tracking
candidate) would not, by itself, fix a capture that is already correctly
identified but wrong in its content - both of those approaches solve a
bookkeeping problem this driver does not currently appear to have. Even
replacing the whole handshake with a single-SM design would not obviously
avoid this: the observed corruption windows follow timing disruptions, not
handshake identity confusion.

The practical implication, independent of which synchronization approach is
eventually chosen: nothing in this driver can currently tell a good capture
from a bad one without the full offline GCR/CRC decode this project still
only runs on a PC. Whatever design is chosen for the open decision above
should be paired with an on-device validity check against the real CRC, not
just the structural marker-bit check this characterization work used - a
capture that merely looks well-formed is demonstrably not enough evidence
that its content can be trusted.

## References

- [Brushless Whoop - Bidirectional DShot](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
- [Betaflight - DShot RPM Filtering](https://www.betaflight.com/docs/wiki/guides/current/DSHOT-RPM-Filtering)
- [Bluejay Firmware](https://github.com/mathiasvr/bluejay)
- [Bluejay Configurator](https://github.com/mathiasvr/bluejay-configurator)
- [BLHeli_S to Bluejay Flashing Guide](https://github.com/mathiasvr/bluejay/wiki/Flashing)
- [AM32 Firmware](https://github.com/am32-firmware/AM32) - `Src/dshot.c`, `Src/main.c`, `Src/signal.c`
- [Betaflight Pico/RP2350 DShot PR #14618](https://github.com/betaflight/betaflight/pull/14618/files) - `src/platform/PICO/dshot.pio`, `dshot_bidir_pico.c`, `dshot_pico.c`
- DShot Protocol Specification: `specification/DSHOT_PROTOCOL.md`
