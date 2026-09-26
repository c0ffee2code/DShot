# ADR-002: Bidirectional DShot Implementation

**Status:** Deferred — superseded by the 2026-08 implementation work below (RX
capture + eRPM decode verified on hardware, 100% CRC-valid across two
independent confirmation sweeps). The driver now exposes bidirectional
telemetry through `BidirectionalDShot` and `MotorGroup` (see
[ADR-005](ADR-005-bidirectional-telemetry-data-flow.md)); the formal flip to
Accepted is still pending, as decoding non-eRPM frames (extended telemetry, the
stopped-motor value) and the motor pole count are still open. The
"Implementation Update (2026-08-23)" section and everything below it hold
the real investigation history, including dead ends - each subsection's own
heading/status line says whether it's verified or superseded, so read those
markers rather than assuming everything under "Implementation Update" is
final. Sections above "Implementation Update" are this ADR's original
pre-implementation analysis and contain some estimates/assumptions later
found inaccurate (flagged inline where relevant). Test scripts and tooling
named in the dated sections below (`test_bidir_rx_*.py`,
`decode_bidir_capture.py`'s early forms and similar) were retired once their
findings were recorded here; read those names as provenance, not as things
you can still run.
**Date:** 2026-02-01 (original analysis); implementation findings added 2026-08-23/24
**Context:** Exploring ESC telemetry via bidirectional DShot for the test bench

## Verification status

What has and has not been shown on hardware, kept current as gates pass. Each
row's evidence is in the dated sections below.

| Layer | Status |
|---|---|
| Inverted TX (ESC detects bidirectional mode) | Verified |
| Physical RX capture | Verified |
| GCR decode | Verified |
| CRC validation (inverted polarity only) | Verified |
| eRPM value from a CRC-valid eRPM frame | Verified; mechanical RPM is not - the motor pole count is an unverified constant |
| Continuous RX synchronization | Verified in steady operation and after deliberate FIFO stalls (no lost pairing); corruption after a stall under the production command loop not re-tested |
| RX FIFO management | 8 words deep (joined), drained on every command-loop tick before the commands are sent, capped per call (ADR-005); two bidirectional motors verified 100% CRC-valid at both speeds, deliberate consumer stalls not re-tested |
| Two or more bidirectional motors at once | Channels 1 and 3 (separate blocks, and sharing one block) verified at 100% CRC-valid; channel 2 replies; channel 4 fails and is parked, cause unknown; four bidirectional motors through the facade not verified |
| Public API integration | Implemented (`BidirectionalDShot`, `MotorGroup`); one bidirectional motor verified through the facade, several not yet |
| DShot600 bidirectional | Verified for short, settled-throttle captures; no saturation or stall-recovery run |
| Non-eRPM frames (extended telemetry, stopped-motor value) | Not handled |
| Telemetry loss and health tracking | Not implemented |
| Post-disarm line state (bidirectional) | Verified: `stop()` drives the line low instead of releasing it; the ESC recovers with no reset; re-arm works |

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

*Superseded: this project supports exactly two ESC firmware families -
BLHeli_S (unidirectional only, in its stock form) and AM32 (bidirectional).
No Bluejay flash is planned, and the AM32 ESC that arrived on the bench made
this recommendation moot. Kept as the original analysis.*

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
| Update rate | Full speed | ~50% (wait for response) - a pre-implementation estimate, never measured; the driver's loop rate is set by the application, not by waiting for the reply |

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
"Implementation Update" below for the source-verification method.
`specification/DSHOT_PROTOCOL.md` never carried a symbol table of its own, and
now states that the encode table matches AM32's exactly):

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

**Precision update (2026-09-12):** across four independent `rx_speed`
settings tested for the fixed-ratio retune (see "Fixed-ratio RX sampling
retune" below), the measured real reply bitrate is consistently ~388,000
bps (bit period ~2.58µs) - about 3.4% above the 375kbit/s nominal 5/4
figure, confirmed independently at each setting rather than being one
measurement's rounding artifact. `driver/gcr_decode.py`'s
`estimate_bit_period_fixed` (see below) is tuned against this measured
value, not the nominal 375kbit/s figure.

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

# 3. Extract CRC + data fields from the reassembled 16-bit number, and check
#    the CRC before trusting anything else. AM32 sends the INVERTED polarity
#    (every CRC-valid capture from real hardware has been inverted, none
#    plain), so the expected value is the complement of the usual nibble XOR.
crc = dshot_full_number & 0x0F
data12 = (dshot_full_number >> 4) & 0xFFF
if crc != (~(data12 ^ (data12 >> 4) ^ (data12 >> 8))) & 0x0F:
    return None                    # not a valid reply - discard it
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

### State machines and instruction memory (2026-09-20)

A PIO block has two separate resources, and it helps to keep them apart:

- **4 state machines**, the workers. Each has its own clock divider, FIFOs and
  registers, and runs one program at a time. The chip has 3 blocks, so 12.
- **32 instruction slots**, shared by the 4 state machines of that block. A
  program takes its slots once per block, however many state machines run it: two
  state machines running the same program use the same copy. (Confirmed on hardware
  on 2026-08-30: two bidirectional pairs on one block, each pair needing 23 slots,
  ran and answered - two separate copies would have been 46 slots and would not
  have loaded.)

What the programs take:

| Program | Slots |
|---|---|
| `dshot` (unidirectional transmit) | 4 |
| `dshot_bidir_tx` (bidirectional transmit) | 13 |
| `dshot_bidir_rx` (receive, oversampling) | 10 |

A bidirectional motor is one transmit and one receive state machine on the same
block, the receiver one id above the transmitter (see the synchronisation
sections below), so a pair needs 23 slots and 2 state machines, and every further
pair on that block adds 2 state machines and no slots. Four bidirectional motors,
one pair each:

| Block | State machines | Programs loaded | Slots used |
|---|---|---|---|
| PIO0 | sm0 TX + sm1 RX (motor 1), sm2 TX + sm3 RX (motor 2) | `dshot_bidir_tx` + `dshot_bidir_rx` | 23 of 32 |
| PIO1 | sm4 TX + sm5 RX (motor 3), sm6 TX + sm7 RX (motor 4) | `dshot_bidir_tx` + `dshot_bidir_rx` | 23 of 32 |
| PIO2 | free (sm8 to sm11) | none | 0 of 32 |

That is 8 of the 12 state machines. The bench's present layout (channels 1 and 3
bidirectional, 2 and 4 unidirectional) puts a bidirectional pair and one
unidirectional state machine on each of PIO0 and PIO1: 23 + 4 = 27 slots. Each
pair's two state machines have their own synchronisation flag (see the per-pair
sections below), so pairs on one block do not interfere.

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
measurement later replaced: the real delay before RX starts listening is about
14 RX cycles - roughly 4.15µs at DSHOT300's current RX clock (~4.7µs was the
figure at the 3MHz clock of the superseded design, see "Implementation
Update"'s RX redesign section) - not 30µs, and it is only a lower bound that
keeps RX from re-triggering on TX's own tail, not the reply's start. The
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
3. Integration with `MotorGroup` facade

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
bench and already proven for unidirectional DShot300 (the unidirectional group scenario `smoke_unidirectional.json`, formerly
`tests/test_slow_spin.py`, and README's "Verified Parameters"). AM32 supports bidirectional DShot natively -
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
  GCR symbol table this ADR originally carried - they agree on 7 of 16
  entries and diverge after that. That original table was wrong (or at least
  not what this firmware implements) and could not be trusted for a decoder.
  Confirmed independently: AM32's table matches betaflight's own `gcrs[]`
  reverse-lookup table exactly (see below), so the AM32-derived table is the
  one to build a decoder against.

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
- `MotorGroup`/`DShotPIO`'s `rx_resync()` (`StateMachine.restart()`)
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

### Design candidates for RX synchronization (decided: Option A' was built and is what ships)

*The decision this heading once left open has been made: the dual-SM IRQ
handshake below is the implemented design, and the transaction model built on
it is recorded in [ADR-005](ADR-005-bidirectional-telemetry-data-flow.md).
Option B is not being pursued. The text below is the original weighing.*

**Option A' - keep dual-SM, add a PIO-to-PIO IRQ handshake.** TX program
(unchanged from the Phase-2-verified waveform) raises a PIO IRQ right after
releasing the pin each frame; RX program waits on that IRQ, then a fixed
~25-30µs delay, then listens with 3x oversampling per bit. Both state
machines already sit on the same PIO block (required since they share a
GPIO), which PIO IRQ signalling needs anyway.
- Pro: the exact TX waveform already arm-verified on this ESC never
  changes - zero new risk to arming, which was thought fragile at the time
  this option was weighed (a 500ms arm window was believed not enough,
  requiring 3000ms). That fragility claim came from test runs made before the
  board-reset problem described below was found, so it is unconfirmed rather
  than disproven: a later re-test (2026-09-12) showed the ESC replying with
  telemetry at arm windows of 300-1000ms, but a telemetry reply only shows
  the ESC is armed, not that the motor runs, and the runs that checked the
  reported eRPM saw the motor at rest at those windows in some runs and
  spinning in others; the cause was not established. The option's pro holds
  for the reason argued - an unchanged, already-verified TX waveform carries
  zero new risk.
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
   right and should not be touched. *(Disproven later, and not a standing
   instruction: the real bit period measured a few percent off the 5/4-derived
   figure, and that small mismatch, accumulating over the frame, is what
   defeated this design - see "RX redesign: unslotted dense oversampling".)*
   The predelay was shortened to ~4.7µs at that design's 3MHz clock
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

**Status: Phase 3 verified. Not yet integrated into `MotorGroup` or
`DShotPIO`'s public API** (Phase 4/5 per the original plan) - the decode
pipeline currently lives only in the offline `scripts/decode_bidir_capture.py`
tool. *(As of 2026-09: the decode now also runs on the device in
`driver/gcr_decode.py`, and telemetry is exposed through `BidirectionalDShot`
and `MotorGroup` - see ADR-005. This paragraph describes the state on
2026-08-23.)* The 128-sample/4-word capture width and `MAX_SNAPSHOT_WORDS` in
`tests/test_bidir_rx_raw.py` (since removed) are still sized for investigation (generous
margin for finding period/alignment), not necessarily final production
values - revisit if/when integrating into the driver proper.

### Confirmation sweep: full throttle range, gradual ramp (2026-08-23)

A longer, wider-coverage run (`tests/test_bidir_rx_sweep.py`, since removed) to confirm
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
(`tests/test_bidir_rx_sweep.py`, since removed), updated profile: 60 as a brief 3s
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
state-machine restart was ever required. A first look at a sample of the
captures taken immediately after each of the 22 resumes (88 total, decoded
offline) found 0/88 (0%) CRC-valid - but this run's held-throttle phase as a
whole was already unusually poor (31.0% CRC-valid on 271 samples, well
below the undisturbed run above's 80.5%), so the first question is whether
"right after a resume" is actually worse than the rest of this same run, or
just as bad as everything else in it.

It is measurably worse. For each of the 22 resumes, comparing the four
post-resume samples against the surrounding saturation-phase samples taken
within 100ms of that same resume (7-8 samples each, drawn from the same
run, same nearby stretch of time) shows those local neighborhoods averaging
19.2% CRC-valid, ranging from 0% up to 57.1% depending on the cycle - so
this run's baseline quality varied a lot from moment to moment, but was
rarely all bad. 16 of the 22 neighborhoods had a nonzero local rate. Against
that backdrop, all 22 post-resume samples still coming back 0/4 is far too
consistent to be explained as an unlucky draw from an already-poor
baseline - if post-resume captures decoded the same way their immediate
neighbors did, getting exactly zero across all 16 of the nonzero-baseline
cycles would be a roughly one-in-a-billion coincidence. So there are two
separate effects in this data, not one: something about running repeated
5ms drain stalls depresses this driver's overall decode quality for the
whole session (the 19.2% local average and the 31.0% run-wide average are
both far below the clean run's 80.5%, and neither is explained by this
data), and on top of that, the captures landing immediately after a resume
are reliably worse still than their own already-degraded neighborhood. The
exactly-4-frames recovery timing, with zero variance across all 22 cycles,
doesn't help distinguish between these - it says the driver's structural
detector is timing something mechanical about how fast the state machine
resynchronizes, not that a real recovery happened, but it's neutral on
which of the two effects is at play.

Frames provably lost while undrained - bounded by the 4-word FIFO's
capacity to hold at most one capture's worth during a stall - totalled
1,269 across the 22 cycles, an average of roughly 58 per 5ms window. That
implies the driver and ESC together reach on the order of 11-12kHz once the
receiving side's own Python-level polling overhead is taken out of the
loop, well above the ~2,000 frames/second the unpaced run above otherwise
achieved.

Taken together with the run above, the standing conclusion is unchanged:
any telemetry-validity signal exposed upward from this layer needs to be
gated on a real CRC check, not a structural one - a capture that is
complete, correctly marked, and even repeatedly "recovered" by the driver's
own detector is not, on its own, sufficient evidence that the reply it
carries is genuine. But this run's own repeated-stall design left every
resume without a clean, undisturbed local baseline to compare against, so
it cannot separate "resuming a stall corrupts the next few captures" from
"repeated stalls degrade this run's decode quality generally, and resuming
is no different from any other moment in it." Isolating that needs a rerun
with stalls spaced far enough apart (seconds, not 200ms) that each resume
has an undisturbed neighborhood on both sides to compare against - not yet
done.

### Implications for the RX-synchronization decision (2026-09-06)

*The decision discussed here has since been made: keep the dual-SM handshake,
drain the RX FIFO on every command-loop tick, decode elsewhere, and gate
captures on the CRC - see [ADR-005](ADR-005-bidirectional-telemetry-data-flow.md).*

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
reply that fails CRC. The clean unpaced run above ties this to short,
disruption-adjacent windows (a settling transient right after the throttle
transition, and one unexplained mid-run window) rather than a steady rate
effect. The starvation run's own repeated-stall design couldn't cleanly
separate "a resume specifically corrupts the next few captures" from
"repeated stalls degrade this run's decode quality generally" (see that
section above) - so right now only the *first* run's disruption-adjacent
pattern is solid evidence that timing disruptions specifically matter, not
the second run's post-resume number on its own. Either way, keeping TX from
getting ahead of RX (the lockstep candidate) or stamping captures with a
sequence number (the epoch-tracking candidate) would not, by itself, fix a
capture that is already correctly identified but wrong in its content -
both of those approaches solve a
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

### Confirmation reruns (2026-09-07)

Two follow-up runs, aimed at the two open questions the sections above left
unanswered.

**Repeating the clean, undisturbed run unchanged** reproduced both the
overall result and its odd shape: 78.5% CRC-valid this time (298 samples,
close to the earlier 80.5%), with the same two-part failure pattern -
44 of the run's 64 failures land in the first half-second after the
throttle transition (consistent with a settling transient), then a clean
stretch with zero failures from 0.5s to 3.5s, then a second cluster of 20
failures from 3.5s to the run's end around 5s. That second cluster now
showing up in the same few-second window in two independent runs of
identical code makes chance a poor explanation. One plausible mechanism,
not yet confirmed: this window is late enough in a several-thousand-frame
run that MicroPython's own automatic garbage collection - not the harness's
own accounting, which does zero file I/O during this phase, but the
interpreter reclaiming memory from the small objects this loop allocates
every iteration - could plausibly fall in this window and briefly stall the
CPU long enough to disturb the receiving state machine's timing-sensitive
listen window. This is a hypothesis worth testing directly (forcing a
collection on a schedule and seeing whether the failure window moves with
it), not yet something this data confirms on its own.

**Rerunning the starvation scenario with cycles spaced 3 seconds apart
instead of 200 milliseconds** (3 cycles instead of 22, to keep the run
short enough for the Pico's memory - the first attempt at a much longer,
more heavily sampled version of this rerun ran out of memory partway
through and had to be scaled back) removes the confound the original run
left open. This time the surrounding baseline recovered to a healthy 80.8%
CRC-valid overall - in line with the clean run above, not the previous
run's depressed 31% - confirming that the earlier run's poor baseline was
specific to stalling every 200ms, not something starvation does in
general. Within that healthy baseline, each of the 3 resumes still has a
fully clean, 100%-valid local neighborhood on both sides (10/10 sampled
captures within 300ms) - and every one of the 3 resumes still produced
0/4 CRC-valid immediately after. With the confound removed, this is now a
clean, unambiguous result: resuming the RX side after a drain stall
reliably corrupts the next several captures, on a baseline that is
otherwise perfectly healthy.

Put together, both open questions from the sections above are answered:
the two-part failure pattern in the clean run reproduces and is not a
one-off ("done" for that half of the earlier open item), and the
starvation run's core finding survives with the confound removed - a
resume-adjacent corruption effect exists independent of, and in addition
to, whatever caused the earlier run's general degradation. The
implication drawn above stands on firmer ground now: any validity signal
this driver exposes needs to gate on a real CRC check, because a
structurally perfect capture taken right after a stall is, reliably, not
a valid one.

### On-device telemetry validity check: real GCR/CRC decode replaces the structural check (2026-09-08/09)

*Superseded in part (2026-09): the `poll_telemetry()` method described below
bundled draining and decoding and has been removed. `BidirectionalDShot` now
drains in `drain_rx()`, hands out the latest capture via `latest_capture()`,
and decodes on request via `decode_capture()` (see ADR-005); the decode
algorithm and its timing findings below are unchanged. The remark that no
telemetry consumer exists yet is also out of date: `MotorGroup`
integration has since been done.*

The two characterization sections above ("Unpaced continuous send/drain
characterization" and "RX-starvation and recovery characterization")
proved this driver's cheap structural check ("does the capture's first
sample bit read 0") is not a trustworthy validity signal - it passed
every capture in a starvation-recovery run while the real, decoded
CRC-valid rate in the same window was 0%. The fix: port the real GCR
decode + 4-bit CRC check (`scripts/dshot_bidir_decode.py`, the PC-side
reference tool this project already had) onto the Pico itself, as a new
module `driver/gcr_decode.py`, and wire it into `DShotPIO` via a new
`poll_telemetry()` method that drains a completed 4-word capture and
returns the decoded result (`crc_ok` is the real signal now, not the old
structural check).

The port is kept honest by a permanent offline regression check,
`scripts/verify_gcr_decode_port.py`, which diffs the on-device port
against the PC reference on every real capture ever pulled in this
project (2,040 groups across all sessions) - 0 real mismatches. Two
deliberate differences from the reference are pinned from that same
data rather than guessed: CRC polarity (the port accepts only
`inverted` - 714/714, later 798/798, real CRC-valid captures pulled from
hardware have been inverted, 0 plain, matching AM32's own firmware
source) and the bit-period search range (below).

**Correction (2026-09-12):** the "2,040 groups" figure above was wrong.
`verify_gcr_decode_port.py` hardcoded the older stress-harness record
format (`<IBBB4I`); every session recorded via the newer main-scenario-
capture format (`<I4H16I`, `tests/harness/bidir_capture_sink.py`) was
silently misparsed into garbage records that happened to never satisfy
`crc_kind=="inverted"`, so they were bucketed as harmless "expected
divergences" rather than flagged - the check never failed, but for those
sessions it was also not actually checking anything, for the entire time
this section describes. Fixed via a shared, format-aware loader
(`scripts/capture_session.py`) that reads each session's own
`record_fmt`/`bidir_motor_indices` from `meta.txt`. Re-run against the
same historical sessions this section originally covered: 377,721 real
groups checked, 0 real mismatches - the underlying decode algorithm was
never wrong, only this regression check's coverage of it was.

**On-device timing was measured, not assumed, and drove two rounds of
real optimization:**

1. First port, unoptimized, measured at **174-430ms per decode**
   on-device (MicroPython on RP2350), an order of magnitude
   worse than a first guess from the algorithm's raw operation count
   would suggest, and confirming that interpreter overhead - not the
   RP2350's genuine hardware FPU, which is real and ~6x faster than the
   RP2040's software float emulation - dominates this loop's cost.
2. Stage-by-stage timing (bracketing each of `raw_samples`,
   `find_edges`, `estimate_bit_period`, `reconstruct_bits` separately)
   isolated `estimate_bit_period`'s 250-candidate brute-force sweep as
   90-96% of total cost, dwarfing everything else.
3. Two safe, verified optimizations were kept: `find_edges` now carries
   each edge's sample index so `reconstruct_bits` reads the transitioned
   value directly (O(1)) instead of `value_at_cycle`'s old O(128) linear
   scan per bit; and the sweep's inner loop reuses one division
   (`q = g/p`) for both the rounded multiple and the residual instead of
   computing a second, redundant division. Net: 174-430ms -> 115-288ms.
4. A two-phase coarse (0.5-step) then fine (0.04-step, +-0.5 window)
   version of the sweep was tried next, aiming at the sweep's O(250)
   candidate count directly. **Reverted**: checked against
   `verify_gcr_decode_port.py`'s full 2,040-group diff, it gave the
   wrong period for 511/2,040 real captures - the residual-vs-period
   surface isn't well-behaved enough at 0.5-step granularity for the
   coarse pass to reliably land in the right neighborhood, and the fine
   pass then has no way to recover from a wrong neighborhood.
5. Rather than guess a better search strategy, the real data was
   tallied directly: `period_cycles` across all 798 CRC-valid captures
   pulled from hardware so far clusters in [9.64, 10.52] (mean 10.30,
   std 0.13) - nowhere near the original algorithm's full 6-16 cycle
   search range. A bare fixed constant at the center of that band
   reproduces the full sweep's answer exactly on all 798 valid captures
   (0 disagreements) - the true period on this rig is effectively
   constant, not something that needs discovering fresh on every reply.
   Shipped a middle ground instead of the bare constant: a narrowed
   search range (8.5-12.0 cycles, grid-aligned to the reference sweep's
   own float sequence so `verify_gcr_decode_port.py`'s float-tolerance
   check compares like with like) that keeps real margin against period
   drift on different hardware or thermal conditions, while cutting the
   candidate count from 250 to ~35.
6. Final, deployed, on-device numbers: **42-103ms per decode**, roughly
   a 4x improvement over the initial port, with
   `estimate_bit_period` still 79-88% of the total (down from 90-96%,
   but still dominant - a bare constant would remove nearly all of the
   remaining cost, at the price of the drift margin above).

**Where this leaves the scoping question, decided rather than left
open:** the pipeline's unavoidable floor (`raw_samples` + `find_edges`,
independent of any period-search strategy) is roughly 10ms, so ~100
decodes/sec is the ceiling for this pipeline shape no matter how far the
sweep itself is optimized. This project's own regressions produce
500-2,000 replies/sec, so per-reply decode was never on the table -
every viable option lands in the same regime: a periodically sampled
validity signal somewhere between the ~10-24/sec now shipped and a
theoretical ~100/sec. With no telemetry consumer built yet
(`MotorGroup` integration remains deferred, as before), nothing
currently needs more than what's shipped now, so this is where the
optimization work stops - not because a faster version isn't possible
(the bare-constant measurement proves one is, exactly), but because
nothing yet exists that would notice the difference.

### Fixed-ratio RX sampling retune; mpremote board-reset corruption discovered (2026-09-12)

Following up on the previous section's finding that a bare fixed constant
reproduces the brute-force sweep exactly (0 disagreements on 798 valid
captures at the old, non-integer ~10.3-cycle rate), a plan was made to
retune `rx_speed` itself to a clean value and verify a genuine
Betaflight-style bare divisor (no search at all) on real hardware, testing
candidate integer ratios K in {8, 9, 10, 11} (`rx_speed = K * 375_000`,
the nominal 5/4 DSHOT300 reply rate) against two constraints that trade
off in opposite directions as K rises: raw sample density (`dshot_bidir_rx`
samples every fixed 2 PIO cycles, so density = period_cycles/2, against an
established "5-6+ samples/bit" comfort floor) and capture-window margin
(the fixed 128-sample/~262-cycle window must exceed the real ~21-bit frame
duration with room to spare - DSHOT1200@16MHz previously failed this
margin completely, 0/8 CRC-valid via truncation).

**A serious false lead, root-caused and fixed.** The first hardware round
(2026-09-10) produced deeply confusing results: one candidate worked once
then failed identically on rerun, every other candidate failed outright,
and even the untouched, historically-proven original rate failed on two
fresh reruns - pointing initially at a hardware/bench fault. Extensive
investigation (channel-swap tests, a controlled `git stash`-based A/B test
against the exact last-committed code, physical bench inspection)
eventually isolated the real cause: **`mpremote run` does not reset the
board between invocations** - it execs a script into the same live
MicroPython VM the previous invocation left behind, and a hardware test
run immediately following another one (same or different channel, clean
completion or an uncaught exception - none of it mattered) reliably
corrupted the next run's RX capture: TX still went out and the ESC still
armed normally, but every captured word came back zero, perfectly
mimicking a dead ESC or wiring fault. A hard reset before the run made it
succeed every time observed; skipping the reset reliably failed.
`scripts/deploy.py` now resets the board (with a ~3s settle for USB
re-enumeration) before every run - see its own docstring for the
confirming sequence of back-to-back test results. **This retroactively
voids the entire 2026-09-10 K-candidate dataset** (collected as a rapid,
unreset sequence of `mpremote run` invocations - exactly the corrupting
condition) and the "channel 1 might be uniquely faulty" conclusion briefly
drawn from it: a clean rerun on 2026-09-12 confirmed both channel 1 and
channel 3 are fully healthy hardware (100% real replies each, motor
visibly spinning), with no wiring or ESC fault ever actually present. This
is a general `mpremote`/RP2 MicroPython gotcha worth carrying to any
project using `mpremote run` for iterative hardware testing, not something
specific to this codebase.

**Clean K-candidate retest (2026-09-12, channel 1, with the reset fix in
place):**

| K | rx_speed | CRC-valid | mean period_cycles | std | samples/bit |
|---|---|---|---|---|---|
| 8  | 3,000,000 | 97.8% (4601/4705) | 7.7406  | 0.0684 | 3.87 |
| 9  | 3,375,000 | 100% (4743/4743)  | 8.7069  | 0.0477 | 4.35 |
| 10 | 3,750,000 | 100% (4738/4738)  | 9.6061  | 0.0960 | 4.80 |
| 11 | 4,125,000 | 100% (4656/4656)  | 10.6530 | 0.0561 | 5.33 |

K=8's measurably lower validity (vs. 100% for the other three) is
consistent with it being genuinely too thin on samples/bit, below the
design's stated floor. The other three all cleared 100% - a more forgiving
real result than the "5-6+ samples/bit" floor assumed, since only K=11
actually reaches it. All four measurements agree the real reply bit period
is ~2.56-2.58µs regardless of K (a useful cross-check - see the bitrate
correction above for the ~388,000bps figure this implies).

**Decision: K=9** (`rx_speed=3_375_000`), bare fixed divisor
(`expected_ratio=8.7069`, `ratio_tolerance=0.0`) - tightest spread of the
three fully-valid candidates (std=0.0477, comfortably under the 0.5-cycle
rounding boundary a bare divisor needs) and better capture-window margin
than K=10/K=11 (K=11's margin, while still positive, sits closest of the
three to the kind of truncation risk that sank DSHOT1200@16MHz). DSHOT600
has not yet been retested against this same K range - do not assume it
inherits DSHOT300's K=9 result; that retest is still open (see below).

**Wired and verified on hardware, 2026-09-12:** `driver/dshot_profiles.py`'s
DSHOT300 entry now carries this K=9 tuple live, and `poll_telemetry()`
passes `expected_ratio`/`ratio_tolerance` through to `analyze_capture()` for
every profile (DSHOT600 still resolves to the brute-force sweep via its own
`expected_ratio=None`, unaffected). `scripts/verify_gcr_decode_port.py`
confirms both bars: the brute-force path is byte-for-byte unchanged
(359,448 groups checked, 9,381 pre-existing real mismatches - both from the
already-documented stale-`SEARCH_RANGE` K=8 sessions, not new - 7,309
expected polarity divergences), and the new fixed path is clean (4,743
groups checked against `captures/2026-09-12_13-04-36`, 0 real mismatches).

**On-device timing, measured, not projected:** `tests/test_gcr_decode_timing.py`
(since removed) now benchmarked both paths side by side. Sweep (DSHOT300@4MHz, the old path):
min=45,020µs mean=70,331µs max=108,063µs. Fixed (DSHOT300@3.375MHz, K=9):
min=9,737µs mean=10,852µs max=21,064µs - a **6.5x mean speedup**, and the
worst-case implied sustainable rate rose from 9 to 47 decodes/sec,
approaching the ~10ms `raw_samples`+`find_edges` floor this ADR's previous
section already identified as the pipeline's true bare-minimum cost. Every
group still decodes correctly on both paths (7/7 OK, no mismatches).

**Two smaller hardening fixes landed alongside this work:**
- `scripts/verify_gcr_decode_port.py` (see correction above) and a new
  `scripts/tally_period_cycles.py` now share format-aware session loading
  (`scripts/capture_session.py`) and both distinguish a session that
  legitimately has no bidir groups from one that declares a bidir motor
  but never got a real reply (an anomaly, not a silent pass).
- `tests/harness/run_scenario.py` (then `tests/test_scenario_capture.py`) gained a runtime reply failsafe
  (`check_reply_failsafe`): any scenario with a bidirectional motor now
  fails fast (~2s grace) if not one single captured record carries a
  real, non-all-zero reply, rather than running to completion and only
  revealing a dead ESC/bench in a post-hoc tally. This is what caught the
  `mpremote`-reset corruption above quickly once added, instead of
  requiring another multi-session investigation.

### Fixed-ratio RX sampling: DSHOT600 retune, sweep retirement, close-out (2026-09-12, continued)

**DSHOT600 retested on real hardware, same K range, same method as DSHOT300:**

| K | rx_speed | CRC-valid | mean period_cycles | std | samples/bit |
|---|---|---|---|---|---|
| 8  | 6,000,000 | 99.3% (2422/2439) | 7.7411  | 0.0780 | 3.87 |
| 9  | 6,750,000 | 100% (2463/2463)  | 8.7129  | 0.0576 | 4.36 |
| 10 | 7,500,000 | 100% (2455/2455)  | 9.6128  | 0.0957 | 4.81 |
| 11 | 8,250,000 | 100% (2458/2458)  | 10.6602 | 0.0587 | 5.33 |

A near-perfect mirror of DSHOT300's result: the 262-cycle capture window
and `period_cycles/2` samples-per-bit math are expressed in PIO cycles,
not time, so the margin/density reasoning transfers unchanged across
speeds - K=9's margin ratio computes to ~1.43x here too, matching DSHOT300
K=9's almost exactly. **Decision: K=9 for DSHOT600 too**
(`rx_speed=6_750_000`, `expected_ratio=8.7129`, `ratio_tolerance=0.0`) -
same reasoning as DSHOT300 (tightest spread among the fully-valid
candidates, better margin than K=10/K=11, K=8 again shows the one
measurable validity dip from under-sampling). Wired into
`driver/dshot_profiles.py` and verified: `scripts/verify_gcr_decode_port.py`
shows 2,463 groups checked against `captures/2026-09-12_16-01-39`, 0 real
mismatches.

**Sweep retirement scope, decided:** once both speeds had a verified fixed
ratio, `estimate_bit_period`/`SEARCH_RANGE_START`/`SEARCH_RANGE_END` were
deleted from `driver/gcr_decode.py` entirely - `analyze_capture()` now
requires `expected_ratio` (no more `None` fallback to a sweep that no
longer exists). This left one real question: what happens to
`scripts/verify_gcr_decode_port.py`'s brute-force-vs-port comparison for
every historical (pre-retune) session, since the on-device port side of
that comparison no longer exists? Decided: **drop historical coverage**.
`verify_gcr_decode_port.py`'s `check_one` (brute-force arm) was deleted
along with it - the tool now only ever checks sessions recorded at a rate
a live `BIDIR_PROFILES` entry is currently tuned for, reporting everything
else as `SKIPPED (no tuned profile for this session's recorded rate)`
rather than silently ignoring it or crashing. `scripts/dshot_bidir_decode.py`
(the PC-side reference) deliberately kept its own full sweep permanently -
its job is analyzing any capture from any era on demand, which the
historical sessions remain fully available for via direct use of that
script, just no longer through the automated regression check. Final,
full-corpus run after both retunes and the retirement: **11,928 groups
checked (all three sessions with a currently-tuned rate: 4,743 at DSHOT300
K=9, 4,722 at DSHOT300 K=9's real-hardware confirmation run, 2,463 at
DSHOT600 K=9), 0 real mismatches, 0 expected polarity divergences, 0
anomalies** - 24 sessions correctly skipped as self-diagnosed failures
(`outcome=failed`), 25 correctly skipped as recorded at a now-untuned rate.

**On-device timing, both speeds, post-retirement:** `tests/test_gcr_decode_timing.py`
(since removed) was rewritten to drop the sweep arm entirely (there is nothing left to
benchmark it against) and now benchmarks both speeds' fixed-ratio paths
side by side. Period search (`estimate_bit_period_fixed`) costs ~55-56µs
either way - about 1% of total decode cost - down from the sweep's 79-88%
dominance documented in the previous section. Full pipeline: DSHOT300
(K=9) min=9,693µs mean=10,799µs max=22,660µs; DSHOT600 (K=9)
min=9,704µs mean=10,914µs max=20,700µs - both essentially identical, both
close to the ~10ms `raw_samples`+`find_edges` floor identified earlier as
this pipeline's true bare-minimum cost, confirming period search is now
effectively free for both speeds.

**Status: this effort is closed.** Both DSHOT300 and DSHOT600 have a
verified, hardware-confirmed fixed-ratio RX sampling configuration; the
brute-force sweep is fully retired from the on-device driver; the
PC-side reference and regression check are both updated and passing
clean. Adding bidirectional support for any DShot speed beyond these two
would need the same measure-K-candidates-on-hardware method repeated from
scratch - the tooling built for it (`tests/harness/scenarios/
period_tally_short*.json` (since removed), `scripts/tally_period_cycles.py`,
`scripts/capture_session.py`, `scripts/deploy.py`'s reset-before-run) is
all reusable as-is.

### Per-pair TX/RX synchronization: stale-signal clearing and relative IRQ addressing (2026-08-29/30)

The TX state machine tells its paired RX state machine that the pin has been
released by raising a PIO IRQ flag. Two separate problems with that signal
were found and fixed, and both fixes are load-bearing in
`dshot_bidir_tx`/`dshot_bidir_rx` today.

**The flag is sticky, not a queue (2026-08-29).** If the RX state machine is
still busy when TX signals a later frame - typically because `autopush` has
stalled on a full 4-word RX FIFO that nobody drained - that signal stays
latched, and RX consumes it as if it were fresh the next time it reaches its
wait. The capture is then phased against the wrong point in time, and
`wait(0, pin, 0)` can trigger on TX's own LOW bits, capturing TX's waveform as
a "reply". The latched flag also survives `restart()`, so a stale signal from
before `stop()` leaked into the next run. The fix is `irq(clear, rel(0))` at
the top of every RX iteration, so the wait blocks for a genuinely new release
every time (including the first one after `start()`), plus `start()` flushing
the RX FIFO and activating RX before TX.

Verified with a targeted repro: not draining the RX FIFO for 10 frames at a
settled throttle (enough to fill it and stall the state machine), then
resuming and decoding offline. Before the fix, 19 of 22 captures were
CRC-valid, with 3 consecutive corrupted captures at the stall boundary - one
measured a 6.0-cycle bit period against a ~10.2-10.3 cycle baseline, a
plausible-looking but wrong decode. After the fix, 20 of 21 were CRC-valid
with exactly one affected capture, a cleanly truncated pattern that fails CRC
and is rejected. The standard throttle sweep was unaffected (17/17 CRC-valid,
eRPM ~21.6k / ~48.8k / ~75.6k at throttle 100/200/300). These runs predate
the fixed-ratio RX retune, so the cycle counts are at the old 4MHz `rx_speed`.

**The flag was shared by the whole PIO block, not private to a pair
(2026-08-30).** The first implementation used a literal `irq(4)` /
`wait(1, irq, 4)`. IRQ flags 4-7 never reach the CPU, but every state machine
on a PIO block shares them: there is one flag 4 per block. With one
bidirectional pair per block that is invisible; with two pairs on one block,
both RX state machines wait on the same flag and either can consume the signal
meant for the other. This was flagged by an external review and confirmed by
re-reading the assembly, while extending the bench harness to several
bidirectional channels.

The fix is relative IRQ addressing. `rel(k)` resolves at runtime to a flag
derived from the executing state machine's own id, so TX fires `irq(rel(1))`
and its RX waits on `irq(rel(0))` and both land on the same flag, while a pair
with different state machine ids lands on a different one. That only holds if
every pair uses the same TX-to-RX id offset, which is why `BidirectionalDShot`
requires `rx_state_machine_id == state_machine_id + 1`. One program serves
every pair; nothing is assembled per pair.

The first attempt at this fix, on 2026-08-30, was reverted after channel 1
(sm0/rx1) produced no completed telemetry groups either alone or paired with
channel 3 on the same PIO0 block, while channel 3, using the identical
mechanism in the same run, was 100% CRC-valid. That revert rested on
confounded evidence: channel 1's ESC power turned out to have been off, which
explains "fails alone and paired, while channel 3 is fine" better than a
driver bug does. The retry, with ESC power confirmed on every channel under
test, passed:

- one bidirectional pair: 17,624 of 17,624 captures CRC-valid;
- two pairs sharing PIO0 (channel 1 sm0/rx1 at throttle 150, channel 3
  sm2/rx3 at throttle 300): 15,437 of 15,437 CRC-valid on each, with distinct,
  throttle-proportional eRPM (~61.5k vs ~143.5k) and no cross-talk;
- two pairs on separate blocks (channel 1 on PIO0, channel 3 on PIO1) running
  diverging throttle profiles for 60 seconds: 30,221 of 30,221 and 30,220 of
  30,221 CRC-valid, ~504 records/s sustained, no dropped records.

Not verified: a fourth pair (channel 4, sm6/rx7 on PIO1) reproducibly failed -
about 322 records/s against a 450/s floor, and 61.1% CRC-valid - even when it
was the only pair on its block, which rules out block sharing as the cause.
The cause is unknown (its pin, wiring or ESC channel are all candidates) and
it is parked; the relative-IRQ mechanism itself is confirmed by the other
three channels.

### Command-loop and decode performance (2026-09-19)

Measured on the bench board (MicroPython v1.28.0, 150MHz, no global interpreter
lock) with no ESC or motor: state machines on unused pins, the transmitter's own
waveform captured as stand-in replies. The benchmarks were `tests/bench_cpu_costs.py`,
`bench_loop_gaps.py`, `bench_drain_real.py` and `bench_decode.py`, removed once their figures
were recorded here (they remain in git history).

**Where the time goes.** The interpreter is slow on this build - an empty loop
iteration takes 1.7us - and cost follows bytecode and call count, not the work
being done: a plain function call is 4.3us, a method call 16us, a class-attribute
read through an instance 5.5us (an instance attribute 0.8us), and a small
allocation 5-30us. The command loop is therefore CPU-bound, never wire-bound:
one unidirectional motor's tick took 139us against 53us of wire time, four took
318us, and adding one bidirectional motor whose RX state machine had words to
drain took it to 658us.

**Heap churn and stalls.** Reading the RX FIFO word by word makes a Python
integer per word, and any word above 30 bits is a heap object: about 74 bytes per
capture, over 100KB/s of garbage from the command loop alone. A garbage
collection on either core pauses both. Forced collections on the application
core stalled the command loop for up to 4.9ms on a small heap and for over 10ms
(up to 14ms) with 150KB live; the old per-sample decoder, allocating 11KB per
capture, stalled it for up to 12ms. Sustained allocation on the application core
cut the loop's rate about 2.5 times, and with collection disabled for the same
allocation the loop recovered, so it is the collections, not the allocations,
that cost. A stall long enough to leave the RX FIFO undrained is the condition
the earlier starvation runs tied to corrupted captures, which makes garbage
collection a plausible cause of the unexplained mid-run CRC failures; that link
has not been shown on a real ESC.

**What changed.**

- The drain reads a whole capture with one bulk `get(array)`, straight into the
  published slot, when 4 words are waiting. That is 15us and no allocation
  against about 50us and 74 bytes for four reads plus about 400us of bookkeeping
  in the loop. A tick with one bidirectional motor went from 388us to 175us and
  from 73 bytes allocated to none; four unidirectional plus one bidirectional
  motor went from 658us (1,519 ticks/s, 973 of about 4,500 ticks over 1ms) to
  366us (2,730 ticks/s, none over 1ms, worst tick 448us), and the command loop's
  own allocation from 113KB/s to 1.4KB/s.
- `send_throttle_command` passes the 16-bit packet to `put(packet, 16)` and lets
  the C side do the shift. Shifting in Python made a heap integer per frame at
  higher throttle. The words put on the FIFO were checked to be identical for
  every throttle in both CRC polarities (`tests/test_put_shift.py`, since replaced by the packet unit test in
  `tests/unit/test_dshot_packet.py`), and sends
  now allocate nothing.
- The decoder works on integers: the edges come from XOR-ing each half-word with
  itself shifted by one, the frame is built as an integer and differential
  decoding is one XOR. A decode went from 10.1ms and 11KB allocated to 1.3ms and
  under 1KB. Its results are identical to the previous decoder on 5.57 million
  comparisons over every real session on disk (five bit-period ratios) and on
  400,000 random captures, and `scripts/verify_gcr_decode_port.py` reports no
  mismatch against the PC-side reference over 690,901 groups.

**How the integer decoder works, on a real capture.** The example is the first
DSHOT300 capture in `tests/test_gcr_decode_timing.py` (its captures now live in
`tests/unit/test_gcr_decode.py`), taken from the bench ESC
with a slowly turning motor. The wire is read 128 times and the readings are
packed into four 32-bit words; the decoder has to recover the 21-bit reply from
them. The old decoder expanded all 128 readings into a list of (cycle, value)
tuples and walked it. The new one notices that only a handful of readings matter:
the ones where the signal flips.

*Step 0 - the raw material.* The first two words:

```
word 0:  0000000111110000 | 0000111111111000
word 1:  0000000001111000 | 0000000000111111
```

*Step 1 - find the flips by XOR-ing with a shifted copy.* `find_edges()` takes one
16-bit half of a word at a time. Shifting a copy right by one place lines every
reading up with the reading before it, and XOR marks where the two differ:

```
readings          0000000111110000
shifted right     0000000011111000     (the reading before each one)
XOR               0000000100001000
                         ^    ^
                         |    +-- reading 12: flipped high -> low
                         +------- reading 7:  flipped low -> high
```

A 1 in the XOR row means "this reading differs from the one just before it". The
second half works the same way:

```
readings          0000111111111000
shifted right     0000011111111100
XOR               0000100000000100     -> flips at readings 20 and 29
```

Across the whole capture only 9 of the 128 readings are flips:

```
readings: 00000001111100000000111111111000000000000111100000000000001111111111110000...
flips:    .......^....^.......^........^...........^...^............^...........^......
```

*Step 2 - visit only the 1s.* `d & -d` isolates the lowest 1 in a number, and
XOR-ing it back out clears it:

```
d      = 0000000100001000
-d     = 1111111011111000     (negating flips everything above the lowest 1)
d & -d = 0000000000001000     <- only the lowest 1 survives

d = 0000000100001000   lowest is bit 3 -> reading 12
d = 0000000100000000   lowest is bit 8 -> reading 7
d = 0000000000000000   done
```

The flips come out newest first, so the list is sorted at the end. A dictionary
maps the isolated bit to its position, because this MicroPython has no
`int.bit_length()`.

*Step 3 - turn the stretches into bits.* Each flip ends a stretch of constant
value. Its length in cycles divided by the profile's bit period (8.7069 cycles
here) is the number of bits it holds:

```
readings   0..6    low   14 cycles / 8.7 = 1.6 -> 2 bits of 0
readings   7..11   high  10 cycles / 8.7 = 1.1 -> 1 bit  of 1
readings  12..19   low   16 cycles / 8.7 = 1.8 -> 2 bits of 0
readings  20..28   high  18 cycles / 8.7 = 2.1 -> 2 bits of 1
readings  29..40   low   26 cycles / 8.7 = 3.0 -> 3 bits of 0
readings  41..44   high   8 cycles / 8.7 = 0.9 -> 1 bit  of 1
readings  45..57   low   26 cycles / 8.7 = 3.0 -> 3 bits of 0
readings  58..69   high  26 cycles / 8.7 = 3.0 -> 3 bits of 1
readings  70..82   low   26 cycles / 8.7 = 3.0 -> 3 bits of 0
```

The nine stretches give 20 bits. The reply has 21: the last data bit is a 1 that
merged into the idle-high line and has no flip of its own, so `reconstruct_frame()`
pads idle-valued 1s up to 21 bits. That padding is load-bearing.

*Step 4 - build the reply as one number by shifting.* Each stretch is shifted onto
a single integer instead of being appended to a list of 21 bit objects:

```
start                  0
append 2 x 0        ->  00
append 1 x 1        ->  001
append 2 x 0        ->  00100
append 2 x 1        ->  0010011
append 3 x 0        ->  0010011000
append 1 x 1        ->  00100110001
append 3 x 0        ->  00100110001000
append 3 x 1        ->  00100110001000111
append 3 x 0        ->  00100110001000111000
pad the idle 1      ->  001001100010001110001    (21 bits)
```

The first bit is the marker, always 0; the 20 after it are the data.

*Step 5 - undo the differential encoding with another XOR-shift.* The ESC sends
each bit as "did the signal flip?", so recovering the data is the same trick as
step 1, and the shift brings in a 0 at the front, the marker's own value:

```
data20             01001100010001110001
data20 shifted     00100110001000111000
XOR                01101010011001001001
```

*Step 6 - look up the 5-bit groups.* The 20 decoded bits are four 5-bit GCR
symbols, and a 16-entry table turns each into a nibble:

```
01101   01001   10010   01001
  D       9       2       9      ->  0xD929
```

The last four bits are the CRC, which checks out. The other 12 bits are the
payload: mantissa 402, exponent 6, so the period is 402 << 6 = 25,728us and the
eRPM is 60,000,000 / 25,728, about 2,332.

*Why 16-bit halves.* A MicroPython integer above about 30 bits is a heap object
and needs an allocation. Scanning 16 bits at a time keeps every intermediate
value small, so `find_edges()` allocates almost nothing, and that matters because
garbage is what triggers the collection that pauses both cores.

**Not addressed.** `send_throttle_command` still costs about 53us per motor, of
which the packet arithmetic is about 7us, and `update()` has about 87us of fixed
overhead per tick; a lookup table of packets and a flattened loop measured about
10 times faster in a prototype but restructure the hot path and move the
throttle validation, so they were left. When to collect garbage - for example at
points where a stall is harmless - is a scheduling matter for the application.
The scenario harness no longer reads words itself: it runs through `MotorGroup` and samples the latest capture (ADR-005).

### Idea, not built: run-length capture in the PIO receiver (2026-09-20)

The receiver today records the pin 128 times at a fixed rate and leaves all the
interpretation to the CPU. The integer decoder above made that interpretation
cheap, but the work it does - find where the signal flips, measure the stretch
between flips, turn each stretch into bits - is exactly what a counter and a
small state machine do in PIO, with no CPU involved. This records the idea and
what would have to be settled; nothing here has been built or measured on
hardware.

*Level 1 - the PIO measures the stretches.* A down-counter loop that runs while
the pin holds its level and stops at the next flip. The length of the stretch is
the count:

```
    mov x, ~null        ; x = all ones, used as a down-counter
low:
    jmp pin, done       ; the pin went high? the stretch is over
    jmp x--, low        ; otherwise count one more (2 cycles per loop)
done:
    mov isr, ~x         ; ~x is how many loops the low stretch lasted
    push                ; hand that length to the CPU
```

A mirror-image loop measures the high stretches. Each count is 2 cycles, the same
resolution as the current sampling, so nothing is lost. A reply has about 9 to 15
stretches and each count is a few bits, so they pack several to a word and a reply
takes two or three words instead of four. The CPU no longer needs `find_edges()`
(about 0.44ms of the 1.27ms decode).

*Level 2 - the PIO builds the frame.* PIO has no divide, but it can subtract in a
loop. Preload a second counter with the bit period, count it down while the
stretch counter runs, and every time it reaches zero shift one bit of the current
level into the input shift register and reload it. A 26-cycle stretch then yields
3 bits directly, and the receiver hands the CPU the finished 21-bit frame. The CPU
is left with the differential XOR, the table lookup and the CRC check, roughly
0.3ms by the measured stage costs (`find_edges` 0.44ms, `reconstruct_frame`
0.54ms, period estimate 0.1ms, `decode` 0.15ms, `check_crc` 0.09ms), and
`estimate_bit_period_fixed` disappears as a CPU step.

The counters restart at every flip, which is how a hardware UART receiver stays in
step with a sender whose clock is slightly off. The early point-sampling designs
in this ADR failed because they sampled at an assumed bit period and never
re-synchronised, so a few percent of error accumulated across the frame; re-syncing
at each flip bounds the error to one stretch.

*What would need settling before building it:*

- **Program space.** A PIO block has 32 instructions shared by every state machine
  on it. The bidirectional transmit program uses 13 and the current receiver 10,
  so the new receiver would replace the oversampling one rather than sit
  beside it, and it has to fit.
- **The bit period.** With no fractional subtraction, either the receiver's clock
  divider is tuned so a bit is a whole number of cycles (the divider is fractional,
  so this is reachable per ESC oscillator, but it must be measured per unit), or
  an integer period is accepted with a few percent of error. Re-syncing at each flip
  keeps that error inside the half-bit rounding margin for the 3-bit stretches
  GCR frames contain.
- **The end of a frame and silence.** Today the capture is a fixed 128-reading
  window. A run-length receiver has to recognise "idle long enough, the frame is
  over", and cope with an ESC that does not reply or replies partially, without
  stalling the FIFO - an undrained or stalled receiver is the condition this ADR
  ties to corrupted captures.
- **Diagnostics.** The raw 128 readings are what exposed the transmit echoes, the
  stall corruption and the bit-period drift. Compressing them hides that, so the
  raw receiver would stay available as a diagnostic profile.
- **Echoes and noise before arming.** The receiver must reject or flag frames that
  are the transmitter's own waveform, as the raw receiver is checked for today.

*How it could be validated.* Several state machines can read one pin, so the new
receiver can run beside the existing one on the same motor, using a spare state
machine, and every frame it produces can be compared with the current decoder's
result on the same reply. Agreement over many thousands of real replies, including
during arming and after a stall, is the bar before it replaces anything.

*Spike result (2026-09-20).* A level-2 receiver was built as a second receiver
program, `dshot_bidir_rx_rle`, and run on the bench. It differs from the sketch
above in how it finds bit boundaries: instead of subtracting the bit period in a
loop it runs a per-bit timer - a count-down of 2-cycle passes that tests the
pin on every pass. When the timer runs out with no flip, it reads the pin (one
bit, one whole bit after the previous read) and reloads the timer. When the pin
flips, it jumps to the other level's loop with a short count, so the next read is
half a bit after the edge, at the centre of the new bit. This is the way a
hardware UART receiver re-synchronises, and it needs no division. It reads
exactly 21 bits and autopushes them as one word, marker at the top, so the end of
a frame never has to be detected.

- **Clock.** The bit is 16 receiver cycles at both levels, so the receiver clock
  is 16 times the reply bit rate: 6.20MHz at DSHOT300 and 12.40MHz at DSHOT600
  (`rle_rx_speed()`; the rate is `rx_speed / expected_ratio`, the measured value).
  The two levels' paths are made the same length with a nop; before that, high
  bits took 15 cycles and low bits 16, and the last read drifted early enough to
  fail 1.4% of replays.
- **Program space.** It fits with `dshot_bidir_tx` only at 19 instructions,
  because the pre-delay is one `nop` with a 26-cycle delay slot instead of a
  counted loop. Together they fill the block: it cannot also hold the raw
  receiver (replacing the raw receiver of a constructed `BidirectionalDShot`
  fails with ENOMEM), nor the unidirectional program. A unidirectional motor has
  to sit on another block. Four bidirectional motors with this receiver, one pair
  each (see "State machines and instruction memory"; identical programs are loaded
  once per block, so a second pair on a block adds state machines but no slots):

  | Block | State machines | Programs loaded | Slots used |
  |---|---|---|---|
  | PIO0 | sm0 TX + sm1 RX (motor 1), sm2 TX + sm3 RX (motor 2) | `dshot_bidir_tx` + `dshot_bidir_rx_rle` | 32 of 32 |
  | PIO1 | sm4 TX + sm5 RX (motor 3), sm6 TX + sm7 RX (motor 4) | `dshot_bidir_tx` + `dshot_bidir_rx_rle` | 32 of 32 |
  | PIO2 | free (sm8 to sm11): unidirectional motors go here | `dshot` | 4 of 32 |

  Two of these pairs on one exactly-full block has not been run on hardware.
- **Model.** A PC model of the program (`scripts/simulate_rle_receiver.py`)
  replays stored captures with the pin's waveform rebuilt from the raw samples.
  At the profile's clock it rebuilds the frame `gcr_decode` builds for all 10,867
  CRC-valid replies in four sessions (three DSHOT300, one DSHOT600). With the
  receiver clock 1.5% off it is still 99.5%, 3% off 96.8% (slow) or 85.6% (fast),
  5% off 74% or 44%. The replayed waveform has more edge jitter than the real
  signal, so these are pessimistic.
- **Bench.** On the AM32 ESC (channel 1, DSHOT300, throttle 100, motor spinning):
  9,971 replies in 5 seconds, every one with the marker bit 0, valid GCR symbols
  and a valid CRC, eRPM 21.4k to 21.7k (the raw receiver's bench figures are
  100% CRC-valid and 21.4k to 21.8k). Decoding a frame - `decode()` and
  `check_crc()`, all that is left for the CPU - took 214us on average and 345us at
  most, against 1.27ms for the raw path. No frame was lost from a 64-frame ring
  drained by the application core. At DSHOT600 (receiver clock 12.40MHz) the same
  bench gave 10,009 replies in 5 seconds, all valid, eRPM 21.6k to 21.8k, decode
  212us on average and 298us at most.

Not settled by the spike: behaviour when the ESC does not
reply, replies partially or before arming (the program waits for a falling edge
like the raw receiver, so it can take the transmitter's own waveform for a reply
in the same way, and nothing has checked how that looks in a 21-bit frame); a
stalled or slow drain (a word per reply is far less pressure than four, but the
same undrained-FIFO condition applies); comparison against the raw receiver on
the very same replies (the bench shows both are valid, not that they agree
frame for frame - the model does that on stored captures); and integrating it
with `CaptureMailbox`, which takes whole 4-word captures.

*What it would buy.* About a millisecond of application-core time per decode, a
smaller FIFO payload, and no oversampling density to tune per DShot speed. The
decode is already off the command loop, so this is application headroom, not
command-loop speed. It has to be weighed against replacing a receiver that is
verified on hardware for both supported speeds.

*Integration (2026-09-26).* The receiver is now a `receiver=` option on
`BidirectionalDShot`'s constructor, named for what each hands the CPU:
`SAMPLE_RECEIVER` (the default) or `FRAME_RECEIVER` - rather than a standalone
spike built by hand. Still EXPERIMENTAL, not yet validated against the sample
receiver on live replies. What changed to make it selectable:

- `CaptureMailbox` takes a `capture_words` argument (4 for the sample
  receiver's raw samples, 1 for the frame receiver's already-reconstructed
  frame) instead of a fixed constant, so one class serves both.
- `gcr_decode` gained `analyze_frame()`, the frame receiver's own entry point:
  it shares `analyze_capture()`'s decode/CRC/eRPM tail (now factored out as
  `decode_result()`) but skips `find_edges()`, `estimate_bit_period_fixed()`
  and `reconstruct_frame()` entirely, since the receiver did that work in
  hardware. Its result carries no period fields (nothing is measured per
  capture), and both entry points now also carry a `marker_ok` field - the
  frame's own top bit read back as 0 - worth surfacing next to the CRC check
  while this receiver is unproven.
- The constructor picks the receiver program, its clock, and `jmp_pin` (the
  frame program's only receiver-specific pin wiring - the sample receiver has
  no `jmp(pin, ...)` instructions and does not get it) once, and `start()`
  replays that same choice on every `arm()`, the first one included. The
  spike's own bench harness never exercised this at all: its `BidirectionalDShot`
  subclass overrode `__init__()` and `start()` to build the frame receiver's
  state machine directly, so `rx_sm.init()` never ran with this program - on a
  block it fills on its own (see the layout table above) or otherwise - and
  still has not run on hardware.
- The bench script now drives the frame receiver through
  `BidirectionalDShot`/`MotorGroup` like every other motor - reading it with
  `raw_telemetry()`/`decode_telemetry()` and counting missed frames from the
  mailbox's own sequence number - rather than a private ring buffer and a
  `BidirectionalDShot` subclass that bypassed the `start()`/`stop()` path
  above entirely.

Not settled by this pass, same as the spike: behaviour when the ESC does not
reply, replies partially, or before arming; a stalled or slow drain;
frame-for-frame agreement against the sample receiver on live replies (the PC
model agrees on stored captures - see "Model" above - which is not the same
claim); and whether the untested `start()`/`stop()` path above actually
succeeds on hardware. The spike's own decode-cost figures (~200-215us) also
don't carry over as-is: they measured `decode()` plus `check_crc()` alone,
not `decode_capture()`'s or `analyze_frame()`'s own overhead (a 9-key dict
built per call) - the bench script measures the real thing, but through a
one-slot mailbox that can no longer report a per-frame count the way the
spike's private ring did. Adoption is still an open decision, gated on
hardware validation.

*Hardware validation, part 1 (2026-09-26).* The integrated `start()`/`rx_sm.init()` path above -
the one thing this pass could not exercise without a real board - has now run: `rle_bench.py` on
channel 1 (motor + prop mounted) gave 4,413/4,413 CRC-valid at DSHOT300 (median eRPM 21,067) and
4,410/4,410 at DSHOT600 (median eRPM 21,186), both matching the spike's own CRC-valid rate and
eRPM range. A new device test (`tests/experimental/test_rle_restart_cycles.py`) ran two
arm/spin/disarm/arm cycles at DSHOT300 on the same wiring: both cycles 143/143 CRC-valid, median
eRPM 21,126, no ENOMEM - so a second `rx_sm.init()` with this program, on a block with zero free
slots, does not re-add it and run out of space. That was the specific risk this integration
carried; it did not materialize.

What did surface: the bench's own decode cost (mean 698-699us, max up to 7,546us) is far above
the spike's 212-214us. The spike measured `decode()` plus `check_crc()` alone; `rle_bench.py`
measures the real `decode_capture()`/`analyze_frame()` call, including `decode_result()`'s dict
construction, and it decodes every new capture rather than sampling every Nth one the way
`run_scenario.py` does - so this is not yet evidence that `analyze_frame()` itself is slow, only
that this particular loop is. The same run lost more replies to mailbox overwrite than it kept
(5,017 of ~9,430 at 300; 4,970 of ~9,380 at 600), consistent with a poll loop too slow to keep up
with a one-slot mailbox at the full reply rate. Neither figure is a regression from anything
verified before now - there was no prior number to regress from - but both need separating out
(loop overhead vs. `analyze_frame()` cost) before the decode-cost comparison against the sample
receiver can be trusted.

Still not settled: everything else this section's "Not settled by this pass" already named -
behaviour when the ESC does not reply, replies partially, or before arming; a stalled or slow
drain; and frame-for-frame agreement against the sample receiver on live replies.

*Hardware validation, part 2 - statistical comparison (2026-09-26).* Same bench session as part 1
(same motor, prop, ESC, throttle 100): the sample receiver's existing regression scenarios
(`telemetry_settled_300/600.json`) gave 99/99 CRC-valid (a decoded sample - `run_scenario.py`
decodes every 20th new capture by design) with median eRPM 21,127 at DSHOT300 and 21,490 at
DSHOT600; the frame receiver's `rle_bench.py` run from part 1 gave 4,413/4,413 and 4,410/4,410
(every capture it saw) with median eRPM 21,067 and 21,186. Both receivers clear the same
threshold at both speeds, with eRPM agreeing within ~450 (300) and ~300 (600) - normal
run-to-run variation, not a discrepancy.

This is a statistical comparison, not the frame-for-frame one this section originally asked for:
the frame receiver's real 19-instruction program fills its PIO block alongside `dshot_bidir_tx`
(32 of 32 slots), so there is no room left for `dshot_bidir_rx` on the same block to capture the
same live reply the way the original validation plan assumed a spare state machine could. The
frame-for-frame check that *is* possible stays the offline one already done: the PC model
(`scripts/simulate_rle_receiver.py`) against 10,867 stored raw captures.

The two "records published but never seen" counts from these runs (sample: 10,039 of 12,029 at
300; frame: 5,017 of ~9,430 at 300) are not a fair receiver-to-receiver comparison either -
`run_scenario.py`'s loop also does per-tick SD writes and steps a throttle profile across 4
motors, work `rle_bench.py`'s loop doesn't do at all, so they measure two different loops'
overhead more than the two receivers' relative cost. Separating decode cost from loop overhead is
still open, per part 1's note above.

*Hardware validation, part 3 - stalled drain, corrected (2026-09-26).* An earlier version of this
note reported that a full RX FIFO corrupts a frame-receiver capture rather than merely delaying
it, based on `tests/experimental/test_rle_stalled_drain.py`: a burst of 20 frames with no drain at
all came back with 3 of 5 drained captures corrupted (bad CRC; one also failing `marker_ok`). A
fix (`fifo_join=PIO.JOIN_RX`, doubling the FIFO to 8 one-word captures, the same fix
`dshot_bidir_rx` already has) was implemented on that basis. Both the finding and the fix were
wrong, and both are reverted - `dshot_bidir_rx_rle` has no `fifo_join`.

The finding was a bug in the test, not the receiver. The burst paced commands at `motor.frame_us`
(~54us at DSHOT300) - the TX bit-shift time only, not the ESC's own reply (another ~54us at this
profile's bit period, plus a ~4us predelay). Re-arming TX that fast drove the line again before
the ESC's reply had finished, corrupting it by interference on the wire - a failure that looks
identical to a genuine RX-FIFO-stall corruption once only the drained result is inspected.
Re-running the same burst with the interval widened to comfortably exceed a full reply's duration
(200us) came back clean - at the FIFO's original, unmodified depth, and at bursts up to 60 frames
with zero drains, both with and without the (now-reverted) `fifo_join`. Every run showed the same
pattern: exactly depth+1 captures drained (5 at depth 4, 9 at depth 8) regardless of how many
frames were sent beyond that, all of them `marker_ok` and CRC-valid. This matches
`dshot_bidir_rx_rle`'s own comment, and the structural read that motivated it: the 21st of 21
reads is the one autopush fires on, and every bit is already shifted into the ISR by then, so a
full FIFO stalls holding a complete, correct value - it does not corrupt one. The cost of a long
stall is silently missing later replies (the state machine does not resume watching for the next
release IRQ until room frees), not wrong data.

Lesson worth keeping: reading the PIO program's structure predicted the correct answer twice (the
frame-for-frame agreement in the "Model" section above, and this property) and the badly-paced
test contradicted it once, before a corrected test confirmed the structural reading was right.
Read the structure, then verify the test itself paces the wire correctly before trusting a
hardware result that disagrees with it.

Not yet checked: the arming-window and no-reply behaviours this section's "Not settled" list
still names.

*Hardware validation, part 4 - arming window (2026-09-26).* `dshot_bidir_rx_rle` waits for a
falling edge exactly like `dshot_bidir_rx`, so early in arming - before the ESC has locked onto
bidirectional DShot - a capture could in principle be TX's own waveform rather than a genuine
reply. `tests/experimental/test_rle_arming_echo.py` drove a lone bidirectional motor plus the
usual three idle unidirectional ones directly (no `MotorGroup`, so nothing discards captures the
way `update()` does while `ARMING`) at zero throttle for 3 seconds, bucketing every capture by
500ms slice, then continued at throttle 100 for 2 seconds as a clean baseline.

Result: `marker_ok` was 100% in every bucket, including the very first 500ms - no sign of the
receiver locking onto TX's own waveform, which would be expected to show up as scrambled marker
bits, not a clean 0 every time. `crc_ok` told a different story: 0% in the first 500ms, 74% in the
second, 100% by the third (1000ms), an unexplained dip back to 39% at 1500ms, then 100% from
2000ms onward and through the whole spin baseline. Marker-bit correctness with a fluctuating CRC
rate reads as the ESC replying almost immediately with genuine (not echoed) frames, some of them
bit-error-prone during its own bidirectional-mode lock-on transient, rather than the receiver
mis-triggering on our own signal. This has no bearing on correctness today - `MotorGroup.update()`
already discards every capture taken during `ARMING` regardless of validity - but it is new
information should the arming duration or lock-on timing ever need tuning.

*Hardware validation, part 5 - no reply (2026-09-26, reasoned from source, not bench-forced).*
Simulating "the ESC never replies" needs depowering or disconnecting it mid-run, which this
session's remote access to the bench can't do. Reasoning from the program instead:
`dshot_bidir_rx_rle`'s `wait(0, pin, 0)` marker-wait has no timeout, identical to
`dshot_bidir_rx`'s own step 3 - if the ESC never replies to a given frame, the state machine
blocks there indefinitely, and does not return to `wrap_target()` to watch for the *next* frame's
release IRQ until some falling edge, any falling edge, finally arrives. When the ESC eventually
does reply again, that reply's own marker edge is what unblocks it; the read logic doesn't care
which frame's window it's nominally in, so it decodes correctly, then resyncs cleanly via the
fresh IRQ wait for every frame after that. `CaptureMailbox.latest()` returns `None` until the
first real publish regardless of receiver (unit-tested, receiver-independent), so an application
sees nothing during the gap rather than stale or garbage data.

This is a smaller extrapolation than it would have been before part 3 above: the stalled-drain
test already exercised the same "not listening for many frames, then resyncs cleanly on
`wrap_target()`" pattern far more aggressively (60 frames of TX activity with RX not listening,
there stalled on the FIFO push rather than the marker wait) and it held up on the bench. Not
proof of the no-reply case specifically, but not a bare unforced reading either.

*Decision (2026-09-26): adopt the frame receiver.* Parts 1-5 above are the validation this idea's
own "How it could be validated" and "Done when" asked for before deciding whether it replaces the
oversampling (sample) receiver, stays as a second option, or is dropped. Summary of what was
checked: `start()`/`arm()` re-initializing this program on a PIO block it fills alone, at both
DSHOT300 (4,413/4,413 CRC-valid, median eRPM 21,067) and DSHOT600 (4,410/4,410, 21,186), and across
a disarm/arm restart (143/143 CRC-valid both cycles); a same-day statistical comparison against
the sample receiver's own regression scenarios, agreeing within normal run-to-run variation at
both speeds; a stalled drain up to 60 frames with zero drains, holding correct values rather than
corrupting them, at the FIFO's unmodified default depth; an arming window showing no sign of the
receiver mis-triggering on TX's own waveform; and no-reply behavior reasoned to match the sample
receiver's own, on the same resync mechanism the stalled-drain test already exercised. No problem
specific to the frame receiver surfaced anywhere in this pass.

**Decision: adopt the frame receiver as the sole production receiver.** The sample receiver
(`dshot_bidir_rx`) moves out of `BidirectionalDShot` entirely rather than staying as a second
option - the deciding factor was not a technical shortcoming of either receiver, but the ongoing
cost of a testing/analysis harness that would otherwise support two capture formats indefinitely
for no production benefit, now that the replacement is validated. Raw capture is not deleted
outright: it still has two uses beyond diagnostics (measuring `BIDIR_PROFILES`' `expected_ratio`
for a new ESC unit, since the frame receiver has no period search of its own to fall back on; and
the PC-side tooling - `scripts/simulate_rle_receiver.py`, `scripts/verify_gcr_decode_port.py` -
that consumes raw captures directly) and moves to a standalone script instead. Tracked as W27
(port the harness to frame-only 1-word records) and W28 (remove the sample receiver from the
driver, build the standalone tool) in the backlog.

### The receiver's FIFO is joined to 8 words (2026-09-20)

The receiver program pushes each reply as exactly 4 words, and its FIFO was 4 words deep, on the reasoning that one capture could then never block mid-frame. That holds only while the CPU takes every capture before the next reply's first word arrives, and the next command (which starts the next capture) is queued within tens of microseconds of the drain. With two bidirectional motors driven from one command loop, the motor drained last lost its replies: the capture's first word blocked on the full FIFO, the receiver's sampling paused while the reply carried on, and it resumed after the reply had ended, so the words were a short burst followed by idle-level words. It repeated on every following capture, and which channel it hit changed from run to run.

The receiver never uses its TX FIFO, so `fifo_join=PIO.JOIN_RX` gives its RX FIFO the whole 8 words at no cost: room for one capture the CPU has not taken yet plus the next. On the bench (two bidirectional motors, DSHOT300 and DSHOT600, 8-second and 60-second scenarios) it removed the failure: the old send-first order with the joined FIFO was clean in 4 of 4 runs, and with the drain moved before the send (ADR-005) both channels were 100% CRC-valid in the 60-second runs. Stalling the consumer for longer than a capture is still possible and still produces a corrupted capture, but it now takes two late drains in a row rather than one.

## Implementation Update (2026-09-25): post-disarm line state

A bidirectional motor's ESC went silent after `disarm()` instead of returning to its normal
idle tune, and stayed that way until the Pico was hard-reset. Every bidirectional motor did
this, not only when two ran together — an unrelated companion unidirectional motor's own idle
tune had been masking the same failure in every earlier single-motor test.

**Cause.** `stop()` released the line to input, held high only by its pull-up, and never drove
it again. AM32's own firmware self-reboots after a signal-loss timeout (confirmed from its
source: 0.5s while armed, 2s while unarmed) — this reboot is normal, constant idle behavior, not
itself a problem. On that reboot, AM32's bootloader finds no path to the application while the
Pico holds the line high, and its own receive loop then waits for a UART byte with no timeout
for a line that never goes low, so it hangs. A line that is actively driven low, as a
unidirectional motor's frozen line always is, does not hit this: the bootloader finds a path to
the application on the very next reboot. A stuck ESC can be rescued the same way after the
fact — forcing the line low (or a genuine chip reset, which removes the pull-up along with
everything else) gets it out via a different, ~20ms timeout inside the bootloader's own receive
loop. (Which exact bootloader build is flashed on this Skystar KM55A2 was not independently
confirmed; the mechanism above is read from AM32's reference source, not this board's binary.)

**Fix.** `stop()` now waits a fixed 300µs after `drain()` returns (a generous guess, not computed
from the ESC's actual reply timing — nothing guarantees the reply has actually finished by then),
then drives the line low. `start()` reclaims the pin for PIO before resuming. Considered and
rejected: keeping the pin under PIO and forcing it low with `sm.exec()` (never tried on hardware,
so it would need its own verification before trusting it); doing this at the application/harness
level instead of in the driver (pushes a correctness requirement onto every future caller, and
doesn't even avoid touching pin ownership once a re-arm is needed). `stop()` was chosen over
`disarm()`-only sequencing so every caller, including the low-level single-motor usage example,
gets the fix automatically.

Verified on hardware: `telemetry_settled_300/600` and `two_channel_divergent_300/600` (single
and multiple bidirectional motors, the latter the scenario that first showed the bug) all
recover audibly with no reset; three repeated arm/spin/disarm cycles in one session, run twice,
all decoded 100% CRC-valid with real spin, confirming the pin-reclaim path with a real ESC
attached; `test_pio_lifecycle.py` (no ESC, GPIO 10/11) re-armed 15 more times after 30
build/arm/disarm cycles, asserting the disarmed bidirectional line now reads low (previously
asserted high) — all passed, including a fresh capture and a restarted sequence on every re-arm.
Full investigation and verification trail: `git show ffae59d..ca1db7d` (commits on branch
`fix/bidir-disarm-line-state`; the range survives a merge but not a squash).

**Ruled out along the way, not the cause:**
- An RP2350 silicon erratum (E9): can only hold a pulled-*down* pad high through leakage; the
  driver's pull-up isn't affected by it. Raised only because a side diagnostic happened to use a
  forced pull-down and read high — that diagnostic's own result, not this symptom.
- Two channels contending for one shared ESC CPU, or cross-talk between the two signal lines:
  both require two bidirectional motors running together. A single bidirectional motor,
  disarmed completely alone, hangs the same way — proven on the bench (see D2) — which rules out
  both regardless of the argument for or against either.
- A real `disarm()`/`update()` race on another core: fixed in `5342e9c` regardless, since it was
  a genuine bug, but the ESC still got stuck after that fix landed, so it wasn't this.
- Releasing the line to `Pin.IN, PULL_UP` in `stop()` instead of leaving it alone: tried and
  reverted 2026-09-24 — a released line is exactly the failing state, so this "fix" changed
  nothing.
- A hard `machine.reset()` in the harness's own cleanup, considered earlier as a possible
  workaround, is no longer needed.

**Open, separate issue.** One `two_channel_divergent_600` run after this fix landed showed
channel 1 replying with valid telemetry (97.7% CRC-valid) but at the at-rest eRPM value — the
motor did not spin, though the ESC still recovered normally after disarm. A second, identical
run spun both motors cleanly. Genuinely intermittent, not reproduced enough to explain, and not
caused by this fix (the ESC recovered either way). Untested lead, not a conclusion: the pull-up
is applied at `BidirectionalDShot.__init__`, before `arm()`, which is a window the fix doesn't
touch.

## References

- [Brushless Whoop - Bidirectional DShot](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
- [Betaflight - DShot RPM Filtering](https://www.betaflight.com/docs/wiki/guides/current/DSHOT-RPM-Filtering)
- [Bluejay Firmware](https://github.com/mathiasvr/bluejay)
- [Bluejay Configurator](https://github.com/mathiasvr/bluejay-configurator)
- [BLHeli_S to Bluejay Flashing Guide](https://github.com/mathiasvr/bluejay/wiki/Flashing)
- [AM32 Firmware](https://github.com/am32-firmware/AM32) - `Src/dshot.c`, `Src/main.c`, `Src/signal.c`
- [Betaflight Pico/RP2350 DShot PR #14618](https://github.com/betaflight/betaflight/pull/14618/files) - `src/platform/PICO/dshot.pio`, `dshot_bidir_pico.c`, `dshot_pico.c`
- DShot Protocol Specification: `specification/DSHOT_PROTOCOL.md`
