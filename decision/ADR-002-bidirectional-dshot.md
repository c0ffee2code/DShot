# ADR-002: Bidirectional DShot Implementation

**Status:** Deferred
**Date:** 2026-02-01
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
| + Start bit | 21 | Final transmission size |

**GCR Symbol Table:**

| Nibble | GCR | Nibble | GCR |
|--------|-----|--------|-----|
| 0x0 | 0x19 | 0x8 | 0x16 |
| 0x1 | 0x1B | 0x9 | 0x0E |
| 0x2 | 0x12 | 0xA | 0x0F |
| 0x3 | 0x13 | 0xB | 0x07 |
| 0x4 | 0x1D | 0xC | 0x17 |
| 0x5 | 0x15 | 0xD | 0x0D |
| 0x6 | 0x14 | 0xE | 0x05 |
| 0x7 | 0x1C | 0xF | 0x06 |

### Bitrate Calculation

GCR response is transmitted at 5/4× the DShot bitrate:

| DShot Variant | TX Bitrate | RX Bitrate (GCR) | Bit Period |
|---------------|------------|------------------|------------|
| DShot300 | 300 kbit/s | 375 kbit/s | 2.67µs |
| DShot600 | 600 kbit/s | 750 kbit/s | 1.33µs |
| DShot1200 | 1200 kbit/s | 1500 kbit/s | 0.67µs |

### eRPM Decoding

```python
# GCR decode (XOR with shifted self)
decoded = gcr_value ^ (gcr_value >> 1)

# Extract fields
crc = decoded & 0x0F
mantissa = (decoded >> 4) & 0x1FF  # 9 bits
exponent = (decoded >> 13) & 0x07  # 3 bits

# Calculate period in microseconds
period_us = mantissa << exponent

# Convert to eRPM
if period_us > 0:
    erpm = 60_000_000 / period_us

# Convert to mechanical RPM (motor poles / 2)
rpm = erpm / (motor_poles / 2)  # 1104 motors typically have 12 poles
```

## PIO Implementation Analysis

### Resource Requirements

**Current (TX only):**
- 1 state machine per motor
- 2 motors = 2 SMs
- 6 SMs available

**Bidirectional options:**

| Approach | SMs per Motor | Total (2 motors) | Remaining |
|----------|---------------|------------------|-----------|
| **A. Mode switching** | 1 | 2 | 6 |
| **B. Dual SM (TX+RX)** | 2 | 4 | 4 |
| **C. Shared RX** | 1.5 | 3 | 5 |

### Recommended: Option B (Dual SM per Motor)

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
- **Total: ~85µs per motor** (effective update rate ~11.7kHz per motor)

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

### Current State
- Standard (unidirectional) DShot works reliably
- No eRPM telemetry available
- Motor speed estimation would require external sensor (optical/magnetic encoder)

### When Implemented
- Real-time eRPM feedback (~11.7kHz update rate)
- Closed-loop speed control capability
- Extended telemetry (temperature, voltage, current) with EDT
- ~50% reduction in effective command rate (acceptable trade-off)

## References

- [Brushless Whoop - Bidirectional DShot](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
- [Betaflight - DShot RPM Filtering](https://www.betaflight.com/docs/wiki/guides/current/DSHOT-RPM-Filtering)
- [Bluejay Firmware](https://github.com/mathiasvr/bluejay)
- [Bluejay Configurator](https://github.com/mathiasvr/bluejay-configurator)
- [BLHeli_S to Bluejay Flashing Guide](https://github.com/mathiasvr/bluejay/wiki/Flashing)
- DShot Protocol Specification: `specification/DSHOT_PROTOCOL.md`
