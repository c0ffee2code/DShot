# ADR-003: DShot Special Commands Implementation

**Status:** Implemented (not functional on current hardware)
**Date:** 2026-02-01
**Context:** Implementing DShot special commands beyond throttle control

## Context

DShot protocol reserves throttle values 1-47 for special commands. These commands control ESC behavior such as beeps, motor spin direction, and settings persistence.

The test bench needs:
1. **Beep commands** - Audio feedback for testing/debugging
2. **Spin direction** - Configure motor rotation for test scenarios

## Hardware Limitation Discovered

**The current ESC firmware does not support DShot special commands.**

| Component | Details |
|-----------|---------|
| ESC | JHEMCU Brushless Wing Dual 40A 2-in-1 |
| Firmware | BLHeli_S G-H-30 V16.7 |
| Behavior | Treats command values 1-47 as throttle, not commands |

### Testing Results

| Test | Expected | Actual |
|------|----------|--------|
| Beep commands (1-5) | Audible tones | Motor attempts to spin (low throttle) |
| Spin direction (20/21) | Direction change | No effect, motor spins same direction |

### Packet Encoding Verification

Packet encoding was verified correct:
```
BEEP1 (value=1):     packet=0x0022
THROTTLE_70 (value=70): packet=0x08C4
```

The packets are correctly formatted. The ESC firmware simply doesn't recognize values 1-47 as special commands - it interprets all values as throttle.

### Compatible Firmware

DShot commands require ESC firmware that implements them:
- **Bluejay** - Free, open source, requires flashing
- **BLHeli_32** - Native support, requires different ESC hardware
- **AM32** - Free, open source, requires ARM-based ESC

## Decision

Implement special commands in `DShotPIO` driver. The code is ready for compatible ESCs, but non-functional on current hardware.

### API Design

```python
class DSHOT_CMD:
    """DShot special command codes (0-47)."""
    MOTOR_STOP = 0
    BEEP1 = 1
    BEEP2 = 2
    BEEP3 = 3
    BEEP4 = 4
    BEEP5 = 5
    SPIN_DIRECTION_NORMAL = 20
    SPIN_DIRECTION_REVERSED = 21


class DShotPIO:
    def sendCommand(self, command, telemetry=False):
        """
        Send a DShot special command (0-47).

        Note: Requires ESC firmware that supports DShot commands
        (Bluejay, BLHeli_32, AM32). Stock BLHeli_S may not support.
        """
        ...
```

### Implementation Status

| Component | Status |
|-----------|--------|
| `DSHOT_CMD` constants | Implemented |
| `sendCommand()` method | Implemented |
| Beep functionality | Code ready, ESC incompatible |
| Spin direction | Code ready, ESC incompatible |

## Consequences

### Current State

- Code is implemented and ready for compatible ESCs
- No test programs included (removed due to hardware incompatibility)
- Motors already spin in opposite directions, no configuration needed

### Future

When compatible ESCs are available (Bluejay flashed or BLHeli_32 hardware):
- Add test programs for beeps and spin direction
- Verify command functionality
- Consider adding utility scripts for ESC configuration

## Command Reference

For future reference when compatible ESCs are available:

### Beep Commands (1-5)

| Code | Command | Wait After |
|------|---------|------------|
| 1 | BEEP1 | 260ms |
| 2 | BEEP2 | 260ms |
| 3 | BEEP3 | 260ms |
| 4 | BEEP4 | 280ms |
| 5 | BEEP5 | 1020ms |

### Spin Direction Commands

| Code | Command | Repeat | Notes |
|------|---------|--------|-------|
| 20 | SPIN_DIRECTION_NORMAL | 6x | Preferred |
| 21 | SPIN_DIRECTION_REVERSED | 6x | Preferred |
| 7 | SPIN_DIRECTION_1 | 6x | Legacy, ESC-dependent |
| 8 | SPIN_DIRECTION_2 | 6x | Legacy, ESC-dependent |

## References

- DShot Protocol Specification: `specification/DSHOT_PROTOCOL.md`
- [BLHeli_S Command Reference](https://github.com/bitdump/BLHeli/blob/master/BLHeli_S%20SiLabs/Dshotprog%20spec%20BLHeli_S.txt)
- [Betaflight DShot Commands](https://betaflight.com/docs/development/API/Dshot)
