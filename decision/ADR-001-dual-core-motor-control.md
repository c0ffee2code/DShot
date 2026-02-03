# ADR-001: Dual-Core Architecture for Reliable Motor Control

**Status:** Accepted
**Date:** 2026-02-01
**Context:** DShot driver for flight control test bench with multiple motors

## Context

The test bench controls 2 drone motors via DShot protocol. Each motor has an independent ESC and signal wire. The current implementation has unreliable arming behavior - often one motor arms successfully while the other fails (indicated by beeping).

### Problem Analysis

ESCs require continuous DShot commands at high frequency (every 1-2ms) to:
1. Complete the arming sequence (typically 300ms of throttle=0 commands)
2. Remain armed (timeout if no valid commands received)

The current single-threaded implementation has gaps in command transmission caused by:
- UI updates (display rendering)
- Button polling and debouncing
- Sleep intervals in the main loop (50ms in disarmed state)
- Any other application logic

These gaps cause ESCs to reset their arming counters or disarm entirely.

### Future Requirements

The test bench will implement closed-loop control where motors receive different thrust levels based on:
- Desired attitude setpoints (pitch, roll)
- Yaw rotation commands
- Sensor feedback (magnetic encoder, IMU)

This means individual motors will have independent, continuously varying throttle values - not synchronized identical commands.

## Decision

Implement a **three-layer architecture** using the RP2040/RP2350 dual-core capability:

```
┌─────────────────────────────────────────────────────────────┐
│                      CLIENT CODE (Core 0)                   │
│                                                             │
│  - User interface (display, buttons)                        │
│  - Control algorithms (PID, attitude control)               │
│  - Sensor reading and fusion                                │
│  - Throttle calculations per motor                          │
│                                                             │
│                 motorGroup.setThrottle(index, value)        │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                  MOTOR GROUP FACADE (spans both cores)      │
│                                                             │
│  Core 0 side:                                               │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  setThrottle(index, value)                          │    │
│  │  setAllThrottles([v1, v2, ...])                     │    │
│  │  arm() / disarm()                                   │    │
│  │  start() / stop()                                   │    │
│  └──────────────────────┬──────────────────────────────┘    │
│                         │                                   │
│        Shared throttle array (lock-free writes)             │
│                         │                                   │
│  Core 1 side:           ▼                                   │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Dedicated loop running at 1kHz:                    │    │
│  │    - Read current throttle values                   │    │
│  │    - Send DShot command to each motor               │    │
│  │    - Maintain precise timing                        │    │
│  └─────────────────────────────────────────────────────┘    │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                     DRIVER (DShotPIO)                       │
│                                                             │
│  - PIO state machine management                             │
│  - DShot packet encoding (throttle + telemetry + CRC)       │
│  - Single responsibility: send one command to one motor     │
│                                                             │
│         motor.sendThrottleCommand(throttle_value)           │
└─────────────────────────────────────────────────────────────┘
```

### Layer Responsibilities

| Layer | Responsibilities | Core |
|-------|------------------|------|
| **Client** | Application logic, UI, control algorithms, sensor fusion | Core 0 |
| **MotorGroup Facade** | Thread lifecycle, timing guarantees, command queue, arming orchestration | Both |
| **DShotPIO Driver** | Packet encoding, PIO control, single-motor command transmission | Core 1 (called from) |

### Why Three Layers

1. **DShotPIO remains simple**: No threading awareness, no state beyond PIO. Easy to test and understand.

2. **Facade absorbs complexity**: Threading, synchronization, timing - all hidden from client code.

3. **Client stays focused**: Control algorithms don't need to know about DShot timing requirements.

4. **Independent throttle values**: Each motor has its own throttle in the shared array, supporting differential thrust for attitude control.

## Sub-Decision: Lock-Free Shared State

**Decision:** Use lock-free writes for throttle values instead of mutex synchronization.

### Rationale

On ARM Cortex-M0+/M33 (RP2040/RP2350), aligned integer writes are atomic at the machine level. MicroPython integers that fit in a machine word (32-bit) are written atomically.

```python
# This is atomic - no lock needed
self._throttles[motor_index] = value
```

### Trade-offs

| Aspect | With Lock | Lock-Free |
|--------|-----------|-----------|
| **Latency** | Lock acquisition overhead | Zero overhead |
| **Complexity** | Need lock management | Simpler code |
| **Consistency** | All values read together | Each value independent |
| **Worst case** | Brief contention | Core 1 sees stale value for 1 cycle (1ms) |

### Why Lock-Free is Acceptable

1. **Independent motors**: Each throttle value is independent. No requirement to update all motors atomically together.

2. **High update rate**: Core 1 runs at 1kHz. A 1-cycle-stale value means at most 1ms delay in applying new throttle.

3. **Control loop tolerance**: Flight control loops typically run at 100-500Hz. The 1ms potential staleness is well within tolerance.

4. **Simpler debugging**: No deadlock risk, no lock ordering concerns.

### Implementation Pattern

```python
class MotorThrottleGroup:
    def __init__(self, pins, dshot_speed=DSHOT_SPEEDS.DSHOT600):
        # pins: list of Pin objects, e.g. [Pin(4), Pin(5)]
        # Create DShotPIO instances internally
        self._motors = [DShotPIO(i, pin, dshot_speed) for i, pin in enumerate(pins)]
        self._throttles = array.array('H', [0] * len(pins))  # Unsigned 16-bit
        self._running = False

    def setThrottle(self, motor_index, value):
        """Called from Core 0 - atomic write"""
        self._throttles[motor_index] = value

    def _core1_loop(self):
        """Runs on Core 1"""
        while self._running:
            for i, motor in enumerate(self._motors):
                motor.sendThrottleCommand(self._throttles[i])  # Atomic read
            utime.sleep_us(1000)
```

Using `array.array('H', ...)` ensures:
- Contiguous memory layout
- Guaranteed 16-bit aligned access
- No Python object overhead per element

## Consequences

### Positive

- **Reliable arming**: Commands flow continuously regardless of Core 0 activity
- **Consistent timing**: 1kHz update rate maintained even during heavy UI updates
- **Scalable**: Pattern works for 2, 4, or more motors
- **Clean separation**: Each layer has single responsibility
- **Testable**: DShotPIO can be tested independently without threading

### Negative

- **Core 1 dedicated**: Cannot use Core 1 for other tasks while motors are active
- **Debugging harder**: Core 1 exceptions don't appear in REPL
- **Thread lifecycle**: Must properly start/stop Core 1 thread to avoid resource leaks
- **MicroPython limitation**: `_thread` module has limited features (no join, limited stack size control)

### Risks and Mitigations

| Risk | Mitigation |
|------|------------|
| Core 1 crash goes unnoticed | Add heartbeat flag that Core 0 monitors |
| Thread not stopped on exception | Use try/finally in main code to call `stop()` |
| Stack overflow on Core 1 | Keep Core 1 loop minimal, no deep call chains |

## Implementation Notes

### Arming Sequence

The facade should provide an `arm()` method that:
1. Sets all throttles to 0
2. Waits for arming duration (300-500ms depending on ESC firmware)
3. Returns success (future: verify via bidirectional DShot telemetry)

Since Core 1 is continuously sending, arming just requires holding throttle at 0 for the required duration - no special timing code needed.

### Disarming

Set all throttles to 0. ESCs interpret continuous throttle=0 as disarmed state.

### Emergency Stop

```python
def emergencyStop(self):
    """Immediate stop - called from Core 0"""
    for i in range(len(self._throttles)):
        self._throttles[i] = 0
```

This is safe because each write is atomic and Core 1 will pick up the zeros within 1ms.

## Verification

**Status:** Verified on hardware (2026-02-01)

### Test Hardware

| Component | Model | Specifications |
|-----------|-------|----------------|
| **Controller** | Raspberry Pi Pico 2 | RP2350, dual ARM Cortex-M33 |
| **Motors** | BetaFPV Lava Series 1104 (x2) | 7200KV, 5g weight |
| **ESC** | JHEMCU Brushless Wing Dual 40A 2-in-1 | 40A×2 channels, 2-6S, BLHeli_S G-H-30 V16.7 |
| **Protocol** | DShot600 | 4.8MHz PIO clock |

### Test Results

| Test | Result | Notes |
|------|--------|-------|
| `test_dshot_single_motor.py` | Pass | Low-level driver, 1ms command interval |
| `test_motor_throttle_group.py` | Pass | Facade, both motors arm reliably |

### Verified Parameters

Tested with specific hardware (JHEMCU 40A ESC + test bench motors). Not tested with alternative hardware.

| Parameter | Value | Notes |
|-----------|-------|-------|
| Minimum throttle | 70 | Hardware-specific; values 50-69 unreliable on test bench |
| Command interval (single-threaded) | 1ms | 2ms caused occasional dropouts |
| Command interval (facade) | 1ms | Core 1 maintains steady 1kHz |
| Arming duration | 500ms | Reliable with BLHeli_S firmware |

### Key Finding

The dual-core architecture completely eliminated arming failures. Previously, arming succeeded ~50% of the time with one motor often failing. After implementing `MotorThrottleGroup`, both motors arm reliably every time.

## References

- [RP2040 Datasheet - Dual Core](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [MicroPython _thread module](https://docs.micropython.org/en/latest/library/_thread.html)
- [ARM Cortex-M0+ Memory Ordering](https://developer.arm.com/documentation/dui0662/latest/)
- DShot Protocol Specification: `specification/DSHOT_PROTOCOL.md`
- [BetaFPV Lava 1104 Motors](https://betafpv.com/products/lava-series-1104-brushless-motors)
- [JHEMCU Dual 40A ESC](https://www.jhemcu.com/e_productshow/?81-JHEMCU-BRUSELESS-WING-DUAL-40A-2IN1-ESC-81.html)
