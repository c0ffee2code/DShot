# BUG-004: Bidirectional frames end on a released, slowly rising edge, and the receiver trusts a fixed delay after it

**Status:** Both fixes IMPLEMENTED and bench-verified 2026-09-27 (`driver/dshot_pio.py`). **Does
NOT clear BUG-002** - see "Bench result" below. Kept regardless: it removes a real, Betaflight-
documented risk and caused no regression across 6 previously-passing scenarios.
**Severity:** Medium-High
**Component:** `driver/dshot_pio.py` (`dshot_bidir_tx`, `dshot_bidir_rx_frame`)
**Found:** 2026-09-27, Betaflight Pico comparison (`specification/AM32_ARMING_AND_BETAFLIGHT.md`, B4)

## Summary

**Transmitter.** When a frame's last bit is a "1", `dshot_bidir_tx` releases the pin while it is
still low (`dshot_pio.py:85`, `set(pindirs, 0).side(0)`). The final rising edge is then made only
by the two weak pull-ups: the Pico's and AM32's. Every zero-throttle frame ends in "1", and so does
throttle 100.

**Receiver.** `dshot_bidir_rx_frame` waits a fixed 27 cycles after the release
(`dshot_pio.py:212`), then triggers on the first low (`:213`). That is **2.18 µs at DSHOT600**
and 4.35 µs at DSHOT300.

**Why it matters.** Two things depend on that edge:

- **Our receiver.** If the rise is slow, or noise pulls the weakly held line back down, the pin
  still reads low after the predelay. The receiver then captures the tail of our own frame, and
  the capture fails CRC.
- **AM32.** It measures each frame's length to its last edge, against a ±1/16 window learned from
  zero-throttle frames ([`dshot.c#L74-L83`][am32-frame], [`signal.c#L166-L174`][am32-avg]).
  - An extra noise edge shifts its 32-edge capture, so it decodes garbage or rejects the frame on
    length.
  - A frame ending in "0" is driven and therefore shorter. It is rejected if the slow edge lags by
    more than ~1.1 µs at DSHOT600 (~2.2 µs at DSHOT300).

Both margins halve at DSHOT600.

**Betaflight's Pico port** ends every frame with the line **driven high**, waits 21 cycles
(~280 ns), then releases ([`dshot.pio#L84-L104`][bf-pio-tx]). Its receiver waits for *high* before
it waits for the falling edge ([`dshot.pio#L109-L112`][bf-pio-rx]).

## Why it is the BUG-002 suspect

- BUG-002 fails only with DSHOT600 and two bidirectional motors: two released lines (GPIO6 and
  GPIO8) side by side while two motors switch. In the passing one-motor scenarios GPIO8 is
  unidirectional, so always driven.
- BUG-002's CRC-failing prefix is exactly what our receiver produces when it captures its own
  frame tail.
- Not yet explained: why the ESC never recovers once the lines are quiet (see BUG-002).

## Fix plan

Two independent changes, each free: same instruction counts, and the TX/RX block stays at 32/32.
Apply and bench-test **one at a time**.

1. **Transmitter: drive the edge, then release.** Reorder the "one" tail so the line goes high
   while still driven:

   ```python
   jmp(y_dec, "bitloop")      .side(0)   [2] # "one" path: 3 cycles LOW (6 in total, unchanged)
   irq(rel(1))                .side(1)   [0] # drive HIGH (a driven rising edge) and signal RX
   set(pindirs, 0)            .side(1)   [1] # then release a line that is already high
   jmp("frame_start")         .side(1)   [0]
   ```

   - The "zero" path already drives high for 3 cycles before releasing, so it is unchanged.
   - RX now sees the IRQ one TX cycle (0.21 µs at DSHOT600) before the release, well inside its
     27-cycle predelay.
   - Update `dshot_bidir_tx`'s comment block and ADR-002's TX description.
2. **Receiver: wait for high before waiting for low.** Replace the predelay
   `nop() [26]` with `wait(1, pin, 0) [26]`. It keeps the same lower bound once the line is high,
   and never starts looking for a falling edge while the line is still low. It needs no extra
   instruction.
   - A silent ESC behaves as today: the receiver still falls through to our next frame, and the
     BUG-008 failsafe catches that.
   - `scripts/simulate_frame_receiver.py` models the receiver from `wait(0, pin, 0)` on, so only
     its header comment changes.

## Verification

- Unit: `tests/unit/test_dshot_packet.py` style check that the assembled programs keep 13/19
  instructions. Not added - `tests/unit/fakes.py` deliberately does not assemble PIO programs
  (`asm_pio` returns the decorated function unrun), so instruction count can only be checked by
  hand or by deploying to real hardware; both instruction counts were confirmed unchanged by hand
  after editing.
- Bench, 2026-09-27:
  1. **Change 1 alone**, 3 runs each on `two_channel_divergent_600` and `two_channel_gc_600`:
     6/6 still failed, identical signature (both motors stuck at 917; one `two_channel_gc_600` run
     had only one motor affected).
  2. **Change 1 + change 2 together**, 3 runs on `two_channel_divergent_600`: 3/3 still failed
     (one run again had only one motor affected - motor 2 spun cleanly at 33,557 eRPM while
     motor 0 stayed stuck).
  3. Regression: all 6 previously-passing scenarios re-run once each with both changes in place -
     all still pass, no CRC-valid-rate or eRPM regression.
  4. Not done: a logic analyzer/scope check of the actual GPIO6/8 rise time. The fix is applied on
     the strength of the PIO-semantics argument in section 4 of
     `AM32_ARMING_AND_BETAFLIGHT.md`, not a direct electrical measurement - worth doing if this
     bug is revisited, to confirm the release edge is actually clean now even though it didn't
     explain BUG-002.

**Conclusion:** both changes are real correctness improvements (they match Betaflight's Pico port
and remove an actual released/floating-edge risk) and are kept, but neither alone nor together
clears BUG-002 - see that report's updated findings. BUG-002 remains open; its cause is not fully
explained by B4.

## Related

- BUG-002

[am32-frame]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/dshot.c#L74-L83
[am32-avg]: https://github.com/am32-firmware/AM32/blob/55c96847a0cddfee9852eb65d2b10e58f563b3d7/Src/signal.c#L166-L174
[bf-pio-tx]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/platform/PICO/dshot.pio#L84-L104
[bf-pio-rx]: https://github.com/betaflight/betaflight/blob/e5071ce4ee436a7d8778e4596b6398f3d705d114/src/platform/PICO/dshot.pio#L109-L112
