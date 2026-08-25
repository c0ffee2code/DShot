# Diagnostic probe - NOT a driver test, NOT part of the bidirectional
# checkpoint sequence. Spins no motors and needs no bench/power supply -
# just the Pico connected over USB.
#
# Purpose: measure the actually-achieved PIO clock frequency for a state
# machine constructed with freq=3_000_000 (dshot_bidir_rx's rate at
# DSHOT300, per dshot_pio.py's `rx_speed = dshot_speed * 5 // 4`), to check
# whether the RX capture's observed ~4/5 effective-rate shortfall (see
# decision/ADR-002-bidirectional-dshot.md's Implementation Update and
# tonight's advisor consultation) traces back to the requested frequency
# not actually landing - e.g. a wrong system-clock assumption - rather than
# to the PIO program's cycle accounting (already re-verified by hand).
#
# Method: a scratch PIO program counts down a large, Python-supplied number
# of cycles (one `jmp(x_dec, ...)` per cycle) then pushes once. Python times
# wall-clock from writing that count to receiving the push. achieved_freq =
# N_cycles / elapsed_seconds. No FIFO throughput concerns (only one word is
# ever pushed), so this is precise regardless of how fast MicroPython itself
# can poll.

from machine import freq
from rp2 import PIO, StateMachine, asm_pio
import utime

N_CYCLES = 300_000  # ~100ms at the intended 3MHz, ~125ms at 2.4MHz - either way a quick, precise measurement
REQUESTED_FREQ = 3_000_000


@asm_pio(autopush=False)
def clock_probe():
    pull(block)                  # Python supplies N via put()
    mov(x, osr)          [0]
    label("loop")
    jmp(x_dec, "loop")   [0]     # 1 cycle/iteration, N total
    push(block)                  # signal completion (content is irrelevant)


def main():
    print("=== RX Clock Probe (no bench/power supply needed) ===")
    print(f"machine.freq() (system clock): {freq()} Hz")
    print(f"Requesting PIO freq={REQUESTED_FREQ} Hz for the scratch state machine...")

    sm = StateMachine(0, clock_probe, freq=REQUESTED_FREQ)
    sm.active(1)

    start_us = utime.ticks_us()
    sm.put(N_CYCLES)
    sm.get()  # blocks until the countdown completes and pushes
    elapsed_us = utime.ticks_diff(utime.ticks_us(), start_us)

    sm.active(0)

    elapsed_s = elapsed_us / 1_000_000
    achieved_freq = N_CYCLES / elapsed_s

    print(f"{N_CYCLES} cycles took {elapsed_us}us -> achieved freq = {achieved_freq:,.0f} Hz")
    print(f"Ratio achieved/requested = {achieved_freq / REQUESTED_FREQ:.4f}")
    print(f"(For reference: 2,400,000 Hz would be ratio {2_400_000 / REQUESTED_FREQ:.4f})")
    print()
    print("=== Probe Complete ===")


main()
