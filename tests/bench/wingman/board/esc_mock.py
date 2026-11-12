# EXAMPLE APPLICATION CODE - not part of the DShot library; a test fixture: nothing in the driver or the command loop
# uses it.
#
# Runs on the SECOND Pico (the wingman) and answers the main Pico's DShot frames the way an AM32 ESC does, with
# replies recorded from a real ESC (esc_mock_replies_dshot600.bin, built by tests/bench/wingman/tools/esc_mock_fixture.py), so the main Pico's whole
# reply path runs with no ESC and no motor. Wiring is the wingman's: the same GPIO numbers on both boards (ESC signal
# lines 6-9) and a shared ground. The ESC itself must be disconnected from those lines.
#
# What it does per line, with no CPU work per frame (one PIO state machine and one DMA channel per line):
#   1. waits until the line has been high for about 164 us, so only a frame's first falling edge can start it;
#   2. samples the 11 throttle bits of the frame: 0.9 us after each bit's falling edge the pin is still low for a
#      1-bit (low 1.25 us) and already high for a 0-bit (low 0.625 us);
#   3. all 11 bits zero (throttle 0) -> replies with AM32's "not running" word, else with the next recorded reply;
#   4. waits so the reply's first falling edge lands 48.8 us after the frame's first edge (median of the real ESC) and
#      shifts the 21 reply levels out at 1.2845 us per bit (the real ESC's 1.284 us).
# The mock drives the line, both levels, only during its reply window, which starts about 22 us after the main Pico's
# frame has ended and its pin is released; after the last level it releases the line, and the rest of the time the pin
# is a plain input. (Releasing the line for the high levels, so that only the main Pico's pull-up made them, was
# tried first: the rises came about 1 us late and short highs vanished; the real ESC's rises are as crisp as its falls.)
#
# Not like the real ESC: it answers from the first frame (the ESC is silent for 104 frames and for ~301 ms after its
# arming gate), and the speed is replayed, not derived from the throttle. See README.md.
#
# Timing arithmetic (one cycle = 80.3 ns at SM_FREQ; the input synchroniser delays what an instruction reads by 2-3
# cycles, so the pin is read at about n + 1.5 cycles after the physical edge for a wait + nop [n] + jmp pin):
#   sample point  n = SAMPLE_NOP = 10 -> 11.5 cycles = 0.92 us after the fall (0-bit high from 0.625, 1-bit low to 1.25)
#   reply start   the 11th sample is 10 bit periods + 0.92 us after the first fall; the code after it takes about
#                 21.5 cycles up to the delay loop and the delay loop + trim 379 cycles (DELAY_PASSES, TRIM_NOP), so the
#                 reply's first fall comes 48.8 us after the frame's first fall (real ESC: 48.5-51.0 us).

import struct
import uctypes
import rp2
from machine import Pin, mem32
import esc_mock_format

SM_FREQ = 12_456_000                # 16 cycles per reply bit: 16 / 12.456 MHz = 1.2845 us
BIT_CYCLES = 16
SAMPLE_NOP = 10                     # nop [n] before reading the pin
DELAY_PASSES = 22                   # (22 + 1) passes of 16 cycles ...
TRIM_NOP = 10                       # ... plus one nop [10] = 379 cycles
FIRST_SM = 0                        # PIO block 0 (the recorder uses block 2)
PIO0_BASE = 0x50200000
TXF0 = PIO0_BASE + 0x10
IO_BANK0_BASE = 0x40028000          # GPIOn_CTRL = base + 8 * n + 4, FUNCSEL in bits 4:0 (RP2350 datasheet)
FUNCSEL_PIO0 = 6
RING_BITS = 13                      # 2048 words = [not_running, reply] x 1024
RING_BYTES = 1 << RING_BITS
COUNT = 0x0FFFFFFF


# One reply per frame, in this order (see the file header for the why). x counts the throttle bits left, y is first
# the quiet-gap outer counter, then the "a 1-bit was seen" flag, then the reply bit counter. The ISR holds the
# not-running word, the OSR the recorded reply (or, for throttle 0, a copy of the not-running word). 32 of the block's 32
# instruction slots.
def make_program(sample_nop, delay_passes, trim_nop):
    """The program with its timing constants as closure variables: the PIO assembler cannot see module globals."""
    @rp2.asm_pio(out_init=rp2.PIO.IN_LOW, set_init=rp2.PIO.IN_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=False)
    def esc_mock_reply():
        label("rearm")
        set(y, 31)
        label("outer")
        set(x, 31)
        label("inner")
        jmp(pin, "high")
        jmp("rearm")                       # the line went low: start counting again
        label("high")
        jmp(x_dec, "inner")
        jmp(y_dec, "outer")                # 32 x 32 passes of 2 cycles = ~164 us of a high line
        pull()
        mov(isr, osr)                      # not-running word
        pull()                             # this frame's recorded reply
        set(y, 0)
        set(x, 10)                         # 11 throttle bits
        wait(0, pin, 0)                    # the frame's first falling edge
        label("bit")
        nop()                      [sample_nop]
        jmp(pin, "zero")                   # high already: a 0-bit
        set(y, 1)                          # still low: a 1-bit, so a nonzero throttle
        label("zero")
        jmp(x_dec, "more")
        jmp("go")                          # the 11th sample: no wait for its rise, the reply time must not depend on it
        label("more")
        wait(1, pin, 0)
        wait(0, pin, 0)
        jmp("bit")
        label("go")
        jmp(y_dec, "real")
        mov(osr, isr)                      # throttle 0: answer with the not-running word
        label("real")
        set(x, delay_passes)
        label("delay")
        nop()                      [14]
        jmp(x_dec, "delay")
        nop()                      [trim_nop]
        set(pindirs, 1)                    # the pin latch is low: the line is driven low, the reply's first level
        set(y, 20)                         # 21 reply levels
        label("outbit")
        out(pins, 1)               [14]    # the level, driven
        jmp(y_dec, "outbit")
        set(pindirs, 0)                    # release; the wrap goes back to "rearm"
    return esc_mock_reply


PROGRAM = make_program(SAMPLE_NOP, DELAY_PASSES, TRIM_NOP)


def aligned_ring():
    raw = bytearray(2 * RING_BYTES)
    offset = (-uctypes.addressof(raw)) % RING_BYTES
    return raw, memoryview(raw)[offset:offset + RING_BYTES]


def funcsel(gpio):
    return mem32[IO_BANK0_BASE + 8 * gpio + 4] & 0x1F


class Line:
    def __init__(self, index, gpio, frames):
        self.gpio = gpio
        self.raw, self.ring = aligned_ring()
        not_running = esc_mock_format.pio_word(esc_mock_format.NOT_RUNNING_FRAME)
        for k, frame in enumerate(frames):
            struct.pack_into("<II", self.ring, 8 * k, not_running, esc_mock_format.pio_word(frame))
        pin = Pin(gpio, Pin.IN)
        self.sm = rp2.StateMachine(FIRST_SM + index, PROGRAM, freq=SM_FREQ,
                                   in_base=pin, out_base=pin, set_base=pin, jmp_pin=pin)
        self.dma = rp2.DMA()
        ctrl = self.dma.pack_ctrl(size=2, inc_read=True, inc_write=False, ring_size=RING_BITS, ring_sel=0,
                                  treq_sel=FIRST_SM + index, irq_quiet=True)
        self.dma.config(read=self.ring, write=TXF0 + 4 * index, count=COUNT, ctrl=ctrl, trigger=True)

    def start(self):
        self.sm.active(1)

    def stop(self):
        self.sm.active(0)
        self.dma.active(0)
        self.dma.close()
        Pin(self.gpio, Pin.IN)             # back to a plain input: the pin is released for good


class EscMock:
    def __init__(self, pins, fixture, lines):
        """pins: the GPIO numbers to answer on; lines[i]: which line of the fixture's recorded replies pin i plays. Both come from
        the scenario (tests/bench/wingman/tools/wingman_plan.py); nothing here knows which pins the bench uses."""
        if len(pins) != len(lines):
            raise ValueError("%d pins but %d fixture lines" % (len(pins), len(lines)))
        with open(fixture, "rb") as f:
            recorded = esc_mock_format.parse_fixture(f.read())
        for line in lines:
            if line >= len(recorded):
                raise ValueError("the fixture has %d lines, line %d asked for" % (len(recorded), line))
        if len(recorded[0]) * 8 != RING_BYTES:
            raise ValueError("the fixture must hold %d replies per line" % (RING_BYTES // 8))
        self.lines = [Line(i, gpio, recorded[lines[i]]) for i, gpio in enumerate(pins)]
        for line in self.lines:
            line.start()

    def state(self):
        """What a check needs to see: per line, whether the state machine runs, the pin's function select (6 = PIO
        block 0) and how many words wait in the TX FIFO."""
        return [{"gpio": l.gpio, "active": l.sm.active(), "funcsel": funcsel(l.gpio), "tx_fifo": l.sm.tx_fifo()}
                for l in self.lines]

    def stop(self):
        for line in self.lines:
            line.stop()


def start(pins, fixture, lines):
    return EscMock(pins, fixture, lines)


def guard(pins):
    """A tick() for observer.run(): on its first call it raises unless every mocked pin still belongs to PIO
    block 0, so a mock that lost its pins cannot produce a session that looks fine."""
    state = {"checked": False}

    def tick():
        if state["checked"]:
            return
        state["checked"] = True
        for gpio in pins:
            if funcsel(gpio) != FUNCSEL_PIO0:
                raise RuntimeError("ESC mock: GPIO %d is not on PIO block 0 (function select %d)" % (gpio, funcsel(gpio)))
    return tick
