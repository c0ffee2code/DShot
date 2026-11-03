# EXAMPLE APPLICATION CODE - not part of the DShot library, and a diagnostic: nothing in the
# driver or the command loop uses it.
#
# A logic analyzer inside the Pico. One state machine of the spare PIO block reads up to four
# consecutive GPIO pins as inputs at SAMPLE_HZ (1 MHz by default) while other state machines drive
# those pins, packs 8 samples into each 32-bit word, and a DMA channel copies the words into a
# buffer with no help from either core. Afterwards find_edges() turns the buffer into an edge
# list for pin_timing.py.
#
# It shows the level of the Pico's own pad, which is where our frames leave: it measures how
# our software spaced the frames, not what a cable does to them. At the default 1 MHz a DSHOT600 bit
# (1.67us) is not resolved, so that is for frame starts and spacing; at 10 MHz (0.1us) the bits are (a
# 100 KB buffer then holds 20ms) and dshot_frame_decode.py reads the frames.
#
# Uses state machine 11 (the last one of PIO block 2, which the bench keeps free for the idle
# unidirectional motors) and one DMA channel. It never writes to a pin.

import micropython
import rp2
from array import array

SAMPLE_HZ = 1_000_000       # default sampling clock: one sample per microsecond
PIO_BASES = (0x50200000, 0x50300000, 0x50400000)   # RP2350: PIO0, PIO1, PIO2
RXF0_OFFSET = 0x20                                  # first RX FIFO register of a block


@rp2.asm_pio(autopush=True, push_thresh=32, in_shiftdir=rp2.PIO.SHIFT_RIGHT, fifo_join=rp2.PIO.JOIN_RX)
def sample_pins():
    in_(pins, 4)


@micropython.viper
def find_edges_raw(buf: ptr32, nwords: int, bit: int, out: ptr32, cap: int) -> int:
    """Edges of one pin (bit 0 = the first pin) from the packed samples: out[n] = (sample index << 1) | new
    level. The line is taken to idle high at the start. Returns how many edges there were, which can be
    more than cap (only cap are stored)."""
    n = 0
    prev = 1
    for w in range(nwords):
        word = int(buf[w])
        for k in range(8):
            level = (word >> (4 * k + bit)) & 1
            if level != prev:
                if n < cap:
                    out[n] = (((w << 3) + k) << 1) | level
                n += 1
                prev = level
    return n


class PinSampler:
    def __init__(self, first_pin, sm_id=11, sample_hz=SAMPLE_HZ):
        self.sm_id = sm_id
        self.sample_hz = sample_hz
        self.sample_ns = 1_000_000_000 // sample_hz
        block = sm_id // 4
        local = sm_id % 4
        self.sm = rp2.StateMachine(sm_id, sample_pins, freq=sample_hz, in_base=first_pin)
        self.dma = rp2.DMA()
        self.fifo_address = PIO_BASES[block] + RXF0_OFFSET + 4 * local
        self.dreq = (block << 3) + 4 + local          # the RX data request of this state machine
        self.last_count = 0

    def start(self, buf):
        """Begin filling buf (len(buf) // 4 words, 8 samples per word) and return at once. The recording
        takes len(buf) / 4 * 8 samples; poll done(), then call finish()."""
        self.sm.active(0)
        self.sm.restart()
        while self.sm.rx_fifo():
            self.sm.get()
        ctrl = self.dma.pack_ctrl(size=2, inc_read=False, inc_write=True, treq_sel=self.dreq, irq_quiet=True)
        self.dma.config(read=self.fifo_address, write=buf, count=len(buf) // 4, ctrl=ctrl, trigger=True)
        self.sm.active(1)

    def done(self):
        return not self.dma.active()

    def finish(self):
        """Stop the recorder; True when the transfer had completed."""
        complete = not self.dma.active()
        self.sm.active(0)
        if not complete:
            self.dma.active(0)
        return complete

    def capture(self, buf, timeout_ms=2000):
        """start(), wait for the transfer, finish(). Blocks for the whole recording."""
        import utime
        self.start(buf)
        deadline = utime.ticks_add(utime.ticks_ms(), timeout_ms)
        while not self.done():
            if utime.ticks_diff(deadline, utime.ticks_ms()) < 0:
                break
        return self.finish()

    def edges(self, buf, bit, scratch, unit_ns=1000):
        """Generator of (time, level) for one pin, the time in units of unit_ns nanoseconds (default 1000:
        microseconds; use 100 for a 10 MHz recording), so the edges are never all held as objects at once (a
        recording holds thousands). scratch is an array('I') the raw edges are written to; its length is
        the capacity. Raises OverflowError when the recording held more edges than that. self.last_count is
        the number of edges once the generator has started."""
        count = find_edges_raw(buf, len(buf) // 4, bit, scratch, len(scratch))
        self.last_count = count
        if count > len(scratch):
            raise OverflowError("recording holds " + str(count) + " edges, capacity " + str(len(scratch)))
        for i in range(count):
            value = scratch[i]
            yield (value >> 1) * self.sample_ns // unit_ns, value & 1

    def close(self):
        self.sm.active(0)
        self.dma.close()
