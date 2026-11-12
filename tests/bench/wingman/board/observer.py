# EXAMPLE APPLICATION CODE - not part of the DShot library, and a diagnostic: nothing in the driver or the
# command loop uses it.
#
# The observer: runs on the wingman (the SECOND Pico) and only watches. It records every edge of up to four lines
# (the ESC signal wires of the bench) and streams the record to the PC over USB while a start/end line is
# high. Nothing is ever written to a spied-on pin.
#
# Wiring: every spied-on line goes to the SAME GPIO number on the wingman as on the board under test (the
# ESC signals are GPIO 6-9 there), the start/end line (start_line.START_PIN, GPIO 15; the recorded test
# holds it high for the duration of the test) likewise, and a ground wire joins the boards.
#
# How: one PIO state machine per line counts time in 100ns ticks and, at every edge, hands over one 16-bit
# entry: bit 15 = the level of the run that just ended (1 = it was high), bits 14..0 = 0x7FFF minus the
# ticks counted. All lines wait on the start line in hardware, so they begin on the same clock cycle.
# A DMA channel per line copies the entries into a ring buffer without help from either core; the core only
# forwards what has arrived. Decoding is done on the PC (tests/bench/wingman/tools/observer_edge_log.py).
#
# Time arithmetic (the program below is exact): with the state machine at 20 MHz one instruction is 50ns.
# The time from one detected edge to the next is (2 * ticks + 4) instructions for a run that was high and
# (2 * ticks + 6) for a run that was low, where ticks = 0x7FFF - the 15-bit field. A field of exactly 0x7FFF
# is not a run but a marker: 32767 ticks passed with no edge (the line is quiet); the run continues in the
# next entry of the same level. A marker carries the OPPOSITE level bit to the run it belongs to, so an edge that
# lands right after a marker (field 0x7FFF, 0 ticks counted) cannot be mistaken for one (stream version "marker=v2";
# before 2026-10-28 markers carried the run's own bit and such an edge was lost). One marker covers
# 2 * 32768 + 4 = 65540 instructions = 3277.0us.
#
# Lost data is never silent: the end packet carries, per line, the bytes received, ring overruns (the core
# was too slow to forward) and the PIO's receive-stall flag (the FIFO was full when the state machine had
# data). A record with either is not valid.
#
# Stream format, after whatever the REPL prints: packets of  0xD5, type, channel, length (2 bytes,
# little endian), payload.  type 'S' = start (text), 'D' = data (entries, little endian 32-bit words, each
# holding two entries, first entry in the high half), 'E' = end (text).

import gc
import sys
import uctypes
import utime
import rp2
from machine import Pin, mem32

SM_FREQ = 20_000_000
TICK_NS = 100
PIO2_BASE = 0x50400000
FDEBUG = PIO2_BASE + 0x008
FIRST_SM = 8                        # state machines 8..11 = all of PIO block 2
RXF0 = PIO2_BASE + 0x20
DREQ0 = (2 << 3) + 4
RING_BITS = 13
RING_BYTES = 1 << RING_BITS
COUNT = 0x0FFFFFFF
MAGIC = 0xD5
SEND_MIN_BYTES = 512
SEND_MAX_MS = 20
TAIL_MS = 30
GATE_LOW_MS = 2                   # the start line must have been low this long before the gate is armed

programs = {}


def make_program(sync_pin):
    """The state machine program that waits on GPIO sync_pin (the number is part of the instruction)."""
    @rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_LEFT, autopush=True, push_thresh=32, fifo_join=rp2.PIO.JOIN_RX)
    def edge_timer():
        mov(y, invert(null))
        in_(y, 15)
        mov(y, isr)                     # y = 0x7FFF, kept: reload value and the "1" level bit
        mov(isr, null)
        wait(1, gpio, sync_pin)
        mov(x, y)
        jmp(pin, "high_run")
        label("low_run")
        jmp(pin, "low_end")
        jmp(x_dec, "low_run")
        in_(y, 1)                       # 32767 ticks without an edge, still low: marker (level bit inverted)
        in_(x, 15)
        mov(x, y)
        jmp("low_run")
        label("low_end")
        in_(null, 1)
        in_(x, 15)
        mov(x, y)
        label("high_run")
        jmp(pin, "high_cont")
        jmp("high_end")
        label("high_cont")
        jmp(x_dec, "high_run")
        in_(null, 1)                    # marker, still high (level bit inverted)
        in_(x, 15)
        mov(x, y)
        jmp("high_run")
        label("high_end")
        in_(y, 1)
        in_(x, 15)
        mov(x, y)
        jmp("low_run")
    return edge_timer


def aligned_ring():
    raw = bytearray(2 * RING_BYTES)
    offset = (-uctypes.addressof(raw)) % RING_BYTES
    return raw, memoryview(raw)[offset:offset + RING_BYTES]


class Channel:
    def __init__(self, index, pin, program):
        self.index = index
        self.raw, self.ring = aligned_ring()
        self.sm = rp2.StateMachine(FIRST_SM + index, program, freq=SM_FREQ, jmp_pin=Pin(pin))
        self.dma = rp2.DMA()
        self.read_words = 0
        self.overruns = 0
        self.sent_bytes = 0
        self.last_send = utime.ticks_ms()
        ctrl = self.dma.pack_ctrl(size=2, inc_read=False, inc_write=True, ring_size=RING_BITS, ring_sel=1,
                                  treq_sel=DREQ0 + index, irq_quiet=True)
        self.dma.config(read=RXF0 + 4 * index, write=self.ring, count=COUNT, ctrl=ctrl, trigger=True)

    def written_words(self):
        return COUNT - (self.dma.count & COUNT)

    def forward(self, out, final=False):
        """Send what has arrived since the last call (in at most two packets, the ring wraps)."""
        written = self.written_words()
        pending = written - self.read_words
        if pending * 4 > RING_BYTES - 256:           # the ring may have been overwritten
            self.overruns += 1
            self.read_words = written
            return
        now = utime.ticks_ms()
        if not final and pending * 4 < SEND_MIN_BYTES and utime.ticks_diff(now, self.last_send) < SEND_MAX_MS:
            return
        self.last_send = now
        while pending > 0:
            position = (self.read_words * 4) % RING_BYTES
            n = min(pending * 4, RING_BYTES - position)
            out.write(bytes((MAGIC, 0x44, self.index, n & 0xFF, n >> 8)))
            out.write(self.ring[position:position + n])
            self.sent_bytes += n
            self.read_words += n // 4
            pending -= n // 4

    def stop(self):
        self.sm.active(0)
        self.dma.active(0)
        self.dma.close()


def packet(out, kind, channel, text):
    data = text.encode()
    out.write(bytes((MAGIC, ord(kind), channel, len(data) & 0xFF, len(data) >> 8)))
    out.write(data)


def run(pins, sync_pin, wait_ms=120_000, tick=None, configure_pins=True):
    """Listen on the GPIO numbers in pins (at most four) with the start line on sync_pin; stream until that
    line has been high and then low again (or wait_ms passed without it going high). tick(), if given, is
    called on every pass of the forwarding loop (a self-test uses it to drive its own start line)."""
    if configure_pins:
        for p in tuple(pins) + (sync_pin,):
            Pin(p, Pin.IN)
    sync = Pin(sync_pin)
    if sync_pin not in programs:
        programs[sync_pin] = make_program(sync_pin)
    gc.collect()
    channels = [Channel(i, pin, programs[sync_pin]) for i, pin in enumerate(pins)]
    mem32[FDEBUG] = 0x0F                              # clear the receive-stall flags
    out = sys.stdout.buffer
    packet(out, "S", 255, "pins=%s sync_pin=%d tick_ns=%d ring_bytes=%d marker=v2" % (
        ",".join(str(p) for p in pins), sync_pin, TICK_NS, RING_BYTES))
    # The state machines open on a high start line, so they are only started once the line has been seen low: a line
    # that floats or is left high (the main Pico before its script drives it) would otherwise open the gate early and
    # put the time origin seconds before the test.
    began = utime.ticks_ms()
    low_since = None
    reason = "ended"
    while reason == "ended":
        if tick is not None:
            tick()
        if sync.value():
            low_since = None
        elif low_since is None:
            low_since = utime.ticks_ms()
        elif utime.ticks_diff(utime.ticks_ms(), low_since) >= GATE_LOW_MS:
            break
        if utime.ticks_diff(utime.ticks_ms(), began) > wait_ms:
            reason = "timeout: the start line never went low"
        utime.sleep_us(300)
    if reason == "ended":
        for c in channels:
            c.sm.active(1)
    began = utime.ticks_ms()
    seen_high = False
    low_since = None
    while reason == "ended":
        if tick is not None:
            tick()
        for c in channels:
            c.forward(out)
        if sync.value():
            seen_high = True
            low_since = None
        elif seen_high:
            if low_since is None:
                low_since = utime.ticks_ms()
            elif utime.ticks_diff(utime.ticks_ms(), low_since) >= TAIL_MS:
                break
        elif utime.ticks_diff(utime.ticks_ms(), began) > wait_ms:
            reason = "timeout: the start line never went high"
            break
        utime.sleep_us(300)
    for c in channels:
        c.sm.active(0)
    stalls = mem32[FDEBUG] & 0x0F
    for c in channels:
        c.forward(out, True)
    parts = ["reason=" + reason]
    valid = reason == "ended"
    for c in channels:
        stalled = (stalls >> c.index) & 1
        parts.append("ch%d bytes=%d overruns=%d rx_stall=%d" % (c.index, c.sent_bytes, c.overruns, stalled))
        valid = valid and c.overruns == 0 and not stalled
    parts.append("valid=%d" % valid)
    packet(out, "E", 255, " ".join(parts))
    for c in channels:
        c.stop()
    return valid
