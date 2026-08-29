# SPDX-License-Identifier: GPL-3.0-or-later
# Original implementation: https://github.com/jrddupont/DShotPIO
# Licensed under GNU General Public License v3.0
# DShot protocol reference: https://brushlesswhoop.com/dshot-and-bidirectional-dshot/

import utime
from machine import Pin
from rp2 import PIO, StateMachine, asm_pio

class InvalidThrottleException(Exception):
    def __init__(self,message):
        self.message=message

# PIO assembly code for sending DShot throttle packets
# This is set up to transmit the 16 high order bits of a 32 bit input from high to low order
# Each bit is sent in 8 clock cycles
@asm_pio(sideset_init=PIO.OUT_LOW, out_shiftdir=PIO.SHIFT_LEFT, autopull=True, pull_thresh=16)
def dshot():
    wrap_target()
    label("start")
    out(x, 1)            .side(0)    [1] # 2 cycle, Read the next bit into x register. Start at zero so the output is always low when waiting for new data
    jmp(not_x, "zero")   .side(1)    [2] # 3 cycles, Jump on x register
    jmp("start")         .side(1)    [2] # 3 cycles, "ONE" condition
    label("zero")
    jmp("start")         .side(0)    [2] # 3 cycles, "ZERO" condition
    wrap()

# Bidirectional DShot TX. Same bit timing as dshot() above for every bit
# (every side-set value flipped, so idle and the "0" duty portion sit HIGH
# instead of LOW - this is what lets an AM32 ESC auto-detect bidirectional
# mode; see ADR-002 and DShotPIO.__init__'s bidirectional docstring for why
# this must be in use for the *entire* arm sequence, not switched in
# afterward), but unlike dshot() this releases the pin (pindirs -> input)
# after each 16-bit frame so an RX state machine sharing the same pin - or
# the ESC itself - can drive it for the GCR telemetry reply, then reclaims
# it before the next frame.
#
# This needs a manual pull() + bit counter instead of autopull: autopull
# refills the OSR transparently, with no signal a program can branch on, so
# there is no way to know "a full frame just finished" without counting it
# ourselves - and that is exactly the point where the pin needs to be
# released. pull() is placed so it blocks with the pin already released:
# the gap where an RX window can happen is the time between one frame ending
# and Python supplying the next one, which is also the time this state
# machine spends stalled here.
#
# The "bit=1" and "bit=0" paths each carry their own copy of the release-and-
# loop tail rather than sharing one: a single PIO instruction encodes exactly
# one side-set value, and the two paths need different ones, so they cannot
# converge on one shared instruction. Duplicating a couple of instructions
# costs program memory (32 words available, this uses 12) but not cycles -
# the per-bit timing is unaffected because both paths still spend exactly 8
# cycles on out()+jmp(not_x)+jmp(y_dec) before falling into their own tail on
# the last bit only.
#
# irq(4) (Option A' - see ADR-002) fires once per frame, right after the pin
# is released, to tell dshot_bidir_rx on the paired state machine that it is
# safe to start its own fixed post-release delay before listening. This is a
# non-blocking set (no "block" argument) - it does not wait for RX to be
# listening, so it costs nothing if RX is inactive or still busy with a
# previous capture. IRQ 4 is one of the four (4-7) that never reach the CPU,
# used here purely for this SM-to-SM handshake.
@asm_pio(sideset_init=PIO.OUT_HIGH, set_init=PIO.OUT_HIGH, out_shiftdir=PIO.SHIFT_LEFT, autopull=False)
def dshot_bidir_tx():
    label("frame_start")
    pull()                     .side(1)   [1] # blocks here with the pin already released - this is the gap/RX window
    set(pindirs, 1)            .side(1)   [1] # a new command is ready - reclaim the pin as output
    set(y, 15)                 .side(1)   [1] # 16-bit counter
    label("bitloop")
    out(x, 1)                  .side(1)   [1] # 2 cycles, HIGH (idle level) while shifting in the next bit
    jmp(not_x, "zero")         .side(0)   [2] # 3 cycles, LOW, always executed regardless of bit value
    jmp(y_dec, "bitloop")      .side(0)   [2] # "one" path: 3 cycles LOW, loop unless this was bit 16 - mostly LOW (25% high) same as dshot_bidir's original bit=1
    set(pindirs, 0)            .side(0)   [1] # bit 16 only ("one" path): release the pin
    irq(4)                     .side(0)   [0] # tell RX the pin was just released
    jmp("frame_start")         .side(0)   [0]
    label("zero")
    jmp(y_dec, "bitloop")      .side(1)   [2] # "zero" path: 3 cycles HIGH, loop unless this was bit 16 - mostly HIGH (62.5% high)
    set(pindirs, 0)            .side(1)   [1] # bit 16 only ("zero" path): release the pin
    irq(4)                     .side(1)   [0] # tell RX the pin was just released
    jmp("frame_start")         .side(1)   [0]

# GCR capture for a bidirectional DShot ESC's eRPM reply (Option A' - see
# ADR-002's "Implementation Update" for the design rationale, and its
# "RX capture diagnosis" section for the full history of what this program
# used to do and why it changed again here).
#
# Two earlier designs were tried and diagnosed to their limit:
#
# 1. A single point-sample per bit (no fixed delay) - pure noise, 4/84 valid
#    GCR symbols.
# 2. A "slotted" design assuming a specific bit period (8 PIO cycles, 5/4x
#    the DShot bitrate per the generic spec), 3 samples per assumed slot at
#    fixed offsets. After fixing several real bugs (predelay overshoot, a
#    marker-skip cycle-count error, an illegal `set` immediate), this got
#    close - alignment confirmed, the first GCR symbol decoded correctly and
#    deterministically in all 17 test captures - but CRC validated in only
#    1/17, with failures growing monotonically deeper into the frame. That
#    signature (clean front, degrading back) doesn't match a framing bug
#    (would corrupt the front first) or uniform noise (would hit evenly) -
#    it matches a *small* residual rate error the coarse 3-sample vote
#    couldn't resolve from noise. Every attempt to pin down that error from
#    the existing 3-samples/slot data (run-length histograms, resampling
#    onto candidate periods) came back too weak to act on - see ADR-002.
#
# This version removes the "slot" assumption entirely instead of refining
# it further. It does not assume any particular bit period - it just
# samples the pin uniformly and continuously, densely enough (5-6+ samples
# per plausible real bit) that the actual bit period and phase can be
# measured directly from the raw waveform in software, rather than assumed
# by the PIO program and inferred backward from a coarse vote. The marker
# bit itself is captured too (previously skipped and used only for edge
# timing) so software has an unambiguous, self-evident zero to anchor
# everything else against.
#
# `rx_speed` (see DShotPIO.__init__) is chosen independently of any assumed
# GCR bitrate - just fast enough to safely oversample the plausible range of
# real bit periods (previously measured to sit somewhere around 7-8 PIO
# cycles at the old 3MHz clock, i.e. roughly 2.3-2.7us) with margin on both
# sides, tuned for DShot300 (the only speed this project currently runs;
# revisit before using bidirectional=True at any other speed).
#
# VERIFIED on hardware (see ADR-002): scripts/decode_bidir_capture.py's
# run-length reconstruction (not simple resampling - see its module
# comment for why that distinction matters) decodes 17/17 real captures
# with valid CRC, eRPM rising monotonically across throttle steps. The
# real bit period measured ~10.1-10.4 PIO cycles at this rx_speed (~2.5-
# 2.6us) - within the range this program was tuned to oversample.
#
# 128 raw samples packed via autopush at push_thresh=32 (the max, to
# minimise word count) - exactly 4 32-bit words per real reply, matching the
# RX FIFO depth exactly so a single capture can never stall waiting for
# Python to drain mid-frame.
#
# `set`'s immediate operand is a 5-bit field (max 31) - `set(y, 127)` is
# illegal and would have silently misassembled (this is the same class of
# bug as `set(y, 39)` earlier in this ADR's history, caught this time before
# a hardware round rather than after). 128 samples needs a nested loop: an
# outer pass of 4 (`x`), each running an inner loop of 32 samples (`y`).
# This is NOT perfectly uniform: within a pass, consecutive samples are 2
# cycles apart (`in_` + `jmp(y_dec)`, 1 cycle each); at each of the 3 pass
# boundaries, reloading `y` and looping `x` costs 2 *extra* cycles (a failed
# `jmp(y_dec)` + `jmp(x_dec)` + `set(y, 31)` vs. the single taken `jmp` a
# within-pass transition would have cost), so those 3 gaps are 4 cycles
# instead of 2. This is fully deterministic - scripts/decode_bidir_capture.py
# computes each sample's exact absolute cycle position (not just its index)
# to account for it, rather than assuming uniform spacing.
@asm_pio(in_shiftdir=PIO.SHIFT_LEFT, autopush=True, push_thresh=32)
def dshot_bidir_rx():
    wrap_target()
    # IRQ 4 is a single sticky, block-level flag, not a queue: if this SM was
    # still busy (autopush stalled on a full RX FIFO - see rx_read()'s
    # comment) when dshot_bidir_tx fired irq(4) for a frame we then missed,
    # that signal would otherwise sit latched and get consumed as if it were
    # fresh the moment we reach wait() below - re-phasing the predelay +
    # marker search against the wrong point in time and risking a capture of
    # TX's own waveform instead of a real reply (see R1/R2 in
    # bidirectional_dshot_review.md). Clearing first forces the wait below to
    # block for a genuinely new release, every time - including the very
    # first iteration after start(), which is what also prevents a flag from
    # a previous run surviving stop()'s restart() into this one.
    irq(clear, 4)
    wait(1, irq, 4)                  # block for dshot_bidir_tx's per-frame release signal (auto-clears the flag)

    # ~4.7us fixed delay before listening - empirically confirmed correct
    # (see ADR-002): AM32's actual reply turnaround on this ESC sits here,
    # not the ~25-30us a generic reference suggested. wait(0, pin, 0) below
    # is a level wait, not an edge detector - too short a predelay re-
    # triggers instantly on TX's own still-LOW tail (this project's very
    # first RX attempt's all-zero-capture failure), so this is a lower
    # bound, not zero.
    set(x, 1)
    label("predelay")
    jmp(x_dec, "predelay")     [6]   # 2 iterations x 7 cycles = 14 cycles

    wait(0, pin, 0)                  # the reply's leading (marker) edge

    # 4 outer passes x 32 inner samples = 128 total - see module comment for
    # why this can't be one flat loop, and for the resulting (deterministic,
    # accounted-for-in-software) timing seam every 32 samples.
    set(x, 3)
    label("outer")
    set(y, 31)
    label("inner")
    in_(pins, 1)                      # 1 cycle
    jmp(y_dec, "inner")               # 1 cycle - 2 cycles/sample within a pass
    jmp(x_dec, "outer")               # 1 cycle - only reached once per pass, after 32 samples
    wrap()

# The different DShot speeds. The Pico and Pico W should be fast enough to transmit at any of these speeds
class DSHOT_SPEEDS:
    DSHOT150  = 1_200_000 #   150,000 bit/s * 8 cycle/bit
    DSHOT300  = 2_400_000 #   300,000 bit/s * 8 cycle/bit
    DSHOT600  = 4_800_000 #   600,000 bit/s * 8 cycle/bit
    DSHOT1200 = 9_600_000 # 1,200,000 bit/s * 8 cycle/bit

# rx_speed to use for dshot_bidir_rx per DShot request speed - hardware-verified
# (bidirectional_dshot_review.md's W1 item), not a fixed ratio of dshot_speed.
# DSHOT600 and DSHOT1200 share one entry rather than needing separate ones:
# AM32 only bins detected input rate into two reply-timing bands (confirmed
# against its checkDshot() in Src/signal.c - one config for ~150/300, another
# for ~600/1200), so the two speeds produce an identical real GCR reply bit
# period on this ESC (~1.28-1.29us, measured). Going faster than the profile
# a speed actually needs doesn't just waste margin - at DSHOT1200 a 16MHz
# rx_speed measurably broke decoding, because the 128-sample capture window
# shrinks in wall-clock time as rx_speed rises, and it fell below the frame's
# real duration.
BIDIR_PROFILES = {
    DSHOT_SPEEDS.DSHOT300:  4_000_000,  # ~2.5-2.6us measured bit period, 17/17 CRC-valid
    DSHOT_SPEEDS.DSHOT600:  8_000_000,  # ~1.28us measured, 6/6 CRC-valid
    DSHOT_SPEEDS.DSHOT1200: 8_000_000,  # same reply timing as DSHOT600 on this ESC, 4/4 CRC-valid
}


class DShotPIO:
    # Words the PIO TX FIFO holds before put() starts blocking
    TX_FIFO_DEPTH = 4

    # Creates the state machine but leaves it inactive - call start() to enable it
    def __init__(self, state_machine_id, pin, dshot_speed=DSHOT_SPEEDS.DSHOT150,
                 bidirectional=False, rx_state_machine_id=None):
        """
        Args:
            bidirectional: Use the inverted TX waveform an AM32 (or other
                bidirectional-capable) ESC needs to auto-detect bidirectional
                DShot. Detection only happens while the ESC is disarmed, so
                this must be set for the whole arm sequence - there is no way
                to arm with a normal signal and switch afterward.
            rx_state_machine_id: Required when bidirectional=True. Creates a
                second state machine that listens on the same pin for the
                ESC's GCR telemetry reply (see dshot_bidir_rx). Must be on the
                same PIO block as state_machine_id - a GPIO's function select
                routes to one PIO block at a time (ids 0-3 -> PIO0, 4-7 ->
                PIO1, 8-11 -> PIO2 on RP2350), so a TX/RX pair sharing a pin
                must share a block, and IRQ 4 (see dshot_bidir_tx's irq(4))
                must reach both, which inter-SM IRQs only do within one
                block. start() activates both state machines; from then on
                RX synchronises itself to each TX frame via that IRQ with no
                further calls needed - just drain rx_read() periodically.
        """
        self.bidirectional = bidirectional
        program = dshot_bidir_tx if bidirectional else dshot

        if bidirectional:
            # Both sides release the line between frames (see dshot_bidir_tx)
            # so the other can drive it - with nobody driving, an undriven
            # pad floats rather than sitting at a defined level, and on this
            # hardware it was observed floating LOW, which reads as a false
            # start bit to dshot_bidir_rx's wait(0, pin, 0). A weak pull-up
            # holds the line at the expected idle-HIGH level whenever neither
            # side is actively driving, without resisting either one when
            # they are - the same fix any shared/open-drain-style bus needs.
            # Pad-level pull config is independent of which peripheral's
            # FUNCSEL claims the pin, so this holds even once the state
            # machines below take over.
            pin.init(Pin.IN, Pin.PULL_UP)

        self.sm = StateMachine(state_machine_id, program, freq=dshot_speed,
                                sideset_base=pin, set_base=pin)

        # Wall-clock time of one 16-bit frame at 8 PIO cycles per bit, rounded
        # up so a wait built from it is never short
        self.frame_us = (16 * 8 * 1_000_000 + dshot_speed - 1) // dshot_speed

        self.rx_sm = None
        if bidirectional:
            if rx_state_machine_id is None:
                raise ValueError("rx_state_machine_id is required when bidirectional=True")

            rx_speed = BIDIR_PROFILES.get(dshot_speed)
            if rx_speed is None:
                raise ValueError("bidirectional=True needs a dshot_speed with a verified "
                                  "BIDIR_PROFILES entry (DSHOT300, DSHOT600, or DSHOT1200 currently)")
            self.rx_sm = StateMachine(rx_state_machine_id, dshot_bidir_rx,
                                       freq=rx_speed, in_base=pin)

    def start(self):
        if self.rx_sm is not None:
            # Flush any leftover words from a previous run and start RX
            # listening before TX can release the pin and fire its first
            # irq(4) - see dshot_bidir_rx's irq(clear, 4) comment for the
            # rest of this epoch-clean boundary.
            while self.rx_sm.rx_fifo():
                self.rx_sm.get()
            self.rx_sm.active(1)
        self.sm.active(1)

    def rx_read(self):
        """
        Return the next raw captured word from the RX FIFO, or None if empty.

        dshot_bidir_rx synchronises itself to every TX frame via a PIO IRQ -
        no per-read setup call is needed. Each real reply produces four raw
        32-bit words back to back (128 uniformly-spaced, un-slotted samples
        covering the marker bit, the 20 real data bits, and idle tail - see
        dshot_bidir_rx's comments). Raw and unpaired: determining the real
        bit period/phase from this and decoding into eRPM is a later phase
        (see ADR-002).
        """
        if not self.rx_sm.rx_fifo():
            return None
        return self.rx_sm.get()

    def drain(self):
        """
        Block until everything queued has been transmitted.

        Call this before stop() when the queued frames still need to reach the
        ESC - stop() cuts them off otherwise. It also parks the line at its
        idle level (low for dshot(), high for dshot_bidir_tx()): with the
        FIFO empty the program stalls (on the out instruction for dshot(),
        on pull() for dshot_bidir_tx()), holding that instruction's side-set
        value - which for dshot_bidir_tx() also means the pin is released
        (see its own comments), letting an RX state machine or the ESC drive
        the line.

        Only meaningful on an active state machine. The wait is bounded rather
        than a spin on tx_fifo(), so calling it on an inactive one costs a few
        hundred microseconds instead of hanging.
        """
        for _ in range(self.TX_FIFO_DEPTH):
            if not self.sm.tx_fifo():
                break
            utime.sleep_us(self.frame_us)

        # tx_fifo() reaching zero only means the last word has been pulled into
        # the shift register - it is still going out on the wire
        utime.sleep_us(self.frame_us)

    def stop(self):
        """
        Deactivate the state machine.

        The signal line stops carrying DShot transitions, so the ESC times out
        and cannot spin.

        Stopping mid-frame truncates that frame - it fails CRC and the ESC
        drops it - and leaves the pin at whatever level the frame was driving.
        Call drain() first when that matters.

        Call start() to reactivate - the PIO program stays loaded.
        """
        self.sm.active(0)

        # Clears the shift state and jumps to the start of the program, so a
        # frame interrupted by active(0) cannot resume mid-bit on restart.
        # This does NOT drain the TX FIFO: whatever is still queued goes out on
        # the next start(), which is why drain() exists as a separate call.
        self.sm.restart()

        if self.rx_sm is not None:
            self.rx_sm.active(0)
            self.rx_sm.restart()

    def send_throttle_command(self, throttle):
        """
        Send a throttle command to the ESC.

        Args:
            throttle: Throttle value (0-2047)

        Note: DShot protocol includes a telemetry request bit, but this implementation
        always sets it to 0. AM32 sends a GCR telemetry reply after every frame once
        bidirectional mode is detected regardless of this bit (confirmed from AM32
        firmware source - see ADR-002), so leaving it 0 does not suppress telemetry.
        """
        if throttle < 0:
            raise InvalidThrottleException("Throttle should be greater than 0.")
        if throttle > 2047:
            raise InvalidThrottleException("Throttle value is too high. Maximum value is 2047.")

        # Build 12-bit value: 11-bit throttle shifted left, telemetry bit = 0
        packetValue = throttle << 1

        # Calculate 4-bit CRC. Bidirectional DShot inverts it - this is what
        # actually rides on the wire once dshot_bidir_tx's waveform inverts
        # the bit levels themselves; ESCs use the inverted CRC to validate
        # frames once they've auto-detected bidirectional mode (see ADR-002).
        crc = (packetValue ^ (packetValue >> 4) ^ (packetValue >> 8)) & 0x0F
        if self.bidirectional:
            crc = (~crc) & 0x0F

        # Build 16-bit packet: SSSSSSSSSSSTCCCC (S=throttle, T=telemetry=0, C=CRC)
        dShotPacket = (packetValue << 4) | crc
        
        # Since the state machine consumes the bits from high order to low order, we need to shift the
        #  data all the way to the high bit
        rightPaddedPacket = dShotPacket << 16

        # Put the packet into the PIO machine
        self.sm.put(rightPaddedPacket)
