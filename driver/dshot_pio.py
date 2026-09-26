# SPDX-License-Identifier: GPL-3.0-or-later
# Original implementation: https://github.com/jrddupont/DShotPIO
# Licensed under GNU General Public License v3.0
# DShot protocol reference: https://brushlesswhoop.com/dshot-and-bidirectional-dshot/

import utime
from machine import Pin
from rp2 import PIO, StateMachine, asm_pio

import gcr_decode
from capture_mailbox import CaptureMailbox
from dshot_profiles import DSHOT_SPEEDS, BIDIR_PROFILES, RLE_CYCLES_PER_BIT, rle_rx_speed

# Highest value the 11-bit throttle field of a DShot packet can carry. A module
# constant rather than a class attribute lookup because send_throttle_command()
# reads it on every frame of every motor.
MAX_THROTTLE = 2047


class InvalidThrottleException(Exception):
    def __init__(self,message):
        self.message=message

class UnsupportedOperationException(Exception):
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

# Bidirectional DShot TX. Same bit timing as dshot(), with every side-set level
# flipped so the line idles HIGH and the "0" bit's duty portion sits HIGH. An
# AM32 ESC recognises bidirectional mode from this inverted polarity, and only
# while it is disarmed, so this program has to be in use for the whole arm
# sequence (see BidirectionalDShot).
#
# Unlike dshot(), it releases the pin (pindirs -> input) after each 16-bit frame
# so the ESC can drive the line for its GCR telemetry reply, then reclaims it
# for the next frame.
#
# It counts bits by hand and uses a manual pull() instead of autopull: autopull
# refills the OSR with nothing a program can branch on, so the end of a frame -
# the moment the pin must be released - could not be detected. pull() blocks
# with the pin already released, so the idle time between frames is also the
# ESC's reply window.
#
# The bit=1 and bit=0 paths each carry their own release-and-loop tail because
# one PIO instruction encodes exactly one side-set value and the two paths need
# different ones. That costs program memory (12 of 32 words) but no cycles: the
# per-bit timing is identical on both paths.
#
# irq(rel(1)) fires once per frame, right after the release, telling the paired
# dshot_bidir_rx that it may start its post-release delay. It is non-blocking,
# so it costs nothing when RX is inactive or still busy with the previous
# capture.
#
# The IRQ is relative (rel) rather than a literal flag number because flags 4-7
# are shared by every state machine on a PIO block: with a literal flag, two
# bidirectional pairs on one block would consume each other's signal. rel(k)
# resolves to a flag derived from the executing state machine's own id, so a TX
# and its RX share one flag and every other pair gets a different one - provided
# every pair uses the same TX-to-RX id offset. That is why BidirectionalDShot
# requires rx_state_machine_id == state_machine_id + 1 (TX fires rel(1), RX
# waits on rel(0)). See ADR-002's per-pair synchronization section.
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
    irq(rel(1))                .side(0)   [0] # tell paired RX (id = this SM's id + 1) the pin was just released
    jmp("frame_start")         .side(0)   [0]
    label("zero")
    jmp(y_dec, "bitloop")      .side(1)   [2] # "zero" path: 3 cycles HIGH, loop unless this was bit 16 - mostly HIGH (62.5% high)
    set(pindirs, 0)            .side(1)   [1] # bit 16 only ("zero" path): release the pin
    irq(rel(1))                .side(1)   [0] # tell paired RX (id = this SM's id + 1) the pin was just released
    jmp("frame_start")         .side(1)   [0]

# Captures an ESC's GCR telemetry reply as a dense, uniform raw waveform for
# software to decode (see gcr_decode.py).
#
# The program assumes nothing about the reply's bit period. It samples the pin
# every 2 PIO cycles, continuously, 128 times, which is several samples per bit
# at the rx_speed chosen for the DShot speed (BIDIR_PROFILES) - dense enough that
# software can find bit boundaries from the run lengths between edges rather
# than trusting a bit period baked into the PIO program. The marker bit is
# captured too, giving software an unambiguous 0 to anchor against.
#
# 128 samples are packed by autopush at push_thresh=32 (the maximum, which keeps
# the word count down): exactly 4 words per reply. This state machine never uses
# its TX FIFO, so fifo_join gives the RX FIFO that depth as well: 8 words, room
# for a capture the CPU has not taken yet plus the next one. With the default
# 4-word FIFO a capture the CPU was late to take left no room for the next: that
# capture's first word blocked on the full FIFO, its sampling paused mid-reply,
# and it came back as a short burst followed by idle-level words.
#
# What the program does, in order:
#
# 1. Clear the IRQ flag, then wait for the paired TX to raise it (its "pin
#    released" signal). The flag is sticky, not a queue: a signal left over
#    from a frame this state machine missed (autopush stalled on a full FIFO)
#    would be consumed as fresh at the wait, phasing the capture against the
#    wrong moment and possibly capturing TX's own waveform as a "reply".
#    Clearing first makes the wait block for a genuinely new release every
#    iteration - including the first after start(), which also stops a flag
#    from a previous run surviving stop()'s restart(). rel(0) resolves to the
#    same flag the paired TX raises with rel(1); see dshot_bidir_tx.
#
# 2. Delay about 14 cycles (two 7-cycle iterations): about 4.15us at DSHOT300's
#    RX clock, 2.07us at DSHOT600's. This is a lower bound, not the reply's
#    start - step 3 finds that. Step 3 is a level wait, not an edge detector, so
#    a delay too short would let it re-trigger immediately on TX's own
#    still-LOW tail.
#
# 3. Wait for the pin to go LOW: the reply's leading (marker) edge.
#
# 4. Take the 128 samples. set()'s immediate is a 5-bit field (max 31), so they
#    cannot come from one loop: it is 4 outer passes (x) of 32 inner samples
#    (y). Samples within a pass are 2 cycles apart, but at each of the 3 pass
#    boundaries reloading y and looping x costs 2 extra cycles, so those gaps
#    are 4. The seam is deterministic, and gcr_decode.sample_cycle() accounts
#    for it.
@asm_pio(in_shiftdir=PIO.SHIFT_LEFT, autopush=True, push_thresh=32, fifo_join=PIO.JOIN_RX)
def dshot_bidir_rx():
    wrap_target()
    irq(clear, rel(0))               # step 1: drop any stale release signal...
    wait(1, irq, rel(0))             # ...then block for the paired TX's per-frame release signal (auto-clears the flag)
    set(x, 1)                        # step 2: predelay
    label("predelay")
    jmp(x_dec, "predelay")     [6]   # 2 iterations x 7 cycles = 14 cycles
    wait(0, pin, 0)                  # step 3: the reply's leading (marker) edge
    set(x, 3)                        # step 4: 4 outer passes
    label("outer")
    set(y, 31)
    label("inner")
    in_(pins, 1)                     # 1 cycle
    jmp(y_dec, "inner")              # 1 cycle - 2 cycles/sample within a pass
    jmp(x_dec, "outer")              # 1 cycle - only reached once per pass, after 32 samples
    wrap()

# EXPERIMENTAL alternative to dshot_bidir_rx: the state machine rebuilds the
# reply itself and hands the CPU one word per reply - the frame's 21 bits with
# the marker at the top, the same integer gcr_decode.reconstruct_frame() returns
# - so the CPU is left with gcr_decode.decode() and check_crc().
#
# It reads the pin once per bit, at the bit's centre, and starts the count for
# the next centre afresh at every flip - the way a hardware UART stays in step
# with a sender whose clock is a little off - so a timing error can grow only
# within one run of equal bits, not across the frame. That needs a receiver
# clock at a whole number of cycles per reply bit: 16, see RLE_CYCLES_PER_BIT.
# It takes exactly 21 bits, so it never has to recognise the end of a frame, and
# a reply's worth of work always ends: it cannot stall the FIFO mid-frame.
#
# What the program does, in order:
#
# 1. Wait for the paired TX's release signal and then for the reply's leading
#    (marker) edge - the same synchronisation as dshot_bidir_rx, see there.
#
# 2. Count down from a preset while the pin holds its level: one pass of the
#    counting loop is 2 cycles (a pin test and a decrement), the loop for the
#    level the pin is at. When the pin flips, jump to the other level's loop with
#    the counter set to a short count, so the next read lands about half a bit
#    after the edge. When the counter runs out with no flip, the read lands one
#    whole bit after the last one.
#
# 3. Read the pin into the input shift register (autopush hands it over after 21
#    reads), count the bit, and reload the counter for a whole bit.
#
# The two levels' paths are made the same length on purpose: the low path spends
# a nop where the high path spends a jump, so a bit is 16 cycles at either level
# and a run of low bits does not drift against a run of high bits.
#
# It takes 19 of a PIO block's 32 instruction slots and dshot_bidir_tx takes 13,
# so a block that carries this pair is full: no other program fits beside it,
# not even dshot_bidir_rx.
@asm_pio(in_shiftdir=PIO.SHIFT_LEFT, autopush=True, push_thresh=21)
def dshot_bidir_rx_rle():
    wrap_target()
    irq(clear, rel(0))               # step 1: as dshot_bidir_rx
    wait(1, irq, rel(0))
    nop()                      [26]  # 27 cycles: the same 4.2us / 2.2us lower bound dshot_bidir_rx waits (its step 2)
    wait(0, pin, 0)                  # the marker edge: the pin is now low
    set(y, 20)                       # 21 reads: y counts 20..0
    label("flip_low")
    set(x, 0)                        # step 2: the pin just went low - 1 pass, then read
    jmp("low")
    label("flip_high")
    set(x, 0)                        # the pin just went high - 1 pass, then read
    label("high")
    jmp(pin, "high_count")           # pin still high: count a pass
    jmp("flip_low")                  # pin went low
    label("high_count")
    jmp(x_dec, "high")               # 2 cycles a pass; falls through when the count is out
    jmp("emit")
    label("again")
    set(x, 4)                  [1]   # step 3: a whole bit - 5 passes, plus the cycles the path around them takes
    jmp(pin, "high")
    nop()                            # matches the jump the high path takes to "emit"
    label("low")
    jmp(pin, "flip_high")            # pin went high
    jmp(x_dec, "low")                # 2 cycles a pass; falls through into "emit" when the count is out
    label("emit")
    in_(pins, 1)                     # the read
    jmp(y_dec, "again")              # after the 21st, wraps back to wait for the next reply

# DSHOT_SPEEDS and BIDIR_PROFILES live in dshot_profiles.py (pure data, no
# hardware imports) so PC-side tooling can read them directly - see that
# module's docstring. Imported and re-exported above.


class DShotPIO:
    """
    Common base for UnidirectionalDShot and BidirectionalDShot - construct one
    of those, not this. Holds everything the two share: the TX state machine,
    start()/stop()/drain(), and send_throttle_command().
    """

    # Words the PIO TX FIFO holds before put() starts blocking
    TX_FIFO_DEPTH = 4

    # Exposed for applications and MotorGroup; see the module constant
    MAX_THROTTLE = MAX_THROTTLE

    # Each subclass overrides this. send_throttle_command() inverts the CRC
    # when it is True, and application code reads it to tell the two apart.
    bidirectional = False

    # Creates the state machine but leaves it inactive - call start() to enable it
    def __init__(self, state_machine_id, pin, dshot_speed, program):
        # Kept so a group of motors can check that no two share hardware
        self.state_machine_id = state_machine_id
        self.pin = pin

        # Kept so BidirectionalDShot.start() can re-init this state machine
        # after a stop() that gave the pin to SIO (see BidirectionalDShot.stop())
        self.dshot_speed = dshot_speed
        self.program = program

        self.sm = StateMachine(state_machine_id, program, freq=dshot_speed,
                                sideset_base=pin, set_base=pin)

        # Wall-clock time of one 16-bit frame at 8 PIO cycles per bit, rounded
        # up so a wait built from it is never short
        self.frame_us = (16 * 8 * 1_000_000 + dshot_speed - 1) // dshot_speed

    def start(self):
        self.sm.active(1)

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

    def send_throttle_command(self, throttle):
        """
        Send a throttle command to the ESC.

        Args:
            throttle: Throttle value (0 to MAX_THROTTLE)

        Note: DShot protocol includes a telemetry request bit, but this implementation
        always sets it to 0. AM32 sends a GCR telemetry reply after every frame once
        bidirectional mode is detected regardless of this bit (confirmed from AM32
        firmware source - see ADR-002), so leaving it 0 does not suppress telemetry.
        """
        if throttle < 0:
            raise InvalidThrottleException("Throttle cannot be negative.")
        if throttle > MAX_THROTTLE:
            raise InvalidThrottleException("Throttle value is too high. Maximum value is " + str(MAX_THROTTLE) + ".")

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

        # The state machine consumes the bits from high order to low order, so the
        # 16-bit packet has to sit in the top of the 32-bit word. put() shifts it
        # there itself: shifting in Python instead makes a heap integer once the
        # value passes 30 bits, an allocation on every frame at higher throttle.
        self.sm.put(dShotPacket, 16)

    # The telemetry interface every motor shares. Only BidirectionalDShot
    # captures replies, so these raise by default: reaching one on a
    # unidirectional motor means the application wired the wrong kind of motor
    # into a place that needs telemetry - its own configuration bug, surfaced
    # loudly rather than as a silent empty result.
    def rx_read(self):
        raise UnsupportedOperationException("This motor captures no telemetry")

    def latest_capture(self):
        raise UnsupportedOperationException("This motor captures no telemetry")

    def decode_capture(self, words):
        raise UnsupportedOperationException("This motor captures no telemetry")


class UnidirectionalDShot(DShotPIO):
    """Sends commands only - the plain DShot waveform, no reply capture."""

    def __init__(self, state_machine_id, pin, dshot_speed=DSHOT_SPEEDS.DSHOT600):
        super().__init__(state_machine_id, pin, dshot_speed, dshot)


class BidirectionalDShot(DShotPIO):
    """
    Sends commands with the inverted TX waveform an AM32 (or other
    bidirectional-capable) ESC needs to auto-detect bidirectional DShot, and
    captures the ESC's GCR telemetry reply on a second state machine sharing
    the same pin. Detection only happens while the ESC is disarmed, so this
    class must be in use for the whole arm sequence - there is no way to arm
    with a normal signal and switch afterward.

    Two receiver programs are available, chosen with the `receiver` argument by
    what each hands the CPU: SAMPLE_RECEIVER (default), dshot_bidir_rx, hands
    the CPU 128 raw samples per reply for gcr_decode.analyze_capture() to
    reconstruct into a frame; FRAME_RECEIVER, the EXPERIMENTAL
    dshot_bidir_rx_rle (see its own comment and ADR-002's "run-length capture"
    section - not yet validated against the sample receiver on live replies),
    reconstructs the frame itself and hands the CPU one word, for the cheaper
    gcr_decode.analyze_frame(). A frame-receiver pair fills its PIO block on
    its own (dshot_bidir_tx is 13 instructions, dshot_bidir_rx_rle 19, of the
    block's 32) - unlike a sample-receiver pair, it cannot share a block with a
    unidirectional motor or another pair.
    """

    bidirectional = True

    # Most captures drain_rx() takes in one call. It bounds how long one call can
    # hold the command loop - which also feeds TX - if the receiver keeps
    # producing while the FIFO is being emptied; anything left over is taken on
    # the next call.
    RX_DRAIN_LIMIT = 4

    # receiver= argument values - see the class docstring
    SAMPLE_RECEIVER = "sample"
    FRAME_RECEIVER = "frame"

    def __init__(self, state_machine_id, pin, dshot_speed=DSHOT_SPEEDS.DSHOT600,
                 rx_state_machine_id=None, receiver=SAMPLE_RECEIVER):
        """
        Args:
            rx_state_machine_id: Required. The second state machine, listening
                on the same pin for the ESC's GCR reply (see dshot_bidir_rx).
                Two constraints, both enforced here:
                (1) Same PIO block as state_machine_id (ids 0-3 -> PIO0, 4-7 ->
                    PIO1, 8-11 -> PIO2 on RP2350). A GPIO's function select
                    routes to one PIO block at a time, and inter-SM IRQs only
                    reach state machines on the same block.
                (2) Exactly state_machine_id + 1. TX and RX synchronise
                    through relative IRQ addressing, which resolves to a flag
                    derived from the executing state machine's own id; a fixed
                    TX-to-RX offset is what gives every pair on a shared block
                    its own private flag (see dshot_bidir_tx).
                start() activates both state machines. From then on RX
                synchronises itself to each TX frame with no further calls -
                the application only has to drain it (see drain_rx(), which
                MotorGroup.update() calls every tick).
            receiver: SAMPLE_RECEIVER (default) or FRAME_RECEIVER - see the
                class docstring.
        """
        # Validate before claiming any hardware: a constructor that raises
        # partway through shouldn't leave a stray, half-configured state
        # machine bound to the pin behind it.
        if rx_state_machine_id is None:
            raise ValueError("rx_state_machine_id is required for BidirectionalDShot")

        if rx_state_machine_id != state_machine_id + 1:
            raise ValueError(
                "rx_state_machine_id must be state_machine_id + 1 (got "
                "state_machine_id=" + str(state_machine_id) +
                ", rx_state_machine_id=" + str(rx_state_machine_id) +
                ") - the TX/RX pair's relative-IRQ synchronization "
                "depends on this fixed offset, see this constructor's "
                "own docstring"
            )

        # Ids 0-3 are PIO0, 4-7 PIO1, 8-11 PIO2: with the +1 offset, a TX on the
        # last state machine of a block would put its RX in the next block
        if state_machine_id // 4 != rx_state_machine_id // 4:
            raise ValueError(
                "state_machine_id " + str(state_machine_id) + " and rx_state_machine_id " +
                str(rx_state_machine_id) + " must share a PIO block (ids 0-3 -> PIO0, "
                "4-7 -> PIO1, 8-11 -> PIO2) - see this constructor's own docstring"
            )

        if receiver not in (self.SAMPLE_RECEIVER, self.FRAME_RECEIVER):
            raise ValueError(
                "receiver must be BidirectionalDShot.SAMPLE_RECEIVER or "
                ".FRAME_RECEIVER, got " + repr(receiver)
            )

        profile = BIDIR_PROFILES.get(dshot_speed)
        if profile is None:
            raise ValueError("BidirectionalDShot needs a dshot_speed with a verified "
                              "BIDIR_PROFILES entry (DSHOT300 or DSHOT600 currently)")

        if receiver == self.FRAME_RECEIVER:
            rx_program = dshot_bidir_rx_rle
            rx_speed = rle_rx_speed(dshot_speed)
            capture_words = 1
            expected_ratio = None
            ratio_tolerance = None
        else:
            rx_program = dshot_bidir_rx
            rx_speed = profile["rx_speed"]
            capture_words = 4
            expected_ratio = profile["expected_ratio"]
            ratio_tolerance = profile["ratio_tolerance"]

        # Both sides release the line between frames (see dshot_bidir_tx), so
        # for part of each frame nobody drives it. An undriven pad can float
        # LOW, which dshot_bidir_rx's wait(0, pin, 0) would read as the start of
        # a reply. A weak pull-up holds the line at its idle-HIGH level while
        # nobody drives, without resisting either side when they do. Pad pull
        # configuration is independent of which peripheral owns the pin, so it
        # still applies once the state machines below take over.
        pin.init(Pin.IN, Pin.PULL_UP)

        super().__init__(state_machine_id, pin, dshot_speed, dshot_bidir_tx)

        # jmp_pin wires the pin dshot_bidir_rx_rle's jmp(pin, ...) instructions
        # read; dshot_bidir_rx has none, so the sample receiver does not need
        # it. Kept on self, alongside the program and clock, so start() can
        # replay the same rx_sm.init() call on every run, not only the first.
        rx_init_kwargs = {"in_base": pin}
        if receiver == self.FRAME_RECEIVER:
            rx_init_kwargs["jmp_pin"] = pin

        self.rx_sm = StateMachine(rx_state_machine_id, rx_program, freq=rx_speed, **rx_init_kwargs)
        self.rx_program = rx_program
        self.rx_init_kwargs = rx_init_kwargs
        self.receiver = receiver
        self.rx_clock_hz = rx_speed
        self.expected_ratio = expected_ratio
        self.ratio_tolerance = ratio_tolerance

        self.rx_state_machine_id = rx_state_machine_id

        # Assembles replies from the RX FIFO and holds the latest one for the
        # application to read from another core.
        #
        # drain_rx(publish) empties the RX FIFO. Call it on every command-loop
        # tick: an undrained FIFO stalls the RX state machine, and the captures
        # taken right after a stall come back corrupted (see ADR-002). It takes
        # whole captures only (capture_words - 4 for the sample receiver, 1 for
        # the frame receiver), leaving fewer waiting words for the next call,
        # so the grouping cannot slip; a completed capture replaces the single
        # published one (read it with latest_capture()) when `publish` is true
        # and is dropped otherwise. It takes at most RX_DRAIN_LIMIT captures
        # per call, does no decoding (that is the application's job, on its
        # own schedule - see decode_capture()), and must be called from one
        # place only (MotorGroup.update()): while the loop runs it is the only
        # writer of the published capture.
        #
        # It is the mailbox's own method, bound here, rather than a method of
        # this class that calls the mailbox: it runs on every tick, and a
        # Python-level call layer per tick measurably slowed the command loop.
        self.mailbox = CaptureMailbox(self.rx_sm, self.RX_DRAIN_LIMIT, utime.ticks_us, capture_words)
        self.drain_rx = self.mailbox.drain

    def start(self):
        # Reclaim the pin from stop()'s SIO hand-off (see stop()'s own
        # comment) before touching either state machine. Re-applying the
        # pull-up here, not just once at construction, means the very first
        # start() and every one after a stop() both get it - and the pin's
        # pull configuration is independent of which peripheral owns it, so
        # this is safe to redo even on a pin that never left PIO.
        self.pin.init(Pin.IN, Pin.PULL_UP)
        self.sm.init(self.program, freq=self.dshot_speed,
                     sideset_base=self.pin, set_base=self.pin)
        self.rx_sm.init(self.rx_program, freq=self.rx_clock_hz, **self.rx_init_kwargs)

        # Start each run from a clean slate. RX listens before TX can release
        # the pin and raise its first irq(rel(1)) (see dshot_bidir_rx's
        # irq(clear, rel(0)) comment for the other half of this). Leftover
        # words from a previous run are flushed because they would misalign
        # the first new capture, and a stale published capture must not
        # survive into the new run.
        while self.rx_sm.rx_fifo():
            self.rx_sm.get()
        self.mailbox.reset()
        self.rx_sm.active(1)
        super().start()

    def rx_read(self):
        """
        Return the next raw captured word from the RX FIFO, or None if empty.

        The receiver synchronises itself to every TX frame via a PIO IRQ - no
        per-read setup call is needed. Each real reply produces this motor's
        capture_words words back to back: 4 for the sample receiver (128
        uniformly-spaced, un-slotted samples covering the marker bit, the 20
        real data bits, and idle tail - see dshot_bidir_rx's comments), 1 for
        the frame receiver (the already-reconstructed 21-bit frame - see
        dshot_bidir_rx_rle's comments). Raw and unpaired: decoding into eRPM is
        a separate step (decode_capture()).

        Diagnostic access. Use either this or drain_rx() on a given motor,
        never both - they consume the same FIFO.
        """
        if not self.rx_sm.rx_fifo():
            return None
        return self.rx_sm.get()

    def latest_capture(self):
        """
        Return the latest published capture as (ticks_us, sequence, words), or
        None if there is none to hand out right now: nothing has been published
        since start(), or the writer kept rewriting it for every attempt (see
        CaptureMailbox). Callers polling for telemetry treat both the same way -
        ask again later.

        ticks_us is utime.ticks_us() at publication (wraps - compare with
        ticks_diff). sequence counts published captures since start(), so a
        caller can tell a fresh capture from one it has already seen. words is
        a tuple of this motor's capture_words raw 32-bit RX words (see
        rx_read()). Safe to call from a different core than drain_rx().
        """
        return self.mailbox.latest()

    def decode_capture(self, words):
        """
        Decode one raw capture with this motor's own receiver and RX profile,
        and return the result dict gcr_decode.analyze_capture() (sample
        receiver) or gcr_decode.analyze_frame() (frame receiver) returns -
        crc_ok is the validity signal in both, a capture that is complete and
        correctly framed can still fail it. The sample receiver's decode costs
        about 1.3ms on the Pico, several command-loop ticks, so call it at
        whatever pace the application can afford, never from the command loop;
        the frame receiver skips the reconstruction that costs most of that
        1.3ms (measured on the spike bench at ~200-215us for decode() plus
        check_crc() alone - not this method's own dict-building overhead).
        """
        if self.receiver == self.FRAME_RECEIVER:
            return gcr_decode.analyze_frame(words[0])
        return gcr_decode.analyze_capture(words, self.rx_clock_hz, self.expected_ratio, self.ratio_tolerance)

    # One shutdown-only wait, not on any hot path: the ESC's reply to the last
    # frame TX sent is still in flight for a while after TX's own last bit -
    # dshot_bidir_rx's own predelay is explicitly a lower bound on when that
    # reply starts, not a measured one, so the actual turnaround can't be
    # computed exactly here. A generous fixed margin, comfortably longer than
    # a whole reply at either supported speed, before handing the pin to SIO
    # avoids fighting the ESC's own drive.
    STOP_REPLY_MARGIN_US = 300

    def stop(self):
        """
        Deactivate both state machines, then hand the pin to SIO, driven low.

        AM32 ESCs reboot into their bootloader after a signal-loss timeout,
        and a bootloader that finds the line permanently high never escapes -
        which is exactly what merely deactivating a released, pulled-up-high
        BidirectionalDShot line leaves behind. Driving it low instead means
        that reboot's own bootloader check sees an actively low line and
        jumps straight back to the application - the same path a
        unidirectional motor's frozen-low line already takes for free. Call
        start() to reclaim the pin for PIO and resume.
        """
        super().stop()
        self.rx_sm.active(0)
        self.rx_sm.restart()
        utime.sleep_us(self.STOP_REPLY_MARGIN_US)
        self.pin.init(Pin.OUT, value=0)
