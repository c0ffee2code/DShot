# SPDX-License-Identifier: GPL-3.0-or-later
# Original implementation: https://github.com/jrddupont/DShotPIO
# Licensed under GNU General Public License v3.0
#
# DShot transmit and bidirectional-telemetry receive on the RP2350's PIO, one
# motor per object. Wire protocol: specification/DSHOT.md.
# Design: ADR-002, ADR-005.

import micropython
import utime
from machine import Pin
from rp2 import PIO, StateMachine, asm_pio

import gcr_decode
from capture_mailbox import CaptureMailbox, FRAME_MASK
# DSHOT_SPEEDS and BIDIR_PROFILES are also imported from here by callers
from dshot_profiles import DSHOT_SPEEDS, BIDIR_PROFILES, FRAME_CYCLES_PER_BIT, frame_rx_speed

# Highest value of the 11-bit throttle field of a DShot packet.
MAX_THROTTLE = 2047

# TEMPORARY: highest throttle accepted until higher ones are tested (BUG-005).
THROTTLE_CEILING = 250


class InvalidThrottleException(Exception):
    def __init__(self,message):
        self.message=message

class UnsupportedOperationException(Exception):
    def __init__(self,message):
        self.message=message

# DShot TX. Sends the top 16 bits of each 32-bit FIFO word, MSB first, 8 cycles
# per bit: high 3 cycles for a 0, 6 for a 1. The line idles low between frames.
@asm_pio(sideset_init=PIO.OUT_LOW, out_shiftdir=PIO.SHIFT_LEFT, autopull=True, pull_thresh=16)
def dshot():
    wrap_target()
    label("start")
    out(x, 1)            .side(0)    [1] # 2 cycles, LOW; stalls here when the FIFO is empty
    jmp(not_x, "zero")   .side(1)    [2] # 3 cycles, HIGH
    jmp("start")         .side(1)    [2] # 3 cycles, HIGH: bit 1
    label("zero")
    jmp("start")         .side(0)    [2] # 3 cycles, LOW: bit 0
    wrap()

# Bidirectional DShot TX. dshot()'s bit timing with every level inverted: the
# line idles high, which is how AM32 detects bidirectional mode while disarmed.
# After the 16th bit it releases the pin (input, held high by the pull-up) so
# the ESC can reply, and raises irq(rel(1)) to start the paired receiver; it
# reclaims the pin when the next word arrives. Bits are counted by hand with a
# manual pull(), because autopull gives no point at which to release. Each bit
# path has its own release tail, because an instruction carries one side-set
# value; both tails release from a driven-high line. rel(1) reaches the
# receiver only if it runs on state machine id + 1 of the same block.
@asm_pio(sideset_init=PIO.OUT_HIGH, set_init=PIO.OUT_HIGH, out_shiftdir=PIO.SHIFT_LEFT, autopull=False)
def dshot_bidir_tx():
    label("frame_start")
    pull()                     .side(1)   [1] # pin released; waits here between frames (reply window)
    set(pindirs, 1)            .side(1)   [1] # reclaim pin
    set(y, 15)                 .side(1)   [1] # 16 bits
    label("bitloop")
    out(x, 1)                  .side(1)   [1] # 2 cycles, HIGH
    jmp(not_x, "zero")         .side(0)   [2] # 3 cycles, LOW
    jmp(y_dec, "bitloop")      .side(0)   [2] # 3 cycles, LOW: bit 1
    irq(rel(1))                .side(1)   [0] # after bit 16 (bit 1): HIGH, signal RX
    set(pindirs, 0)            .side(1)   [1] # release
    jmp("frame_start")         .side(1)   [0]
    label("zero")
    jmp(y_dec, "bitloop")      .side(1)   [2] # 3 cycles, HIGH: bit 0
    set(pindirs, 0)            .side(1)   [1] # after bit 16 (bit 0): release
    irq(rel(1))                .side(1)   [0] # signal RX
    jmp("frame_start")         .side(1)   [0]

# Bidirectional DShot RX, paired with dshot_bidir_tx on state machine id + 1 of
# the same block. After the TX's irq(rel(1)) it waits for the line high plus
# 26 cycles, then for the reply's falling marker edge, and reads exactly
# 21 bits, 16 cycles per bit (FRAME_CYCLES_PER_BIT): one read per bit, the count
# restarting at every flip so timing error cannot build up across a frame.
# Autopush hands the CPU one word per capture: the 21-bit frame, marker on top.
# With a silent ESC it captures the driver's own next frame instead.
# A full RX FIFO stalls it holding a complete word; captures are then lost
# until room frees. With dshot_bidir_tx (13 words) it fills the 32-word block.
# scripts/simulate_frame_receiver.py transcribes it by hand: change both.
@asm_pio(in_shiftdir=PIO.SHIFT_LEFT, autopush=True, push_thresh=21)
def dshot_bidir_rx_frame():
    wrap_target()
    irq(clear, rel(0))               # drop a stale TX signal
    wait(1, irq, rel(0))             # TX released the line
    wait(1, pin, 0)            [26]  # line high, then 26 more cycles
    wait(0, pin, 0)                  # marker edge
    set(y, 20)                       # 21 reads
    label("flip_low")
    set(x, 0)                        # flipped low: read in 6 cycles
    jmp("low")
    label("flip_high")
    set(x, 0)                        # flipped high: read in 5 cycles
    label("high")
    jmp(pin, "high_count")
    jmp("flip_low")
    label("high_count")
    jmp(x_dec, "high")               # 2 cycles a pass
    jmp("emit")
    label("again")
    set(x, 4)                  [1]   # one whole bit: 16 cycles to the next read
    jmp(pin, "high")
    nop()                            # evens the low path with the high path's jmp("emit")
    label("low")
    jmp(pin, "flip_high")
    jmp(x_dec, "low")                # 2 cycles a pass
    label("emit")
    in_(pins, 1)                     # read one bit
    jmp(y_dec, "again")


class DShotPIO:
    """
    Common base of UnidirectionalDShot and BidirectionalDShot; construct one of
    those, not this. Holds the TX state machine, start(), stop(), drain() and
    the senders.
    """

    # Words the TX FIFO holds before put() blocks
    TX_FIFO_DEPTH = 4

    MAX_THROTTLE = MAX_THROTTLE
    THROTTLE_CEILING = THROTTLE_CEILING

    # True on BidirectionalDShot: its packets carry the inverted CRC
    bidirectional = False

    # Creates the state machine inactive; start() enables it
    def __init__(self, state_machine_id, pin, dshot_speed, program):
        self.state_machine_id = state_machine_id
        self.pin = pin

        # For re-initialising on every start() and stop()
        self.dshot_speed = dshot_speed
        self.program = program

        self.sm = StateMachine(state_machine_id, program, freq=dshot_speed,
                                sideset_base=pin, set_base=pin)

        # Duration of one 16-bit frame at 8 PIO cycles per bit, rounded up, in us
        self.frame_us = (16 * 8 * 1_000_000 + dshot_speed - 1) // dshot_speed

        # Packet for throttle 0, fixed for this instance; see send_zero()
        self.zero_packet = 0x000F if self.bidirectional else 0x0000

    def init_state_machines(self):
        """
        Put this motor's state machines into their fresh, never-run state.

        Reloads the program, which empties the TX FIFO and restarts at the
        first instruction, so no frame queued by an earlier run is sent.
        """
        self.sm.init(self.program, freq=self.dshot_speed,
                     sideset_base=self.pin, set_base=self.pin)

    def start(self):
        self.sm.active(1)

    def drain(self):
        """
        Block until every queued frame has gone out on the wire.

        Call it before stop() when queued frames must still reach the ESC;
        stop() discards them. Afterwards the line is at its idle level (low;
        for a bidirectional motor high and released). Meant for an active
        state machine; on an inactive one it returns after a few frame times.
        """
        for _ in range(self.TX_FIFO_DEPTH):
            if not self.sm.tx_fifo():
                break
            utime.sleep_us(self.frame_us)

        # An empty FIFO means the last word is in the shift register, still being sent
        utime.sleep_us(self.frame_us)

    def stop(self):
        """
        Deactivate the state machine and leave this motor reset.

        Nothing is transmitted on the way out: the ESC stops the motor on its
        own signal-loss timeout (specification/AM32.md#15). A caller that needs
        it stopped sooner sends throttle 0 first. Queued frames are discarded
        and the pin is left a plain output driven low. start() runs it again.
        """
        self.sm.active(0)
        self.init_state_machines()
        self.pin.init(Pin.OUT, value=0)

    @micropython.native
    def send_throttle_command(self, throttle):
        """
        Queue one frame with this throttle, after checking its range.

        Args:
            throttle: 0 to THROTTLE_CEILING.

        Raises InvalidThrottleException for a negative value or one above
        THROTTLE_CEILING. Values 1-47 are sent as given, and AM32 executes
        them as DShot commands; MotorGroup.set_throttle() refuses them.
        """
        if throttle < 0:
            raise InvalidThrottleException("Throttle cannot be negative.")
        if throttle > THROTTLE_CEILING:
            raise InvalidThrottleException("Throttle value is too high. The temporary bench ceiling is " + str(THROTTLE_CEILING) + ".")

        self.send_unchecked(throttle)

    @micropython.native
    def send_unchecked(self, throttle):
        """
        Queue one frame with this throttle, without a range check.

        For callers that validated the value already, such as
        MotorGroup.update(); others use send_throttle_command(). A value
        outside 0..2047 builds a wrong packet. The telemetry-request bit is
        always 0; AM32 replies in bidirectional mode regardless.
        """
        # 11-bit throttle, telemetry bit 0
        packetValue = throttle << 1

        # 4-bit CRC, inverted for bidirectional DShot
        crc = (packetValue ^ (packetValue >> 4) ^ (packetValue >> 8)) & 0x0F
        if self.bidirectional:
            crc = (~crc) & 0x0F

        # SSSSSSSSSSSTCCCC: throttle, telemetry bit, CRC
        dShotPacket = (packetValue << 4) | crc

        # put() shifts the 16 bits to the top of the word; a Python shift would allocate
        self.sm.put(dShotPacket, 16)

    @micropython.native
    def send_zero(self):
        """
        Queue one frame with throttle 0; same as send_throttle_command(0),
        without building the packet. For MotorGroup.update()'s arming ticks.
        """
        self.sm.put(self.zero_packet, 16)

    # Telemetry interface. Only BidirectionalDShot captures replies; on any
    # other motor these raise, as the application asked the wrong kind of motor.
    def rx_read(self):
        raise UnsupportedOperationException("This motor captures no telemetry")

    def latest_capture(self):
        raise UnsupportedOperationException("This motor captures no telemetry")

    def decode_capture(self, value):
        raise UnsupportedOperationException("This motor captures no telemetry")


class UnidirectionalDShot(DShotPIO):
    """Sends commands only, with the plain DShot waveform; no reply capture."""

    def __init__(self, state_machine_id, pin, dshot_speed=DSHOT_SPEEDS.DSHOT600):
        super().__init__(state_machine_id, pin, dshot_speed, dshot)

    def start(self):
        # Reloading the program takes the pin back from stop()'s plain output
        self.init_state_machines()
        super().start()


class BidirectionalDShot(DShotPIO):
    """
    Sends commands with the inverted waveform and captures the ESC's telemetry
    reply on a second state machine on the same pin.

    AM32 detects bidirectional mode only while disarmed, so this class must
    drive the motor through the whole arm sequence. Its TX/RX pair fills a PIO
    block (dshot_bidir_tx and dshot_bidir_rx_frame): no unidirectional motor
    fits beside it, a second bidirectional pair does. The application drains
    the receiver on every command-loop tick (drain_rx() or discard_rx()).
    A capture is not proof of a reply: with a silent ESC the receiver captures
    the driver's own next frame, and at some throttles that capture passes the
    CRC.
    """

    bidirectional = True

    # Most captures one drain_rx() or discard_rx() call takes; bounds the call's time
    RX_DRAIN_LIMIT = 4

    def __init__(self, state_machine_id, pin, dshot_speed=DSHOT_SPEEDS.DSHOT600,
                 rx_state_machine_id=None):
        """
        Args:
            dshot_speed: DSHOT300 or DSHOT600 (a BIDIR_PROFILES entry).
            rx_state_machine_id: Required; the receiver's state machine. It
                must be state_machine_id + 1, because TX and RX meet on a
                relative IRQ flag, and in the same PIO block (ids 0-3 PIO0,
                4-7 PIO1, 8-11 PIO2), because a pin and inter-state-machine
                IRQs reach one block only.

        Raises ValueError otherwise, before claiming any hardware.
        """
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

        # A TX on the last state machine of a block would put its RX in the next one
        if state_machine_id // 4 != rx_state_machine_id // 4:
            raise ValueError(
                "state_machine_id " + str(state_machine_id) + " and rx_state_machine_id " +
                str(rx_state_machine_id) + " must share a PIO block (ids 0-3 -> PIO0, "
                "4-7 -> PIO1, 8-11 -> PIO2) - see this constructor's own docstring"
            )

        if BIDIR_PROFILES.get(dshot_speed) is None:
            raise ValueError("BidirectionalDShot needs a dshot_speed with a verified "
                              "BIDIR_PROFILES entry (DSHOT300 or DSHOT600 currently)")
        rx_speed = frame_rx_speed(dshot_speed)

        # Holds the released line at its idle high; a floating low would read as a reply
        pin.init(Pin.IN, Pin.PULL_UP)

        super().__init__(state_machine_id, pin, dshot_speed, dshot_bidir_tx)

        # jmp_pin is the pin the receiver's jmp(pin, ...) reads
        rx_init_kwargs = {"in_base": pin, "jmp_pin": pin}
        self.rx_sm = StateMachine(rx_state_machine_id, dshot_bidir_rx_frame,
                                   freq=rx_speed, **rx_init_kwargs)
        self.rx_init_kwargs = rx_init_kwargs
        self.rx_clock_hz = rx_speed

        self.rx_state_machine_id = rx_state_machine_id

        # drain_rx() publishes captures for latest_capture(); discard_rx() drops
        # them (arming). One of them on every tick, from one caller only: an
        # undrained RX FIFO stalls the receiver. Bound directly to the mailbox's
        # methods, as a call layer per tick costs loop time.
        self.mailbox = CaptureMailbox(self.rx_sm, self.RX_DRAIN_LIMIT)
        self.drain_rx = self.mailbox.drain
        self.discard_rx = self.mailbox.discard

    def init_state_machines(self):
        """
        Fresh TX and RX state machines, an empty RX FIFO and no published
        capture, so nothing from a previous run reaches the next one.
        """
        super().init_state_machines()
        self.rx_sm.init(dshot_bidir_rx_frame, freq=self.rx_clock_hz, **self.rx_init_kwargs)
        while self.rx_sm.rx_fifo():
            self.rx_sm.get()
        self.mailbox.reset()

    def start(self):
        # Take the pin back from stop()'s plain output, with the pull-up
        self.pin.init(Pin.IN, Pin.PULL_UP)

        # RX first, so it is waiting before TX raises its first irq(rel(1))
        self.init_state_machines()
        self.rx_sm.active(1)
        super().start()

    def rx_read(self):
        """
        Return the next raw word from the RX FIFO, or None if it is empty.

        One word per capture: a 21-bit reply frame, or the driver's own next
        frame when the ESC is silent. Undecoded (see decode_capture()).
        Diagnostic access: use this or drain_rx() on a motor, never both, as
        they consume the same FIFO.
        """
        if not self.rx_sm.rx_fifo():
            return None
        return self.rx_sm.get()

    def latest_capture(self):
        """
        Return the latest published capture as one integer, or None while
        nothing has been published since start().

        The low 21 bits are the frame; the bits above are a token that changes
        on every publish, so a value that differs from the last one seen is a
        new capture. No timestamp. Pass it to decode_capture() unchanged. Safe
        to call from another core than drain_rx().
        """
        return self.mailbox.latest()

    def decode_capture(self, value):
        """
        Decode one capture as latest_capture() returned it; returns
        gcr_decode.analyze_frame()'s dict.

        crc_ok False means the capture is not a valid reply. crc_ok True does
        not prove a reply: a capture of the driver's own frame from a silent
        ESC passes the CRC at some throttles.
        """
        return gcr_decode.analyze_frame(value & FRAME_MASK)

    # Wait in stop() for a reply still in flight; longer than a whole reply at either speed
    STOP_REPLY_MARGIN_US = 300

    def stop(self):
        """
        Deactivate both state machines, reset them, and leave the pin a plain
        output driven low.

        Low, not released high: an AM32 ESC reboots into its bootloader after
        a signal-loss timeout and stays there while the line is held high.
        Nothing is transmitted on the way out; no queued frame or capture
        survives into the next start().
        """
        self.sm.active(0)
        self.rx_sm.active(0)
        utime.sleep_us(self.STOP_REPLY_MARGIN_US)
        self.init_state_machines()
        self.pin.init(Pin.OUT, value=0)
