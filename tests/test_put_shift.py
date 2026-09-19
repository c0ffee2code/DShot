# Test: send_throttle_command() puts the same 32-bit word on the TX FIFO as the
# shift-in-Python version it replaced
#
# Purpose: DShotPIO.send_throttle_command() hands the 16-bit packet to
# StateMachine.put(packet, 16) and lets the C side shift it into the top of the
# 32-bit word, instead of shifting in Python (which allocates a heap integer once
# the value passes 30 bits). The transmit state machines consume that word, so a
# wrong shift would corrupt every frame.
#
# A state machine running a trivial program (pull a word, push it straight back)
# stands in for the transmitter, so the exact FIFO word can be read back. The real
# send_throttle_command() runs against it for every throttle, in both CRC
# polarities, and each word is compared with the formula the old code used. It
# also checks the call no longer allocates.
#
# No motor and no ESC signal: the state machine touches no pin. State machine 11
# (PIO2) is not used by anything else.

import gc
from array import array
from rp2 import PIO, StateMachine, asm_pio

from dshot_pio import DShotPIO, MAX_THROTTLE


@asm_pio()
def echo():
    pull()
    mov(isr, osr)
    push()


class Holder:
    """Just enough of a motor for send_throttle_command() to run against."""

    def __init__(self, sm, bidirectional):
        self.sm = sm
        self.bidirectional = bidirectional


def old_word(throttle, inverted):
    """The word the pre-change code put on the FIFO."""
    packet_value = throttle << 1
    crc = (packet_value ^ (packet_value >> 4) ^ (packet_value >> 8)) & 0x0F
    if inverted:
        crc = (~crc) & 0x0F
    return ((packet_value << 4) | crc) << 16


def test_put_shift():
    print("=== send_throttle_command word test (no motor) ===")
    sm = StateMachine(11, echo, freq=2_000_000)
    sm.active(1)

    mismatches = 0
    for inverted in (False, True):
        holder = Holder(sm, inverted)
        for throttle in range(MAX_THROTTLE + 1):
            DShotPIO.send_throttle_command(holder, throttle)
            got = sm.get()
            if got != old_word(throttle, inverted):
                mismatches += 1
                if mismatches <= 3:
                    print("  MISMATCH throttle %d inverted %s: got %08x want %08x" % (
                        throttle, inverted, got, old_word(throttle, inverted)))
    if mismatches:
        raise Exception("FAIL " + str(mismatches) + " words differ")
    print("  OK   all %d throttles x 2 polarities produce the same word as before" % (2 * (MAX_THROTTLE + 1)))

    holder = Holder(sm, True)
    buf = array('I', [0])  # bulk get() into an array allocates nothing, unlike get()
    for throttle in (0, 300, 1500, MAX_THROTTLE):
        gc.collect()
        gc.disable()
        m0 = gc.mem_alloc()
        for _ in range(200):
            DShotPIO.send_throttle_command(holder, throttle)
            sm.get(buf)
        used = gc.mem_alloc() - m0
        gc.enable()
        print("  throttle %4d: %d bytes allocated over 200 sends" % (throttle, used))
        if used > 300:
            raise Exception("FAIL send allocates at throttle " + str(throttle))
    print("  OK   sends allocate (almost) nothing at any throttle")

    sm.active(0)
    print()
    print("=== Test Complete ===")


test_put_shift()
