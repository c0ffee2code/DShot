# SPDX-License-Identifier: GPL-3.0-or-later
# CaptureMailbox: takes an ESC's telemetry reply from the RX state machine and
# holds the latest completed one for another core to read.
#
# Pure Python with no hardware imports, so it can be exercised on a PC and kept
# apart from the PIO driver that feeds it (see BidirectionalDShot).

import micropython
from array import array
from micropython import const

from gcr_decode import AM32_NOT_RUNNING_FRAME

# Words in one capture: dshot_bidir_rx_frame pushes one already-reconstructed
# 21-bit frame per reply.
WORDS = 1

# The slot is one array element: the reply frame in the low FRAME_BITS bits and a
# token above it. MicroPython keeps an integer unboxed only up to 30 bits, so the
# token gets the 9 bits that are left; a larger value would be a heap object per
# publish (see ADR-002 and ADR-005 for what that garbage costs).
# const(): folded into the code where it is used, so drain() pays nothing for them
FRAME_BITS = const(21)
FRAME_MASK = const((1 << FRAME_BITS) - 1)
TOKEN_MAX = const(511)  # tokens run 1..TOKEN_MAX and wrap, so a published value is never 0


class CaptureMailbox:
    """
    One writer drains captures into it; any other core may read the latest one.

    A reply arrives as exactly WORDS words. drain() takes a whole capture from
    the source (the RX state machine's FIFO). The slot keeps only the latest
    capture: the application samples telemetry, so a newer capture always
    replaces an older one.

    Only whole captures are ever read: a capture is complete when WORDS words
    are waiting; with fewer, drain() leaves them where they are, and the word
    grouping cannot slip. How deep the source's own FIFO is relative to
    WORDS - whether it can hold more than one capture at a time - is the
    driver's concern, not this class's (see BidirectionalDShot).

    The two cores run in parallel with no global interpreter lock, so what a
    reader sees must never be half of one capture and half of the next. The
    slot is therefore a single array element, frame and token together: a
    store or a load of one element is atomic, so there is nothing to tear and
    nothing to retry. The token is the "ETag" of the capture, not a clock: it
    steps by one on every publish (1..TOKEN_MAX, then back to 1), so a caller
    that keeps the last value it saw knows a capture is new when the value
    differs. A token comes round again after TOKEN_MAX publishes; a caller
    that polls that rarely, and meets the same frame as well, holds the data
    that was published - only the claim that it is new is lost.

    drain() runs on every command-loop tick, so it is written for the hot
    path: get() into a preallocated array allocates nothing (a word above 30
    bits would otherwise be a heap object - see ADR-002 for the measured GC
    cost of that), and it is one flat function with no calls to helpers of
    its own, since a Python-level call costs about as much as the rest of the
    loop body.
    """

    def __init__(self, source, limit):
        """
        Args:
            source: Where captures come from - anything with rx_fifo() (how many
                words are waiting) and get(buffer) (fill an array with that many
                words, blocking if fewer are waiting), like an RX state machine.
            limit: Most captures one drain() call takes.
        """
        self.source = source
        self.limit = limit

        # frame | token << FRAME_BITS, or 0 while nothing is published
        self.slot = array('I', [0])
        # Where a capture is read to; drain() keeps the count and the one store
        # to the slot to itself
        self.scratch = array('I', [0] * WORDS)
        self.token = 0

    def reset(self):
        """Discard the published capture (start of a run)."""
        self.slot[0] = 0
        self.token = 0

    # measured win - see ADR-002's "Further hot-path tightening" section
    @micropython.native
    def drain(self):
        """
        Take up to `limit` whole captures from the source and publish each, the
        latest replacing the one in the slot. Fewer than WORDS words waiting
        means no complete capture yet: nothing is taken, so get() can never
        block here.

        Must be called from one place only: while running it is the only writer
        of the slot.

        Returns how many of the captures taken were AM32's not-running reply
        (gcr_decode.AM32_NOT_RUNNING_FRAME): MotorGroup's arming gate counts
        those as the ESC answering (bug-reports/BUG-003). One integer compare
        per capture.
        """
        source = self.source
        remaining = self.limit
        not_running = AM32_NOT_RUNNING_FRAME
        replies = 0
        scratch = self.scratch
        slot = self.slot
        while remaining and source.rx_fifo() >= WORDS:
            remaining -= 1
            source.get(scratch)
            value = scratch[0]
            token = self.token + 1
            if token > TOKEN_MAX:
                token = 1
            self.token = token
            slot[0] = (token << FRAME_BITS) | (value & FRAME_MASK)
            if value == not_running:
                replies += 1
        return replies

    # The loop MotorGroup.update() runs on every arming tick: replies still have
    # to be taken off the FIFO (an undrained FIFO stalls the receiver) and
    # counted for the arming gate, but are not yet trusted, so none is published.
    # Kept as small as it was before the slot became a single word - arming
    # reliability reacts to this path's cost.
    @micropython.native
    def discard(self):
        """
        Take up to `limit` whole captures from the source and drop them. Same
        count as drain(): how many were AM32's not-running reply.
        """
        source = self.source
        remaining = self.limit
        not_running = AM32_NOT_RUNNING_FRAME
        replies = 0
        while remaining and source.rx_fifo() >= WORDS:
            remaining -= 1
            source.get(self.scratch)
            value = self.scratch[0]
            if value == not_running:
                replies += 1
        return replies

    def latest(self):
        """
        Return the latest published capture as one integer (the frame in the low
        FRAME_BITS bits, the token above it), or None while nothing has been
        published since reset(). Compare it with the value seen last time: it
        differs when a newer capture has been published. The frame is
        `value & FRAME_MASK`.
        """
        value = self.slot[0]
        if value == 0:
            return None
        return value
