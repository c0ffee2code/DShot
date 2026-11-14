# SPDX-License-Identifier: GPL-3.0-or-later
# CaptureMailbox: takes captures off a bidirectional motor's RX state machine
# and holds the latest one for another core to read. No hardware imports, so
# it runs under the PC tests. Design: ADR-005.

import micropython
from array import array
from micropython import const

from gcr_decode import AM32_NOT_RUNNING_FRAME

# The slot holds frame | token << FRAME_BITS. The token gets the 9 bits left
# below 30, as a larger integer is a heap object per publish.
# const(): folded in where used, so drain() pays nothing for them
FRAME_BITS = const(21)
FRAME_MASK = const((1 << FRAME_BITS) - 1)
TOKEN_MAX = const(511)  # tokens run 1..TOKEN_MAX and wrap, so a published value is never 0


class CaptureMailbox:
    """
    One writer drains captures into a single slot; any core may read the
    latest one.

    The slot keeps only the latest capture, frame and token in one array
    element, so a read on another core is one atomic load and never mixes two
    captures. The token is not a clock: it steps on every publish
    (1..TOKEN_MAX, then 1 again), so a value that differs from the last one
    seen is new. After TOKEN_MAX publishes a token repeats; a caller that
    polls that rarely and meets the same frame holds the published data, but
    cannot tell that it is new. drain() and discard() run on every
    command-loop tick: they allocate nothing and call no helpers.
    """

    def __init__(self, source, limit):
        """
        Args:
            source: Where captures come from - anything with rx_fifo() (how many
                words are waiting) and get() (take one word, blocking if none is
                waiting), like an RX state machine. A word is one whole capture,
                below 2**30 (dshot_bidir_rx_frame pushes 21 bits).
            limit: Most captures one drain() call takes.
        """
        self.source = source
        self.limit = limit

        # frame | token << FRAME_BITS, or 0 while nothing is published
        self.slot = array('I', [0])
        self.token = 0

    def reset(self):
        """Discard the published capture (start of a run)."""
        self.slot[0] = 0
        self.token = 0

    @micropython.native
    def drain(self):
        """
        Take up to `limit` whole captures from the source and publish each, the
        latest replacing the one in the slot. Takes only complete captures, so
        get() never blocks here.

        Call it from one place only: it is the slot's only writer.

        Returns how many captures were AM32's not-running reply
        (gcr_decode.AM32_NOT_RUNNING_FRAME), the count MotorGroup's reply gate
        uses.
        """
        source = self.source
        remaining = self.limit
        not_running = AM32_NOT_RUNNING_FRAME
        replies = 0
        slot = self.slot
        while remaining and source.rx_fifo():
            remaining -= 1
            value = source.get()
            token = self.token + 1
            if token > TOKEN_MAX:
                token = 1
            self.token = token
            slot[0] = (token << FRAME_BITS) | (value & FRAME_MASK)
            if value == not_running:
                replies += 1
        return replies

    @micropython.native
    def discard(self):
        """
        Take up to `limit` whole captures from the source and drop them; for
        arming, when replies are counted but not trusted. Returns the same
        count as drain().
        """
        source = self.source
        remaining = self.limit
        not_running = AM32_NOT_RUNNING_FRAME
        replies = 0
        while remaining and source.rx_fifo():
            remaining -= 1
            value = source.get()
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
