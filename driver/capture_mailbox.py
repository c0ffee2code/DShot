# SPDX-License-Identifier: GPL-3.0-or-later
# CaptureMailbox: takes an ESC's telemetry reply from the RX state machine and
# holds the latest completed one for another core to read.
#
# Pure Python with no hardware imports, so it can be exercised on a PC and kept
# apart from the PIO driver that feeds it (see BidirectionalDShot).

from array import array


class CaptureMailbox:
    """
    One writer drains captures into it; any other core may read the latest one.

    A reply arrives as a fixed number of words - 4 for dshot_bidir_rx's raw
    samples, 1 for dshot_bidir_rx_rle's already-reconstructed frame - given as
    capture_words at construction. drain() takes a whole capture from the
    source (the RX state machine's FIFO) in one bulk read, straight into the
    published slot. The slot keeps only the latest capture: the application
    samples telemetry, so a newer capture always replaces an older one.

    Only whole captures are ever read: a capture is complete when capture_words
    words are waiting; with fewer, drain() leaves them where they are, and the
    word grouping cannot slip. How deep the source's own FIFO is relative to
    capture_words - whether it can hold more than one capture at a time - is
    the driver's concern, not this class's (see BidirectionalDShot).

    The two cores run in parallel with no global interpreter lock, and a
    capture is several stores, so a reader could otherwise see half of one
    capture and half of the next. The slot is guarded by a sequence counter
    (a seqlock): 0 means nothing published yet, odd means the writer is
    mid-update, even means stable. The reader copies the words and accepts them
    only if the counter was even and unchanged across the copy.

    drain() runs on every command-loop tick, so it is written for the hot path.
    Reading a capture as one bulk get() into a preallocated array costs about a
    quarter of four separate get() calls and allocates nothing: a single get()
    returns a Python integer, and a 32-bit word above 30 bits is a heap object,
    which fed the garbage collector and stalled both cores when it ran. It is
    also one flat function with no calls to helpers of its own, because a
    Python-level call here costs about as much as the rest of the loop body.
    """

    # How many times latest() re-reads a slot the writer keeps rewriting before
    # giving up, so a reader can never spin
    LATEST_ATTEMPTS = 4

    def __init__(self, source, limit, clock, capture_words=4):
        """
        Args:
            source: Where captures come from - anything with rx_fifo() (how many
                words are waiting) and get(buffer) (fill an array with that many
                words, blocking if fewer are waiting), like an RX state machine.
            limit: Most captures one drain() call takes.
            clock: Zero-argument callable returning a timestamp in microseconds
                (utime.ticks_us on the device), used to stamp published captures.
            capture_words: Words in one capture from this source (see the class
                docstring). Default 4 matches dshot_bidir_rx's raw samples.
        """
        self.source = source
        self.limit = limit
        self.clock = clock
        self.capture_words = capture_words

        self.slot_words = array('I', [0] * capture_words)
        self.slot_ticks_us = 0
        self.slot_seq = 0

        # Where a capture goes when it is being dropped rather than published
        self.scratch = array('I', [0] * capture_words)

    def reset(self):
        """Discard the published capture (start of a run)."""
        self.slot_seq = 0

    def drain(self, publish):
        """
        Take up to `limit` whole captures from the source. Each is published,
        stamped with clock(), when `publish` is true and dropped otherwise.
        Fewer than capture_words words waiting means no complete capture yet:
        nothing is taken, so get() can never block here.

        Must be called from one place only: while running it is the only writer
        of the published slot.
        """
        source = self.source
        remaining = self.limit
        capture_words = self.capture_words
        while remaining and source.rx_fifo() >= capture_words:
            remaining -= 1
            if publish:
                seq = self.slot_seq
                self.slot_seq = seq + 1  # odd: update in progress
                source.get(self.slot_words)
                self.slot_ticks_us = self.clock()
                self.slot_seq = seq + 2  # even: stable
            else:
                source.get(self.scratch)

    def latest(self):
        """
        Return the latest published capture as (ticks_us, sequence, words), or
        None if there is none to hand out right now: nothing has been published
        since reset(), or the writer kept rewriting the slot for every attempt.

        sequence counts published captures since reset(), so a caller can tell
        a fresh capture from one it has already seen. words is a tuple of
        capture_words raw 32-bit words.
        """
        for _ in range(self.LATEST_ATTEMPTS):
            seq = self.slot_seq
            if seq == 0:
                return None
            if seq & 1:
                continue
            words = tuple(self.slot_words)
            ticks_us = self.slot_ticks_us
            if self.slot_seq == seq:
                return (ticks_us, seq >> 1, words)
        return None
