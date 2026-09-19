# SPDX-License-Identifier: GPL-3.0-or-later
# CaptureMailbox: assembles an ESC's telemetry reply from the words the RX state
# machine produces, and holds the latest completed one for another core to read.
#
# Pure Python with no hardware imports, so it can be exercised on a PC and kept
# apart from the PIO driver that feeds it (see BidirectionalDShot).

from array import array


class CaptureMailbox:
    """
    One writer drains words into it; any other core may read the latest capture.

    A reply arrives as 4 words (128 samples). drain() collects them from a word
    source (the RX state machine's FIFO) and publishes each completed capture
    into the single published slot. The slot keeps only the latest capture: the
    application samples telemetry, so a newer capture always replaces an older
    one.

    The two cores run in parallel with no global interpreter lock, and a
    capture is several stores, so a reader could otherwise see half of one
    capture and half of the next. The slot is guarded by a sequence counter
    (a seqlock): 0 means nothing published yet, odd means the writer is
    mid-update, even means stable. The reader copies the words and accepts them
    only if the counter was even and unchanged across the copy.

    drain() runs on every command-loop tick, so it is written for the hot path:
    every buffer is allocated once, because allocating costs time and invites
    garbage-collection pauses, and it is one flat function with no calls to
    helpers of its own, because a Python-level call here costs about as much as
    the rest of the loop body.
    """

    # Words in one capture
    WORDS = 4

    # How many times latest() re-reads a slot the writer keeps rewriting before
    # giving up, so a reader can never spin
    LATEST_ATTEMPTS = 4

    def __init__(self, source, limit, clock):
        """
        Args:
            source: Where words come from - anything with rx_fifo() (how many
                are waiting) and get() (take one), like an RX state machine.
            limit: Most words one drain() call takes.
            clock: Zero-argument callable returning a timestamp in microseconds
                (utime.ticks_us on the device), used to stamp published captures.
        """
        self.source = source
        self.limit = limit
        self.clock = clock

        # The capture being assembled. A partial group carries over between
        # drain() calls, so it survives being drained in several pieces.
        self.buf = array('I', [0] * self.WORDS)
        self.fill = 0

        self.slot_words = array('I', [0] * self.WORDS)
        self.slot_ticks_us = 0
        self.slot_seq = 0

    def reset(self):
        """Discard the partial capture and the published one (start of a run)."""
        self.fill = 0
        self.slot_seq = 0

    def drain(self, publish):
        """
        Move up to `limit` words from the source into captures. A completed
        capture is published, stamped with clock(), when `publish` is true and
        dropped otherwise; either way the word grouping stays aligned.

        Must be called from one place only: while running it is the only writer
        of the published slot.
        """
        source = self.source
        buf = self.buf
        words = self.WORDS
        for _ in range(self.limit):
            if not source.rx_fifo():
                break
            fill = self.fill
            buf[fill] = source.get()
            fill += 1
            if fill < words:
                self.fill = fill
                continue
            self.fill = 0
            if publish:
                seq = self.slot_seq
                self.slot_seq = seq + 1  # odd: update in progress
                slot = self.slot_words
                slot[0] = buf[0]
                slot[1] = buf[1]
                slot[2] = buf[2]
                slot[3] = buf[3]
                self.slot_ticks_us = self.clock()
                self.slot_seq = seq + 2  # even: stable

    def latest(self):
        """
        Return the latest published capture as (ticks_us, sequence, words), or
        None if there is none to hand out right now: nothing has been published
        since reset(), or the writer kept rewriting the slot for every attempt.

        sequence counts published captures since reset(), so a caller can tell
        a fresh capture from one it has already seen. words is a tuple of the 4
        raw 32-bit words.
        """
        for _ in range(self.LATEST_ATTEMPTS):
            seq = self.slot_seq
            if seq == 0:
                return None
            if seq & 1:
                continue
            slot = self.slot_words
            words = (slot[0], slot[1], slot[2], slot[3])
            ticks_us = self.slot_ticks_us
            if self.slot_seq == seq:
                return (ticks_us, seq >> 1, words)
        return None
