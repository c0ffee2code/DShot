# SPDX-License-Identifier: GPL-3.0-or-later
# CaptureMailbox: takes an ESC's telemetry reply from the RX state machine and
# holds the latest completed one for another core to read.
#
# Pure Python with no hardware imports, so it can be exercised on a PC and kept
# apart from the PIO driver that feeds it (see BidirectionalDShot).

from array import array

from gcr_decode import AM32_NOT_RUNNING_FRAME

# Words in one capture: dshot_bidir_rx_frame pushes one already-reconstructed
# 21-bit frame per reply.
WORDS = 1


class CaptureMailbox:
    """
    One writer drains captures into it; any other core may read the latest one.

    A reply arrives as exactly WORDS words. drain() takes a whole capture from
    the source (the RX state machine's FIFO) in one bulk read, straight into
    the published slot. The slot keeps only the latest capture: the
    application samples telemetry, so a newer capture always replaces an
    older one.

    Only whole captures are ever read: a capture is complete when WORDS words
    are waiting; with fewer, drain() leaves them where they are, and the word
    grouping cannot slip. How deep the source's own FIFO is relative to
    WORDS - whether it can hold more than one capture at a time - is the
    driver's concern, not this class's (see BidirectionalDShot).

    The two cores run in parallel with no global interpreter lock, and
    publishing a capture is more than one store (the word, then the
    timestamp), so a reader could otherwise see one from an old capture and
    one from a new one. The slot is guarded by a sequence counter (a
    seqlock): 0 means nothing published yet, odd means the writer is
    mid-update, even means stable. The reader copies the words and accepts them
    only if the counter was even and unchanged across the copy.

    drain() runs on every command-loop tick, so it is written for the hot
    path: get() into a preallocated array allocates nothing (a word above 30
    bits would otherwise be a heap object - see ADR-002 for the measured GC
    cost of that), and it is one flat function with no calls to helpers of
    its own, since a Python-level call costs about as much as the rest of the
    loop body.
    """

    # How many times latest() re-reads a slot the writer keeps rewriting before
    # giving up, so a reader can never spin
    LATEST_ATTEMPTS = 4

    # Diagnostic (BUG-002): most low-streak-ended events reset_log holds - see
    # enable_class_bins(). Not a ring: entries past this are silently dropped,
    # matching MotorGroup.REBOOT_LOG_CAPACITY's own choice.
    RESET_LOG_CAPACITY = 16

    def __init__(self, source, limit, clock):
        """
        Args:
            source: Where captures come from - anything with rx_fifo() (how many
                words are waiting) and get(buffer) (fill an array with that many
                words, blocking if fewer are waiting), like an RX state machine.
            limit: Most captures one drain() call takes.
            clock: Zero-argument callable returning a timestamp in microseconds
                (utime.ticks_us on the device), used to stamp published captures.
        """
        self.source = source
        self.limit = limit
        self.clock = clock

        self.slot_words = array('I', [0] * WORDS)
        self.slot_ticks_us = 0
        self.slot_seq = 0

        # Where a capture goes when it is being dropped rather than published
        self.scratch = array('I', [0] * WORDS)

        # Diagnostic (BUG-002): ground-truth classification of every capture
        # drain() takes, bucketed over time - see enable_class_bins(). None
        # (the default) costs one None-check per capture.
        self.class_bins = None
        self.class_bin_width_us = 0
        self.class_bin_count = 0
        self.class_bin_t0_us = 0

        # Diagnostic (BUG-002): ground-truth log of every low-streak-ended
        # event - a run of zero("low")-classified captures starting and then
        # stopping, AM32's startup-tune signature observed directly, whether
        # or not this motor has ever replied yet. Populated alongside
        # class_bins - see enable_class_bins() and MotorGroup.reboot_log().
        # Always allocated (16 uint32s x 2 is nothing) since it is only ever
        # written inside the same "class_bins is not None" check as the bins.
        self.reset_log_ms = array('I', [0] * self.RESET_LOG_CAPACITY)
        self.reset_log_gap_ms = array('I', [0] * self.RESET_LOG_CAPACITY)
        self.reset_log_count = 0
        self.in_low_streak = False
        self.low_streak_start_us = 0

    def reset(self):
        """Discard the published capture (start of a run). Also re-zeroes and
        re-times the class-bin diagnostic, if enabled, so it reflects only
        this run - see enable_class_bins()."""
        self.slot_seq = 0
        if self.class_bins is not None:
            for i in range(len(self.class_bins)):
                self.class_bins[i] = 0
            self.class_bin_t0_us = self.clock()
            self.reset_log_count = 0
            self.in_low_streak = False

    def enable_class_bins(self, bin_width_us, num_bins):
        """
        Diagnostic (BUG-002): from the next reset() (start()) on, drain() also
        classifies every capture it takes - published or not - into
        not_running / zero ("low") / other, bucketed into num_bins windows of
        bin_width_us each, timed from reset()'s own clock() reading. This is
        the one place that sees every capture the receiver ever produces, so
        it is immune to how often another core happens to poll
        latest_capture() - unlike a capture log built from polling, which is
        what motivated this (see bug-reports/BUG-002-...md).

        class_bins holds 3 uint32s per bin, in class order (not_running,
        zero, other): class_bins[i*3 + c]. A capture that would land before
        time 0 or past the last bin is folded into the nearest end, rather
        than lost, since reset() (not this call) is what actually starts the
        clock.

        Reusable across repeated arm() cycles in one session: call once after
        construction, and each reset() re-zeroes and re-times it. 0 disables
        it again (the array is kept, only class_bin_width_us drives whether
        drain() uses it).

        Also enables reset_log: every time a run of zero("low")-classified
        captures ends, drain() records (ms_since_t0, duration_ms) into it -
        see MotorGroup.reboot_log(), which is what reads it. Unlike the bins,
        this needs no separate array to size, so there is no toggle for it
        beyond this call. A streak still open when the run ends is not
        recorded - the bin dump already shows that case directly, as a
        low-dominated tail with no matching close.
        """
        if self.class_bins is None or self.class_bin_count != num_bins:
            self.class_bins = array('I', [0] * (num_bins * 3))
        self.class_bin_width_us = bin_width_us
        self.class_bin_count = num_bins

    def drain(self, publish):
        """
        Take up to `limit` whole captures from the source. Each is published,
        stamped with clock(), when `publish` is true and dropped otherwise.
        Fewer than WORDS words waiting means no complete capture yet: nothing
        is taken, so get() can never block here.

        Must be called from one place only: while running it is the only writer
        of the published slot.

        Returns how many of the captures taken were AM32's not-running reply
        (gcr_decode.AM32_NOT_RUNNING_FRAME), published or not: MotorGroup's
        arming gate counts those as the ESC answering (bug-reports/BUG-003).
        One integer compare per capture.

        Also feeds class_bins, if enable_class_bins() was called - see there.
        """
        source = self.source
        remaining = self.limit
        not_running = AM32_NOT_RUNNING_FRAME
        replies = 0
        class_bins = self.class_bins
        last_bin = self.class_bin_count - 1
        bin_width_us = self.class_bin_width_us
        t0_us = self.class_bin_t0_us
        while remaining and source.rx_fifo() >= WORDS:
            remaining -= 1
            if publish:
                seq = self.slot_seq
                self.slot_seq = seq + 1  # odd: update in progress
                source.get(self.slot_words)
                now = self.clock()
                self.slot_ticks_us = now
                self.slot_seq = seq + 2  # even: stable
                value = self.slot_words[0]
            else:
                source.get(self.scratch)
                value = self.scratch[0]
                now = self.clock() if class_bins is not None else 0
            if value == not_running:
                replies += 1
            if class_bins is not None:
                # Plain subtraction, not ticks_diff: one arming attempt is
                # well under ticks_us's ~71-minute wraparound period.
                index = (now - t0_us) // bin_width_us
                if index < 0:
                    index = 0
                elif index > last_bin:
                    index = last_bin
                cls = 0 if value == not_running else (1 if value == 0 else 2)
                class_bins[index * 3 + cls] += 1
                if cls == 1:
                    if not self.in_low_streak:
                        self.in_low_streak = True
                        self.low_streak_start_us = now
                elif self.in_low_streak:
                    self.in_low_streak = False
                    count = self.reset_log_count
                    if count < self.RESET_LOG_CAPACITY:
                        self.reset_log_ms[count] = (self.low_streak_start_us - t0_us) // 1000
                        self.reset_log_gap_ms[count] = (now - self.low_streak_start_us) // 1000
                        self.reset_log_count = count + 1
        return replies

    def latest(self):
        """
        Return the latest published capture as (ticks_us, sequence, words), or
        None if there is none to hand out right now: nothing has been published
        since reset(), or the writer kept rewriting the slot for every attempt.

        sequence counts published captures since reset(), so a caller can tell
        a fresh capture from one it has already seen. words is a tuple of
        WORDS raw 32-bit words.
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
