# CaptureMailbox: capture-taking and one-slot semantics.
#
# CaptureMailbox is pure Python. These cover the single-threaded behaviour: the
# slot is one integer (frame with a token above it), publishing replaces it and
# steps the token, a call takes at most `limit` captures, dropping keeps later
# captures aligned, reset() clears the slot, and the token wraps without ever
# making a published value 0; discard() takes captures without publishing. Behaviour with two cores running is checked on the
# Pico by tests/bench/pio_and_cores/test_capture_slot_stress.py.

import unittest

import fakes  # noqa: F401  puts driver/ on sys.path
from capture_mailbox import CaptureMailbox, FRAME_BITS, FRAME_MASK, TOKEN_MAX
from gcr_decode import AM32_NOT_RUNNING_FRAME


class FakeSource:
    """Stands in for an RX state machine: rx_fifo() counts words, get(buf) fills buf."""

    def __init__(self):
        self.words = []

    def add(self, words):
        self.words.extend(words)

    def rx_fifo(self):
        return len(self.words)

    def get(self, buf):
        # a real bulk get() blocks when fewer words are waiting than the buffer holds
        if len(self.words) < len(buf):
            raise AssertionError("get() would have blocked: %d words waiting" % len(self.words))
        for i in range(len(buf)):
            buf[i] = self.words.pop(0)


def new_mailbox(source, limit=4):
    return CaptureMailbox(source, limit)


def packed(token, frame):
    return (token << FRAME_BITS) | frame


class CaptureMailboxTest(unittest.TestCase):
    def setUp(self):
        self.source = FakeSource()
        self.mailbox = new_mailbox(self.source)

    def test_nothing_published_to_start_with(self):
        self.assertIsNone(self.mailbox.latest())

    def test_a_single_word_makes_a_whole_capture(self):
        self.source.add([0x1FFFFF])
        self.mailbox.drain()
        self.assertEqual(self.mailbox.latest(), packed(1, 0x1FFFFF))
        self.assertEqual(self.source.words, [])

    def test_the_frame_is_the_low_bits_of_the_value(self):
        self.source.add([0xC8BB3])
        self.mailbox.drain()
        self.assertEqual(self.mailbox.latest() & FRAME_MASK, 0xC8BB3)

    def test_several_waiting_captures_are_all_taken_latest_wins(self):
        self.source.add([1, 2, 3, 4])
        self.mailbox.drain()
        self.assertEqual(self.mailbox.latest(), packed(4, 4))
        self.assertEqual(self.source.words, [])

    def test_a_call_takes_at_most_limit_captures(self):
        self.source.add(list(range(100, 110)))  # 10 captures waiting, limit is 4
        self.mailbox.drain()
        self.assertEqual(len(self.source.words), 6)
        self.assertEqual(self.mailbox.latest(), packed(4, 103))

    def test_dropping_takes_captures_without_publishing_and_stays_aligned(self):
        self.source.add([1])
        self.mailbox.drain()
        before = self.mailbox.latest()

        self.source.add([2, 3])
        self.mailbox.discard()
        self.assertEqual(self.mailbox.latest(), before)
        self.assertEqual(self.source.words, [])

        self.source.add([4])
        self.mailbox.drain()
        self.assertEqual(self.mailbox.latest(), packed(2, 4))

    def test_reset_clears_the_slot_and_restarts_the_token(self):
        self.source.add([1])
        self.mailbox.drain()
        self.mailbox.reset()
        self.assertIsNone(self.mailbox.latest())
        self.source.add([2])
        self.mailbox.drain()
        self.assertEqual(self.mailbox.latest(), packed(1, 2))

    def test_the_same_frame_published_twice_reads_as_a_new_value(self):
        self.source.add([0x52951])
        self.mailbox.drain()
        first = self.mailbox.latest()
        self.source.add([0x52951])
        self.mailbox.drain()
        self.assertNotEqual(self.mailbox.latest(), first)

    def test_an_all_zero_reply_is_still_a_published_capture(self):
        self.source.add([0])
        self.mailbox.drain()
        self.assertIsNotNone(self.mailbox.latest())
        self.assertEqual(self.mailbox.latest() & FRAME_MASK, 0)

    def test_a_published_value_is_never_zero_across_the_whole_token_cycle(self):
        for _ in range(3 * TOKEN_MAX):
            self.source.add([0])
            self.mailbox.drain()
            self.assertNotEqual(self.mailbox.latest(), 0)

    def test_the_token_runs_one_to_max_and_wraps_to_one(self):
        tokens = []
        for _ in range(TOKEN_MAX + 2):
            self.source.add([7])
            self.mailbox.drain()
            tokens.append(self.mailbox.latest() >> FRAME_BITS)
        self.assertEqual(tokens[0], 1)
        self.assertEqual(tokens[TOKEN_MAX - 1], TOKEN_MAX)
        self.assertEqual(tokens[TOKEN_MAX], 1, "wrapped past the top")
        self.assertEqual(tokens[TOKEN_MAX + 1], 2)

    def test_the_value_stays_inside_the_unboxed_integer_range(self):
        # MicroPython keeps an integer unboxed up to 30 bits: no allocation per publish
        for _ in range(TOKEN_MAX + 1):
            self.source.add([FRAME_MASK])
            self.mailbox.drain()
            self.assertLess(self.mailbox.latest(), 1 << 30)

    def test_bits_above_the_frame_are_not_published(self):
        # the receiver pushes 21 bits; a stray higher bit must not corrupt the token
        self.source.add([0xFFFFFFFF])
        self.mailbox.drain()
        self.assertEqual(self.mailbox.latest(), packed(1, FRAME_MASK))


class NotRunningCountTest(unittest.TestCase):
    """drain() and discard() report how many captures were AM32's not-running
    reply, for MotorGroup's arming gate, whether they publish them or not."""

    def test_counted_when_published_and_when_dropped(self):
        for method in ("drain", "discard"):
            source = FakeSource()
            mailbox = new_mailbox(source)
            source.add([AM32_NOT_RUNNING_FRAME, 0, AM32_NOT_RUNNING_FRAME, 0xC8BB3])
            self.assertEqual(getattr(mailbox, method)(), 2, method)

    def test_nothing_waiting_counts_nothing(self):
        self.assertEqual(new_mailbox(FakeSource()).discard(), 0)
        self.assertEqual(new_mailbox(FakeSource()).drain(), 0)


if __name__ == "__main__":
    unittest.main()
