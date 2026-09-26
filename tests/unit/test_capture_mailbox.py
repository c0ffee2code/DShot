# CaptureMailbox: capture-taking and one-slot semantics.
#
# CaptureMailbox is pure Python. These cover the single-threaded behaviour:
# publishing replaces the slot and advances the sequence, a call takes at most
# `limit` captures, dropping keeps later captures aligned, reset() clears the
# slot, and a reader gives up instead of spinning when the writer is
# mid-update. Consistency between two cores is checked on the Pico by
# tests/device/test_capture_slot_stress.py.

import unittest

import fakes  # noqa: F401  puts driver/ on sys.path
from capture_mailbox import CaptureMailbox


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
    ticks = [0]

    def clock():
        ticks[0] += 100
        return ticks[0]

    return CaptureMailbox(source, limit, clock)


class CaptureMailboxTest(unittest.TestCase):
    def setUp(self):
        self.source = FakeSource()
        self.mailbox = new_mailbox(self.source)

    def test_nothing_published_to_start_with(self):
        self.assertIsNone(self.mailbox.latest())

    def test_a_single_word_makes_a_whole_capture(self):
        self.source.add([0x1FFFFF])
        self.mailbox.drain(True)
        self.assertEqual(self.mailbox.latest(), (100, 1, (0x1FFFFF,)))
        self.assertEqual(self.source.words, [])

    def test_several_waiting_captures_are_all_taken_latest_wins(self):
        self.source.add([1, 2, 3, 4])
        self.mailbox.drain(True)
        # 4 captures published, one clock() call each -> ticks_us is the last one's
        self.assertEqual(self.mailbox.latest(), (400, 4, (4,)))
        self.assertEqual(self.source.words, [])

    def test_a_call_takes_at_most_limit_captures(self):
        self.source.add(list(range(100, 110)))  # 10 captures waiting, limit is 4
        self.mailbox.drain(True)
        self.assertEqual(len(self.source.words), 6)
        self.assertEqual(self.mailbox.latest(), (400, 4, (103,)))

    def test_dropping_takes_captures_without_publishing_and_stays_aligned(self):
        self.source.add([1])
        self.mailbox.drain(True)
        before = self.mailbox.latest()

        self.source.add([2, 3])
        self.mailbox.drain(False)
        self.assertEqual(self.mailbox.latest(), before)
        self.assertEqual(self.source.words, [])

        self.source.add([4])
        self.mailbox.drain(True)
        self.assertEqual(self.mailbox.latest(), (200, 2, (4,)))

    def test_reset_clears_the_slot_and_restarts_the_sequence(self):
        self.source.add([1])
        self.mailbox.drain(True)
        self.mailbox.reset()
        self.assertIsNone(self.mailbox.latest())
        self.source.add([2])
        self.mailbox.drain(True)
        self.assertEqual(self.mailbox.latest()[1], 1)

    def test_32_bit_words_survive_unchanged(self):
        for word in (0xFFFFFFFF, 0x80000000, 0, 0x12345678):
            self.source.add([word])
            self.mailbox.drain(True)
            self.assertEqual(self.mailbox.latest()[2], (word,))

    def test_a_slot_left_mid_update_is_not_returned(self):
        self.mailbox.slot_seq = 5  # odd: the writer was in the middle of an update
        self.assertIsNone(self.mailbox.latest())


if __name__ == "__main__":
    unittest.main()
