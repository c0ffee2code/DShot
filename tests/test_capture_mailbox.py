# Test: CaptureMailbox word assembly and slot semantics
#
# Purpose: CaptureMailbox is pure Python with no hardware, so it can be checked
# on a PC as well as on the Pico:
#   python tests/test_capture_mailbox.py         (from the project root, on a PC)
#   python scripts/deploy.py test_capture_mailbox.py   (on the Pico)
#
# It covers the single-threaded behaviour: a capture completes on the fourth
# word, a partial capture carries over between calls, publishing replaces the
# slot and advances the sequence, reset() clears both, and a reader gives up
# instead of spinning when the writer is mid-update. Consistency between two
# cores is checked separately by tests/test_capture_slot_stress.py.

import sys

sys.path.insert(0, "driver")

from capture_mailbox import CaptureMailbox


def check(condition, label):
    if not condition:
        raise Exception("FAIL " + label)
    print("  OK   " + label)


class FakeSource:
    """Stands in for an RX state machine: rx_fifo() counts words, get() takes one."""

    def __init__(self, words):
        self.words = list(words)

    def rx_fifo(self):
        return len(self.words)

    def get(self):
        return self.words.pop(0)


def feed(mailbox, words, publish_ticks=None, limit=100):
    """Drain `words` through the mailbox; returns how many words were left over."""
    source = FakeSource(words)
    mailbox.source = source
    mailbox.limit = limit
    mailbox.clock = lambda: publish_ticks
    mailbox.drain(publish_ticks is not None)
    return len(source.words)


def test_capture_mailbox():
    print("=== CaptureMailbox Test ===")

    mailbox = CaptureMailbox(FakeSource([]), 100, lambda: 0)
    check(mailbox.latest() is None, "nothing published to start with")

    feed(mailbox, [1, 2, 3], 500)
    check(mailbox.latest() is None, "three words do not complete a capture")
    feed(mailbox, [4], 1000)
    check(mailbox.latest() == (1000, 1, (1, 2, 3, 4)), "the fourth word completes and publishes it, sequence 1")

    feed(mailbox, [5, 6], 1500)
    check(mailbox.latest() == (1000, 1, (1, 2, 3, 4)), "a partial capture leaves the published one alone")
    feed(mailbox, [7, 8], 2000)
    check(mailbox.latest() == (2000, 2, (5, 6, 7, 8)), "a partial capture carries over and completes correctly")

    feed(mailbox, [9, 10, 11, 12, 13, 14, 15, 16], 3000)
    check(mailbox.latest() == (3000, 4, (13, 14, 15, 16)), "the latest replaces earlier ones, sequence 4")

    feed(mailbox, [1, 2, 3, 4], None)
    check(mailbox.latest() == (3000, 4, (13, 14, 15, 16)), "with publish off a completed capture is dropped")
    feed(mailbox, [1, 2], None)
    feed(mailbox, [3, 4], 4000)
    check(mailbox.latest() == (4000, 5, (1, 2, 3, 4)), "grouping stays aligned across unpublished words")

    check(feed(mailbox, list(range(40)), 5000, limit=16) == 24, "a call takes at most `limit` words")
    mailbox.reset()
    check(mailbox.latest() is None, "reset clears the published capture")
    feed(mailbox, [1], 1)
    mailbox.reset()
    feed(mailbox, [1, 2, 3, 4], 10)
    check(mailbox.latest() == (10, 1, (1, 2, 3, 4)), "reset also drops a partial capture; sequence restarts")

    words = (0xFFFFFFFF, 0x80000000, 0, 0x12345678)
    feed(mailbox, list(words), 20)
    check(mailbox.latest()[2] == words, "32-bit words survive unchanged")

    mailbox.slot_seq = 5
    check(mailbox.latest() is None, "a slot left mid-update is not returned, and the reader gives up")

    print()
    print("=== Test Complete ===")


test_capture_mailbox()
