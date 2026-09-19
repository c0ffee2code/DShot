# Test: CaptureMailbox capture-taking and slot semantics
#
# Purpose: CaptureMailbox is pure Python with no hardware, so it can be checked
# on a PC as well as on the Pico:
#   python tests/test_capture_mailbox.py         (from the project root, on a PC)
#   python scripts/deploy.py test_capture_mailbox.py   (on the Pico)
#
# It covers the single-threaded behaviour: only whole 4-word captures are taken,
# fewer than 4 waiting words are left alone (so the word grouping cannot slip
# and a bulk read cannot block), publishing replaces the slot and advances the
# sequence, a call takes at most `limit` captures, dropping keeps later captures
# aligned, reset() clears the slot, and a reader gives up instead of spinning when
# the writer is mid-update. Consistency between two cores is checked separately by
# tests/test_capture_slot_stress.py.

import sys

sys.path.insert(0, "driver")

from capture_mailbox import CaptureMailbox


def check(condition, label):
    if not condition:
        raise Exception("FAIL " + label)
    print("  OK   " + label)


class FakeSource:
    """Stands in for an RX state machine: rx_fifo() counts words, get(buf) fills buf."""

    def __init__(self, words=()):
        self.words = list(words)

    def add(self, words):
        self.words.extend(words)

    def rx_fifo(self):
        return len(self.words)

    def get(self, buf):
        # a real bulk get() blocks when fewer words are waiting than the buffer holds
        if len(self.words) < len(buf):
            raise Exception("FAIL get() would have blocked: " + str(len(self.words)) + " words waiting")
        for i in range(len(buf)):
            buf[i] = self.words.pop(0)


def new_mailbox(source, limit=4):
    ticks = [0]

    def clock():
        ticks[0] += 100
        return ticks[0]

    return CaptureMailbox(source, limit, clock)


def test_capture_mailbox():
    print("=== CaptureMailbox Test ===")

    source = FakeSource()
    mailbox = new_mailbox(source)
    check(mailbox.latest() is None, "nothing published to start with")

    source.add([1, 2, 3])
    mailbox.drain(True)
    check(mailbox.latest() is None and len(source.words) == 3, "three waiting words are left alone, nothing published")

    source.add([4])
    mailbox.drain(True)
    check(mailbox.latest() == (100, 1, (1, 2, 3, 4)) and not source.words, "the fourth word makes a capture: published, sequence 1")

    source.add([5, 6])
    mailbox.drain(True)
    check(mailbox.latest() == (100, 1, (1, 2, 3, 4)) and len(source.words) == 2, "a partial capture leaves the published one alone")
    source.add([7, 8])
    mailbox.drain(True)
    check(mailbox.latest() == (200, 2, (5, 6, 7, 8)), "it completes aligned once the rest arrives")

    source.add([9, 10, 11, 12, 13, 14, 15, 16, 17, 18])
    mailbox.drain(True)
    check(mailbox.latest() == (400, 4, (13, 14, 15, 16)) and source.words == [17, 18],
          "several waiting captures are all taken, the latest replaces earlier ones, the partial stays")
    source.words = []

    source.add(list(range(100, 140)))
    mailbox.drain(True)
    check(len(source.words) == 40 - 16 and mailbox.latest()[1] == 8, "a call takes at most `limit` captures (4 of 10)")
    source.words = []

    before = mailbox.latest()
    source.add([1, 2, 3, 4])
    mailbox.drain(False)
    check(mailbox.latest() == before and not source.words, "with publish off a capture is taken and dropped")

    source.add([31, 32])
    mailbox.drain(False)
    check(len(source.words) == 2, "a partial capture is left alone when dropping too")
    source.add([33, 34, 35, 36, 37, 38])
    mailbox.drain(False)
    check(mailbox.latest() == before and not source.words, "later captures stay aligned after words were dropped")
    source.add([41, 42, 43, 44])
    mailbox.drain(True)
    check(mailbox.latest()[2] == (41, 42, 43, 44), "publishing resumes with the next whole capture")

    mailbox.reset()
    check(mailbox.latest() is None, "reset clears the published capture")
    source.add([1, 2, 3, 4])
    mailbox.drain(True)
    check(mailbox.latest()[1] == 1, "sequence restarts after reset")

    words = (0xFFFFFFFF, 0x80000000, 0, 0x12345678)
    source.add(list(words))
    mailbox.drain(True)
    check(mailbox.latest()[2] == words, "32-bit words survive unchanged")

    mailbox.slot_seq = 5
    check(mailbox.latest() is None, "a slot left mid-update is not returned, and the reader gives up")

    print()
    print("=== Test Complete ===")


test_capture_mailbox()
