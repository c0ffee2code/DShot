# tests/bench/dshot_frame_decode.py on synthetic edges built the way dshot_bidir_tx puts a frame on the line
# (the line idles high; each bit starts with 2 of its 8 PIO cycles high, then low for 6 cycles for a "1"
# and 3 cycles for a "0", then high for the rest). Times are in 0.1us; one PIO cycle at DSHOT600 is
# 2.0833 units.

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tests" / "bench"))

import dshot_frame_decode as wd  # noqa: E402

CYCLE = 100 / 48            # 0.2083us per cycle at 4.8 MHz, in 0.1us units = 2.0833


def packet(throttle, telemetry=0):
    value = (throttle << 1) | telemetry
    crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0xF
    return (value << 4) | crc


def frame_edges(word, start, cycle=CYCLE):
    """Edges of one frame starting at `start` (time of the first cycle of bit 15)."""
    edges = []
    t = float(start)
    for k in range(15, -1, -1):
        bit = (word >> k) & 1
        edges.append((round(t + 2 * cycle), 0))                       # falls after 2 cycles high
        low = 6 if bit else 3
        edges.append((round(t + (2 + low) * cycle), 1))               # rises after the low part
        t += 8 * cycle
    return edges


class CrcTest(unittest.TestCase):
    def test_a_correct_packet_passes_and_a_flipped_bit_fails(self):
        word = packet(60)
        self.assertTrue(wd.crc_ok(word))
        self.assertFalse(wd.crc_ok(word ^ 0x20))

    def test_the_polarity_is_the_inverted_one(self):
        # throttle 0 with the non-inverted CRC (0) would be rejected: the inverted one is 0xF
        self.assertEqual(packet(0) & 0xF, 0xF)


class DecodeTest(unittest.TestCase):
    def test_a_frame_decodes_to_the_throttle_it_carries(self):
        for throttle in (0, 48, 60, 100, 250, 2047):
            frame = wd.decode_frame(frame_edges(packet(throttle), 1000))
            self.assertTrue(frame["ok"])
            self.assertEqual(frame["throttle"], throttle)
            self.assertEqual(frame["telemetry"], 0)
            self.assertTrue(frame["crc_ok"])

    def test_the_telemetry_bit_is_read(self):
        frame = wd.decode_frame(frame_edges(packet(60, telemetry=1), 1000))
        self.assertEqual(frame["telemetry"], 1)
        self.assertTrue(frame["crc_ok"])

    def test_a_flipped_bit_is_reported_as_a_crc_failure(self):
        frame = wd.decode_frame(frame_edges(packet(60) ^ 0x100, 1000))
        self.assertTrue(frame["ok"])
        self.assertFalse(frame["crc_ok"])

    def test_bit_timing_is_measured(self):
        summary = wd.timing_summary([wd.decode_frame(frame_edges(packet(60), 1000))])
        self.assertAlmostEqual(summary["period_mean"], 16.667, delta=0.3)
        self.assertAlmostEqual(summary["one_low"], 12.5, delta=0.6)
        self.assertAlmostEqual(summary["zero_low"], 6.25, delta=0.6)

    def test_frames_are_split_by_the_quiet_gap(self):
        edges = frame_edges(packet(0), 1000) + frame_edges(packet(60), 11000) + frame_edges(packet(60), 21000)
        frames = wd.decode_frames(edges)
        self.assertEqual([f["throttle"] for f in frames], [0, 60, 60])

    def test_a_recording_that_starts_inside_a_frame_gives_a_partial_one_that_is_not_ok(self):
        edges = frame_edges(packet(60), 1000)[10:] + frame_edges(packet(60), 11000)
        frames = wd.decode_frames(edges)
        self.assertFalse(frames[0]["ok"])
        self.assertTrue(frames[1]["ok"])

    def test_a_slow_transmitter_clock_still_decodes_and_shows_in_the_period(self):
        frame = wd.decode_frame(frame_edges(packet(60), 1000, cycle=CYCLE * 1.05))
        self.assertTrue(frame["crc_ok"])
        self.assertGreater(wd.timing_summary([frame])["period_mean"], 17.2)


if __name__ == "__main__":
    unittest.main()
