# Tests of scripts/session_timeline.py: the main Pico's decode stamps laid on the observer's wire record.
# No real recording exists for this tool yet, so the session here is built from known edges and a known clock offset,
# and the checks are that the lag and the alignment come back as built.

import json
import random
import struct
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import bench_paths  # noqa: F401  (the wingman tools and driver/ are importable from here on)

import gcr_decode
import session_timeline
import observer_edge_log
from test_observer_edge_log import edge_payload, packets, tx_frame_edges

FRAME_US = 1045
INSTR_PER_FRAME = FRAME_US * 20                      # one instruction is 50 ns
LINE_HIGH_TICKS = (1 << 30) - 5_000                  # close to the wrap, so the conversion has to handle it


def reply_levels(data12):
    """The 21 line levels of a running reply with this 12-bit value (as test_observer_edge_log builds them)."""
    number = (data12 << 4) | gcr_decode.crc_inverted(data12)
    symbols = 0
    for shift in (12, 8, 4, 0):
        symbols = (symbols << 5) | gcr_decode.GCR_ENCODE_TABLE[(number >> shift) & 0xF]
    levels = 0
    level = 0
    for k in range(19, -1, -1):
        level ^= (symbols >> k) & 1
        levels = (levels << 1) | level
    return levels


def capture_word(frame, token=1):
    return frame | (token << gcr_decode.FRAME_LENGTH_BITS)


def token_of(wire_index, first_published):
    """The token the mailbox gives the reply at this wire index when the reply at first_published was its first publish."""
    return (wire_index - first_published) % session_timeline.TOKEN_MAX + 1


def reply_edges_instr(frame, start_instr):
    """Edges (instructions, new level) of a reply whose bits are the line levels of frame, 750 kbit/s."""
    bit = 26.667
    edges = []
    level = 1
    for k in range(gcr_decode.FRAME_LENGTH_BITS):
        bit_level = (frame >> (gcr_decode.FRAME_LENGTH_BITS - 1 - k)) & 1
        if bit_level != level:
            edges.append((round(start_instr + k * bit), bit_level))
            level = bit_level
    if level == 0:
        edges.append((round(start_instr + gcr_decode.FRAME_LENGTH_BITS * bit), 1))
    return edges


class TicksTests(unittest.TestCase):
    def test_difference_across_the_wrap(self):
        self.assertEqual(session_timeline.ticks_diff_us(10, (1 << 30) - 5), 15)
        self.assertEqual(session_timeline.ticks_diff_us((1 << 30) - 5, 10), -15)


class SampleFileTests(unittest.TestCase):
    def write(self, directory, records):
        path = Path(directory) / "capture.bin"
        path.write_bytes(b"".join(struct.pack(session_timeline.EXPECTED_RECORD_FMT, *r) for r in records))
        return path

    def test_stamp_is_counted_from_the_start_line_across_the_wrap(self):
        with tempfile.TemporaryDirectory() as directory:
            ticks = (LINE_HIGH_TICKS + 2_500_000) & session_timeline.TICKS_MASK     # 2.5 s after the line rose
            path = self.write(directory, [(ticks, 2 | session_timeline.FLAG_FAILED, 0, 0, 215, 0, 0, 0, 77, 0)])
            samples = session_timeline.read_samples(path, {"record_fmt": session_timeline.EXPECTED_RECORD_FMT}, LINE_HIGH_TICKS)
        self.assertEqual(len(samples), 1)
        self.assertAlmostEqual(samples[0]["t_ms"], 2500.0)
        self.assertEqual(samples[0]["motor"], 2)
        self.assertTrue(samples[0]["failed"])
        self.assertEqual(samples[0]["capture"], 77)

    def test_another_record_format_is_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write(directory, [])
            with self.assertRaises(session_timeline.SessionError):
                session_timeline.read_samples(path, {"record_fmt": "<IB4H"}, 0)
            with self.assertRaises(session_timeline.SessionError):
                session_timeline.read_samples(path, {}, 0)


def make_replies(frames, period_ms=1.045, first_end_ms=100.0):
    return [{"end_ms": first_end_ms + k * period_ms, "status": "run", "erpm": 1, "frame": frame} for k, frame in enumerate(frames)]


def make_sample(replies, wire_index, first_published, lag_ms, frame=None):
    reply = replies[wire_index]
    word = capture_word(reply["frame"] if frame is None else frame, token_of(wire_index, first_published))
    return {"t_ms": reply["end_ms"] + lag_ms, "motor": 0, "failed": False, "capture": word}


class PlaceTests(unittest.TestCase):
    FIRST = 7

    def test_a_steady_eRPM_is_still_placed_by_the_token(self):
        # every reply carries the same frame, so a match by value would take the newest reply before the stamp (lag under
        # 1 ms) instead of the true one 15 ms back; 1200 replies make the token wrap once
        frame = reply_levels((3 << 9) | 120)
        replies = make_replies([frame] * 1200)
        samples = [make_sample(replies, k, self.FIRST, 15.0) for k in (50, 300, 620, 900, 1150)]
        armed_ms = replies[self.FIRST - 1]["end_ms"] + 0.5
        placed = session_timeline.place_samples(samples, replies, armed_ms)
        self.assertEqual(placed["agree"], 5)
        for _, lag in placed["pairs"]:
            self.assertAlmostEqual(lag, 15.0, places=6)
        self.assertGreater(len(placed["tied"]), 1)                      # nothing in the frames pins the offset: said so
        low, high = placed["tied_lag_ms"]
        self.assertLessEqual(low, 15.0)
        self.assertGreaterEqual(high, 15.0)

    def test_changing_frames_pin_the_offset_without_a_prior(self):
        rng = random.Random(5)
        frames = [reply_levels((3 << 9) | rng.randrange(100, 500)) for _ in range(1200)]
        replies = make_replies(frames)
        samples = [make_sample(replies, k, self.FIRST, 15.0) for k in (50, 300, 620, 900, 1150)]
        placed = session_timeline.place_samples(samples, replies, None)
        self.assertEqual(placed["tied"], [self.FIRST % session_timeline.TOKEN_MAX])
        self.assertEqual(placed["agree"], 5)
        for _, lag in placed["pairs"]:
            self.assertAlmostEqual(lag, 15.0, places=6)

    def test_a_frame_that_differs_from_the_wire_is_counted(self):
        rng = random.Random(6)
        frames = [reply_levels((3 << 9) | rng.randrange(100, 500)) for _ in range(300)]
        replies = make_replies(frames)
        samples = [make_sample(replies, k, self.FIRST, 15.0) for k in (50, 100, 150, 200)]
        samples[2] = make_sample(replies, 150, self.FIRST, 15.0, frame=reply_levels((3 << 9) | 77))
        placed = session_timeline.place_samples(samples, replies, replies[self.FIRST - 1]["end_ms"] + 0.5)
        self.assertEqual((placed["agree"], placed["disagree"]), (3, 1))
        self.assertIsNone(placed["pairs"][2][1])

    def test_not_running_replies_are_placed_too(self):
        rng = random.Random(7)
        frames = [gcr_decode.AM32_NOT_RUNNING_FRAME] * 40 + [reply_levels((3 << 9) | rng.randrange(100, 500)) for _ in range(300)]
        replies = make_replies(frames)
        samples = [make_sample(replies, k, self.FIRST, 15.0) for k in (20, 100, 200, 300)]
        placed = session_timeline.place_samples(samples, replies, replies[self.FIRST - 1]["end_ms"] + 0.5)
        self.assertEqual(placed["agree"], 4)
        self.assertAlmostEqual(placed["pairs"][0][1], 15.0, places=6)

    def test_a_stamp_before_every_reply_is_unplaced(self):
        replies = make_replies([reply_levels((3 << 9) | 120)] * 5)
        sample = {"t_ms": 10.0, "motor": 0, "failed": False, "capture": capture_word(replies[0]["frame"], 1)}
        placed = session_timeline.place_samples([sample], replies, None)
        self.assertEqual(placed["unplaced"], 1)

    def test_gaps_are_per_motor(self):
        samples = [{"t_ms": 0.0, "motor": 0}, {"t_ms": 30.0, "motor": 1}, {"t_ms": 100.0, "motor": 0}, {"t_ms": 140.0, "motor": 1}]
        self.assertEqual(session_timeline.sample_gaps(samples), [(100.0, 100.0), (140.0, 110.0)])


class WholeSessionTests(unittest.TestCase):
    """A session folder built from known edges: 40 frames of 1045 us on one line (GPIO 6), each answered by a running reply
    whose eRPM changes every frame; the main Pico stamps every 10th reply 7 ms after the reply ended."""
    FRAMES = 40
    LAG_MS = 7.0
    ARM_MS = 12.0
    FIRST = 5                                         # wire index of the first reply the main Pico published

    def build(self, directory):
        directory = Path(directory)
        edges = []
        reply_ends_ms = {}
        for k in range(self.FRAMES):
            start = 1000 + k * INSTR_PER_FRAME
            edges += tx_frame_edges(48 << 5, start)
            frame = reply_levels((3 << 9) | (100 + k))
            reply_start = start + 16 * 34 + 500
            edges += reply_edges_instr(frame, reply_start)
            reply_ends_ms[k] = (reply_start + gcr_decode.FRAME_LENGTH_BITS * 26.667) * 50e-6
        (directory / "signal_observation.bin").write_bytes(packets({0: edge_payload(edges)}, start="first_pin=6 pins=6 marker=v2"))
        records = []
        for k in range(self.FIRST, self.FRAMES, 10):
            ticks = (LINE_HIGH_TICKS + round((reply_ends_ms[k] + self.LAG_MS) * 1000)) & session_timeline.TICKS_MASK
            frame = reply_levels((3 << 9) | (100 + k))
            records.append((ticks, 0, 0, 0, 0, 0, capture_word(frame, token_of(k, self.FIRST)), 0, 0, 0))
        (directory / "capture.bin").write_bytes(b"".join(struct.pack(session_timeline.EXPECTED_RECORD_FMT, *r) for r in records))
        device = {"record_fmt": session_timeline.EXPECTED_RECORD_FMT, "line_high_ticks_us": str(LINE_HIGH_TICKS),
                  "arm_ticks_us": str((LINE_HIGH_TICKS + int(self.ARM_MS * 1000)) & session_timeline.TICKS_MASK),
                  "armed_ticks_us": str((LINE_HIGH_TICKS + round((reply_ends_ms[self.FIRST - 1] + 0.5) * 1000)) & session_timeline.TICKS_MASK),
                  "sample_every": "10", "update_interval_us": "1045", "bidir_motor_indices": "0",
                  "schedule_behind_at_ms_since_arm": "5,9"}
        (directory / "session.json").write_text(json.dumps({"session": "fake", "device": device}), encoding="utf-8")
        (directory / "scenario.json").write_text(json.dumps({"motors": [{"pin": 6}]}), encoding="utf-8")
        return reply_ends_ms

    def test_lag_and_alignment_come_back_as_built(self):
        with tempfile.TemporaryDirectory() as directory:
            self.build(directory)
            session = session_timeline.load_session(directory)
            self.assertEqual(session["line_of_motor"], {0: 0})
            result = session_timeline.build(session)
            samples = session_timeline.read_samples(Path(directory) / "capture.bin", session["device"], session["line_high"])
            session_timeline.attach_samples(result, samples)
        data = result["lines"][0]
        self.assertEqual(len(data["rows"]), self.FRAMES)
        self.assertEqual(len(data["replies"]), self.FRAMES)
        lags = [lag for _, lag in data["lags"]]
        self.assertEqual(len(lags), 4)
        self.assertEqual(data["placement"]["agree"], 4)
        for lag in lags:
            self.assertIsNotNone(lag)
            self.assertAlmostEqual(lag, self.LAG_MS, delta=0.1)
        self.assertEqual([round(b) for b in result["behind"]], [17, 21])   # 12 ms of arm() + 5 and 9

    def test_the_picture_is_written(self):
        with tempfile.TemporaryDirectory() as directory:
            self.build(directory)
            session = session_timeline.load_session(directory)
            result = session_timeline.build(session)
            samples = session_timeline.read_samples(Path(directory) / "capture.bin", session["device"], session["line_high"])
            session_timeline.attach_samples(result, samples)
            out = Path(directory) / "timeline.png"
            session_timeline.plot(result, str(out), "fake")
            self.assertTrue(out.stat().st_size > 1000)

    def test_a_session_without_the_pico_record_is_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(session_timeline.SessionError):
                session_timeline.load_session(directory)


if __name__ == "__main__":
    unittest.main()
