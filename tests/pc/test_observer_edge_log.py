# Tests of tests/bench/wingman/tools/observer_edge_log.py, which turns the wingman's stream into edges, frames and replies.
# The streams are built here from known edges, so every constant (instruction overhead, 15-bit field,
# marker) is checked against the arithmetic documented at the top of tests/bench/wingman/board/observer.py.

import struct
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import bench_paths  # noqa: F401  (the wingman tools and driver/ are importable from here on)

import observer_edge_log
import gcr_decode

MARKER = observer_edge_log.MARKER


def encode_runs(runs):
    """runs: [(level of the run that ended, counted ticks)] -> payload bytes as the PIO program makes them."""
    entries = []
    for level, ticks in runs:
        while ticks >= MARKER:
            entries.append((level << 15) | MARKER)
            ticks -= MARKER
        entries.append((level << 15) | (MARKER - ticks))
    if len(entries) % 2:
        entries.append(MARKER)                   # padding: a quiet-line marker, which carries no edge
    return b"".join(struct.pack("<I", (a << 16) | b) for a, b in zip(entries[::2], entries[1::2]))


def packets(channels, start="first_pin=0", end="reason=ended valid=1"):
    out = b"OK" + bytes((observer_edge_log.MAGIC, ord("S"), 255, len(start), 0)) + start.encode()
    for ch, payload in channels.items():
        out += bytes((observer_edge_log.MAGIC, ord("D"), ch, len(payload) & 0xFF, len(payload) >> 8)) + payload
    return out + bytes((observer_edge_log.MAGIC, ord("E"), 255, len(end), 0)) + end.encode()


def runs_for(edge_list):
    """Edges (time in instructions, new level) -> the runs the program reports: the time between two detected
    edges minus the fixed overhead, in ticks of two instructions."""
    runs = []
    last = 0
    for t, level in edge_list:
        ended = 1 - level
        overhead = observer_edge_log.HIGH_RUN_OVERHEAD if ended else observer_edge_log.LOW_RUN_OVERHEAD
        runs.append((ended, (t - last - overhead) // 2))
        last = t
    return runs


def tx_frame_edges(word, start):
    """A frame as edges in instructions: 34 per bit period, low for 25 of them for a 1 and 12 for a 0."""
    edges = []
    t = start
    for bit in range(15, -1, -1):
        one = (word >> bit) & 1
        edges.append((t, 0))
        edges.append((t + (25 if one else 12), 1))
        t += 34
    return edges


class StreamTests(unittest.TestCase):
    def test_round_trip_of_packets(self):
        payload = encode_runs([(1, 100), (0, 25), (1, 12)])
        start, channels, end = observer_edge_log.parse_stream(packets({2: payload}))
        self.assertEqual(start, "first_pin=0")
        self.assertEqual(bytes(channels[2]), payload)
        self.assertTrue(observer_edge_log.parse_end(end)["valid"])

    def test_reader_accepts_any_chunking(self):
        data = packets({0: encode_runs([(1, 5)] * 40)})
        reader = observer_edge_log.StreamReader()
        for i in range(0, len(data), 7):
            reader.feed(data[i:i + 7])
        self.assertTrue(reader.done)
        self.assertEqual(len(reader.channels[0]), 80)

    def test_missing_end_packet_is_an_error(self):
        with self.assertRaises(observer_edge_log.StreamError):
            observer_edge_log.parse_stream(packets({})[:-5])

    def test_damaged_framing_is_an_error(self):
        data = bytearray(packets({0: encode_runs([(1, 5)] * 4)}))
        data[2 + 5 + len("first_pin=0")] = 0x00          # the first data packet's magic byte
        with self.assertRaises(observer_edge_log.StreamError):
            observer_edge_log.parse_stream(bytes(data))

    def test_parse_end_per_channel(self):
        info = observer_edge_log.parse_end("reason=ended ch0 bytes=12 overruns=0 rx_stall=0 ch1 bytes=8 overruns=2 rx_stall=1 valid=0")
        self.assertEqual(info["channels"][1], {"bytes": 8, "overruns": 2, "rx_stall": 1})
        self.assertFalse(info["valid"])


class EdgeTests(unittest.TestCase):
    def test_edge_times_follow_the_instruction_arithmetic(self):
        wanted = [(10, 0), (36, 1), (80, 0)]
        got = observer_edge_log.edges(edge_payload(wanted))
        # one instruction is 50ns = half a unit of 0.1us
        self.assertEqual([round(t * 2) for t, _ in got], [t for t, _ in wanted])
        self.assertEqual([level for _, level in got], [0, 1, 0])

    def test_quiet_line_marker_adds_time_without_an_edge(self):
        got = observer_edge_log.edges(encode_runs([(1, 3 * MARKER + 10)]))
        self.assertEqual(len(got), 1)
        # every marker is 65540 instructions: 6 more than its 2 * MARKER counting ones
        self.assertEqual(round(got[0][0] * 2), 2 * (3 * MARKER + 10) + 3 * observer_edge_log.MARKER_EXTRA + observer_edge_log.HIGH_RUN_OVERHEAD)

    def test_a_3277us_gap_is_exact_to_the_instruction_in_both_stream_versions(self):
        self.assertEqual(2 * MARKER + observer_edge_log.MARKER_EXTRA, 65540)       # 3277.0us at 50ns

    def test_v2_edge_right_after_a_marker_is_not_taken_for_a_marker(self):
        # a low run of 32767 ticks exactly, then a high run of 5 ticks: v2 markers carry the opposite level bit
        low_marker = (1 << 15) | MARKER                   # still low: bit inverted -> 1
        low_end_at_zero = (0 << 15) | MARKER              # edge found at once after the marker: the run's own bit, field 0x7FFF
        high_end = (1 << 15) | (MARKER - 5)
        words = [low_marker, low_end_at_zero, high_end, low_marker]
        payload = observer_edge_log.Payload(b"".join(struct.pack("<I", (a << 16) | b) for a, b in zip(words[::2], words[1::2])))
        payload.marker_v2 = True
        got = observer_edge_log.edges(payload)
        self.assertEqual([level for _, level in got], [1, 0])
        t0 = 2 * MARKER + observer_edge_log.MARKER_EXTRA + observer_edge_log.LOW_RUN_OVERHEAD
        self.assertEqual(round(got[0][0] * 2), t0)
        self.assertEqual(round(got[1][0] * 2), t0 + 2 * 5 + observer_edge_log.HIGH_RUN_OVERHEAD)

    def test_the_same_bits_in_a_v1_stream_lose_that_edge(self):
        # documents why v2 exists: v1 cannot tell the two apart
        words = [(0 << 15) | MARKER, (0 << 15) | MARKER]
        payload = struct.pack("<I", (words[0] << 16) | words[1])
        self.assertEqual(observer_edge_log.edges(payload), [])

    def test_reconcile_reports_missing_bytes(self):
        info = observer_edge_log.parse_end("reason=ended ch0 bytes=12 overruns=0 rx_stall=0 ch1 bytes=8 overruns=0 rx_stall=0 valid=1")
        self.assertEqual(observer_edge_log.reconcile({0: b"x" * 12, 1: b"y" * 8}, info), [])
        self.assertEqual(len(observer_edge_log.reconcile({0: b"x" * 12, 1: b"y" * 4}, info)), 1)

    def test_a_frame_decodes_through_dshot_frame_decode(self):
        value = 60 << 1
        crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0xF
        word = (value << 4) | crc
        records = observer_edge_log.analyse(observer_edge_log.edges(edge_payload(tx_frame_edges(word, 100))))
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]["tx"]["throttle"], 60)
        self.assertTrue(records[0]["tx"]["crc_ok"])
        self.assertIsNone(records[0]["reply"])


    def test_a_unidirectional_frame_decodes_with_plain_crc(self):
        value = 60 << 1
        crc = (value ^ (value >> 4) ^ (value >> 8)) & 0xF
        word = (value << 4) | crc
        # idle low, a bit is a high pulse: the mirror image of the bidirectional shape
        mirrored = [(t, 1 - level) for t, level in tx_frame_edges(word, 100)]
        payload = edge_payload(mirrored)
        record = observer_edge_log.analyse(observer_edge_log.edges(payload, invert=True), unidirectional=True)[0]
        self.assertEqual(record["tx"]["throttle"], 60)
        self.assertTrue(record["tx"]["crc_ok"])
        self.assertIsNone(record["reply"])
        # read as bidirectional, the same frame is not valid: the CRC rule differs
        wrong = observer_edge_log.analyse(observer_edge_log.edges(payload, invert=True))[0]
        self.assertFalse(wrong["tx"]["crc_ok"])


def edge_payload(edge_list):
    return encode_runs(runs_for(edge_list))


class ReplyTests(unittest.TestCase):
    def reply_edges(self, frame, start, bit=13.333):
        """Edges of a 21-bit reply whose line levels are the bits of frame, top bit first (times in 0.1us)."""
        edges = []
        level = 1
        for k in range(observer_edge_log.REPLY_BITS):
            bit_level = (frame >> (observer_edge_log.REPLY_BITS - 1 - k)) & 1
            if bit_level != level:
                edges.append((start + k * bit, bit_level))
                level = bit_level
        if level == 0:
            edges.append((start + observer_edge_log.REPLY_BITS * bit, 1))
        return edges

    def test_not_running_reply(self):
        reply = observer_edge_log.decode_reply(self.reply_edges(gcr_decode.AM32_NOT_RUNNING_FRAME, 1000))
        self.assertEqual(reply["frame"], gcr_decode.AM32_NOT_RUNNING_FRAME)
        self.assertTrue(reply["crc_ok"])
        self.assertTrue(reply["not_running"])

    def test_a_running_reply_survives_a_slightly_wrong_bit_time(self):
        data12 = (3 << 9) | 120                     # mantissa 120, exponent 3
        number = (data12 << 4) | gcr_decode.crc_inverted(data12)
        symbols = 0
        for shift in (12, 8, 4, 0):
            symbols = (symbols << 5) | gcr_decode.GCR_ENCODE_TABLE[(number >> shift) & 0xF]
        levels = 0
        level = 0
        for k in range(19, -1, -1):
            level ^= (symbols >> k) & 1
            levels = (levels << 1) | level
        reply = observer_edge_log.decode_reply(self.reply_edges(levels, 1000, bit=13.0))
        self.assertTrue(reply["crc_ok"])
        self.assertEqual(reply["data12"], data12)

    def test_no_edges_no_reply(self):
        self.assertIsNone(observer_edge_log.decode_reply([]))


class StrayBurstTests(unittest.TestCase):
    def frame(self, start, throttle=0):
        """Our bidirectional 16-bit frame as edges in 0.1us units: 16.7 per bit, low 6 for a 0 and 12.5 for a 1."""
        import dshot_frame_decode
        body = throttle << 5
        word = next(body | crc for crc in range(16) if dshot_frame_decode.crc_ok(body | crc, True))
        edges = []
        for k in range(16):
            bit = (word >> (15 - k)) & 1
            edges += [(start + k * 16.7, 0), (start + k * 16.7 + (12.5 if bit else 6), 1)]
        return edges

    def burst(self, start):
        """Fifteen reply-like edges on a 1.3us grid (what the ESC put on the line before one frame)."""
        steps = [26, 13, 13, 13, 26, 12, 13, 13, 26, 12, 13, 13, 13, 13]
        edges, t, level = [(start, 0)], start, 0
        for step in steps:
            t += step
            level = 1 - level
            edges.append((t, level))
        return edges

    def test_an_esc_burst_just_before_our_frame_does_not_spoil_the_frame(self):
        burst = self.burst(1000)
        records = observer_edge_log.analyse(burst + self.frame(burst[-1][0] + 22, throttle=0))
        self.assertEqual(1, len(records))
        self.assertTrue(records[0]["tx"]["crc_ok"])
        self.assertEqual(0, records[0]["tx"]["throttle"])
        self.assertEqual(len(burst), records[0]["stray_edges_before"])
        self.assertAlmostEqual(22, records[0]["stray_clearance"], places=3)

    def test_a_plain_frame_has_no_stray_edges(self):
        records = observer_edge_log.analyse(self.frame(1000) + self.frame(11450))
        self.assertEqual([0, 0], [r["stray_edges_before"] for r in records])

    def test_a_frame_with_a_wrong_crc_is_still_reported_as_it_is(self):
        edges = self.frame(1000)
        fall, rise = edges[-2][0], edges[-1][0]; edges[-1] = (fall + (6 if rise - fall > 9 else 12.5), 1)     # flip the last bit, so the CRC breaks
        records = observer_edge_log.analyse(edges)
        self.assertFalse(records[0]["tx"]["crc_ok"])
        self.assertEqual(0, records[0]["stray_edges_before"])


if __name__ == "__main__":
    unittest.main()
