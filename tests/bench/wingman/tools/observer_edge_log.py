"""
observer_edge_log.py - read the stream the wingman sends (tests/bench/wingman/board/observer.py) and turn it into edges.

Pure Python, no hardware: tests/pc/test_observer_edge_log.py exercises it. The meaning of an entry and the time
arithmetic are documented at the top of observer.py; the constants here must agree with it.
"""

import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "driver"))
sys.path.insert(0, str(ROOT / "tests" / "bench"))
import gcr_decode
import dshot_frame_decode

OBSERVATION_FILE = "signal_observation.bin"      # the recording the observer makes of a session: the one place that names it
MAGIC = 0xD5
MARKER = 0x7FFF
INSTR_NS = 50                       # state machine at 20 MHz
HIGH_RUN_OVERHEAD = 4               # instructions per high run besides the counting loop
LOW_RUN_OVERHEAD = 6
HEADER = 5


MARKER_EXTRA = 6                    # instructions a marker takes beyond its 2 * MARKER counting ones (65540 in all)


class StreamError(Exception):
    pass


class Payload(bytearray):
    """The bytes of one line, tagged with the stream version: marker_v2 = markers carry the inverted level bit."""
    marker_v2 = False


class StreamReader:
    """Takes the observer's bytes as they arrive (feed) and keeps the start text, the per-channel payload bytes
    and the end text. Anything before the first packet (the REPL's own 'OK') is skipped. feed() raises
    StreamError on a damaged stream; done is True once the end packet is in."""

    def __init__(self):
        self.pending = bytearray()
        self.started = False
        self.start = None
        self.end = None
        self.channels = {}
        self.marker_v2 = False
        self.done = False

    def feed(self, data):
        if self.done:
            return
        self.pending.extend(data)
        if not self.started:
            pos = self.pending.find(bytes((MAGIC, ord("S"))))
            if pos < 0:
                del self.pending[:max(0, len(self.pending) - 1)]
                return
            del self.pending[:pos]
            self.started = True
        pos = 0
        buf = self.pending
        while not self.done and pos + HEADER <= len(buf):
            if buf[pos] != MAGIC:
                raise StreamError("lost packet framing %d bytes into the stream" % pos)
            kind = chr(buf[pos + 1])
            channel = buf[pos + 2]
            length = buf[pos + 3] | (buf[pos + 4] << 8)
            if pos + HEADER + length > len(buf):
                break
            body = bytes(buf[pos + HEADER:pos + HEADER + length])
            pos += HEADER + length
            if kind == "S":
                self.start = body.decode()
                self.marker_v2 = "marker=v2" in self.start
            elif kind == "E":
                self.end = body.decode()
                self.done = True
            elif kind == "D":
                if channel not in self.channels:
                    self.channels[channel] = Payload()
                    self.channels[channel].marker_v2 = self.marker_v2
                self.channels[channel].extend(body)
            else:
                raise StreamError("unknown packet type %r" % kind)
        del buf[:pos]


def parse_stream(data):
    """Whole-stream form of StreamReader: (start text, {channel: payload}, end text). Raises StreamError when
    the stream is damaged or has no end packet."""
    reader = StreamReader()
    reader.feed(data)
    if not reader.done:
        raise StreamError("the stream has no %s packet" % ("start" if reader.start is None else "end"))
    for problem in reconcile(reader.channels, parse_end(reader.end)):
        sys.stderr.write("WARNING: stream loss, do not trust this recording: %s\n" % problem)
    return reader.start, reader.channels, reader.end


def parse_end(text):
    """'reason=ended ch0 bytes=12 overruns=0 rx_stall=0 ... valid=1' -> dict."""
    info = {"channels": {}}
    channel = None
    for token in text.split():
        if token.startswith("ch") and "=" not in token:
            channel = int(token[2:])
            info["channels"][channel] = {}
            continue
        key, _, value = token.partition("=")
        if channel is None or key == "valid":
            info[key] = value
        else:
            info["channels"][channel][key] = int(value)
    info["valid"] = info.get("valid") == "1"
    return info


def reconcile(channels, info):
    """Problems found by comparing the bytes the PC received per line with the 'bytes=' the observer's end packet
    says it sent (a mismatch means the stream lost or gained bytes on the way, which the observer cannot see
    itself). Empty list = they agree."""
    problems = []
    for channel, fields in info["channels"].items():
        got = len(channels.get(channel, b""))
        sent = fields.get("bytes", 0)
        if got != sent:
            problems.append("line %d: the observer sent %d bytes, the PC received %d" % (channel, sent, got))
    return problems


def entries(payload):
    """The 16-bit entries of a payload, in time order: (level of the run that ended, 15-bit field)."""
    out = []
    for (word,) in struct.iter_unpack("<I", bytes(payload[:len(payload) // 4 * 4])):
        for entry in (word >> 16, word & 0xFFFF):
            out.append((entry >> 15, entry & MARKER))
    return out


def edges(payload, unit_ns=100, invert=False):
    """Edges (time in units of unit_ns since the recording began, new level) of one channel. A run that ended
    while high means the line went low there. Time is exact to the instruction; the edge itself is only known
    to within the 2-instruction counting loop (100ns; an edge at a 3.28ms marker boundary can read up to 0.2us late, measured 2026-10-29). invert=True flips every level: unidirectional DShot
    idles low and carries a bit in a high pulse, which then reads exactly like the bidirectional shape."""
    t = 0
    out = []
    v2 = getattr(payload, "marker_v2", False)
    current = None                                # level of the run being counted (v2 streams only)
    for level, field in entries(payload):
        if field == MARKER:
            if not v2 or current is None or level != current:
                if v2 and current is None:
                    current = 1 - level
                t += 2 * MARKER + MARKER_EXTRA    # quiet for 32767 ticks, no edge
                continue
            # v2: the level bit of the run itself with 0 ticks counted is an edge right after a marker
        current = 1 - level
        t += 2 * (MARKER - field) + (HIGH_RUN_OVERHEAD if level else LOW_RUN_OVERHEAD)
        out.append((t * INSTR_NS / unit_ns, level if invert else 1 - level))
    return out


GROUP_GAP_UNITS = 2000              # 200us without an edge ends one frame-and-reply group (frames are ~1ms apart)
TX_EDGES = 32                       # our frame: 16 bits, one fall and one rise each
REPLY_BIT_UNITS = 13.333            # 750 kbit/s, the reply rate for DSHOT600 (0.1us units)
REPLY_BITS = gcr_decode.FRAME_LENGTH_BITS


def groups(edge_list, gap_units=GROUP_GAP_UNITS):
    """Split edges into the groups the line shows once per frame: our frame, then the ESC's reply if any."""
    out = []
    current = []
    for edge in edge_list:
        if current and edge[0] - current[-1][0] >= gap_units:
            out.append(current)
            current = []
        current.append(edge)
    if current:
        out.append(current)
    return out


def decode_reply(reply_edges, bit_units=REPLY_BIT_UNITS):
    """The ESC's reply from its edges: the 21 line levels, sampled in the middle of each bit from the first edge
    on, as the integer gcr_decode.analyze_frame() takes (first bit on top). The bit time is refined from the
    edges themselves (every interval is a whole number of bits). Returns a dict, or None without edges."""
    if not reply_edges:
        return None
    times = [t for t, _ in reply_edges]
    gaps = [b - a for a, b in zip(times, times[1:])]
    bits = [max(1, round(g / bit_units)) for g in gaps]
    measured = sum(gaps) / sum(bits) if gaps else bit_units
    if not 0.8 * bit_units < measured < 1.2 * bit_units:
        measured = bit_units
    start = times[0]
    frame = 0
    for k in range(REPLY_BITS):
        at = start + (k + 0.5) * measured
        level = 1
        for t, new_level in reply_edges:
            if t <= at:
                level = new_level
            else:
                break
        frame = (frame << 1) | level
    result = gcr_decode.analyze_frame(frame)
    result["frame"] = frame
    result["edges"] = len(reply_edges)
    result["bit_units"] = measured
    result["start"] = start
    return result


PITCH_SPREAD = 1.3                  # a real frame's longest bit period is within this factor of its shortest


def frame_offset(group, bidirectional):
    """Where our frame starts in a group. Normally at its first edge. If the first 32 edges are not a clean frame but a
    later 32 are (CRC right, even bit pitch), edges the ESC put on the line before our frame came first (an ESC burst
    ending a few us before our frame); that offset is returned. Anything else: 0, the group is judged as it stands."""
    first = dshot_frame_decode.decode_frame(group[:TX_EDGES], bidirectional) if len(group) >= TX_EDGES else None
    if first is None or (first["ok"] and first["crc_ok"] and max(first["periods"]) <= PITCH_SPREAD * min(first["periods"])):
        return 0
    for k in range(1, len(group) - TX_EDGES + 1):
        tx = dshot_frame_decode.decode_frame(group[k:k + TX_EDGES], bidirectional)
        if tx is not None and tx["ok"] and tx["crc_ok"] and max(tx["periods"]) <= PITCH_SPREAD * min(tx["periods"]):
            return k
    return 0


def analyse(edge_list, unidirectional=False):
    """One record per group: our frame (dshot_frame_decode's dict, or None when the group does not start with a
    complete 16-bit frame) and the reply that follows it (decode_reply's dict, or None when the line stayed
    quiet). "stray_edges_before" counts edges that preceded our frame in its group (see frame_offset); "stray_clearance" is the time from the last of them to our frame's first edge (0.1us units). unidirectional: the edges were read with invert=True, the CRC is not inverted and no reply exists."""
    all_groups = groups(edge_list)
    offsets = [frame_offset(group, not unidirectional) for group in all_groups]
    txs = []
    for group, k in zip(all_groups, offsets):
        tx = dshot_frame_decode.decode_frame(group[k:k + TX_EDGES], not unidirectional) if len(group) >= k + TX_EDGES else None
        txs.append(tx if tx is not None and tx["ok"] else None)
    # the reply runs at 5/4 of the command's bit rate, so its bit time follows the speed of the frames (1.33 us at DSHOT600)
    periods = sorted(p for tx in txs[:200] if tx is not None for p in tx["periods"])
    reply_bit = REPLY_BIT_UNITS * (2 if periods and periods[len(periods) // 2] > 25 else 1)
    records = []
    for group, tx, k in zip(all_groups, txs, offsets):
        reply = decode_reply(group[k + TX_EDGES:], reply_bit) if tx is not None and not unidirectional else None
        if reply is not None:
            reply["delay"] = reply["start"] - group[k + TX_EDGES - 1][0]
        records.append({"start": group[k][0], "tx": tx, "reply": reply, "edges": len(group) - k,
                        "tx_edges": group[k:k + TX_EDGES] if tx is not None else None, "stray_edges_before": k,
                        "stray_clearance": group[k][0] - group[k - 1][0] if k else None})
    return records
