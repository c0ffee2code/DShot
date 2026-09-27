"""
classify_reply_timeline.py - what each bidirectional ESC was doing, moment by
moment, in a pulled scenario session (see scripts/pull_captures.py). Written for
BUG-002: it tells "the ESC was silent" apart from "the ESC was replying but the
replies were corrupted", which the CRC counts alone cannot.

Run from the project root:
  python scripts/classify_reply_timeline.py [captures/<session>] [--window-ms N]

With no session argument, uses the most recently pulled one.

Every capture of every bidirectional motor gets one of four labels:

  spin     CRC-valid reply with a real eRPM: the motor is turning.
  stop     CRC-valid reply carrying AM32's "not running" payload (0xFFF, the
           917 eRPM sentinel): the ESC is listening and replying, the motor is
           not turning. AM32 sends it disarmed and armed alike.
  echo     Not a reply at all. When the ESC stays silent, dshot_bidir_rx_frame
           waits past the release for the next low level, which is the first bit
           of our own next frame, and reconstructs that. Those words depend only
           on the throttle we sent, so they are recognised by simulating the
           receiver (simulate_frame_receiver.run()) on our own TX waveform at the
           recorded throttle. A run of these means the ESC was not replying:
           rebooting, still before bidirectional detection, or inside one of
           AM32's interrupts-off tunes.
  garbled  Anything else that fails CRC: a reply that was there but corrupted
           or collided, or the receiver triggered on noise.

CRC is checked first, so a word is "echo" only when it is not a valid reply.
Some throttles' echoes happen to pass CRC (about 1 in 16 of random words do);
those are counted as replies, which is why a CRC-valid rate alone cannot prove
the ESC replied.

The output is a per-motor timeline of windows (default 100 ms) labelled with the
class that dominates each, merged into segments, followed by one line saying
which AM32 state the pattern matches. Time 0 is the first record, which the
harness writes right after the group reports ARMED.
"""

import argparse
import struct
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "driver"))
sys.path.insert(0, str(ROOT / "scripts"))

import gcr_decode
from dshot_profiles import frame_rx_speed
from simulate_frame_receiver import run

# Must match tests/harness/bidir_capture_sink.py's BidirCaptureSink.RECORD_FMT
RECORD_FMT = "<I4H4I"
RECORD_SIZE = struct.calcsize(RECORD_FMT)

CLASSES = ("spin", "stop", "echo", "garbled")

# Receiver phases against our own frame's first edge, per simulated throttle. The
# TX and RX clocks both come from the Pico's system clock, so phase is the only
# thing that varies from one echo to the next.
ECHO_PHASES = 32


def load_meta(session_dir):
    meta = {}
    for line in (session_dir / "meta.txt").read_text().splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            meta[key.strip()] = value.strip()
    return meta


def load_records(session_dir):
    raw = (session_dir / "capture.bin").read_bytes()
    return [struct.unpack_from(RECORD_FMT, raw, i * RECORD_SIZE)
            for i in range(len(raw) // RECORD_SIZE)]


def most_recent_session(captures_dir):
    sessions = sorted(p for p in captures_dir.iterdir() if p.is_dir())
    if not sessions:
        sys.exit("No sessions under " + str(captures_dir) + " - run scripts/pull_captures.py first.")
    return sessions[-1]


def bidir_packet(throttle):
    """The 16-bit frame BidirectionalDShot.send_throttle_command() sends."""
    value = throttle << 1
    crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0xF
    return (value << 4) | crc


def tx_level(throttle, dshot_speed):
    """
    dshot_bidir_tx's line level at time t (ns), t = 0 at the frame's first
    falling edge. Each bit is 8 TX cycles: 2 high, 3 low, then 3 low for a 1 or
    3 high for a 0. The line is high before and after the frame.
    """
    packet = bidir_packet(throttle)
    cycle_ns = 1e9 / dshot_speed
    bits = [(packet >> (15 - i)) & 1 for i in range(16)]

    def level(t):
        if t < 0:
            return 1
        bit = int(t // (8 * cycle_ns))
        if bit >= 16:
            return 1
        low_cycles = 6 if bits[bit] else 3
        return 0 if t - bit * 8 * cycle_ns < low_cycles * cycle_ns else 1

    return level


class EchoModel:
    """Words dshot_bidir_rx_frame pushes when it captures our own next frame."""

    def __init__(self, dshot_speed):
        self.dshot_speed = dshot_speed
        self.ns_per_rx_cycle = 1e9 / frame_rx_speed(dshot_speed)
        self.cache = {}

    def words(self, throttle):
        found = self.cache.get(throttle)
        if found is None:
            level = tx_level(throttle, self.dshot_speed)
            step = self.ns_per_rx_cycle / ECHO_PHASES
            found = {run(level, self.ns_per_rx_cycle, k * step) for k in range(ECHO_PHASES)}
            found.discard(None)
            self.cache[throttle] = found
        return found


def classify(word, throttles, echo_model):
    """One capture's label; `throttles` are the throttles that may have been on the wire."""
    result = gcr_decode.analyze_frame(word)
    if result["crc_ok"]:
        return "stop" if result["not_running"] else "spin"
    for throttle in throttles:
        if word in echo_model.words(throttle):
            return "echo"
    return "garbled"


def timeline(records, index, echo_model, t0_us):
    """(time_ms, label, throttle) for every capture of one motor."""
    out = []
    previous_throttle = None
    for record in records:
        throttle = record[1 + index]
        word = record[5 + index]
        if word:
            candidates = (throttle,) if previous_throttle in (None, throttle) else (throttle, previous_throttle)
            time_ms = ((record[0] - t0_us) & 0xFFFFFFFF) / 1000
            out.append((time_ms, classify(word, candidates, echo_model), throttle))
        previous_throttle = throttle
    return out


def segments(events, window_ms):
    """
    Label each window by its dominant class and merge equal neighbours.
    Returns [start_ms, end_ms, label, Counter of that segment's captures].
    """
    merged = []
    if not events:
        return merged
    window = {}
    for time_ms, label, _ in events:
        window.setdefault(int(time_ms // window_ms), Counter())[label] += 1
    for key in sorted(window):
        counts = window[key]
        label = counts.most_common(1)[0][0]
        start, end = key * window_ms, (key + 1) * window_ms
        if merged and merged[-1][2] == label and merged[-1][1] == start:
            merged[-1][1] = end
            merged[-1][3].update(counts)
        else:
            merged.append([start, end, label, Counter(counts)])
    return merged


def longest_silence(events):
    """Longest time between two consecutive captures of the motor, and where it started."""
    best, at = 0.0, None
    for (a, _, _), (b, _, _) in zip(events, events[1:]):
        if b - a > best:
            best, at = b - a, a
    return best, at


def final_stop_start(segs, events):
    """
    When the final 'stop' segment took over: the first 'stop' capture after the
    last capture carrying the label of the segment before it.
    """
    before = segs[-2][2]
    last = max(t for t, label, _ in events if label == before and t < segs[-1][1])
    return next(t for t, label, _ in events if label == "stop" and t > last)


def reading(segs, events):
    """One line naming the AM32 state the segment pattern matches."""
    labels = [s[2] for s in segs]
    if not labels:
        return "no captures for this motor"
    if "spin" in labels:
        first = next(s for s in segs if s[2] == "spin")
        return "spun (real eRPM from %.2f s)" % (first[0] / 1000)
    if labels == ["stop"]:
        return ("'not running' from the first capture to the last: the ESC was replying "
                "already at ARMED but never started the motor - it had not armed "
                "(AM32 needs >1 s of zero throttle after it starts listening), or "
                "it rejected our throttle frames")
    if labels[-1] == "stop" and set(labels[:-1]) <= {"echo"}:
        return ("silent until %.3f s after ARMED, then 'not running' to the end: the ESC was "
                "not listening when throttle started (rebooting, before bidirectional "
                "detection, or in an interrupts-off tune), came up under non-zero "
                "throttle, and so never armed" % (final_stop_start(segs, events) / 1000))
    if labels[-1] == "stop" and "garbled" in labels[:-1]:
        return ("replies corrupted until %.3f s after ARMED, then clean 'not running' to the "
                "end: the ESC was active and its line noisy, then quiet - matches failed "
                "start attempts ending in AM32's stuck-rotor protection" % (final_stop_start(segs, events) / 1000))
    return "no single AM32 state matches; read the segments above"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("session", nargs="?", help="captures/<session> (default: most recent)")
    parser.add_argument("--window-ms", type=float, default=100.0,
                        help="window each label is judged over (default 100)")
    args = parser.parse_args()

    session_dir = Path(args.session) if args.session else most_recent_session(ROOT / "captures")
    meta = load_meta(session_dir)
    if meta.get("record_fmt", RECORD_FMT) != RECORD_FMT:
        sys.exit("Session records are " + meta["record_fmt"] + ", not the frame receiver's " +
                 RECORD_FMT + " - this tool reads frame-receiver sessions only (2026-09-26 on).")
    dshot_speed = int(meta["dshot_speed"])
    bidir = [int(s) for s in meta.get("bidir_motor_indices", "").split(",") if s]
    records = load_records(session_dir)
    print("Session: %s  (%s, bidirectional motors %s, %d records, outcome=%s)" % (
        session_dir, "DSHOT%d" % (dshot_speed // 8000), bidir, len(records), meta.get("outcome")))
    if not records or not bidir:
        return

    echo_model = EchoModel(dshot_speed)
    t0_us = records[0][0]
    for index in bidir:
        events = timeline(records, index, echo_model, t0_us)
        totals = Counter(label for _, label, _ in events)
        print()
        print("Motor %d: %d captures - %s" % (index, len(events), ", ".join(
            "%s %d" % (c, totals[c]) for c in CLASSES if totals[c])))
        segs = segments(events, args.window_ms)
        for start, end, label, counts in segs:
            share = 100.0 * counts[label] / sum(counts.values())
            throttles = sorted({t for time_ms, _, t in events if start <= time_ms < end})
            thr = str(throttles[0]) if len(throttles) == 1 else "%d-%d" % (throttles[0], throttles[-1])
            print("  %8.2f - %8.2f s  %-7s %5.1f%% of %5d captures  throttle %s" % (
                start / 1000, end / 1000, label, share, sum(counts.values()), thr))
        silence, at = longest_silence(events)
        if at is not None:
            print("  longest stretch with no capture: %.1f ms, from %.3f s" % (silence, at / 1000))
        print("  reading: " + reading(segs, events))


if __name__ == "__main__":
    main()
