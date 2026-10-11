"""
classify_reply_timeline.py - what each bidirectional ESC was doing, moment by
moment, in a pulled scenario session (see scripts/pull_captures.py). Written for
BUG-002: it tells "the ESC was silent" and "the ESC was holding the line low"
apart from "the ESC was replying but the replies were corrupted", which the CRC
counts alone cannot.

Run from the project root:
  python scripts/classify_reply_timeline.py [captures/<session>] [--window-ms N]

With no session argument, uses the most recently pulled one.

Every capture of every bidirectional motor gets one of five labels:

  spin     CRC-valid reply with a real eRPM: the motor is turning.
  stop     CRC-valid reply carrying AM32's "not running" payload (0xFFF, the
           917 eRPM sentinel): the ESC is listening and replying, the motor is
           not turning. AM32 sends it disarmed and armed alike.
  echo     Not a reply at all. When the ESC stays silent, dshot_bidir_rx_frame
           waits past the release for the next low level, which is the first bit
           of our own next frame, and reconstructs that. Those words depend only
           on the throttle we sent, so they are recognised by simulating the
           receiver (simulate_frame_receiver.run()) on our own TX waveform at the
           recorded throttle. A run of these means the ESC was not replying and
           its line sat high between our frames: before bidirectional detection,
           or inside one of AM32's interrupts-off tunes played after it enabled
           its pull-up (the arming tune).
  low      An all-zero word: the line fell low within ~2 us of our release and
           stayed low for the whole ~27 us capture. Nothing we send looks like
           that, so the ESC side was holding the line low. AM32 does this while
           it plays its 600 ms startup tune after a reset (the signal pin gets
           its pull-up only after the tune), so a ~600 ms run of these, then
           echo, then 'stop' is an ESC booting.
  garbled  Anything else that fails CRC: a reply that was there but corrupted
           or collided, or the receiver triggered on noise.

CRC is checked first, so a word is "echo" only when it is not a valid reply.
Some throttles' echoes happen to pass CRC (about 1 in 16 of random words do);
those are counted as replies, which is why a CRC-valid rate alone cannot prove
the ESC replied.

A record's word is also zero for a motor that had no new capture when the record
was written (the harness writes one whenever any motor has one). In practice
every record carries a new capture for every bidirectional motor - records are
several command ticks apart - so a zero is read as 'low'; an isolated one is
noise the window vote ignores.

The output is a per-motor timeline of windows (default 100 ms) labelled with the
class that dominates each, merged into segments, then the AM32 events found in
it (a boot, an arming tune) and one line saying which state the run ended in.
Time 0 is ARMED: meta.txt's armed_ticks_us when the session has it, else the
first record, which the harness writes right after ARMED. Sessions that logged
the arming phase (arming.bin, same record format) show it at negative times.
With --from-arm, time 0 is arm() instead (meta.txt's arm_ticks_us): ARMED's time
then depends on the arming gate (bug-reports/BUG-003), so this is the view for
timing what the ESC did after our first frame. A run that failed while arming
records arm_ticks_us too; one from before that uses its first arming capture.
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

CLASSES = ("spin", "stop", "echo", "low", "garbled")

# Receiver phases against our own frame's first edge, per simulated throttle. The
# TX and RX clocks both come from the Pico's system clock, so phase is the only
# thing that varies from one echo to the next.
ECHO_PHASES = 32

# AM32's tunes, as the receiver sees them (Src/sounds.c): the startup tune is
# 3 x 200 ms with the line held low, the arming tune 3 x 100 ms with the line
# high and no replies. The ranges allow for the window quantisation and for the
# ESC clock running a few percent off.
STARTUP_TUNE_MS = (400, 800)
ARMING_TUNE_MS = (200, 500)

# AM32 arms ~0.97 s after its first reply, measured on this bench (BUG-003's fix plan)
ARMS_AFTER_FIRST_REPLY_MS = 1100

# What time 0 is, for the readings: ARMED, or arm() with --from-arm
T0_NAME = "ARMED"


def load_meta(session_dir):
    meta = {}
    for line in (session_dir / "meta.txt").read_text().splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            meta[key.strip()] = value.strip()
    return meta


def load_records(session_dir, name="capture.bin"):
    path = session_dir / name
    if not path.exists():
        return []
    raw = path.read_bytes()
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
    if word == 0:
        return "low"
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
        candidates = (throttle,) if previous_throttle in (None, throttle) else (throttle, previous_throttle)
        delta_us = (record[0] - t0_us) & 0xFFFFFFFF
        if delta_us >= 0x80000000:
            delta_us -= 0x100000000
        out.append((delta_us / 1000, classify(word, candidates, echo_model), throttle))
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


def longest_gap(events):
    """Longest time between two consecutive records for the motor, and where it started."""
    best, at = 0.0, None
    for (a, _, _), (b, _, _) in zip(events, events[1:]):
        if b - a > best:
            best, at = b - a, a
    return best, at


def span(seg, events):
    """
    First and last capture carrying the segment's own label inside it, each
    extended over the unbroken run of that label it belongs to, so a boundary is
    placed to the capture rather than to the window.
    """
    inside = [i for i, (t, label, _) in enumerate(events) if label == seg[2] and seg[0] <= t < seg[1]]
    first, last = inside[0], inside[-1]
    while first > 0 and events[first - 1][1] == seg[2]:
        first -= 1
    while last < len(events) - 1 and events[last + 1][1] == seg[2]:
        last += 1
    return events[first][0], events[last][0]


def first_reply_after(t_ms, events):
    return next((t for t, label, _ in events if t > t_ms and label in ("stop", "spin")), None)


def reply_gap(start, end, events):
    """
    How long the ESC went without replying around the echo run start..end:
    from the last reply before it to the first after it. The echo run alone can
    come out short, as a garbled word at either edge of a tune ends it early.
    """
    before = max((t for t, label, _ in events if t <= start and label in ("stop", "spin")),
                 default=start)
    after = first_reply_after(end, events)
    return (after if after is not None else end) - before


def am32_events(segs, events):
    """
    (kind, start_ms, end_ms, extra) for each AM32 event found in the segments:
    'boot' is a startup-tune-length run of 'low' (extra = first reply after it),
    'armed' an arming-tune-length run of 'echo' between two runs of replies.
    """
    found = []
    for i, seg in enumerate(segs):
        start, end = span(seg, events)
        length = end - start
        if seg[2] == "low" and STARTUP_TUNE_MS[0] <= length <= STARTUP_TUNE_MS[1]:
            found.append(("boot", start, end, first_reply_after(end, events)))
        if (seg[2] == "echo" and 0 < i < len(segs) - 1
                and segs[i - 1][2] in ("stop", "spin") and segs[i + 1][2] in ("stop", "spin")
                and ARMING_TUNE_MS[0] <= reply_gap(start, end, events) <= ARMING_TUNE_MS[1]):
            found.append(("armed", start, end, None))
    return found


def describe(event):
    kind, start, end, first_reply = event
    if kind == "boot":
        reply = ("first reply at %.3f s" % (first_reply / 1000) if first_reply is not None
                 else "no reply after it")
        return ("ESC booted: startup tune (line held low) %.3f - %.3f s, %s" %
                (start / 1000, end / 1000, reply))
    return "ESC armed: arming tune (no replies) %.3f - %.3f s" % (start / 1000, end / 1000)


def final_stop_start(segs, events):
    """
    When the final 'stop' segment took over: the first 'stop' capture after the
    last capture carrying the label of the segment before it.
    """
    before = segs[-2][2]
    last = max(t for t, label, _ in events if label == before and t < segs[-1][1])
    return next(t for t, label, _ in events if label == "stop" and t > last)


def reading(segs, events, found):
    """One line naming the AM32 state the segment pattern matches."""
    labels = [s[2] for s in segs]
    if not labels:
        return "no captures for this motor"
    if "spin" in labels:
        first = next(s for s in segs if s[2] == "spin")
        return "spun (real eRPM from %.2f s)" % (first[0] / 1000)
    boots = [e for e in found if e[0] == "boot"]
    if boots and labels[-1] == "stop":
        boot = boots[-1]
        armed_after = [e for e in found if e[0] == "armed" and e[1] > boot[2]]
        if armed_after:
            armed_at = armed_after[0][1]
            if not any(thr for t, _, thr in events if t > armed_at):
                return ("rebooted (tune ended %.3f s), then armed at %.3f s under our zero "
                        "throttle; commanded 0 from then on, so 'not running' is expected" %
                        (boot[2] / 1000, armed_at / 1000))
            return ("rebooted (tune ended %.3f s), then armed at %.3f s, never spun: armed "
                    "but the motor did not start" % (boot[2] / 1000, armed_at / 1000))
        if boot[3] is not None and any(thr for t, _, thr in events if t >= boot[3]):
            return ("rebooted: startup tune ended %.3f s after %s, first reply at %.3f s, "
                    "no arming tune afterwards - the ESC came back up under our non-zero "
                    "throttle and never armed (path A, caused by a reset)" %
                    (boot[2] / 1000, T0_NAME, boot[3] / 1000))
        if boot[3] is not None and events[-1][0] - boot[3] < ARMS_AFTER_FIRST_REPLY_MS:
            return ("rebooted: startup tune ended %.3f s after %s, first reply at %.3f s, "
                    "throttle 0 from then to the end, which came before AM32 could arm "
                    "(~0.97 s after its first reply) - a run that timed out while arming" %
                    (boot[2] / 1000, T0_NAME, boot[3] / 1000))
    if labels == ["stop"] and not any(thr for _, _, thr in events):
        return ("'not running' from the first capture to the last, at throttle 0 throughout: "
                "the ESC was replying already at ARMED; whether it had armed is not visible "
                "(its arming tune, if any, came before the first capture)")
    if labels == ["stop"]:
        return ("'not running' from the first capture to the last: the ESC was replying "
                "already at ARMED but never started the motor - it had not armed "
                "(AM32 needs >1 s of zero throttle after it starts listening), or "
                "it rejected our throttle frames")
    if (labels[-1] == "stop" and set(labels[:-1]) <= {"echo"}
            and any(thr for t, _, thr in events if t >= final_stop_start(segs, events))):
        return ("silent until %.3f s after %s, then 'not running' to the end: the ESC was "
                "not listening when throttle started (before bidirectional detection, or in "
                "an interrupts-off tune), came up under non-zero throttle, and so never armed"
                % (final_stop_start(segs, events) / 1000, T0_NAME))
    if labels[-1] == "stop" and "garbled" in labels[:-1] and "low" not in labels:
        return ("replies corrupted until %.3f s after %s, then clean 'not running' to the "
                "end: the ESC was active and its line noisy, then quiet - matches failed "
                "start attempts ending in AM32's stuck-rotor protection" %
                (final_stop_start(segs, events) / 1000, T0_NAME))
    return "no single AM32 state matches; read the segments and events above"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("session", nargs="?", help="captures/<session> (default: most recent)")
    parser.add_argument("--window-ms", type=float, default=100.0,
                        help="window each label is judged over (default 100)")
    parser.add_argument("--from-arm", action="store_true",
                        help="time 0 = arm() (needs arm_ticks_us in meta.txt) instead of ARMED")
    args = parser.parse_args()

    session_dir = Path(args.session) if args.session else most_recent_session(ROOT / "captures")
    meta = load_meta(session_dir)
    if meta.get("record_fmt", RECORD_FMT) != RECORD_FMT:
        sys.exit("Session records are " + meta["record_fmt"] + ", not the frame receiver's " +
                 RECORD_FMT + " - this tool reads frame-receiver sessions only (2026-09-26 on).")
    dshot_speed = int(meta["dshot_speed"])
    bidir = [int(s) for s in meta.get("bidir_motor_indices", "").split(",") if s]
    arming = load_records(session_dir, "arming.bin")
    records = load_records(session_dir)
    print("Session: %s  (%s, bidirectional motors %s, %d records + %d while arming, outcome=%s)" % (
        session_dir, "DSHOT%d" % (dshot_speed // 8000), bidir, len(records), len(arming),
        meta.get("outcome")))
    if not (records or arming) or not bidir:
        return

    global T0_NAME
    if args.from_arm:
        T0_NAME = "arm()"
        if "arm_ticks_us" in meta:
            t0_us = int(meta["arm_ticks_us"])
            print("Time 0 = arm().")
        elif arming and "armed_ticks_us" not in meta:
            # A run that failed while arming, from before the harness recorded
            # arm() on failure: the first arming capture is our first frame's
            t0_us = arming[0][0]
            print("Time 0 = arm(), approximated by the first capture while arming (within ~1 ms).")
        else:
            sys.exit("--from-arm needs arm_ticks_us in meta.txt (sessions from the arming gate on)")
    else:
        if "armed_ticks_us" in meta:
            t0_us = int(meta["armed_ticks_us"])
        elif records:
            t0_us = records[0][0]
        else:
            t0_us = arming[-1][0]
        if records or "armed_ticks_us" in meta:
            print("Time 0 = ARMED; negative times are the arming window.")
        else:
            T0_NAME = "the last arming capture"
            print("Time 0 = the last capture while arming, as ARMED never came; "
                  "--from-arm counts from arm() instead.")
    if "arm_ticks_us" in meta and "armed_ticks_us" in meta:
        print("ARMED %.3f s after arm()." % (
            ((int(meta["armed_ticks_us"]) - int(meta["arm_ticks_us"])) & 0x3FFFFFFF) / 1e6))
    elif arming and "armed_ticks_us" not in meta:
        print("ARMED never reached: the run failed while arming.")

    echo_model = EchoModel(dshot_speed)
    for index in bidir:
        events = timeline(arming + records, index, echo_model, t0_us)
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
        gap, at = longest_gap(events)
        if at is not None:
            print("  longest gap between records: %.1f ms, from %.3f s" % (gap, at / 1000))
        found = am32_events(segs, events)
        for event in found:
            print("  event: " + describe(event))
        print("  reading: " + reading(segs, events, found))


if __name__ == "__main__":
    main()
