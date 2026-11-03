"""
session_timeline.py - one picture of a recorded session: the wingman's wire record (our throttle frames and the ESC's
replies, every one) with the main Pico's own record laid on the same time axis.

  python scripts/session_timeline.py captures/<session>                      # writes <session>/timeline.png
  python scripts/session_timeline.py captures/<session> --from-ms 5000 --to-ms 5400   # zoom into a stretch
  python scripts/session_timeline.py captures/<session> --show               # interactive window: zoom and pan with the toolbar

Time axis: milliseconds since the start line (GPIO 15) rose. The spy's clock starts when it sees that rise; the main
Pico's ticks are converted with `line_high_ticks_us`, the tick it stored just before raising the line. The two clocks are
therefore aligned to within the spy's gate latency (not measured); the "lag" figures below show how well that holds.

What the main Pico's record holds (tests/bench/session_recorder/bidir_capture_sink.py): one record each time Core 0 decodes
a capture, stamped with the main Pico's tick. Core 0 decodes every `sample_every`-th capture it sees per motor, and it sees at
most one per poll pass (`poll_ms`), so the record is sparse by design (about one stamp a second per motor at the default
settings; the main Pico does the bare minimum of I/O while it controls motors). The wire record is the dense one.

Alignment: the spy's state machines start on the same clock cycle as the start line rises; the main Pico stored its tick
just before it raised the line, so the two clocks agree to within the few microseconds between those two points.

Where a stamp's reply is on the wire: every reply the receiver publishes steps a 9-bit token (1..511, driver/capture_mailbox.py),
counted from the first reply after ARMED. So a sample's token names its reply on the wire up to a lap of 511 replies (about
half a second) and up to one unknown offset: the number of replies the ESC sent before the first one that was published. That
offset is searched in a window around the replies counted before ARMED, scoring each candidate by how many samples carry the
same 21-bit frame as the wire reply it predicts; ties (a steady throttle repeats frames) are reported with the lag range they
imply. A sample whose frame differs from the wire reply at its predicted place is reported, never silently dropped.

Panels:

  1. Frame spacing on the wire, every frame a dot: the pacing of our update loop as the ESC sees it.
  2. Per line: the ESC's replies (one tick each: real eRPM, AM32's "not running", damaged), the main Pico's decode stamps
     (diamonds), and where the main Pico noted that Core 1 fell behind its schedule (dashed vertical lines, all panels).
  3. Lag: main Pico's decode stamp minus the end of the same reply on the wire.
  4. Time between two consecutive decode stamps of one motor.
  5. Spin-up: the ESC's real eRPM per line as a percentage of that motor's own settled speed, so all lines share one scale;
     small tick = first nonzero throttle frame, dot = where it reached 80% (the start KPI's spin-up, measured by
     spy_start_summary.spin_up), dashed line = the 80% level. A line with no dot never settled or never got there.
"""

import argparse
import bisect
import json
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bench_paths  # noqa: F401  (the wingman tools and driver/ are importable from here on)
import gcr_decode
import session_files
import spy_analyze
import spy_edge_log
import spy_start_summary
import spy_stats

UNIT_MS = spy_analyze.UNIT_US / 1000       # one edge unit of the wire record, in ms
TICKS_MASK = 0x3FFFFFFF                     # MicroPython ticks_us() wraps at 2**30
EXPECTED_RECORD_FMT = "<IB4H4I"             # bidir_capture_sink.BidirCaptureSink.RECORD_FMT
FLAG_FAILED = 0x80                          # bidir_capture_sink.FLAG_FAILED
FLAG_MOTOR_MASK = 0x03                      # bidir_capture_sink.FLAG_MOTOR_MASK
FRAME_MASK = (1 << gcr_decode.FRAME_LENGTH_BITS) - 1   # the capture word: the 21-bit reply frame, publish token above
CLOCK_SLACK_MS = 2.0                        # a stamp may sit this far before the end of its reply (clock alignment)
TOKEN_MAX = 511                             # driver/capture_mailbox.py: tokens run 1..TOKEN_MAX
ARMED_WINDOW = 30                           # replies either side of the count at ARMED searched for the first published one


class SessionError(Exception):
    pass


def ticks_diff_us(a, b):
    """a - b for MicroPython ticks_us values (wrap at 2**30), signed."""
    diff = (a - b) & TICKS_MASK
    return diff - (TICKS_MASK + 1) if diff >= (TICKS_MASK + 1) // 2 else diff


def read_samples(path, device, line_high_ticks_us):
    """capture.bin -> [{"t_ms", "motor", "failed", "capture"}], t_ms counted from the start line. Only the current record
    format is read; a session whose meta names another format raises."""
    fmt = device.get("record_fmt")
    if fmt != EXPECTED_RECORD_FMT:
        raise SessionError("capture.bin record format is %r, this tool reads %r" % (fmt, EXPECTED_RECORD_FMT))
    data = Path(path).read_bytes()
    size = struct.calcsize(fmt)
    out = []
    for offset in range(0, len(data) - size + 1, size):
        ticks, flags, _t0, _t1, _t2, _t3, c0, c1, c2, c3 = struct.unpack_from(fmt, data, offset)
        motor = flags & FLAG_MOTOR_MASK
        out.append({"t_ms": ticks_diff_us(ticks, line_high_ticks_us) / 1000.0, "motor": motor,
                    "failed": bool(flags & FLAG_FAILED), "capture": (c0, c1, c2, c3)[motor]})
    return out


def wire_replies(records):
    """Every reply of one line from spy_edge_log.analyse, damaged ones too (the receiver publishes every word it takes):
    [{"end_ms", "status", "erpm", "frame"}] in time order. end_ms is where the last of the reply's 21 bits ends. status:
    'run' (CRC right, real eRPM), 'idle' (AM32's not-running frame), 'bad'."""
    out = []
    for record in records:
        reply = record["reply"]
        if record["tx"] is None or reply is None:
            continue
        end = (reply["start"] + spy_edge_log.REPLY_BITS * reply["bit_units"]) * UNIT_MS
        status = spy_stats.reply_status(reply)
        out.append({"end_ms": end, "status": status, "erpm": reply["erpm"] if status == "run" else None, "frame": reply["frame"]})
    return out


def place_at(ends, offset, sample):
    """The wire index the sample's token names for this offset: the latest reply of its lap that ended no later than the stamp
    (plus CLOCK_SLACK_MS). None when there is none."""
    top = bisect.bisect_right(ends, sample["t_ms"] + CLOCK_SLACK_MS) - 1
    if top < 0:
        return None
    wanted = (offset + (sample["capture"] >> gcr_decode.FRAME_LENGTH_BITS) - 1) % TOKEN_MAX
    index = top - ((top - wanted) % TOKEN_MAX)
    return index if index >= 0 else None


def place_samples(samples, replies, armed_ms=None):
    """Find each sample's reply on the wire by its token (see the top of this file). Returns
    {"pairs": [(sample, lag_ms or None)], "agree", "disagree", "unplaced", "tied": [offsets that score equally],
     "tied_lag_ms": (lowest, highest median lag over the tied offsets) or None, "offset"}.
    Only the best-scoring offset's lags are in "pairs"; among equal scores the one closest to the count of replies before ARMED."""
    ends = [r["end_ms"] for r in replies]
    frames = [r["frame"] for r in replies]
    prior = bisect.bisect_right(ends, armed_ms) if armed_ms is not None else None
    if prior is None:
        candidates = list(range(TOKEN_MAX))
    else:
        candidates = sorted({(prior + d) % TOKEN_MAX for d in range(-ARMED_WINDOW, ARMED_WINDOW + 1)})

    def agreeing_lags(offset):
        lags = []
        for sample in samples:
            index = place_at(ends, offset, sample)
            if index is not None and frames[index] == sample["capture"] & FRAME_MASK:
                lags.append(sample["t_ms"] - ends[index])
        return lags

    def distance(offset):
        return 0 if prior is None else min((offset - prior) % TOKEN_MAX, (prior - offset) % TOKEN_MAX)

    scores = {offset: len(agreeing_lags(offset)) for offset in candidates}
    best = max(scores.values(), default=0)
    tied = sorted((o for o in candidates if scores[o] == best), key=distance)
    result = {"pairs": [], "agree": 0, "disagree": 0, "unplaced": 0, "tied": tied, "tied_lag_ms": None, "offset": None}
    if not tied or not samples:
        return result
    result["offset"] = tied[0]
    medians = [spy_stats.percentile(sorted(lags), 0.5) for lags in (agreeing_lags(o) for o in tied) if lags]
    if medians:
        result["tied_lag_ms"] = (min(medians), max(medians))
    for sample in samples:
        index = place_at(ends, tied[0], sample)
        if index is None:
            result["unplaced"] += 1
            result["pairs"].append((sample, None))
        elif frames[index] == sample["capture"] & FRAME_MASK:
            result["agree"] += 1
            result["pairs"].append((sample, sample["t_ms"] - ends[index]))
        else:
            result["disagree"] += 1
            result["pairs"].append((sample, None))
    return result


def sample_gaps(samples):
    """[(t_ms, gap_ms)] between consecutive stamps of the same motor."""
    last = {}
    out = []
    for sample in samples:
        if sample["motor"] in last:
            out.append((sample["t_ms"], sample["t_ms"] - last[sample["motor"]]))
        last[sample["motor"]] = sample["t_ms"]
    return out


def behind_events_ms(device, line_high_ticks_us):
    """Times (ms from the start line) at which the main Pico noted Core 1 behind its schedule. The device stores them in ms
    since arm(), so they have the resolution of a millisecond."""
    text = device.get("schedule_behind_at_ms_since_arm", "")
    if not text or "arm_ticks_us" not in device:
        return []
    arm_ms = ticks_diff_us(int(device["arm_ticks_us"]), line_high_ticks_us) / 1000.0
    return [arm_ms + int(v) for v in text.split(",") if v]


def percentiles(values):
    if not values:
        return None
    ordered = sorted(values)
    return {"n": len(ordered), "min": ordered[0], "median": spy_stats.percentile(ordered, 0.5),
            "p99": spy_stats.percentile(ordered, 0.99), "max": ordered[-1]}


def load_session(session_dir):
    session_dir = Path(session_dir)
    manifest = session_files.read_manifest(session_dir)
    if not manifest or not isinstance(manifest.get("device"), dict):
        raise SessionError("%s has no session.json with the Pico's record (a session pulled from the SD card)" % session_dir)
    device = manifest["device"]
    if "line_high_ticks_us" not in device:
        raise SessionError("the Pico's record has no line_high_ticks_us: the run stopped before it raised the start line")
    spy_path = session_dir / "spy.bin"
    if not spy_path.is_file():
        raise SessionError("%s has no spy.bin (the spy was not recording)" % session_dir)
    scenario_path = session_dir / "scenario.json"
    scenario = json.loads(scenario_path.read_text(encoding="utf-8")) if scenario_path.is_file() else {}
    start, channels, end = spy_edge_log.parse_stream(spy_path.read_bytes())
    pins = {}
    for token in start.split():
        if token.startswith("pins="):
            pins = dict(enumerate(int(p) for p in token[5:].split(",")))
    line_of_motor = {}
    for index, motor in enumerate(scenario.get("motors", [])):
        for line, pin in pins.items():
            if pin == motor.get("pin"):
                line_of_motor[index] = line
    return {"device": device, "scenario": scenario, "start": start, "end": end, "channels": channels, "pins": pins,
            "line_of_motor": line_of_motor, "line_high": int(device["line_high_ticks_us"]),
            "bidir_motors": [int(x) for x in device.get("bidir_motor_indices", "").split(",") if x]}


def build(session):
    """Everything the picture and the text need, as plain data."""
    device = session["device"]
    line_high = session["line_high"]
    result = {"lines": {}, "behind": behind_events_ms(device, line_high), "session": session}
    for line, payload in sorted(session["channels"].items()):
        records = spy_edge_log.analyse(spy_edge_log.edges(payload))
        rows = spy_stats.frame_table(records, 0)
        result["lines"][line] = {"rows": rows, "replies": wire_replies(records), "samples": [], "lags": [], "gaps": [], "placement": None}
    return result


def attach_samples(result, samples):
    session = result["session"]
    marks = arm_marks_ms(session["device"], session["line_high"])
    motor_of_line = {line: motor for motor, line in session["line_of_motor"].items()}
    for line, data in result["lines"].items():
        motor = motor_of_line.get(line)
        mine = [s for s in samples if s["motor"] == motor]
        data["samples"] = mine
        data["placement"] = place_samples(mine, data["replies"], marks.get("ARMED"))
        data["lags"] = data["placement"]["pairs"]
        data["gaps"] = sample_gaps(mine)


def arm_marks_ms(device, line_high):
    marks = {}
    for key, label in (("arm_ticks_us", "arm()"), ("armed_ticks_us", "ARMED")):
        if key in device:
            marks[label] = ticks_diff_us(int(device[key]), line_high) / 1000.0
    return marks


def print_report(result, tolerance_us):
    session = result["session"]
    device = session["device"]
    marks = arm_marks_ms(device, session["line_high"])
    print("start: %s" % session["start"])
    print("end:   %s" % session["end"])
    for label, value in marks.items():
        print("main Pico %s at %.1f ms after the start line" % (label, value))
    print("sample_every=%s update_interval_us=%s" % (device.get("sample_every"), device.get("update_interval_us")))
    behind = result["behind"]
    print("Core 1 behind-schedule events noted by the main Pico: %d%s" % (
        len(behind), "" if not behind else " at ms: " + ", ".join("%.0f" % b for b in behind[:12]) + (" ..." if len(behind) > 12 else "")))
    for line, data in sorted(result["lines"].items()):
        pin = session["pins"].get(line)
        rows = data["rows"]
        print("\nline %d (GPIO %s): %d frames on the wire, %d replies (%d with a real eRPM)" % (
            line, pin, len(rows), len(data["replies"]), sum(1 for r in data["replies"] if r["status"] == "run")))
        if rows and "arm()" in marks:
            print("   first frame on the wire %.1f ms after the start line; main Pico's arm() was at %.1f ms" % (
                rows[0]["t_ms"], marks["arm()"]))
        hiccups = spy_stats.hiccups(rows, tolerance_us)
        near = sum(1 for t, _ in hiccups if any(0 <= t - b <= 3 for b in behind))
        print("   frames whose spacing is more than %.0f us from the median: %d (%d of them within 3 ms after a behind-schedule event)" % (
            tolerance_us, len(hiccups), near))
        stats = percentiles([lag for _, lag in data["lags"] if lag is not None])
        placed = data["placement"]
        print("   main Pico stamps: %d" % len(data["samples"]))
        if placed and data["samples"]:
            print("   found on the wire by token: %d (frame equals the sample's); frame differs: %d; no reply of that lap yet: %d" % (
                placed["agree"], placed["disagree"], placed["unplaced"]))
            if len(placed["tied"]) > 1 and placed["tied_lag_ms"]:
                print("   NOTE: %d offsets of the first published reply fit equally well (frames repeat); the median lag could be anywhere "
                      "from %.1f to %.1f ms" % (len(placed["tied"]), placed["tied_lag_ms"][0], placed["tied_lag_ms"][1]))
            if placed["disagree"] and placed["disagree"] * 2 >= len(data["samples"]):
                print("   NOTE: most samples carry a frame other than the wire's at their predicted place: the receiver and the spy "
                      "disagree, or the reply counts drifted apart; the lags below are not trustworthy")
        if stats:
            print("   lag, decode stamp minus end of the reply on the wire (ms): min %.1f  median %.1f  99%% %.1f  max %.1f" % (
                stats["min"], stats["median"], stats["p99"], stats["max"]))
            if stats["min"] < -CLOCK_SLACK_MS / 2:
                print("   NOTE: a negative lag means the two clocks are not aligned as assumed")
        gaps = percentiles([g for _, g in data["gaps"]])
        if gaps:
            print("   time between consecutive stamps (ms): min %.1f  median %.1f  99%% %.1f  max %.1f" % (
                gaps["min"], gaps["median"], gaps["p99"], gaps["max"]))


def plot_spin_up(ax, rows, color, inside):
    """Draw one line's eRPM as a percentage of its own settled speed, its step tick and spin-up dot; return the legend text (None when the
    line has no step). The 80% level is the same line for every motor, drawn by the caller."""
    step = spy_start_summary.find_step(rows)
    if step is None:
        return None
    t0 = rows[step]["t_ms"]
    spin_ms, settled = spy_start_summary.spin_up(rows, step)
    if inside(t0):
        ax.plot([t0], [0], "|", color=color, markersize=9, markeredgewidth=1.5, clip_on=False)
    if settled is None:
        return "never settled"
    real = [(r["t_ms"], 100.0 * r["erpm"] / settled) for r in rows if r["erpm"] is not None and r["erpm"] < 1_000_000 and inside(r["t_ms"])]
    if real:
        ax.plot([t for t, _ in real], [v for _, v in real], "-", color=color, linewidth=1.0, alpha=0.9)
    if spin_ms is None:
        return "settled %d, never reached 80%%" % settled
    if inside(t0 + spin_ms):
        ax.plot([t0 + spin_ms], [100.0 * spy_start_summary.SPIN_UP_FRACTION], "o", color=color, markersize=6,
                markeredgecolor=spy_analyze.SURFACE, markeredgewidth=0.8)
    return "%.0f ms" % spin_ms


def plot(result, out_path, title, from_ms=None, to_ms=None, show=False):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    P = spy_analyze
    session = result["session"]
    lines = sorted(result["lines"])
    marks = arm_marks_ms(session["device"], session["line_high"])
    panels = ["spacing", "events", "spinup", "lag", "gap"]
    heights = {"spacing": 2.6, "events": 0.55 * len(lines) + 1.0, "spinup": 2.6, "lag": 2.2, "gap": 2.0}
    fig, axes = plt.subplots(len(panels), 1, sharex=True, figsize=(13, sum(heights.values()) + 1.2),
                             gridspec_kw={"height_ratios": [heights[p] for p in panels]}, facecolor=P.SURFACE)
    axes = dict(zip(panels, axes))
    for ax in axes.values():
        ax.set_facecolor(P.SURFACE)
        ax.grid(True, color=P.GRID, linewidth=0.6)
        ax.tick_params(colors=P.INK_SECONDARY, labelsize=9)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(P.GRID)

    def inside(t):
        return (from_ms is None or t >= from_ms) and (to_ms is None or t <= to_ms)

    all_rows = [r for line in lines for r in result["lines"][line]["rows"]]
    spacing_values = [r["spacing_us"] for r in all_rows if r["spacing_us"] is not None]
    bounds = P.robust_range(spacing_values, 25) if spacing_values else (0, 1)
    axes["spacing"].set_title("Time between the start of one frame and the next on the wire (us); a triangle on the edge is off the scale",
                              loc="left", fontsize=10, color=P.INK)
    axes["events"].set_title("Per line: ESC replies on the wire (tick: real eRPM; grey: not running; x: damaged) "
                             "and the main Pico's decode stamps (diamonds)", loc="left", fontsize=10, color=P.INK)
    axes["spinup"].set_title("Spin-up: real eRPM as % of each motor's own settled speed; tick = first nonzero throttle frame, "
                             "dot = reaches 80% (the start KPI's spin-up)", loc="left", fontsize=10, color=P.INK)
    axes["lag"].set_title("Lag: main Pico's decode stamp minus the end of the same reply on the wire (ms)", loc="left", fontsize=10, color=P.INK)
    axes["gap"].set_title("Time between two consecutive decode stamps of one motor (ms)", loc="left", fontsize=10, color=P.INK)

    spin_labels = []
    for i, line in enumerate(lines):
        color = P.COLORS[i % len(P.COLORS)]
        data = result["lines"][line]
        pts = [(r["t_ms"], r["spacing_us"]) for r in data["rows"] if r["spacing_us"] is not None and inside(r["t_ms"])]
        if pts:
            P.dots(axes["spacing"], pts, color, bounds)
        row = len(lines) - 1 - i
        run = [r["end_ms"] for r in data["replies"] if r["status"] == "run" and inside(r["end_ms"])]
        idle = [r["end_ms"] for r in data["replies"] if r["status"] == "idle" and inside(r["end_ms"])]
        bad = [r["end_ms"] for r in data["replies"] if r["status"] == "bad" and inside(r["end_ms"])]
        axes["events"].plot(run, [row + 0.18] * len(run), "|", color=color, markersize=7)
        axes["events"].plot(idle, [row + 0.18] * len(idle), "|", color="#b4b2ac", markersize=5)
        axes["events"].plot(bad, [row + 0.18] * len(bad), "x", color=color, markersize=5)
        stamps = [s["t_ms"] for s in data["samples"] if inside(s["t_ms"])]
        axes["events"].plot(stamps, [row - 0.22] * len(stamps), "D", color=color, markersize=4.5, markeredgecolor=P.SURFACE,
                            markeredgewidth=0.6)
        spin_label = plot_spin_up(axes["spinup"], data["rows"], color, inside)
        if spin_label:
            spin_labels.append("line %d: %s" % (line, spin_label))
        lag = [(s["t_ms"], lg) for s, lg in data["lags"] if lg is not None and inside(s["t_ms"])]
        if lag:
            axes["lag"].plot([p[0] for p in lag], [p[1] for p in lag], "o", color=color, markersize=3.2, alpha=0.85)
        gap = [(t, g) for t, g in data["gaps"] if inside(t)]
        if gap:
            axes["gap"].plot([p[0] for p in gap], [p[1] for p in gap], "o", color=color, markersize=3.2, alpha=0.85)
    axes["events"].set_yticks([len(lines) - 1 - i for i in range(len(lines))])
    axes["events"].set_yticklabels(["line %d (GPIO %s)" % (line, session["pins"].get(line)) for line in lines], fontsize=8)
    axes["events"].set_ylim(-0.7, len(lines) - 0.3)
    if spacing_values:
        median = spy_stats.percentile(sorted(spacing_values), 0.5)
        axes["spacing"].axhline(median, color=P.INK_SECONDARY, linewidth=0.8, linestyle=(0, (4, 3)))
    every = session["device"].get("sample_every")
    poll_ms = session["scenario"].get("poll_ms")
    floor_ms = int(every) * poll_ms if every and poll_ms else None
    if floor_ms:
        axes["gap"].axhline(floor_ms, color=P.INK_SECONDARY, linewidth=0.8, linestyle=(0, (4, 3)))
        axes["gap"].annotate("shortest possible: %s decodes x %s ms poll = %s ms" % (every, poll_ms, floor_ms), (0.005, 1),
                             xycoords=("axes fraction", "axes fraction"), xytext=(0, -3), textcoords="offset points",
                             ha="left", va="top", fontsize=8, color=P.INK_SECONDARY)
    # A floor on each range, so a flat result is drawn flat instead of magnified.
    lags = [lg for line in lines for s, lg in result["lines"][line]["lags"] if lg is not None and inside(s["t_ms"])]
    axes["lag"].set_ylim(min(0, min(lags, default=0)) - 1, max(max(lags, default=0), 30) * 1.05)
    gaps = [g for line in lines for t, g in result["lines"][line]["gaps"] if inside(t)]
    axes["gap"].set_ylim(0, max(max(gaps, default=0), 2 * (floor_ms or 100)) * 1.05)
    for ax in axes.values():
        for b in result["behind"]:
            if inside(b):
                ax.axvline(b, color="#c8341e", linewidth=0.8, linestyle=(0, (3, 2)), alpha=0.8)
        for label, value in marks.items():
            if inside(value):
                ax.axvline(value, color=P.INK, linewidth=0.8, alpha=0.6)
    for label, value in marks.items():
        if inside(value):
            axes["spacing"].annotate(label, (value, 1), xycoords=("data", "axes fraction"), xytext=(3, -3),
                                     textcoords="offset points", fontsize=8, color=P.INK, va="top")
    if spin_labels:
        axes["spinup"].annotate("spin-up: " + "; ".join(spin_labels), (0.005, 1), xycoords=("axes fraction", "axes fraction"),
                                xytext=(0, -3), textcoords="offset points", ha="left", va="top", fontsize=8, color=P.INK_SECONDARY)
    axes["spinup"].axhline(100.0 * spy_start_summary.SPIN_UP_FRACTION, color=P.INK_SECONDARY, linewidth=0.8, linestyle=(0, (4, 3)))
    axes["spinup"].set_ylim(-5, 125)
    axes["spinup"].set_ylabel("% of settled", color=P.INK_SECONDARY, fontsize=9)
    axes["spacing"].set_ylabel("us", color=P.INK_SECONDARY, fontsize=9)
    axes["lag"].set_ylabel("ms", color=P.INK_SECONDARY, fontsize=9)
    axes["gap"].set_ylabel("ms", color=P.INK_SECONDARY, fontsize=9)
    axes["gap"].set_xlabel("milliseconds since the start line (GPIO 15) rose; dashed red: main Pico noted Core 1 behind schedule; "
                           "black: arm() / ARMED", color=P.INK_SECONDARY, fontsize=9)
    if from_ms is not None or to_ms is not None:
        axes["gap"].set_xlim(from_ms, to_ms)
    handles = [Line2D([0], [0], color=P.COLORS[i % len(P.COLORS)], linewidth=2, label="line %d" % line) for i, line in enumerate(lines)]
    fig.suptitle(title, x=0.01, ha="left", fontsize=11, color=P.INK)
    fig.legend(handles=handles, loc="upper right", ncol=len(handles), frameon=False, fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    if out_path:
        fig.savefig(out_path, dpi=120, facecolor=P.SURFACE)
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", help="a session folder holding session.json, spy.bin and capture.bin")
    parser.add_argument("--from-ms", type=float, default=None)
    parser.add_argument("--to-ms", type=float, default=None)
    parser.add_argument("--out", default=None, help="picture path (default: <session>/timeline.png)")
    parser.add_argument("--show", action="store_true", help="open an interactive window instead of only writing the file")
    parser.add_argument("--tolerance-us", type=float, default=50, help="a frame spacing this far from the median is a hiccup (default 50)")
    args = parser.parse_args()
    try:
        session = load_session(args.session)
        result = build(session)
        capture_path = Path(args.session) / "capture.bin"
        samples = read_samples(capture_path, session["device"], session["line_high"]) if capture_path.is_file() else []
    except SessionError as err:
        sys.exit("ERROR: %s" % err)
    attach_samples(result, samples)
    if not samples:
        print("NOTE: no capture.bin records: only the wire is drawn")
    print_report(result, args.tolerance_us)
    out = args.out or str(Path(args.session) / "timeline.png")
    plot(result, out, Path(args.session).name, args.from_ms, args.to_ms, args.show)
    print("\nPicture: %s" % out)
    if args.show:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
