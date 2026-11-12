"""
observer_am32_table.py - how closely does what we put on the wire match what AM32 expects? One table per DSHOT speed,
one column per recording, one row per measure, the worst of the recorded lines in each cell.

  python tests/bench/wingman/tools/observer_am32_table.py [--unidirectional] captures/<session>/signal_observation.bin ...

Every row is a measure of tests/bench/wingman/tools/observer_check_am32.py (see its docstring for where in AM32's source each
condition comes from) with the limit AM32 imposes next to it, so repeated runs can be laid side by side and the
spread between them read off directly. This is the input side only: it reads our own frames on the observer, no
ESC is involved. It describes the wire as the observer (+-0.1 us per edge) saw it; the tick-based limits are the F421
ones, the only family supported (see observer_check_am32.py). The last block (frame spacing) is our own pace, not an AM32 condition: AM32 does not measure
the time between frames.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import observer_check_am32 as check
import observer_edge_log
import observer_stats

ORDER = {check.FAIL: 3, check.WARN: 2, check.INFO: 1, check.PASS: 0, check.NA: -1}


def worst(statuses):
    return max(statuses, key=lambda s: ORDER[s]) if statuses else check.NA


def find(results, fragment):
    return next(r for r in results if fragment in r.name)


def reply_metrics(results, records):
    """The reply-side numbers of one line (None when it has no replies)."""
    if not any(r["reply"] is not None for r in records):
        return None
    latch = find(results, "bidirectional latch")
    coverage = find(results, "an answer to every frame")
    silence = find(results, "arming silence")
    integrity = find(results, "CRC-valid")
    timing = find(results, "reply bit time")
    start = find(results, "stuck-rotor")
    frames = [r for r in records if r["tx"] is not None]
    glitch = sum(1 for r in frames if r["reply"] is not None and r["reply"]["crc_ok"] and r["reply"]["data12"] == 2)
    rows = observer_stats.frame_table(records)
    step = next((r for r in rows if r["throttle"] >= 48), None)
    steady = [r["erpm"] for r in rows if step is not None and r["erpm"] and 1000 < r["t_ms"] - step["t_ms"]]
    return {
        "first_reply_frame": latch.values.get("first_reply_frame"),
        "coverage_pct": coverage.values.get("coverage_pct"),
        "silence_start_ms": silence.values.get("start_ms"),
        "silence_ms": silence.values.get("duration_ms"),
        "bad_crc": integrity.values.get("bad_crc", 0),
        "edt_shaped": integrity.values.get("edt_shaped", 0),
        "bit_us": timing.values.get("bit_us"),
        "expected_us": timing.values.get("expected_us"),
        "off_pct": timing.values.get("off_pct"),
        "delay_us": timing.values.get("delay_us"),
        "first_running_ms": start.values.get("first_running_ms"),
        "glitch_replies": glitch,
        "steady_erpm": observer_stats.percentile(sorted(steady), 0.5) if steady else None,
        "status": {"latch": latch.status, "coverage": coverage.status, "silence": silence.status, "integrity": integrity.status},
    }


def line_metrics(records, unidirectional):
    """The numbers and verdicts of one recorded line."""
    results = check.run_checks(records, unidirectional)
    bit = find(results, "bit threshold")
    window = find(results, "frame length is inside")
    band = find(results, "classify as DSHOT")
    crc = find(results, "CRC polarity")
    throttle = find(results, "throttle values")
    gap = find(results, "no gap")
    rows = observer_stats.frame_table(records)
    return {
        "speed": band.values.get("speed"),
        "frames": bit.values["frames"],
        "bits": bit.values["frames"] * 16,
        "mismatches": bit.values["mismatches"],
        "margin_zero": bit.values["margin_zero_us"],
        "margin_one": bit.values["margin_one_us"],
        "threshold": bit.values["threshold_us"],
        "worst_pct": window.values.get("worst_pct"),
        "outside": window.values.get("outside", 0),
        "shortest_min": band.values["shortest_min_us"],
        "shortest_max": band.values["shortest_max_us"],
        "band_low": band.values["band_low_us"],
        "band_high": band.values["band_high_us"],
        "crc_bad": crc.values["bad"],
        "commands": throttle.values["command_frames"],
        "max_gap_ms": gap.values["max_gap_ms"],
        "spacings": [r["spacing_us"] for r in rows if r["spacing_us"] is not None],
        "reply": None if unidirectional else reply_metrics(results, records),
        "status": {"bit": bit.status, "window": window.status, "band": band.status, "crc": crc.status,
                   "throttle": throttle.status, "gap": gap.status},
    }


def combine(lines):
    """One recording's cell values: the worst over its lines (the pooled spacings for the pace block)."""
    spacings = sorted(s for m in lines for s in m["spacings"])
    pct = [m["worst_pct"] for m in lines if m["worst_pct"] is not None]
    summary = observer_stats.summary(spacings)
    return {
        "speed": lines[0]["speed"],
        "lines": len(lines),
        "frames": sum(m["frames"] for m in lines),
        "mismatches": sum(m["mismatches"] for m in lines),
        "bits": sum(m["bits"] for m in lines),
        "margin_zero": min(m["margin_zero"] for m in lines),
        "margin_one": min(m["margin_one"] for m in lines),
        "threshold": sum(m["threshold"] for m in lines) / len(lines),
        "worst_pct": max(pct) if pct else None,
        "outside": sum(m["outside"] for m in lines),
        "shortest_min": min(m["shortest_min"] for m in lines),
        "shortest_max": max(m["shortest_max"] for m in lines),
        "band_low": lines[0]["band_low"],
        "band_high": lines[0]["band_high"],
        "crc_bad": sum(m["crc_bad"] for m in lines),
        "commands": sum(m["commands"] for m in lines),
        "max_gap_ms": max(m["max_gap_ms"] for m in lines),
        "spacing": summary,
        "reply": next((m["reply"] for m in lines if m["reply"] is not None), None),
        "status": {k: worst([m["status"][k] for m in lines]) for k in lines[0]["status"]},
    }


def f(value, fmt):
    return "-" if value is None else fmt % value


# (label, value of a cell, limit AM32 imposes, status key)
ROWS = (
    ("bits AM32 would read differently from ours", lambda c: "%d of %d" % (c["mismatches"], c["bits"]), "0", "bit"),
    ("0-bit pulse below AM32's threshold (us)", lambda c: f(c["margin_zero"], "%.2f"), "> 0 (observer +-0.1)", "bit"),
    ("1-bit pulse above AM32's threshold (us)", lambda c: f(c["margin_one"], "%.2f"), "> 0 (observer +-0.1)", "bit"),
    ("worst frame length off the learned one (%)", lambda c: f(c["worst_pct"], "%.1f"), "< 6.25", "window"),
    ("frames outside the learned window", lambda c: str(c["outside"]), "0", "window"),
    ("shortest edge interval, first 8 frames (us)",
     lambda c: "%.1f-%.1f" % (c["shortest_min"], c["shortest_max"]), None, "band"),
    ("frames with the wrong CRC polarity", lambda c: str(c["crc_bad"]), "0", "crc"),
    ("frames carrying a command value (1-47)", lambda c: str(c["commands"]), "0", "throttle"),
    ("largest gap between frames (ms)", lambda c: f(c["max_gap_ms"], "%.1f"), "< 500", "gap"),
)
def fnum(value, fmt):
    return "-" if value is None else fmt % value


# the reply side, for recordings with an ESC answering: (label, value of a cell's reply block, limit, status key)
REPLY_ROWS = (
    ("first reply after (frames)", lambda r: fnum(r["first_reply_frame"], "%d"), ">= 101 (latch)", "latch"),
    ("frames answered outside explained silences (%)", lambda r: fnum(r["coverage_pct"], "%.1f"), "100", "coverage"),
    ("arming silence starts after our first frame (ms)", lambda r: fnum(r["silence_start_ms"], "%.0f"), ">= 1020", "silence"),
    ("arming silence lasts (ms)", lambda r: fnum(r["silence_ms"], "%.0f"), "about 300 per cell", None),
    ("replies with a bad CRC / shaped like EDT", lambda r: "%d / %d" % (r["bad_crc"], r["edt_shaped"]), "0 / 0", "integrity"),
    ("reply bit time (us)", lambda r: fnum(r["bit_us"], "%.3f"), "F421: 1.2833 / 2.5667", None),
    ("  off the F421 figure (%)", lambda r: fnum(r["off_pct"], "%+.1f"), "within +-1.5", None),
    ("reply starts after our frame (us), median", lambda r: fnum(r["delay_us"], "%.1f"), "", None),
    ("first running reply after the throttle step (ms)", lambda r: fnum(r["first_running_ms"], "%.0f"), "", None),
    ("replies reading data12=2 (30,000,000 eRPM)", lambda r: str(r["glitch_replies"]), "", None),
    ("steady eRPM, median (1 s after the step on)", lambda r: fnum(r["steady_erpm"], "%.0f"), "", None),
)

PACE_ROWS = (
    ("frame spacing, median (us)", "median"), ("frame spacing, 1% .. 99% (us)", None),
    ("frame spacing, min .. max (us)", None), ("frame spacing, standard deviation (us)", "std"),
)


def pace_value(label, c):
    s = c["spacing"]
    if "median" in label:
        return f(s["median"], "%.1f")
    if "1%" in label:
        return "%s .. %s" % (f(s["p1"], "%.1f"), f(s["p99"], "%.1f"))
    if "min" in label:
        return "%s .. %s" % (f(s["min"], "%.1f"), f(s["max"], "%.1f"))
    return f(s["std"], "%.2f")


def table(speed, labels, cells):
    first = cells[0]
    lines = ["DSHOT%s - %d recording(s), %d lines each, %d frames per line" % (
        speed, len(cells), first["lines"], first["frames"] // first["lines"])]
    width = max(len(r[0]) for r in ROWS + REPLY_ROWS + tuple((p[0], 0, 0, 0) for p in PACE_ROWS))
    header = "%-*s" % (width, "") + "".join("  %-16s" % l for l in labels) + "  %-22s %s" % ("limit AM32 imposes", "verdict")
    lines.append(header)
    for label, value, limit, key in ROWS:
        shown = limit
        if key == "band":
            shown = "%.2f .. %.2f" % (first["band_low"], first["band_high"])
        lines.append("%-*s" % (width, label) + "".join("  %-16s" % value(c) for c in cells)
                     + "  %-22s %s" % (shown, worst([c["status"][key] for c in cells])))
    if all(c["reply"] is not None for c in cells):
        lines.append("%-*s" % (width, "what the ESC answers"))
        for label, value, limit, key in REPLY_ROWS:
            verdict = worst([c["reply"]["status"][key] for c in cells]) if key else ""
            lines.append("%-*s" % (width, label) + "".join("  %-16s" % value(c["reply"]) for c in cells)
                         + "  %-22s %s" % (limit, verdict))
    lines.append("%-*s" % (width, "our pace (AM32 does not measure it)"))
    for label, _ in PACE_ROWS:
        lines.append("%-*s" % (width, label) + "".join("  %-16s" % pace_value(label, c) for c in cells))
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="+")
    parser.add_argument("--unidirectional", action="store_true")
    parser.add_argument("--lines", default=None, help="comma-separated line indexes to use (default all)")
    args = parser.parse_args()
    by_speed = {}
    for path in sorted(args.files):
        start, channels, end = observer_edge_log.parse_stream(Path(path).read_bytes())
        if not observer_edge_log.parse_end(end)["valid"]:
            print("WARNING: %s: the observer reports lost data (%s)" % (path, end))
        wanted = sorted(channels) if args.lines is None else [int(x) for x in args.lines.split(",")]
        lines = [line_metrics(observer_edge_log.analyse(observer_edge_log.edges(channels[c], invert=args.unidirectional), args.unidirectional),
                              args.unidirectional) for c in wanted if c in channels]
        cell = combine(lines)
        by_speed.setdefault(cell["speed"], []).append((Path(path).stem, cell))
    for speed in sorted(by_speed, reverse=True):
        entries = by_speed[speed]
        labels = ["run %d" % (i + 1) for i in range(len(entries))]
        print(table(speed, labels, [c for _, c in entries]))
        for label, (stem, _) in zip(labels, entries):
            print("  %s = %s" % (label, stem))
        print()


if __name__ == "__main__":
    main()
