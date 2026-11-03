"""
spy_analyze.py - what the wingman saw on the ESC signal wires.

  python tests/bench/wingman/tools/spy_analyze.py captures/<session>/spy.bin
  python tests/bench/wingman/tools/spy_analyze.py <file> --plot                       # also write <file>.png
  python tests/bench/wingman/tools/spy_analyze.py <file> --plot --from-ms 1200 --to-ms 1500   # zoom into a stretch
  python tests/bench/wingman/tools/spy_analyze.py <file> --lines 0,2                  # only some lines (index in the recording)

For every recorded line: the frames we sent (throttle value, CRC) and the ESC replies that followed them, read
from the edges alone. Prints the throttle runs, how many frames got a reply, then min / 1% / median / 99% /
max / mean / std of the frame spacing, the bit timing, the frame length, the pulse lengths and the reply delay and eRPM, the frames whose spacing
is off the median by more than --tolerance-us (hiccups), and a timeline in --bucket-ms steps. Time zero is the
moment the start line went high, which is just before arming. A recording the spy marked invalid (lost
data) is reported as such and still analysed, so the loss is visible next to the numbers.

--plot draws the same as time series (every frame is a point, so a stall shows as a spike): the throttle we
sent, the time between frames, the length of each frame, and
when replies exist the reply delay, the eRPM and which frames got no reply or a damaged one.
"""

import argparse
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import spy_edge_log
import spy_stats

UNIT_US = 0.1

# categorical slots 1-4 of the dataviz reference palette (light mode), chart surface and ink
COLORS = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100")
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
GRID = "#e4e3df"
ERPM_CAP = 60000


def runs_text(records):
    runs = []
    for r in records:
        if r["tx"] is None:
            continue
        if runs and runs[-1][0] == r["tx"]["throttle"]:
            runs[-1][1] += 1
        else:
            runs.append([r["tx"]["throttle"], 1])
    return " ".join("%dx%d" % (v, n) for v, n in runs)


def timeline(rows, bucket_ms):
    buckets = {}
    for r in rows:
        row = buckets.setdefault(int(r["t_ms"] // bucket_ms), {"frames": 0, "replies": 0, "bad": 0, "idle": 0, "erpm": []})
        row["frames"] += 1
        if r["reply"] != "none":
            row["replies"] += 1
        if r["reply"] == "bad":
            row["bad"] += 1
        elif r["reply"] == "idle":
            row["idle"] += 1
        elif r["erpm"]:
            row["erpm"].append(r["erpm"])
    print("   %-9s %6s %7s %5s %11s %s" % ("from ms", "frames", "replies", "bad", "not running", "eRPM min / mean / max"))
    for b in sorted(buckets):
        row = buckets[b]
        e = row["erpm"]
        spread = "%d / %d / %d" % (min(e), statistics.mean(e), max(e)) if e else "-"
        print("   %-9d %6d %7d %5d %11d %s" % (b * bucket_ms, row["frames"], row["replies"], row["bad"], row["idle"], spread))


def print_summaries(rows):
    print("   %-40s %6s %9s %9s %9s %9s %9s %9s %8s" % ("", "n", "min", "1%", "median", "99%", "max", "mean", "std"))
    for label, s in spy_stats.summaries(rows):
        print("   %-40s %6d %9.2f %9.2f %9.2f %9.2f %9.2f %9.2f %8.3f" % (
            label, s["n"], s["min"], s["p1"], s["median"], s["p99"], s["max"], s["mean"], s["std"]))


def print_hiccups(rows, tolerance_us, limit=12):
    found = spy_stats.hiccups(rows, tolerance_us)
    spacings = sorted(r["spacing_us"] for r in rows if r["spacing_us"] is not None)
    if not found:
        print("   frame spacing: no frame is more than %d us from the median" % tolerance_us)
        return
    median = spy_stats.percentile(spacings, 0.5)
    found.sort(key=lambda x: -abs(x[1] - median))
    print("   frame spacing: %d frames more than %d us from the median (%.1f us); the largest:" % (len(found), tolerance_us, median))
    for t, spacing in found[:limit]:
        print("      at %9.1f ms  %9.1f us  (%+.1f)" % (t, spacing, spacing - median))


def robust_range(values, min_half):
    """(low, high) centred on the median, wide enough for the 1%-99% spread (and at least min_half each way)."""
    ordered = sorted(values)
    if not ordered:
        return (0, 1)
    median = spy_stats.percentile(ordered, 0.5)
    half = max(min_half, 1.5 * (spy_stats.percentile(ordered, 0.99) - spy_stats.percentile(ordered, 0.01)))
    return (median - half, median + half)


def dots(ax, points, color, bounds):
    """Draw (x, y) points; a point outside bounds sits on the edge as a triangle, so no outlier is ever hidden."""
    low, high = bounds
    inside = [p for p in points if low <= p[1] <= high]
    ax.plot([p[0] for p in inside], [p[1] for p in inside], ".", color=color, markersize=2.5, alpha=0.8)
    above = [p[0] for p in points if p[1] > high]
    below = [p[0] for p in points if p[1] < low]
    if above:
        ax.plot(above, [high] * len(above), "^", color=color, markersize=5, markeredgecolor=SURFACE, markeredgewidth=0.8)
    if below:
        ax.plot(below, [low] * len(below), "v", color=color, markersize=5, markeredgecolor=SURFACE, markeredgewidth=0.8)
    ax.set_ylim(low - (high - low) * 0.04, high + (high - low) * 0.04)


def plot_pulses(pulse_sets, labels, out_path, title):
    """How far the pulse of every bit sits from the line AM32 decides on (its frame span / 32)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ones = [x for o, _, _ in pulse_sets for x in o]
    zeros = [x for _, z, _ in pulse_sets for x in z]
    thresholds = [x for _, _, t in pulse_sets for x in t]
    if not ones or not zeros or not thresholds:
        return
    fig, ax = plt.subplots(figsize=(10, 4.2), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    low = min(zeros) - 0.2
    high = max(ones) + 0.2
    step = 0.1
    bins = [low + step * i for i in range(int((high - low) / step) + 2)]
    ax.hist(zeros, bins=bins, color=COLORS[0], alpha=0.9, label="pulse of a 0 bit")
    ax.hist(ones, bins=bins, color=COLORS[1], alpha=0.9, label="pulse of a 1 bit")
    ax.axvspan(min(thresholds), max(thresholds), color=INK, alpha=0.25, linewidth=0)
    ax.axvline(sum(thresholds) / len(thresholds), color=INK, linewidth=1.2)
    mean_zero = sum(zeros) / len(zeros)
    mean_one = sum(ones) / len(ones)
    mean_threshold = sum(thresholds) / len(thresholds)
    top = ax.get_ylim()[1]
    ax.annotate("AM32 decides at %.2f us (frame span / 32)" % mean_threshold, (mean_threshold, top * 0.97),
                xytext=(6, 0), textcoords="offset points", fontsize=9, color=INK, va="top")
    ax.annotate("", xy=(mean_threshold, top * 0.55), xytext=(mean_zero, top * 0.55),
                arrowprops={"arrowstyle": "<->", "color": INK_SECONDARY, "linewidth": 0.8})
    ax.text((mean_zero + mean_threshold) / 2, top * 0.58, "%.2f us" % (mean_threshold - mean_zero), ha="center", fontsize=9, color=INK_SECONDARY)
    ax.annotate("", xy=(mean_one, top * 0.55), xytext=(mean_threshold, top * 0.55),
                arrowprops={"arrowstyle": "<->", "color": INK_SECONDARY, "linewidth": 0.8})
    ax.text((mean_one + mean_threshold) / 2, top * 0.58, "%.2f us" % (mean_one - mean_threshold), ha="center", fontsize=9, color=INK_SECONDARY)
    ax.set_xlabel("pulse length (us) - one pulse reads +-0.1 us on this spy", color=INK_SECONDARY, fontsize=9)
    ax.set_ylabel("bits", color=INK_SECONDARY, fontsize=9)
    ax.set_title("%s - every bit's pulse against AM32's threshold, %d line%s" % (
        title, len(labels), "" if len(labels) == 1 else "s"), loc="left", fontsize=10, color=INK)
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.tick_params(colors=INK_SECONDARY, labelsize=9)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.legend(frameon=False, fontsize=9, loc="upper right", bbox_to_anchor=(1, 0.85))
    fig.tight_layout()
    fig.savefig(out_path, dpi=120, facecolor=SURFACE)
    plt.close(fig)


def plot(tables, labels, out_path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    replies = [any(r["reply"] != "none" for r in rows) for rows in tables]
    has_replies = any(replies)
    panels = ["throttle", "spacing", "bits"] + (["delay", "erpm", "missing"] if has_replies else [])
    heights = {"throttle": 1.2, "spacing": 2.6, "bits": 2.0, "delay": 1.8, "erpm": 2.4, "missing": 1.4}
    fig, axes = plt.subplots(len(panels), 1, sharex=True, figsize=(12, sum(heights[p] for p in panels) + 1.2),
                             gridspec_kw={"height_ratios": [heights[p] for p in panels]}, facecolor=SURFACE)
    axes = dict(zip(panels, axes if len(panels) > 1 else [axes]))
    for ax in axes.values():
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRID, linewidth=0.6)
        ax.tick_params(colors=INK_SECONDARY, labelsize=9)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(GRID)

    def heading(ax, text):
        ax.set_title(text, loc="left", fontsize=10, color=INK)

    all_rows = [r for rows in tables for r in rows]
    bounds = {
        "spacing": robust_range([r["spacing_us"] for r in all_rows if r["spacing_us"] is not None], 25),
        "bits": robust_range([r["frame_us"] for r in all_rows], 0.5),
        "delay": robust_range([r["delay_us"] for r in all_rows if r["delay_us"] is not None], 3),
    }
    heading(axes["throttle"], "Throttle value we sent")
    heading(axes["spacing"], "Time between the start of one frame and the next (us) - every frame is a dot; a triangle on the edge is off the scale")
    heading(axes["bits"], "Length of each frame, from its first bit to its last bit (us) - 15 bit periods")
    for i, rows in enumerate(tables):
        color = COLORS[i % len(COLORS)]
        t = [r["t_ms"] for r in rows]
        axes["throttle"].step(t, [r["throttle"] for r in rows], where="post", color=color, linewidth=1.2)
        pts = [(r["t_ms"], r["spacing_us"]) for r in rows if r["spacing_us"] is not None]
        if pts:
            dots(axes["spacing"], pts, color, bounds["spacing"])
        dots(axes["bits"], [(r["t_ms"], r["frame_us"]) for r in rows], color, bounds["bits"])
    axes["throttle"].set_ylabel("throttle", color=INK_SECONDARY, fontsize=9)
    axes["spacing"].set_ylabel("us", color=INK_SECONDARY, fontsize=9)
    axes["bits"].set_ylabel("us", color=INK_SECONDARY, fontsize=9)
    spacings = sorted(r["spacing_us"] for r in all_rows if r["spacing_us"] is not None)
    if spacings:
        median = spy_stats.percentile(spacings, 0.5)
        axes["spacing"].axhline(median, color=INK_SECONDARY, linewidth=0.8, linestyle=(0, (4, 3)))
        axes["spacing"].annotate("median %.1f us" % median, (0.005, 1), xycoords=("axes fraction", "axes fraction"),
                                 xytext=(0, -3), textcoords="offset points", ha="left", va="top", fontsize=8,
                                 color=INK_SECONDARY)
        widest = max(all_rows, key=lambda r: abs((r["spacing_us"] or median) - median))
        if widest["spacing_us"] is not None and abs(widest["spacing_us"] - median) > 20:
            low, high = bounds["spacing"]
            axes["spacing"].annotate("largest: %.0f us at %.0f ms" % (widest["spacing_us"], widest["t_ms"]),
                                     (widest["t_ms"], min(max(widest["spacing_us"], low), high)), xytext=(10, -2),
                                     textcoords="offset points",
                                     fontsize=8, color=INK,
                                     arrowprops={"arrowstyle": "-", "color": INK_SECONDARY, "linewidth": 0.6})

    if has_replies:
        heading(axes["delay"], "Delay from the end of our frame to the start of the ESC's reply (us)")
        heading(axes["erpm"], "eRPM the ESC reported (values above %d are drawn as a triangle on the top edge)" % ERPM_CAP)
        heading(axes["missing"], "Frames whose reply is missing (|) or damaged (x)")
        for i, rows in enumerate(tables):
            if not replies[i]:
                continue
            color = COLORS[i % len(COLORS)]
            d = [(r["t_ms"], r["delay_us"]) for r in rows if r["delay_us"] is not None]
            dots(axes["delay"], d, color, bounds["delay"])
            run = [(r["t_ms"], r["erpm"]) for r in rows if r["erpm"] is not None]
            inside = [p for p in run if p[1] <= ERPM_CAP]
            above = [p for p in run if p[1] > ERPM_CAP]
            axes["erpm"].plot([p[0] for p in inside], [p[1] for p in inside], ".", color=color, markersize=2.5, alpha=0.8)
            if above:
                axes["erpm"].plot([p[0] for p in above], [ERPM_CAP] * len(above), "^", color=color, markersize=5,
                                  markeredgecolor=SURFACE, markeredgewidth=0.8)
            idle = [(r["t_ms"], 0) for r in rows if r["reply"] == "idle"]
            if idle:
                axes["erpm"].plot([p[0] for p in idle], [0] * len(idle), ".", color=color, markersize=2, alpha=0.35)
            none = [r["t_ms"] for r in rows if r["reply"] == "none"]
            bad = [r["t_ms"] for r in rows if r["reply"] == "bad"]
            axes["missing"].plot(none, [i] * len(none), "|", color=color, markersize=8)
            axes["missing"].plot(bad, [i] * len(bad), "x", color=color, markersize=5)
        axes["delay"].set_ylabel("us", color=INK_SECONDARY, fontsize=9)
        axes["erpm"].set_ylabel("eRPM", color=INK_SECONDARY, fontsize=9)
        axes["missing"].set_yticks(range(len(tables)))
        axes["missing"].set_yticklabels(labels, fontsize=8)
        axes["missing"].set_ylim(-0.7, len(tables) - 0.3)
    list(axes.values())[-1].set_xlabel("milliseconds since the first frame of ours", color=INK_SECONDARY, fontsize=9)
    handles = [Line2D([0], [0], color=COLORS[i % len(COLORS)], linewidth=2, label=labels[i]) for i in range(len(tables))]
    fig.suptitle(title, x=0.01, ha="left", fontsize=11, color=INK)
    fig.legend(handles=handles, loc="upper right", ncol=len(handles), frameon=False, fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_path, dpi=120, facecolor=SURFACE)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("file")
    parser.add_argument("--bucket-ms", type=int, default=250)
    parser.add_argument("--unidirectional", action="store_true", help="the lines carried unidirectional frames (idle low, plain CRC, no reply)")
    parser.add_argument("--plot", action="store_true", help="write the time-series picture next to the file (or at --out)")
    parser.add_argument("--out", default=None, help="picture path (default: the file name with .png)")
    parser.add_argument("--from-ms", type=float, default=None)
    parser.add_argument("--to-ms", type=float, default=None)
    parser.add_argument("--lines", default=None, help="comma-separated line indexes to show (default all)")
    parser.add_argument("--tolerance-us", type=float, default=50, help="a frame spacing this far from the median is a hiccup (default 50)")
    args = parser.parse_args()
    start, channels, end = spy_edge_log.parse_stream(Path(args.file).read_bytes())
    info = spy_edge_log.parse_end(end)
    pins = {}
    for token in start.split():
        if token.startswith("pins="):
            pins = dict(enumerate(int(p) for p in token[5:].split(",")))
    print("start: %s" % start)
    print("end:   %s" % end)
    if not info["valid"]:
        print("WARNING: the spy reports lost data - gaps below may be the recorder's, not the line's")
    wanted = sorted(channels) if args.lines is None else [int(x) for x in args.lines.split(",")]
    tables = []
    labels = []
    pulse_sets = []
    analysed = {}
    for channel in wanted:
        if channel in channels:
            analysed[channel] = spy_edge_log.analyse(spy_edge_log.edges(channels[channel], invert=args.unidirectional), args.unidirectional)
    firsts = [next((r["start"] for r in recs if r["tx"] is not None), None) for recs in analysed.values()]
    firsts = [x for x in firsts if x is not None]
    origin = min(firsts) if firsts else 0
    if origin * UNIT_US / 1000 > 200:
        print("NOTE: the recording holds %.0f ms of quiet line before the first frame of ours (the spy's gate opened early); "
              "all times below count from the first frame" % (origin * UNIT_US / 1000))
    for channel in wanted:
        if channel not in channels:
            continue
        records = analysed[channel]
        rows = spy_stats.window(spy_stats.frame_table(records, origin), args.from_ms, args.to_ms)
        label = "line %d (GPIO %d)" % (channel, pins[channel]) if channel in pins else "line %d" % channel
        tables.append(rows)
        labels.append(label)
        pulse_sets.append(spy_stats.pulse_data(records))
        with_tx = [r for r in records if r["tx"] is not None]
        print("\n%s: %d groups, %d with a complete frame of ours, %d with a reply" % (
            label, len(records), len(with_tx), sum(1 for r in with_tx if r["reply"] is not None)))
        print("   throttle sent: %s" % runs_text(with_tx))
        print("   our frames with a bad CRC: %d; groups without a complete frame of ours: %d" % (
            sum(1 for r in with_tx if not r["tx"]["crc_ok"]), len(records) - len(with_tx)))
        if args.from_ms is not None or args.to_ms is not None:
            print("   (statistics below: %s .. %s ms only)" % (args.from_ms, args.to_ms))
        print_summaries(rows)
        print_hiccups(rows, args.tolerance_us)
        timeline(rows, args.bucket_ms)
    if args.plot and tables:
        out = args.out or str(Path(args.file).with_suffix(".png"))
        plot(tables, labels, out, Path(args.file).name)
        print("\nPicture: %s" % out)
        pulses_out = str(Path(out).with_name(Path(out).stem + "_pulses.png"))
        plot_pulses(pulse_sets, labels, pulses_out, Path(args.file).name)
        print("Picture: %s" % pulses_out)


if __name__ == "__main__":
    main()
