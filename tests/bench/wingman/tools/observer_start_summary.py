"""
observer_start_summary.py - what each recorded line did after its throttle step, read from the wire.

  python tests/bench/wingman/tools/observer_start_summary.py captures/<session>/signal_observation.bin [more.bin ...] [--lines 0,1]

For every line that carries replies (a bidirectional motor): when the first nonzero throttle went out, the replies
right after it (the data12=2 start glitch), the median eRPM in windows after the step, whether and when the ESC went
back to its not-running reply while throttle was still on, and any silence in the replies longer than 5 ms after
the step. eRPM is electrical; what a normal start reads depends on the throttle sent (NORMAL_ERPM: 8,400 at throttle 60, about 52,600 at 215).
--table prints one line per start instead (class normal / STALL / PLATEAU / no start by the median eRPM 0.5-1.5 s after the
step, the number of data12=2 replies, when the ESC went back to not-running) and a count per line.
"""

import argparse
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import observer_edge_log
import observer_stats

WINDOWS = ((0, 100), (100, 500), (500, 1500), (1500, 2500), (2500, 4000), (4000, 6000))
# what a normal start of this bench's motor reads once settled, by throttle (eRPM). 60 and 100: measured 2026-10-28 on channels 1-3.
# 215: the median of the four-motor bidirectional runs of 2026-11-01 11-29 to 11-36 (Gemfan D2.2 props), 52.3k-53.0k on every line.
NORMAL_ERPM = {60: 8400, 100: 21800, 215: 52600}
# classify() judges the settled median against the normal reading of the throttle that was sent. The ratios are the old fixed limits
# (stall under 3,000, plateau over 12,000) of the 8,400 that throttle 60 reads, so throttle 60 is judged exactly as before.
STALL_RATIO = 3000 / 8400
PLATEAU_RATIO = 12000 / 8400
TOLERANCE = 0.35
# Spin-up: how long the motor takes from the first nonzero throttle frame to speed. 'Speed' is relative to the motor's own settled speed (the
# median eRPM of its replies from SETTLE_FROM_MS to SETTLE_TO_MS after the step), so no eRPM is written into the tool and it holds for any props.
SETTLE_FROM_MS = 1500
SETTLE_TO_MS = 4000
MIN_SETTLE_REPLIES = 20          # fewer replies than this in the settle window: the motor never settled, no spin-up
SPIN_UP_FRACTION = 0.8           # spun up = at 80% of its settled speed ...
SPIN_UP_REPLIES = 3              # ... for this many replies in a row (a lone spike is not speed)
FINAL_CUT_MIN_MS = 500      # a cut is final when replies stay 'not running' for at least this long with the throttle still on
SETTLE_MS = 600
WATCH_MS = 4000


def find_step(rows):
    """Index of the first nonzero throttle that is real: three frames in a row with the same value of 48 or more. A single odd
    frame (one garbled by an overlapping reply) must not count as the step."""
    for i in range(len(rows) - 2):
        a, b, c = rows[i]["throttle"], rows[i + 1]["throttle"], rows[i + 2]["throttle"]
        if a >= 48 and a == b == c:
            return i
    return None


def spin_up(rows, step):
    """(spin-up ms, settled eRPM): ms from the first nonzero throttle frame to the first of SPIN_UP_REPLIES replies in a row at SPIN_UP_FRACTION of
    the motor's settled speed, and that settled speed. (None, None) when the motor never settled (too few real replies from SETTLE_FROM_MS to
    SETTLE_TO_MS after the step, e.g. it was cut early); (None, settled) when it settled but never reached the fraction."""
    t0 = rows[step]["t_ms"]
    settle = [r["erpm"] for r in rows[step:] if r["erpm"] is not None and r["erpm"] < 1_000_000 and r["throttle"] >= 48
              and SETTLE_FROM_MS <= r["t_ms"] - t0 < SETTLE_TO_MS]
    if len(settle) < MIN_SETTLE_REPLIES:
        return None, None
    settled = statistics.median(settle)
    threshold = SPIN_UP_FRACTION * settled
    streak = 0
    for i in range(step, len(rows)):
        erpm = rows[i]["erpm"]
        streak = streak + 1 if erpm is not None and threshold <= erpm < 1_000_000 else 0
        if streak == SPIN_UP_REPLIES:
            return rows[i - SPIN_UP_REPLIES + 1]["t_ms"] - t0, round(settled)
    return None, round(settled)


def hold_outside_pct(rows, step, spin_up_ms, settled):
    """Percent of the answered replies from the moment the motor spun up to the end of the throttle-on period that are outside +-TOLERANCE of its own settled
    speed (a not-running reply, a start glitch or a stall counts as outside). A motor that stalls or derails without being cut shows here. None when it never
    spun up."""
    if spin_up_ms is None or not settled:
        return None
    start = rows[step]["t_ms"] + spin_up_ms
    span = [r for r in rows[step:] if r["t_ms"] >= start and r["throttle"] >= 48 and r["reply"] != "none"]
    if not span:
        return None
    outside = sum(1 for r in span if r["erpm"] is None or r["erpm"] >= 1_000_000 or abs(r["erpm"] - settled) > TOLERANCE * settled)
    return 100.0 * outside / len(span)


def final_cut_after_ms(rows, step):
    """Ms after the step at which the ESC went back to 'not running' and STAYED there while the throttle was still on (the cut
    AM32's stuck-rotor protection makes: AM32.md section 18), or None. A brief 'not running' stretch followed by running replies
    again is not a cut (stop_after_ms reports that first return to 'not running')."""
    t0 = rows[step]["t_ms"]
    last_running = None
    for i in range(len(rows) - 1, step - 1, -1):
        if rows[i]["throttle"] >= 48 and rows[i]["reply"] not in ("idle", "none"):
            last_running = i
            break
    if last_running is None:
        return None
    tail = [r for r in rows[last_running + 1:] if r["throttle"] >= 48 and r["reply"] == "idle"]
    if not tail or tail[-1]["t_ms"] - tail[0]["t_ms"] < FINAL_CUT_MIN_MS:
        return None
    return tail[0]["t_ms"] - t0


def summarize(records):
    frames = [r for r in records if r["tx"] is not None]
    rows = observer_stats.frame_table(records)
    step = find_step(rows)
    if step is None or not any(r["reply"] != "none" for r in rows):
        return None
    t0 = rows[step]["t_ms"]
    out = {"step_ms": t0, "throttle": rows[step]["throttle"]}
    sequence = []
    for i in range(step, min(step + 40, len(frames))):
        rp = frames[i]["reply"]
        sequence.append(rp["data12"] if rp is not None and rp["crc_ok"] else None)
    runs = []
    for v in sequence:
        if runs and runs[-1][0] == v:
            runs[-1][1] += 1
        else:
            runs.append([v, 1])
    out["first_replies"] = " ".join("%s x%d" % ("0x%03X" % v if v is not None else "none", n) for v, n in runs[:6])
    out["glitch"] = sum(1 for f in frames[step:] if f["reply"] is not None and f["reply"]["crc_ok"] and f["reply"]["data12"] == 2)
    windows = []
    for a, b in WINDOWS:
        values = [r["erpm"] for r in rows[step:] if r["erpm"] and a <= r["t_ms"] - t0 < b and r["erpm"] < 1_000_000]
        idle = sum(1 for r in rows[step:] if r["reply"] == "idle" and a <= r["t_ms"] - t0 < b)
        total = sum(1 for r in rows[step:] if a <= r["t_ms"] - t0 < b)
        windows.append((a, b, statistics.median(values) if values else None, idle, total))
    out["windows"] = windows
    stop = None
    seen_run = False
    for i in range(step, len(rows)):
        r = rows[i]
        if r["reply"] == "run":
            seen_run = True
        if seen_run and r["reply"] == "idle" and r["throttle"] >= 48 and all(x["reply"] == "idle" for x in rows[i:i + 3]):
            stop = r["t_ms"] - t0
            break
    out["stop_after_ms"] = stop
    out["cut_after_ms"] = final_cut_after_ms(rows, step)
    out["spin_up_ms"], out["settled_erpm"] = spin_up(rows, step)
    silences = []
    previous = None
    for r in rows[step:]:
        if r["reply"] == "none":
            continue
        if previous is not None and r["t_ms"] - previous["t_ms"] > 5:
            silences.append((previous["t_ms"] - t0, r["t_ms"] - previous["t_ms"]))
        previous = r
    out["silences"] = silences
    return out


def show(name, line, s):
    print("%s, line %d: throttle %d from %.0f ms" % (name, line, s["throttle"], s["step_ms"]))
    print("   first replies after the step: %s   (data12=2 replies in total: %d)" % (s["first_replies"], s["glitch"]))
    for a, b, med, idle, total in s["windows"]:
        if total:
            print("   %5d-%5d ms after the step: median eRPM %s, %d of %d replies 'not running'" % (
                a, b, "-" if med is None else "%d" % med, idle, total))
    print("   back to 'not running' with throttle still on: %s" % (
        "never" if s["stop_after_ms"] is None else "%.0f ms after the step" % s["stop_after_ms"]))
    print("   silences in the replies after the step (> 5 ms): %s" % (
        ", ".join("%.0f ms long at +%.0f ms" % (d, t) for t, d in s["silences"]) or "none"))


def derail(records, throttle=None):
    """How a start behaves after it should have settled: (ms outside the normal reading between SETTLE_MS and WATCH_MS after
    the step, when it first left it, how many times it dropped to a very low reading). A reply is outside when it is the
    not-running word, a data12=2 glitch, or more than TOLERANCE off the normal eRPM for the throttle. None when the
    throttle has no known normal reading."""
    frames = [r for r in records if r["tx"] is not None]
    rows = observer_stats.frame_table(records)
    step = find_step(rows)
    if step is None:
        return None
    normal = NORMAL_ERPM.get(rows[step]["throttle"])
    if normal is None:
        return None
    t0 = rows[step]["t_ms"]
    outside = 0
    first = None
    lows = 0
    in_low = False
    for i in range(step, len(rows)):
        t = rows[i]["t_ms"] - t0
        if t < SETTLE_MS:
            continue
        if t > WATCH_MS:
            break
        rp = frames[i]["reply"]
        if rp is None or not rp["crc_ok"]:
            continue
        erpm = rp["erpm"]
        bad = rp["not_running"] or rp["data12"] == 2 or (erpm is not None and abs(erpm - normal) > TOLERANCE * normal)
        if bad:
            outside += 1
            if first is None:
                first = t
            if erpm is not None and erpm < 0.35 * normal and not in_low:
                lows += 1
                in_low = True
        else:
            in_low = False
    return {"outside_ms": outside, "first_ms": first, "lows": lows, "normal": normal}


def classify(s):
    """normal / STALL / PLATEAU, from the median eRPM 0.5-1.5 s after the step against what the throttle that was sent normally reads
    (STALL_RATIO and PLATEAU_RATIO of NORMAL_ERPM); no start without a median; no reference for a throttle NORMAL_ERPM does not know."""
    window = next((w for w in s["windows"] if w[0] == 500), None)
    median = window[2] if window else None
    if median is None:
        return "no start"
    normal = NORMAL_ERPM.get(s["throttle"])
    if normal is None:
        return "no reference"
    if median < STALL_RATIO * normal:
        return "STALL"
    return "normal" if median < PLATEAU_RATIO * normal else "PLATEAU"


def verdict(d):
    if d is None:
        return "?"
    return "stays normal" if d["outside_ms"] <= 0.01 * (WATCH_MS - SETTLE_MS) else "DERAILS"


def table(paths, wanted):
    print("%-26s %4s %6s %8s %9s %14s %10s %9s %12s %22s %s" % ("recording", "line", "class", "data12=2", "eRPM .5-1.5s", "back to idle", "final cut", "spin up", "to 4 s", "ms outside / first at", "first replies after the step"))
    counts = {}
    for path in paths:
        start, channels, end = observer_edge_log.parse_stream(Path(path).read_bytes())
        for line in sorted(channels) if wanted is None else wanted:
            if line not in channels:
                continue
            s = summarize(observer_edge_log.analyse(observer_edge_log.edges(channels[line])))
            if s is None:
                continue
            window = next(w for w in s["windows"] if w[0] == 500)
            d = derail(observer_edge_log.analyse(observer_edge_log.edges(channels[line])))
            kind = verdict(d)
            counts.setdefault(line, {}).setdefault(kind, []).append(s["glitch"])
            print("%-26s %4d %6s %8d %12s %14s %10s %9s %12s %22s %s" % (
                Path(path).stem, line, classify(s), s["glitch"], "-" if window[2] is None else "%d" % window[2],
                "never" if s["stop_after_ms"] is None else "%.0f ms" % s["stop_after_ms"],
                "none" if s["cut_after_ms"] is None else "%.0f ms" % s["cut_after_ms"],
                "-" if s["spin_up_ms"] is None else "%.0f ms" % s["spin_up_ms"], kind,
                "-" if d is None else "%d / %s" % (d["outside_ms"], "-" if d["first_ms"] is None else "%.0f ms" % d["first_ms"]),
                s["first_replies"]))
    print()
    for line in sorted(counts):
        parts = []
        for kind, glitches in sorted(counts[line].items()):
            parts.append("%s x%d (data12=2 replies after the step: %s)" % (kind, len(glitches), ", ".join(str(g) for g in glitches)))
        print("line %d: %s" % (line, "; ".join(parts)))


COLORS = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100")      # dataviz reference palette, categorical slots 1-4
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
GRID = "#e4e3df"
CAP = 30000                         # the plot's top edge when the starts' throttle has no known normal reading


def reference(paths, wanted):
    """(throttle, normal eRPM) when every start in paths used one throttle that NORMAL_ERPM knows, else (None, None)."""
    throttles = set()
    for path in paths:
        start, channels, end = observer_edge_log.parse_stream(Path(path).read_bytes())
        for line in sorted(channels) if wanted is None else wanted:
            if line not in channels:
                continue
            rows = observer_stats.frame_table(observer_edge_log.analyse(observer_edge_log.edges(channels[line])))
            step = find_step(rows)
            if step is not None and any(r['reply'] != 'none' for r in rows):
                throttles.add(rows[step]['throttle'])
    if len(throttles) == 1 and next(iter(throttles)) in NORMAL_ERPM:
        throttle = next(iter(throttles))
        return throttle, NORMAL_ERPM[throttle]
    return None, None


def plot(paths, wanted, out_path):
    """Overlay of the eRPM every start reported after its throttle step, time counted from the step: one thin line per
    start, coloured by line; replies above the cap (the data12=2 start glitch) sit on the top edge as triangles and a
    return to 'not running' is a dot on the bottom edge."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), facecolor=SURFACE, gridspec_kw={"height_ratios": [1, 1.4]})
    ranges = ((-20, 600, "First 600 ms after the step"), (-100, 4000, "First 4 s after the step"))
    ref_throttle, ref_normal = reference(paths, wanted)
    cap = CAP if ref_normal is None else int(round(1.5 * ref_normal, -3))       # the top edge: 1.5 x what a normal start reads
    used = set()
    for path in paths:
        start, channels, end = observer_edge_log.parse_stream(Path(path).read_bytes())
        for line in sorted(channels) if wanted is None else wanted:
            if line not in channels:
                continue
            records = observer_edge_log.analyse(observer_edge_log.edges(channels[line]))
            rows = observer_stats.frame_table(records)
            step_index = find_step(rows)
            step = None if step_index is None else rows[step_index]
            if step is None or not any(r["reply"] != "none" for r in rows):
                continue
            used.add(line)
            summary = summarize(records)
            failing = classify(summary) != "normal"
            color = COLORS[line % len(COLORS)]
            xs, ys, over, idle = [], [], [], []
            for r in rows:
                t = r["t_ms"] - step["t_ms"]
                if t < -100 or t > 4000 or r["reply"] == "none":
                    continue
                if r["reply"] == "idle":
                    idle.append(t)
                elif r["erpm"] is not None:
                    if r["erpm"] > cap:
                        over.append(t)
                    else:
                        xs.append(t)
                        ys.append(r["erpm"])
            for ax in axes:
                ax.plot(xs, ys, "-" if not failing else "-", color=color, linewidth=1.8 if failing else 0.8,
                        alpha=0.95 if failing else 0.5, zorder=3 if failing else 2)
                ax.plot(over, [cap] * len(over), "^", color=color, markersize=5, markeredgecolor=SURFACE, markeredgewidth=0.6, zorder=4)
                ax.plot(idle, [0] * len(idle), ".", color=color, markersize=2.5, alpha=0.6, zorder=2)
    for ax, (lo, hi, title) in zip(axes, ranges):
        ax.set_xlim(lo, hi)
        ax.set_ylim(-0.027 * cap, 1.04 * cap)
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRID, linewidth=0.6)
        ax.tick_params(colors=INK_SECONDARY, labelsize=9)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(GRID)
        ax.set_title(title, loc="left", fontsize=10, color=INK)
        ax.set_ylabel("eRPM (electrical)", color=INK_SECONDARY, fontsize=9)
        if ref_normal is not None:
            ax.axhline(ref_normal, color=INK_SECONDARY, linewidth=0.7, linestyle=(0, (4, 3)))
    if ref_normal is not None:
        axes[1].annotate("%d: what throttle %d reads on a normal start" % (ref_normal, ref_throttle), (3950, ref_normal), xytext=(0, 4),
                         textcoords="offset points", ha="right", fontsize=8, color=INK_SECONDARY)
    axes[1].set_xlabel("milliseconds after the first nonzero throttle frame", color=INK_SECONDARY, fontsize=9)
    handles = [Line2D([0], [0], color=COLORS[i % len(COLORS)], linewidth=2, label="line %d (channel %d)" % (i, i + 1)) for i in sorted(used)]
    handles.append(Line2D([0], [0], color=INK_SECONDARY, marker="^", linestyle="", markersize=5, label="data12=2 reply (30,000,000 eRPM), off the scale"))
    handles.append(Line2D([0], [0], color=INK_SECONDARY, marker=".", linestyle="", markersize=5, label="'not running' reply"))
    fig.legend(handles=handles, loc="upper right", ncol=3, frameon=False, fontsize=8)
    fig.suptitle("%d starts, thick = not a normal start" % len(paths), x=0.01, ha="left", fontsize=11, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out_path, dpi=120, facecolor=SURFACE)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="+")
    parser.add_argument("--lines", default=None)
    parser.add_argument("--plot", default=None, metavar="PNG", help="overlay the eRPM of every start after its step into this picture")
    parser.add_argument("--table", action="store_true", help="one line per start, and a count per line of normal / plateau / no start")
    args = parser.parse_args()
    if args.plot:
        plot(args.files, None if args.lines is None else [int(x) for x in args.lines.split(",")], args.plot)
        print("Picture: %s" % args.plot)
    if args.table:
        table(args.files, None if args.lines is None else [int(x) for x in args.lines.split(",")])
        return
    if args.plot:
        return
    for path in args.files:
        start, channels, end = observer_edge_log.parse_stream(Path(path).read_bytes())
        wanted = sorted(channels) if args.lines is None else [int(x) for x in args.lines.split(",")]
        for line in wanted:
            if line not in channels:
                continue
            records = observer_edge_log.analyse(observer_edge_log.edges(channels[line]))
            s = summarize(records)
            if s is not None:
                show(Path(path).stem, line, s)
                print()


if __name__ == "__main__":
    main()
