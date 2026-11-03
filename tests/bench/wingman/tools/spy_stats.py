"""
spy_stats.py - numbers and per-frame series from a spy recording (tests/bench/wingman/tools/spy_edge_log.py records).

Pure Python, no plotting: tests/pc/test_spy_stats.py exercises it; tests/bench/wingman/tools/spy_analyze.py prints and
plots what it returns. Times are milliseconds from the origin given to frame_table (by default the start of the recording), durations microseconds.
"""

UNIT_US = 0.1                       # spy_edge_log times are in units of 0.1us


def percentile(sorted_values, fraction):
    if not sorted_values:
        return None
    index = min(len(sorted_values) - 1, max(0, int(round(fraction * (len(sorted_values) - 1)))))
    return sorted_values[index]


def summary(values):
    """n, min, p1, median, p99, max, mean, std of a list of numbers (None entries when it is empty)."""
    values = sorted(values)
    n = len(values)
    if not n:
        return {"n": 0, "min": None, "p1": None, "median": None, "p99": None, "max": None, "mean": None, "std": None}
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / n
    return {"n": n, "min": values[0], "p1": percentile(values, 0.01), "median": percentile(values, 0.5),
            "p99": percentile(values, 0.99), "max": values[-1], "mean": mean, "std": var ** 0.5}


def reply_status(reply):
    if reply is None:
        return "none"
    if not reply["crc_ok"]:
        return "bad"
    if reply["not_running"]:
        return "idle"
    return "run"


def frame_table(records, origin=0):
    """One row per group that starts with a complete frame of ours, in time order:
    t_ms, throttle, spacing_us (to the previous such frame; None for the first), period_min/mean/max_us (bit
    period within the frame), frame_us (first bit start to last bit start, 15 bit periods), low1_us / low0_us
    (mean pulse length of its 1 and 0 bits: the low pulse of a bidirectional frame, the high pulse of a
    unidirectional one), and the reply: status
    ('none', 'bad', 'idle' = AM32's not-running reply, 'run'), delay_us, erpm. t_ms counts from origin (edge units),
    the start of the recording by default."""
    rows = []
    previous = None
    for r in records:
        tx = r["tx"]
        if tx is None:
            continue
        ones = [low for bit, low in zip(tx["bits"], tx["lows"]) if bit]
        zeros = [low for bit, low in zip(tx["bits"], tx["lows"]) if not bit]
        reply = r["reply"]
        rows.append({
            "t_ms": (r["start"] - origin) * UNIT_US / 1000,
            "throttle": tx["throttle"],
            "crc_ok": tx["crc_ok"],
            "spacing_us": None if previous is None else (r["start"] - previous) * UNIT_US,
            "period_min_us": min(tx["periods"]) * UNIT_US,
            "period_mean_us": sum(tx["periods"]) / len(tx["periods"]) * UNIT_US,
            "period_max_us": max(tx["periods"]) * UNIT_US,
            "frame_us": sum(tx["periods"]) * UNIT_US,
            "low1_us": sum(ones) / len(ones) * UNIT_US if ones else None,
            "low0_us": sum(zeros) / len(zeros) * UNIT_US if zeros else None,
            "reply": reply_status(reply),
            "delay_us": None if reply is None else reply["delay"] * UNIT_US,
            "erpm": reply["erpm"] if reply is not None and reply["crc_ok"] and not reply["not_running"] else None,
        })
        previous = r["start"]
    return rows


def hiccups(rows, tolerance_us=50):
    """Frames whose spacing to the previous frame is more than tolerance_us from the median spacing:
    [(t_ms, spacing_us)], in time order."""
    spacings = [r["spacing_us"] for r in rows if r["spacing_us"] is not None]
    if not spacings:
        return []
    median = percentile(sorted(spacings), 0.5)
    return [(r["t_ms"], r["spacing_us"]) for r in rows
            if r["spacing_us"] is not None and abs(r["spacing_us"] - median) > tolerance_us]


def window(rows, from_ms=None, to_ms=None):
    return [r for r in rows if (from_ms is None or r["t_ms"] >= from_ms) and (to_ms is None or r["t_ms"] <= to_ms)]


SUMMARY_FIELDS = (
    ("frame spacing (us)", "spacing_us"),
    ("bit period, shortest in a frame (us)", "period_min_us"),
    ("bit period, mean of a frame (us)", "period_mean_us"),
    ("bit period, longest in a frame (us)", "period_max_us"),
    ("frame length, first to last bit start (us)", "frame_us"),
    ("pulse length of a 1 bit (us)", "low1_us"),
    ("pulse length of a 0 bit (us)", "low0_us"),
    ("reply delay after our frame (us)", "delay_us"),
    ("reply eRPM", "erpm"),
)


def summaries(rows):
    """[(label, summary)] over the rows for each field of SUMMARY_FIELDS that has any value."""
    out = []
    for label, key in SUMMARY_FIELDS:
        values = [r[key] for r in rows if r[key] is not None]
        if values:
            out.append((label, summary(values)))
    return out


def pulse_data(records):
    """(ones, zeros, thresholds): the pulse length in microseconds of every 1 bit and every 0 bit of every
    complete frame, and AM32's threshold for each frame (its first-to-last-edge span / 32), so a picture can
    show how far each pulse sits from the line AM32 decides on."""
    ones = []
    zeros = []
    thresholds = []
    for r in records:
        if r["tx"] is None or r.get("tx_edges") is None:
            continue
        edges = r["tx_edges"]
        thresholds.append((edges[31][0] - edges[0][0]) / 32 * UNIT_US)
        for i in range(16):
            pulse = (edges[2 * i + 1][0] - edges[2 * i][0]) * UNIT_US
            (ones if r["tx"]["bits"][i] else zeros).append(pulse)
    return ones, zeros, thresholds
