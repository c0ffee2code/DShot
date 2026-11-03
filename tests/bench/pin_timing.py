# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# Turns the edge list of a pin recording (see pin_sampler.py) into frame timing: when each
# DShot frame started on a line, how far apart the frames are, how steady that spacing is,
# and how the lines of several channels sit relative to each other inside one pass of the
# command loop. Pure Python (no rp2 or machine), so tests/pc exercises it on a PC and the
# self-test runs the same code on the Pico.
#
# An edge is (time_us, level): the time since the recording started and the level the line
# went to. A DShot frame is a burst of edges; on a bidirectional line the ESC's reply follows
# it after a short pause, so edges closer together than BURST_GAP_US belong to one
# frame-and-reply cycle and its first edge is the frame start (a falling edge: the idle
# level of a bidirectional line is high).

BURST_GAP_US = 150      # a frame (27us) + reply pause (about 30us) + reply (about 30us) fit well inside
RUN_GAP_US = 5          # consecutive transmitter edges are at most 3.4us apart (two bit times at DSHOT600)


def analyze_edges(edges, gap_us=BURST_GAP_US, run_gap_us=RUN_GAP_US):
    """One pass over (time_us, level) edges - any iterable, so a recording never has to be held as a list.
    Returns (frame start times, count of bursts whose first edge was rising, frame lengths), where a
    frame length is the first edge to the last edge of the frame's own run (the ESC's reply follows
    after a longer pause and is not part of it)."""
    starts = []
    durations = []
    odd = 0
    last = None
    run_open = False
    run_last = 0
    start = 0
    for t, level in edges:
        if last is None or t - last >= gap_us:
            if last is not None:
                durations.append(run_last - start)
            start = t
            run_last = t
            run_open = True
            starts.append(t)
            if level != 0:
                odd += 1
        elif run_open:
            if t - last >= run_gap_us:
                run_open = False
            else:
                run_last = t
        last = t
    if last is not None:
        durations.append(run_last - start)
    return starts, odd, durations


def frame_starts(edges, gap_us=BURST_GAP_US):
    """(frame start times in us, count of bursts whose first edge was rising - anything but a frame start)."""
    starts, odd, _ = analyze_edges(edges, gap_us)
    return starts, odd


def frame_durations(edges, gap_us=BURST_GAP_US, run_gap_us=RUN_GAP_US):
    return analyze_edges(edges, gap_us, run_gap_us)[2]


def intervals(starts):
    return [b - a for a, b in zip(starts, starts[1:])]


def percentile(sorted_values, fraction):
    if not sorted_values:
        return None
    return sorted_values[min(len(sorted_values) - 1, int(fraction * len(sorted_values)))]


def spacing_stats(starts, period_us):
    """Summary of the spacing of frame starts against the period the loop is meant to hold."""
    gaps = intervals(starts)
    if not gaps:
        return {"frames": len(starts), "mean_us": None, "min_us": None, "max_us": None,
                "p99_dev_us": None, "short": 0, "long": 0}
    deviations = sorted(abs(g - period_us) for g in gaps)
    return {
        "frames": len(starts),
        "mean_us": (starts[-1] - starts[0]) / len(gaps),
        "min_us": min(gaps),
        "max_us": max(gaps),
        "p99_dev_us": percentile(deviations, 0.99),
        "short": sum(1 for g in gaps if g < period_us // 2),   # a frame far too soon after the last
        "long": sum(1 for g in gaps if g > period_us * 3 // 2),  # a frame missing or late by half a period
    }


def phase_offsets(reference_starts, other_starts):
    """For each frame start of another channel: microseconds after the latest start of the reference
    channel that is not later than it. Returns the sorted list (empty when nothing pairs up)."""
    offsets = []
    i = 0
    for t in other_starts:
        while i + 1 < len(reference_starts) and reference_starts[i + 1] <= t:
            i += 1
        if reference_starts and reference_starts[i] <= t:
            offsets.append(t - reference_starts[i])
    offsets.sort()
    return offsets
