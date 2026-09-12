"""
tally_period_cycles.py - measure the real samples-per-bit distribution
(period_cycles) across every CRC-valid capture in one or more sessions.

Used to validate a candidate rx_speed retune (see decision/
ADR-002-bidirectional-dshot.md's fixed-ratio RX sampling section): after
flashing a candidate K and pulling a fresh capture session, run this
against that session for min/max/mean/std of the real measured
period_cycles - the same methodology used once already this project
(798-capture tally, 2026-09-08) to justify narrowing gcr_decode.py's
sweep, now made reusable per candidate.

Run from project root:
  python scripts/tally_period_cycles.py captures/<session> [captures/<session> ...]

With no arguments, tallies every session under captures/.

Only counts groups the reference decoder itself considers genuinely valid
(crc_kind == "inverted") - a garbage capture has no "real" period to
contribute to this measurement.

Uses scripts/capture_session.py for session loading, same as
scripts/verify_gcr_decode_port.py - see that module's docstring for why
sharing this matters (a hardcoded-format bug here previously produced a
false "0 CRC-valid groups" result on a real, healthy capture).
"""

import sys
from pathlib import Path

from dshot_bidir_decode import analyze_capture
from capture_session import load_meta, iter_groups, rx_clock_hz_for


def tally_session(session_dir):
    meta = load_meta(session_dir)
    rx_clock_hz = rx_clock_hz_for(meta)
    periods = []
    for words in iter_groups(session_dir, meta):
        result = analyze_capture(words, rx_clock_hz)
        if result is not None and result["crc_kind"] == "inverted":
            periods.append(result["period_cycles"])
    return periods


def summarize(periods):
    n = len(periods)
    if n == 0:
        return None
    mean = sum(periods) / n
    variance = sum((p - mean) ** 2 for p in periods) / n
    return {"n": n, "min": min(periods), "max": max(periods),
            "mean": mean, "std": variance ** 0.5}


def _print_summary(label, summary):
    if summary is None:
        print(f"{label}: 0 CRC-valid groups")
        return
    print(f"{label}: n={summary['n']} min={summary['min']:.4f} "
          f"max={summary['max']:.4f} mean={summary['mean']:.4f} "
          f"std={summary['std']:.4f}")


def main():
    args = sys.argv[1:]
    if args:
        sessions = [Path(a) for a in args]
    else:
        captures_dir = Path("captures")
        sessions = sorted(p for p in captures_dir.iterdir() if p.is_dir())

    if not sessions:
        sys.exit("No capture sessions found - run scripts/pull_captures.py first.")

    all_periods = []
    for session_dir in sessions:
        periods = tally_session(session_dir)
        all_periods.extend(periods)
        _print_summary(str(session_dir), summarize(periods))

    print()
    _print_summary("Overall", summarize(all_periods))


if __name__ == "__main__":
    main()
