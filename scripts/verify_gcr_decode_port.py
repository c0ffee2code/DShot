"""
verify_gcr_decode_port.py - diff driver/gcr_decode.py (the on-device port)
against scripts/dshot_bidir_decode.py (the PC-side reference) on identical
real hardware capture words, using each session's own tuned fixed-ratio
profile (driver/dshot_profiles.py's BIDIR_PROFILES). Catches a mechanical
porting bug for free, before MicroPython ever enters the picture.

Not a one-off: this is a permanent regression check, kept alongside both
files - re-run it whenever either changes.

Run from project root:
  python scripts/verify_gcr_decode_port.py [captures/<session> ...]

With no arguments, checks every session under captures/.

**Scope, since 2026-09-12 (see decision/ADR-002-bidirectional-dshot.md's
fixed-ratio RX sampling section):** this only checks sessions recorded at
a rate a live BIDIR_PROFILES entry is currently tuned for - both DSHOT300
and DSHOT600 now have a verified fixed ratio, and the on-device brute-force
sweep (estimate_bit_period) that used to back a "reference sweep vs port
sweep" comparison was deleted from driver/gcr_decode.py once both speeds
were retuned, so there is no port-side sweep left to compare against for
historical (pre-retune) sessions. This was a deliberate scope decision, not
an oversight: those sessions (~377k groups at the old 4MHz DSHOT300 /
8MHz DSHOT600 rates, plus the interim K=8/10/11 candidate sessions whose
rx_speed no longer matches the committed K=9 profiles) simply stop being
regression-checked going forward - they still exist on disk and remain
analyzable ad hoc via scripts/dshot_bidir_decode.py directly (which keeps
its own brute-force sweep permanently, specifically for this), just not
through this automated diff any more. Sessions in that position are
reported as SKIPPED (no tuned profile), not silently ignored and not
counted as failures.

A mismatch is only a real bug if it appears while the reference's own
crc_kind is "inverted" - i.e. the reference itself considers this reply
genuinely valid. Any disagreement on a group the reference does NOT
consider crc_kind=="inverted" (a "plain" hit, or fully invalid) is
expected divergence, not a porting bug, and is counted separately rather
than flagged.

A session that declares a bidirectional motor but yields zero groups is
flagged as an anomaly (ESC never replied), not silently reported [OK] -
iter_groups only yields a group when a motor's words are non-all-zero, so a
session where the ESC never replied at all naturally has zero groups to
check and would otherwise pass with "0 groups, 0 mismatches" looking
identical to a real, clean result. See capture_session.py's
expects_bidir_groups(). A session whose own on-device runtime check (the
reply failsafe - see tests/harness/run_scenario.py's
check_reply_failsafe - or any other expect-block violation) already
tripped and recorded outcome=failed is a known, self-diagnosed failure and
is skipped outright rather than flagged - the anomaly path is specifically
for a zero-group session that still claims outcome=completed, i.e. one the
failsafe should have caught but didn't (or one recorded before the
failsafe existed). Anomaly detection happens independent of whether a
tuned profile exists for that session's rate - zero real replies is worth
flagging regardless of whether the rate is even checkable.

Each session's own meta.txt records the rx_clock_hz it was actually
captured at - read per-session, not assumed globally, since sessions
recorded before and after a rx_speed retune are mixed together under
captures/. Session loading (including the two different on-disk record
formats and the historical rx_clock_hz fallback) is shared with
scripts/tally_period_cycles.py via scripts/capture_session.py - see that
module's docstring for why sharing this matters (a hardcoded-format bug
here previously went undetected on every main-scenario-path session for
exactly this reason).
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "driver"))

from dshot_bidir_decode import analyze_capture as reference_analyze
from gcr_decode import analyze_capture as port_analyze
from dshot_profiles import BIDIR_PROFILES
from capture_session import load_meta, iter_groups, rx_clock_hz_for, dshot_speed_for, expects_bidir_groups

_FLOAT_FIELDS = ("period_cycles", "period_us", "bitrate_bps", "erpm")
_EXACT_FIELDS = ("full", "crc_ok", "crc_kind", "data12")
_FLOAT_TOLERANCE = 1e-9


def floats_agree(a, b):
    if a is None or b is None:
        return a is None and b is None
    return abs(a - b) < _FLOAT_TOLERANCE


def _compare(ref, port):
    """
    Returns (ok, expected_divergence) - the latter True whenever the
    reference itself doesn't consider this reply genuinely valid
    (crc_kind != "inverted"): a "plain" CRC hit the port correctly
    rejects, or fully invalid data where the two implementations' bit-
    period estimates are free to disagree since neither answer is "the"
    real period.
    """
    if ref is None or port is None:
        return (ref is None and port is None), False

    if ref["crc_kind"] != "inverted":
        return True, True

    for field in _EXACT_FIELDS:
        if ref[field] != port[field]:
            return False, False
    for field in _FLOAT_FIELDS:
        if not floats_agree(ref[field], port[field]):
            return False, False
    return True, False


def check_one_fixed(words, rx_clock_hz, expected_ratio, ratio_tolerance):
    """
    Diff the fixed-ratio decode path (estimate_bit_period_fixed) on both
    port and reference - see decision/ADR-002-bidirectional-dshot.md's
    fixed-ratio RX sampling section. Only meaningful for a session
    recorded at a rate a BIDIR_PROFILES entry has actually been tuned for
    - verify_session skips any session whose recorded rate has no live
    expected_ratio rather than calling this.
    """
    ref = reference_analyze(words, rx_clock_hz, expected_ratio, ratio_tolerance)
    port = port_analyze(words, rx_clock_hz, expected_ratio, ratio_tolerance)
    return _compare(ref, port)


def _tuned_profile_for(dshot_speed, rx_clock_hz):
    """
    Returns (expected_ratio, ratio_tolerance) if dshot_speed has a live
    BIDIR_PROFILES entry AND that entry's rx_speed matches what this
    session was actually recorded at (a session predating a retune must
    not be checked against a newer profile's ratio) - otherwise None.
    """
    profile = BIDIR_PROFILES.get(dshot_speed)
    if profile is None or profile["expected_ratio"] is None:
        return None
    if profile["rx_speed"] != rx_clock_hz:
        return None
    return profile["expected_ratio"], profile["ratio_tolerance"]


def verify_session(session_dir):
    """
    Returns (total, mismatches, expected_divergences, status) where status
    is one of:
      "skipped_failed" - outcome=failed, a runtime check already flagged this run
      "anomaly"        - declares a bidir motor, claims outcome=completed, zero groups
      "untuned"        - no live BIDIR_PROFILES entry matches this session's rate
      "ok"             - checked; total/mismatches/expected_divergences are meaningful
    """
    meta = load_meta(session_dir)

    # A session whose own reply failsafe already tripped (see
    # tests/harness/run_scenario.py's check_reply_failsafe) is a known,
    # self-diagnosed failure - skip it rather than counting it against the
    # regression bar. The alternative (still flagging it) would make this
    # check permanently red for as long as any failed run sits in
    # captures/, and a permanently-red check gets ignored.
    if meta.get("outcome") == "failed":
        return (0, [], 0, "skipped_failed")

    groups = list(iter_groups(session_dir, meta))

    # A session that declares a bidir motor, claims outcome=completed, yet
    # yielded zero groups is an anomaly the reply failsafe should have
    # caught (or predates it) - not a clean pass, and not simply "untuned"
    # even if its rate also has no live profile. See capture_session.py's
    # expects_bidir_groups() docstring.
    if not groups:
        return (0, [], 0, "anomaly" if expects_bidir_groups(meta) else "ok")

    rx_clock_hz = rx_clock_hz_for(meta)
    dshot_speed = dshot_speed_for(meta)
    tuned = _tuned_profile_for(dshot_speed, rx_clock_hz) if dshot_speed is not None else None
    if tuned is None:
        return (0, [], 0, "untuned")

    expected_ratio, ratio_tolerance = tuned
    total = 0
    mismatches = []
    expected_divergences = 0
    for words in groups:
        total += 1
        ok, expected = check_one_fixed(words, rx_clock_hz, expected_ratio, ratio_tolerance)
        if expected:
            expected_divergences += 1
        elif not ok:
            mismatches.append(words)

    return (total, mismatches, expected_divergences, "ok")


def main():
    args = sys.argv[1:]
    if args:
        sessions = [Path(a) for a in args]
    else:
        captures_dir = Path("captures")
        sessions = sorted(p for p in captures_dir.iterdir() if p.is_dir())

    if not sessions:
        sys.exit("No capture sessions found - run scripts/pull_captures.py first.")

    grand_total = 0
    grand_mismatches = 0
    grand_expected = 0
    grand_skipped_failed = 0
    grand_untuned = 0
    anomaly_sessions = []
    for session_dir in sessions:
        total, mismatches, expected, status = verify_session(session_dir)

        if status == "skipped_failed":
            grand_skipped_failed += 1
            print(f"{session_dir}: SKIPPED (outcome=failed - a runtime check already flagged this run)")
            continue

        if status == "untuned":
            grand_untuned += 1
            print(f"{session_dir}: SKIPPED (no tuned profile for this session's recorded rate)")
            continue

        grand_total += total
        grand_mismatches += len(mismatches)
        grand_expected += expected
        if status == "anomaly":
            anomaly_sessions.append(session_dir)
            status_label = "ANOMALY: NO GROUPS - bidir motor declared, outcome=completed, but never replied"
        else:
            status_label = "OK" if not mismatches else "MISMATCH"
        print(f"{session_dir}: {total} groups, {len(mismatches)} mismatches, "
              f"{expected} expected divergences [{status_label}]")
        for words in mismatches[:5]:
            print(f"    mismatch: {[hex(w) for w in words]}")

    print()
    print(f"Total: {grand_total} groups checked, {grand_mismatches} real mismatches, "
          f"{grand_expected} expected polarity divergences")
    if grand_skipped_failed:
        print(f"Skipped: {grand_skipped_failed} session(s) with outcome=failed (reply failsafe already tripped)")
    if grand_untuned:
        print(f"Skipped: {grand_untuned} session(s) with no tuned profile for their recorded rate")
    if anomaly_sessions:
        print(f"\n{len(anomaly_sessions)} session(s) declared a bidir motor, claim outcome=completed, "
              f"but yielded ZERO groups (predates the reply failsafe, or the failsafe failed to catch it):")
        for session_dir in anomaly_sessions:
            print(f"    {session_dir}")
    if grand_mismatches or anomaly_sessions:
        sys.exit(1)


if __name__ == "__main__":
    main()
