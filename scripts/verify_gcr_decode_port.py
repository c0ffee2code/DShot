"""
verify_gcr_decode_port.py - diff driver/gcr_decode.py (the on-device port)
against scripts/dshot_bidir_decode.py (the PC-side reference) on identical
real hardware capture words. Catches a mechanical porting bug for free,
before MicroPython ever enters the picture.

Not a one-off: this is a permanent regression check, kept alongside both
files - re-run it whenever either changes.

Run from project root:
  python scripts/verify_gcr_decode_port.py [captures/<session> ...]

With no arguments, checks every session under captures/.

Two deliberate, expected differences between the two modules:

1. CRC polarity - the reference accepts plain or inverted, the port
   accepts only inverted (see both files' docstrings).
2. Bit-period search range - the reference sweeps the full 6-16 cycle
   range, the port sweeps a narrower measured band (see gcr_decode.py's
   estimate_bit_period docstring). Both sweeps agree on every real
   CRC-valid capture measured so far, but on data neither module can
   actually decode, they can land on different "best" answers to the
   same ill-posed question - there's no real period to recover from
   garbage, so there's nothing to agree on.

So a mismatch is only a real bug if it appears while the reference's own
crc_kind is "inverted" - i.e. the reference itself considers this reply
genuinely valid. Any disagreement on a group the reference does NOT
consider crc_kind=="inverted" (a "plain" hit, or fully invalid) is
expected divergence, not a porting bug, and is counted separately rather
than flagged.
"""

import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "driver"))

from dshot_bidir_decode import analyze_capture as reference_analyze
from gcr_decode import analyze_capture as port_analyze

_RECORD_FMT = "<IBBB4I"
_RECORD_SIZE = struct.calcsize(_RECORD_FMT)

_FLOAT_FIELDS = ("period_cycles", "period_us", "bitrate_bps", "erpm")
_EXACT_FIELDS = ("full", "crc_ok", "crc_kind", "data12")
_FLOAT_TOLERANCE = 1e-9


def load_records(session_dir):
    raw = (session_dir / "capture.bin").read_bytes()
    count = len(raw) // _RECORD_SIZE
    return [struct.unpack_from(_RECORD_FMT, raw, i * _RECORD_SIZE) for i in range(count)]


def floats_agree(a, b):
    if a is None or b is None:
        return a is None and b is None
    return abs(a - b) < _FLOAT_TOLERANCE


def check_one(words, rx_clock_hz):
    """
    Returns (ok, expected_divergence) - the latter True whenever the
    reference itself doesn't consider this reply genuinely valid
    (crc_kind != "inverted"): a "plain" CRC hit the port correctly
    rejects, or fully invalid data where the reference's and port's
    differing bit-period search ranges (see module docstring) are free to
    disagree with each other since neither answer is "the" real period.
    """
    ref = reference_analyze(words, rx_clock_hz)
    port = port_analyze(words, rx_clock_hz)

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


def verify_session(session_dir, rx_clock_hz):
    records = load_records(session_dir)
    total = 0
    mismatches = []
    expected_divergences = 0
    for record in records:
        word_count = record[3]
        if word_count != 4:
            continue
        words = list(record[4:8])
        total += 1
        ok, expected = check_one(words, rx_clock_hz)
        if expected:
            expected_divergences += 1
        elif not ok:
            mismatches.append(words)
    return total, mismatches, expected_divergences


def main():
    args = sys.argv[1:]
    if args:
        sessions = [Path(a) for a in args]
    else:
        captures_dir = Path("captures")
        sessions = sorted(p for p in captures_dir.iterdir() if p.is_dir())

    if not sessions:
        sys.exit("No capture sessions found - run scripts/pull_captures.py first.")

    rx_clock_hz = 4_000_000  # DSHOT300's BIDIR_PROFILES entry - see driver/dshot_pio.py
    grand_total = 0
    grand_mismatches = 0
    grand_expected = 0
    for session_dir in sessions:
        total, mismatches, expected = verify_session(session_dir, rx_clock_hz)
        grand_total += total
        grand_mismatches += len(mismatches)
        grand_expected += expected
        status = "OK" if not mismatches else "MISMATCH"
        print(f"{session_dir}: {total} groups, {len(mismatches)} mismatches, "
              f"{expected} expected divergences [{status}]")
        for words in mismatches[:5]:
            print(f"    mismatch: {[hex(w) for w in words]}")

    print()
    print(f"Total: {grand_total} groups checked, {grand_mismatches} real mismatches, "
          f"{grand_expected} expected polarity divergences")
    if grand_mismatches:
        sys.exit(1)


if __name__ == "__main__":
    main()
