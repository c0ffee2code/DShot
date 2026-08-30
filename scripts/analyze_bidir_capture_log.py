"""
analyze_bidir_capture_log.py — decode a pulled raw capture session (see
scripts/pull_captures.py) into CRC-valid rate and eRPM history, using the
shared decode algorithm in dshot_bidir_decode.py.

Run from project root:
  python scripts/analyze_bidir_capture_log.py [captures/<session>]

With no argument, analyzes the most recently pulled session under captures/.

This is what replaces the on-device live-decode soak tests entirely: raw
records are captured on-device (tests/test_bidir_rx_capture.py) and written
to SD unbatched; all GCR decoding happens here, offline, where there's no
MicroPython stack/CPU budget to worry about.
"""

import struct
import sys
from pathlib import Path

from dshot_bidir_decode import analyze_capture

# ticks_us, throttle, word0, word1, word2, word3 - must match
# tests/bidir_capture_sink.py's _RECORD_FMT
_RECORD_FMT = "<IH4I"
_RECORD_SIZE = struct.calcsize(_RECORD_FMT)


def load_meta(session_dir):
    meta = {}
    for line in (session_dir / "meta.txt").read_text().splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        meta[key.strip()] = value.strip()
    return meta


def load_records(session_dir):
    raw = (session_dir / "capture.bin").read_bytes()
    count = len(raw) // _RECORD_SIZE
    records = []
    for i in range(count):
        ticks_us, throttle, w0, w1, w2, w3 = struct.unpack_from(
            _RECORD_FMT, raw, i * _RECORD_SIZE
        )
        records.append((ticks_us, throttle, w0, w1, w2, w3))
    return records


def most_recent_session(captures_dir):
    sessions = sorted(p for p in captures_dir.iterdir() if p.is_dir())
    if not sessions:
        sys.exit(f"No sessions found under {captures_dir} - run "
                  f"scripts/pull_captures.py first.")
    return sessions[-1]


def main():
    if len(sys.argv) > 1:
        session_dir = Path(sys.argv[1])
    else:
        session_dir = most_recent_session(Path("captures"))

    print(f"Session: {session_dir}")
    meta = load_meta(session_dir)
    rx_clock_hz = int(meta["rx_clock_hz"])
    print(f"dshot_speed={meta.get('dshot_speed')} rx_clock_hz={rx_clock_hz}")

    records = load_records(session_dir)
    print(f"Records: {len(records)}")
    print()

    crc_valid = 0
    fail_streak = 0
    longest_fail_streak = 0
    last_success_ticks_us = None
    largest_gap_us = 0
    erpm_min = None
    erpm_max = None
    erpm_sum = 0.0
    erpm_count = 0

    for ticks_us, throttle, w0, w1, w2, w3 in records:
        result = analyze_capture([w0, w1, w2, w3], rx_clock_hz)
        ok = result is not None and result["crc_ok"]
        if ok:
            crc_valid += 1
            fail_streak = 0
            if last_success_ticks_us is not None:
                gap = (ticks_us - last_success_ticks_us) & 0xFFFFFFFF
                if gap > largest_gap_us:
                    largest_gap_us = gap
            last_success_ticks_us = ticks_us
            erpm = result["erpm"]
            if erpm is not None:
                erpm_count += 1
                erpm_sum += erpm
                if erpm_min is None or erpm < erpm_min:
                    erpm_min = erpm
                if erpm_max is None or erpm > erpm_max:
                    erpm_max = erpm
        else:
            fail_streak += 1
            if fail_streak > longest_fail_streak:
                longest_fail_streak = fail_streak

    total = len(records)
    rate = (crc_valid / total * 100) if total else 0.0
    print(f"CRC-valid: {crc_valid}/{total} ({rate:.1f}%)")
    print(f"Longest consecutive CRC-failure streak: {longest_fail_streak}")
    print(f"Largest gap between successful decodes: {largest_gap_us / 1000:.1f}ms")
    if erpm_count:
        avg_erpm = erpm_sum / erpm_count
        print(f"eRPM: min={erpm_min:.0f} max={erpm_max:.0f} avg={avg_erpm:.0f} "
              f"(spread={erpm_max - erpm_min:.0f})")
    else:
        print("eRPM: no valid decodes to report")


if __name__ == "__main__":
    main()
