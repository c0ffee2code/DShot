"""
analyze_bidir_stress_log.py — decode a pulled W4/W5 stress-test session's
sampled raw captures (see tests/test_bidir_rx_stress.py /
tests/harness/stress_capture_sink.py) into a real CRC-valid rate per
channel/phase, using the shared decode algorithm in dshot_bidir_decode.py.

Run from project root:
  python scripts/analyze_bidir_stress_log.py [captures/<session>]

With no argument, analyzes the most recently pulled session under captures/.

Unlike analyze_bidir_capture_log.py, this does not depend on
tests/harness/scenario.py's schema-validated 4-motor scenario.json - a
stress-test session's scenario.json is a small, free-form provenance
record (channels, throttle, target frames, starvation settings), not a
throttle-profile scenario, so this reads it as plain JSON instead.
"""

import json
import struct
import sys
from pathlib import Path

from dshot_bidir_decode import analyze_capture

# ticks_us, channel, phase, word_count, w0..w3 - must match
# tests/harness/stress_capture_sink.py's _RECORD_FMT
_RECORD_FMT = "<IBBB4I"
_RECORD_SIZE = struct.calcsize(_RECORD_FMT)

PHASE_NAMES = {0: "arm", 1: "saturation", 2: "post_resume", 3: "boundary_partial"}


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
    return [struct.unpack_from(_RECORD_FMT, raw, i * _RECORD_SIZE) for i in range(count)]


def most_recent_session(captures_dir):
    sessions = sorted(p for p in captures_dir.iterdir() if p.is_dir())
    if not sessions:
        sys.exit(f"No sessions found under {captures_dir} - run scripts/pull_captures.py first.")
    return sessions[-1]


def main():
    if len(sys.argv) > 1:
        session_dir = Path(sys.argv[1])
    else:
        session_dir = most_recent_session(Path("captures"))

    print(f"Session: {session_dir}")
    meta = load_meta(session_dir)
    print(f"Outcome: {meta.get('outcome', 'unknown')}")

    provenance = json.loads((session_dir / "scenario.json").read_text())
    dshot_speed = provenance.get("dshot_speed")
    rx_clock_hz = provenance.get("rx_clock_hz")
    if rx_clock_hz is None:
        # Sessions predating this harness recording its own rx_clock_hz
        # (see tests/test_bidir_rx_stress.py's provenance dict) - all such
        # sessions on disk today are DSHOT300 runs at that era's rx_speed.
        if dshot_speed == 2_400_000:
            rx_clock_hz = 4_000_000
        else:
            sys.exit(f"No known rx_clock_hz for dshot_speed={dshot_speed} "
                      "(session predates rx_clock_hz provenance and isn't DSHOT300)")
    print(f"dshot_speed={dshot_speed} rx_clock_hz={rx_clock_hz} "
          f"channels={provenance.get('channels')} "
          f"starvation_enabled={provenance.get('starvation_enabled')}")
    print()

    records = load_records(session_dir)
    print(f"Sampled records: {len(records)}")
    print()

    by_channel_phase = {}
    for ticks_us, channel, phase, word_count, w0, w1, w2, w3 in records:
        key = (channel, phase)
        stats = by_channel_phase.setdefault(key, {"total": 0, "crc_valid": 0, "no_edges": 0})
        stats["total"] += 1
        if word_count != 4:
            continue  # partial group - nothing to decode
        result = analyze_capture([w0, w1, w2, w3], rx_clock_hz)
        if result is None:
            stats["no_edges"] += 1
        elif result["crc_ok"]:
            stats["crc_valid"] += 1

    for (channel, phase), stats in sorted(by_channel_phase.items()):
        pct = (stats["crc_valid"] / stats["total"] * 100) if stats["total"] else 0.0
        print(f"Channel {channel} / {PHASE_NAMES.get(phase, phase)}: "
              f"CRC-valid {stats['crc_valid']}/{stats['total']} ({pct:.1f}%)  "
              f"no_edges={stats['no_edges']}")

    print()
    print("Full meta.txt counters:")
    for key, value in sorted(meta.items()):
        print(f"  {key}={value}")


if __name__ == "__main__":
    main()
