"""
analyze_bidir_capture_log.py — decode a pulled scenario capture session (see
scripts/pull_captures.py) into per-motor CRC-valid rate, eRPM history, and
throttle-trace verification, using the shared decode algorithm in
dshot_bidir_decode.py and the same scenario.py/throttle_profile.py the
device itself validated the scenario against.

Run from project root:
  python scripts/analyze_bidir_capture_log.py [captures/<session>]

With no argument, analyzes the most recently pulled session under captures/.

Refuses to compute anything beyond outcome/record count on a session whose
meta.txt says outcome != completed - mirrors Flight-Benchy's gate.py failing
fast before verdict.py ever runs on a bad flight. Then re-checks the
scenario's own "expect" thresholds PC-side (belt-and-suspenders against the
on-device check) and exits non-zero if any is missed.
"""

import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests" / "harness"))

from dshot_bidir_decode import analyze_capture
from scenario import load_scenario

# ticks_us, throttle0..3, then one 4-word GCR group per motor - must match
# tests/harness/bidir_capture_sink.py's _RECORD_FMT
_RECORD_FMT = "<I4H16I"
_RECORD_SIZE = struct.calcsize(_RECORD_FMT)

_ZERO_GROUP = (0, 0, 0, 0)


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
        sys.exit(f"No sessions found under {captures_dir} - run "
                  f"scripts/pull_captures.py first.")
    return sessions[-1]


def _motor_words(record, motor_index):
    base = 5 + motor_index * 4
    return record[base:base + 4]


def _new_motor_state():
    return {
        "crc_valid": 0, "completed_groups": 0,
        "fail_streak": 0, "longest_fail_streak": 0,
        "last_success_ticks_us": None, "largest_gap_us": 0,
        "erpm_min": None, "erpm_max": None, "erpm_sum": 0.0, "erpm_count": 0,
    }


def analyze_records(records, bidir_indices, rx_clock_hz):
    per_motor = {i: _new_motor_state() for i in bidir_indices}
    overall_largest_gap_us = 0
    last_ticks_us = None

    for record in records:
        ticks_us = record[0]

        if last_ticks_us is not None:
            gap = (ticks_us - last_ticks_us) & 0xFFFFFFFF
            if gap > overall_largest_gap_us:
                overall_largest_gap_us = gap
        last_ticks_us = ticks_us

        for index in bidir_indices:
            words = _motor_words(record, index)
            if words == _ZERO_GROUP:
                continue  # no telemetry group completed for this motor this record

            state = per_motor[index]
            state["completed_groups"] += 1
            result = analyze_capture(list(words), rx_clock_hz)
            ok = result is not None and result["crc_ok"]
            if ok:
                state["crc_valid"] += 1
                state["fail_streak"] = 0
                if state["last_success_ticks_us"] is not None:
                    gap = (ticks_us - state["last_success_ticks_us"]) & 0xFFFFFFFF
                    if gap > state["largest_gap_us"]:
                        state["largest_gap_us"] = gap
                state["last_success_ticks_us"] = ticks_us
                erpm = result["erpm"]
                if erpm is not None:
                    state["erpm_count"] += 1
                    state["erpm_sum"] += erpm
                    if state["erpm_min"] is None or erpm < state["erpm_min"]:
                        state["erpm_min"] = erpm
                    if state["erpm_max"] is None or erpm > state["erpm_max"]:
                        state["erpm_max"] = erpm
            else:
                state["fail_streak"] += 1
                if state["fail_streak"] > state["longest_fail_streak"]:
                    state["longest_fail_streak"] = state["fail_streak"]

    return per_motor, overall_largest_gap_us


def check_expect(expect, dropped, largest_gap_us, total_records, elapsed_ms, per_motor):
    failures = []

    max_dropped = expect.get("max_dropped")
    if max_dropped is not None and dropped > max_dropped:
        failures.append(f"dropped={dropped} > max_dropped={max_dropped}")

    max_gap_ms = expect.get("max_gap_ms")
    if max_gap_ms is not None and largest_gap_us / 1000 > max_gap_ms:
        failures.append(f"largest_gap={largest_gap_us / 1000:.1f}ms > max_gap_ms={max_gap_ms}")

    min_rate = expect.get("min_record_rate_hz")
    if min_rate is not None and elapsed_ms > 0:
        rate = total_records / (elapsed_ms / 1000)
        if rate < min_rate:
            failures.append(f"record_rate={rate:.1f}/s < min_record_rate_hz={min_rate}")

    for motor_index_str, min_pct in expect.get("min_crc_valid_pct", {}).items():
        motor_index = int(motor_index_str)
        state = per_motor.get(motor_index)
        completed = state["completed_groups"] if state else 0
        actual_pct = (state["crc_valid"] / completed * 100) if state and completed else 0.0
        if actual_pct < min_pct:
            failures.append(f"motor {motor_index} crc_valid={actual_pct:.1f}% < "
                             f"min_crc_valid_pct={min_pct}%")

    return failures


def main():
    if len(sys.argv) > 1:
        session_dir = Path(sys.argv[1])
    else:
        session_dir = most_recent_session(Path("captures"))

    print(f"Session: {session_dir}")
    meta = load_meta(session_dir)
    outcome = meta.get("outcome", "unknown")
    print(f"Outcome: {outcome}")
    if outcome != "completed":
        sys.exit(f"Refusing to analyze: outcome={outcome!r}, not 'completed' - "
                  f"this session's capture.bin is from an aborted run.")

    rx_clock_hz = int(meta["rx_clock_hz"])
    bidir_indices = [int(s) for s in meta.get("bidir_motor_indices", "").split(",") if s]
    dropped = int(meta.get("dropped", 0))
    print(f"dshot_speed={meta.get('dshot_speed')} rx_clock_hz={rx_clock_hz} "
          f"bidir_motors={bidir_indices}")

    scenario = load_scenario(session_dir / "scenario.json")

    records = load_records(session_dir)
    total = len(records)
    print(f"Records: {total}  dropped(device-side): {dropped}")
    print()

    per_motor, overall_largest_gap_us = analyze_records(records, bidir_indices, rx_clock_hz)

    for index in bidir_indices:
        state = per_motor[index]
        completed = state["completed_groups"]
        pct = (state["crc_valid"] / completed * 100) if completed else 0.0
        print(f"Motor {index}: CRC-valid {state['crc_valid']}/{completed} ({pct:.1f}%)  "
              f"longest_fail_streak={state['longest_fail_streak']}  "
              f"largest_gap={state['largest_gap_us'] / 1000:.1f}ms")
        if state["erpm_count"]:
            avg = state["erpm_sum"] / state["erpm_count"]
            print(f"           eRPM: min={state['erpm_min']:.0f} max={state['erpm_max']:.0f} "
                  f"avg={avg:.0f} (spread={state['erpm_max'] - state['erpm_min']:.0f})")
        else:
            print("           eRPM: no valid decodes to report")

    print()
    print(f"Overall largest gap between any two records: {overall_largest_gap_us / 1000:.1f}ms")

    print()
    failures = check_expect(scenario.expect, dropped, overall_largest_gap_us, total,
                             scenario.duration_ms, per_motor)
    if failures:
        print("expect thresholds NOT met:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)

    print("All expect thresholds met." if scenario.expect else "No expect thresholds declared.")


if __name__ == "__main__":
    main()
