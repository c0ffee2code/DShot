"""
analyze_bidir_capture_log.py - decode the sampled capture of a scenario session (captures/<session>/capture.bin, written
by tests/bench/session_recorder/bidir_capture_sink.py) with the PC-side decoder in dshot_bidir_decode.py, and check it
against what the Pico decoded itself.

Run from project root:
  python scripts/analyze_bidir_capture_log.py [captures/<session>]

With no argument, analyzes the most recently pulled session under captures/.

What the log is: one record each time the Pico decoded a reply (every `decode_every`-th capture of a motor, the
"sampled" decode): the time, the four throttles, each motor's latest raw capture (21-bit reply frame with a 9-bit publish
token above it) and a flag byte naming the motor that was decoded and whether that decode failed on the Pico. The
Pico writes nothing per reply; every reply on the wire is in spy.bin. So this script checks the decoder:
each sampled word is decoded here, the per-motor tally must equal the Pico's, and every failed flag must agree with the
decode here. The words carry their token, so any of them can be found on the spy's recording.

Refuses a session whose meta does not say capture_kind=sampled (an older per-pass log is not read) and one whose outcome is
not completed. Judges no threshold: the session verdict is scripts/session_summary.py's, from the wire recording
(decision/ADR-008). Exits non-zero when the decode here disagrees with the Pico's tally or failed flags.
"""

import struct
import sys
from pathlib import Path
import session_files  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests" / "bench"))
sys.path.insert(0, str(ROOT / "driver"))

from dshot_bidir_decode import analyze_frame, FRAME_LENGTH_BITS
from decode_tally import DecodeTally
from scenario import load_scenario

# ticks_us, flags, throttle0..3, capture0..3 - must match
# tests/bench/session_recorder/bidir_capture_sink.py's BidirCaptureSink.RECORD_FMT
RECORD_FMT = "<IB4H4I"
RECORD_SIZE = struct.calcsize(RECORD_FMT)
FLAG_FAILED = 0x80
FLAG_MOTOR_MASK = 0x03
CAPTURE_KIND = "sampled"
# A capture is the 21-bit reply frame with the 9-bit publish token above it (driver/capture_mailbox.py, which needs
# MicroPython and so cannot be imported here)
FRAME_BITS = FRAME_LENGTH_BITS
FRAME_MASK = (1 << FRAME_BITS) - 1


def load_meta(session_dir):
    return session_files.load_device_meta(session_dir)


def load_records(session_dir):
    """The log as dicts: ticks_us, motor, device_failed, throttles, captures (4 each)."""
    raw = (session_dir / "capture.bin").read_bytes()
    if len(raw) % RECORD_SIZE:
        sys.exit(f"capture.bin is {len(raw)} bytes, not a whole number of {RECORD_SIZE}-byte records")
    records = []
    for i in range(len(raw) // RECORD_SIZE):
        fields = struct.unpack_from(RECORD_FMT, raw, i * RECORD_SIZE)
        records.append({"ticks_us": fields[0], "motor": fields[1] & FLAG_MOTOR_MASK,
                        "device_failed": bool(fields[1] & FLAG_FAILED),
                        "throttles": fields[2:6], "captures": fields[6:10]})
    return records


def most_recent_session(captures_dir):
    sessions = sorted(p for p in captures_dir.iterdir() if session_files.is_session(p) and session_files.is_evidence(p))
    if not sessions:
        sys.exit(f"No sessions found under {captures_dir} - run "
                 f"scripts/run_session.py or scripts/pull_captures.py first.")
    return sessions[-1]


def decode_failed(result):
    """The rule the Pico uses (run_scenario.py): no result, no reconstructable frame, or a CRC that does not match."""
    return result is None or result["full"] is None or not result["crc_ok"]


def new_motor_state():
    return {"sample": DecodeTally(), "erpms": [], "failures": [], "flag_mismatches": 0, "samples": 0}


def analyze_records(records, bidir_indices):
    per_motor = {i: new_motor_state() for i in bidir_indices}
    for record in records:
        state = per_motor.get(record["motor"])
        if state is None:
            continue
        capture = record["captures"][record["motor"]]
        frame = capture & FRAME_MASK
        result = analyze_frame(frame)
        state["samples"] += 1
        state["sample"].add(result)
        failed = decode_failed(result)
        if failed != record["device_failed"]:
            state["flag_mismatches"] += 1
        if failed:
            state["failures"].append({"ticks_us": record["ticks_us"], "token": capture >> FRAME_BITS, "frame": frame,
                                      "device_flagged": record["device_failed"]})
        elif result["erpm"] is not None:
            state["erpms"].append(result["erpm"])
    return per_motor


def compare_with_device(meta, per_motor, records):
    """Messages for every disagreement between this decode of the log and what the Pico wrote about the same run."""
    failures = []
    if "samples_written" in meta and int(meta["samples_written"]) != len(records):
        failures.append(f"the Pico wrote {meta['samples_written']} samples, the log holds {len(records)}")
    flagged = sum(1 for r in records if r["device_failed"])
    if "samples_failed" in meta and int(meta["samples_failed"]) != flagged:
        failures.append(f"the Pico counted {meta['samples_failed']} failed samples, {flagged} are flagged in the log")
    for index, state in per_motor.items():
        prefix = f"motor{index}_decode_"
        if prefix + "sampled" in meta:
            for name in ("sampled", "crc_ok", "crc_fail", "invalid"):
                on_device = int(meta[prefix + name])
                here = getattr(state["sample"], name)
                if on_device != here:
                    failures.append(f"motor {index}: the device tallied {name}={on_device}, the "
                                    f"same words decoded here give {here}")
        if state["flag_mismatches"]:
            failures.append(f"motor {index}: {state['flag_mismatches']} samples where the device's failed flag "
                            f"disagrees with the decode here")
    return failures


def median(values):
    ordered = sorted(values)
    return ordered[len(ordered) // 2]


def main():
    if len(sys.argv) > 1:
        session_dir = Path(sys.argv[1])
    else:
        session_dir = most_recent_session(Path("captures"))

    print(f"Session: {session_dir}")
    meta = load_meta(session_dir)
    outcome = meta.get("outcome", "unknown")
    print(f"Outcome: {outcome}")
    if meta.get("capture_kind") != CAPTURE_KIND:
        sys.exit(f"Refusing to analyze: capture_kind={meta.get('capture_kind')!r}, not {CAPTURE_KIND!r} - this script reads "
                 f"only the sampled log (an older per-pass log is not read).")
    if outcome != "completed":
        sys.exit(f"Refusing to analyze: outcome={outcome!r}, not 'completed' - "
                 f"this session's capture.bin is from an aborted run.")

    bidir_indices = [int(s) for s in meta.get("bidir_motor_indices", "").split(",") if s]
    missed = int(meta.get("captures_missed", 0))
    print(f"dshot_speed={meta.get('dshot_speed')} sample_every={meta.get('sample_every')} bidir_motors={bidir_indices}")

    scenario_path = session_dir / "scenario.json"
    if not scenario_path.is_file():
        sys.exit(f"No scenario.json in {session_dir}: scripts/run_session.py puts the scenario there; "
                 f"a session pulled by hand has none.")
    scenario = load_scenario(scenario_path, historic=True)   # a pulled session ran under its own day's rules

    records = load_records(session_dir)
    print(f"Samples: {len(records)} ({meta.get('samples_failed', '?')} flagged as failed by the device)  "
          f"passes with a new capture: {meta.get('capture_passes', '?')}  "
          f"captures published but never seen (device-side): {missed}")
    for index in bidir_indices:
        published = meta.get(f"motor{index}_captures_published")
        if published is not None:
            print(f"  motor {index}: {published} captures published")
    print()

    per_motor = analyze_records(records, bidir_indices)

    for index in bidir_indices:
        state = per_motor[index]
        tally = state["sample"]
        hardware = scenario.motors[index].hardware
        print(f"Motor {index}: {state['samples']} samples, CRC-valid {tally.crc_ok}/{tally.sampled} "
              f"({tally.crc_valid_pct():.1f}%)  crc_fail={tally.crc_fail} invalid={tally.invalid}")
        if hardware is not None:
            print(f"           {hardware.name}, {hardware.kv} KV, {hardware.magnets} magnets "
                  f"({hardware.pole_pairs} pole pairs), propeller: {hardware.propeller}")
        if state["erpms"]:
            erpms = state["erpms"]
            avg = sum(erpms) / len(erpms)
            print(f"           eRPM: min={min(erpms):.0f} median={median(erpms):.0f} "
                  f"max={max(erpms):.0f} avg={avg:.0f}")
            if hardware is not None:
                print(f"           shaft RPM (eRPM / {hardware.pole_pairs}): "
                      f"min={hardware.shaft_rpm(min(erpms)):.0f} "
                      f"median={hardware.shaft_rpm(median(erpms)):.0f} "
                      f"max={hardware.shaft_rpm(max(erpms)):.0f}")
        else:
            print("           eRPM: no valid decodes to report")
        for failure in state["failures"][:10]:
            print(f"           failed sample at {failure['ticks_us']} us: token {failure['token']} frame "
                  f"0x{failure['frame']:06x} (device flagged it: {failure['device_flagged']})")
        if len(state["failures"]) > 10:
            print(f"           ... and {len(state['failures']) - 10} more failed samples")

    print()
    if "max_loop_gap_us" in meta:
        print(f"Command loop: longest gap between update() calls {int(meta['max_loop_gap_us']) / 1000:.1f}ms")
    for index in bidir_indices:
        if f"motor{index}_decode_sampled" in meta:
            print(f"Motor {index} sampled decode (every {scenario.decode_every}th capture), replayed here: "
                  f"{per_motor[index]['sample'].summary()}")
    failures = compare_with_device(meta, per_motor, records)
    if failures:
        print("The decode here disagrees with the Pico's:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)

    print("The decode here agrees with the Pico's tally and failed flags.")


if __name__ == "__main__":
    main()
