"""
capture_session.py - shared raw-capture session loading for PC-side
tooling (scripts/verify_gcr_decode_port.py, scripts/tally_period_cycles.py).

Two raw record formats exist on disk, and a script that hardcodes one
will silently misparse sessions recorded in the other - discovered
2026-09-10 when scripts/tally_period_cycles.py (which had copied
verify_gcr_decode_port.py's hardcoded format) reported "0 CRC-valid
groups" on a freshly captured session. That turned out to be a parsing
bug, not a real decode failure - and verify_gcr_decode_port.py had the
same latent bug the whole time, quietly checking almost nothing on every
main-scenario-path session already in captures/: its garbage-parsed
"groups" never matched the reference's own crc_kind=="inverted", so they
were silently counted as expected divergences rather than flagged. The
check never failed, but for those sessions it also was not actually
checking anything.

1. Stress-harness format ("<IBBB4I", tests/harness/stress_capture_sink.py):
   (ticks_us, channel, phase, word_count, w0, w1, w2, w3) - one record per
   sampled tick; word_count==4 means this record IS a complete group.
   meta.txt for these sessions has no "record_fmt" field at all (predates
   this format even needing one, since only this one format existed at
   the time) - default to this format when record_fmt is absent.

2. Main scenario-capture format ("<I4H16I",
   tests/harness/bidir_capture_sink.py / scenario_runner.py): (ticks_us,
   throttle0..3, motor0_w0..w3, motor1_w0..w3, motor2_w0..w3,
   motor3_w0..w3) - one record per completed telemetry group from ANY
   motor, carrying all 4 motors' throttle plus whichever motor(s) actually
   completed a group this tick; a non-completing motor's word slot is
   explicitly zeroed for that record (see scenario_runner.py's loop()), so
   a real capture is never all-zero and "all zero" reliably means "this
   motor didn't complete this tick." Which motor indices are even
   bidirectional-capable is recorded in meta.txt's bidir_motor_indices
   (comma-separated).
"""

import struct

_STRESS_FMT = "<IBBB4I"
_MAIN_FMT = "<I4H16I"

# Fallback only for sessions recorded before their harness tracked its own
# rx_clock_hz (see the stress format's meta.txt shape above) - all such
# sessions on disk today are DSHOT300 runs at that era's rx_speed.
DEFAULT_RX_CLOCK_HZ = 4_000_000


def load_meta(session_dir):
    meta = {}
    meta_path = session_dir / "meta.txt"
    if not meta_path.exists():
        return meta
    for line in meta_path.read_text().splitlines():
        if "=" not in line:
            continue
        key, _, value = line.partition("=")
        meta[key] = value
    return meta


def rx_clock_hz_for(meta):
    return int(meta.get("rx_clock_hz", DEFAULT_RX_CLOCK_HZ))


def dshot_speed_for(meta):
    return int(meta["dshot_speed"]) if "dshot_speed" in meta else None


def expects_bidir_groups(meta):
    """
    True if this session declares at least one bidirectional motor (stress
    format has no such field and is assumed bidir - it predates any other
    kind of session). Used to distinguish a session that legitimately has
    zero groups (TX-only, nothing to decode) from one where a bidir motor
    was declared but never actually produced a real reply - see this
    module's docstring for why that distinction went unnoticed for so long.
    """
    if "record_fmt" not in meta:
        return True
    indices_str = meta.get("bidir_motor_indices", "")
    return any(s.strip() != "" for s in indices_str.split(","))


def iter_groups(session_dir, meta=None):
    """
    Yields a list of 4 raw 32-bit ints for every genuinely completed bidir
    capture group in this session, regardless of which on-disk record
    format it was recorded in.
    """
    if meta is None:
        meta = load_meta(session_dir)
    record_fmt = meta.get("record_fmt", _STRESS_FMT)
    raw = (session_dir / "capture.bin").read_bytes()
    record_size = struct.calcsize(record_fmt)
    count = len(raw) // record_size

    if record_fmt == _STRESS_FMT:
        for i in range(count):
            record = struct.unpack_from(record_fmt, raw, i * record_size)
            if record[3] == 4:
                yield list(record[4:8])
        return

    if record_fmt == _MAIN_FMT:
        indices_str = meta.get("bidir_motor_indices", "")
        motor_indices = [int(s) for s in indices_str.split(",") if s.strip() != ""]
        for i in range(count):
            record = struct.unpack_from(record_fmt, raw, i * record_size)
            for motor_index in motor_indices:
                offset = 5 + motor_index * 4
                words = list(record[offset:offset + 4])
                if any(words):
                    yield words
        return

    raise ValueError("unknown record_fmt: " + repr(record_fmt))
