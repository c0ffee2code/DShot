"""
esc_mock_fixture.py - build the ESC mock's replay fixture from a recorded session.

The ESC mock (tests/bench/wingman/) replays real ESC replies. This script takes the replies the wingman
recorded in a session's signal_observation.bin and keeps, per line, a run of consecutive real-eRPM replies from the middle of the
run (the motors are up to speed there). Every kept reply is decoded again with the driver's own decoder and must have a
right CRC and a real speed; the script stops otherwise.

  python tests/bench/wingman/tools/esc_mock_fixture.py captures/<session>                   # writes tests/bench/wingman/board/esc_mock_replies_dshot600.bin (and the .json beside it)
  python tests/bench/wingman/tools/esc_mock_fixture.py captures/<session> --out some.bin --frames 1024 --after-ms 5000
"""

import argparse
import json
import statistics
import sys
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(ROOT / "driver"))
sys.path.insert(0, str(ROOT / "tests" / "bench" / "wingman" / "shared"))
import esc_mock_format
import gcr_decode
import observer_edge_log

DEFAULT_OUT = ROOT / "tests" / "bench" / "wingman" / "board" / "esc_mock_replies_dshot600.bin"
DEFAULT_FRAMES = 1024
DEFAULT_AFTER_MS = 5000          # recording time from which replies are taken (ticks of 0.1 us)


def real_replies(edge_list, after_ms):
    """The 21-bit reply frames, in time order, of the replies after `after_ms` that are CRC-valid and carry a real
    speed (not AM32's not-running word). edge_list: one line's edges from observer_edge_log.edges()."""
    out = []
    for record in observer_edge_log.analyse(edge_list):
        reply = record["reply"]
        if reply is None or record["start"] < after_ms * 10_000:
            continue
        if reply["crc_ok"] and not reply["not_running"] and reply["erpm"]:
            out.append(reply["frame"])
    return out


def provenance(session_folder, frames, after_ms, medians, built=None):
    """Where the recorded replies came from, for the sidecar file next to the fixture (and, through the plan, for session.json)."""
    session_folder = Path(session_folder)
    manifest = {}
    if (session_folder / "session.json").is_file():
        manifest = json.loads((session_folder / "session.json").read_text(encoding="utf-8"))
    return {"source_session": session_folder.resolve().name, "source_scenario": manifest.get("scenario"),
            "source_git_commit": manifest.get("git_commit"), "source_load": manifest.get("load"),
            "built": built or date.today().isoformat(), "replies_per_line": frames, "taken_after_ms": after_ms,
            "median_erpm_per_line": [round(m) for m in medians]}


def build(observer_bytes, frames, after_ms):
    """(fixture bytes, [median eRPM per line]) from a signal_observation.bin."""
    _, channels, _ = observer_edge_log.parse_stream(observer_bytes)
    lines, medians = [], []
    for channel in sorted(channels):
        replies = real_replies(observer_edge_log.edges(channels[channel]), after_ms)
        if len(replies) < frames:
            raise SystemExit("line %d: only %d real replies after the cut, need %d" % (channel, len(replies), frames))
        kept = replies[:frames]
        erpms = []
        for frame in kept:
            result = gcr_decode.analyze_frame(frame)
            if not result["crc_ok"] or result["not_running"] or not result["erpm"]:
                raise SystemExit("line %d: a kept reply does not decode as a real speed" % channel)
            erpms.append(result["erpm"])
        lines.append(kept)
        medians.append(statistics.median(erpms))
    return esc_mock_format.pack_fixture(lines), medians


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", help="a captures/<session> folder holding signal_observation.bin")
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    parser.add_argument("--frames", type=int, default=DEFAULT_FRAMES, help="replies kept per line (default 1024)")
    parser.add_argument("--after-ms", type=int, default=DEFAULT_AFTER_MS, help="take replies from this recording time on")
    args = parser.parse_args()
    observer_file = Path(args.session) / observer_edge_log.OBSERVATION_FILE
    data, medians = build(observer_file.read_bytes(), args.frames, args.after_ms)
    Path(args.out).write_bytes(data)
    sidecar = Path(args.out).with_suffix(".json")
    sidecar.write_text(json.dumps(provenance(args.session, args.frames, args.after_ms, medians), indent=2) + "\n", encoding="utf-8")
    print("wrote %s (where the replies came from)" % sidecar)
    print("wrote %s: %d bytes, %d lines x %d replies" % (args.out, len(data), len(medians), args.frames))
    for index, median in enumerate(medians):
        print("  line %d: median eRPM %.0f" % (index, median))


if __name__ == "__main__":
    main()
