"""
wingman_run.py - run the wingman for one test: the observer records the ESC signal lines, and with --esc-mock the wingman also
answers the main Pico's frames with recorded ESC replies.

The observer records the ESC signal wires while the start line (GPIO 15, the same number on both boards) is high and streams the
edges to this PC over USB (tests/bench/wingman/board/observer.py); observer_edge_log.py reads them. No SD card is involved: the stream
is the record. Run the recorded test on the main Pico afterwards or in parallel; this script waits up to --wait seconds for the
start line. scripts/run_session.py calls this for every session.

  python tests/bench/wingman/tools/wingman_run.py --scenario <file>              # wait for the test to raise the start line, save the stream
  python tests/bench/wingman/tools/wingman_run.py --scenario <file> --out some.bin
  python tests/bench/wingman/tools/wingman_run.py --scenario <file> --esc-mock   # ESC disconnected: the wingman plays the ESC as well

The scenario is the one the main Pico runs: the wingman reads the pins, the lines' directions, the DSHOT speed
and the run length from it (wingman_plan.py); nothing about the main Pico is written into this script.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent))
import observer_edge_log
import wingman_board
import wingman_plan

sys.stdout.reconfigure(line_buffering=True)

ROOT = wingman_board.ROOT
OBSERVER_FILES = [("tests/bench/wingman/board/observer.py", "observer.py")]
ESC_MOCK_FILES = [("tests/bench/wingman/board/esc_mock.py", "esc_mock.py"),
                  ("tests/bench/wingman/shared/esc_mock_format.py", "esc_mock_format.py")]     # plus the plan's fixture


def tup(numbers):
    """A Python tuple literal for the board's program."""
    return "(" + "".join("%d," % n for n in numbers) + ")"


def observer_code(plan):
    """The board program that only spies."""
    return "import observer; observer.run(pins=%s, sync_pin=%d, wait_ms=%d)" % (tup(plan.observer_pins), plan.start_line_pin, plan.run_seconds * 1000)


def esc_mock_code(plan):
    """The board program with the ESC mock: the mock starts first and plays only the plan's bidirectional lines; the observer must not
    take the pins back (configure_pins=False), so every other spied pin and the start line are made inputs here; the mock stops in a
    finally, and the observer's first tick checks that the mock's pins are still the mock's."""
    mock = plan.mock
    others = [pin for pin in plan.observer_pins if pin not in mock.pins] + [plan.start_line_pin]
    lines = ["import esc_mock, observer",
             "from machine import Pin",
             "mock = esc_mock.start(pins=%s, fixture=%r, lines=%s)" % (tup(mock.pins), mock.fixture, tup(mock.lines)),
             "try:"]
    lines += ["    Pin(%d, Pin.IN)" % pin for pin in others]
    lines += ["    observer.run(pins=%s, sync_pin=%d, wait_ms=%d, configure_pins=False, tick=esc_mock.guard(%s))" % (
                  tup(plan.observer_pins), plan.start_line_pin, plan.run_seconds * 1000, tup(mock.pins)),
              "finally:",
              "    mock.stop()"]
    return "\n".join(lines)


def stream(port, code, wait_s, out_path):
    """Run code in the wingman's raw REPL and collect the binary stream until its end packet."""
    link = serial.Serial(port, 115200, timeout=0.2)
    try:
        link.write(b"\x03\x03")
        time.sleep(0.3)
        link.reset_input_buffer()
        link.write(b"\x01")
        time.sleep(0.2)
        link.read_all()
        link.write(code.encode() + b"\x04")
        reader = observer_edge_log.StreamReader()
        raw = bytearray()
        deadline = time.time() + wait_s
        while not reader.done and time.time() < deadline:
            chunk = link.read(65536)
            if chunk:
                raw.extend(chunk)
                try:
                    reader.feed(chunk)
                except observer_edge_log.StreamError as error:
                    reader.damage = str(error)
                    break
        link.write(b"\x03")
    finally:
        link.close()
    Path(out_path).write_bytes(bytes(raw))
    if getattr(reader, "damage", None):
        print("STREAM DAMAGED: %s (%d bytes saved to %s)" % (reader.damage, len(raw), out_path))
        sys.exit(2)
    if not reader.done:
        tail = bytes(raw[-300:])
        sys.exit("no end packet after %d s (%d bytes saved to %s). Last bytes: %r" % (wait_s, len(raw), out_path, tail))
    return reader


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", required=True, help="the scenario the main Pico runs: the wingman reads the wiring and settings from it")
    parser.add_argument("--out", default=None)
    parser.add_argument("--session", default=None, help="session folder name under captures/ to put signal_observation.bin in (default: the current time)")
    parser.add_argument("--no-upload", action="store_true")
    parser.add_argument("--esc-mock", action="store_true",
                        help="also play the ESC on the wingman (tests/bench/wingman/): the ESC must be disconnected from the signal lines")
    args = parser.parse_args()

    try:
        plan = wingman_plan.plan_from_scenario(json.loads(Path(args.scenario).read_text(encoding="utf-8")), args.esc_mock)
    except (OSError, ValueError) as error:
        sys.exit("the wingman cannot serve %s: %s" % (args.scenario, error))
    port = wingman_board.find_port()
    if port is None:
        sys.exit("The wingman Pico (ID %s) is not connected." % wingman_board.WINGMAN_ID)
    print("Wingman on %s" % port)
    wingman_board.check_board(port)
    code = esc_mock_code(plan) if args.esc_mock else observer_code(plan)
    session = args.session or time.strftime("%Y-%m-%d_%H-%M-%S")
    out = Path(args.out) if args.out else ROOT / "captures" / session / observer_edge_log.OBSERVATION_FILE
    out.parent.mkdir(parents=True, exist_ok=True)
    wait = plan.run_seconds + 10
    if not args.no_upload:
        files = OBSERVER_FILES
        if args.esc_mock:
            files = files + ESC_MOCK_FILES + [("tests/bench/wingman/board/" + plan.mock.fixture, plan.mock.fixture)]
        wingman_board.upload(port, files)
        wingman_board.reset(port)
        time.sleep(3)
        port = wingman_board.find_port() or port
    print("Recording (waiting for the start line)...")
    try:
        reader = stream(port, code, wait, out)
    finally:
        if args.esc_mock:
            wingman_board.reset(port)     # the mock must not outlive the run: a board left mocking would answer on the signal lines
    info = observer_edge_log.parse_end(reader.end)
    problems = observer_edge_log.reconcile(reader.channels, info)
    if problems:
        info["valid"] = False
        for line in problems:
            print("STREAM LOSS: " + line)
    print("Saved %s" % out)
    print("start: %s" % reader.start)
    print("end:   %s" % reader.end)
    sys.exit(0 if info["valid"] else 1)


if __name__ == "__main__":
    main()
