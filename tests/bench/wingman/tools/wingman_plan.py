"""
wingman_plan.py - everything the wingman needs to know about the main Pico, read from the scenario file.

Design principle (owner, 2026-11-01): the wingman takes the main Pico's configuration from the scenario the
main Pico runs; no pin, speed or fixture is written into the wingman's own code. This module is the one place that reads the scenario
for the wingman. It works on the raw JSON dict and needs only the `motors` / `pin` / `bidirectional` / `dshot_speed` / `duration_ms` keys, so
it stays independent of the loader that runs on the main Pico.

Where each thing comes from:
  observer lines          motors[].pin, in list order (the order session_summary uses)
  line direction     motors[].bidirectional (the ESC mock answers only on bidirectional lines)
  start line         start_line.START_PIN (tests/bench/wingman/main_pico/start_line.py), the pin the main Pico drives
  DSHOT speed        top-level dshot_speed (selects the ESC mock's recorded replies)
  how long to wait   duration_ms (when the file has one) plus RUN_MARGIN_S
Wiring convention, written once here: the wingman's GPIO number is the main Pico's GPIO number, on every line, plus a shared ground.

A scenario the wingman cannot serve raises ValueError with a message that names the problem; callers print it and stop before any
hardware is touched.
"""

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "main_pico"))
sys.path.insert(0, str(HERE.parent / "shared"))
import esc_mock_format
import start_line

BOARD_DIR = HERE.parent / "board"
MAX_LINES = 4                       # the observer's state machines: one per line, one PIO block
KNOWN_SPEEDS = ("DSHOT300", "DSHOT600")
# The recorded ESC replies, per DSHOT speed. Only DSHOT600 was ever recorded, so the ESC mock serves nothing else (its timing was
# measured at DSHOT600 as well). A speed is added here together with a fixture recorded and timed at that speed.
SPEED_FIXTURES = {"DSHOT600": "esc_mock_replies_dshot600.bin"}
# Seconds the wingman waits beyond the scenario's own duration: the deploy to the main Pico (about 40 s, an estimate), the arming
# time-out (18 s) and slack. A scenario file with no duration_ms gets this margin alone.
RUN_MARGIN_S = 130


class MockPlan:
    def __init__(self, pins, lines, fixture, source):
        self.pins = pins            # GPIO numbers the ESC mock answers on
        self.lines = lines          # for each pin, which line of the fixture's recorded replies it plays
        self.fixture = fixture      # file name in tests/bench/wingman/board/
        self.source = source        # where the recorded replies came from (the fixture's .json file, written by esc_mock_fixture.py)

    def manifest_entry(self):
        """What session.json records about the replies the mock played."""
        return {"fixture": self.fixture, "source": self.source}


class WingmanPlan:
    def __init__(self, observer_pins, bidirectional, start_line_pin, dshot_speed, run_seconds, mock=None):
        self.observer_pins = observer_pins
        self.bidirectional = bidirectional
        self.start_line_pin = start_line_pin
        self.dshot_speed = dshot_speed
        self.run_seconds = run_seconds
        self.mock = mock


def plan_from_scenario(scenario, esc_mock=False):
    """The WingmanPlan for a scenario dict. esc_mock=True also plans the ESC mock (bidirectional lines only)."""
    motors = scenario.get("motors")
    if not isinstance(motors, list) or not motors:
        raise ValueError("the scenario lists no motors, so the wingman does not know which lines to watch")
    if len(motors) > MAX_LINES:
        raise ValueError("the scenario lists %d motors; the wingman watches at most %d lines" % (len(motors), MAX_LINES))
    pins, bidirectional = [], []
    for index, motor in enumerate(motors):
        pin = motor.get("pin") if isinstance(motor, dict) else None
        if not isinstance(pin, int) or isinstance(pin, bool):
            raise ValueError("motor %d has no whole-number pin" % index)
        if pin in pins:
            raise ValueError("motor %d uses pin %d, which an earlier motor already uses" % (index, pin))
        flag = motor.get("bidirectional")
        if not isinstance(flag, bool):
            raise ValueError("motor %d does not say whether it is bidirectional" % index)
        pins.append(pin)
        bidirectional.append(flag)
    if start_line.START_PIN in pins:
        raise ValueError("the start line (GPIO %d) is also a motor pin" % start_line.START_PIN)
    speed = scenario.get("dshot_speed")
    if speed not in KNOWN_SPEEDS:
        raise ValueError("dshot_speed is %r; the wingman needs one of %s" % (speed, ", ".join(KNOWN_SPEEDS)))
    duration = scenario.get("duration_ms", 0)
    if not isinstance(duration, int) or isinstance(duration, bool) or duration < 0:
        raise ValueError("duration_ms must be a whole number of milliseconds, got %r" % (duration,))
    mock = plan_mock(pins, bidirectional, speed) if esc_mock else None
    return WingmanPlan(pins, bidirectional, start_line.START_PIN, speed, duration / 1000 + RUN_MARGIN_S, mock)


def plan_mock(pins, bidirectional, speed):
    lines = [i for i, flag in enumerate(bidirectional) if flag]
    if not lines:
        raise ValueError("the ESC mock answers bidirectional motors, and this scenario has none")
    fixture = SPEED_FIXTURES.get(speed)
    if fixture is None:
        raise ValueError("the ESC mock has recorded replies for %s only, none for %s" % (", ".join(sorted(SPEED_FIXTURES)), speed))
    path = BOARD_DIR / fixture
    if not path.is_file():
        raise ValueError("the ESC mock's recorded replies %s are missing (scripts: esc_mock_fixture.py builds them)" % path)
    sidecar = path.with_suffix(".json")
    if not sidecar.is_file():
        raise ValueError("the ESC mock's recorded replies %s have no provenance file %s (esc_mock_fixture.py writes both)" % (path.name, sidecar.name))
    source = json.loads(sidecar.read_text(encoding="utf-8"))
    available = esc_mock_format.fixture_line_count(path.read_bytes()[:esc_mock_format.HEADER_SIZE])
    if lines[-1] >= available:
        raise ValueError("the ESC mock has recorded replies for %d lines; motor %d needs line %d" % (available, lines[-1], lines[-1]))
    return MockPlan([pins[i] for i in lines], lines, fixture, source)
