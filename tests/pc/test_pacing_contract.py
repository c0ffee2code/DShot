# The one command loop is paced (MotorGroup.UPDATE_INTERVAL_US, held by Core1Runner's
# absolute deadline). Before 2026-10-19 the library default was "as fast as possible"
# (interval 0) and many tests and scenarios quietly ran on it, so results from different
# runs were not comparable. These checks read the source files, so the old habit cannot
# come back unnoticed: every Core1Runner on the bench is built on the group's own period,
# nothing sets a literal period, and the library default is not zero.

import re
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import fakes  # noqa: E402,F401

ROOT = Path(__file__).resolve().parents[2]
BENCH_SOURCES = [p for p in (ROOT / "tests" / "bench").rglob("*.py") if p.name != "core1_runner.py"]
RUNNER_CALL = re.compile(r"Core1Runner\(([^)]*)\)")


class PacingContractTest(unittest.TestCase):
    def test_every_bench_core1_runner_uses_the_groups_own_period(self):
        calls = 0
        for path in BENCH_SOURCES:
            for line in path.read_text(encoding="utf-8").splitlines():
                code = line.split("#", 1)[0]
                match = RUNNER_CALL.search(code)
                if match:
                    calls += 1
                    with self.subTest(path.name + ": " + code.strip()):
                        self.assertIn("UPDATE_INTERVAL_US", match.group(1))
        self.assertGreater(calls, 0)

    def test_no_bench_file_or_driver_file_sets_a_zero_period(self):
        pattern = re.compile(r"UPDATE_INTERVAL_US\s*=\s*0\b|interval_us\s*=\s*0\b")
        for path in BENCH_SOURCES + list((ROOT / "driver").glob("*.py")):
            for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
                code = line.split("#", 1)[0]
                with self.subTest(path.name + ":" + str(number)):
                    self.assertIsNone(pattern.search(code), line.strip())

    def test_no_scenario_file_sets_its_own_period(self):
        for path in (ROOT / "tests" / "bench").rglob("*.json"):
            with self.subTest(path.name):
                self.assertNotIn("core1_interval_us", path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
