"""
run_optical_rpm_modes.py - does the DShot protocol variant change how fast the motor really turns?

Runs tests/bench/rpm_accuracy/run_optical_rpm_mode.py once per mode in
tests/bench/rpm_accuracy/modes/ (regular and bidirectional DShot, DSHOT300 and
DSHOT600, motor 1 at one fixed throttle), each through run_test.py so the Pico is
reset before every run, and compares the shaft RPM the optical sensor measures. Only the optical RPM is compared: regular DShot has no ESC eRPM.

Two rounds, the second in reverse order, so a slow drift (battery, motor warming up)
lands on every mode alike. Pass: the gap between the fastest and the slowest mode's mean
RPM is within TOLERANCE_PCT of the mean over all modes, and every run delivered a figure. Prints PASSED or FAILED: n and
exits non-zero on failure.

Run from project root:
  python scripts/run_optical_rpm_modes.py

Spins motor 1; make sure the disk is secure and nothing can touch it first.
"""

import re
import subprocess
import sys
from pathlib import Path

sys.stdout.reconfigure(line_buffering=True)

PYTHON = sys.executable
ROOT = Path(__file__).resolve().parents[1]
RUN_TEST = ROOT / "scripts" / "run_test.py"
MODES_DIR = ROOT / "tests" / "bench" / "rpm_accuracy" / "modes"
DEVICE_SCRIPT = "run_optical_rpm_mode.py"

TOLERANCE_PCT = 3.0      # the fastest and slowest mode may differ by this much of the all-mode mean
RUN_TIMEOUT_S = 180

RESULT_RE = re.compile(r"OPTICAL_RPM rpm=([0-9.]+) pulses=(\d+)")


def judge(runs, tolerance_pct=TOLERANCE_PCT):
    """
    runs: list of (mode label, optical RPM or None, why it has none). Returns (table lines,
    failures). The modes agree when the gap between the highest and the lowest mode mean is
    within tolerance_pct of the mean of the mode means (one outlier then fails the spread
    once, instead of dragging the others over a limit against a mean it skewed itself); a
    run without an RPM is itself a failure.
    """
    failures = [label + ": " + why for label, rpm, why in runs if rpm is None]
    by_mode = {}
    for label, rpm, _ in runs:
        if rpm is not None:
            by_mode.setdefault(label, []).append(rpm)
    lines = []
    if by_mode:
        means = {label: sum(v) / len(v) for label, v in by_mode.items()}
        overall = sum(means.values()) / len(means)
        lines.append("mean optical RPM per mode (all modes: %.0f):" % overall)
        for label, mean in means.items():
            off_pct = 100 * (mean - overall) / overall
            values = ", ".join("%.0f" % v for v in by_mode[label])
            lines.append("  %-24s %6.0f RPM  %+.2f%%  (runs: %s)" % (label, mean, off_pct, values))
        spread_pct = 100 * (max(means.values()) - min(means.values())) / overall
        lines.append("spread between the fastest and slowest mode: %.2f%% (limit %.1f%%)" % (spread_pct, tolerance_pct))
        if spread_pct > tolerance_pct:
            failures.append("the modes differ by %.2f%% (limit %.1f%%)" % (spread_pct, tolerance_pct))
    return lines, failures


def run_mode(mode_path):
    """One run on a freshly reset Pico. Returns (rpm or None, pulses or why it failed)."""
    try:
        result = subprocess.run(
            [PYTHON, str(RUN_TEST), DEVICE_SCRIPT, "--scenario", str(mode_path)],
            capture_output=True, encoding="utf-8", errors="replace", timeout=RUN_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        return None, "timed out after %ds" % RUN_TIMEOUT_S
    output = result.stdout + result.stderr
    match = RESULT_RE.search(output)
    if match:
        return float(match.group(1)), int(match.group(2))
    tail = [line for line in output.strip().splitlines() if line.strip()][-1:]
    return None, "no optical RPM printed (" + (tail[0].strip() if tail else "no output") + ")"


def main():
    modes = [(p.stem.replace("_", " "), p) for p in sorted(MODES_DIR.glob("*.json"))]
    if not modes:
        print("no mode files in " + str(MODES_DIR))
        sys.exit(1)
    runs = []
    for label, path in modes + list(reversed(modes)):
        print("%s ..." % label)
        rpm, detail = run_mode(path)
        if rpm is None:
            print("  failed: " + detail)
            runs.append((label, None, detail))
        else:
            print("  optical %.0f RPM (%d pulses)" % (rpm, detail))
            runs.append((label, rpm, ""))
    lines, failures = judge(runs)
    print()
    for line in lines:
        print(line)
    print()
    for line in failures:
        print("FAIL " + line)
    print("PASSED" if not failures else "FAILED: " + str(len(failures)))
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
