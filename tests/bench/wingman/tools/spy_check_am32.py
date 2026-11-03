"""
spy_check_am32.py - does what the spy recorded on the wire match what AM32's firmware needs?

  python tests/bench/wingman/tools/spy_check_am32.py captures/<session>/spy.bin [--unidirectional] [--from-first-frame]

Each check is derived from AM32's source (pinned commit 55c96847, the same one specification/AM32.md cites:
Src/dshot.c computeDshotDMA, Src/signal.c detectInput / checkDshot / the frame-time window, Src/main.c
arming gate, signal-loss timeout and stuck-rotor protection) and tested against the edges the wingman
recorded, so it reads the wire and not our own decoder's opinion of it. Statuses: PASS (the recording
satisfies the source's condition), FAIL (it contradicts it), WARN (inside the condition but with little
margin, or not what the source predicts), INFO (a measurement to compare, nothing to fail) and N/A.

What it needs from the recording:
 - checks of the INPUT side (what AM32 sees of our frames) need no ESC and run on every recording;
 - checks of the REPLY side need replies on the wire, so they are N/A on a unidirectional recording.
--from-first-frame says the recording began when our frames began (the start line went high before
arming, as tests/bench/wingman/tools/wingman_run.py's real runs do); without it the checks that count from the first frame the ESC
saw (arming gate, the 101-frame latch) can only say INFO.

The reply-side checks have so far been exercised only on synthetic recordings built with this file's own assumptions
(tests/pc/test_spy_check_am32.py); the first recording with an ESC attached, compared with the main Pico's own decoded
replies, is what validates them. The input-side checks need no such validation: they read our own frames.

The tick-based limits assume the capture clock of one MCU family (--mcu, default F051: 5.333 MHz during speed
classification); the bench's target is not known, and other families have other clocks (AM32.md section 13 lists the
reply bit times per family; the loop timer is 1 MHz x 51 us on F051, E230 and F421 alike, so the 1.02 s arming gate does
not depend on the family).
"""

import argparse
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import spy_edge_log
import spy_stats

UNIT_US = 0.1
# Capture-clock tick while the speed is classified, per MCU family (microseconds): ic_timer_prescaler is set from the CPU clock in each
# family's Mcu/*/Src/IO.c - F051: 48 MHz / (8+1) = 5.333 MHz; F421: 120 MHz / (17+1) = 6.667 MHz; E230: 72 MHz / (12+1) = 5.538 MHz.
MCU_TICK_US = {"F051": 6 / 32, "F421": 18 / 120, "E230": 13 / 72}
IC_TICK_US = MCU_TICK_US["F051"]


def set_mcu(name):
    """Choose the MCU family whose capture clock the tick-based limits use (the bench's is not known; F051 is the default)."""
    global IC_TICK_US
    IC_TICK_US = MCU_TICK_US[name]
RESOLUTION_US = 0.1                 # the spy's edge resolution
CLASSIFY_FRAMES = 8                 # the speed is classified on the first frames AM32 captures
LATCH_FRAMES = 101                  # high_pin_count > 100
ARMING_GATE_MS = 1020               # armed_timeout_count > 20000 at ~19.6 kHz
ZERO_FRAMES_FOR_ARMING = 31         # zero_input_count > 30
SIGNAL_LOSS_ARMED_MS = 500
SIGNAL_LOSS_DISARMED_MS = 2000
TUNE_GAP_MS = (250, 400)            # playInputTune: three 100 ms beeps with interrupts disabled
REBOOT_GAP_MS = 550                 # startup tune 600 ms, bootloader, then the 101-frame latch again
STUCK_TIMEOUT_MS = 22.5
# DSHOT600 reply bit times by MCU family (half of the DSHOT300 figures in AM32.md section 13), microseconds
REPLY_BIT_US = {"F051/F031": 1.2917, "F421": 1.2833, "F415/V203": 1.3333, "G431": 1.3625, "L431": 1.3875,
                "E230": 1.4028, "G071/G031": 1.4531}

PASS, FAIL, WARN, INFO, NA = "PASS", "FAIL", "WARN", "INFO", "N/A"


class Result:
    def __init__(self, name, status, detail, source, values=None, esc_only=False):
        self.name = name
        self.status = status
        self.detail = detail
        self.source = source
        self.values = values or {}         # the numbers behind the verdict, for tests/bench/wingman/tools/spy_am32_table.py
        self.esc_only = esc_only           # about the real ESC's own behaviour: does not apply when the ESC mock played the ESC


def span(record):
    edges = record["tx_edges"]
    return edges[31][0] - edges[0][0]


def check_bit_threshold(frames):
    name = "AM32's own bit threshold reads every bit the way our decoder does"
    source = "dshot.c computeDshotDMA: bit = (edge 2i+1 - edge 2i) > (edge 31 - edge 0) / 32"
    mismatches = 0
    ones = []
    zeros = []
    thresholds = []
    for r in frames:
        edges = r["tx_edges"]
        threshold = span(r) / 32
        thresholds.append(threshold)
        for i in range(16):
            pulse = edges[2 * i + 1][0] - edges[2 * i][0]
            if (pulse > threshold) != bool(r["tx"]["bits"][i]):
                mismatches += 1
            (ones if r["tx"]["bits"][i] else zeros).append(pulse)
    mean_threshold = sum(thresholds) / len(thresholds) * UNIT_US
    mean_one = sum(ones) / len(ones) * UNIT_US if ones else 0
    mean_zero = sum(zeros) / len(zeros) * UNIT_US if zeros else 0
    margin_one = mean_one - mean_threshold if ones else None
    margin_zero = mean_threshold - mean_zero if zeros else None
    detail = ("%d frames; AM32's threshold %.2f-%.2f us; mean 1-bit pulse %.3f us (%.2f us above it), mean 0-bit pulse %.3f us "
              "(%.2f us below it); one pulse reads +-%.1f us on this spy, so a single frame's own margin is only known to that") % (
        len(frames), min(thresholds) * UNIT_US, max(thresholds) * UNIT_US, mean_one, margin_one or 0,
        mean_zero, margin_zero or 0, RESOLUTION_US)
    values = {"frames": len(frames), "mismatches": mismatches, "threshold_us": mean_threshold, "margin_one_us": margin_one,
              "margin_zero_us": margin_zero, "mean_one_us": mean_one, "mean_zero_us": mean_zero}
    if mismatches:
        return Result(name, FAIL, "%d bits would be read differently by AM32 than by our decoder (%s)" % (mismatches, detail), source, values)
    margins = [m for m in (margin_one, margin_zero) if m is not None]
    if margins and min(margins) < RESOLUTION_US:
        return Result(name, WARN, "margin under the spy's resolution: " + detail, source, values)
    return Result(name, PASS, detail, source, values)


def check_input_crc(frames, unidirectional):
    name = "every frame of ours carries the CRC polarity AM32 expects"
    source = ("dshot.c: plain CRC; once latched bidirectional the inverted one (checkCRC = ~checkCRC + 16). Before the latch "
              "(about 101 frames) AM32 rejects our inverted CRC, which does not block arming (AM32.md section 5)")
    bad = [r for r in frames if not r["tx"]["crc_ok"]]
    kind = "plain" if unidirectional else "inverted"
    values = {"frames": len(frames), "bad": len(bad)}
    if bad:
        return Result(name, FAIL, "%d of %d frames do not carry the %s CRC" % (len(bad), len(frames), kind), source, values)
    return Result(name, PASS, "all %d frames carry the %s CRC" % (len(frames), kind), source, values)


def check_frame_window(frames):
    name = "every frame length is inside the window AM32 learns from its first zero-throttle frames"
    source = ("signal.c: averages 8 zero-throttle frames (counted from the ESC's own first frames after speed detection; all early frames "
              "of ours are identical), then accepts avg +-1/16; dshot.c: frame length = edge 31 - edge 0")
    leading = []
    for r in frames:
        if r["tx"]["throttle"] != 0:
            break
        leading.append(r)
    if len(leading) < 14:
        return Result(name, NA, "fewer than 14 zero-throttle frames at the start of the recording", source)
    average = sum(span(r) for r in leading[6:14]) / 8
    low, high = average - average / 16, average + average / 16
    worst = max(frames, key=lambda r: abs(span(r) - average))
    outside = [r for r in frames if not low < span(r) < high]
    detail = "learned %.2f us, window %.2f-%.2f us; frames range %.2f-%.2f us, the largest deviation %.1f%% (throttle %d)" % (
        average * UNIT_US, low * UNIT_US, high * UNIT_US, min(span(r) for r in frames) * UNIT_US,
        max(span(r) for r in frames) * UNIT_US, abs(span(worst) - average) / average * 100, worst["tx"]["throttle"])
    values = {"frames": len(frames), "outside": len(outside), "learned_us": average * UNIT_US,
              "worst_pct": abs(span(worst) - average) / average * 100, "limit_pct": 100 / 16}
    if outside:
        return Result(name, FAIL, "%d frames outside (%s)" % (len(outside), detail), source, values)
    return Result(name, PASS, detail, source, values)


def frame_speed(frames):
    """600 or 300, from the median bit period on the wire (1.67 us or 3.33 us)."""
    periods = sorted(p for r in frames[:200] for p in r["tx"]["periods"])
    return 600 if periods[len(periods) // 2] * UNIT_US < 2.5 else 300


def check_speed_band(frames):
    speed = frame_speed(frames)
    name = "the first frames classify as DSHOT%d (shortest edge interval, average interval)" % speed
    source = ("signal.c detectInput/checkDshot: smallest of the intervals between edges 0-30 in 1..3 ticks and sum/32 < 60 ticks for "
              "DSHOT600, in 4..8 ticks and sum/32 < 100 for DSHOT300 (F051: 0.1875 us per tick - holds only if the reply bit time check "
              "lands on F051/F031)")
    low_ticks, high_ticks, average_limit = (1, 3.999, 60) if speed == 600 else (4, 8, 100)
    smallest = []
    averages = []
    # AM32 classifies the speed once, on the first frames it captures (inputSet stays 1 afterwards), so only the first frames count
    for r in frames[:CLASSIFY_FRAMES]:
        times = [e[0] for e in r["tx_edges"]]
        intervals = [times[j] - times[j - 1] for j in range(1, 31)]
        smallest.append(min(intervals) * UNIT_US)
        averages.append(sum(intervals) / 32 * UNIT_US / IC_TICK_US)
    worst = max(smallest)
    best = min(smallest)
    detail = ("first %d frames (throttle %s): shortest interval %.1f-%.1f us (%.1f-%.1f ticks; the band is %s ticks), "
              "average %.1f-%.1f ticks (under %d)") % (
        len(smallest), "0" if all(r["tx"]["throttle"] == 0 for r in frames[:CLASSIFY_FRAMES]) else "mixed", best, worst,
        best / IC_TICK_US, worst / IC_TICK_US, "%g to under 4" % low_ticks if speed == 600 else "4 to 8",
        min(averages), max(averages), average_limit)
    low_us, high_us = low_ticks * IC_TICK_US, high_ticks * IC_TICK_US
    values = {"speed": speed, "shortest_min_us": best, "shortest_max_us": worst, "band_low_us": low_us, "band_high_us": high_us,
              "average_ticks": max(averages), "average_limit_ticks": average_limit}
    if worst - RESOLUTION_US >= high_us or best + RESOLUTION_US <= low_us or max(averages) >= average_limit:
        return Result(name, FAIL, detail, source, values)
    if worst + RESOLUTION_US >= high_us or best - RESOLUTION_US <= low_us:
        return Result(name, WARN, detail + ": within the spy's resolution of the band edge", source, values)
    return Result(name, PASS, detail, source, values)


def check_throttle_values(frames):
    name = "throttle values are 0 or 48 and up (1-47 are DShot commands)"
    source = "dshot.c: 1-47 become a pending command with throttle 0; main.c: the motor starts at input >= 47"
    sent = [r["tx"]["throttle"] for r in frames]
    commands = [v for v in sent if 0 < v < 48]
    detail = "values sent: %s" % sorted(set(sent))
    values = {"command_frames": len(commands), "values": sorted(set(sent))}
    if commands:
        return Result(name, FAIL, "%d frames carry a command value (%s)" % (len(commands), sorted(set(commands))), source, values)
    return Result(name, PASS, detail, source, values)


def check_signal_loss(frames):
    name = "no gap between frames long enough to reset the ESC"
    source = "main.c: signal-loss reset after 500 ms armed, 2 s disarmed (AM32.md section 15)"
    starts = [r["start"] for r in frames]
    gaps = [(b - a) * UNIT_US / 1000 for a, b in zip(starts, starts[1:])]
    if not gaps:
        return Result(name, NA, "fewer than two frames", source)
    worst = max(gaps)
    detail = "largest gap between frames %.1f ms (%d gaps above 5 ms)" % (worst, sum(1 for g in gaps if g > 5))
    values = {"max_gap_ms": worst, "gaps_over_5ms": sum(1 for g in gaps if g > 5), "limit_ms": SIGNAL_LOSS_ARMED_MS}
    if worst >= SIGNAL_LOSS_DISARMED_MS:
        return Result(name, FAIL, detail + ": past even the disarmed 2 s limit", source, values)
    if worst >= SIGNAL_LOSS_ARMED_MS:
        return Result(name, WARN, detail + ": would reset an armed ESC", source, values)
    return Result(name, PASS, detail, source, values)


def check_arming_gate(frames, from_first_frame):
    name = "zero throttle long enough before the first throttle, for AM32 to arm"
    source = "main.c arming gate: 1.02 s of zero input with more than 30 zero frames counted; any nonzero input resets the count (AM32.md section 16)"
    nonzero = next((i for i, r in enumerate(frames) if r["tx"]["throttle"] != 0), None)
    if nonzero is None:
        return Result(name, NA, "no nonzero throttle in the recording", source)
    elapsed = (frames[nonzero]["start"] - frames[0]["start"]) * UNIT_US / 1000
    detail = "first nonzero throttle (%d) after %d zero frames and %.0f ms of recording" % (
        frames[nonzero]["tx"]["throttle"], nonzero, elapsed)
    enough = elapsed >= ARMING_GATE_MS and nonzero >= ZERO_FRAMES_FOR_ARMING
    if enough:
        return Result(name, PASS, detail + ": meets the lower bounds of 1020 ms and 31 frames (necessary, not sufficient: where the ESC "
                      "was in its boot cycle when our frames began is unknown)", source)
    if from_first_frame:
        return Result(name, FAIL, detail + ": the gate needs 1020 ms and 31 frames", source)
    return Result(name, INFO, detail + ": less than the gate needs, fine only if the ESC had been receiving frames before the recording began", source)


def reply_rows(records):
    return spy_stats.frame_table(records)


def check_latch(rows, from_first_frame):
    name = "first reply comes only after the bidirectional latch (101 frames)"
    source = "dshot.c: dshot_telemetry latches when the line is high after more than 100 frames while disarmed; replies start only then"
    first = next((i for i, r in enumerate(rows) if r["reply"] != "none"), None)
    if first is None:
        return Result(name, WARN, "no reply in %d frames" % len(rows), source)
    detail = "first reply at frame %d (%.0f ms after our first frame)" % (first, rows[first]["t_ms"] - rows[0]["t_ms"])
    values = {"first_reply_frame": first, "first_reply_ms": rows[first]["t_ms"] - rows[0]["t_ms"]}
    if first >= LATCH_FRAMES:
        return Result(name, PASS if from_first_frame else INFO,
                      detail + ": consistent with the source (101 frames is a lower bound, not proof of when the ESC began counting; a first "
                      "reply later than about 700 ms would fit an ESC still in its startup tune - a hypothesis, AM32.md sections 1-2)", source, values)
    if from_first_frame:
        return Result(name, WARN, detail + ": earlier than the source allows, so the ESC had already latched before this recording", source, values)
    return Result(name, INFO, detail + ": the ESC may have been latched before the recording began", source, values)


def check_reply_coverage(rows):
    name = "an answer to every frame once latched, except the gaps the source explains"
    source = "dshot.c/signal.c: a reply goes out for every captured frame; the arming tune (main.c, sounds.c) and a reboot silence the ESC"
    first = next((i for i, r in enumerate(rows) if r["reply"] != "none"), None)
    if first is None:
        return Result(name, NA, "no replies", source)
    tail = rows[first:]
    answered = [r for r in tail if r["reply"] != "none"]
    episodes = []
    silences = []
    explained = 0
    previous = None
    for r in tail:
        if r["reply"] == "none":
            continue
        if previous is not None:
            gap = r["t_ms"] - previous["t_ms"]
            if gap > 5:
                if TUNE_GAP_MS[0] <= gap <= TUNE_GAP_MS[1]:
                    kind = "arming tune (about 300 ms, interrupts disabled)"
                elif gap >= REBOOT_GAP_MS:
                    kind = "reboot, or an arming tune of several cells (one tune per cell, AM32.md section 16)"
                else:
                    kind = "NOT explained by the source"
                episodes.append((previous["t_ms"], gap, kind))
                skipped = sum(1 for x in tail if previous["t_ms"] < x["t_ms"] < r["t_ms"])
                if kind != "NOT explained by the source":
                    explained += skipped
                    silences.append((previous["t_ms"], r["t_ms"]))
        previous = r
    missing = len(tail) - len(answered) - explained
    coverage = (len(tail) - missing) / len(tail) * 100
    lines = ["%.1f%% of %d frames answered" % (coverage, len(tail))]
    if len(silences) > 1:
        lines.append("%d long silences (starting at %s ms): a reboot makes the ESC arm again, so a second silence about 1 s later would point "
                     "to a reboot, a single one to an arming tune" % (len(silences), ", ".join("%.0f" % a for a, _ in silences)))
    for start, gap, kind in episodes[:6]:
        lines.append("silence of %.0f ms after %.0f ms: %s" % (gap, start, kind))
    detail = "; ".join(lines)
    values = {"frames": len(tail), "coverage_pct": coverage, "unanswered": missing, "episodes": episodes}
    if any(kind.startswith("NOT") for _, _, kind in episodes) or coverage < 99.5:
        return Result(name, WARN, detail, source, values)
    return Result(name, PASS, detail, source, values)


def check_reply_integrity(records):
    name = "replies are CRC-valid with the structure AM32 produces"
    source = ("dshot.c: the reply CRC is always inverted; an eRPM field has mantissa bit 8 set whenever its exponent is not 0 - a frame "
              "that breaks that rule is shaped like an Extended DShot Telemetry frame (AM32.md section 10)")
    replies = [r["reply"] for r in records if r["reply"] is not None]
    if not replies:
        return Result(name, NA, "no replies", source)
    bad = [x for x in replies if not x["crc_ok"]]
    structure = 0
    for x in replies:
        if x["crc_ok"]:
            data12 = x["data12"]
            if (data12 >> 9) & 7 and not (data12 >> 8) & 1:
                structure += 1
    detail = "%d replies: %d with a bad CRC, %d shaped like Extended DShot Telemetry frames" % (len(replies), len(bad), structure)
    values = {"replies": len(replies), "bad_crc": len(bad), "edt_shaped": structure}
    if structure or bad:
        return Result(name, WARN, detail, source, values)
    return Result(name, PASS, detail, source, values)


def check_reply_timing(records, speed=600):
    name = "reply bit time matches an AM32 MCU family"
    source = "IO.c per MCU family (AM32.md section 13): DSHOT600 reply bit times 1.28-1.45 us, twice that at DSHOT300"
    replies = [r["reply"] for r in records if r["reply"] is not None]
    if not replies:
        return Result(name, NA, "no replies", source)
    bit = statistics.median(x["bit_units"] for x in replies) * UNIT_US
    delay = statistics.median(x["delay"] for x in replies) * UNIT_US
    scale = 600 / speed
    family, expected = min(((k, v * scale) for k, v in REPLY_BIT_US.items()), key=lambda kv: abs(kv[1] - bit))
    off = (bit - expected) / expected * 100
    detail = "measured %.3f us per reply bit; nearest %s (%.4f us, %+.1f%%); the reply starts %.1f us after our frame ends (%.1f bit times; the source pads %d bit times of idle first)" % (
        bit, family, expected, off, delay, delay / bit, 14 if speed == 600 else 7)
    values = {"bit_us": bit, "family": family, "off_pct": off, "delay_us": delay,
              "f051_off_pct": (bit - REPLY_BIT_US["F051/F031"] * scale) / (REPLY_BIT_US["F051/F031"] * scale) * 100}
    if abs(off) > 1.5:
        return Result(name, WARN, "no family within 1.5%: " + detail, source, values)
    if family != "F051/F031":
        detail += ("; the speed-band and window numbers in this report assume the F051 capture clocks, so with %s they must be "
                   "re-derived from its IO.c first" % family)
    return Result(name, INFO, detail, source, values)


def check_start_and_stop(rows):
    name = "motor start and stuck-rotor timing, as the replies show them"
    source = "main.c: start at input >= 47; stuck-rotor cut after 101 failed zero-crossing checks of 22.5 ms (2.27 s) below throttle 150, or 11 (0.25 s) at 150 and above (AM32.md section 18)"
    nonzero = next((r for r in rows if r["throttle"] >= 48), None)
    if nonzero is None or not any(r["reply"] != "none" for r in rows):
        return Result(name, NA, "no throttle step or no replies", source)
    after = [r for r in rows if r["t_ms"] >= nonzero["t_ms"]]
    first_run = next((r for r in after if r["reply"] == "run"), None)
    if first_run is None:
        return Result(name, INFO, "throttle %d from %.0f ms, but no reply ever reported a running motor" % (nonzero["throttle"], nonzero["t_ms"]), source)
    text = "throttle %d from %.0f ms; first running reply after %.0f ms" % (
        nonzero["throttle"], nonzero["t_ms"], first_run["t_ms"] - nonzero["t_ms"])
    values = {"throttle": nonzero["throttle"], "first_running_ms": first_run["t_ms"] - nonzero["t_ms"], "stopped": False}
    stop = None
    for i, r in enumerate(after):
        if r["t_ms"] > first_run["t_ms"] and r["throttle"] >= 48 and r["reply"] == "idle":
            if all(x["reply"] == "idle" for x in after[i:i + 3]):
                stop = r
                break
    if stop is not None:
        elapsed = stop["t_ms"] - nonzero["t_ms"]
        limit = (11 if nonzero["throttle"] >= 150 else 101) * STUCK_TIMEOUT_MS
        text += "; replies went back to not-running at %.0f ms (%.0f ms after the step) with throttle still %d - the source's stuck-rotor cut needs at least %.0f ms of failed checks at this throttle" % (
            stop["t_ms"], elapsed, stop["throttle"], limit)
        values.update({"stopped": True, "stop_after_ms": elapsed})
    return Result(name, INFO, text, source, values)


def check_arming_silence(rows, from_first_frame):
    name = "the ESC's arming silence comes before our first nonzero throttle"
    source = ("main.c: armed = 1 plays the arming tune with interrupts disabled (about 300 ms for one cell); any nonzero input before that "
              "restarts the 1.02 s count (AM32.md section 16)")
    first = next((i for i, r in enumerate(rows) if r["reply"] != "none"), None)
    nonzero = next((r for r in rows if r["throttle"] != 0), None)
    if first is None or nonzero is None:
        return Result(name, NA, "no replies or no nonzero throttle", source)
    silence = None
    previous = None
    for r in rows[first:]:
        if r["reply"] == "none":
            continue
        if previous is not None and r["t_ms"] - previous["t_ms"] >= TUNE_GAP_MS[0]:
            silence = (previous["t_ms"], r["t_ms"])
            break
        previous = r
    if silence is None:
        return Result(name, WARN, "no silence of 250 ms or more in the replies: the arming tune is not visible, so the ESC's arming "
                      "is not shown by this recording (throttle first nonzero at %.0f ms)" % nonzero["t_ms"], source)
    detail = "silence %.0f-%.0f ms (%.0f ms), our first nonzero throttle at %.0f ms" % (
        silence[0], silence[1], silence[1] - silence[0], nonzero["t_ms"])
    if from_first_frame:
        detail += "; it began %.0f ms after our first frame (the gate needs at least 1020 ms)" % (silence[0] - rows[0]["t_ms"])
    values = {"start_ms": silence[0] - rows[0]["t_ms"], "duration_ms": silence[1] - silence[0],
              "first_throttle_ms": nonzero["t_ms"] - rows[0]["t_ms"]}
    if nonzero["t_ms"] >= silence[0]:
        return Result(name, PASS, detail, source, values)
    return Result(name, WARN, detail + ": throttle came first, so this silence is not the arming tune the source describes", source, values)


def esc_only(result):
    """Mark a check as being about the real ESC's own behaviour (session_summary leaves it out for an ESC mock session)."""
    result.esc_only = True
    return result


def run_checks(records, unidirectional=False, from_first_frame=False):
    frames = [r for r in records if r["tx"] is not None]
    if not frames:
        return [Result("frames", FAIL, "no complete frame of ours in the recording", "")]
    results = [
        check_bit_threshold(frames),
        check_frame_window(frames),
        check_speed_band(frames),
        check_throttle_values(frames),
        check_signal_loss(frames),
        check_arming_gate(frames, from_first_frame),
        check_input_crc(frames, unidirectional),
    ]
    rows = reply_rows(records)
    if unidirectional:
        results.append(Result("replies", NA, "a unidirectional recording has no replies", ""))
    else:
        results += [
            esc_only(check_latch(rows, from_first_frame)),
            check_reply_coverage(rows),
            esc_only(check_arming_silence(rows, from_first_frame)),
            check_reply_integrity(records),
            check_reply_timing(records, frame_speed(frames)),
            check_start_and_stop(rows),
        ]
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("file")
    parser.add_argument("--unidirectional", action="store_true")
    parser.add_argument("--from-first-frame", action="store_true",
                        help="the recording began when our frames began (the start line rose before arming)")
    parser.add_argument("--lines", default=None, help="comma-separated line indexes (default all)")
    parser.add_argument("--mcu", default="F051", choices=sorted(MCU_TICK_US), help="MCU family for the tick-based limits (default F051)")
    args = parser.parse_args()
    set_mcu(args.mcu)
    start, channels, end = spy_edge_log.parse_stream(Path(args.file).read_bytes())
    info = spy_edge_log.parse_end(end)
    print("recording: %s" % start)
    if not info["valid"]:
        print("WARNING: the spy reports lost data - %s" % end)
    failed = 0
    wanted = sorted(channels) if args.lines is None else [int(x) for x in args.lines.split(",")]
    for channel in wanted:
        if channel not in channels:
            continue
        records = spy_edge_log.analyse(spy_edge_log.edges(channels[channel], invert=args.unidirectional), args.unidirectional)
        print("\nline %d" % channel)
        for result in run_checks(records, args.unidirectional, args.from_first_frame):
            print("  [%-4s] %s" % (result.status, result.name))
            print("         %s" % result.detail)
            if result.source:
                print("         source: %s" % result.source)
            failed += result.status == FAIL
    print("\n%d check(s) FAILED" % failed if failed else "\nno check failed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
