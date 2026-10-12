# boot_timeline.py - per-ESC-boot timeline from a session's arming.bin.
#
# arming.bin logging was retired 2026-10-06 (BUG-002: real per-tick overhead,
# ~90us/call even idle, on the exact loop being measured - see BUG-002.md's
# "Telemetry overhead and the timed window" section). This still works on
# the ~900 sessions already archived from before that date; a session
# captured after it has no arming.bin, and this script has nothing to read
# for it.
#
# BUG-002 diagnostic (2026-10-04): find_stall_position.py's "word==0 means this
# motor is in trouble" check is only valid for a single-bidirectional-motor
# session - with two, a merged record shows 0 in one motor's slot whenever only
# the OTHER motor published that pass, which looks identical to real silence.
# This tracks each bidirectional motor's own last-seen reply independently of
# the other motor's updates, splits its timeline into "boots" (a boot ends on a
# gap of BOOT_GAP_MS or more, i.e. a real reboot-length silence - see BUG-002.md's
# "99% screen and verification" section for why ~600ms is a real firmware
# reboot, not a receiver hiccup), and reports, per boot:
#   - start time since arm() (since the previous boot's blackout ended, or the
#     very start of arming for boot 0)
#   - time from the boot's own start to its first literal not-running reply
#     (gcr_decode.AM32_NOT_RUNNING_FRAME), or "never" if none arrived before
#     the boot ended
#   - the largest gap between two not-running replies within the boot, if
#     >=GOOD_GAP_FLOOR_MS (150ms - long enough to not be ordinary reply jitter)
#   - whether the boot ends in a blackout (vs. being the run's last, unfinished
#     boot) and that blackout's duration
#
# Usage: python scripts/boot_timeline.py captures/<session> [more session dirs...]
#   (reads <session>/arming.bin and <session>/meta.txt)

import struct
import sys
from pathlib import Path

RECORD_FMT = "<I4H4I"
RECORD_SIZE = struct.calcsize(RECORD_FMT)
TICKS_MASK = 0x3FFFFFFF  # MicroPython ticks_us() wraps at 2**30, same as fakes.py
NOT_RUNNING = 0x52951  # gcr_decode.AM32_NOT_RUNNING_FRAME

BOOT_GAP_MS = 100          # a gap at least this long, in ANY reply, ends a boot
GOOD_GAP_FLOOR_MS = 150    # report not-running gaps at least this long within a boot


def ticks_diff(a, b):
    diff = (a - b) & TICKS_MASK
    return diff - (TICKS_MASK + 1) if diff >= (TICKS_MASK + 1) // 2 else diff


def read_meta(session_dir):
    meta = {}
    path = session_dir / "meta.txt"
    for line in path.read_text().splitlines():
        if "=" in line:
            key, _, value = line.partition("=")
            meta[key] = value
    return meta


def read_records(arming_bin_path):
    data = arming_bin_path.read_bytes()
    n = len(data) // RECORD_SIZE
    for i in range(n):
        rec = struct.unpack_from(RECORD_FMT, data, i * RECORD_SIZE)
        yield rec[0], rec[5:9]  # ticks_us, 4 motor words


def boots_for_motor(records, motor_index, arm_ticks_us):
    """
    Split one motor's own reply timeline (ignoring every record where this
    motor's word is 0 - no new capture from it that pass) into boots. Returns
    a list of dicts, oldest first. The last boot has blackout_ms=None if the
    log ends before one starts (the run finished mid-boot, which is normal for
    a clean run).
    """
    replies = [(t, w) for t, w in records if w[motor_index] != 0]
    boots = []
    if not replies:
        return boots

    boot_start_idx = 0
    for i in range(1, len(replies) + 1):
        ended = i == len(replies) or ticks_diff(replies[i][0], replies[i - 1][0]) / 1000 >= BOOT_GAP_MS
        if not ended:
            continue
        segment = replies[boot_start_idx:i]
        boot_start_t = segment[0][0]
        first_reply_ms = 0.0  # by definition, the boot starts at its own first reply

        first_not_running_ms = None
        not_running_times = []
        pre_lock_words = []  # every word seen before the first not-running reply (or the whole boot, if none)
        for t, w in segment:
            if w[motor_index] == NOT_RUNNING:
                not_running_times.append(t)
            elif not not_running_times:
                pre_lock_words.append(w[motor_index])
        if not_running_times:
            first_not_running_ms = ticks_diff(not_running_times[0], boot_start_t) / 1000

        largest_nr_gap = None
        for j in range(1, len(not_running_times)):
            gap_ms = ticks_diff(not_running_times[j], not_running_times[j - 1]) / 1000
            if gap_ms >= GOOD_GAP_FLOOR_MS and (largest_nr_gap is None or gap_ms > largest_nr_gap[1]):
                start_ms = ticks_diff(not_running_times[j - 1], boot_start_t) / 1000
                largest_nr_gap = (start_ms, gap_ms)

        blackout_ms = None
        if i < len(replies):
            blackout_ms = ticks_diff(replies[i][0], replies[i - 1][0]) / 1000

        boots.append({
            "motor": motor_index,
            "boot_index": len(boots),
            "start_since_arm_ms": ticks_diff(boot_start_t, arm_ticks_us) / 1000,
            "first_reply_ms": first_reply_ms,
            "first_not_running_ms": first_not_running_ms,
            "good_gap": largest_nr_gap,  # (start_ms_since_boot, duration_ms) or None
            "blackout_ms": blackout_ms,  # None = boot never ended (run finished or no more replies)
            "reply_count": len(segment),
            "lifetime_ms": ticks_diff(segment[-1][0], boot_start_t) / 1000,
            "pre_lock_words": pre_lock_words,
        })
        boot_start_idx = i

    return boots


def analyze(session_dir):
    meta = read_meta(session_dir)
    arm_ticks_us = int(meta["arm_ticks_us"])
    motor_indices = [int(x) for x in meta["bidir_motor_indices"].split(",")]
    records = list(read_records(session_dir / "arming.bin"))

    print(session_dir.name)
    for motor_index in motor_indices:
        boots = boots_for_motor(records, motor_index, arm_ticks_us)
        for b in boots:
            fnr = "never" if b["first_not_running_ms"] is None else "%.1fms" % b["first_not_running_ms"]
            gg = "none" if b["good_gap"] is None else "%.1fms@%.1fms" % (b["good_gap"][1], b["good_gap"][0])
            bo = "none (run ended)" if b["blackout_ms"] is None else "%.1fms" % b["blackout_ms"]
            print("  motor%d boot%d start=%.1fms_since_arm first_not_running=%s good_gap=%s "
                  "blackout_after=%s replies=%d" % (
                      b["motor"], b["boot_index"], b["start_since_arm_ms"], fnr, gg, bo, b["reply_count"]))


def aggregate(session_dirs):
    """
    Pool boots_for_motor() across many sessions and print the checks BUG-002.md's
    "99% screen and verification" section needs: tight-band fractions for the
    failed-boot lifetime and blackout, the pre-lock-on word family (bad boots vs
    good boots), and whether each timed event is frame-count-driven (its ms
    value scales with arming_call_avg_us) or time-driven (its ms value stays
    fixed regardless of tick) - all computed separately for boot 0 (the one
    running when arm() was called) vs later boots (after this motor's own
    reset), since a clock anchored to "frames arriving since boot start" should
    read the same in both; one anchored to "ESC power-up" would not run again
    on a later boot the same way.

    A session with no `armed_ticks_us` in its meta.txt never reached ARMED (a
    refusal) - excluded here because a refusal's capture log is dozens of
    reboot fragments, not one clean record of the mechanism this is measuring.
    """
    import statistics
    from collections import Counter

    refused = 0
    rows = []  # list of boot dicts, each tagged with avg_us and is_first_boot
    for s in session_dirs:
        arming_path = s / "arming.bin"
        if not arming_path.exists() or arming_path.stat().st_size == 0:
            continue
        meta = read_meta(s)
        if "armed_ticks_us" not in meta:
            refused += 1
            continue
        if "bidir_motor_indices" not in meta or "arm_ticks_us" not in meta:
            continue
        arm_ticks_us = int(meta["arm_ticks_us"])
        avg_us = int(meta["arming_call_avg_us"]) if "arming_call_avg_us" in meta else None
        records = list(read_records(arming_path))
        for mi in (int(x) for x in meta["bidir_motor_indices"].split(",")):
            for b in boots_for_motor(records, mi, arm_ticks_us):
                b["avg_us"] = avg_us
                b["is_first_boot"] = (b["boot_index"] == 0)
                rows.append(b)

    print("sessions: refused (excluded)=%d" % refused)
    failed = [b for b in rows if b["blackout_ms"] is not None]
    good = [b for b in rows if b["blackout_ms"] is None and b["first_not_running_ms"] is not None]
    print("boots: failed=%d good(reached ARMED-equivalent)=%d" % (len(failed), len(good)))

    def band_fraction(vals, lo, hi):
        return sum(1 for v in vals if lo <= v <= hi) / len(vals) if vals else 0.0

    for label, population in (("all boots", failed), ("boot 0 only", [b for b in failed if b["is_first_boot"]]),
                               ("later boots", [b for b in failed if not b["is_first_boot"]])):
        lifetimes = [b["lifetime_ms"] for b in population]
        blackouts = [b["blackout_ms"] for b in population]
        if not lifetimes:
            continue
        print("[%s] n=%d  lifetime median=%.1f in[1840,1865]=%.1f%%   blackout median=%.1f in[595,620]=%.1f%%" % (
            label, len(lifetimes), statistics.median(lifetimes), 100 * band_fraction(lifetimes, 1840, 1865),
            statistics.median(blackouts), 100 * band_fraction(blackouts, 595, 620)))
        any_nr = sum(1 for b in population if b["first_not_running_ms"] is not None)
        print("           failed boots with >=1 not-running reply before blackout: %d/%d" % (any_nr, len(population)))

    bad_words = Counter()
    good_words = Counter()
    for b in failed:
        for w in b["pre_lock_words"]:
            bad_words[hex(w)] += 1
    for b in good:
        for w in b["pre_lock_words"]:
            good_words[hex(w)] += 1
    print("bad-boot pre-lock words:", bad_words.most_common(8))
    print("good-boot pre-lock words:", good_words.most_common(8))

    print("timer vs frame-count (ms value vs implied frame count = ms*1000/tick_us), by boot 0 / later:")
    for kind, population, field in (
        ("first_nr", good, "first_not_running_ms"),
        ("good_gap_start", [b for b in good if b["good_gap"]], None),
        ("failed_lifetime", failed, "lifetime_ms"),
        ("blackout", failed, "blackout_ms"),
    ):
        for which, pop in (("boot0", [b for b in population if b["is_first_boot"]]),
                           ("later", [b for b in population if not b["is_first_boot"]])):
            bucket = {}
            for b in pop:
                if not b["avg_us"]:
                    continue
                val = b["good_gap"][0] if field is None else b[field]
                key = round(b["avg_us"] / 150) * 150
                bucket.setdefault(key, []).append(val)
            for tick in sorted(bucket):
                vals = bucket[tick]
                if len(vals) < 15:
                    continue
                med = statistics.median(vals)
                print("  %-16s %-5s tick~%5dus n=%4d median_ms=%7.1f implied_frames=%.1f" % (
                    kind, which, tick, len(vals), med, med * 1000 / tick))


def by_scenario(session_dirs):
    """
    Group sessions by scenario identity (bidir motor count, core1_interval_us,
    core1_fixed_schedule, and the scenario's own _comment - the closest thing to
    a stable scenario-file name an archived session has, since the deployed
    file is always just "scenario.json" on the Pico) and report, per group:
    sessions, refused count, the per-SESSION "any bidirectional motor's first
    boot was bad" rate (not a per-motor-boot rate - see BUG-002.md's "archive
    scenario comparison" section for why that distinction matters), and the
    median measured `arming_call_avg_us`/`arming_call_min_us`.

    This is a cross-batch, cross-code-version comparison by construction (an
    archive spans weeks) - report it as descriptive, not as proof one scenario
    beats another; see this project's own CLAUDE.md on checking for a code
    change before trusting a cross-session difference.
    """
    import json
    import statistics
    from collections import defaultdict

    groups = defaultdict(lambda: {"sessions": 0, "refused": 0, "bad_sessions": 0, "clean_sessions": 0,
                                   "avg_us": [], "min_us": [], "n_bidir": None, "interval": None,
                                   "fixed_sched": None, "comment": None})

    for s in session_dirs:
        arming_path = s / "arming.bin"
        scenario_path = s / "scenario.json"
        if not arming_path.exists() or arming_path.stat().st_size == 0 or not scenario_path.exists():
            continue
        try:
            scenario = json.loads(scenario_path.read_text())
        except Exception:
            continue
        meta = read_meta(s)
        if "bidir_motor_indices" not in meta or "arm_ticks_us" not in meta:
            continue
        bidir_indices = [int(x) for x in meta["bidir_motor_indices"].split(",")]
        n_bidir = len(bidir_indices)
        interval = scenario.get("core1_interval_us")
        fixed_sched = scenario.get("core1_fixed_schedule", False)
        comment = scenario.get("_comment", "")[:80]
        key = (n_bidir, interval, fixed_sched, comment)

        g = groups[key]
        g["n_bidir"], g["interval"], g["fixed_sched"], g["comment"] = n_bidir, interval, fixed_sched, comment
        g["sessions"] += 1
        if "armed_ticks_us" not in meta:
            g["refused"] += 1
            continue
        if "arming_call_avg_us" in meta:
            g["avg_us"].append(int(meta["arming_call_avg_us"]))
        if "arming_call_min_us" in meta:
            g["min_us"].append(int(meta["arming_call_min_us"]))
        arm_ticks_us = int(meta["arm_ticks_us"])
        records = list(read_records(arming_path))
        session_bad = False
        for mi in bidir_indices:
            boots = boots_for_motor(records, mi, arm_ticks_us)
            if boots and boots[0]["blackout_ms"] is not None:
                session_bad = True
        if session_bad:
            g["bad_sessions"] += 1
        else:
            g["clean_sessions"] += 1

    rows = []
    for g in groups.values():
        total = g["bad_sessions"] + g["clean_sessions"]
        if total < 10:
            continue
        avg = statistics.median(g["avg_us"]) if g["avg_us"] else None
        mn = statistics.median(g["min_us"]) if g["min_us"] else None
        rows.append((g["n_bidir"], avg, mn, g["interval"], g["fixed_sched"], g["sessions"], g["refused"],
                     total, g["bad_sessions"], g["comment"]))

    rows.sort(key=lambda r: (r[0], r[1] or 0))
    print("%-3s %-7s %-7s %-6s %-5s %-5s %-5s %-5s %-5s %-7s %s" % (
        "nb", "avg~us", "min~us", "intvl", "fixed", "sess", "refus", "n", "bad", "P(bad)", "comment"))
    for n_bidir, avg, mn, interval, fixed, sess, refused, n, bad, comment in rows:
        print("%-3d %-7s %-7s %-6s %-5s %-5d %-5d %-5d %-5d %6.1f%%  %s" % (
            n_bidir, "%.0f" % avg if avg else "?", "%.0f" % mn if mn else "?", str(interval), str(fixed),
            sess, refused, n, bad, 100 * bad / n if n else 0, comment[:50]))


if __name__ == "__main__":
    if sys.argv[1:2] == ["--aggregate"]:
        aggregate([Path(p) for p in sys.argv[2:]])
    elif sys.argv[1:2] == ["--by-scenario"]:
        by_scenario([Path(p) for p in sys.argv[2:]])
    else:
        for path in sys.argv[1:]:
            analyze(Path(path))
