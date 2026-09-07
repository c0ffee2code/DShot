# W4/W5 bidirectional DShot stress harness (see
# bidirectional_dshot_review.md's backlog and decision/ADR-002-bidirectional-dshot.md).
#
# W4: saturated continuous-capture stress harness. After arming, sends
# throttle commands with no application-level pacing - send_throttle_command()
# itself blocks once the 4-deep TX FIFO fills, which is what "no pacing"
# means here - and drains the RX FIFO every iteration, counting frames
# queued, complete/misaligned/partial 4-word capture groups, and max
# observed RX FIFO occupancy. CHANNELS=(1,) alone is the true saturation
# case (a multi-channel round-robin send is inherently paced by the send
# order); CHANNELS=(1, 3) together is a secondary comparison point against
# the bench's real 2-motor configuration, not required for "done."
#
# W5: extends the same harness with STARVATION_ENABLED=True - periodically
# stops draining the RX FIFO (TX keeps sending) for a few ms, then measures
# how the RX state machine recovers.
#
# This is characterization, not pass/fail: a low association rate, FIFO
# stalls, or a starvation cycle that never recovers are the data this
# script exists to produce, not failures. Only a genuine driver exception
# aborts the run.
#
# Talks to DShotPIO directly - no MotorThrottleGroup, no ScenarioRunner, no
# Core 1 - because those all pace their send loop (a fixed-interval tick,
# round-robin over up to 4 channels), and W4/W5 specifically need an
# unpaced single-channel loop to be the true saturation case the review's
# P0 finding is about.
#
# Captures a bounded sample of raw words to the PicoBell's SD card (see
# tests/harness/stress_capture_sink.py) for offline CRC decoding - the
# on-device marker-bit check below is a cheap structural proxy, not a real
# CRC check, and full GCR decode stays off-device per the existing
# pipeline (see scripts/analyze_bidir_stress_log.py).

import gc

from machine import Pin
from dshot_pio import DShotPIO, DSHOT_SPEEDS
from stress_capture_sink import StressCaptureSink
import utime

# --- Configuration ------------------------------------------------------

CHANNEL_WIRING = {
    1: {"pin": 6, "sm_id": 0, "rx_sm_id": 1},
    3: {"pin": 8, "sm_id": 4, "rx_sm_id": 5},
}
CHANNELS = (1,)  # set to (1, 3) for the secondary comparison run

DSHOT_SPEED = DSHOT_SPEEDS.DSHOT300
ARM_DURATION_MS = 3000
ARM_THROTTLE = 0
RUN_THROTTLE = 60  # confirmed smooth post-arm value - see ADR-002's confirmation sweeps
# 20,000 (2026-09-07, down from a 60,000/1600-sample attempt that hit a MicroPython
# MemoryError partway through - the Pico's heap couldn't sustain that long a run plus that
# large a sample buffer at once). ~10s at this harness's ~2000 frames/s, giving 3 starvation
# cycles at the 3s spacing below.
TARGET_FRAMES = 20_000

STARVATION_ENABLED = True  # set True for the W5 run
STARVATION_WINDOW_MS = 5
# Widened from 200ms to 3000ms (2026-09-07 rerun): the original 200ms spacing left every
# resume without an undisturbed local baseline to compare against - captures within +-100ms
# of a resume were themselves inside another stall's disturbance. 3s apart gives each resume
# a clean neighborhood on both sides.
STARVATION_INTERVAL_MS = 3000
RECOVERY_K = 3               # consecutive good groups required to call a cycle recovered
RECOVERY_TIMEOUT_FRAMES = 200

SAMPLE_CAP = 500
SAMPLE_ARM_FIRST_N = 20
SAMPLE_SATURATION_FIRST_N = 100
# Widened from 50 to 100 (2026-09-07) alongside TARGET_FRAMES's cut - keeps total saturation
# sample volume down (~300 over 20,000 frames) while still giving a handful of local samples
# within +-100ms of each of the 3 widely-spaced starvation cycles.
SAMPLE_SATURATION_EVERY = 100
SAMPLE_RESUME_FIRST_N = 10

# How often (in saturation-loop iterations) to force a GC pass - cheap insurance against the
# heap fragmentation that caused the MemoryError above, on a run now long enough (thousands
# of small per-iteration allocations) for fragmentation to matter.
GC_INTERVAL_FRAMES = 2000

PHASE_ARM = 0
PHASE_SATURATION = 1
PHASE_POST_RESUME = 2
PHASE_BOUNDARY_PARTIAL = 3


class ChannelState:
    def __init__(self, channel, motor):
        self.channel = channel
        self.motor = motor
        self.pending = []
        self.frames_queued = 0
        self.arm_window_groups = 0
        self.complete_captures = 0
        self.misaligned_captures = 0
        self.partial_captures = 0
        self.fifo_hist = [0, 0, 0, 0, 0]
        self.samples = []  # (ticks_us, phase, word_count, w0, w1, w2, w3)
        self.starvation_cycles = []  # list of dicts, one per starvation cycle


# --- Word grouping / sampling -------------------------------------------

def is_marker_valid(words):
    # dshot_bidir_rx's wait(0, pin, 0) is a blocking level-wait, so the
    # very first captured sample (word0's top bit) is always the reply's
    # marker bit - always 0 - on a real capture. Confirmed against an
    # existing hardware capture: 100% match on 30,221/30,221 real groups.
    return (words[0] >> 31) == 0


def _padded_words(words):
    padded = list(words) + [0] * (4 - len(words))
    return padded[0], padded[1], padded[2], padded[3]


def maybe_sample(state, phase, ticks_us, words, counters):
    key = (state.channel, phase)
    index_in_phase = counters.get(key, 0)
    counters[key] = index_in_phase + 1

    if len(state.samples) >= SAMPLE_CAP:
        return
    take = False
    if phase == PHASE_ARM:
        take = index_in_phase < SAMPLE_ARM_FIRST_N
    elif phase == PHASE_SATURATION:
        take = (index_in_phase < SAMPLE_SATURATION_FIRST_N
                or index_in_phase % SAMPLE_SATURATION_EVERY == 0)
    elif phase == PHASE_POST_RESUME:
        take = index_in_phase < SAMPLE_RESUME_FIRST_N
    elif phase == PHASE_BOUNDARY_PARTIAL:
        take = True

    if take:
        w0, w1, w2, w3 = _padded_words(words)
        state.samples.append((ticks_us, phase, len(words), w0, w1, w2, w3))


def drain_channel(state, phase, counters):
    """Drain all currently-queued raw words, grouping every 4 into one
    capture. The RX FIFO depth (4 words) exactly matches one capture and a
    full FIFO stalls the RX state machine rather than corrupting data, so
    under healthy draining `pending` should only ever hold 0-3 words
    between groups - it should never carry stale words across a phase or
    starvation boundary (see flush_boundary)."""
    motor = state.motor
    while True:
        word = motor.rx_read()
        if word is None:
            break
        state.pending.append(word)
        if len(state.pending) == 4:
            ticks_us = utime.ticks_us()
            group = state.pending
            state.pending = []
            if phase == PHASE_ARM:
                # Every capture in the arm window is a TX echo by
                # construction (the ESC doesn't reply before it's armed) -
                # counted separately, never classified complete/misaligned.
                state.arm_window_groups += 1
            elif is_marker_valid(group):
                state.complete_captures += 1
            else:
                state.misaligned_captures += 1
            maybe_sample(state, phase, ticks_us, group, counters)


def flush_boundary(state, counters):
    """Count and discard any leftover partial group at a phase/starvation
    boundary. A non-empty `pending` here is a real partial/misaligned
    group at the boundary, not routine backlog - see drain_channel."""
    if state.pending:
        ticks_us = utime.ticks_us()
        state.partial_captures += 1
        maybe_sample(state, PHASE_BOUNDARY_PARTIAL, ticks_us, state.pending, counters)
        state.pending = []


# --- Phases ---------------------------------------------------------------

def arm(states, counters):
    print("Arming for {}ms...".format(ARM_DURATION_MS))
    start = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start) < ARM_DURATION_MS:
        for state in states.values():
            state.motor.send_throttle_command(ARM_THROTTLE)
            drain_channel(state, PHASE_ARM, counters)
    print("Armed.")


def run_saturation(states, counters):
    print("Saturation run: target {} frames/channel...".format(TARGET_FRAMES))
    next_starve_ms = utime.ticks_add(utime.ticks_ms(), STARVATION_INTERVAL_MS)
    any_state = next(iter(states.values()))
    while not all(s.frames_queued >= TARGET_FRAMES for s in states.values()):
        for state in states.values():
            state.motor.send_throttle_command(RUN_THROTTLE)
            state.frames_queued += 1
            occ = state.motor.rx_sm.rx_fifo()
            state.fifo_hist[occ] += 1
            drain_channel(state, PHASE_SATURATION, counters)

        if any_state.frames_queued % GC_INTERVAL_FRAMES == 0:
            gc.collect()

        if STARVATION_ENABLED and utime.ticks_diff(utime.ticks_ms(), next_starve_ms) >= 0:
            run_starvation_cycle(states, counters)
            next_starve_ms = utime.ticks_add(utime.ticks_ms(), STARVATION_INTERVAL_MS)
    print("Saturation run complete.")


def run_starvation_cycle(states, counters):
    # NOTE: per-channel recovery below is sequential - while one channel's
    # recovery loop runs (up to RECOVERY_TIMEOUT_FRAMES iterations), any
    # other channel in `states` gets no send_throttle_command() calls at
    # all, risking an ESC signal-loss timeout on that channel. Harmless at
    # the default CHANNELS=(1,), which is W5's primary evidence; a real
    # gap in the secondary CHANNELS=(1, 3) comparison mode, accepted here
    # rather than adding concurrent-recovery bookkeeping to throwaway
    # characterization code.
    pre_occ = {ch: s.motor.rx_sm.rx_fifo() for ch, s in states.items()}
    frames_before = {ch: s.frames_queued for ch, s in states.items()}

    # The deliberate starvation window: TX keeps sending (the only choice
    # that leaves the arm-proven back-to-back TX cadence untouched), only
    # rx_read() stops being called - the only way to actually exercise
    # autopush blocking in_() on a full RX FIFO.
    stall_start = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), stall_start) < STARVATION_WINDOW_MS:
        for state in states.values():
            state.motor.send_throttle_command(RUN_THROTTLE)
            state.frames_queued += 1

    for ch, state in states.items():
        frames_during_stall = state.frames_queued - frames_before[ch]
        resume_occ = state.motor.rx_sm.rx_fifo()
        flush_boundary(state, counters)
        # Sample the first N groups after *this* resume, not a running
        # total across every resume in the run.
        counters[(ch, PHASE_POST_RESUME)] = 0

        recovered = False
        consecutive_good = 0
        frames_to_recovery = RECOVERY_TIMEOUT_FRAMES
        invalid_after_resume = 0

        for frame_n in range(RECOVERY_TIMEOUT_FRAMES):
            state.motor.send_throttle_command(RUN_THROTTLE)
            state.frames_queued += 1
            before_complete = state.complete_captures
            before_misaligned = state.misaligned_captures
            drain_channel(state, PHASE_POST_RESUME, counters)
            if state.complete_captures > before_complete:
                consecutive_good += 1
                if consecutive_good >= RECOVERY_K:
                    frames_to_recovery = frame_n + 1
                    recovered = True
                    break
            elif state.misaligned_captures > before_misaligned:
                invalid_after_resume += 1
                consecutive_good = 0

        restart_required = not recovered
        if restart_required:
            _restart_and_rearm(state, counters)

        # The RX FIFO holds at most one capture's worth while stalled, and
        # a stalled RX SM cannot start a new capture cycle at all - so
        # every reply the ESC generated during the stall beyond the one
        # (if any) sitting in the FIFO at resume is provably uncaptured.
        captures_lost = max(0, frames_during_stall - min(1, resume_occ))

        state.starvation_cycles.append({
            "pre_occ": pre_occ[ch],
            "resume_occ": resume_occ,
            "frames_during_stall": frames_during_stall,
            "captures_lost": captures_lost,
            "invalid_after_resume": invalid_after_resume,
            "frames_to_recovery": frames_to_recovery,
            "recovered": recovered,
            "restart_required": restart_required,
        })


def _restart_and_rearm(state, counters):
    # Expensive and, per the arm sequence's back-to-back requirement,
    # likely de-arms the ESC - that cost/frequency is itself the headline
    # datum this path exists to surface. In multi-channel mode this stalls
    # other channels' sends for the ~3s re-arm window (their TX FIFOs
    # aren't fed), which is an accepted limitation of this throwaway
    # harness, not something worth engineering around here.
    print("  Channel {}: starvation cycle did not recover in {} frames - "
          "restarting and re-arming.".format(state.channel, RECOVERY_TIMEOUT_FRAMES))
    motor = state.motor
    motor.stop()
    motor.start()
    state.pending = []
    start = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start) < ARM_DURATION_MS:
        motor.send_throttle_command(ARM_THROTTLE)
        drain_channel(state, PHASE_ARM, counters)
    flush_boundary(state, counters)


# --- Reporting --------------------------------------------------------

def meta_fields_for(state):
    fields = {
        "frames_queued": state.frames_queued,
        "arm_window_groups": state.arm_window_groups,
        "complete_captures": state.complete_captures,
        "misaligned_captures": state.misaligned_captures,
        "partial_captures": state.partial_captures,
        "fifo_occupancy_hist": ",".join(str(n) for n in state.fifo_hist),
    }
    if state.starvation_cycles:
        cycles = state.starvation_cycles
        n = len(cycles)
        recovery_frames = [c["frames_to_recovery"] for c in cycles]
        fields["starvation_cycles"] = n
        fields["captures_lost_total"] = sum(c["captures_lost"] for c in cycles)
        fields["invalid_after_resume_total"] = sum(c["invalid_after_resume"] for c in cycles)
        fields["frames_to_recovery_min"] = min(recovery_frames)
        fields["frames_to_recovery_max"] = max(recovery_frames)
        fields["frames_to_recovery_mean"] = sum(recovery_frames) / n
        fields["restart_required_count"] = sum(1 for c in cycles if c["restart_required"])
    return fields


def summarize(states):
    print()
    print("=== Summary ===")
    for ch, state in states.items():
        assoc = (state.complete_captures / state.frames_queued
                  if state.frames_queued else 0.0)
        print("Channel {}:".format(ch))
        print("  frames_queued={} arm_window_groups={}".format(
            state.frames_queued, state.arm_window_groups))
        print("  complete_captures={} misaligned_captures={} partial_captures={}".format(
            state.complete_captures, state.misaligned_captures, state.partial_captures))
        print("  association_rate={:.4f}".format(assoc))
        print("  fifo_occupancy_hist={}".format(state.fifo_hist))
        if state.starvation_cycles:
            fields = meta_fields_for(state)
            print("  starvation_cycles={} captures_lost_total={} invalid_after_resume_total={}".format(
                fields["starvation_cycles"], fields["captures_lost_total"],
                fields["invalid_after_resume_total"]))
            print("  frames_to_recovery min={} max={} mean={:.1f} restart_required_count={}".format(
                fields["frames_to_recovery_min"], fields["frames_to_recovery_max"],
                fields["frames_to_recovery_mean"], fields["restart_required_count"]))
        print()


# --- Entry point ------------------------------------------------------

def test_bidir_rx_stress():
    print("=== Bidir RX Stress ({}) ===".format(
        "W5 starvation" if STARVATION_ENABLED else "W4 saturation"))

    sink = StressCaptureSink()
    provenance = {
        "channels": list(CHANNELS),
        "dshot_speed": DSHOT_SPEED,
        "run_throttle": RUN_THROTTLE,
        "target_frames": TARGET_FRAMES,
        "starvation_enabled": STARVATION_ENABLED,
        "starvation_window_ms": STARVATION_WINDOW_MS,
        "starvation_interval_ms": STARVATION_INTERVAL_MS,
    }
    sink.init_session(provenance)
    print("Session:", sink.path)
    print()

    states = {}
    counters = {}  # (channel, phase) -> count seen so far, for sampling
    outcome = "failed"

    try:
        for ch in CHANNELS:
            wiring = CHANNEL_WIRING[ch]
            motor = DShotPIO(wiring["sm_id"], Pin(wiring["pin"]), DSHOT_SPEED,
                              bidirectional=True, rx_state_machine_id=wiring["rx_sm_id"])
            motor.start()
            states[ch] = ChannelState(ch, motor)

        arm(states, counters)
        for state in states.values():
            flush_boundary(state, counters)

        run_saturation(states, counters)
        outcome = "completed"

    except KeyboardInterrupt:
        print("\nInterrupted!")
        raise

    finally:
        print("Stopping...")
        for state in states.values():
            state.motor.send_throttle_command(0)
        utime.sleep_ms(50)
        for state in states.values():
            state.motor.drain()
            state.motor.stop()
        print("Motors stopped and deactivated.")

        gc.collect()
        all_records = []
        for state in states.values():
            for ticks_us, phase, word_count, w0, w1, w2, w3 in state.samples:
                all_records.append((ticks_us, state.channel, phase, word_count, w0, w1, w2, w3))
        sink.write_records(all_records)

        meta_fields = {}
        for ch, state in states.items():
            for key, value in meta_fields_for(state).items():
                meta_fields["ch{}_{}".format(ch, key)] = value
        sink.finalize(outcome, meta_fields)
        sink.close()
        print("SD card flushed and unmounted.")

        summarize(states)
        print("=== Test Complete ===" if outcome == "completed" else "=== Test FAILED ===")


test_bidir_rx_stress()
