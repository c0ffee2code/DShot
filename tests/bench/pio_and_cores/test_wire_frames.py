# Test: what exactly do our frames look like on the wire, and does a throttle step come out clean?
#
# Purpose: the motors struggle to start and nothing proves yet that the signal leaving the Pico is right at
# the level of single bits. This records the four transmit pins at 10 MHz (0.1us) with the pin recorder,
# while the production loop runs (four bidirectional motors in a MotorGroup, Core1Runner at
# MotorGroup.UPDATE_INTERVAL_US, armed), and decodes every frame from the waveform alone with
# dshot_frame_decode.py: bit period, the low time of the 1 and 0 bits, the 16-bit word, its inverted CRC. Five
# 20ms recordings: the end of arming (where the group switches from its precomputed zero frame to the normal
# path), steady zero throttle, a step 0 -> 60 inside the recording, steady 60, a step 60 -> 0.
#
# No ESC and no motor: GPIO 10-13, nothing attached; throttle 60 is only ever a number on four loose pins.
#
# Pass, on every channel:
#   - every complete frame has a valid inverted CRC and a clear telemetry bit;
#   - the throttle sequence in each recording is exactly what was commanded: all 0, then 0s followed by 60s
#     with nothing in between, all 60, then 60s followed by 0s - no frame with any other value;
#   - bit timing: mean bit period 1.667us +-15ns, every single bit period within 1.5-1.85us, mean low time
#     of a 1 bit 1.25us +-80ns and of a 0 bit 0.625us +-80ns;
#   - at most 2 partial frames per recording (the ones cut by its two ends);
#   - frame starts in the arming-end recording are one period apart (no frame missing or doubled at the switch).
#
# Run with: python scripts/run_session.py test_wire_frames.py

import utime
import pacing_probe
import dshot_frame_decode

SAMPLE_HZ = 10_000_000
UNIT_NS = 100
STEP_AT_US = 8000
THROTTLE = 60


def record(probe, action=None):
    """One 20ms recording; action() is called once, STEP_AT_US after it started. Returns True when complete."""
    sampler = probe.sampler
    sampler.start(probe.buf)
    start = utime.ticks_us()
    pending = action
    while not sampler.done():
        if pending is not None and utime.ticks_diff(utime.ticks_us(), start) >= STEP_AT_US:
            pending()
            pending = None
    return sampler.finish()


def channel_frames(probe, bit):
    return dshot_frame_decode.decode_frames(probe.sampler.edges(probe.buf, bit, probe.scratch, UNIT_NS))


def sequence(frames):
    """Run-length summary of the throttle values of the complete frames, e.g. '0x9 60x11'."""
    out = []
    for f in frames:
        if not f["ok"]:
            continue
        if out and out[-1][0] == f["throttle"]:
            out[-1][1] += 1
        else:
            out.append([f["throttle"], 1])
    return " ".join("%dx%d" % (v, n) for v, n in out)


def check_window(name, frames, expected, failures, channel):
    ok = [f for f in frames if f["ok"]]
    partial = len(frames) - len(ok)
    if partial > 2:
        failures.append("%s GPIO %d: %d partial frames" % (name, channel, partial))
    bad_crc = sum(1 for f in ok if not f["crc_ok"])
    telemetry = sum(1 for f in ok if f["telemetry"])
    if bad_crc or telemetry:
        failures.append("%s GPIO %d: %d frames with a bad CRC, %d with the telemetry bit set" % (
            name, channel, bad_crc, telemetry))
    values = [f["throttle"] for f in ok]
    if len(values) < 12:
        failures.append("%s GPIO %d: only %d complete frames" % (name, channel, len(values)))
        return
    if expected == "steady0" and any(values):
        failures.append("%s GPIO %d: not all zero: %s" % (name, channel, sequence(frames)))
    if expected == "steady60" and any(v != THROTTLE for v in values):
        failures.append("%s GPIO %d: not all %d: %s" % (name, channel, THROTTLE, sequence(frames)))
    if expected in ("up", "down"):
        first, second = (0, THROTTLE) if expected == "up" else (THROTTLE, 0)
        switched = values.index(second) if second in values else len(values)
        clean = all(v == first for v in values[:switched]) and all(v == second for v in values[switched:])
        if not clean or switched < 3 or len(values) - switched < 3:
            failures.append("%s GPIO %d: the step is not clean: %s" % (name, channel, sequence(frames)))


def main():
    print("=== Wire-level frames (no ESC, GPIO %d-%d, nothing attached), 10 MHz recording ===" % (
        pacing_probe.FIRST_PIN, pacing_probe.FIRST_PIN + pacing_probe.CHANNELS - 1))
    probe = pacing_probe.PacingProbe(sample_hz=SAMPLE_HZ)
    group = probe.group
    failures = []
    stats = [dshot_frame_decode.TimingStats() for _ in range(pacing_probe.CHANNELS)]

    def process(name, expected, per_channel):
        for bit, frames in enumerate(per_channel):
            channel = pacing_probe.FIRST_PIN + bit
            check_window(name, frames, expected, failures, channel)
            for frame in frames:
                stats[bit].add(frame)
            print("%-11s GPIO %d: %d frames (%d complete): %s" % (
                name, channel, len(frames), sum(1 for f in frames if f["ok"]), sequence(frames)))

    def set_all(value):
        def action():
            for i in range(pacing_probe.CHANNELS):
                group.set_throttle(i, value)
        return action

    try:
        # the end of arming: recording starts 10ms before the 2000ms window is over
        probe.runner.start()
        group.arm(pacing_probe.ARM_MS)
        began = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), began) < pacing_probe.ARM_MS - 10:
            pass
        if not record(probe):
            failures.append("arming end: the recording did not complete")
        else:
            per_channel = [channel_frames(probe, bit) for bit in range(pacing_probe.CHANNELS)]
            process("arming end", "steady0", per_channel)
            for bit, frames in enumerate(per_channel):
                starts = [f["start"] for f in frames if f["ok"]]
                gaps = [b - a for a, b in zip(starts, starts[1:])]
                period = probe.period_us * 10
                if not gaps or min(gaps) < period * 0.5 or max(gaps) > period * 1.5:
                    failures.append("arming end GPIO %d: frame starts %s (0.1us units, period %d)" % (
                        pacing_probe.FIRST_PIN + bit, gaps, period))
        while not group.is_armed():
            if probe.runner.error is not None:
                raise probe.runner.error
            utime.sleep_ms(1)
        utime.sleep_ms(100)
        for name, expected, action in (("zero", "steady0", None), ("step 0->60", "up", set_all(THROTTLE)),
                                       ("steady 60", "steady60", None), ("step 60->0", "down", set_all(0))):
            if not record(probe, action):
                failures.append(name + ": the recording did not complete")
                continue
            process(name, expected, [channel_frames(probe, bit) for bit in range(pacing_probe.CHANNELS)])
    finally:
        if not probe.close():
            print("WARNING: Core 1 loop did not stop - deactivate() raced update()")

    print()
    print("bit timing over all frames (in us; expected period 1.667, low of a 1 bit 1.250, of a 0 bit 0.625):")
    for bit in range(pacing_probe.CHANNELS):
        channel = pacing_probe.FIRST_PIN + bit
        s = stats[bit].summary()
        if not s["frames"]:
            failures.append("GPIO %d: no frame decoded" % channel)
            continue
        print("GPIO %d: %d frames, period mean %.3f min %.2f max %.2f, low(1) %.3f, low(0) %.3f" % (
            channel, s["frames"], s["period_mean"] / 10, s["period_min"] / 10, s["period_max"] / 10,
            s["one_low"] / 10, s["zero_low"] / 10))
        if abs(s["period_mean"] - dshot_frame_decode.BIT_UNITS) > 0.15:
            failures.append("GPIO %d: mean bit period %.3fus" % (channel, s["period_mean"] / 10))
        if s["period_min"] < 15 or s["period_max"] > 18.5:
            failures.append("GPIO %d: a bit period of %.2f-%.2fus" % (channel, s["period_min"] / 10, s["period_max"] / 10))
        if abs(s["one_low"] - dshot_frame_decode.ONE_LOW_UNITS) > 0.8:
            failures.append("GPIO %d: low time of a 1 bit %.3fus" % (channel, s["one_low"] / 10))
        if abs(s["zero_low"] - dshot_frame_decode.ZERO_LOW_UNITS) > 0.8:
            failures.append("GPIO %d: low time of a 0 bit %.3fus" % (channel, s["zero_low"] / 10))
    print()
    for line in failures:
        print("FAIL " + line)
    print("PASSED" if not failures else "FAILED: " + str(len(failures)))


main()
