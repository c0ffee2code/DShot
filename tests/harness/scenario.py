# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# Loads and validates a scenario JSON file (see tests/harness/scenarios/ and
# tests/harness/run_scenario.py) into a Scenario/MotorSpec object graph.
#
# Every check here is fail-fast and raises ValueError before any hardware is
# touched - a scenario with a bad throttle curve or an impossible PIO wiring
# must never reach arm(), let alone start spinning a motor.
#
# Pure Python plus MicroPython's built-in `json` module (also present in
# CPython under the same name) - reused as-is by the PC-side
# scripts/check_scenario.py smoke test, not just on-device.

import json

from throttle_profile import ThrottleProfile

# Mirrors driver/dshot_pio.py's DSHOT_SPEEDS values directly rather than
# importing them: dshot_pio.py has top-level `import utime`/`machine`/`rp2`
# imports (MicroPython-only), and this module is also loaded PC-side by
# scripts/check_scenario.py and scripts/analyze_bidir_capture_log.py under
# plain CPython. These two numbers are as stable as they come (source of
# truth is driver/dshot_pio.py's own DSHOT_SPEEDS class and its
# bit_rate * 8_cycles_per_bit comment) - update both places together if they
# ever change.
DSHOT_SPEED_NAMES = {
    "DSHOT300": 2_400_000,  # 300,000 bit/s * 8 cycles/bit
    "DSHOT600": 4_800_000,  # 600,000 bit/s * 8 cycles/bit
}


class MotorSpec:
    def __init__(self, pin, sm_id, bidirectional, rx_sm_id, profile):
        self.pin = pin
        self.sm_id = sm_id
        self.bidirectional = bidirectional
        self.rx_sm_id = rx_sm_id
        self.profile = profile


class Scenario:
    def __init__(self, dshot_speed, duration_ms, arm_duration_ms,
                 status_interval_ms, poll_ms, decode_every, gc_every_ms, expect, motors,
                 core1_interval_us=None, arming_frame_gap_us=0):
        self.dshot_speed = dshot_speed
        self.duration_ms = duration_ms
        self.arm_duration_ms = arm_duration_ms
        self.status_interval_ms = status_interval_ms
        self.poll_ms = poll_ms
        self.decode_every = decode_every
        self.gc_every_ms = gc_every_ms
        self.expect = expect
        self.motors = motors
        # Overrides MotorGroup.UPDATE_INTERVAL_US for this run only (None = use the
        # library's own default). Exists to investigate BUG-002: it lets a scenario
        # reproduce a slower Core 1 tick (e.g. the one two bidirectional motors'
        # RX drain naturally produces) without actually wiring a second motor, to
        # separate "the tick period" from "a second bidirectional TX/RX pair" as
        # BUG-002's trigger. Not a knob for tuning real applications.
        self.core1_interval_us = core1_interval_us
        # Sets MotorGroup.arming_frame_gap_us for this run (0 = frames back to
        # back, the library's own behavior). A BUG-002 diagnostic: it spaces
        # the bidirectional motors' frames while arming, to test whether their
        # arriving close together decides whether an ESC accepts them.
        self.arming_frame_gap_us = arming_frame_gap_us

    @property
    def bidir_indices(self):
        return [i for i, m in enumerate(self.motors) if m.bidirectional]


def _require(data, key, context="scenario"):
    """
    Fail-fast field lookup: this is a test suite, not an application with
    reasonable runtime defaults, so a scenario JSON that omits a setting is a
    scenario mistake, not a signal to silently substitute a guessed value -
    every one of these controls something that changes test behavior (timing,
    sampling, thresholds), and a silently-defaulted one has already produced a
    scenario that looked passing for the wrong reason (see git history around
    2026-09-19's arm-duration investigation).
    """
    if key not in data:
        raise ValueError(context + " is missing required field: " + key)
    return data[key]


def pio_block(sm_id):
    # RP2350: state machine ids 0-3 -> PIO0, 4-7 -> PIO1, 8-11 -> PIO2, the same
    # mapping BidirectionalDShot.__init__ enforces. Checked here as well so a
    # scenario with a cross-block bidirectional pair is rejected when it is
    # loaded, before anything is constructed or armed.
    return sm_id // 4


def load_scenario(path):
    with open(path) as f:
        data = json.load(f)
    return build_scenario(data)


def build_scenario(data):
    dshot_speed_name = data["dshot_speed"]
    dshot_speed = DSHOT_SPEED_NAMES.get(dshot_speed_name)
    if dshot_speed is None:
        raise ValueError("unknown dshot_speed: " + str(dshot_speed_name))

    duration_ms = data["duration_ms"]

    motors_raw = data["motors"]
    if len(motors_raw) != 4:
        raise ValueError("scenario must declare exactly 4 motors, got " + str(len(motors_raw)))

    motors = []
    seen_pins = set()
    seen_sm_ids = set()
    for index, entry in enumerate(motors_raw):
        pin = entry["pin"]
        sm_id = entry["sm_id"]
        bidirectional = _require(entry, "bidirectional", "motor " + str(index))
        rx_sm_id = entry.get("rx_sm_id")

        if pin in seen_pins:
            raise ValueError("motor " + str(index) + ": duplicate pin " + str(pin))
        seen_pins.add(pin)

        if sm_id in seen_sm_ids:
            raise ValueError("motor " + str(index) + ": duplicate sm_id " + str(sm_id))
        seen_sm_ids.add(sm_id)

        if bidirectional:
            if rx_sm_id is None:
                raise ValueError("motor " + str(index) + ": rx_sm_id is required when bidirectional")
            if rx_sm_id in seen_sm_ids:
                raise ValueError("motor " + str(index) + ": duplicate sm_id " + str(rx_sm_id))
            seen_sm_ids.add(rx_sm_id)
            # Mirrors BidirectionalDShot.__init__'s own two checks exactly (see its
            # docstring) - failing here means a bad scenario JSON is caught
            # before any hardware is touched, rather than at motor
            # construction time. rx_sm_id must be sm_id+1: dshot_bidir_tx/
            # dshot_bidir_rx synchronize via RP2040/2350's relative IRQ
            # addressing, which gives every pair on a shared PIO block its
            # own private flag as long as the TX-to-RX id offset is this
            # fixed constant (see driver/dshot_pio.py). CONFIRMED on
            # hardware 2026-08-30: two bidirectional pairs sharing PIO0
            # (channel 1 sm0/rx1, channel 3 sm2/rx3) both produced 100%
            # CRC-valid, independent telemetry with distinct, plausible
            # eRPM values - captures/2026-08-30_21-09-16. No cross-block
            # restriction is needed beyond this fixed-offset requirement.
            if rx_sm_id != sm_id + 1:
                raise ValueError(
                    "motor " + str(index) + ": rx_sm_id must be sm_id + 1 "
                    "(got sm_id=" + str(sm_id) + ", rx_sm_id=" + str(rx_sm_id) + ")"
                )
            block = pio_block(sm_id)
            if block != pio_block(rx_sm_id):
                raise ValueError(
                    "motor " + str(index) + ": sm_id " + str(sm_id) + " and rx_sm_id " +
                    str(rx_sm_id) + " must share a PIO block (ids 0-3 -> PIO0, "
                    "4-7 -> PIO1, 8-11 -> PIO2) - sm_id " + str(sm_id) +
                    " is the last slot in its block, so sm_id+1 crosses into the next one"
                )
        elif rx_sm_id is not None:
            raise ValueError("motor " + str(index) + ": rx_sm_id set but bidirectional is false")

        profile_raw = entry.get("profile")
        if not profile_raw:
            raise ValueError("motor " + str(index) + ": profile is required (idle motors still "
                              "need an explicit hold-at-0 profile)")
        profile = ThrottleProfile(profile_raw)
        if profile.total_duration_ms != duration_ms:
            raise ValueError(
                "motor " + str(index) + ": profile duration " +
                str(profile.total_duration_ms) + "ms != scenario duration_ms " +
                str(duration_ms) + "ms"
            )

        motors.append(MotorSpec(pin, sm_id, bidirectional, rx_sm_id, profile))

    # A bidirectional motor's frame receiver fills its own PIO block alone
    # (dshot_bidir_tx + dshot_bidir_rx_frame use all 32 of the block's
    # instruction slots between them - see BidirectionalDShot's constructor
    # docstring), so a unidirectional motor sharing that block would fail
    # with ENOMEM at construction time. Caught here instead, before anything
    # is armed. Two bidirectional motors ARE allowed to share one block:
    # identical programs are loaded once per block, so a second pair adds
    # state machines but no instructions (see ADR-002's layout table).
    bidir_blocks = {pio_block(m.sm_id) for m in motors if m.bidirectional}
    for index, m in enumerate(motors):
        if not m.bidirectional and pio_block(m.sm_id) in bidir_blocks:
            raise ValueError(
                "motor " + str(index) + ": sm_id " + str(m.sm_id) + " shares PIO block " +
                str(pio_block(m.sm_id)) + " with a bidirectional motor - a bidirectional "
                "motor's frame receiver fills its own PIO block and cannot share it with a "
                "unidirectional motor's program"
            )

    # NOTE: expect.min_crc_valid_pct and expect.min_median_erpm are validated
    # here (shape + motor index) but NOT enforced on-device - run_scenario.py
    # deliberately does no GCR/CRC decoding during the run, because decoding on
    # the device costs milliseconds per capture. They are checked PC-side on the
    # pulled session by scripts/analyze_bidir_capture_log.py. The one on-device
    # reply check is a coarser, cheap proxy: check_reply_failsafe in
    # run_scenario.py, which only asserts "at least one non-all-zero reply
    # appeared".
    # Every Nth new capture per bidirectional motor is decoded on the device (0 = never).
    # Decoding a capture costs about 1.3ms, so only a sample of them is decoded on the
    # device; this is often enough for a percentage and rare enough to stay cheap.
    decode_every = _require(data, "decode_every")
    if not isinstance(decode_every, int) or decode_every < 0:
        raise ValueError("decode_every must be a whole number >= 0, got " + str(decode_every))

    # Core 0 forces a garbage collection this often (0 = never). A collection
    # pauses both cores, which is what heap churn does to a real application: it
    # shows whether a stall leaves lost or corrupted replies behind it.
    gc_every_ms = _require(data, "gc_every_ms")
    if not isinstance(gc_every_ms, int) or gc_every_ms < 0:
        raise ValueError("gc_every_ms must be a whole number >= 0, got " + str(gc_every_ms))

    core1_interval_us = data.get("core1_interval_us")
    if core1_interval_us is not None and (not isinstance(core1_interval_us, int) or core1_interval_us < 0):
        raise ValueError("core1_interval_us must be a whole number >= 0, got " + str(core1_interval_us))

    arming_frame_gap_us = data.get("arming_frame_gap_us", 0)
    if not isinstance(arming_frame_gap_us, int) or arming_frame_gap_us < 0:
        raise ValueError("arming_frame_gap_us must be a whole number >= 0, got " + str(arming_frame_gap_us))

    expect = _require(data, "expect")
    bidir_indices = {i for i, m in enumerate(motors) if m.bidirectional}
    for name in ("min_crc_valid_pct", "min_median_erpm"):
        for key in expect.get(name, {}):
            if int(key) not in bidir_indices:
                raise ValueError(
                    "expect." + name + " references motor " + str(key) +
                    ", which is not declared bidirectional"
                )

    return Scenario(
        dshot_speed=dshot_speed,
        duration_ms=duration_ms,
        arm_duration_ms=_require(data, "arm_duration_ms"),
        status_interval_ms=_require(data, "status_interval_ms"),
        poll_ms=_require(data, "poll_ms"),
        decode_every=decode_every,
        gc_every_ms=gc_every_ms,
        expect=expect,
        motors=motors,
        core1_interval_us=core1_interval_us,
        arming_frame_gap_us=arming_frame_gap_us,
    )
