# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# Loads and validates a scenario JSON file (see tests/harness/scenarios/ and
# tests/test_scenario_capture.py) into a Scenario/MotorSpec object graph.
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
    def __init__(self, dshot_speed, duration_ms, arm_duration_ms, stop_duration_ms,
                 status_interval_ms, poll_ms, expect, motors):
        self.dshot_speed = dshot_speed
        self.duration_ms = duration_ms
        self.arm_duration_ms = arm_duration_ms
        self.stop_duration_ms = stop_duration_ms
        self.status_interval_ms = status_interval_ms
        self.poll_ms = poll_ms
        self.expect = expect
        self.motors = motors

    @property
    def bidir_indices(self):
        return [i for i, m in enumerate(self.motors) if m.bidirectional]


def _pio_block(sm_id):
    # RP2350: state machine ids 0-3 -> PIO0, 4-7 -> PIO1, 8-11 -> PIO2. Must
    # match BidirectionalDShot.__init__'s own rx_state_machine_id docstring exactly -
    # BidirectionalDShot itself does not validate this, so a scenario with a
    # cross-block bidir pair would otherwise fail confusingly (or silently
    # misbehave) only once armed.
    return sm_id // 4


def load_scenario(path):
    with open(path) as f:
        data = json.load(f)
    return _build_scenario(data)


def _build_scenario(data):
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
        bidirectional = entry.get("bidirectional", False)
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
            block = _pio_block(sm_id)
            if block != _pio_block(rx_sm_id):
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

    # NOTE: expect.min_crc_valid_pct is validated here (shape + motor index)
    # but is NOT enforced on-device - test_scenario_capture.py deliberately
    # does no GCR/CRC decoding during capture (that cost, 42-103ms/decode
    # worst case measured on-device, would wreck the achieved record rate).
    # It's PC-side, post-run provenance only: run scripts/
    # verify_gcr_decode_port.py or scripts/tally_period_cycles.py against
    # the pulled session afterward to actually check this threshold. The
    # one on-device runtime check that DOES exist is a coarser, cheap
    # proxy: _check_reply_failsafe in test_scenario_capture.py, which only
    # asserts "at least one non-all-zero reply appeared" - not a CRC-valid
    # percentage.
    expect = data.get("expect", {})
    bidir_indices = {i for i, m in enumerate(motors) if m.bidirectional}
    for key in expect.get("min_crc_valid_pct", {}):
        if int(key) not in bidir_indices:
            raise ValueError(
                "expect.min_crc_valid_pct references motor " + str(key) +
                ", which is not declared bidirectional"
            )

    return Scenario(
        dshot_speed=dshot_speed,
        duration_ms=duration_ms,
        arm_duration_ms=data.get("arm_duration_ms", 500),
        stop_duration_ms=data.get("stop_duration_ms", 300),
        status_interval_ms=data.get("status_interval_ms", 15000),
        poll_ms=data.get("poll_ms", 10),
        expect=expect,
        motors=motors,
    )
