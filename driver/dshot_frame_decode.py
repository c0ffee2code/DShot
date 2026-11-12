# Reads bidirectional DShot command frames from a list of edges: the frames our transmitter
# (driver/dshot_pio.py, dshot_bidir_tx) put on the line, decoded from the waveform alone. The command-frame
# counterpart of gcr_decode.py (the ESC's reply frames).
#
# Not used by the library at runtime. It lives in driver/ because it is pure Python (no rp2 or machine) that two
# kinds of code need: the PC tools that read the wingman's observer recording (tests/bench/wingman/tools/observer_edge_log.py)
# and the pin test that runs on the Pico (tests/bench/pio_and_cores/test_wire_frames.py, edges from pin_sampler.py),
# the same reason gcr_decode.py and dshot_profiles.py are here. tests/pc exercises it on a PC.
#
# The waveform (8 PIO cycles per bit, the line idles high): every bit starts with 2 cycles high, then
# goes low; a "1" stays low for 6 cycles (75% of the bit), a "0" for 3 cycles (37.5%) and then goes
# high again. So each bit has exactly one falling edge, the falling edges are one bit period apart,
# and the length of the low part says the bit. Edges are (time, level) in units of 0.1us.

UNIT_NS = 100
BIT_UNITS = 16.667          # DSHOT600: 1.667us
ONE_LOW_UNITS = 12.5        # 6 of 8 cycles
ZERO_LOW_UNITS = 6.25       # 3 of 8 cycles
FRAME_GAP_UNITS = 200       # 20us without an edge ends a frame (a frame's own edges are at most 12.5 units apart)
FRAME_BITS = 16


def split_frames(edges, gap_units=FRAME_GAP_UNITS):
    frames = []
    current = []
    last = None
    for t, level in edges:
        if last is not None and t - last >= gap_units:
            frames.append(current)
            current = []
        current.append((t, level))
        last = t
    if current:
        frames.append(current)
    return frames


def crc_ok(word, inverted=True):
    """The CRC of a frame: the 4 low bits are the xor of the three nibbles above them, complemented for
    bidirectional DShot (inverted=True) and as it is for unidirectional."""
    value = word >> 4
    crc = (value ^ (value >> 4) ^ (value >> 8)) & 0xF
    return ((~crc & 0xF) if inverted else crc) == (word & 0xF)


def decode_frame(frame, inverted_crc=True):
    """One frame's edges -> dict. 'ok' is False for anything but a full 16-bit frame (a recording that starts
    or ends inside a frame holds partial ones). For a full frame: word, throttle, telemetry bit, crc_ok, the
    bit periods and the low lengths (in 0.1us units)."""
    falls = [t for t, level in frame if level == 0]
    rises = [t for t, level in frame if level == 1]
    if len(falls) != FRAME_BITS or frame[0][1] != 0:
        return {"ok": False, "falls": len(falls)}
    periods = [b - a for a, b in zip(falls, falls[1:])]
    lows = []
    r = 0
    for fall in falls:
        while r < len(rises) and rises[r] <= fall:
            r += 1
        if r == len(rises):
            return {"ok": False, "falls": len(falls)}
        lows.append(rises[r] - fall)
    mean_period = sum(periods) / len(periods)
    threshold = mean_period * (ONE_LOW_UNITS + ZERO_LOW_UNITS) / 2 / BIT_UNITS
    word = 0
    for low in lows:
        word = (word << 1) | (1 if low > threshold else 0)
    return {"ok": True, "start": falls[0], "word": word, "throttle": word >> 5, "telemetry": (word >> 4) & 1,
            "crc_ok": crc_ok(word, inverted_crc), "periods": periods, "lows": lows,
            "bits": [1 if low > threshold else 0 for low in lows]}


def decode_frames(edges, gap_units=FRAME_GAP_UNITS, inverted_crc=True):
    return [decode_frame(f, inverted_crc) for f in split_frames(edges, gap_units)]


class TimingStats:
    """Running bit-timing totals over decoded frames, in 0.1us units, without keeping the frames (a recording
    holds many and the Pico's heap is small)."""

    def __init__(self):
        self.frames = 0
        self.period_sum = 0
        self.period_count = 0
        self.period_min = None
        self.period_max = None
        self.one_sum = 0
        self.one_count = 0
        self.zero_sum = 0
        self.zero_count = 0

    def add(self, frame):
        if not frame["ok"]:
            return
        self.frames += 1
        for period in frame["periods"]:
            self.period_sum += period
            self.period_count += 1
            if self.period_min is None or period < self.period_min:
                self.period_min = period
            if self.period_max is None or period > self.period_max:
                self.period_max = period
        for bit, low in zip(frame["bits"], frame["lows"]):
            if bit:
                self.one_sum += low
                self.one_count += 1
            else:
                self.zero_sum += low
                self.zero_count += 1

    def summary(self):
        return {
            "frames": self.frames,
            "period_mean": self.period_sum / self.period_count if self.period_count else None,
            "period_min": self.period_min,
            "period_max": self.period_max,
            "one_low": self.one_sum / self.one_count if self.one_count else None,
            "zero_low": self.zero_sum / self.zero_count if self.zero_count else None,
        }


def timing_summary(frames):
    """Bit timing over decoded frames: mean/min/max of the bit period, and the mean low length of the 1 bits
    and of the 0 bits (0.1us units). None entries when there is nothing to average."""
    stats = TimingStats()
    for frame in frames:
        stats.add(frame)
    return stats.summary()
