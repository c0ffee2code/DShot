# Decodes the bidirectional DShot command frames dshot_bidir_tx puts on the line
# from a list of edges, (time, level) in 0.1 us units. The command-frame
# counterpart of driver/gcr_decode.py. Test code, not part of the library: pure
# Python shared by the wingman's PC tools and a pin test on the Pico.
#
# Waveform: 8 PIO cycles per bit, line idle high. Each bit is 2 cycles high, then
# low for 6 cycles (1) or 3 cycles (0); one falling edge per bit, so falling
# edges are a bit period apart and the low length gives the bit.

UNIT_NS = 100
BIT_UNITS = 16.667          # DSHOT600: 1.667us
ONE_LOW_UNITS = 12.5        # 6 of 8 cycles
ZERO_LOW_UNITS = 6.25       # 3 of 8 cycles
FRAME_GAP_UNITS = 200       # 20us without an edge ends a frame; far longer than any gap inside one
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
    """True when the 4 low bits are the XOR of the three nibbles above them, complemented for
    bidirectional DShot (inverted=True), plain for unidirectional."""
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
