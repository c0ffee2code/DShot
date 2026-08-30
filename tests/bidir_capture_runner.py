# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# Replaces the on-device live-decode soak tests (test_bidir_rx_soak.py,
# test_bidir_rx_soak_dual.py, both retired) with a dual-core split: Core 1
# owns ALL ESC communication (send commands, drain raw bidir RX words) and
# Core 0 is free to orchestrate - poll stats, and eventually (once the
# PicoBell SD+RTC breakout is wired - see bidirectional_dshot_review.md)
# write raw captures to SD. No GCR decoding happens on-device at all in this
# split; that moves to a PC-side pipeline built on
# scripts/dshot_bidir_decode.py.
#
# Same "library doesn't own the core, application does" principle as
# core1_runner.py (see decision/ADR-004-client-owned-command-loop.md) - this
# is app/test-rig code, not driver code, so owning a thread here is fine.
#
# Copy this into your own project and adapt it, same as core1_runner.py.

import _thread
import utime
from array import array


class BidirCaptureRunner:
    """
    Drives one bidirectional DShot channel (plus any number of TX-only
    motors held at zero) from a dedicated Core 1 thread, and buffers raw
    4-word RX captures for Core 0 to drain.

    Usage:
        runner = BidirCaptureRunner(bidir_motor, other_motors=[m2, m3, m4])
        runner.start()                  # begins sending throttle=0 immediately
        utime.sleep_ms(3000)            # arm window - continuous zero commands
        runner.set_throttle(300)
        while ...:
            for record in runner.drain():
                ticks_us, throttle, w0, w1, w2, w3 = record
                ...
        runner.set_throttle(0)
        utime.sleep_ms(300)
        runner.stop()
    """

    # How often stop() re-checks whether the thread has exited
    POLL_US = 200

    # Allowance on top of one interval, covering the time the loop spends
    # inside one send+drain cycle before it gets back to the running check
    STOP_GRACE_US = 4000

    def __init__(self, bidir_motor, other_motors=(), interval_us=1000, ring_size=512):
        """
        Args:
            bidir_motor: A bidirectional=True DShotPIO instance. Its
                rx_read() is drained every tick; do not call rx_read() on it
                from anywhere else once the runner is started.
            other_motors: TX-only DShotPIO instances driven at throttle 0
                every tick, so the rest of the bench stays powered/armed
                without contributing captures.
            interval_us: Delay between send/drain cycles (default 1kHz).
            ring_size: Capacity of the raw-capture ring buffer. A capture is
                dropped (counted, not stored) if Core 0 falls behind by more
                than this many captures.
        """
        self.bidir_motor = bidir_motor
        self.other_motors = list(other_motors)
        self.interval_us = interval_us

        self.throttle = 0
        self.running = False
        self.stopped = True
        self.error = None

        # Ring buffer: single-producer (Core 1, this loop) / single-consumer
        # (Core 0, via drain()) - each index is only ever written by its own
        # side, same discipline as MotorThrottleGroup's shared throttle array
        # (see decision/ADR-001-dual-core-motor-control.md).
        self.ring_size = ring_size
        self.ticks_buf = array('I', [0] * ring_size)
        self.throttle_buf = array('H', [0] * ring_size)
        self.w0_buf = array('I', [0] * ring_size)
        self.w1_buf = array('I', [0] * ring_size)
        self.w2_buf = array('I', [0] * ring_size)
        self.w3_buf = array('I', [0] * ring_size)
        self.write_index = 0
        self.read_index = 0
        self.dropped = 0

    def start(self):
        """Start the Core 1 loop. No-op if already running."""
        if self.running:
            return

        self.error = None
        self.stopped = False
        self.running = True

        try:
            _thread.start_new_thread(self.loop, ())
        except Exception as e:
            # Mirrors core1_runner.Core1Runner.start(): leaving running set
            # here would wedge the runner permanently if Core 1 is still
            # busy with an earlier thread.
            self.running = False
            self.stopped = True
            self.error = e
            raise

    def stop(self):
        """Stop the Core 1 loop and wait for the thread to exit."""
        if not self.running:
            return

        self.running = False

        for _ in range((self.interval_us + self.STOP_GRACE_US) // self.POLL_US):
            if self.stopped:
                return
            utime.sleep_us(self.POLL_US)

    def set_throttle(self, value):
        """
        Set the bidir motor's throttle. Lock-free, safe to call from Core 0
        while the loop runs on Core 1 - same atomic-write discipline as
        MotorThrottleGroup.set_throttle.
        """
        self.throttle = value

    def drain(self):
        """
        Return a list of raw capture records queued since the last drain().

        Each record is (ticks_us, throttle, word0, word1, word2, word3).
        Call this from Core 0 as often as you like - it never blocks and
        never touches the bidir motor's rx_read() directly.
        """
        records = []
        ring_size = self.ring_size
        ticks_buf = self.ticks_buf
        throttle_buf = self.throttle_buf
        w0_buf = self.w0_buf
        w1_buf = self.w1_buf
        w2_buf = self.w2_buf
        w3_buf = self.w3_buf
        while self.read_index < self.write_index:
            slot = self.read_index % ring_size
            records.append((
                ticks_buf[slot], throttle_buf[slot],
                w0_buf[slot], w1_buf[slot], w2_buf[slot], w3_buf[slot],
            ))
            self.read_index += 1
        return records

    def loop(self):
        """The Core 1 loop itself. Started by start(); do not call directly."""
        bidir_motor = self.bidir_motor
        other_motors = self.other_motors
        interval_us = self.interval_us
        ring_size = self.ring_size

        pending = []

        try:
            while self.running:
                throttle = self.throttle
                bidir_motor.send_throttle_command(throttle)
                for motor in other_motors:
                    motor.send_throttle_command(0)

                while True:
                    word = bidir_motor.rx_read()
                    if word is None:
                        break
                    pending.append(word)
                    if len(pending) == 4:
                        if self.write_index - self.read_index >= ring_size:
                            self.dropped += 1
                        else:
                            slot = self.write_index % ring_size
                            self.ticks_buf[slot] = utime.ticks_us() & 0xFFFFFFFF
                            self.throttle_buf[slot] = throttle
                            self.w0_buf[slot] = pending[0]
                            self.w1_buf[slot] = pending[1]
                            self.w2_buf[slot] = pending[2]
                            self.w3_buf[slot] = pending[3]
                            self.write_index += 1
                        pending = []

                utime.sleep_us(interval_us)
        except Exception as e:
            self.error = e
            self.running = False
        finally:
            self.stopped = True
