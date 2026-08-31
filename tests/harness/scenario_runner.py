# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# Named for what it does - runs a scenario - not for the fact that it
# happens to capture telemetry along the way. Replaces
# bidir_capture_runner.py's BidirCaptureRunner, generalized from one fixed
# bidirectional channel (plus 3 always-zero TX-only motors) to exactly 4
# independently-throttled motors, any subset of which may be bidirectional -
# see tests/harness/scenario.py and tests/test_scenario_capture.py.
#
# Same "library doesn't own the core, application does" principle as
# core1_runner.py (see decision/ADR-004-client-owned-command-loop.md) - this
# is app/test-rig code, not driver code, so owning a thread here is fine.
#
# Copy this into your own project and adapt it, same as core1_runner.py.
#
# Every array in this module is a flat, individually-named attribute rather
# than a list-of-arrays indexed by motor, and the tick loop unrolls all 4
# motors explicitly instead of looping over MOTOR_COUNT. That's more verbose
# than the generic version this replaced, but it isn't stylistic: with a
# single bidirectional motor, an ESC reply completes on a MAJORITY of ticks
# (measured ~65% at DSHOT300), so the completed-record write path runs at
# nearly the 1kHz tick rate for a 3-minute hold. A `for m in range(4):
# word_bufs[m][k][slot] = ...` version of this (list-of-lists, double
# indirection, loop overhead) was measured on hardware to hold the achieved
# record rate to ~550-570/s, short of the original single-motor/6-field
# record design's ~650/s baseline. Flat arrays and an unrolled loop are what
# closed that gap - see the single_channel_baseline.json regression run.

import _thread
import utime
from array import array

MOTOR_COUNT = 4


class ScenarioRunner:
    """
    Drives exactly 4 DShot motors (any subset bidirectional) from a
    dedicated Core 1 thread, and buffers raw captures for Core 0 to drain.

    Usage:
        runner = ScenarioRunner(motors)  # motors: 4 DShotPIO instances
        runner.start()                   # begins sending throttle=0 immediately
        utime.sleep_ms(3000)             # arm window - continuous zero commands
        runner.set_throttle(0, 300)
        while ...:
            for record in runner.drain():
                (ticks_us, t0, t1, t2, t3,
                 m0w0, m0w1, m0w2, m0w3,
                 m1w0, m1w1, m1w2, m1w3,
                 m2w0, m2w1, m2w2, m2w3,
                 m3w0, m3w1, m3w2, m3w3) = record
                ...
        for i in range(4):
            runner.set_throttle(i, 0)
        utime.sleep_ms(300)
        runner.stop()
    """

    # How often stop() re-checks whether the thread has exited
    POLL_US = 200

    # Allowance on top of one interval, covering the time the loop spends
    # inside one send+drain cycle before it gets back to the running check
    STOP_GRACE_US = 4000

    def __init__(self, motors, interval_us=1000, ring_size=512):
        """
        Args:
            motors: Exactly 4 DShotPIO instances, in motor-index order. Any
                subset may be bidirectional=True - their rx_read() is
                drained every tick; do not call rx_read() on any of them
                from anywhere else once the runner is started.
            interval_us: Delay between send/drain cycles (default 1kHz).
            ring_size: Capacity of the raw-capture ring buffer. A capture is
                dropped (counted, not stored) if Core 0 falls behind by more
                than this many captures.
        """
        if len(motors) != MOTOR_COUNT:
            raise ValueError("ScenarioRunner needs exactly 4 motors, got " + str(len(motors)))

        self.motors = list(motors)
        self.bidir_indices = [i for i, m in enumerate(self.motors) if m.bidirectional]
        self.interval_us = interval_us

        self.throttles = array('H', [0] * MOTOR_COUNT)
        self.running = False
        self.stopped = True
        self.error = None

        # Ring buffer: single-producer (Core 1, this loop) / single-consumer
        # (Core 0, via drain()) - each index is only ever written by its own
        # side, same discipline as MotorThrottleGroup's shared throttle array
        # (see decision/ADR-001-dual-core-motor-control.md).
        #
        # One record per completed telemetry group (matching the original
        # single-channel design's semantics: a record represents a captured
        # reply, not an idle tick), carrying ALL 4 motors' throttle at that
        # instant plus whichever motor(s) actually completed a 4-word group
        # this tick - other motors' word slots are zero for that record.
        #
        # Flat named arrays, not a list-of-arrays indexed by motor - see
        # module docstring for why (this is the hot path).
        self.ring_size = ring_size
        self.ticks_buf = array('I', [0] * ring_size)
        self.t0_buf = array('H', [0] * ring_size)
        self.t1_buf = array('H', [0] * ring_size)
        self.t2_buf = array('H', [0] * ring_size)
        self.t3_buf = array('H', [0] * ring_size)
        self.m0w0_buf = array('I', [0] * ring_size)
        self.m0w1_buf = array('I', [0] * ring_size)
        self.m0w2_buf = array('I', [0] * ring_size)
        self.m0w3_buf = array('I', [0] * ring_size)
        self.m1w0_buf = array('I', [0] * ring_size)
        self.m1w1_buf = array('I', [0] * ring_size)
        self.m1w2_buf = array('I', [0] * ring_size)
        self.m1w3_buf = array('I', [0] * ring_size)
        self.m2w0_buf = array('I', [0] * ring_size)
        self.m2w1_buf = array('I', [0] * ring_size)
        self.m2w2_buf = array('I', [0] * ring_size)
        self.m2w3_buf = array('I', [0] * ring_size)
        self.m3w0_buf = array('I', [0] * ring_size)
        self.m3w1_buf = array('I', [0] * ring_size)
        self.m3w2_buf = array('I', [0] * ring_size)
        self.m3w3_buf = array('I', [0] * ring_size)
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

    def set_throttle(self, index, value):
        """
        Set one motor's throttle. Lock-free, safe to call from Core 0 while
        the loop runs on Core 1 - same atomic-write discipline as
        MotorThrottleGroup.set_throttle.
        """
        self.throttles[index] = value

    def drain(self):
        """
        Return a list of raw capture records queued since the last drain().

        Each record is a flat 21-tuple:
        (ticks_us, throttle0..3, motor0_w0..w3, motor1_w0..w3,
         motor2_w0..w3, motor3_w0..w3) - matches
        tests/harness/bidir_capture_sink.py's _RECORD_FMT field order exactly,
        so sink.write_record(*record) works directly.

        Call this from Core 0 as often as you like - it never blocks and
        never touches any motor's rx_read() directly. Not the hot path (Core
        0 calls this every poll_ms, not every tick), so no need to unroll.
        """
        records = []
        ring_size = self.ring_size
        ticks_buf = self.ticks_buf
        t_bufs = (self.t0_buf, self.t1_buf, self.t2_buf, self.t3_buf)
        w_bufs = (
            (self.m0w0_buf, self.m0w1_buf, self.m0w2_buf, self.m0w3_buf),
            (self.m1w0_buf, self.m1w1_buf, self.m1w2_buf, self.m1w3_buf),
            (self.m2w0_buf, self.m2w1_buf, self.m2w2_buf, self.m2w3_buf),
            (self.m3w0_buf, self.m3w1_buf, self.m3w2_buf, self.m3w3_buf),
        )
        while self.read_index < self.write_index:
            slot = self.read_index % ring_size
            row = [ticks_buf[slot]]
            for m in range(MOTOR_COUNT):
                row.append(t_bufs[m][slot])
            for m in range(MOTOR_COUNT):
                bufs = w_bufs[m]
                row.append(bufs[0][slot])
                row.append(bufs[1][slot])
                row.append(bufs[2][slot])
                row.append(bufs[3][slot])
            records.append(tuple(row))
            self.read_index += 1
        return records

    def loop(self):
        """The Core 1 loop itself. Started by start(); do not call directly."""
        # Bound once as 4 flat locals rather than indexed through a list
        # every tick - see module docstring. MOTOR_COUNT is always exactly
        # 4, never variable, so both the send step and the completed-record
        # write step below are fully unrolled rather than generic loops.
        motor0, motor1, motor2, motor3 = self.motors
        motors = (motor0, motor1, motor2, motor3)  # for the variable-length bidir_indices loop below
        bidir_indices = self.bidir_indices
        throttles = self.throttles
        interval_us = self.interval_us
        ring_size = self.ring_size

        ticks_buf = self.ticks_buf
        t0_buf, t1_buf, t2_buf, t3_buf = self.t0_buf, self.t1_buf, self.t2_buf, self.t3_buf
        m0w0_buf, m0w1_buf, m0w2_buf, m0w3_buf = self.m0w0_buf, self.m0w1_buf, self.m0w2_buf, self.m0w3_buf
        m1w0_buf, m1w1_buf, m1w2_buf, m1w3_buf = self.m1w0_buf, self.m1w1_buf, self.m1w2_buf, self.m1w3_buf
        m2w0_buf, m2w1_buf, m2w2_buf, m2w3_buf = self.m2w0_buf, self.m2w1_buf, self.m2w2_buf, self.m2w3_buf
        m3w0_buf, m3w1_buf, m3w2_buf, m3w3_buf = self.m3w0_buf, self.m3w1_buf, self.m3w2_buf, self.m3w3_buf

        pending = [[] for _ in range(MOTOR_COUNT)]

        try:
            while self.running:
                # Reassigning 4 plain locals to None costs nothing (no
                # allocation - None is a singleton) - this is NOT the same
                # as `completed_words = [None] * MOTOR_COUNT`, which DOES
                # allocate a fresh list every tick and was measured costing
                # enough overhead to visibly drag the achieved record rate
                # down. Resetting here, unconditionally, at the top of every
                # tick (rather than only after a completed-record write)
                # also closes a real bug: a stale reference could otherwise
                # survive into a later tick's write.
                w0 = w1 = w2 = w3 = None

                t0 = throttles[0]
                t1 = throttles[1]
                t2 = throttles[2]
                t3 = throttles[3]
                motor0.send_throttle_command(t0)
                motor1.send_throttle_command(t1)
                motor2.send_throttle_command(t2)
                motor3.send_throttle_command(t3)

                completed_any = False
                for i in bidir_indices:
                    motor = motors[i]
                    motor_pending = pending[i]
                    while True:
                        word = motor.rx_read()
                        if word is None:
                            break
                        motor_pending.append(word)
                        if len(motor_pending) == 4:
                            if i == 0:
                                w0 = motor_pending
                            elif i == 1:
                                w1 = motor_pending
                            elif i == 2:
                                w2 = motor_pending
                            else:
                                w3 = motor_pending
                            # Rebind to a NEW list, not just pending[i] - the
                            # OLD list object is now referenced by w0/w1/w2/w3
                            # above, and motor_pending would otherwise still
                            # point at it: if the FIFO has more than one
                            # reply queued up this tick, further appends
                            # would corrupt the group just captured instead
                            # of starting a fresh one.
                            pending[i] = motor_pending = []
                            completed_any = True

                if completed_any:
                    if self.write_index - self.read_index >= ring_size:
                        self.dropped += 1
                    else:
                        slot = self.write_index % ring_size
                        ticks_buf[slot] = utime.ticks_us() & 0xFFFFFFFF
                        t0_buf[slot] = t0
                        t1_buf[slot] = t1
                        t2_buf[slot] = t2
                        t3_buf[slot] = t3

                        if w0 is None:
                            m0w0_buf[slot] = 0
                            m0w1_buf[slot] = 0
                            m0w2_buf[slot] = 0
                            m0w3_buf[slot] = 0
                        else:
                            m0w0_buf[slot] = w0[0]
                            m0w1_buf[slot] = w0[1]
                            m0w2_buf[slot] = w0[2]
                            m0w3_buf[slot] = w0[3]

                        if w1 is None:
                            m1w0_buf[slot] = 0
                            m1w1_buf[slot] = 0
                            m1w2_buf[slot] = 0
                            m1w3_buf[slot] = 0
                        else:
                            m1w0_buf[slot] = w1[0]
                            m1w1_buf[slot] = w1[1]
                            m1w2_buf[slot] = w1[2]
                            m1w3_buf[slot] = w1[3]

                        if w2 is None:
                            m2w0_buf[slot] = 0
                            m2w1_buf[slot] = 0
                            m2w2_buf[slot] = 0
                            m2w3_buf[slot] = 0
                        else:
                            m2w0_buf[slot] = w2[0]
                            m2w1_buf[slot] = w2[1]
                            m2w2_buf[slot] = w2[2]
                            m2w3_buf[slot] = w2[3]

                        if w3 is None:
                            m3w0_buf[slot] = 0
                            m3w1_buf[slot] = 0
                            m3w2_buf[slot] = 0
                            m3w3_buf[slot] = 0
                        else:
                            m3w0_buf[slot] = w3[0]
                            m3w1_buf[slot] = w3[1]
                            m3w2_buf[slot] = w3[2]
                            m3w3_buf[slot] = w3[3]

                        self.write_index += 1

                utime.sleep_us(interval_us)
        except Exception as e:
            self.error = e
            self.running = False
        finally:
            self.stopped = True
