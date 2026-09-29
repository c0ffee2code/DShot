# SPDX-License-Identifier: GPL-3.0-or-later
# MotorGroup: facade over the PIO state machines and throttle state
# of a group of DShot motors.
#
# See decision/ADR-004-client-owned-command-loop.md for the threading model
# See decision/ADR-001-dual-core-motor-control.md for the lock-free throttle store

import utime
from array import array

from dshot_pio import DShotPIO, UnsupportedOperationException

# Lifecycle states, as reported by MotorGroup.state
DISARMED = 0
ARMING = 1
ARMED = 2


class MotorGroupException(Exception):
    def __init__(self, message):
        self.message = message


class MotorGroup:
    """
    Facade for controlling throttle on a group of DShot motors.

    This class owns the throttle values and the lifecycle of the motors it is
    given. The motors themselves (UnidirectionalDShot or BidirectionalDShot,
    one per motor, 1 to 4 of them) are built by the application, which picks
    their state machines and pins. It does NOT own a command loop: the application decides which core, thread, timer
    or main loop calls update(), because that is an architecture choice of the
    application, not of this library.

    The only requirement is that update() is called at least every
    UPDATE_INTERVAL_US while armed - ESCs disarm if commands stop arriving.

    Usage (application runs the loop on Core 1):
        from machine import Pin
        from dshot_pio import UnidirectionalDShot, BidirectionalDShot, DSHOT_SPEEDS
        from motor_group import MotorGroup

        group = MotorGroup([
            UnidirectionalDShot(0, Pin(4), DSHOT_SPEEDS.DSHOT600),
            BidirectionalDShot(2, Pin(5), DSHOT_SPEEDS.DSHOT600, rx_state_machine_id=3),
        ])

        runner = Core1Runner(group.update)  # application-supplied, see tests/
        runner.start()

        group.arm()
        while not group.is_armed():         # arming is non-blocking
            utime.sleep_ms(10)

        group.set_throttle(0, 100)
        group.set_throttle(1, 150)

        runner.stop()          # stop the loop before disarm() - see disarm()'s own docstring
        group.disarm()

    Usage (application pumps update() from its own main loop):
        group.arm()
        while True:
            group.update()
            ...application work, kept under UPDATE_INTERVAL_US...
            utime.sleep_us(group.UPDATE_INTERVAL_US)
    """

    # How often the application must call update(). 0 means "as fast as
    # possible, no delay between calls", which is the rate the facade has been
    # hardware-verified at (see the "Verified Parameters" table in README.md).
    UPDATE_INTERVAL_US = 0

    # Default arming window in milliseconds: the least time ARMING lasts. For a
    # unidirectional motor it is all there is - nothing comes back to observe -
    # and 2000ms covers AM32's own gate from a cold boot (>1s zero-throttle
    # requirement, plus its 600ms startup tune, plus margin). A group with
    # bidirectional motors also waits for the reply gate below, so for it this
    # is only a floor.
    DEFAULT_ARM_DURATION_MS = 2000

    # Arming gate for bidirectional motors (bug-reports/BUG-003): ARMED also
    # needs every bidirectional ESC to have replied AM32's not-running frame
    # for READY_SPAN_MS, with no gap between replies longer than READY_GAP_MS,
    # the latest at most READY_FRESH_MS ago. What BUG-002 caught was an ESC
    # that had not yet accepted our frames at all when a timer-only ARMED
    # fired regardless, then rebooted (a silent stretch of >=680ms - its
    # ~600ms startup tune plus bidirectional latch - visible as a held-low
    # line) after ARMED had already sent it non-zero throttle, so it could
    # never satisfy its own zero-throttle arming gate afterward. The gap
    # tolerates the arming tune's ~300ms silence but restarts the count on a
    # reboot's longer one, and the span is sized to comfortably outlast one
    # full reboot cycle before trusting the streak. A reply says the ESC is
    # listening, not that it armed or that the motor will start - that is
    # still not observable here. Bench-verified across 28 runs
    # (bug-reports/BUG-002-...md's "Verification"): every reset seen came from
    # an ESC that had never replied - one that rejects every frame can still
    # arm (AM32 arms without validating them) and then time out - never from
    # one that had been replying.
    READY_SPAN_MS = 2000
    READY_GAP_MS = 450
    READY_FRESH_MS = 50

    # Diagnostic (BUG-002): most reboot-length gaps (see READY_GAP_MS) logged
    # per bidirectional motor in one arming attempt - see reboot_log(). Not a
    # ring: an entry past this is silently dropped, since no run so far has
    # needed more than 4.
    REBOOT_LOG_CAPACITY = 16

    # Highest throttle a motor will transmit; set_throttle() clamps to it
    MAX_THROTTLE = DShotPIO.MAX_THROTTLE

    # Zero-throttle frames disarm() transmits before cutting the signal. One
    # commands the stop; the rest are margin against a frame lost to noise.
    DISARM_FRAMES = 4

    # Pause between disarm()'s zero-frame rounds, so a bidirectional motor's
    # own next frame does not start while its ESC is still driving the
    # previous one's reply - a real bus contention, not just a corrupted
    # capture (specification/AM32_ARMING_AND_BETAFLIGHT.md, B3: the ESC's
    # reply drives the line for ~48us at DSHOT600, ~78us at DSHOT300, on top
    # of the frame itself). disarm() is not hot-path, so one constant that
    # comfortably covers both speeds costs nothing.
    DISARM_ROUND_GAP_US = 200

    # Motors a group can drive: one ESC's worth
    MIN_MOTORS = 1
    MAX_MOTORS = 4

    def __init__(self, motors):
        """
        Initialize motor group.

        The motors' state machines stay inactive - arm() activates them.

        Args:
            motors: List of 1 to 4 UnidirectionalDShot / BidirectionalDShot
                instances, one per motor, in motor-index order. Bidirectional
                ones must have been built as such from the start: an ESC only
                detects bidirectional DShot during arming.
        """
        if len(motors) < self.MIN_MOTORS or len(motors) > self.MAX_MOTORS:
            raise MotorGroupException(
                "Expected 1 to 4 motors, got " + str(len(motors))
            )

        # The application chose each motor's state machines and pin, so check
        # they do not collide: two motors on one state machine would silently
        # replace each other, and two on one pin would fight over the line.
        # A bidirectional motor uses two state machines.
        used_state_machines = []
        used_pins = []
        for motor in motors:
            ids = [motor.state_machine_id]
            if motor.bidirectional:
                ids.append(motor.rx_state_machine_id)
            for state_machine_id in ids:
                if state_machine_id in used_state_machines:
                    raise MotorGroupException(
                        "State machine " + str(state_machine_id) + " is used by more than one motor"
                    )
                used_state_machines.append(state_machine_id)
            if motor.pin in used_pins:
                raise MotorGroupException("Two motors share the same pin")
            used_pins.append(motor.pin)

        self.motor_count = len(motors)
        self.motors = list(motors)

        # The subset whose reply must be drained on every update(), resolved
        # once so the command loop does no per-tick type checks
        self.bidir_motors = [m for m in self.motors if m.bidirectional]
        self.bidir_indices = [i for i in range(self.motor_count) if self.motors[i].bidirectional]

        # The arming gate's evidence, per motor (bidirectional ones only): when
        # its current unbroken run of not-running replies began and when the
        # latest arrived (ticks_ms), and whether one has arrived since arm()
        self.ready_first_ms = [0] * self.motor_count
        self.ready_last_ms = [0] * self.motor_count
        self.ready_seen = [False] * self.motor_count

        # Diagnostic (BUG-002): ground-truth log of reboot-length gaps seen by
        # the arming gate above, per motor - see reboot_log(). Arrays sized
        # for every motor for simplicity; only bidirectional ones ever get an
        # entry.
        self.reboot_log_ms = [array('I', [0] * self.REBOOT_LOG_CAPACITY) for _ in range(self.motor_count)]
        self.reboot_log_gap_ms = [array('I', [0] * self.REBOOT_LOG_CAPACITY) for _ in range(self.motor_count)]
        self.reboot_log_count = [0] * self.motor_count

        # Shared throttle array - lock-free access (atomic on ARM).
        # Using unsigned 16-bit integers ('H') for DShot throttle values.
        # See ADR-001: the application may write these from a different core
        # than the one calling update().
        self.throttles = array('H', [0] * self.motor_count)

        # One of DISARMED / ARMING / ARMED
        self.state = DISARMED

        # Diagnostic: when True, replies drained while ARMING are published to
        # each bidirectional motor's latest capture instead of being dropped.
        # raw_telemetry() still hands nothing out before ARMED; a caller that
        # wants the arming-phase replies reads motor.latest_capture() itself.
        # The bench harness sets it to log what the ESC did while arming
        # (bug-reports/BUG-002). Set it before arm().
        self.publish_while_arming = False

        # The reply gate (READY_SPAN_MS) applies while this is True. Only a
        # bench test with no ESC attached turns it off, so its bidirectional
        # motors arm on the window alone; an application leaves it on.
        self.wait_for_replies = True

        # Diagnostic, BUG-002: while ARMING, wait this many microseconds after
        # each bidirectional motor's frame before sending the next motor's
        # (not after the last one). 0, the default, sends every frame back to
        # back. The bench harness sets it to test whether two bidirectional
        # lines' frames arriving close together decides whether an ESC accepts
        # them; an application leaves it at 0.
        self.arming_frame_gap_us = 0
        self.gap_after_indices = self.bidir_indices[:-1]

        # Diagnostic (BUG-002): when set (a bin width in microseconds; 0, the
        # default, disables it), arm() has every bidirectional motor's
        # CaptureMailbox classify every capture it drains - not just the one
        # another core happens to poll - into not_running/zero("low")/other,
        # bucketed over time (CaptureMailbox.enable_class_bins()). Ground
        # truth immune to sampling loss; read back via each motor's own
        # mailbox.class_bins. The bench harness sets it; an application
        # leaves it at 0.
        self.arming_class_bin_width_us = 0
        self.arming_class_bin_count = 300

        self.arm_duration_ms = self.DEFAULT_ARM_DURATION_MS
        self.arm_started_ms = 0
        self.last_update_ms = utime.ticks_ms()

    def arm(self, duration_ms=DEFAULT_ARM_DURATION_MS):
        """
        Begin arming all ESCs.

        Activates the PIO state machines and starts the arming window. This
        does NOT block: arming completes inside update(), so the application
        must be calling update() for arming to progress. Poll is_armed().

        A group with bidirectional motors arms when the window has elapsed AND
        every bidirectional ESC has been replying for READY_SPAN_MS (see the
        constant) - usually ~2.1s after arm(), several seconds if an ESC was
        mid-reboot. An ESC that never replies keeps the group ARMING, sending
        zeros, for good: how long to wait is the application's decision
        (arming_status() says which motor is missing).

        Any throttle set before arm() is discarded - arming always starts
        from zero.

        Only valid while disarmed. Restarting the motors under a live command
        loop would flush their RX FIFOs and reset their published telemetry
        mid-write and snap the throttles to zero, so calling it while ARMING or
        ARMED raises MotorGroupException; disarm() first to start over.

        Args:
            duration_ms: Arming duration (default: DEFAULT_ARM_DURATION_MS)
        """
        if self.state != DISARMED:
            raise MotorGroupException("arm() called while already arming or armed")

        for i in range(self.motor_count):
            self.throttles[i] = 0

        # Before start(): start() calls each mailbox's reset(), which is what
        # timestamps class_bin_t0_us - it must already have an array to time,
        # or the first arm() in a session times nothing (class_bins stays
        # None through that reset()) and bins its captures from ticks_us's
        # zero point (boot) instead of from this arm().
        if self.arming_class_bin_width_us:
            for motor in self.bidir_motors:
                motor.mailbox.enable_class_bins(self.arming_class_bin_width_us, self.arming_class_bin_count)

        for motor in self.motors:
            motor.start()

        if self.arming_class_bin_width_us:
            # Overrides the per-motor t0 each mailbox's own reset() (inside
            # start(), above) just set from its own clock() reading. Left
            # alone, two bidirectional motors' bins would not even share a
            # zero point: start() runs one motor at a time, and reset()'s
            # own zeroing loop (900 array elements) measurably delays the
            # next motor's start() - see bug-reports/BUG-002-...md's
            # "instrumentation moved the arming sequence" section. One shared
            # reading, taken once every motor is up, makes every bidirectional
            # motor's bin N the same wall-clock window, and matches
            # arm_started_ms below closely enough for a 100ms-scale bin.
            sync_us = utime.ticks_us()
            for motor in self.bidir_motors:
                motor.mailbox.class_bin_t0_us = sync_us

        now = utime.ticks_ms()
        self.arm_duration_ms = duration_ms
        self.arm_started_ms = now
        self.last_update_ms = now
        for i in range(self.motor_count):
            self.ready_seen[i] = False
            self.reboot_log_count[i] = 0

        # Set last: update() must not run before the state machines are active
        self.state = ARMING

    def disarm(self):
        """
        Stop all motors and disarm the ESCs.

        Commands zero throttle, waits for those frames to reach the ESCs, then
        deactivates the PIO state machines so the signal line stops carrying
        DShot transitions and the ESC cannot spin.

        This is the emergency stop. It transmits the zeros itself rather than
        leaving them for update(), so it works even when the application's
        command loop is dead - and it stops the motors in about a millisecond
        instead of waiting out the ESC's signal-loss timeout, which is over a
        hundred times longer.

        Blocks for a bit over a millisecond: the zero frames themselves, plus
        DISARM_ROUND_GAP_US between each round so a bidirectional motor's own
        frames do not run into its ESC's reply (see that constant).

        Its own state check keeps a concurrent update() from corrupting the
        group's state, but nothing serialises this method's FIFO/state-machine
        calls against an update() still running on another core - stop that
        loop first (see the class docstring's usage example) rather than
        relying on disarm() to tolerate a still-running caller.

        Idempotent. Call arm() to bring the group back up.
        """
        # Cleared first, so a concurrent update() bails before we start cutting
        # the signal. The prior value also tells us whether the state machines
        # are live - putting to an inactive one fills the TX FIFO and then
        # blocks the caller forever.
        was_live = self.state != DISARMED
        self.state = DISARMED

        for i in range(self.motor_count):
            self.throttles[i] = 0

        if was_live:
            # One round at a time, paced: sending all DISARM_FRAMES rounds
            # back to back (as a full TX_FIFO_DEPTH burst) let only the first
            # round's frame land cleanly on a bidirectional motor - the rest
            # arrived while its own ESC was still driving that frame's reply
            # (B3). The gap is skipped after the last round; drain() below
            # already waits out whatever is still in flight.
            for round_index in range(self.DISARM_FRAMES):
                for motor in self.motors:
                    motor.send_throttle_command(0)
                if round_index < self.DISARM_FRAMES - 1:
                    utime.sleep_us(self.DISARM_ROUND_GAP_US)

            # Cutting the signal before the zeros are on the wire would leave
            # the motors spinning at their last commanded throttle
            for motor in self.motors:
                motor.drain()

            # Only meaningful on a state machine that was actually live: a
            # bidirectional motor's stop() always pays a fixed settle delay to
            # let an in-flight ESC reply finish, which cannot be in flight on
            # a motor that never transmitted (never armed) or was already
            # stopped by an earlier disarm() call.
            for motor in self.motors:
                motor.stop()

        # Re-assert. An update() already past its state check when we started
        # may have promoted the group to ARMED behind us; by now it has long
        # returned, because transmitting the zeros above took far longer than
        # a single update() call. Leaving ARMED set over inactive state
        # machines is what makes the next update() block forever.
        self.state = DISARMED

    def update(self):
        """
        Drain each bidirectional motor's reply FIFO, send one DShot command to
        each motor, and advance the arming sequence.

        The application calls this at least every UPDATE_INTERVAL_US, from
        whichever core or scheduling arrangement it chooses. Draining here is
        deliberate: an RX FIFO left undrained stalls the receiver, losing
        replies until it is drained again, so it must not depend on the
        application remembering a second call. Replies drained while ARMING
        are discarded - not yet trusted, since the ESC may still be
        completing bidirectional detection - unless publish_while_arming is
        set (a diagnostic; raw_telemetry() withholds them either way).

        The drain comes first, before any command is queued, so a new reply's
        words never have to wait behind an undrained old one - see ADR-002
        and ADR-005 for what draining after sending cost with several
        bidirectional motors on one command loop.

        Does nothing while disarmed, so it is always safe to call - including
        before arm() or after disarm(), when the state machines are inactive
        and writing to them would eventually block on a full TX FIFO.
        """
        state = self.state
        if state == DISARMED:
            return

        now = utime.ticks_ms()

        if state == ARMING:
            # A not-running reply extends the motor's run of replies, or starts
            # a new one after a gap long enough to be a reboot
            publish = self.publish_while_arming
            motors = self.motors
            for i in self.bidir_indices:
                if motors[i].drain_rx(publish):
                    if not self.ready_seen[i]:
                        self.ready_first_ms[i] = now
                    else:
                        gap = utime.ticks_diff(now, self.ready_last_ms[i])
                        if gap > self.READY_GAP_MS:
                            self.ready_first_ms[i] = now
                            # Diagnostic (BUG-002): ground truth for when a
                            # reboot-length gap happened and how long it was -
                            # see reboot_log().
                            count = self.reboot_log_count[i]
                            if count < self.REBOOT_LOG_CAPACITY:
                                self.reboot_log_ms[i][count] = utime.ticks_diff(now, self.arm_started_ms)
                                self.reboot_log_gap_ms[i][count] = gap
                                self.reboot_log_count[i] = count + 1
                    self.ready_last_ms[i] = now
                    self.ready_seen[i] = True

            # Send literal zeros rather than the throttle array, so the arming
            # window stays genuinely at zero even if the application sets a
            # throttle early
            gap_us = self.arming_frame_gap_us
            for i in range(self.motor_count):
                motors[i].send_throttle_command(0)
                if gap_us and i in self.gap_after_indices:
                    utime.sleep_us(gap_us)

            if (utime.ticks_diff(now, self.arm_started_ms) >= self.arm_duration_ms
                    and self.bidir_ready(now)):
                # Re-read rather than promoting from the snapshot above: a
                # disarm() on another core may have landed since, and writing
                # ARMED over it would leave the group "armed" with inactive
                # state machines - the case that makes put() block forever
                if self.state == ARMING:
                    self.state = ARMED
        else:
            for motor in self.bidir_motors:
                motor.drain_rx(True)

            throttles = self.throttles
            motors = self.motors
            for i in range(self.motor_count):
                motors[i].send_throttle_command(throttles[i])

        self.last_update_ms = now

    def bidir_ready(self, now):
        """
        True when every bidirectional motor passes the arming gate at `now`
        (ticks_ms): replying for READY_SPAN_MS without a longer gap than
        READY_GAP_MS, the latest reply at most READY_FRESH_MS old. True for a
        group without bidirectional motors, or with wait_for_replies off.
        """
        if not self.wait_for_replies:
            return True
        for i in self.bidir_indices:
            if not self.ready_seen[i]:
                return False
            if utime.ticks_diff(now, self.ready_first_ms[i]) < self.READY_SPAN_MS:
                return False
            if utime.ticks_diff(now, self.ready_last_ms[i]) > self.READY_FRESH_MS:
                return False
        return True

    def arming_status(self):
        """
        What the arming gate has seen, per motor, for an application's
        arming-timeout message: None for a unidirectional motor and for a
        bidirectional one with no reply since arm(), else
        (replying_for_ms, last_reply_ms_ago). Safe to call from another core;
        the two numbers may come from different ticks.
        """
        now = utime.ticks_ms()
        status = []
        for i in range(self.motor_count):
            if self.ready_seen[i]:
                status.append((utime.ticks_diff(now, self.ready_first_ms[i]),
                               utime.ticks_diff(now, self.ready_last_ms[i])))
            else:
                status.append(None)
        return status

    def reboot_log(self, motor_index):
        """
        Diagnostic (BUG-002): every reset this motor's arming gate or its
        low-line signature has recorded since arm(), oldest first, as
        (ms_since_arm, duration_ms, source). Two independent sources, merged:

        - "gap": a reboot-length gap between not_running replies (see
          READY_GAP_MS) - this is the arming gate's own bookkeeping,
          recorded live as it happened, and it is what actually restarts
          the gate's reply-span requirement (see arming_frame_gap_us's
          neighbour, bidir_ready()). It can only fire once this motor has
          replied at least once - there is no streak yet to interrupt
          before that.
        - "low": a run of zero("low")-classified captures ending - AM32's
          startup-tune signature (the line held low) observed directly from
          CaptureMailbox.enable_class_bins(), whether or not this motor has
          ever replied. This is the only source that can catch a reset
          before first contact; empty unless arming_class_bin_width_us was
          set for this arm().

        The two can both appear for one physical reboot, at different times -
        "low" is timestamped at the held-low run's start (the tune
        beginning), "gap" at when the reply streak noticed it was missing a
        reply (the tune's end plus AM32's own bidirectional-latch delay, per
        bug-reports/BUG-002-...md) - so "gap"'s ms_since_arm is always later
        than "low"'s for the same event, and "gap"'s duration includes both
        the tune and that latch. Always empty for a unidirectional motor.

        A third label, "low-open", can appear last: a low streak still
        running right now, not yet closed by a non-low capture. This is
        exactly BUG-002's "reboot once, then never resolve" refusal shape -
        without it, a refusal's reboot_log() would stay empty even though
        the line has in fact been held low the whole time. Its duration is
        how long the streak has run as of this call, not a fixed count.
        """
        count = self.reboot_log_count[motor_index]
        ms = self.reboot_log_ms[motor_index]
        gap = self.reboot_log_gap_ms[motor_index]
        entries = [(ms[i], gap[i], "gap") for i in range(count)]
        motor = self.motors[motor_index]
        if motor.bidirectional:
            mailbox = motor.mailbox
            reset_count = mailbox.reset_log_count
            entries += [(mailbox.reset_log_ms[i], mailbox.reset_log_gap_ms[i], "low")
                        for i in range(reset_count)]
            if mailbox.in_low_streak:
                start_ms = (mailbox.low_streak_start_us - mailbox.class_bin_t0_us) // 1000
                duration_ms = (mailbox.clock() - mailbox.low_streak_start_us) // 1000
                entries.append((start_ms, duration_ms, "low-open"))
        entries.sort()
        return entries

    def is_armed(self):
        """
        Returns:
            True once arming has completed - the window has elapsed and every
            bidirectional ESC has passed the reply gate (see arm()) - and
            throttle commands are being sent
        """
        return self.state == ARMED

    def is_arming(self):
        """
        Returns:
            True while the arming window is still in progress
        """
        return self.state == ARMING

    def raw_telemetry(self, motor_index):
        """
        Latest raw telemetry capture for one bidirectional motor.

        Returns (ticks_us, sequence, words) from that motor's
        latest_capture(), or None while the group is not ARMED or nothing has
        arrived yet. The group stores nothing itself - it only refuses to
        hand out captures taken while its own arming window was still open,
        not yet trusted since the ESC may still be completing bidirectional
        detection.

        ARMED means every bidirectional ESC was replying when the group
        armed, not that it still is: an ESC that resets later goes silent, and
        the captures handed out can then be echoes or noise. The CRC check in
        decode_telemetry() is what tells a real reply from those, and a
        CRC-failed capture should be discarded and asked for again later -
        retry timing is the application's decision.

        Safe to call from a different core than update().

        Raises UnsupportedOperationException for a unidirectional motor,
        whatever the state: asking one for telemetry is an application bug.
        """
        if motor_index < 0 or motor_index >= self.motor_count:
            raise MotorGroupException(
                "Invalid motor index: " + str(motor_index)
            )

        motor = self.motors[motor_index]
        if not motor.bidirectional:
            raise UnsupportedOperationException("Motor " + str(motor_index) + " is unidirectional")

        if self.state != ARMED:
            return None

        return motor.latest_capture()

    def decode_telemetry(self, motor_index, words):
        """
        Decode one capture returned by raw_telemetry(); see
        BidirectionalDShot.decode_capture() for the result and its cost.
        Call it at whatever pace the application can afford, never from the
        command loop. Raises UnsupportedOperationException for a
        unidirectional motor.
        """
        if motor_index < 0 or motor_index >= self.motor_count:
            raise MotorGroupException(
                "Invalid motor index: " + str(motor_index)
            )

        motor = self.motors[motor_index]
        if not motor.bidirectional:
            raise UnsupportedOperationException("Motor " + str(motor_index) + " is unidirectional")

        return motor.decode_capture(words)

    def clamp_throttle(self, value):
        """Limit a throttle to the range a motor will transmit, 0 to MAX_THROTTLE."""
        if value < 0:
            return 0
        if value > self.MAX_THROTTLE:
            return self.MAX_THROTTLE
        return value

    def set_throttle(self, motor_index, value):
        """
        Set throttle for a single motor.

        Args:
            motor_index: Motor index (0-based)
            value: Throttle value (0-2047), clamped to range

        Accepted while disarmed, but nothing is transmitted until the group is
        armed, and arm() resets all throttles to zero.

        Lock-free: safe to call from a different core than the one calling
        update(). The write is atomic (see ADR-001).
        """
        if motor_index < 0 or motor_index >= self.motor_count:
            raise MotorGroupException(
                "Invalid motor index: " + str(motor_index)
            )

        self.throttles[motor_index] = self.clamp_throttle(value)

    def set_all_throttles(self, values):
        """
        Set throttles for all motors.

        Args:
            values: List/tuple of throttle values, one per motor

        Each write is atomic but the batch is not atomic as a whole.
        For flight control this is acceptable (see ADR-001).
        """
        if len(values) != self.motor_count:
            raise MotorGroupException(
                "Expected " + str(self.motor_count) +
                " values, got " + str(len(values))
            )

        for i, value in enumerate(values):
            self.throttles[i] = self.clamp_throttle(value)

    def get_all_throttles(self):
        """
        Get current throttle values as a plain list.

        The throttles array itself is readable directly; this converts it for
        printing and comparison.
        """
        return [self.throttles[i] for i in range(self.motor_count)]

    def update_age_ms(self):
        """
        Milliseconds since update() last transmitted.

        This library does not own the command loop, so it cannot judge whether
        that loop is healthy - it only reports the fact. The application sets
        its own threshold, which should be well under the ESC's disarm timeout.

        Only meaningful while armed or arming; update() does not transmit, and
        so does not refresh this, while disarmed.
        """
        return utime.ticks_diff(utime.ticks_ms(), self.last_update_ms)
