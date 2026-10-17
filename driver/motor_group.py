# SPDX-License-Identifier: GPL-3.0-or-later
# MotorGroup: facade over the PIO state machines and throttle state
# of a group of DShot motors.
#
# See decision/ADR-004-client-owned-command-loop.md for the threading model
# See decision/ADR-001-dual-core-motor-control.md for the lock-free throttle store

import micropython
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
    # (bug-reports/BUG-002-...md): every reset seen came from
    # an ESC that had never replied - one that rejects every frame can still
    # arm (AM32 arms without validating them) and then time out - never from
    # one that had been replying.
    READY_SPAN_MS = 2000
    READY_GAP_MS = 450
    READY_FRESH_MS = 50

    # Highest throttle a motor will transmit; set_throttle() clamps to it
    MAX_THROTTLE = DShotPIO.MAX_THROTTLE
    # DShot values 1-47 are commands (beep, direction, EEPROM save, ...), not
    # throttle. Commands are out of scope for now, so set_throttle() accepts
    # 0 (stop) or MIN_THROTTLE and above only.
    MIN_THROTTLE = 48

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

        # Shared throttle array - lock-free access (atomic on ARM).
        # Using unsigned 16-bit integers ('H') for DShot throttle values.
        # See ADR-001: the application may write these from a different core
        # than the one calling update().
        self.throttles = array('H', [0] * self.motor_count)

        # One of DISARMED / ARMING / ARMED
        self.state = DISARMED

        # The reply gate (READY_SPAN_MS) applies while this is True. Only a
        # bench test with no ESC attached turns it off, so its bidirectional
        # motors arm on the window alone; an application leaves it on.
        self.wait_for_replies = True

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

        for motor in self.motors:
            motor.start()

        now = utime.ticks_ms()
        self.arm_duration_ms = duration_ms
        self.arm_started_ms = now
        self.last_update_ms = now
        for i in range(self.motor_count):
            self.ready_seen[i] = False

        # Set last: update() must not run before the state machines are active
        self.state = ARMING

    def disarm(self):
        """
        Stop all motors and return them to a fresh, never-run state.

        Nothing is transmitted: each motor's state machines are deactivated
        and re-initialised (empty queues, pin driven low), so the signal line
        stops carrying DShot transitions and the ESC's own signal-loss timeout
        stops the motor and resets it (AM32: 500ms armed, specification/AM32.md
        #15). Until then the motor holds its last throttle. An application
        that needs the motor stopped sooner sets throttle 0 and lets update()
        send it before calling disarm().

        Does not block for more than a few hundred microseconds per
        bidirectional motor (a fixed wait for an in-flight ESC reply).

        Its own state check keeps a concurrent update() from corrupting the
        group's state, but nothing serialises this method's state-machine
        calls against an update() still running on another core - stop that
        loop first (see the class docstring's usage example). A word such an
        update() queues after the reset is discarded by the next start().

        Idempotent. Call arm() to bring the group back up.
        """
        # Cleared first, so a concurrent update() bails before we start cutting
        # the signal. The prior value also tells us whether the state machines
        # are live: a group that never armed, or was already disarmed, has
        # nothing to reset.
        was_live = self.state != DISARMED
        self.state = DISARMED

        for i in range(self.motor_count):
            self.throttles[i] = 0

        if was_live:
            for motor in self.motors:
                motor.stop()

        # Re-assert. An update() already past its state check when we started
        # may have promoted the group to ARMED behind us. Leaving ARMED set
        # over inactive state machines is what makes the next update() block
        # forever. This narrows the race rather than closing it - stopping the
        # loop first is what closes it (see the docstring above).
        self.state = DISARMED

    # measured win - see ADR-002's "Further hot-path tightening" section
    @micropython.native
    def update(self):
        """
        Drain each bidirectional motor's reply FIFO, send one DShot command to
        each motor, and advance the arming sequence.

        The application calls this at least every UPDATE_INTERVAL_US, from
        whichever core or scheduling arrangement it chooses. Draining here is
        deliberate: an RX FIFO left undrained stalls the receiver, losing
        replies until it is drained again, so it must not depend on the
        application remembering a second call. Replies taken while ARMING
        are discarded - not yet trusted, since the ESC may still be
        completing bidirectional detection.

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
            motors = self.motors
            for i in self.bidir_indices:
                if motors[i].discard_rx():
                    if not self.ready_seen[i]:
                        self.ready_first_ms[i] = now
                    else:
                        gap = utime.ticks_diff(now, self.ready_last_ms[i])
                        if gap > self.READY_GAP_MS:
                            self.ready_first_ms[i] = now
                    self.ready_last_ms[i] = now
                    self.ready_seen[i] = True

            # Send literal zeros rather than the throttle array, so the arming
            # window stays genuinely at zero even if the application sets a
            # throttle early. send_zero() (not send_throttle_command(0)): the
            # packet is invariant, so it is precomputed once rather than
            # rebuilt and CRC'd on every tick (W33).
            for i in range(self.motor_count):
                motors[i].send_zero()

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
                motor.drain_rx()

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

        Returns that motor's latest_capture() - one integer, the reply frame
        with a token above it that steps on every publish - or None while the
        group is not ARMED or nothing has arrived yet. A caller that keeps the
        last value it saw knows a capture is new when the value differs; it
        forgets that value on every arm(), because each run's tokens start
        over at 1. The group stores nothing itself - it only refuses to
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

    def decode_telemetry(self, motor_index, value):
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

        return motor.decode_capture(value)

    def clamp_throttle(self, value):
        """
        Limit a throttle to what a motor will transmit: 0, or MIN_THROTTLE to
        MAX_THROTTLE. Out-of-range values are clamped; 1..MIN_THROTTLE-1 is
        DShot's command range, which this driver does not offer, and is
        rejected rather than clamped (BUG-005).
        """
        if value < 0:
            return 0
        if value > self.MAX_THROTTLE:
            return self.MAX_THROTTLE
        if 0 < value < self.MIN_THROTTLE:
            raise MotorGroupException(
                "Throttle 1-47 is the DShot command range, not supported"
            )
        return value

    def set_throttle(self, motor_index, value):
        """
        Set throttle for a single motor.

        Args:
            motor_index: Motor index (0-based)
            value: 0 (stop) or MIN_THROTTLE-MAX_THROTTLE (48-2047). Values
                outside 0-2047 are clamped; 1-47 raises MotorGroupException.

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

        # Validate everything before writing anything, so a rejected value
        # does not leave the group half-updated.
        clamped = [self.clamp_throttle(value) for value in values]
        for i, value in enumerate(clamped):
            self.throttles[i] = value

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
