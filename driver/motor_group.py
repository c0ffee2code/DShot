# SPDX-License-Identifier: GPL-3.0-or-later
# MotorGroup, the library's public API: throttle values and arming lifecycle
# of a group of DShot motors, driven by a command loop the application owns.
# Design: ADR-001, ADR-004, ADR-005; arming gate: BUG-003.

import micropython
import utime
from array import array

from dshot_pio import UnsupportedOperationException

# Lifecycle states, as reported by MotorGroup.state
INACTIVE = 0
ARMING = 1
ARMED = 2


class MotorGroupException(Exception):
    def __init__(self, message):
        self.message = message


class MotorGroup:
    """
    Throttle values and arming lifecycle of 1 to 4 DShot motors.

    The application builds the motors (UnidirectionalDShot or
    BidirectionalDShot, choosing their state machines and pins) and owns the
    command loop: it calls update() every UPDATE_INTERVAL_US from whichever
    core, timer or main loop it chooses. Keep the period steady by anchoring
    each deadline to the previous one; there is no unpaced mode, and an ESC
    stops when frames stop arriving. arm() starts arming, update() completes
    it, is_armed() reports it. Stopping the motors is the application's job:
    it sets throttle 0 and keeps update() running; deactivate() only ends the
    signal. Throttles may be set from another core than the one calling
    update(). Examples: README.md, "Quick Start".
    """

    # Period at which the application calls update(), in us (README, "Verified Parameters")
    UPDATE_INTERVAL_US = 1045

    # Least time ARMING lasts, in ms; all a unidirectional-only group waits for (README, "Verified Parameters")
    DEFAULT_ARM_DURATION_MS = 2000

    # Reply gate, in ms: before ARMED every bidirectional ESC has replied
    # AM32's not-running frame
    READY_SPAN_MS = 2000   # for at least this long,
    READY_GAP_MS = 450     # without a longer gap (a longer one restarts the span),
    READY_FRESH_MS = 50    # the latest at most this long ago

    # TEMPORARY: highest throttle set_throttle() accepts until higher ones are tested (BUG-005)
    THROTTLE_CEILING = 250
    # Lowest non-zero throttle; 1-47 are DShot commands, which this class does not send
    MIN_THROTTLE = 48

    # Motors a group can drive: one ESC's worth
    MIN_MOTORS = 1
    MAX_MOTORS = 4

    def __init__(self, motors):
        """
        Take over the motors; their state machines stay inactive until arm().

        Args:
            motors: 1 to 4 UnidirectionalDShot / BidirectionalDShot instances,
                in motor-index order. A bidirectional motor must be built as
                one: AM32 detects bidirectional DShot only while disarmed.

        Raises MotorGroupException for another count, or for two motors
        sharing a state machine or a pin.
        """
        if len(motors) < self.MIN_MOTORS or len(motors) > self.MAX_MOTORS:
            raise MotorGroupException(
                "Expected 1 to 4 motors, got " + str(len(motors))
            )

        # Two motors on one state machine would replace each other; on one pin
        # they would fight over the line. A bidirectional motor uses two.
        used_state_machines = []
        used_pins = []
        for motor in motors:
            for state_machine_id in motor.state_machine_ids:
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

        # Resolved once, so update() does no per-tick type checks
        self.bidir_motors = [m for m in self.motors if m.mailbox is not None]
        self.bidir_indices = [i for i in range(self.motor_count) if self.motors[i].mailbox is not None]

        # Reply-gate evidence per bidirectional motor, in ticks_ms: start of the
        # current run of not-running replies, the latest reply, any since arm()
        self.ready_first_ms = [0] * self.motor_count
        self.ready_last_ms = [0] * self.motor_count
        self.ready_seen = [False] * self.motor_count

        # Written from any core, read by update(): one element's write is atomic
        self.throttles = array('H', [0] * self.motor_count)

        # One of INACTIVE / ARMING / ARMED
        self.state = INACTIVE

        self.arm_duration_ms = self.DEFAULT_ARM_DURATION_MS
        self.arm_started_ms = 0
        self.last_update_ms = utime.ticks_ms()

    def arm(self, duration_ms=DEFAULT_ARM_DURATION_MS):
        """
        Start arming: throttles to 0, motors activated, arming window opened.

        Returns at once; arming completes inside update(), so the command loop
        must run. Poll is_armed(). ARMED needs duration_ms elapsed and every
        bidirectional ESC through the reply gate (READY_SPAN_MS). A reply shows
        the ESC listening, not armed. An ESC that never replies keeps the group
        ARMING, sending zeros, for good: the application decides how long to
        wait, and arming_status() names the missing motor.

        Args:
            duration_ms: least time ARMING lasts.

        Raises MotorGroupException unless INACTIVE; deactivate() first to start over.
        """
        if self.state != INACTIVE:
            raise MotorGroupException("arm() called while already arming or armed")

        for i in range(self.motor_count):
            self.throttles[i] = 0

        for motor in self.motors:
            motor.activate()

        now = utime.ticks_ms()
        self.arm_duration_ms = duration_ms
        self.arm_started_ms = now
        self.last_update_ms = now
        for i in range(self.motor_count):
            self.ready_seen[i] = False

        # Set last: update() must not run before the state machines are active
        self.state = ARMING

    def deactivate(self):
        """
        End the signal to every motor and leave the group in its fresh,
        never-run state. Idempotent.

        Does not stop the motors. Nothing is transmitted: the state machines
        are deactivated and reset and the lines driven low, and each ESC holds
        its last throttle until its signal-loss timeout (AM32: 500 ms armed,
        specification/AM32.md#15). To stop them, set throttle 0 and let
        update() send it first. Blocks a few hundred microseconds per
        bidirectional motor. Stop the command loop first: nothing serialises
        this against an update() running on another core.
        """
        # Cleared first, so a concurrent update() returns before the signal is
        # cut. The prior value says whether there are live motors to reset.
        was_live = self.state != INACTIVE
        self.state = INACTIVE

        for i in range(self.motor_count):
            self.throttles[i] = 0

        if was_live:
            for motor in self.motors:
                motor.deactivate()

        # Again: an update() past its state check may have set ARMED meanwhile,
        # and ARMED over inactive state machines blocks the next update() for
        # good. This narrows the race; stopping the loop first closes it.
        self.state = INACTIVE

    @micropython.native
    def update(self):
        """
        One command-loop tick: drain bidirectional replies, send one frame to
        every motor, advance arming.

        Call it every UPDATE_INTERVAL_US, at a steady period. The drain is part
        of the tick, and comes before the sends, because an undrained RX FIFO
        stalls the receiver. While ARMING every motor is sent 0 whatever its
        throttle, and replies are only counted for the reply gate, never
        published. Does nothing while inactive, so it is safe before arm() and
        after deactivate().
        """
        state = self.state
        if state == INACTIVE:
            return

        now = utime.ticks_ms()

        if state == ARMING:
            # A not-running reply extends the motor's run of replies, or starts
            # a new run after a gap longer than READY_GAP_MS
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

            # Zeros, not the throttle array: arming stays at 0 even if a
            # throttle is set early
            for i in range(self.motor_count):
                motors[i].send_zero()

            if (utime.ticks_diff(now, self.arm_started_ms) >= self.arm_duration_ms
                    and self.bidir_ready(now)):
                # Re-read, not the snapshot: a deactivate() on another core may have
                # landed since, and ARMED over it would make put() block for good
                if self.state == ARMING:
                    self.state = ARMED
        else:
            for motor in self.bidir_motors:
                motor.drain_rx()

            # No range check: set_throttle() and set_all_throttles() validated
            throttles = self.throttles
            motors = self.motors
            for i in range(self.motor_count):
                motors[i].send_frame(throttles[i])

        self.last_update_ms = now

    def bidir_ready(self, now):
        """
        True when every bidirectional motor passes the arming gate at `now`
        (ticks_ms): replying for READY_SPAN_MS without a longer gap than
        READY_GAP_MS, the latest reply at most READY_FRESH_MS old. True for a
        group without bidirectional motors.
        """
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
        """True once arming has completed and update() sends the set throttles."""
        return self.state == ARMED

    def is_arming(self):
        """True between arm() and ARMED."""
        return self.state == ARMING

    def raw_telemetry(self, motor_index):
        """
        Latest capture of one bidirectional motor, or None while the group is
        not ARMED or nothing has been captured.

        The value is the motor's latest_capture(): the frame with a token above
        it that changes on every publish, so a value that differs from the last
        one seen is new. Tokens start over on every arm(); forget the last value
        then. A capture is not proof of a reply: with a silent ESC (one that
        reset after ARMED, say) the receiver captures the driver's own next
        frame, and at some throttles that capture passes the CRC in
        decode_telemetry(). Telling the two apart is the application's job.

        Safe to call from another core than update(). Raises
        UnsupportedOperationException for a unidirectional motor, whatever the
        state, and MotorGroupException for an index out of range.
        """
        if motor_index < 0 or motor_index >= self.motor_count:
            raise MotorGroupException(
                "Invalid motor index: " + str(motor_index)
            )

        motor = self.motors[motor_index]
        if motor.mailbox is None:
            raise UnsupportedOperationException("Motor " + str(motor_index) + " is unidirectional")

        if self.state != ARMED:
            return None

        return motor.latest_capture()

    def decode_telemetry(self, motor_index, value):
        """
        Decode one capture returned by raw_telemetry(); returns
        BidirectionalDShot.decode_capture()'s dict. Too slow for the command
        loop: call it at the application's own pace. Raises
        UnsupportedOperationException for a unidirectional motor.
        """
        if motor_index < 0 or motor_index >= self.motor_count:
            raise MotorGroupException(
                "Invalid motor index: " + str(motor_index)
            )

        motor = self.motors[motor_index]
        if motor.mailbox is None:
            raise UnsupportedOperationException("Motor " + str(motor_index) + " is unidirectional")

        return motor.decode_capture(value)

    def validate_throttle(self, value):
        """
        Return value unchanged if a motor will transmit it: 0 (stop), or
        MIN_THROTTLE to THROTTLE_CEILING. Anything else raises
        MotorGroupException: bad input is rejected, never adjusted. Below
        MIN_THROTTLE lies DShot's command range, which this class does not
        send; THROTTLE_CEILING is the temporary bench ceiling.
        """
        if value != 0 and not self.MIN_THROTTLE <= value <= self.THROTTLE_CEILING:
            raise MotorGroupException(
                "Throttle must be 0 or " + str(self.MIN_THROTTLE) + ".." + str(self.THROTTLE_CEILING) +
                ", got " + str(value)
            )
        return value

    def set_throttle(self, motor_index, value):
        """
        Set throttle for a single motor.

        Args:
            motor_index: Motor index (0-based)
            value: 0 (stop) or MIN_THROTTLE-THROTTLE_CEILING (48-250, a
                temporary bench ceiling). Negative values, 1-47 and anything
                above the ceiling raise MotorGroupException.

        Accepted while inactive, but nothing is transmitted until the group is
        armed, and arm() resets all throttles to zero.

        Lock-free: safe to call from another core than the one calling
        update(), as the write is atomic.
        """
        if motor_index < 0 or motor_index >= self.motor_count:
            raise MotorGroupException(
                "Invalid motor index: " + str(motor_index)
            )

        self.throttles[motor_index] = self.validate_throttle(value)

    def set_all_throttles(self, values):
        """
        Set throttles for all motors.

        Args:
            values: List/tuple of throttle values, one per motor, each valid
                as for set_throttle(); any invalid one raises
                MotorGroupException and nothing is written.

        Each write is atomic, the batch is not: update() may send some new
        values and some old ones in one tick.
        """
        if len(values) != self.motor_count:
            raise MotorGroupException(
                "Expected " + str(self.motor_count) +
                " values, got " + str(len(values))
            )

        # All checked before any is written, so a rejected value leaves nothing half-updated
        checked = [self.validate_throttle(value) for value in values]
        for i, value in enumerate(checked):
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
        Milliseconds since update() last sent frames (or since arm(), before
        its first tick). Not refreshed while inactive.

        A fact, not a health verdict: the application sets its own threshold,
        well under the ESC's signal-loss timeout.
        """
        return utime.ticks_diff(utime.ticks_ms(), self.last_update_ms)
