# EXAMPLE APPLICATION CODE - not part of the DShot library.
#
# The DShot library deliberately does not decide which core runs the command
# loop (see decision/ADR-004-client-owned-command-loop.md). Core assignment is
# an architecture choice of the application, so it lives here, in the client.
#
# This particular arrangement dedicates Core 1 to the loop, which is what the
# test bench needs: Core 0 stays free for the display, buttons and control
# algorithms without ever starving the ESCs of commands.
#
# Copy this into your own project and adapt it - a project using a timer IRQ,
# uasyncio, or a cooperative main loop would write something different and
# still drive the same MotorGroup.update().

import _thread
import utime
from array import array


class Core1Runner:
    """
    Calls an update function on Core 1 at a fixed interval.

    Usage:
        runner = Core1Runner(motors.update)
        runner.start()
        ...
        if not runner.stop():
            print("Core 1 loop did not stop:", runner.error)
        elif runner.error:
            print("Core 1 died:", runner.error)
    """

    # How often stop() re-checks whether the thread has exited
    POLL_US = 200

    # How long stop() waits for the loop to leave update() and exit. Well above
    # the longest garbage-collection pause measured on Core 1 (14ms, ADR-002),
    # so a healthy loop always makes it and only a stuck one does not.
    STOP_TIMEOUT_MS = 200

    def __init__(self, update, interval_us=1000):
        """
        Args:
            update: Zero-argument callable to invoke on Core 1
            interval_us: Target period between calls (default: 1000us = 1kHz).
                0 means no fixed pacing at all (MotorGroup.UPDATE_INTERVAL_US's
                own meaning) - update() is called back to back, as fast as it
                returns.

                A positive value is an absolute period, held by anchoring
                each deadline to the *previous* one and sleeping only what is
                left of the current one - matching Betaflight's own
                fixed-period scheduler (`nextTargetCycles = lastTargetCycles
                + desiredPeriodCycles`, verified against its source
                2026-10-07). This makes the achieved period immune to
                update()'s own cost changing (a GC pause, more queued than
                usual, or - what actually happened in this project -
                removing diagnostic overhead; see BUG-002.md's "Telemetry
                overhead and the timed window" section specifically, not its
                "Root cause" section, which is a separate, larger fix), as
                long as that cost stays under interval_us. A call that still runs past its own next
                deadline is not caught up with a burst of immediate extra
                calls; the schedule skips forward by whole periods instead,
                matching Betaflight's own catch-up behavior (`scheduler.c`
                skips forward by whole periods too, never fires immediately).

                There used to be a second mode here (`fixed_schedule=False`):
                sleep a fixed pad *after* each call, so the achieved period
                was `interval_us + however long that call took`. Removed
                2026-10-09, by the user's explicit direction, not kept as an
                option: every value tuned for it had to be re-tuned the
                moment anything changed update()'s own cost - at one point it
                reached 100% clean arms on this bench, but that 100% was
                earned by a constant tuned to one specific, local measurement
                of one specific code version, not by a mechanism that stays
                correct as the code changes. The absolute-period mode above
                doesn't have that problem by construction. That pad-vs-
                deadline comparison ran under a harness bug found the next
                day (BUG-002.md's "Root cause" section), so it was never
                cleanly measured; the pad mode was never re-run after the
                fix. The deadline-anchored mode here has armed 20/20, then
                13/13, since.
        """
        self.update = update
        self.interval_us = interval_us
        self.running = False

        # How many times the schedule has fallen behind and had to skip
        # forward instead of holding the deadline - the one way this
        # mechanism can degrade toward unpaced, cost-dependent behavior: if
        # update() ever costs more than interval_us on a given tick, that
        # tick's gap is no longer exactly interval_us (see loop()'s own
        # comment for what it becomes instead). Zero cost when on schedule -
        # this only increments inside the branch that already runs on the
        # rare path. Reset by start(), like self.error - read it after
        # stop() for one run's count. Meaningless (stays 0) when
        # interval_us<=0 - there is no deadline to fall behind.
        self.schedule_behind_count = 0

        # Absolute ticks_us() of the first BEHIND_AT_CAPACITY fall-behind
        # events, oldest first (2026-10-08, BUG-002: testing whether these
        # events are disproportionately landing inside AM32's narrow
        # early-boot calibration window, not just "how often" they happen -
        # see bug-reports/BUG-002-...md). Same zero-cost-when-healthy shape
        # as schedule_behind_count; stale entries past schedule_behind_count
        # are meaningless, not re-zeroed on start() since the count already
        # gates every read.
        self.BEHIND_AT_CAPACITY = 8
        self.behind_at_us = array('I', [0] * self.BEHIND_AT_CAPACITY)

        # Set by the thread as it exits, so stop() can wait for the real event
        # rather than guessing at a sleep
        self.stopped = True

        # Exceptions on Core 1 never reach the REPL, so capture rather than
        # swallow - otherwise the loop dies silently
        self.error = None

    def start(self):
        """Start the Core 1 loop. No-op if already running."""
        if self.running:
            return

        self.error = None
        self.schedule_behind_count = 0
        self.stopped = False

        # Set before the thread exists, because loop() tests it immediately
        self.running = True

        try:
            _thread.start_new_thread(self.loop, ())
        except Exception as e:
            # Core 1 may still be busy with an earlier thread. Leaving running
            # set here would wedge the runner permanently: every later start()
            # would return at the guard above and update() would never be
            # called, surfacing downstream as an unexplained arming timeout.
            self.running = False
            self.stopped = True
            self.error = e
            raise

    def stop(self, timeout_ms=STOP_TIMEOUT_MS):
        """
        Stop the Core 1 loop and wait until it has actually exited.

        Returns True once the loop is out of update() and finished. Returns
        False if it was still running after timeout_ms, and sets self.error to
        say so: the caller then knows Core 1 may still be inside update(), and
        that disarm() would race it. A later stop() waits again.
        """
        self.running = False

        # Poll for the exit rather than sleeping blind: the loop leaves as soon
        # as update() returns and it re-checks running.
        for _ in range(timeout_ms * 1000 // self.POLL_US):
            if self.stopped:
                return True
            utime.sleep_us(self.POLL_US)

        if self.stopped:
            return True
        self.error = RuntimeError("Core 1 loop did not exit within " + str(timeout_ms) + "ms")
        return False

    def loop(self):
        """The Core 1 loop itself. Started by start(); do not call directly."""
        update = self.update
        interval_us = self.interval_us

        try:
            if interval_us > 0:
                ticks_us = utime.ticks_us
                ticks_add = utime.ticks_add
                ticks_diff = utime.ticks_diff
                sleep_us = utime.sleep_us
                next_due = ticks_us()
                behind_at_us = self.behind_at_us
                behind_at_capacity = self.BEHIND_AT_CAPACITY
                while self.running:
                    update()
                    next_due = ticks_add(next_due, interval_us)
                    now = ticks_us()
                    remaining = ticks_diff(next_due, now)
                    if remaining <= 0:
                        # Fell behind - skip forward by whole periods
                        # instead of resyncing to "now", which would fire
                        # the very next call with zero gap (a burst) -
                        # see BUG-002.md's "deadline underperforms the
                        # pad" entry for why that burst, not just a
                        # mistuned period, explained a real regression.
                        # Matches Betaflight's own catch-up behavior
                        # (scheduler.c skips forward by whole periods,
                        # never fires immediately).
                        count = self.schedule_behind_count
                        if count < behind_at_capacity:
                            behind_at_us[count] = now
                        self.schedule_behind_count = count + 1
                        while remaining <= 0:
                            next_due = ticks_add(next_due, interval_us)
                            remaining = ticks_diff(next_due, ticks_us())
                    sleep_us(remaining)
            else:
                # No fixed pacing at all - call back to back, as fast as
                # update() returns. There is no deadline to fall behind, so
                # schedule_behind_count stays at its initial 0.
                while self.running:
                    update()
        except Exception as e:
            self.error = e
            self.running = False
        finally:
            self.stopped = True


# How long fast_stop() lets the loop send throttle 0 before stopping it. At
# the loop's usual rate that is dozens of frames; an ESC acts on the first
# valid one.
FAST_STOP_SETTLE_MS = 20


def fast_stop(group, runner):
    """
    End a session with the motors stopped at once, then reset everything.

    MotorGroup.disarm() transmits nothing - it cuts the signal and leaves the
    ESC's own timeout (AM32: up to 500ms at the last throttle) to stop the
    motor. A session that wants the motor stopped immediately says so first:
    command throttle 0 and let the still-running loop send it, then stop the
    loop, then disarm. The order matters - throttle 0 only goes out while the
    loop runs, and the loop must be stopped before disarm() touches the state
    machines.

    If the loop has already died (runner.error), nothing can send the zeros;
    the ESC's timeout stops the motors, as for any lost signal.

    Returns whether the loop really exited (Core1Runner.stop()'s result).
    disarm() runs either way - cutting the signal comes first - but on False
    it ran while Core 1 may still have been inside update(), and the caller
    should report that rather than carry on as if the shutdown was clean.
    """
    if runner.running and runner.error is None:
        group.set_all_throttles([0] * group.motor_count)
        utime.sleep_ms(FAST_STOP_SETTLE_MS)
    exited = runner.stop()
    group.disarm()
    return exited
