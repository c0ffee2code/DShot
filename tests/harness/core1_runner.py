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


class Core1Runner:
    """
    Calls an update function on Core 1 at a fixed interval.

    Usage:
        runner = Core1Runner(motors.update)
        runner.start()
        ...
        runner.stop()
        if runner.error:
            print("Core 1 died:", runner.error)
    """

    # How often stop() re-checks whether the thread has exited
    POLL_US = 200

    # Allowance on top of one interval, covering the time the loop spends
    # inside update() before it gets back to the running check
    STOP_GRACE_US = 4000

    def __init__(self, update, interval_us=1000):
        """
        Args:
            update: Zero-argument callable to invoke on Core 1
            interval_us: Delay between calls (default: 1000us = 1kHz)
        """
        self.update = update
        self.interval_us = interval_us
        self.running = False

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

    def stop(self):
        """Stop the Core 1 loop and wait for the thread to exit."""
        if not self.running:
            return

        self.running = False

        # loop() tests running once per interval, so one interval plus the
        # grace is the bound. Poll for the exit rather than sleeping blind,
        # which is only long enough when interval_us is left at its default.
        for _ in range((self.interval_us + self.STOP_GRACE_US) // self.POLL_US):
            if self.stopped:
                return
            utime.sleep_us(self.POLL_US)

    def loop(self):
        """The Core 1 loop itself. Started by start(); do not call directly."""
        update = self.update
        interval_us = self.interval_us

        try:
            while self.running:
                update()
                utime.sleep_us(interval_us)
        except Exception as e:
            self.error = e
            self.running = False
        finally:
            self.stopped = True
