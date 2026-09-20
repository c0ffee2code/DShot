# Stand-ins for the MicroPython hardware modules (machine, rp2, utime), so that
# driver/ imports and runs on a PC. Import this module before anything from
# driver/: it puts driver/ on sys.path and installs the fakes.
#
# What the fakes do is only as much as the unit tests need:
#   - Clock is a time source the test moves by hand, so arming windows and gaps
#     can be exercised without waiting.
#   - StateMachine records every word put() on its TX FIFO, and serves words a
#     test feeds it through the RX side, the way the mailbox and drain read them.
#   - asm_pio returns the decorated function unrun: the PIO programs are not
#     assembled here, so nothing about PIO timing is checked on a PC.

import sys
import types
from pathlib import Path

DRIVER = Path(__file__).resolve().parents[2] / "driver"
if str(DRIVER) not in sys.path:
    sys.path.insert(0, str(DRIVER))

TICKS_MASK = 0x3FFFFFFF  # MicroPython's ticks wrap at 2**30


class Clock:
    """The fake time source. Time only moves when a test (or sleep_*) moves it."""

    microseconds = 0

    @classmethod
    def reset(cls):
        cls.microseconds = 0

    @classmethod
    def advance_ms(cls, milliseconds):
        cls.microseconds += milliseconds * 1000


def ticks_diff(a, b):
    diff = (a - b) & TICKS_MASK
    return diff - (TICKS_MASK + 1) if diff >= (TICKS_MASK + 1) // 2 else diff


def _install_utime():
    module = types.ModuleType("utime")
    module.ticks_ms = lambda: (Clock.microseconds // 1000) & TICKS_MASK
    module.ticks_us = lambda: Clock.microseconds & TICKS_MASK
    module.ticks_diff = ticks_diff
    module.sleep_us = lambda n: setattr(Clock, "microseconds", Clock.microseconds + n)
    module.sleep_ms = lambda n: setattr(Clock, "microseconds", Clock.microseconds + n * 1000)
    sys.modules["utime"] = module


class Pin:
    IN = 0
    OUT = 1
    PULL_UP = 2

    def __init__(self, pin_id, *args, **kwargs):
        self.id = pin_id

    def init(self, *args, **kwargs):
        pass

    # Real pins with the same number are the same object; two motors on one
    # pin must compare equal for the group's collision check
    def __eq__(self, other):
        return isinstance(other, Pin) and other.id == self.id

    def __hash__(self):
        return hash(self.id)


class StateMachine:
    def __init__(self, state_machine_id, program=None, freq=None, **kwargs):
        self.id = state_machine_id
        self.program = program
        self.freq = freq
        self.kwargs = kwargs
        self.is_active = False
        self.restarts = 0
        self.sent = []      # every word ever put(), in order
        self.pending = []   # words the fake transmitter has not yet "sent"
        self.rx = []        # words a test fed for the program to have "captured"

    def active(self, value=None):
        if value is None:
            return self.is_active
        self.is_active = bool(value)

    def restart(self):
        self.restarts += 1

    # put(value, shift): the value is shifted left by `shift` bits into a 32-bit word
    def put(self, value, shift=0):
        word = (value << shift) & 0xFFFFFFFF
        self.sent.append(word)
        self.pending.append(word)

    # A poll finds the transmitter has taken everything queued so far
    def tx_fifo(self):
        waiting = len(self.pending)
        self.pending = []
        return waiting

    def feed(self, words):
        self.rx.extend(words)

    def rx_fifo(self):
        return len(self.rx)

    def get(self, buffer=None):
        if buffer is None:
            return self.rx.pop(0)
        if len(self.rx) < len(buffer):
            raise AssertionError("get() would have blocked: %d words waiting, %d wanted" % (
                len(self.rx), len(buffer)))
        for i in range(len(buffer)):
            buffer[i] = self.rx.pop(0)


class PIO:
    OUT_LOW = 0
    OUT_HIGH = 1
    IN_LOW = 2
    SHIFT_LEFT = 0
    SHIFT_RIGHT = 1
    JOIN_NONE = 0
    JOIN_TX = 1
    JOIN_RX = 2


def asm_pio(**kwargs):
    def decorate(function):
        return function
    return decorate


def _install_hardware_modules():
    machine = types.ModuleType("machine")
    machine.Pin = Pin
    sys.modules["machine"] = machine

    rp2 = types.ModuleType("rp2")
    rp2.PIO = PIO
    rp2.StateMachine = StateMachine
    rp2.asm_pio = asm_pio
    sys.modules["rp2"] = rp2


# The fakes replace the real modules only when those are absent, which is always
# the case on a PC
if "utime" not in sys.modules:
    _install_utime()
if "machine" not in sys.modules:
    _install_hardware_modules()
