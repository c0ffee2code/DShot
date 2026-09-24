# Established finding (2026-09-24), kept as a runnable record: once a GPIO has
# been passed to a StateMachine() constructor (as sideset_base/set_base/in_base),
# calling machine.Pin.init() on that GPIO number - even after the state machine
# is stop()ped, even via a brand-new Pin object - does NOT hand it back to plain
# GPIO/SIO control on this MicroPython/RP2350 build. PIO keeps driving the line
# regardless of what Pin.init() asks for.
#
# This was checked while investigating whether MotorGroup.disarm() could give an
# ESC an unambiguous "signal genuinely gone" condition by releasing the pin
# after stop() (see the two-bidirectional-motors stuck-ESC investigation). The
# experimental driver change this motivated (stop() calling pin.init(), start()
# reclaiming via StateMachine.init()) was reverted after this result: it was a
# no-op, so it fixed nothing and was pure added complexity.
#
# The proof is comparative: GPIO 10 (claimed by a state machine, then stopped)
# stays high under a forced pull-down, while GPIO 11 (never claimed by
# anything) responds to the same pull-down normally. If Pin.init() had actually
# detached GPIO 10 from PIO, it would behave like GPIO 11.
#
# No ESC, no motor: GPIO 10 and 11, nothing wired to either.

from machine import Pin
from dshot_pio import BidirectionalDShot, DSHOT_SPEEDS
import utime

PIN_NUM = 10


def report(label, pin):
    utime.sleep_us(50)
    print("   %-45s value=%d" % (label, pin.value()))


def main():
    print("=== Pin release quirk check (no ESC, GPIO %d) ===" % PIN_NUM)
    pin = Pin(PIN_NUM)
    motor = BidirectionalDShot(0, pin, DSHOT_SPEEDS.DSHOT300, rx_state_machine_id=1)
    motor.start()
    motor.send_throttle_command(0)
    utime.sleep_ms(2)
    motor.stop()  # already calls pin.init(Pin.IN, Pin.PULL_UP) - see dshot_pio.py

    print("Using the SAME Pin object stop() already released:")
    pin.init(Pin.IN, Pin.PULL_DOWN)
    report("same object, PULL_DOWN forced", pin)
    pin.init(Pin.IN, Pin.PULL_UP)
    report("same object, PULL_UP restored", pin)

    print("Using a FRESH Pin object on the same GPIO number:")
    fresh = Pin(PIN_NUM, Pin.IN, Pin.PULL_DOWN)
    report("fresh object, PULL_DOWN forced", fresh)
    fresh2 = Pin(PIN_NUM, Pin.IN, Pin.PULL_UP)
    report("fresh object, PULL_UP restored", fresh2)

    print("For comparison, a genuinely unclaimed GPIO (11, never given to any SM):")
    control = Pin(11, Pin.IN, Pin.PULL_DOWN)
    report("control pin 11, PULL_DOWN", control)
    control2 = Pin(11, Pin.IN, Pin.PULL_UP)
    report("control pin 11, PULL_UP", control2)

    print("=== Complete - compare GPIO 10's numbers above with GPIO 11's control ===")


main()
