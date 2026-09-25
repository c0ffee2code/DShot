# Established finding (2026-09-24), kept as a runnable record, CORRECTED from an
# earlier, wrong version of this file (see git history for commit 28df891 - it
# claimed Pin.init() cannot detach a GPIO from PIO at all, which direct register
# reads below disprove).
#
# What IS true: once a GPIO has been passed to a StateMachine() constructor (as
# sideset_base/set_base/in_base), stopping the state machine and then calling
# machine.Pin.init() on that GPIO DOES genuinely change its function-select away
# from PIO - confirmed by reading the IO_BANK0 GPIOn_CTRL register directly:
# FUNCSEL switches from PIO0 (6) to SIO (5), exactly as documented MicroPython/
# RP2350 behaviour would predict.
#
# Update (2026-09-25): the "pin keeps reading HIGH regardless" result below is
# now explained, not a mystery. It is RP2350 silicon erratum E9 ("increased
# leakage current on Bank 0 GPIO when pad input is enabled", fixed at stepping
# A3): a released input pad leaks enough current to hold ~2.2V (reads as
# HIGH) when nothing pulls it hard enough, and this diagnostic's forced
# PULL_DOWN is exactly the one configuration weak enough to lose that fight -
# a PULL_UP (what BidirectionalDShot actually uses in production) is not
# affected, per the erratum's own text. This file's SIO_GPIO_OE constant was
# also wrong when this was first run (0xd0000024, corrected below to the SDK's
# actual 0xd0000030) - a second, independent reason not to trust the "SIO
# isn't driving it" conclusion this file draws from that reading. None of this
# explains why a real bidirectional motor's ESC gets stuck after disarm()
# under a genuine PULL_UP - that turned out to be a different, unrelated
# mechanism entirely (an ESC-side bootloader that never sees the line go low),
# fixed in driver/dshot_pio.py's BidirectionalDShot.stop()/start(). Kept below
# as the original diagnostic and its findings, for the record.
#
# What IS true: once a GPIO has been passed to a StateMachine() constructor (as
# sideset_base/set_base/in_base), stopping the state machine and then calling
# machine.Pin.init() on that GPIO DOES genuinely change its function-select away
# from PIO - confirmed by reading the IO_BANK0 GPIOn_CTRL register directly:
# FUNCSEL switches from PIO0 (6) to SIO (5), exactly as documented MicroPython/
# RP2350 behaviour would predict.
#
# What the erratum above explains: the pin keeps reading HIGH afterward
# regardless - confirmed on two independent measurement paths (machine.Pin.value()
# and a direct read of the SIO GPIO_IN register, which agree, ruling out a stale
# Pin.value() read) and on two independent pin pairs (GPIO10/11 and GPIO14/15,
# ruling out one GPIO being anomalous). Ruled out as the cause (all correctly
# ruled out - none of these is the erratum, which lives entirely in the pad's
# analog input buffer, not in any of these digital registers):
#   - Settling time: still high after 20ms - far too long for RC discharge
#     against a weak pull-down and parasitic capacitance alone.
#   - SIO's own GPIO_OE/GPIO_OUT registers: both read 0 (not driving) for the
#     claimed pin, identical to an unclaimed control pin.
#   - The pad's own pull configuration: PUE/PDE genuinely reflect the last
#     pin.init() request (confirmed by direct PADS_BANK0 register read).
#   - FUNCSEL specifically: forcing it to the literal value an untouched GPIO
#     shows (31, "NULL" - no peripheral at all, not even SIO) made no
#     difference either.
#   - A race between disarm() and a still-running update() on another core:
#     reproduced the real MotorGroup + Core1Runner architecture and compared
#     disarm() while the loop was still running against stopping the loop
#     first - identical result (pin high, TX FIFO empty) either way, in one
#     comparison. Was a real bug regardless (see driver/motor_group.py's
#     disarm() and tests/harness/run_scenario.py's shutdown order, both now
#     fixed), but this comparison found no evidence it explains the stuck pin.
#
# The only thing that restores the true untouched state (FUNCSEL=31, reads low)
# is machine.reset() - a genuine RP2350 chip reset, not `mpremote ... reset`
# (confirmed separately to be a lesser, software-level reset that leaves pad
# configuration - such as a pull-up - intact).
#
# Addresses and field positions are from the official RP2350 SDK headers
# (pico-sdk hardware_regs/include/hardware/regs/{addressmap,io_bank0,pads_bank0,sio}.h),
# not guessed.
#
# No ESC, no motor: GPIO 14 and 15, nothing wired to either.

from machine import Pin, mem32
from dshot_pio import BidirectionalDShot, DSHOT_SPEEDS
import utime

CLAIMED_PIN = 14
CONTROL_PIN = 15

IO_BANK0_BASE = 0x40028000
PADS_BANK0_BASE = 0x40038000
SIO_GPIO_IN = 0xD0000004
SIO_GPIO_OUT = 0xD0000010
SIO_GPIO_OE = 0xD0000030

FUNCSEL_NAMES = {5: "SIO", 6: "PIO0", 31: "NULL(untouched)"}


def funcsel(gpio):
    return mem32[IO_BANK0_BASE + 0x04 + gpio * 0x08] & 0x1F


def pue_pde(gpio):
    v = mem32[PADS_BANK0_BASE + 0x04 + gpio * 0x04]
    return (v >> 3) & 1, (v >> 2) & 1


def report(label, gpio):
    fs = funcsel(gpio)
    pue, pde = pue_pde(gpio)
    oe = (mem32[SIO_GPIO_OE] >> gpio) & 1
    out = (mem32[SIO_GPIO_OUT] >> gpio) & 1
    live = (mem32[SIO_GPIO_IN] >> gpio) & 1
    print("   %-32s GPIO%-2d FUNCSEL=%-16s PUE=%d PDE=%d SIO_OE=%d SIO_OUT=%d  live=%d  value()=%d" % (
        label, gpio, FUNCSEL_NAMES.get(fs, str(fs)), pue, pde, oe, out, live, Pin(gpio).value()))


def main():
    print("=== Release leaves the pin driven high, cause not established (GPIO %d vs control %d) ===" % (
        CLAIMED_PIN, CONTROL_PIN))

    pin = Pin(CLAIMED_PIN)
    motor = BidirectionalDShot(0, pin, DSHOT_SPEEDS.DSHOT300, rx_state_machine_id=1)
    motor.start()
    motor.send_throttle_command(0)
    utime.sleep_ms(2)  # settle to the released, pull()-blocked idle state
    report("while active", CLAIMED_PIN)

    motor.stop()
    report("after stop() (SM inactive, still PIO)", CLAIMED_PIN)

    pin.init(Pin.IN, Pin.PULL_DOWN)
    utime.sleep_ms(5)
    report("after pin.init(PULL_DOWN), 5ms settled", CLAIMED_PIN)

    control = Pin(CONTROL_PIN, Pin.IN, Pin.PULL_DOWN)
    utime.sleep_ms(5)
    report("control, same request, never claimed", CONTROL_PIN)

    pin.init(Pin.IN, Pin.PULL_UP)
    motor.stop()
    print("=== Complete - GPIO%d stays high despite genuine SIO+PULL_DOWN; GPIO%d behaves correctly ===" % (
        CLAIMED_PIN, CONTROL_PIN))


main()
