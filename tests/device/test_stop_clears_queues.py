# Test: stop() leaves a motor's queues empty
#
# Purpose: driver/dshot_pio.py's stop() relies on re-initialising the state
# machines to empty the TX FIFO, so a frame queued before a stop can never go
# out at the next start(). The unit tests model that with a fake; this checks it
# on the real PIO, where "init() empties the FIFO" is a property of the
# MicroPython port, not something this repository controls.
#
# Needs the Pico only - no ESC, no motor power. Nothing is transmitted: the
# state machines are never activated, so the words queued below sit in the TX
# FIFO until stop() clears them.
#
# Pass: every line prints OK and the script ends with "PASSED".

from machine import Pin
from dshot_pio import UnidirectionalDShot, BidirectionalDShot, DSHOT_SPEEDS

SPEED = DSHOT_SPEEDS.DSHOT600
DEPTH = UnidirectionalDShot.TX_FIFO_DEPTH

failures = 0


def check(name, ok):
    global failures
    print(("OK   " if ok else "FAIL ") + name)
    if not ok:
        failures += 1


def fill(motor):
    # Inactive state machine: put() takes DEPTH words, then would block
    for _ in range(DEPTH):
        motor.sm.put(0x0E100000)


uni = UnidirectionalDShot(0, Pin(6), SPEED)
fill(uni)
check("unidirectional: words are queued before stop()", uni.sm.tx_fifo() == DEPTH)
uni.stop()
check("unidirectional: TX FIFO empty after stop()", uni.sm.tx_fifo() == 0)

fill(uni)
uni.start()
uni.stop()
check("unidirectional: TX FIFO empty after start() then stop()", uni.sm.tx_fifo() == 0)

bidir = BidirectionalDShot(4, Pin(8), SPEED, rx_state_machine_id=5)
fill(bidir)
check("bidirectional: words are queued before stop()", bidir.sm.tx_fifo() == DEPTH)
bidir.stop()
check("bidirectional: TX FIFO empty after stop()", bidir.sm.tx_fifo() == 0)
check("bidirectional: RX FIFO empty after stop()", bidir.rx_sm.rx_fifo() == 0)

# start()/stop() reload the same program each time; the PIO instruction store
# must not run out (ENOMEM) however often that happens.
try:
    for _ in range(20):
        uni.start()
        uni.stop()
        bidir.start()
        bidir.stop()
    check("20 start/stop cycles on both motor types, no ENOMEM", True)
except OSError as error:
    check("20 start/stop cycles on both motor types, no ENOMEM (" + str(error) + ")", False)

print("PASSED" if failures == 0 else "FAILED: " + str(failures))
