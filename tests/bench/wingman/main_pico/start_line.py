# EXAMPLE APPLICATION CODE - not part of the DShot library, and a diagnostic.
#
# The start/end line to the wingman (tests/bench/wingman/board/spy.py): a recorded test holds this GPIO
# high from just before it arms until the motors are stopped, and the spy records the ESC signal wires
# exactly while it is high. One output, nothing else; with no wingman connected it drives an unconnected
# pin and changes nothing.
#
# The pin must be one nothing else uses on the main Pico (the PicoBell takes GPIO 4-5 and 16-19, the ESC
# signals GPIO 6-9, the optical sensor GPIO 14).

# START_PIN is read by the wingman's PC tools too (wingman_plan.py), which cannot import machine: it is imported inside make().

START_PIN = 15


def make():
    """The start/end line as an output, low."""
    from machine import Pin
    return Pin(START_PIN, Pin.OUT, value=0)
