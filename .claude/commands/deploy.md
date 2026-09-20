Run `python scripts/deploy.py [test_script.py]` and report the result.

Uploads the DShot driver, MotorGroup facade, and Core1Runner to the
Pico (flat filesystem, matching their `from dshot_pio import ...` style
imports), then runs the given test script from `tests/` directly via
`mpremote run` and streams its output live. With no argument, runs
`tests/test_slow_spin.py` - the motor spins at throttle 100 for 2 minutes,
then disarms.
