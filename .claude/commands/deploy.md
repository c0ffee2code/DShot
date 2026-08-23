Run `python scripts/deploy.py` and report the result.

Uploads the DShot driver, MotorThrottleGroup facade, and Core1Runner to the
Pico (flat filesystem, matching their `from dshot_pio import ...` style
imports), then runs `tests/test_slow_spin.py` directly via `mpremote run` and
streams its output live - the motor spins at throttle 100 for 2 minutes, then
disarms.
