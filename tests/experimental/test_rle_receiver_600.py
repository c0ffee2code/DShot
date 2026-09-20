# Test: the run-length PIO receiver against a real ESC at DSHOT600.
# The check itself, its purpose and its wiring are in rle_bench.py.

from dshot_pio import DSHOT_SPEEDS
import rle_bench

rle_bench.run(DSHOT_SPEEDS.DSHOT600)
