# set_rtc.py - set the PicoBell's PCF8523 to the current PC time via mpremote.
#
# Runs on the host (not on the Pico). Reads the local clock, then uses
# mpremote exec to push the time to the PCF8523 via tests/pcf8523.py.
# The RTC is battery-backed (CR1220 coin cell) so this is a one-time setup
# step, not something that needs to run before every capture session -
# BidirCaptureSink.init_session() raises OSError if the clock was never set.
#
# Usage:
#   python scripts/set_rtc.py
#
# Pico must be connected on COM10. mpremote interrupts any running script on
# connect. Assumes tests/pcf8523.py has already been uploaded (deploy.py does
# this as part of its normal LIBRARY_FILES upload).

import datetime
import subprocess
import sys

PYTHON = sys.executable
COM_PORT = "COM10"

# PicoBell Adalogger for Pico: RTC on I2C, SDA=GPIO4, SCL=GPIO5
RTC_SDA_PIN = 4
RTC_SCL_PIN = 5

now = datetime.datetime.now()

# Python weekday(): 0=Mon … 6=Sun
# PCF8523 weekday:  0=Sun … 6=Sat (SUNDAY=0, MONDAY=1, …)
pcf_weekday = (now.weekday() + 1) % 7

dt = (now.year, now.month, now.day, pcf_weekday, now.hour, now.minute, now.second)

pico_code = f"""\
from machine import I2C, Pin
from pcf8523 import PCF8523
i2c = I2C(0, sda=Pin({RTC_SDA_PIN}), scl=Pin({RTC_SCL_PIN}))
rtc = PCF8523(i2c)
rtc.datetime({dt})
y, mo, d, wd, h, mi, s = rtc.datetime()
print(f"{{y:04d}}-{{mo:02d}}-{{d:02d}} {{h:02d}}:{{mi:02d}}:{{s:02d}}")
"""

result = subprocess.run(
    [PYTHON, "-m", "mpremote", "connect", COM_PORT, "exec", pico_code],
    capture_output=True,
    text=True,
    timeout=15,
)

pcf_out = result.stdout.strip()
pc_str = now.strftime("%Y-%m-%d %H:%M:%S")

print(f"PC  time : {pc_str}")
print(f"PCF time : {pcf_out}" if pcf_out else "PCF time : (no response)")

if result.returncode != 0:
    print(result.stderr.strip(), file=sys.stderr)
sys.exit(result.returncode)
