# SPDX-License-Identifier: GPL-3.0-or-later
# PCF8523 Real-Time Clock driver for MicroPython (Raspberry Pi Pico 2 / RP2350)
# Datasheet: NXP PCF8523 Product data sheet, Rev. 7 — 28 April 2015
# Cross-checked against:
#   Adafruit CircuitPython PCF8523 — adafruit_pcf8523/pcf8523.py
#   github.com/adafruit/Adafruit_CircuitPython_PCF8523/blob/main/adafruit_pcf8523/pcf8523.py

# Register addresses — Datasheet §8.1 Table 6, p.7–8
REG_CONTROL_1 = 0x00
REG_CONTROL_2 = 0x01
REG_CONTROL_3 = 0x02
REG_SECONDS   = 0x03
REG_MINUTES   = 0x04
REG_HOURS     = 0x05
REG_DAYS      = 0x06
REG_WEEKDAYS  = 0x07
REG_MONTHS    = 0x08
REG_YEARS     = 0x09

# Weekday encoding — Datasheet §8.6.5 Table 18, p.22
# Default NXP assignment (note: "Definition may be reassigned by the user")
# Cross-checked: adafruit_pcf8523 BCDDateTimeRegister weekday_start=0 (0=Sunday)
SUNDAY    = 0
MONDAY    = 1
TUESDAY   = 2
WEDNESDAY = 3
THURSDAY  = 4
FRIDAY    = 5
SATURDAY  = 6


def bcd_to_dec(bcd):
    return (bcd >> 4) * 10 + (bcd & 0x0F)


def dec_to_bcd(val):
    return ((val // 10) << 4) | (val % 10)


class PCF8523:
    """PCF8523 Real-Time Clock."""

    I2C_ADDR = 0x68  # Datasheet §2: write D0h, read D1h → 7-bit address 0x68

    def __init__(self, i2c, addr=0x68):
        self._i2c = i2c
        self._addr = addr
        self._configure()

    def _configure(self):
        # Control_1 (0x00): ensure 24h mode (bit 3 = 0) and clock running (STOP bit 5 = 0).
        # Read-modify-write to leave CAP_SEL (bit 7) and interrupt-enable bits untouched.
        # CAP_SEL selects crystal load capacitance: 0 = 7pF, 1 = 12.5pF.
        # Adafruit CircuitPython driver never sets CAP_SEL — leaves POR default 0 (7pF) —
        # confirming the Adafruit PCF8523 breakout (#3295) uses a 7pF crystal.
        # Cross-checked: adafruit_pcf8523/pcf8523.py — high_capacitance = RWBit(0x00, 7)
        # Datasheet §8.2.1 Table 7, p.9
        ctrl1 = self._read_reg(REG_CONTROL_1)
        self._write_reg(REG_CONTROL_1, ctrl1 & 0xD7)  # clear STOP[5]=0x20 and 12_24[3]=0x08

        # Control_3 (0x02): enable standard battery switch-over + battery low detection.
        # PM[2:0] bits [7:5] = 000 → standard switch-over, low-battery detection on.
        # Power-on reset default: PM[2:0] = 111 (all disabled) — Datasheet §8.3 Table 10, p.12.
        # Set here (not deferred to datetime write) so battery protection is active immediately,
        # even when the chip already holds a valid time from a previous power cycle.
        # Standard mode (000) is preferred over direct-switching: §8.5.2.2 p.17 warns that
        # direct switching is not recommended when VDD ≈ VBAT (3V3 Pico + ~3V coin cell).
        # Cross-checked: STANDARD_BATTERY_SWITCHOVER_AND_DETECTION = 0b000 in adafruit_pcf8523.py
        # Datasheet §8.5 Table 11, p.15
        ctrl3 = self._read_reg(REG_CONTROL_3)
        self._write_reg(REG_CONTROL_3, ctrl3 & 0x1F)  # clear PM[2:0] = bits [7:5] → 000

    @property
    def lost_power(self):
        """True if the oscillator-stop (OS) flag is set — datetime value is invalid.

        Set on every cold power-up; cleared implicitly when datetime() writes a time.
        datetime() raises OSError on read when this is True — use it to distinguish
        "clock not set" from other OSErrors (e.g. I2C bus fault).
        Datasheet §8.6.1 Table 12, p.20; §8.6.1.1 Oscillator STOP flag, p.20.
        """
        return bool(self._read_reg(REG_SECONDS) & 0x80)

    def datetime(self, dt=None):
        """Read or write RTC time.

        No argument → returns (year, month, day, weekday, hour, minute, second).
        7-tuple argument → sets the time and clears the OS flag.
        Raises OSError on read if the oscillator-stop flag is set (lost_power is True).
        """
        if dt is None:
            return self._read_datetime()
        self._write_datetime(dt)
        return None

    def _read_datetime(self):
        # Single 7-byte burst from 0x03 through 0x09.
        # The chip freezes all time counters for the I2C access duration to prevent
        # torn reads across a carry boundary (e.g. 23:59:59 → 00:00:00). All 7 bytes
        # must arrive in one transaction — a split read can return hours from one second
        # and minutes from the next. Datasheet §8.6.8 Fig 14, p.23.
        buf = bytearray(7)
        self._i2c.readfrom_mem_into(self._addr, REG_SECONDS, buf)
        if buf[0] & 0x80:  # OS flag in Seconds bit 7 — §8.6.1.1 p.20
            raise OSError("PCF8523: oscillator-stop flag set — call datetime(dt) to initialise")
        # buf: [0]=0x03 Seconds  [1]=0x04 Minutes  [2]=0x05 Hours
        #      [3]=0x06 Days     [4]=0x07 Weekdays  [5]=0x08 Months  [6]=0x09 Years
        return (
            bcd_to_dec(buf[6]) + 2000,  # Years   §8.6.7 Table 21, p.23; 0–99 offset from 2000
            bcd_to_dec(buf[5] & 0x1F),  # Months  §8.6.6 Table 19, p.22; bits [4:0]
            bcd_to_dec(buf[3] & 0x3F),  # Days    §8.6.4 Table 16, p.21; bits [5:0]
            buf[4] & 0x07,              # Weekdays §8.6.5 Table 17, p.22; bits [2:0]; raw, not BCD
            bcd_to_dec(buf[2] & 0x3F),  # Hours   §8.6.3 Table 15, p.21; bits [5:0] in 24h mode
            bcd_to_dec(buf[1] & 0x7F),  # Minutes §8.6.2 Table 14, p.21; bits [6:0]
            bcd_to_dec(buf[0] & 0x7F),  # Seconds §8.6.1 Table 12, p.20; bits [6:0] (bit 7 = OS flag)
        )

    def _write_datetime(self, dt):
        year, month, day, weekday, hour, minute, second = dt
        buf = bytearray([
            dec_to_bcd(second),       # 0x03: bit 7 written as 0 → clears OS flag; §8.6.1.1 p.20
            dec_to_bcd(minute),       # 0x04
            dec_to_bcd(hour),         # 0x05: 24h mode guaranteed by _configure
            dec_to_bcd(day),          # 0x06
            weekday & 0x07,           # 0x07: raw 0–6, not BCD — §8.6.5 Table 17, p.22
            dec_to_bcd(month),        # 0x08
            dec_to_bcd(year - 2000),  # 0x09: 2-digit offset year (2000–2099)
        ])
        # Single burst write 0x03–0x09 — Datasheet §8.6.8 p.23
        self._i2c.writeto_mem(self._addr, REG_SECONDS, buf)

    def _read_reg(self, reg):
        return self._i2c.readfrom_mem(self._addr, reg, 1)[0]

    def _write_reg(self, reg, val):
        self._i2c.writeto_mem(self._addr, reg, bytes([val]))
