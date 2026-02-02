# Demo: Manual motor control with display
#
# Interactive demo using MotorThrottleGroup facade for reliable
# arming and smooth throttle control via Core 1.
#
# State machine: DISARMED → ARMED (final state)
#
# Hardware:
#   - Pimoroni Pico Display Pack
#   - Two ESCs + motors on GPIO 4 and GPIO 5
#
# Controls:
#   - Hold B+Y: Arm (only works when disarmed)
#   - A: Motor 1 throttle up
#   - B: Motor 1 throttle down
#   - X: Motor 2 throttle up
#   - Y: Motor 2 throttle down

from dshot_pio import DSHOT_SPEEDS
from motor_throttle_group import MotorThrottleGroup
from machine import Pin
from picographics import PicoGraphics, DISPLAY_PICO_DISPLAY
from micropython import const
import utime

# -----------------------------
# Configuration
# -----------------------------
MOTOR1_PIN = Pin(4)
MOTOR2_PIN = Pin(5)
DSHOT_SPEED = DSHOT_SPEEDS.DSHOT600

THROTTLE_MIN = 70
THROTTLE_MAX = 600     # bench-safe limit
THROTTLE_STEP = 5
UPDATE_PERIOD_MS = 20  # UI update rate (Core 1 runs at 1kHz independently)


# =====================================================
# Buttons (active LOW)
# =====================================================
btn_A = Pin(12, Pin.IN, Pin.PULL_UP)  # M1 up
btn_B = Pin(13, Pin.IN, Pin.PULL_UP)  # M1 down / ARM
btn_X = Pin(14, Pin.IN, Pin.PULL_UP)  # M2 up
btn_Y = Pin(15, Pin.IN, Pin.PULL_UP)  # M2 down / ARM


# =====================================================
# Display setup
# =====================================================
display = PicoGraphics(display=DISPLAY_PICO_DISPLAY, rotate=180)
display.set_backlight(1)

black = display.create_pen(0, 0, 0)
white = display.create_pen(255, 255, 255)
green = display.create_pen(0, 255, 0)
red = display.create_pen(255, 0, 0)

WIDTH, HEIGHT = display.get_bounds()

display.set_font("bitmap8")
SCALE = const(3)

X_COL_1 = const(0)
X_COL_2 = const(120)

Y_ROW_1 = const(0)
Y_ROW_2 = const(56)
Y_ROW_3 = const(111)


def draw_disarmed():
    display.set_pen(black)
    display.clear()
    display.set_pen(white)

    display.text("DISARMED", X_COL_1, Y_ROW_1, scale=SCALE)
    display.text("Hold B+Y", X_COL_1, Y_ROW_2, scale=SCALE)
    display.text("to ARM", X_COL_1, Y_ROW_3, scale=SCALE)

    display.update()


def draw_arming():
    display.set_pen(black)
    display.clear()
    display.set_pen(green)

    display.text("ARMING", X_COL_1, Y_ROW_1, scale=SCALE)
    display.set_pen(white)
    display.text("Please", X_COL_1, Y_ROW_2, scale=SCALE)
    display.text("wait...", X_COL_1, Y_ROW_3, scale=SCALE)

    display.update()


def draw_armed(th1, th2):
    display.set_pen(black)
    display.clear()
    display.set_pen(white)

    display.text("M1:", X_COL_1, Y_ROW_1, scale=SCALE)
    display.text(str(th1), X_COL_2, Y_ROW_1, scale=SCALE)

    display.text("M2:", X_COL_1, Y_ROW_2, scale=SCALE)
    display.text(str(th2), X_COL_2, Y_ROW_2, scale=SCALE)

    display.set_pen(green)
    display.text("ARMED", X_COL_1, Y_ROW_3, scale=SCALE)

    display.update()


def draw_error(msg):
    display.set_pen(black)
    display.clear()
    display.set_pen(red)

    display.text("ERROR", X_COL_1, Y_ROW_1, scale=SCALE)
    display.set_pen(white)
    display.text(msg[:12], X_COL_1, Y_ROW_2, scale=SCALE)

    display.update()


# =====================================================
# Main
# =====================================================
def demo():
    # Create motor throttle group (handles Core 1 command loop)
    # DShotPIO instances are created internally by the facade
    motors = MotorThrottleGroup([MOTOR1_PIN, MOTOR2_PIN], DSHOT_SPEED)
    try:
        # -------------------------
        # DISARMED STATE
        # -------------------------
        draw_disarmed()

        # Wait for ARM combo (B + Y held)
        while btn_B.value() or btn_Y.value():
            utime.sleep_ms(50)

        # Start Core 1 command loop (sends throttle=0 at 1kHz)
        motors.start()
        # Arm sequence
        draw_arming()
        motors.arm()

        # Set initial throttle after arming
        throttle_m1 = THROTTLE_MIN
        throttle_m2 = THROTTLE_MIN
        motors.setAllThrottles([throttle_m1, throttle_m2])

        # -------------------------
        # ARMED STATE (final)
        # -------------------------
        while True:
            # Throttle adjustments (ignore B+Y combo when armed)
            if not btn_A.value():
                throttle_m1 += THROTTLE_STEP
            if not btn_B.value():
                throttle_m1 -= THROTTLE_STEP

            if not btn_X.value():
                throttle_m2 += THROTTLE_STEP
            if not btn_Y.value():
                throttle_m2 -= THROTTLE_STEP

            # Clamp throttle values
            throttle_m1 = max(THROTTLE_MIN, min(THROTTLE_MAX, throttle_m1))
            throttle_m2 = max(THROTTLE_MIN, min(THROTTLE_MAX, throttle_m2))

            # Update throttles (Core 1 sends commands continuously)
            motors.setThrottle(0, throttle_m1)
            motors.setThrottle(1, throttle_m2)

            draw_armed(throttle_m1, throttle_m2)

            utime.sleep_ms(UPDATE_PERIOD_MS)

    except Exception as e:
        draw_error(str(e))
        raise

    finally:
        # Always stop motors on exit
        motors.stop()


demo()
