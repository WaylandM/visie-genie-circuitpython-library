"""
genie_example.py
================
CircuitPython equivalent of the genieArduino_Demo Arduino sketch.

Hardware
--------
- Any CircuitPython board with a hardware UART (e.g. Raspberry Pi Pico,
  Adafruit Feather M4, ESP32-S3, etc.)
- 4D Systems display running a ViSi-Genie project that contains:
    Slider0, CoolGauge0, LedDigits0, String0, UserLed0

Wiring
------
  Board TX  -->  Display RX
  Board RX  <--  Display TX
  Board GND ---  Display GND
  Display VCC -- 3.3 V or 5 V (check your module)

The display reset line is optional; if wired connect it to any free GPIO
and uncomment the reset section below.

Usage
-----
Copy both ``genie_circuitpython.py`` and this file onto your board as
``code.py`` (renaming this file) or import it as a module.
"""

import time
import board
import busio
# import digitalio          # Uncomment if you use a hardware reset line

from genie_circuitpython import (
    Genie,
    GenieFrame,
    GENIE_REPORT_EVENT,
    GENIE_REPORT_OBJ,
    GENIE_OBJ_SLIDER,
    GENIE_OBJ_LED_DIGITS,
    GENIE_OBJ_COOL_GAUGE,
    GENIE_OBJ_USER_LED,
    GENIE_OBJ_STRINGS,
)

# ---------------------------------------------------------------------------
# Optional: hardware reset of the display
# ---------------------------------------------------------------------------
# RESET_PIN = board.D4
# reset = digitalio.DigitalInOut(RESET_PIN)
# reset.direction = digitalio.Direction.OUTPUT
# reset.value = True          # Active-low reset: drive HIGH = not in reset
# time.sleep(0.1)
# reset.value = False         # Assert reset
# time.sleep(0.1)
# reset.value = True          # Release reset
# time.sleep(4.5)             # Allow display to boot (increase if needed)

# ---------------------------------------------------------------------------
# Set up UART and Genie instance
# ---------------------------------------------------------------------------
uart = busio.UART(board.TX, board.RX, baudrate=115200, timeout=0)

genie = Genie()
genie.begin(uart)

# ---------------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------------
slider_val = 0
gauge_val = 50

# ---------------------------------------------------------------------------
# Event handler — called each time a complete frame arrives
# ---------------------------------------------------------------------------
def my_genie_event_handler():
    event = genie.dequeue_event()

    # Slider0 → mirror value to LedDigits0
    if genie.event_is(event, GENIE_REPORT_EVENT, GENIE_OBJ_SLIDER, 0):
        global slider_val
        slider_val = genie.get_event_data(event)
        genie.write_object(GENIE_OBJ_LED_DIGITS, 0, slider_val)

    # UserLed0 poll response → toggle and write back
    if genie.event_is(event, GENIE_REPORT_OBJ, GENIE_OBJ_USER_LED, 0):
        current = genie.get_event_data(event)
        genie.write_object(GENIE_OBJ_USER_LED, 0, 0 if current else 1)


genie.attach_event_handler(my_genie_event_handler)

# ---------------------------------------------------------------------------
# Initial display state
# ---------------------------------------------------------------------------
genie.write_contrast(10)                          # ~2/3 max brightness
genie.write_str(0, "CircuitPython Genie v1.0")    # Update String0

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
UPDATE_INTERVAL = 0.1   # seconds between periodic writes
last_update = time.monotonic()

while True:
    genie.do_events()   # Poll UART and dispatch any incoming frames

    now = time.monotonic()
    if now - last_update >= UPDATE_INTERVAL:
        last_update = now

        # Animate the Cool Gauge (counts 0 → 100 → 0 → ...)
        gauge_val += 1
        if gauge_val > 100:
            gauge_val = 0
        genie.write_object(GENIE_OBJ_COOL_GAUGE, 0, gauge_val)

        # Poll the UserLed so our event handler can toggle it
        genie.read_object(GENIE_OBJ_USER_LED, 0)
