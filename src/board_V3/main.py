"""
Andrea Favero 25/06/2025

Micropython code for Raspberry Pi Pico (RP2040 and RP2350)
It demonstrates how to use TMC2209 StallGuard function for stepper sensorless homing.
PIO is used by the RP2040 (or RP2350) to generate the stepper steps frequency.
"""

# INFO
# when main.py exists, MicroPython execute it right after the booting.
# This makes easy to get your own code automatically started, via an import command.

# RISKS
# In case the code has an endless loop, or deeper issues, it might prevent the board from
# exposing the USB. If this happens, re-flashing the memory might be the only option.

# SUGGESTIONs
# The easy way, for sharp people: Comment out the import when you're working on the code.
# The safer way, for lazy people (myself): Subject the import to a GPIO pin you can alter.

from machine import Pin
auto_start_pin = Pin(0, Pin.IN, Pin.PULL_UP)

if auto_start_pin.value():                          # GPIO high, via internal pullup resistor
    print("\nCode started via import in main.py\n") # feedback is printed to the terminal
    import example                                  # file to execute by main.py, after booting

else:                                               # GPIO forced to GND to skip the autostart
    print("\nGPIO auto_start_pin forced LOW")       # feedback is printed to the terminal
    print("Import at main.py gets skipped\n")       # feedback is printed to the terminal
