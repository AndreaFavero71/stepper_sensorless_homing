"""
Andrea Favero 22/02/2025

Micropython code for Raspberry Pi Pico (RP2040 and RP2350)
It demonstrates how to use StallGuard function from TMC2209 stepper driver.
The RP2040 (or RP2350) use PIO to generate the stepper steps




MIT License

Copyright (c) 2025 Andrea Favero

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from machine import Pin
import time, os, random


def import_led():
    """
    The rgb_led is used to visually feedback the code is started.
    The led flashes for quite some time, allowing to connect to the RP2040 via an IDE (i.e. Thonny).
    During this time is possible to interrupt the code from further imports.
    """
    print("waiting time to eventually stop the code before further imports ...")
    from rgb_led import rgb_led
    ret = False
    ret = rgb_led.heart_beat(n=10, delay=1)
    while not ret:
        time. sleep(0.1)


def import_stepper():
    """
    Stepper module is imported via a function for a delayed import.
    This gives time to eventually stop the code before further imports.
    """
    from stepper import Stepper
    return Stepper
   
    
def pin_handler(pin):
    global centering, homing_requested
    
    time.sleep_ms(debounce_time)     # wait for debounce time
    if pin.value() == 0:             # check if GPIO pin is still LOW after debounce period
        if not centering:            # case a centering process is not in place
            homing_requested = True  # set flag instead of calling function directly



def _centering(pin, stepper_frequencies):
    global last_idx, centering
    
    centering = True
    print("\n\n")
    print("#"*78)
    print("#"*12, "  Stepper centering via SENSORLESS homing function  ", "#"*12)
    print("#"*78)
    
    idx = 0 if last_idx == 1 else 1
    last_idx = idx
    ret = stepper.centering(stepper_frequencies[idx])
    
    if ret:
        print("\nStepper is centered\n\n")
    else:
        print("\nFailed to center the stepper\n\n")
    
    centering = False



def stop_code():
    if 'stepper' in locals():                # case stepper has been imported
        stepper.stop_stepper()               # stepper gets stopped (PIO steps generation)
        stepper.deactivate_pio()             # PIO's get deactivated
    if 'enable_pin' in locals():             # case enable_pin has been defined
        enable_pin.value(1)                  # pin is set high (disable TMC2209 current to stepper)
    if 'homing_pin' in locals():             # case homing_pin has been defined
        homing_pin.irq(handler=None)         # disable IRQ
    print("\nClosing the program ...")       # feedback is printed to the terminal
    

# some variables
stepper_frequencies = (400, 1200)            # stepper speeds in Hz (as example)
last_idx = 1                                 # used to alternate between the 2 stepper frequencies

debounce_time = 10                           # minimum time (ms) for push button debounce
homing_requested = False                     # flag used by the IRQ and main function to start the centering
centering = False                            # flag tracking if centering process in action

debug = True                                 # if True some informative prints will be made on the Shell




try:
    
    rgb_led = import_led()                   # led module for visual feedback
    Stepper = import_stepper()               # stepper module
    board_info = os.uname()                  # determining wich board_type is used

    # assigning max PIO frequency
    if '2040' in board_info.machine:
        if 'W' in board_info.machine:
            board_type = 'RP2040 W'
        else:
            board_type = 'RP2040'
        max_pio_frequency = 125_000_000
        
    elif '2350' in board_info.machine.lower():
        if 'W' in board_info.machine:
            board_type = 'RP2350 W'
        else:
            board_type = 'RP2350'
        max_pio_frequency = 150_000_000
    else:
        board_type = '???'
        max_pio_frequency = 125_000_000


    
    # GPIO pin to enable the motor, if the TMC2209 EN pin is wired to GND
    enable_pin = Pin(2, Pin.IN, Pin.PULL_UP)
    enable_pin.value(0)  # pin is set low (TMC2209 current to stepper)
    
    # GPIO pin used to start the sensorless homing demo
    homing_pin = Pin(9, Pin.IN, Pin.PULL_UP)

    # interrupt for the GPIO pin used to start the sensorless homing
    homing_pin.irq(trigger=Pin.IRQ_FALLING, handler=pin_handler)

    # stepper Class instantiatiation
    stepper = Stepper(max_frequency=max_pio_frequency, frequency=5_000_000, debug=debug)


    print("\nCode running in {} board".format(board_type))
    print("Sensorless homing example")
    print("\nPress the GPIO homing_pin for SENSORLESS homing demo") 


    while True:                              # infinite loop
        if homing_requested:                 # case the homing_requested flag is True
            homing_requested = False         # reset the homing_requested flag
            _centering(homing_pin, stepper_frequencies)  # call the centering function
        time.sleep(0.1)



except KeyboardInterrupt:                    # keyboard interrupts
    print("\nCtrl+C detected!")              # feedback is printed to the terminal
    
except Exception as e:                       # error 
    print(f"\nAn error occured: {e}")        # feedback is printed to the terminal

finally:                                     # closing the try loop
    stop_code()                              # stop_code function to stop PIOs
