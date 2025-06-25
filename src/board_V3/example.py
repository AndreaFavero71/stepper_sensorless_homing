"""
Andrea Favero 16/05/2025   (rev 25/06/2025)

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
import time, os


def import_led():
    """
    The rgb_led is used to visually feedback once the code is started.
    The led flashes for quite some time, allowing to connect to the RP2040 via an IDE (i.e. Thonny).
    During this time is possible to interrupt (CTRL + C) the code from further imports.
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
    """
    Short and fast function to capture the interrupt based on GPIO (push button).
    This function is also used to debounce the push button.
    """
    global centering, homing_requested
    
    time.sleep_ms(debounce_time_ms)          # wait for debounce time
    if pin.value() == 0:                     # check if GPIO pin is still LOW after debounce period
        if not centering:                    # case a motor centering process is not in place
            homing_requested = True          # set a flag instead of calling directly a function



def _centering(pin, stepper_frequencies):
    """
    Internal helper to call the centering method at the stepper.py Class.
    This function updates Global variables.
    """
    global last_idx, centering
    
    centering = True
    print("\n\n")
    print("#"*78)
    print("#"*12, "  Stepper centering via SENSORLESS homing function  ", "#"*12)
    print("#"*78)
    
    idx = 0 if last_idx == 1 else 1          # flag 0 and 1 (alternates every time this function is called)
    last_idx = idx                           # flag tracking the last idx value
    
    # call to the stepper centering method. Note: The stepper frequency alternates each time between
    # the two vaues set in stepper_frequencies, therefore testing the extreme speed cases
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
    


################################################################################################
################################################################################################
################################################################################################

# variables setting

last_idx = 1                                 # flag used to alternate between the 2 stepper frequencies
stepper_frequencies = (400, 2000)            # stepper speeds (Hz) alternatively used for the centering demo
# The stepper_frequencies values for homing could be changed based on your need
# Note1: values within the range 400 ~ 1200Hz respond well to the SENSORLESS HOMING.
# Note2: values outside the range 400 ~ 2000Hz will be clamped to these values by stepper.py

debounce_time_ms = 10                        # minimum time (ms) for push button debounce
homing_requested = False                     # flag used by the IRQ and main function to start the centering
centering = False                            # flag tracking the centering process (not in action / in action)

debug = True                                 # if True some informative prints will be made on the Shell




try:
    
    rgb_led = import_led()                   # led module for visual feedback
    Stepper = import_stepper()               # stepper module
    board_info = os.uname()                  # determining wich board_type is used

    # assigning max PIO frequency, depending on the board type
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



    # GPIO pin to enable the motor
    # Note: Alternatively, the TMC2209 EN pin must be wired to GND
    enable_pin = Pin(2, Pin.IN, Pin.PULL_UP)
    enable_pin.value(0)  # pin is set low (TMC2209 enabled, stepper always energized)

    # GPIO pin used to start each run of the sensorless homing demo
    homing_pin = Pin(9, Pin.IN, Pin.PULL_UP)

    # interrupt for the push button GPIO pin used to start the sensorless homing
    homing_pin.irq(trigger=Pin.IRQ_FALLING, handler=pin_handler)

    # stepper Class instantiatiation
    stepper = Stepper(max_frequency=max_pio_frequency, frequency=5_000_000, debug=debug)

    # case the TMC driver UART reacts properly (it is powered and properly wired)
    if stepper.tmc_test():
        
        # board tupe and other info are printed to the terminal
        print("\nCode running in {} board".format(board_type))
        print("Sensorless homing example")
        print("\nPress the push button for SENSORLESS homing demo") 


        # iterative part of the mainn function
        while True:                              # infinite loop
            if homing_requested:                 # case the homing_requested flag is True
                homing_requested = False         # reset the homing_requested flag
                _centering(homing_pin, stepper_frequencies)  # call the centering function
            time.sleep(0.1)                      # small sleeping while waiting for the 'homing request'
            
            
            # ################################################################################################### #
            # Here you'd put the application code, to be executed after the SENSORLESS HOMING                     #
            #                                                                                                     #
            # Recall to set the StallGuard to a proper value for the application.                                 #
            # If the application do not require Stall control, set the StallGuard threshol to 0 (max torque).     #
            #                                                                                                     #
            # Example of no Stall control:                                                                        #
            # stepper.set_stallguard(threshold = 0)    # set SG threshold acting on the DIAG pin, to max torque   #
            #                                                                                                     #
            # For a proper Stall control, first set the driver current and the stepper speed for your usage case. #
            # Afterward, check the StallGuard value with and without load.                                        #
            # Set the StallGuard threshold to <= 0.5 * measured SG_value in normal application.                   #
            #                                                                                                     #
            # Example of no Stall control:                                                                        #
            # sg_threshold = 0.45 * minimum_measure_SG_value_in_application                                       #
            # stepper.set_stallguard(threshold = sg_threshold)    # set SG threshold acting on the DIAG pin       #
            # ################################################################################################### #


    # case the TMC driver UART does not react (not powered or not wired properly)
    else:
        
        # info is printed to the terminal
        print("\n"*2)
        print("#"*67)
        print("#"*67)
        print("#", " "*63, "#",)
        print("#   The TMC driver UART does not react: IS THE DRIVER POWERED ?   #")
        print("#", " "*63, "#",)
        print("#"*67)
        print("#"*67)
        print("\n"*2)


except KeyboardInterrupt:                    # keyboard interrupts
    print("\nCtrl+C detected!")              # feedback is printed to the terminal
    
except Exception as e:                       # error 
    print(f"\nAn error occured: {e}")        # feedback is printed to the terminal

finally:                                     # closing the try loop
    stop_code()                              # stop_code function to stop PIOs
