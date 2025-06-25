"""
Andrea Favero 16/05/2025

Micropython code for Raspberry Pi Pico (RP2040 and RP2350)
It demonstrates how to use TMC2209 StallGuard function for stepper sensorless homing.
PIO is used by the RP2040 (or RP2350) to generate the stepper steps frequency.




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

from TMC_2209_driver import *
from rgb_led import rgb_led
from rp2 import PIO, StateMachine, asm_pio, asm_pio_encode
from machine import Pin
import time




class Stepper:

    def __init__(self, max_frequency=125000000, frequency=5000000, debug=False):
        print("\nUploading stepper_controller ...")
        
        # debug bool for printout
        self.debug = debug
        
        # frequencies for the PIO state machines
        self.max_frequency = max_frequency
        self.frequency = frequency 
        
        # (RP2040) GPIO pins
        self.STEPPER_STEPS_PIN = Pin(5, Pin.OUT)
        self.STEPPER_DIR = Pin(6, Pin.OUT)
        self.STEPPER_MS1 = Pin(3, Pin.OUT)
        self.STEPPER_MS2 = Pin(4, Pin.OUT)
        self.UART_RX_PIN = Pin(13)
        self.UART_TX_PIN = Pin(12)
        
        # stepper characteristics
        self.STEPPER_STEPS = 200        # number of (full) steps per revolution
        
        # PIO intrustions for steps generation (sm0)
        self.PIO_VAR = 2                # number of PIO commands repeated, when PIO stepper state machine is called
        self.PIO_FIX = 37               # fix number of PIO commands, when PIO stepper state machine is called
        
        # pre-encode some of the PIO instructions (>100 times faster)
        self.PULL_ENCODED = asm_pio_encode("pull()", 0)
        self.MOV_X_OSR_ENCODED = asm_pio_encode("mov(x, osr)", 0)
        self.PUSH_ENCODED = asm_pio_encode("push()", 0)
        self.MOV_ISR_X_ENCODED = asm_pio_encode("mov(isr, x)", 0)        

        # state machine for stepper steps generation
        self.sm0 = StateMachine(0, self.steps_mot_pio, freq=self.frequency, set_base=self.STEPPER_STEPS_PIN)
        self.sm0.put(65535)             # initial OSR to max value (minimum stepper speed)
        self.sm0.active(0)              # state machine is kept deactivated
        
        # state machine for stopping the stepper
        self.sm1 = StateMachine(1, self.stop_stepper_pio, freq=self.max_frequency, in_base=self.STEPPER_STEPS_PIN)
        self.sm1.irq(self._stop_stepper_handler)
        self.sm1.active(0)              # state machine is kept deactivated
        
        # state machine for stepper steps tracking
        self.sm2 = StateMachine(2, self.steps_counter_pio, freq=self.max_frequency, in_base=self.STEPPER_STEPS_PIN)
        self.set_pls_counter(0)         # sets the initial value for stepper steps counting
        self.sm2.active(1)              # starts the state machine for stepper steps counting
        
        # microsteppingmap --> setting: (descriptor, reduction, ms1, ms2)
        self.microstep_map = {0: ("1/8", 0.125, 0, 0),
                              1: ("1/16", 0.0625, 1, 1),
                              2: ("1/32", 0.03125, 1, 0),
                              3: ("1/64", 0.015625, 0, 1)}
        
        # microstep resolution configuration (internal pull-down resistors)
        ms = self.micro_step(0)   # 0=1/8, 1=1/16, 2=1/32, 3=1/64
        
        # get the steps for a full revolution
        if ms is not None:
            self.full_rev = self.get_full_rev(ms)
        
        # instantiation of tmc2209 driver
        # args: pin_step, pin_dir, pin_en, rx_pin, tx_pin, mtr_id=0, baudrate=115200, serialport=0
        self.tmc = TMC_2209( rx_pin=self.UART_RX_PIN, tx_pin=self.UART_TX_PIN, mtr_id=0, serialport=0 )
        
        # set StallGuard to max torque for the GPIO interrupt callback
        self.set_stallguard(threshold = 0)
        
        # flag for stepper spinning status (True when the motor spins)
        self.stepper_spinning = False
        
        # instance variable for the max number of stepper revolution to find home
        self.max_homing_revs = 5
        
        # instance variable for the max number of steps to be done while homing
        self.max_steps = self.max_homing_revs * self.full_rev
        
        # instance variable for the steps to be done
        self.steps_to_do = self.max_steps
        









    # sm0, the PIO program for generating steps 
    @asm_pio(set_init=PIO.OUT_LOW)
    def steps_mot_pio():      # Note: without 'self' in PIO callback function !
        """
        A frequency is generated at PIO: The frequency remains fix (it can activated and stopped).
        A Pin is associated to the Pin
        The speed can be varied by changing the off time (delay) between ON pulses
        This method allows for stepper speed variation, not really for precise positioning.
        """
        label("main")         # entry point from jmp
        pull(noblock)         # get delay data from osr, and maintained in memory
        mov(x,osr)            # OSR (Output Shift Register) content is moved to x, to prepare for future noblock pulls
        mov(y, x)             # delay assigned to y scretch register
        set(pins, 1)  [15]    # set the pin high 16 times
        set(pins, 1)  [15]    # set the pin high 16 times
        label("delay")        # entry point when x_dec>0
        set(pins, 0)          # set the pin low 1 time
        jmp(y_dec, "delay")   # y_dec-=1 if y_dec>0 else jump to "delay" 
        jmp("main")           # jump to "main"


    # sm1, the PIO program for stopping the sm0 (steps generator)
    @asm_pio()
    def stop_stepper_pio():   # Note: without 'self' in PIO callback function !
        """
        This PIO fiunction:
            - Reads the lowering edges from the output GPIO that generates the steps.
            - De-counts the 32bits OSR.
            - The starting value, to decount from, is sent by inline command.
            - Once the decount is zero it stops the steps generation (sm0).
        """
        label("wait_for_step")
        wait(1, pin, 0)       # wait for step signal to go HIGH
        wait(0, pin, 0)       # wait for step signal to go LOW (lowering edge)
        jmp(x_dec, "wait_for_step")
        irq(block, rel(0))    # trigger an IRQ


    # sm2, the PIO program for counting steps
    @asm_pio()
    def steps_counter_pio():   # Note: without 'self' in PIO callback function !
        """
        This PIO fiunction:
            -Reads the rising edges from the output GPIO that generates the steps.
            -De-counts the 32bit OSR.
            -The starting value, to decount from, is sent by inline command.
            -The reading is pushed to the rx_fifo on request, via an inline command.
        """
        label("loop")
        wait(0, pin, 0)       # wait for step signal to go LOW
        wait(1, pin, 0)       # wait for step signal to go HIGH (rising edge)
        jmp(x_dec, "loop")    # decrement x and continue looping


    def set_pls_to_do(self, val):
        """Sets  the initial value for the stpes to do, from which to decount from.
           Pre-encoded instructions are >140 times faster."""
        self.sm1.put(val)                      # val value is passed to sm1 (StateMachine 1)
        self.sm1.exec(self.PULL_ENCODED)       # execute pre-encoded 'pull()' instruction
        self.sm1.exec(self.MOV_X_OSR_ENCODED)  # execute pre-encoded 'mov(x, osr)' instruction
        self.sm1.active(1)                     # sm1 is activated


    def set_pls_counter(self, val):
        """Sets  the initial value for the stpes counter, from which to decount from.
           Pre-encoded instructions are >140 times faster."""
        self.sm2.put(val)
        self.sm2.exec(self.PULL_ENCODED)       # execute pre-encoded 'pull()' instruction
        self.sm2.exec(self.MOV_X_OSR_ENCODED)  # execute pre-encoded 'mov(x, osr)' instruction


    def get_pls_count(self):
        """Gets the steps counter value.
           Pre-encoded instructions are >140 times faster."""
        self.sm2.exec(self.MOV_ISR_X_ENCODED)  # execute pre-encoded 'mov(isr, x)' instruction
        self.sm2.exec(self.PUSH_ENCODED)       # execute pre-encoded 'push()' instruction
        if self.sm2.rx_fifo:                   # case there is data in the rx buffer
            return -self.sm2.get() & 0xffffffff  # return the current sm2 counter value (32-bit unsigned integer)
        else:                                  # case the rx buffer has no data
            return -1                          # returning -1 is a clear exception for a positive counter ...)


    def set_steps_value(self, val):
        """Set the ref position (after homing)."""
        self.position = val


    def _is_tmc_powered(self):
        """Check if the TMC driver is energized, via the UART."""
        rtn = self.read_stallguard()
        if isinstance(rtn, int):
            print("TMC_2209 driver is powered")
            return 1
        elif rtn == ():
            print("TMC_2209 driver is not powered (or UART not connected)")
            return 0


    def read_stallguard(self):
        """Gets the instant StallGuard value from the TMC driver, via the UART."""
        return self.tmc.getStallguard_Result()


    def set_stallguard(self, threshold):
        """Sets the StallGuard threshold at TMC driver, via the UART."""
        # clamp the SG threshold between 0 and 255
        threshold = max(0, min(threshold, 255))
        
        # set the StallGuard threshold and the call-back handler function
        self.tmc.setStallguard_Callback(threshold = threshold, handler = self._stallguard_callback)
        if self.debug:
            if threshold != 0:
                print("\nSetting StallGuard (irq to GPIO) to  value {}.".format(threshold))
            else:
                print("\nSetting StallGuard (irq to GPIO) to 0, meaning max possible torque.")


    def get_stepper_frequency(self, pio_val):
        """Convert the PIO value (delay) to stepper frequency."""
        if pio_val > 0:
            return int(self.frequency / (pio_val * self.PIO_VAR + self.PIO_FIX))
        else:
            return None


    def get_stepper_value(self, stepper_freq):
        """Convert the stepper frequency to PIO value (delay)."""
        if stepper_freq > 0:
            return int((self.frequency - stepper_freq * self.PIO_FIX) / (stepper_freq * self.PIO_VAR))
        else:
            return None


    def micro_step(self, ms):
        """Sets the GPIO for th decided microstepping (ms) setting."""
        settings = self.microstep_map.get(ms)  # retrieve the corresponding settings
        
        if settings:                           # case the ms setting exists
            ms_txt, k, ms1, ms2 = settings     # ms map elements are assigned
            self.STEPPER_MS1.value(ms1)        # STEPPER_MS1 output set to ms1 value
            self.STEPPER_MS2.value(ms2)        # STEPPER_MS2 output set to ms2 value
            print(f"Microstep to {ms_txt}")    # feedback is printed to the terminal
            return ms
        else:                                  # case the ms setting does not exist
            print("Wrong parameter for micro_step") # feedback is printed to the terminal
            return None


    def get_full_rev(self, ms):
        """Calculates the steps for one fulle revolution of the stepper."""
        settings = self.microstep_map.get(ms)  # retrieve the corresponding settings
        
        if settings:                           # case the ms setting exists
            ms_txt, k, ms1, ms2 = settings     # ms map elements are assigned
            full_rev = int(self.STEPPER_STEPS/k)  # steps for a stepper full revolution
            print(f"Full revolution takes {full_rev} steps")  # feedback is printed to the terminal
            return full_rev
        else:                                  # case the ms setting does not exist
            print("Wrong parameter for full revolution") # feedback is printed to the terminal
            return None


    def _stop_stepper_handler(self, sm0):
        """Call-back function by a PIO interrupt"""
        self.sm0.active(0)                     # state machine for stepper-steps generation is deactivated
        self.stepper_spinning = False          # flag tracking the stepper spinning is set False
        if self.steps_to_do < self.max_steps / 2: # case stepper stops within half way of max_steps for homing
            rgb_led.flash_color('green', bright=0.2, times=1, time_s=0.01) # flashing red led


    def _stallguard_callback(self, channel):
        """Call-back function from the StallGuard."""
        print("\nStallGuard detections\n")
        self.sm0.active(0)                     # state machine for stepper-steps generation is deactivated
        self.stepper_spinning = False          # flag tracking the stepper spinning is set False


    def stop_stepper(self):
        self.sm0.active(0)                     # state machine for stepper-steps generation is deactivated
        self.stepper_spinning = False          # flag tracking the stepper spinning is set False


    def start_stepper(self):
        self.stepper_spinning = True           # flag tracking the stepper spinning is set True
        self.sm0.active(1)                     # state machine for stepper-steps generation is activated


    def deactivate_pio(self):
        """Function to deactivate PIO."""
        self.sm0.active(0)                     # sm0 is deactivated
        self.sm1.active(0)                     # sm1 is deactivated
        self.sm2.active(0)                     # sm2 is deactivated
        PIO(0).remove_program()                # reset SM0 block
        PIO(1).remove_program()                # reset SM1 block
        print("State Machines deactivated")


    def _homing(self, h_speed, stepper_freq):
        """
        Spins the stepper at stepper_freq until StillGuard value below threshold.
        Argument h_speed is the PIO value to get stepper_freq at the GPIO pin.
        """
        self.stop_stepper()                      # stepper is stopped (in the case it wasn't)
        self.set_pls_counter(0)                  # sets the initial value for stepper steps counting
        self.set_pls_to_do(self.max_steps)       # (max) number of steps to find home
        self.sm0.put(h_speed)                    # stepper speed
        self.start_stepper()                     # stepper is started

        min_sg_expected = int(0.15 * stepper_freq)  # expected minimum StallGuard value on free spinning
        k = 0.8                                  # reduction coeficient (it should be 0.7 ~ 0.8, tune it on your need)
        sg_threshold = int(k * min_sg_expected)  # StallGuard threshold 
        max_homing_ms = int(self.max_homing_revs * 1000 * self.full_rev / stepper_freq) # timeout in ms
        
        if self.debug:                           # case self.debug is set True
            sg_list = []                         # list for debug purpose
            print(f"Homing with stepper speed of {stepper_freq}Hz and StallGuard threshold of {sg_threshold}")
        
        t_ref = time.ticks_ms()                  # time reference
        i = 0                                    # iterator index
        while time.ticks_ms() - t_ref < max_homing_ms: # while loop until timeout (unless SG detection)
            sg = self.tmc.getStallguard_Result() # StallGuard value is retrieved
            if self.debug:                       # case self.debug is set True
                sg_list.append(sg)               # StallGuard value is appended to the list
            i+=1                                 # iterator index is increased
            if i > 10:                           # case at least 10 SG readings (avoids startup effect to SG)
                if sg < sg_threshold:            # case StallGuard value lower than threshold
                    self.stop_stepper()          # stepper is stopped (if not done by the _homing func)
                    rgb_led.flash_color('red', bright=0.3, times=1, time_s=0.01) # flashing red led
                    if self.debug:               # case self.debug is True
                        print(f"Homing reached. Last SG values: {sg_list}.", \
                              f"Total of {i} iterations in {time.ticks_ms()-t_ref} ms\n")
                    return True                  # return True
                else:                            # case StallGuard value higher than threshold
                    if self.debug:               # case self.debug is set True
                        del sg_list[0]           # first element in list is deleted
                    else:                        # case self.debug is set False
                        pass                     # do nothing
        
        self.stop_stepper()                      # stepper is stopped (if not done by the _homing func)
        print("Failed homing")                   # feedback is printed to the Terminal
        return False                             # False is returned when homing fails



    def centering(self, stepper_freq):
        """
        SENSORLESS homing, on both directions, to stop the stepper in the middle.
        Argument is the stepper speed, in Hz.
        Clockwise (CW) and counter-clockwise (CCW) also depends on motor wiring.
        """
        
        stepper_freq = max(400, min(1200, stepper_freq))   # homing speed (frequency) clamped in range 400 ~ 1200 Hz
        stepper_val = self.get_stepper_value(stepper_freq) # stepper pio value is calculated
        
        self.STEPPER_DIR.value(1)                       # set stepper direction to 1 (CW)
        if self._homing(stepper_val, stepper_freq):     # call the _homing function at CW
            self.STEPPER_DIR.value(0)                   # set stepper direction to 0 (CCW)
            if self._homing(stepper_val, stepper_freq): # call the _homing function at CW
                steps_range = self.get_pls_count()      # retrieves steps done on previous activation run
                half_range = int(steps_range/2)         # half range
                self.STEPPER_DIR.value(1)               # set stepper direction to 1 (CW)
                self.steps_to_do = half_range           # steps to be done assigned to instance variable
                self.set_pls_to_do(self.steps_to_do)    # number of steps to do now is half range
                self.sm0.put(stepper_val)               # stepper speed
                self.start_stepper()                    # stepper is started
                if self.debug:                          # case self.debug is set True
                    print(f"Counted {steps_range} steps in between the 2 homes")
                    print(f"Positioning the stepper at {half_range} from the last detected home")
                return True                             # True is returned when centering succeeds
            
        else:                                           # case the homing fails
            self.steps_to_do = self.max_steps           # max number of homing steps is assigned to the steps to be done next
            self.stop_stepper()                         # stepper is stopped (in the case it wasn't)
            return False                                # False is returned when centering fails


            
        

