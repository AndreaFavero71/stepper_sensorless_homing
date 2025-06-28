# Stepper motor SENSORLESS homing & centering.
This sensorless feature is based on the StallGuard function of Trinamic TMC2209's stepper driver.<br>

The TMC2209 integrates a UART interface, to which the RP2040 is connected.<br>
Via the UART, the StallGuard settings are applied, and its real time value can be accessed.<br>
When the torque on the motor increases, the StallGuard value decreases. This value can then be compared to an expected threshold.<br>
Additionally, the TMC2209's StallGuard controls the DIAG pin: the RP2040 uses an input interrupt to detect the rising edge of the DIAG level.<br>

This MicroPython code is designed for RP2040 or RP2350 microcontrollers, as it leverages the PIO features. Boards using these chips include the Raspberry Pi Pico, Pico 2, RP2040-Zero, RP2350-Zero, and many others. <br>

This is part of a larger project, involving stepper motors and RP2040 microcontrollers, yet this specific part could be useful for other makers.<br>

<br><br><br>


## Showcasing video:
Showcase objective: Stepper motor stops at the midpoint between two constraints (hard stops, aka homes).<br>

In the video:
 - The stepper motor reverses direction 2 times, if it detects high torque within a predefined range (5 complete revolutions).
 - RP2040 counts the steps in between the two constraints (home positions).
 - Steps generation and steps counting are based on PIO. For more details, see [https://github.com/AndreaFavero71/pio_stepper_control](https://github.com/AndreaFavero71/pio_stepper_control).
 - After the second direction change, the stepper stops at the midpoint between the two constraints.
 - Homing speed can be adjusted. Note: StallGuard is not accurate below 400 Hz.
 
   
https://youtu.be/Dh-xW871_UM
[![Watch the Demo](https://i.ytimg.com/vi/Dh-xW871_UM/maxresdefault.jpg)](https://youtu.be/Dh-xW871_UM)
<br><br><br>

## Collaboration with Lewis:
Lewis (DIY Machines) and I collaborated on this topic to make it easier to reproduce this functionality.<br>
Our shared goal is to help others get started and make this technique more well-known and usable in other projects.<br>

This demo uses Lewis’s V2 board (modified as V3) and 3D-printed fixture, along with the latest code release:<br><br>
https://youtu.be/fMuNHKNTSt8
[![Watch the Demo](https://i.ytimg.com/vi/fMuNHKNTSt8/maxresdefault.jpg)](https://youtu.be/fMuNHKNTSt8)

<br>

#### Showcase test setup:
 - 1 NEMA 17 stepper motor.
 - 1 RP2040-Zero board.
 - 1 Trinamic TMC 2209 driver.
 - The stepper is 200 pulses/rev, set to 1/8 microstepping = 1600 pulses/rev.
 - The stepper is controlled by the RP2040-Zero board, running MicroPython.
 - The range in between the hard-stops (homes) is varied along the video.
 - Each time the push button is pressed, a new homing & centering cycle starts.
 - Stepper speed alternates between 400 Hz and 1200 Hz.
 - UART communication between RP2040 and TMC2209.
 - The RGB LED flashes red when SG (StallGuard) is triggered, and green when the stepper is centered (it flashes three times when stalling is detected via UART, once if via the DIAG pin).

<br><br><br>


### StallGuard Depends on Speed as Well as Torque:
The StallGuard value varies with speed: The chart below shows StallGuard values experimentally collected in my setup, with the motor running unloaded.<br>
When the stepper speed varies within a limited frequency range, the SG variation is relatively (and usefully) linear.<br>
In the code, the expected minimum SG is calculated using: `min_expected_SG = 0.15 * speed      # speed is stepper frequency in Hz`<br>

Sensorless homing stops the stepper when the first of these two conditions is true:
- SG value, retrieved from UART, falling below 80% of the expected minimum (parameter `k` in `stepper.py` file).<br>
- DIAG pin raising, when SG falling below the 45% of the expected minimum (parameter `k2` in latest `stepper.py` file).<br>

This method works well from 400Hz to 1200Hz (up to 2000Hz with latest code).<br>
**Note:** StallGuard is ignored for the first 100ms of every motor's startup; This saves quite a bit of trouble :smile: <br>
 
![chart image](/images/sg_chart2.PNG)
 
<br><br>


## Connections:
Wiring diagram kindly shared by Lewis (DIY Machines), for the **V3 board** (this latest board version will be available on early July 2025).<br>
One of the key differences from the V2 board is the GPIO 11 connection to the TMC2209 DIAG pin.<br>
Compare with [board_V2 wiring diagram](./images/connections_V2.jpg) if you plan to upgrade your V2 board.<br>
![connections_image](/images/connections_V3.jpg)	
<br><br>


## Installation:
The easiest setup is to:
- Watch Lewis's tutorial https://youtu.be/TEkM0uLlkHU
- Use a board from DIY Machines and 3D-print the fixture designed by Lewis.<br>

Necessary steps are:
1. Set the TMC2209 Vref according to the driver's datasheet and your stepper motor.
2. Flash your board with Micropython v1.24.1 or later version. If using the RP2040-Zero, refer to V1.24.1 from this link https://micropython.org/download/PIMORONI_TINY2040/
3. Copy all the files from [board_V3 folder](https://github.com/AndreaFavero71/stepper_sensorless_homing/tree/main/src/board_V3) into the root folder in your RP2040-Zero; In case you have a V2 board, you can either upgrade it to V3 (connect GPIO 11 to DIAG), or download the files from [board_V2 folder](https://github.com/AndreaFavero71/stepper_sensorless_homing/tree/main/src/board_V2).
4. The example code `example.py` gets automatically started by `main.py`. To prevent the auto-start, keep GPIO 0 shorted to GND while powering the board.
5. Press the button connected to GPIO 9 to start the homing process.
Every time the button is pressed, the stepper motor speed is alternated between the two frequencies values set at `stepper_frequencies = (400, 1200)` in `example.py` file; If you prefer testing with a single speed, write the same value on both the tuple values.
6. Adjust the k parameter in stepper.py to increase/decrease StallGuard sensitivity (UART), as well as K2 parameters (acting on DIAG pin).
7. In my setup, I could vary the Vref between 1.0V and 1.4V and reliably getting the homing at 400Hz and 1200Hz, without changing the code.
<br><br>


With the latest code release:
- Sensorless homing uses both the SG values readings from the UART, as well as the DIAG pin signal level from the TMC2209 driver.<br>
- The sensorless homing detection is extended up to 2000Hz.
- The sensorless functionality is extended to all the microstepping settings (from 1/8 to 1/64).<br>
  You can configure microstepping in `stepper.py` using `(ms = self.micro_step(0)   # ms --> 0=1/8, 1=1/16, 2=1/32, 3=1/64)`.
- The stepper motor moves shortly backward at the beginning, to ensure enough 'room' while searching for the first hard-stop.

<br><br><br>


## Notes:
The built-in RGB LED flashes in different colors to indicate various activities/results.<br>
This code uses the onboard RGB LED of the RP2040-Zero or RP2350-Zero, which is not available on the Pico, Pico W, or Pico 2.<br>

Please note that the TMC2209 files are subject to license restrictions.<br>
You’re free to use and modify the code, provided you follow the license terms. I welcome any feedback or suggestions for improvement.


**Note:** Use this code at your own risk :smile:

<br><br>


## Acknowledgements:
Many thanks to:
- Lewis (DIY Machines), for the nice collaboration and his detailed [video tutorial](https://youtu.be/TEkM0uLlkHU) on this topic.
- Daniel Frenkel and his ebook on Trinamic drivers, book available in Kindle format at [Amazon](https://www.amazon.com/stores/Daniel-Frenkel/author/B0BNZG6FPD?ref=ap_rdr&isDramIntegrated=true&shoppingPortalEnabled=true).
- Chr157i4n for making the extensive TMC_2209 library.
- anonymousaga for his adaptation of the driver for Raspberry Pi Pico.


Original files I've modified for this demo: TMC_2209_StepperDriver.py and TMC_2209_uart.py<br>
Original source: https://github.com/troxel/TMC_UART<br>
Original source: https://github.com/Chr157i4n/TMC2209_Raspberry_Pi<br>
Original source: https://github.com/kjk25/TMC2209_ESP32<br>
Original source: https://github.com/anonymousaga/TMC2209_RPI_PICO<br>
