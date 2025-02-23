# Stepper motor SENSORLESS homing & centering.
The sensorless feature is based on Trinamic TMC 2209 StallGuard function.<br>
The TMC 2209 integrates a UART, to wich the RP2040 is connected to, as the StallGuard reading can only get shared via UART.<br>
When the torque applied to the motor increases, the StallGuard value decreases, and it can be compared to the expected value.<br>
The TMC 2209 StallGuard also has an interrupt, that can be used to drive the DIA pin of the driver, but this is not the feature used for this demo.<br>


This code is for RP2040 or RP2350 microprocessors, as it leverages on the PIO features; Boards with these micros are Raspberry Pi Pico, Pico2, RP2040-Zero, RP2350-Zero, and many others.<br>
I'm working on a larger project with stepper motors and RP2040 microprocessor, and this specific part might be useful to other makers.

<br><br><br>

## Showcasing video:
Showcase objective: Stepper motor stopping in the middle of two contraints (homes).<br>

In the video:
 - the stepper motor reverses direction 2 times, if it detects high torque within a predefined range (5 complete revolutions).
 - RP2040 counts the steps in between the two contraints (homes).
 - steps generation and steps counting are based on PIO. For more details check [https://github.com/AndreaFavero71/pio_stepper_control](https://github.com/AndreaFavero71/pio_stepper_control).
 - after the second direction change, the stepper is stopped in the midlle of the 2 contraints.
 - the homing speed can be adjusted. Below 400Hz the StallGuard is not accurate.
 
   
https://youtu.be/Dh-xW871_UM
[![Watch the Demo](https://i.ytimg.com/vi/Dh-xW871_UM/maxresdefault.jpg)](https://youtu.be/Dh-xW871_UM)


#### Showcase test setup:
 - 1 NEMA 17 stepper motors.
 - 1 RP2040-Zero board.
 - 1 Trinamic TMC 2209 driver.
 - the stepper is 200 pulses/rev, set to 1/8 microstep, therefore 1600 pulses/rev.
 - the stepper is controlled by the RP2040-Zero board, running MicroPython.
 - the range in between the hard-stops (homes) is varied along the video.
 - each time the push button is pressed, a new homing & centering cycle is started.
 - the stepper speed is alternated between 400Hz and 1200Hz.
 - UART comunication between RP2040 and TMC2209.
 - the rgb led flashes red when SG is crossed, and flashes green when the stepper is centered. 
<br><br><br>

### StallGuard varies with speed, further than torque:
The StallGuard value varies with speed: In below chart, the StallGuard values experimentally collected in my setup, when the stepper was spinning without load.<br>
When the stepper speed varies within a limited frequency rance, the SG variation is rather and conveniently linear.<br>
In the code the expectd minimum SG is calculated from the speed: For the Sensorless homing, the stepper is stopped when the StallGuard falls below 80% of the expected minimum SG.<br>
In my setup, this approach works well for stepper speed ranging from 400Hz to 1200Hz. One important note is to discharge the SG values of first 100ms when the motor is started.<br>
 
  ![chart image](/images/sg_chart2.PNG)
 
<br><br>

## Connections:

| Ref |  Module |   Pin   | Connection |   Pin   |       Ref      |    Module   | Notes                                      |
|:---:|:-------:|:-------:|:----------:|:-------:|:--------------:|:-----------:|--------------------------------------------|
|  5V |  RP2040 |    5V   |    <-->    |         |       5V       |     PSU     | power supply unit for logic                |
| GND |  RP2040 |   GND   |    <-->    |         |       GND      |     PSU     | power supply unit for logic                |
| GND |  RP2040 |   GND   |    <-->    |  Pin 9  |       GND      |   TMC2209   | common ground                              |
| 3V3 |  RP2040 |   3V3   |    <-->    |  Pin 10 |       VDD      |   TMC2209   | use 3v3 for logic                          |
| GND |   PSU   |   GND   |    <-->    |  Pin 1  |       EN       |   TMC2209   | driver always active, stepper energized    |
|     |  RP2040 |  GPIO 2 |    <-->    |  Pin 1  |       EN       |   TMC2209   | (alternative) ENABLE controlled by RP2040  |
|     |  RP2040 |  GPIO 3 |    <-->    |  Pin 2  |       MS1      |   TMC2209   | micro step pin                             |
|     |  RP2040 |  GPIO 4 |    <-->    |  Pin 3  |       MS2      |   TMC2209   | micro step pin                             |
|     |  RP2040 |  GPIO 5 |    <-->    |  Pin 7  |      STEP      |   TMC2209   | stepper frequency                          |
|     |  RP2040 |  GPIO 6 |    <-->    |  Pin 8  |       DIR      |   TMC2209   | Stepper direction                          |
|     |  RP2040 | GPIO 13 |    <-->    |  Pin 4  |    PDN or RX   |   TMC2209   | UART single line                           |
|     |  RP2040 | GPIO 12 |   1 Kohm   | GPIO 13 |                |    RP2040   | UART single line                           |
|     |  RP2040 |  GPIO 9 |    <-->    |    1    |                | Push button | for demo                                   |
|     |  RP2040 |   GND   |    <-->    |    2    |                | Push button | for demo                                   |
|  VM | TMC2209 |  Pin 16 |    <-->    |         |   +12V ~ +20V  |     PSU     | Voltage for stepper                        |
| GND | TMC2209 |  Pin 15 |    <-->    |         |       GND      |     PSU     | Voltage for stepper                        |
|  A2 | TMC2209 |  Pin 14 |    <-->    |         | stepper phase2 |             | stepper                                    |
|  A1 | TMC2209 |  Pin 13 |    <-->    |         | stepper phase2 |             | stepper                                    |
|  B1 | TMC2209 |  Pin 12 |    <-->    |         | stepper phase1 |             | stepper                                    |
|  B2 | TMC2209 |  Pin 11 |    <-->    |         | stepper phase2 |             | stepper                                    |
	
<br><br>

## Installation:
1. Set the TMC2209 Vref according to the driver's datasheet and the stepper characteristics.
2. Copy all the files from `\src\` folder to a folder in your Raspberry Pi Pico.
3. Run the example.py script in MicroPython.
4. Press the push button, connected to GPIO 9, and the Sensorless homing process will start.
5. Adjust the k parameter in stepper.py , to increase or reduce StallGuard sensitivity.
6. In my setup, I could vary the Vref between 1.0V and 1.4V and reliably getting the homing at 400Hz and 1200Hz, without the need to change the code.

<br><br>

## Notes:
In the example provided, the onboard RGB led blinks when the StallGuards falls below the value assigned for Sensorless homing. The code uses the onboard RGB led of RP2040-Zero or RP2350-Zero, yet not available in Pico, nor Pico W nor Pico 2 versions. 

Please Take note of the License related to the TMC2209 files, as having some restrictions.
Feel free to use and change the code to your need, by respecting the License requirements; Please feedback in case of improvements proposals.

Of course, using this code is at your own risk :-)

<br><br>

## Acknowledgements:
Many thanks to Daniel Frenkel and his ebook on Trinamic drivers, book available in Kindle format at [Amazon](https://www.amazon.com/stores/Daniel-Frenkel/author/B0BNZG6FPD?ref=ap_rdr&isDramIntegrated=true&shoppingPortalEnabled=true).<br>

Many thanks to Chr157i4n for making the exstensive TMC_2209 library.<br>
Many thanks to anonymousaga for his adaptation of the driver for Raspberry Pi Pico.<br>
Original files I've modified for this demo: TMC_2209_StepperDriver.py and TMC_2209_uart.py<br>

Original source: https://github.com/troxel/TMC_UART<br>
Original source: https://github.com/Chr157i4n/TMC2209_Raspberry_Pi<br>
Original source: https://github.com/kjk25/TMC2209_ESP32<br>
Original source: https://github.com/anonymousaga/TMC2209_RPI_PICO<br>
