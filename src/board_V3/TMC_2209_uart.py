"""
This code is slightly modified from the original (TMC_2209_reg.py file is not
imported and the few registers used in this project are initialized within the Classes).

Acknowledgements:
Many thanks to Chr157i4n for his estensive TMC_2209 library.
Many thanks to anonymousaga for his adaptation for Raspberry Pi Pico.


Original file: TMC_2209_uart.py

Source: https://github.com/troxel/TMC_UART
Source: https://github.com/Chr157i4n/TMC2209_Raspberry_Pi
Source: https://github.com/kjk25/TMC2209_ESP32
Source: https://github.com/anonymousaga/TMC2209_RPI_PICO

Copyright (c) 2020 troxel
Copyright (c) 2014 The Python Packaging Authority (PyPA)
Copyright (c) 2014 Mapbox

Modified by: Andrea Favero (13/02/2025, rev 25/06/2025)

Licensed under the GNU General Public License v3.0

"""


import time
import sys
import binascii
import struct
from machine import UART

#-----------------------------------------------------------------------
# TMC_UART
#
# this class is used to communicate with the TMC via UART
# it can be used to change the settings of the TMC.
# like the current or the microsteppingmode
#-----------------------------------------------------------------------
class TMC_UART:

    mtr_id=0
    ser = None
    rFrame  = [0x55, 0, 0, 0  ]
    wFrame  = [0x55, 0, 0, 0 , 0, 0, 0, 0 ]
    communication_pause = 0
    
#-----------------------------------------------------------------------
# constructor
#-----------------------------------------------------------------------
    def __init__(self, serialport, baudrate, rxpin, txpin, mtr_id_arg):
        self.ser = UART(serialport, baudrate=baudrate, bits=8, parity=None, stop=1, tx=txpin, rx=rxpin) # baudrate 115200
        self.mtr_id=mtr_id_arg
        #self.ser.timeout = 20000/baudrate            # adjust per baud and hardware. Sequential reads without some delay fail.
        self.communication_pause = 500/baudrate     # adjust per baud and hardware. Sequential reads without some delay fail.
        
        self.IFCNT   =  0x02


        
#-----------------------------------------------------------------------
# destructor
#-----------------------------------------------------------------------
    def __del__(self):
        print("Closing UART")
        self.ser.close()

#-----------------------------------------------------------------------
# this function calculates the crc8 parity bit
#-----------------------------------------------------------------------
    def compute_crc8_atm(self, datagram, initial_value=0):
        crc = initial_value
        # Iterate bytes in data
        for byte in datagram:
            # Iterate bits in byte
            for _ in range(0, 8):
                if (crc >> 7) ^ (byte & 0x01):
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
                # Shift to next bit
                byte = byte >> 1
        return crc
    
#-----------------------------------------------------------------------
# reads the registry on the TMC with a given address.
# returns the binary value of that register
#-----------------------------------------------------------------------
    def read_reg(self, reg):
        
        rtn = ""
        
        self.rFrame[1] = self.mtr_id
        self.rFrame[2] = reg
        self.rFrame[3] = self.compute_crc8_atm(self.rFrame[:-1])

        rt = self.ser.write(bytes(self.rFrame))
        if rt != len(self.rFrame):
            print("TMC2209: Err in write {}".format(__), file=sys.stderr)
            return False
        time.sleep(self.communication_pause)  # adjust per baud and hardware. Sequential reads without some delay fail.
        if self.ser.any():
            rtn = self.ser.read()#read what it self 
        time.sleep(self.communication_pause)  # adjust per baud and hardware. Sequential reads without some delay fail.
        if rtn == None:
            print("TMC2209: Err in read")
            return ""
#         print("received "+str(len(rtn))+" bytes; "+str(len(rtn)*8)+" bits")
        return(rtn[7:11])


#-----------------------------------------------------------------------
# this function tries to read the registry of the TMC 10 times
# if a valid answer is returned, this function returns it as an integer
#-----------------------------------------------------------------------
    def read_int(self, reg):
        tries = 0
        while(True):
            rtn = self.read_reg(reg)
            tries += 1
            if(len(rtn)>=4):
                break
            else:
                if tries <= 1:
                    print("\nTMC2209: did not get the expected 4 data bytes. Instead got "+str(len(rtn))+" Bytes ...")
            if(tries>=10):
                print("TMC2209: after 10 tries not valid answer. exiting")
                print("TMC2209: is Stepper Powersupply switched on ?\n\n")
                return()   # AF
#                 raise SystemExit    # AF
        val = struct.unpack(">i",rtn)[0]
        return(val)


#-----------------------------------------------------------------------
# this function can write a value to the register of the tmc
# 1. use read_int to get the current setting of the TMC
# 2. then modify the settings as wished
# 3. write them back to the driver with this function
#-----------------------------------------------------------------------
    def write_reg(self, reg, val):
        
        self.wFrame[1] = self.mtr_id
        self.wFrame[2] =  reg | 0x80;  # set write bit
        self.wFrame[3] = 0xFF & (val>>24)
        self.wFrame[4] = 0xFF & (val>>16)
        self.wFrame[5] = 0xFF & (val>>8)
        self.wFrame[6] = 0xFF & val
        
        self.wFrame[7] = self.compute_crc8_atm(self.wFrame[:-1])

        rtn = self.ser.write(bytes(self.wFrame))
        if rtn != len(self.wFrame):
            print("TMC2209: Err in write {}".format(__), file=sys.stderr)
            return False
        time.sleep(self.communication_pause)

        return(True)


#-----------------------------------------------------------------------
# this function also writes a value to the register of the TMC
# but it also checks if the writing process was successfully by checking
# the InterfaceTransmissionCounter before and after writing
#-----------------------------------------------------------------------
    def write_reg_check(self, reg, val):
        IFCNT1 = self.read_int(self.IFCNT)
        self.write_reg(reg, val)
        IFCNT2 = self.read_int(self.IFCNT)
        IFCNT2 = self.read_int(self.IFCNT)
        
        if(IFCNT1 >= IFCNT2):
            print("TMC2209: writing not successful!")
            print("reg:{} val:{}", reg, val)
            print("IFCNT:", IFCNT1, IFCNT2)
            return False
        else:
            return True



#-----------------------------------------------------------------------
# this sets a specific bit to 1
#-----------------------------------------------------------------------
    def set_bit(self, value, bit):
        return value | (bit)


#-----------------------------------------------------------------------
# this sets a specific bit to 0
#-----------------------------------------------------------------------
    def clear_bit(self, value, bit):
        return value & ~(bit)


#-----------------------------------------------------------------------
# check the UART functionality by reading the IOIN register (0x06)
#-----------------------------------------------------------------------
    def test(self):
        rtn = self.read_reg(0x06)  # register 0x06 inquires the IOIN pin status
        if len(rtn) >= 4:
            return True
        else:
            return False