# RPI-i2c-slave
Attempt to bring up the i2c slave peripheral on the Raspberry Pi.  

##Introduction
Many years ago I was asked to develop a kernel module to allow the Raspberry Pi to emulate an RTC chip.
I've recently found the code, and am trying to update it to the current kernel intefaces.

At the moment it has only been tested on the pi 3B (Not even the 3B+!)  And full compliance does not
seem possible given the peripheral as described in the datasheet.
