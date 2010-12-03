#!/bin/sh
avrdude -F -p atmega1280 -c stk500 -P /dev/tty.usbserial-A700exmC -b 57600 -U flash:w:AeroQuad.cpp.hex 
