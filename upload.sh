#!/bin/sh
#avrdude -F -p atmega1280 -c stk500 -P /dev/tty.usbserial-A700exmC -b 57600 -U flash:w:AeroQuad.cpp.hex
export ARDUINO_AVRDUDE_CONF=/Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/etc/avrdude.conf
avrdude -C$ARDUINO_AVRDUDE_CONF  -patmega1280 -cstk500v1 -P/dev/tty.usbserial-A700exmC -b57600 -D -Uflash:w:AeroQuad.cpp.hex:i
