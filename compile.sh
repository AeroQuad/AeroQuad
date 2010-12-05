#Create AeroQuad.pde
cat AeroQuad.pde > AeroQuad.cpp

cat FlightCommand.pde >> AeroQuad.cpp
cat FlightControl.pde >> AeroQuad.cpp
cat Sensors.pde >> AeroQuad.cpp
cat SerialCom.pde >> AeroQuad.cpp


export CORES_ARDUINO="/Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino"
# -Wall
export COMMON_ARGS="-c -g -Os -w -ffunction-sections -fdata-sections -mmcu=atmega1280 -DF_CPU=16000000L -DARDUINO=21 -I$CORES_ARDUINO"
export CPP_ARGS="-fno-exceptions $COMMON_ARGS"
export CC_ARGS="$COMMON_ARGS"
export LIBRARIES="/Applications/Arduino.app/Contents/Resources/Java/libraries/"

avr-gcc $CC_ARGS $CORES_ARDUINO/pins_arduino.c -opins_arduino.c.o
avr-gcc $CC_ARGS $CORES_ARDUINO/WInterrupts.c -oWInterrupts.c.o
avr-gcc $CC_ARGS $CORES_ARDUINO/wiring.c -owiring.c.o
avr-gcc $CC_ARGS $CORES_ARDUINO/wiring_analog.c -owiring_analog.c.o
avr-gcc $CC_ARGS $CORES_ARDUINO/wiring_digital.c -owiring_digital.c.o
avr-gcc $CC_ARGS $CORES_ARDUINO/wiring_pulse.c -owiring_pulse.c.o
avr-gcc $CC_ARGS $CORES_ARDUINO/wiring_shift.c -owiring_shift.c.o
avr-gcc $CC_ARGS -I$LIBRARIES/EEPROM -I$LIBRARIES/Wire -I$LIBRARIES/Wire/utility $LIBRARIES/Wire/utility/twi.c -otwi.c.o



avr-g++ $COMMON_ARGS $CORES_ARDUINO/HardwareSerial.cpp -oHardwareSerial.cpp.o
avr-g++ $COMMON_ARGS $CORES_ARDUINO/main.cpp -omain.cpp.o
avr-g++ $COMMON_ARGS $CORES_ARDUINO/Print.cpp -oPrint.cpp.o
avr-g++ $COMMON_ARGS $CORES_ARDUINO/Tone.cpp -oTone.cpp.o
avr-g++ $COMMON_ARGS $CORES_ARDUINO/WMath.cpp -oWMath.cpp.o
avr-g++ $COMMON_ARGS $CORES_ARDUINO/WString.cpp -oWString.cpp.o
avr-g++ $COMMON_ARGS -I$LIBRARIES/EEPROM -I$LIBRARIES/Wire -I$LIBRARIES/EEPROM/utility $LIBRARIES/EEPROM/EEPROM.cpp -oEEPROM.cpp.o
avr-g++ $COMMON_ARGS -I$LIBRARIES/EEPROM -I$LIBRARIES/Wire -I$LIBRARIES/Wire/utility $LIBRARIES/Wire/Wire.cpp -oWire.cpp.o
avr-g++ $COMMON_ARGS -I$LIBRARIES/EEPROM -I$LIBRARIES/Wire AeroQuad.cpp -oAeroQuad.cpp.o

avr-ar rcs core.a pins_arduino.c.o
avr-ar rcs core.a WInterrupts.c.o
avr-ar rcs core.a wiring.c.o
avr-ar rcs core.a wiring_analog.c.o
avr-ar rcs core.a wiring_digital.c.o
avr-ar rcs core.a wiring_pulse.c.o
avr-ar rcs core.a wiring_shift.c.o
avr-ar rcs core.a HardwareSerial.cpp.o
avr-ar rcs core.a main.cpp.o
avr-ar rcs core.a Print.cpp.o
avr-ar rcs core.a Tone.cpp.o
avr-ar rcs core.a WMath.cpp.o
avr-ar rcs core.a WString.cpp.o

avr-gcc -Os -Wl,--gc-sections -mmcu=atmega1280 -o AeroQuad.cpp.elf EEPROM.cpp.o Wire.cpp.o twi.c.o AeroQuad.cpp.o core.a  -lm   # -L/var/folders/7K/7Kj7aXBDEayJZntqcpOHPk+++TI/-Tmp-/build7155846014291482053.tmp

avr-objcopy -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 AeroQuad.cpp.elf AeroQuad.cpp.eep
avr-objcopy -O ihex -R .eeprom AeroQuad.cpp.elf AeroQuad.cpp.hex

rm AeroQuad.cpp *.o
