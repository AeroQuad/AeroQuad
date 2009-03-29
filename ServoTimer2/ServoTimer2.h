/*
  ServoTimer2.h - Interrupt driven Servo library for Arduino using Timer2- Version 0.1
  Copyright (c) 2008 Michael Margolis.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/* 
  This library uses Timer2 to drive up to 8 servos using interrupts so no refresh activity is required from within the sketch. 
  The usage and method naming is similar to the Arduino software servo library http://www.arduino.cc/playground/ComponentLib/Servo
  except that pulse widths can be in microseconds or degrees.
  write() treats parameters of 180 or less as degrees, otherwise values are milliseconds.
 
  
  A servo is activated by creating an instance of the Servo class passing the desired pin to the attach() method.
  The servo is pulsed in the background to the value most recently written using the write() method

  Note that analogWrite of PWM on pins 3 and 11 is disabled when the first servo is attached

  The methods are:

   ServoTimer2 - Class for manipulating servo motors connected to Arduino pins.

   attach(pin )  - Attaches a servo motor to an i/o pin.
   attach(pin, min, max  ) - Attaches to a pin setting min and max values in microseconds
    default min is 544, max is 2400  
 
   write()     - Sets the servo pulse width in microseconds.

   read()      - Gets the last written servo pulse width in microseconds. 

   attached()  - Returns true if there is a servo attached. 

   detach()    - Stops an attached servos from pulsing its i/o pin. 
  

The library takes about 824 bytes of program memory and 32+(1*servos) bytes of SRAM. 
The pulse width timing is accurate to within 1%

 */

// ensure this library description is only included once
#ifndef ServoTimer2_h
#define ServoTimer2_h

#include <inttypes.h>
typedef uint8_t boolean;
typedef uint8_t byte;

#define MIN_PULSE_WIDTH       750        // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH      2250        // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH  1500        // default pulse width when servo is attached
#define FRAME_SYNC_PERIOD   8000        // total frame duration in microseconds 
#define NBR_CHANNELS 4                   // the maximum number of channels, don't change this 

typedef struct  {
      uint8_t nbr        :5 ;  // a pin number from 0 to 31
      uint8_t isActive   :1 ;  // false if this channel not enabled, pin only pulsed if true 
   } ServoPin_t   ;  

typedef struct {
  ServoPin_t Pin;
  byte counter;
  byte remainder;
}  servo_t;

class ServoTimer2
{
  public:
	// constructor:
	ServoTimer2();

	uint8_t attach(int);     // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
	                         // the attached servo is pulsed with the current pulse width value, (see the write method) 
	uint8_t attach(int, int, int); // as above but also sets min and max values for writes. 
    void detach();
    void write(int);         // store the pulse width in microseconds (between MIN_PULSE_WIDTH and MAX_PULSE_WIDTH)for this channel
    int read(); 			 // returns current pulse width in microseconds for this servo
	boolean attached();      // return true if this servo is attached 
 private:
	 uint8_t chanIndex;      // index into the channel data for this servo

};


// the following ServoArrayT2 class is not implemented in the first version of this library
class ServoArrayT2
{
  public:
	// constructor:
	ServoArrayT2();

	uint8_t attach(int);     // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
	                         // channels are assigned consecutively starting from 1
	                         // the attached servo is pulsed with the current pulse width value, (see the write method) 
    void detach(int);        // detach the servo on the given channel
	void write(int,int);     // store the pulse width in microseconds (between MIN_PULSE_WIDTH and MAX_PULSE_WIDTH)for the given channel
    int read(int); 			 // returns current pulse width in microseconds for the given channel
	boolean attached(int);   // return true if the servo on the given channel is attached 
 private:
	 uint8_t chanIndex;      // index into the channel data for this servo

};

#endif
