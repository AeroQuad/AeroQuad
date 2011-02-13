/*
  AeroQuad v2.2 - Feburary 2011
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/***********************************************************/
/********************* PWMtimer Motors *********************/
/***********************************************************/
// Special thanks to CupOfTea for authorting this class
// http://aeroquad.com/showthread.php?1553-Timed-Motors_PWM
// Uses system timers directly instead of analogWrite
/*Some basics about the 16 bit timer:
- The timer counts clock ticks derived from the CPU clock. Using 16MHz CPU clock
  and a prescaler of 8 gives a timer clock of 2MHz, one tick every 0.5us. This
  is also called timer resolution.
- The timer is used as cyclic upwards counter, the counter period is set in the
  ICRx register. IIRC period-1 has to be set in the ICRx register.
- When the counter reaches 0, the outputs are set
- When the counter reaches OCRxy, the corresponding output is cleared.
In the code below, the period shall be 3.3ms (300hz), so the ICRx register is
 set to 6600 ticks of 0.5us/tick. It probably should be 6599, but who cares about
 this 0.5us for the period. This value is #define TOP
The high time shall be 1000us, so the OCRxy register is set to 2000. In the code
 below this can be seen in the line "commandAllMotors(1000);"  A change of
 the timer period does not change this setting, as the clock rate is still one
 tick every 0.5us. If the prescaler was changed, the OCRxy register value would
 be different.
*/

#ifndef _AQ_PWM_TIMER_MOTORS_H_
#define _AQ_PWM_TIMER_MOTORS_H_

#include "Motors.h"

class PWMTimedMotors : public Motors 
{
private:
/*  Motor   Mega Pin Port        Uno Pin Port          HEXA Mega Pin Port
    FRONT         2  PE4              3  PD3
    REAR          3  PE5              9  PB1
    RIGHT         5  PE3             10  PB2                      7  PH4
    LEFT          6  PH3             11  PB3                      8  PH5
*/
public:
  PWMTimedMotors();
  
  void initialize();
  void write();
  void commandAllMotors(int motorCommand);
};

#endif


