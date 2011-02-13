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


#include "PWMTimedMotors.h"

#define PWM_FREQUENCY 300   // in Hz
#define PWM_PRESCALER 8
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)

PWMTimedMotors::PWMTimedMotors()
{
}

void PWMTimedMotors::initialize() 
{
  #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    DDRE = DDRE | B00111000;                                  // Set ports to output PE3-5
  #if defined(plusConfig) || defined(XConfig)
    DDRH = DDRH | B00001000;                                  // Set port to output PH3
  #endif
  #if defined(HEXACOAXIAL) || defined(HEXARADIAL)
    DDRH = DDRH | B00111000;                                  // Set ports to output PH3-5
  #endif
//#endif
//#if defined (__AVR_ATmega328P__)
  #else
    DDRB = DDRB | B00001110;                                  // Set ports to output PB1-3
    DDRD = DDRD | B00001000;                                  // Set port to output PD3
  #endif

  commandAllMotors(1000);                                   // Initialise motors to 1000us (stopped)

  #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // Init PWM Timer 3                                       // WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTTOM
    TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);                 // Prescaler set to 8, that gives us a resolution of 0.5us
    ICR3 = PWM_COUNTER_PERIOD;                               // Clock_speed / ( Prescaler * desired_PWM_Frequency) #defined above.
  #if defined(plusConfig) || defined(XConfig)
    // Init PWM Timer 4
    TCCR4A = (1<<WGM41)|(1<<COM4A1);
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    ICR4 = PWM_COUNTER_PERIOD;
  #endif
  #if defined(HEXACOAXIAL) || defined(HEXARADIAL)
    // Init PWM Timer 4
    TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    ICR4 = PWM_COUNTER_PERIOD;
  #endif
  #else
    // Init PWM Timer 1  16 bit
    TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
    ICR1 = PWM_COUNTER_PERIOD;
    // Init PWM Timer 2   8bit                               // WGMn1 WGMn2 = Mode ? Fast PWM, TOP = 0xFF ,Update of OCRnx at BOTTOM
    TCCR2A = (1<<WGM20)|(1<<WGM21)|(1<<COM2A1)|(1<<COM2B1);  // Clear OCnA/OCnB on compare match, set OCnA/OCnB at BOTTOM (non-inverting mode)
    TCCR2B = (1<<CS22)|(1<<CS21);                            // Prescaler set to 256, that gives us a resolution of 16us
    // TOP is fixed at 255                                   // Output_PWM_Frequency = 244hz = 16000000/(256*(1+255)) = Clock_Speed / (Prescaler * (1 + TOP))
  #endif
}

void PWMTimedMotors::write() 
{
  #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    OCR3B = _motorCommand[FRONT] * 2 ;
    OCR3C = _motorCommand[REAR]  * 2 ;
    OCR3A = _motorCommand[RIGHT] * 2 ;
    OCR4A = _motorCommand[LEFT]  * 2 ;
  #if defined(HEXACOAXIAL) || defined(HEXARADIAL)
    OCR4B = _motorCommand[RIGHT2] * 2 ;
    OCR4C = _motorCommand[LEFT2]  * 2 ;
  #endif
//#endif
//#if defined (__AVR_ATmega328P__)
  #else
    OCR2B = _motorCommand[FRONT] / 16 ;                       // 1000-2000 to 128-256
    OCR1A = _motorCommand[REAR]  * 2 ;
    OCR1B = _motorCommand[RIGHT] * 2 ;
    OCR2A = _motorCommand[LEFT]  / 16 ;
  #endif
}

void PWMTimedMotors::commandAllMotors(int motorCommand) 
{   // Sends commands to all motors
  #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    OCR3B = motorCommand * 2 ;
    OCR3C = motorCommand * 2 ;
    OCR3A = motorCommand * 2 ;
    OCR4A = motorCommand * 2 ;
  #if defined(HEXACOAXIAL) || defined(HEXARADIAL)
    OCR4B = motorCommand * 2 ;
    OCR4C = motorCommand * 2 ;
  #endif
//#endif
//#if defined (__AVR_ATmega328P__)
  #else
    OCR2B = motorCommand / 16 ;
    OCR1A = motorCommand * 2 ;
    OCR1B = motorCommand * 2 ;
    OCR2A = motorCommand / 16 ;
  #endif
}


