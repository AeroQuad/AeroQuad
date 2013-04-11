/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_VARIABLE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_VARIABLE_H_


int8_t YAW_DIRECTION = 1;

#include <Motors.h>

int motorAxisCommandRoll = 0;
int motorAxisCommandPitch = 0;
int motorAxisCommandYaw = 0;

#if defined (__AVR_ATmega328P__) || defined(__AVR_ATmegaUNO__)
  int motorMaxCommand[6] = {0,0,0,0,0,0};
  int motorMinCommand[6] = {0,0,0,0,0,0};
  int motorConfiguratorCommand[6] = {0,0,0,0,0,0};
#else
  int motorMaxCommand[8] = {0,0,0,0,0,0,0,0};
  int motorMinCommand[8] = {0,0,0,0,0,0,0,0};
  int motorConfiguratorCommand[8] = {0,0,0,0,0,0,0,0};
#endif


#endif  // #define _AQ_PROCESS_FLIGHT_CONTROL_VARIABLE_H_

