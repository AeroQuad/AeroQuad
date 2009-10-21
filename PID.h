/*
  AeroQuad v1.4 - October 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Open Source Arduino based quadrocopter.
 
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

#ifndef PID_H
#define PID_H

// PID Values
#define LASTAXIS 3
#define LEVELROLL 3
#define LEVELPITCH 4
#define LASTLEVELAXIS 5
#define HEADING 5 // other axes defined in Receiver.h

struct PIDdata {
  float P, I, D;
  float lastPosition;
  float integratedError;
} PID[6];
float windupGuard; // Read in from EEPROM

float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters);
void zeroIntegralError();

#endif
