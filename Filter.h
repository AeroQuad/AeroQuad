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

#ifndef FILTER_H
#define FILTER_H

// Complementary roll/pitch angle
float flightAngle[2] = {0,0};
float filterTermRoll[4] = {0,0,0,0};
float filterTermPitch[4] = {0,0,0,0};
float timeConstant; // Read in from EEPROM

// Low pass filter parameters
#define GYRO 0
#define ACCEL 1
float smoothFactor[2]; // Read in from EEPROM
float smoothTransmitter[6]; // Read in from EEPROM
float smoothHeading; // Read in from EEPROM

void configureFilter(float timeConstant);
float filterData(float previousAngle, int gyroADC, float angle, float *filterTerm, float dt);
int smooth(int currentData, int previousData, float smoothFactor);
float arctan2(float y, float x);

#endif
