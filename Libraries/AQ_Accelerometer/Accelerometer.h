/*
  AeroQuad v3.0 - May 2011
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

#ifndef _AEROQUAD_ACCELEROMETER_H_
#define _AEROQUAD_ACCELEROMETER_H_

#include "Arduino.h"
#include "GlobalDefined.h"

#define SAMPLECOUNT 400.0

float accelScaleFactor[3] = {0.0,0.0,0.0};
float runTimeAccelBias[3] = {0, 0, 0};
float accelSmoothFactor = 1.0; // can we remove if we go with 4th order filter?
float accelOneG = 0.0;
float meterPerSec[3] = {0.0,0.0,0.0};

long accelSample[3] = {0,0,0};
byte accelSampleCount = 0;
  
void initializeAccel();
void measureAccel();
void measureAccelSum();
void evaluateMetersPerSec();
void computeAccelBias();
  
#endif