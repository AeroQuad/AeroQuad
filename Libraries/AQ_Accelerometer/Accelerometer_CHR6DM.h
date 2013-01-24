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

#ifndef _AEROQUAD_ACCELEROMETER_CHR6DM_H_
#define _AEROQUAD_ACCELEROMETER_CHR6DM_H_

//#include <Accelerometer.h>
#include <Platform_CHR6DM.h>

#define SAMPLECOUNT 400.0

float accelScaleFactor[3] = {0.0,0.0,0.0};
float runTimeAccelBias[3] = {0, 0, 0};
float accelSmoothFactor = 1.0; // can we remove if we go with 4th order filter?
float accelOneG = 0.0;
float meterPerSecSec[3] = {0.0,0.0,0.0};

float accelSample[3] = {0,0,0};
byte accelSampleCount = 0;
  
void initializeAccel();
void measureAccel();
void measureAccelSum();
void evaluateMetersPerSec();
void computeAccelBias();


CHR6DM *accelChr6dm;

void initializeAccel() {
  vehicleState |= ACCEL_DETECTED;
}

void measureAccel() {

  meterPerSecSec[XAXIS] = accelChr6dm->data.ax * accelScaleFactor[XAXIS] + runTimeAccelBias[XAXIS];
  meterPerSecSec[YAXIS] = accelChr6dm->data.ay * accelScaleFactor[YAXIS] + runTimeAccelBias[YAXIS];
  meterPerSecSec[ZAXIS] = accelChr6dm->data.az * accelScaleFactor[ZAXIS] + runTimeAccelBias[ZAXIS];
}

void measureAccelSum() {

  accelSample[XAXIS] += accelChr6dm->data.ax;
  accelSample[YAXIS] += accelChr6dm->data.ay;
  accelSample[ZAXIS] += accelChr6dm->data.az;
  accelSampleCount++;
}

void evaluateMetersPerSec() {
  // do nothing here
}

void computeAccelBias() {
  
  for (int samples = 0; samples < SAMPLECOUNT; samples++) {
    measureAccelSum();
    delay(6);
  }

  for (byte axis = 0; axis < 3; axis++) {
    meterPerSecSec[axis] = (float(accelSample[axis])/SAMPLECOUNT) * accelScaleFactor[axis];
    accelSample[axis] = 0;
  }
  accelSampleCount = 0;

  runTimeAccelBias[XAXIS] = -meterPerSecSec[XAXIS];
  runTimeAccelBias[YAXIS] = -meterPerSecSec[YAXIS];
  runTimeAccelBias[ZAXIS] = -9.8065 - meterPerSecSec[ZAXIS];

  accelOneG = fabs(meterPerSecSec[ZAXIS] + runTimeAccelBias[ZAXIS]);
}

#endif
