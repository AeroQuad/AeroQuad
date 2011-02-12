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


#ifndef _AQ_CHR6DM_FAKE_ACCELEROMETER_H_
#define _AQ_CHR6DM_FAKE_ACCELEROMETER_H_

#include "Accelerometer.h"

class CHR6DMFakeAccelerometer : public Accelerometer 
{
public:
  float fakeAccelRoll;
  float fakeAccelPitch;
  float fakeAccelYaw;
  
  CHR6DMFakeAccelerometer() : Accelerometer() 
  {
    accelScaleFactor = 0;
  }

  void initialize() 
  {
    accelZero[ROLL] = 0;
    accelZero[PITCH] = 0;
    accelZero[ZAXIS] = 0;
    calibrate();
  }

  void measure() 
  {
    currentTime = micros();
      //read done in gyro   //TODO
    accelADC[XAXIS] = fakeAccelRoll - accelZero[XAXIS];
    accelADC[YAXIS] = fakeAccelPitch - accelZero[YAXIS];
    accelADC[ZAXIS] = fakeAccelYaw - accelOneG;

    accelData[XAXIS] = smoothWithTime(accelADC[XAXIS], accelData[XAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
    accelData[YAXIS] = smoothWithTime(accelADC[YAXIS], accelData[YAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
    accelData[ZAXIS] = smoothWithTime(accelADC[ZAXIS], accelData[ZAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
    previousTime = currentTime;
  }
  
  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }

  // Allows user to zero accelerometers on command
  void calibrate() 
  {
   float zeroXreads[FINDZERO];
   float zeroYreads[FINDZERO];
   float zeroZreads[FINDZERO];

   for (int i=0; i<FINDZERO; i++) 
   {
        chr6dm.requestAndReadPacket();
        zeroXreads[i] = 0;
        zeroYreads[i] = 0;
        zeroZreads[i] = 0;
    }

    accelZero[XAXIS] = findMedian(zeroXreads, FINDZERO);
    accelZero[YAXIS] = findMedian(zeroYreads, FINDZERO);
    accelZero[ZAXIS] = findMedian(zeroZreads, FINDZERO);

    // store accel value that represents 1g
    accelOneG = accelZero[ZAXIS];
    // replace with estimated Z axis 0g value
    //accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
  }

  void calculateAltitude(unsigned long currentTime) 
  {
    currentTime = micros();
    if ((abs(CHR_RollAngle) < 5) && (abs(CHR_PitchAngle) < 5))
    { 
      rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    }
    previousTime = currentTime;
  } 
};

#endif