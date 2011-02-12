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

#ifndef _AQ_MULTIPILOT_GYROSCOPE_H_
#define _AQ_MULTIPILOT_GYROSCOPE_H_

#include "Gyroscope.h"

class MultipilotGyroscope : public Gyroscope 
{
public:
  MultipilotGyroscope() : Gyroscope() 
  {
    gyroFullScaleOutput = 300.0;        // ADXR610 full scale output = +/- 300 deg/sec
    gyroScaleFactor = aref / 0.006;     // ADXR610 sensitivity = 6mV/(deg/sec)
  }
  
  void initialize() 
  {
    analogReference(EXTERNAL);
    // Configure gyro auto zero pins
    pinMode (AZPIN, OUTPUT);
    digitalWrite(AZPIN, LOW);
    delay(1);

    // rollChannel = 1
    // pitchChannel = 2
    // yawChannel = 0
    this->_initialize(1,2,0);
  }
  
  void measure() 
  {
    currentTime = micros();
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
      gyroData[axis] = filterSmooth(gyroADC[axis], gyroData[axis], smoothFactor); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
    }
    previousTime = currentTime;
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }

  void calibrate() 
  {
    autoZero();
  }
  
  void autoZero() 
  {
    int findZero[FINDZERO];
    digitalWrite(AZPIN, HIGH);
    delayMicroseconds(750);
    digitalWrite(AZPIN, LOW);
    delay(8);

    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++)
      {
        findZero[i] = analogRead(gyroChannel[calAxis]);
      }
      gyroZero[calAxis] = findMedian(findZero, FINDZERO);
    }
  }
};

#endif