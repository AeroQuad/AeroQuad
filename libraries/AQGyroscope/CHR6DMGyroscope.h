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

#ifndef _AQ_CHR6DM_GYROSCOPE_H_
#define _AQ_CHR6DM_GYROSCOPE_H_

#include "Gyroscope.h"


class CHR6DMGyroscope : public Gyroscope 
{
public:
  CHR6DMGyroscope() : Gyroscope() 
  {
    _gyroFullScaleOutput = 0;
    _gyroScaleFactor = 0;
  }

  void initialize() 
  {
    initCHR6DM();
  }

  void measure()
  {
    _currentTime = micros();
    readCHR6DM();
    _gyroADC[ROLL] = chr6dm.data.rollRate - _gyroZero[ROLL]; //gx yawRate
    _gyroADC[PITCH] = _gyroZero[PITCH] - chr6dm.data.pitchRate; //gy pitchRate
    _gyroADC[YAW] = chr6dm.data.yawRate - _gyroZero[ZAXIS]; //gz rollRate

    _gyroData[ROLL] = filterSmoothWithTime(_gyroADC[ROLL], _gyroData[ROLL], _smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
    _gyroData[PITCH] = filterSmoothWithTime(_gyroADC[PITCH], _gyroData[PITCH], _smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
    _gyroData[YAW] = filterSmoothWithTime(_gyroADC[YAW], _gyroData[YAW], _smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
    _previousTime = _currentTime;
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }

  void calibrate() 
  {
    float zeroXreads[FINDZERO];
    float zeroYreads[FINDZERO];
    float zeroZreads[FINDZERO];

    for (int i=0; i<FINDZERO; i++) 
    {
        readCHR6DM();
        zeroXreads[i] = chr6dm.data.rollRate;
        zeroYreads[i] = chr6dm.data.pitchRate;
        zeroZreads[i] = chr6dm.data.yawRate;
    }

    _gyroZero[XAXIS] = findMedianFloat(zeroXreads, FINDZERO);
    _gyroZero[YAXIS] = findMedianFloat(zeroYreads, FINDZERO);
    _gyroZero[ZAXIS] = findMedianFloat(zeroZreads, FINDZERO);
  }
};

#endif