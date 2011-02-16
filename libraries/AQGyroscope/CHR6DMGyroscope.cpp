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

#include "CHR6DMGyroscope.h"
#include <AQMath.h>

CHR6DMGyroscope::CHR6DMGyroscope()
{
  _gyroFullScaleOutput = 0;
  _gyroScaleFactor = 0;
}

void CHR6DMGyroscope::initialize() 
{
  _chr6dm->initCHR6DM();
}

void CHR6DMGyroscope::measure()
{
  _currentGyroTime = micros();
  _chr6dm->readCHR6DM();
  _gyroADC[ROLL] = _chr6dm->data.rollRate - _gyroZero[ROLL]; //gx yawRate
  _gyroADC[PITCH] = _gyroZero[PITCH] - _chr6dm->data.pitchRate; //gy pitchRate
  _gyroADC[YAW] = _chr6dm->data.yawRate - _gyroZero[ZAXIS]; //gz rollRate

  _gyroData[ROLL] = filterSmoothWithTime(_gyroADC[ROLL], _gyroData[ROLL], _smoothFactor, ((_currentGyroTime - _previousGyroTime) / 5000.0)); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
  _gyroData[PITCH] = filterSmoothWithTime(_gyroADC[PITCH], _gyroData[PITCH], _smoothFactor, ((_currentGyroTime - _previousGyroTime) / 5000.0)); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
  _gyroData[YAW] = filterSmoothWithTime(_gyroADC[YAW], _gyroData[YAW], _smoothFactor, ((_currentGyroTime - _previousGyroTime) / 5000.0)); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
  _previousGyroTime = _currentGyroTime;
}

void CHR6DMGyroscope::calibrate() 
{
  float zeroXreads[FINDZERO];
  float zeroYreads[FINDZERO];
  float zeroZreads[FINDZERO];

  for (int i=0; i<FINDZERO; i++) 
  {
    _chr6dm->readCHR6DM();
    zeroXreads[i] = _chr6dm->data.rollRate;
    zeroYreads[i] = _chr6dm->data.pitchRate;
    zeroZreads[i] = _chr6dm->data.yawRate;
  }

  _gyroZero[XAXIS] = findMedianFloat(zeroXreads, FINDZERO);
  _gyroZero[YAXIS] = findMedianFloat(zeroYreads, FINDZERO);
  _gyroZero[ZAXIS] = findMedianFloat(zeroZreads, FINDZERO);
}

void CHR6DMGyroscope::setChr6dm(CHR6DM *chr6dm)
{
  _chr6dm = chr6dm;
}

