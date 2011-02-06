/*
  AeroQuad v2.1 - January 2011
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

#include <GyroCHR6DM.h>

GyroCHR6DM::GyroCHR6DM(CHR6DM chr6dm)
{
  _chr6dm = &chr6dm;
  gyroFullScaleOutput = 0;
  gyroScaleFactor = 0;
}

void GyroCHR6DM::initialize(void) 
{
  smoothFactor = readFloat(GYROSMOOTH_ADR);
  gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
  gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
  gyroZero[ZAXIS] = readFloat(GYRO_YAW_ZERO_ADR);
  _chr6dm->initCHR6DM();
}

void GyroCHR6DM::measure(void) 
{
  currentTime = micros();
  _chr6dm->readCHR6DM();
  gyroADC[ROLL] = _chr6dm->data.rollRate - gyroZero[ROLL]; //gx yawRate
  gyroADC[PITCH] = gyroZero[PITCH] - _chr6dm->data.pitchRate; //gy pitchRate
  gyroADC[YAW] = _chr6dm->data.yawRate - gyroZero[ZAXIS]; //gz rollRate

  gyroData[ROLL] = filterSmoothWithTime(gyroADC[ROLL], gyroData[ROLL], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
  gyroData[PITCH] = filterSmoothWithTime(gyroADC[PITCH], gyroData[PITCH], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
  gyroData[YAW] = filterSmoothWithTime(gyroADC[YAW], gyroData[YAW], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
  previousTime = currentTime;
}

const int GyroCHR6DM::getFlightData(byte axis) 
{
  return getRaw(axis);
}

void GyroCHR6DM::calibrate() 
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

  gyroZero[XAXIS] = findModeFloat(zeroXreads, FINDZERO);
  gyroZero[YAXIS] = findModeFloat(zeroYreads, FINDZERO);
  gyroZero[ZAXIS] = findModeFloat(zeroZreads, FINDZERO);

  writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
  writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
  writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
}
