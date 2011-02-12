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

#ifndef _AQ_WII_GYROSCOPE_H_
#define _AQ_WII_GYROSCOPE_H_

#include "Gyroscope.h"

class WiiGyroscope : public Gyroscope 
{
public:
  WiiGyroscope() : Gyroscope() 
  {
    // 0.5mV/Âº/s, 0.2mV/ADC step => 0.2/3.33 = around 0.069565217391304
    // @see http://invensense.com/mems/gyro/documents/PS-IDG-0650B-00-05.pdf and
    // @see http://invensense.com/mems/gyro/documents/ps-isz-0650b-00-05.pdf
    _gyroFullScaleOutput = 2000;
    _gyroScaleFactor = 0.069565217391304;
  }
  
  void initialize() 
  {
    Init_Gyro_Acc(); // defined in DataAquisition.h
  }
  
  void measure() 
  {
    _currentGyroTime = micros();
    updateControls(); // defined in DataAcquisition.h
    _gyroADC[ROLL] = NWMP_gyro[ROLL] - _gyroZero[ROLL];
    _gyroData[ROLL] = filterSmoothWithTime(_gyroADC[ROLL], _gyroData[ROLL], _smoothFactor, ((_currentGyroTime - _previousGyroTime) / 5000.0)); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
    _gyroADC[PITCH] = _gyroZero[PITCH] - NWMP_gyro[PITCH];
    _gyroData[PITCH] = filterSmoothWithTime(_gyroADC[PITCH], _gyroData[PITCH], _smoothFactor, ((_currentGyroTime - _previousGyroTime) / 5000.0)); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
    _gyroADC[YAW] =  _gyroZero[YAW] - NWMP_gyro[YAW];
    _gyroData[YAW] = filterSmoothWithTime(_gyroADC[YAW], _gyroData[YAW], _smoothFactor, ((_currentGyroTime - _previousGyroTime) / 5000.0)); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
    _previousGyroTime = _currentGyroTime;
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }

  void calibrate() 
  {
    int findZero[FINDZERO];
  
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++) 
      {
        updateControls();
        findZero[i] = NWMP_gyro[calAxis];
      }
      _gyroZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }
  }
};

#endif