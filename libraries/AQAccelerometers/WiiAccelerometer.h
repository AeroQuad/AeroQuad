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

#ifndef _AQ_WII_ACCELEROMETER_H_
#define _AQ_WII_ACCELEROMETER_H_

#include "Accelerometer.h"

class WiiAccelerometer : public Accelerometer 
{
public:
  WiiAccelerometer() : Accelerometer()
  {
    _accelScaleFactor = 0;    
  }
  
  void measure() 
  {
    _currentAccelTime = micros();
    // Actual measurement performed in gyro class
    // We just update the appropriate variables here
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      _accelADC[axis] = _accelZero[axis] - NWMP_acc[axis];
      _accelData[axis] = filterSmoothWithTime(_accelADC[axis], _accelData[axis], _smoothFactor, ((_currentAccelTime - _previousAccelTime) / 5000.0));
    }
    _previousAccelTime = _currentAccelTime;
  }
  
  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }
 
  // Allows user to zero accelerometers on command
  void calibrate() 
  {
    int findZero[FINDZERO];

    for(byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++) 
      {
        updateControls();
        findZero[i] = NWMP_acc[calAxis];
      }
      _accelZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }
    
    // store accel value that represents 1g
    _accelOneG = getRaw(ZAXIS);
    // replace with estimated Z axis 0g value
    _accelZero[ZAXIS] = (_accelZero[ROLL] + _accelZero[PITCH]) / 2;
  }
};

#endif