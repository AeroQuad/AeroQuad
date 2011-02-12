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

#ifndef _AQ_ADXL335_ACCELEROMETER_H_
#define _AQ_ADXL335_ACCELEROMETER_H_

#include "Accelerometer.h"

class ADXL335Accelerometer : public Accelerometer 
{
public:
  ADXL335Accelerometer() : Accelerometer()
  {
    // Accelerometer Values
    // Update these variables if using a different accel
    // Output is ratiometric for ADXL 335
    // Note: Vs is not AREF voltage
    // If Vs = 3.6V, then output sensitivity is 360mV/g
    // If Vs = 2V, then it's 195 mV/g
    // Then if Vs = 3.3V, then it's 329.062 mV/g
    _accelScaleFactor = 0.000329062;
  }
  
  void initialize() 
  {
    // rollChannel = 1
    // pitchChannel = 0
    // zAxisChannel = 2
    this->_initialize(1, 0, 2);
  }
  
  void measure() 
  {
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      _accelADC[axis] = analogRead(_accelChannel[axis]) - _accelZero[axis];
      _accelData[axis] = filterSmooth(_accelADC[axis], _accelData[axis], _smoothFactor);
    }
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }
  
  // Allows user to zero accelerometers on command
  void calibrate() 
  {
    int findZero[FINDZERO];

    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++)
      {
        findZero[i] = analogRead(_accelChannel[calAxis]);
      }
      _accelZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }
    
    // store accel value that represents 1g
    _accelOneG = _accelZero[ZAXIS];
    // replace with estimated Z axis 0g value
    _accelZero[ZAXIS] = (_accelZero[ROLL] + _accelZero[PITCH]) / 2;
  }
};

#endif