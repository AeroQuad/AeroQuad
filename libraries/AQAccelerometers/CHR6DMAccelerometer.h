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

#ifndef _AQ_CHR6DM_ACCELEROMETER_H_
#define _AQ_CHR6DM_ACCELEROMETER_H_

#include "Accelerometer.h"

class CHR6DMAccelerometer : public Accelerometer 
{
private:
  CHR6DM *_chr6dm;

public:
  CHR6DMAccelerometer(CHR6DM chr6dm) : Accelerometer() 
  {
    _chr6dm = &chr6dm;
    _accelScaleFactor = 0;
  }

  void initialize() 
  {
    calibrate();
  }

  void measure() 
  {
    _currentAccelTime = micros();
    _accelADC[XAXIS] = _chr6dm->data.ax - _accelZero[XAXIS];
    _accelADC[YAXIS] = _chr6dm->data.ay - _accelZero[YAXIS];
    _accelADC[ZAXIS] = _chr6dm->data.az - _accelOneG;

    _accelData[XAXIS] = filterSmoothWithTime(_accelADC[XAXIS], _accelData[XAXIS], _smoothFactor, ((_currentAccelTime - _previousAccelTime) / 5000.0)); //to get around 1
    _accelData[YAXIS] = filterSmoothWithTime(_accelADC[YAXIS], _accelData[YAXIS], _smoothFactor, ((_currentAccelTime - _previousAccelTime) / 5000.0));
    _accelData[ZAXIS] = filterSmoothWithTime(_accelADC[ZAXIS], _accelData[ZAXIS], _smoothFactor, ((_currentAccelTime - _previousAccelTime) / 5000.0));
    _previousAccelTime = _currentAccelTime;
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
        _chr6dm->requestAndReadPacket();
        zeroXreads[i] = _chr6dm->data.ax;
        zeroYreads[i] = _chr6dm->data.ay;
        zeroZreads[i] = _chr6dm->data.az;
    }

    _accelZero[XAXIS] = findMedianFloat(zeroXreads, FINDZERO);
    _accelZero[YAXIS] = findMedianFloat(zeroYreads, FINDZERO);
    _accelZero[ZAXIS] = findMedianFloat(zeroZreads, FINDZERO);
   
    // store accel value that represents 1g
    _accelOneG = _accelZero[ZAXIS];
    // replace with estimated Z axis 0g value
    //accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
  }

  void calculateAltitude(unsigned long currentTime) 
  {
    _currentAccelTime = currentTime;
    if ((abs(CHR_RollAngle) < 5) && (abs(CHR_PitchAngle) < 5)) 
    {
      _rawAltitude += (getZaxis()) * ((_currentAccelTime - _previousAccelTime) / 1000000.0);
    }
    _previousAccelTime = _currentAccelTime;
  } 
};

#endif