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

#ifndef _AQ_ADXL335_ADC_ACCELEROMETER_H_
#define _AQ_ADXL335_ADC_ACCELEROMETER_H_

#include "Accelerometer.h"


class ADXL335_ADCAccelerometer : public Accelerometer 
{
private:
  int _findZero[FINDZERO];
  int _rawADC;

public:
  ADXL335_ADCAccelerometer() : Accelerometer()
  {
    // ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
    // ADXL335 Sensitivity(from datasheet) => 330mV/g, 0.8mV/ADC step => 330/0.8 = 412
    // Tested value : 414
    // #define GRAVITY 414 //this equivalent to 1G in the raw data coming from the accelerometer 
    // #define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square
    _accelScaleFactor = 414.0 / 9.81;    
  }
  
  void initialize() 
  {
    // rollChannel = 5
    // pitchChannel = 4
    // zAxisChannel = 6
    this->_initialize(5, 4, 6);
  }
  
  void measure() 
  {
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      _rawADC = analogRead_ArduCopter_ADC(_accelChannel[axis]);
      if (_rawADC > 500) // Check if measurement good
      {
        _accelADC[axis] = _rawADC - _accelZero[axis];
      }
      _accelData[axis] = _accelADC[axis]; // no smoothing needed
    }
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }
  
  // Allows user to zero accelerometers on command
  void calibrate() 
  {
    for(byte calAxis = 0; calAxis < LASTAXIS; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++) 
      {
        _findZero[i] = analogRead_ArduCopter_ADC(_accelChannel[calAxis]);
        delay(2);
      }
      _accelZero[calAxis] = findMedianInt(_findZero, FINDZERO);
    }

    // store accel value that represents 1g
//    _accelOneG = _accelZero[ZAXIS];
    _accelOneG = 486;    // tested value with the configurator at flat level
    // replace with estimated Z axis 0g value
    _accelZero[ZAXIS] = (_accelZero[ROLL] + _accelZero[PITCH]) / 2;
  }

  void calculateAltitude() 
  {
    _currentTime = micros();
    if ((abs(getRaw(ROLL)) < 1500) && (abs(getRaw(PITCH)) < 1500)) 
    {
      _rawAltitude += (getZaxis()) * ((_currentTime - _previousTime) / 1000000.0);
    }
    _previousTime = _currentTime;
  } 
};

#endif