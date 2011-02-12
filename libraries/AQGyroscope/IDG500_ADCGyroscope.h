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

#ifndef _AQ_IDG500_GYROSCOPE_H_
#define _AQ_IDG500_GYROSCOPE_H_

#include "Gyroscope.h"


class IDG500_ADCGyroscope : public Gyroscope 
{
private:
  int _rawADC;

public:
  IDG500_ADCGyroscope() : Gyroscope() 
  {
    // IDG500 Sensitivity (from datasheet) => 2.0mV/Âº/s, 0.8mV/ADC step => 0.8/3.33 = 0.4
    // Tested values : 
    //#define Gyro_Gain_X 0.4 //X axis Gyro gain
    //#define Gyro_Gain_Y 0.41 //Y axis Gyro gain
    //#define Gyro_Gain_Z 0.41 //Z axis Gyro gain
    //#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
    //#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
    //#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second
    _gyroFullScaleOutput = 500.0;   // IDG/IXZ500 full scale output = +/- 500 deg/sec
    _gyroScaleFactor = 0.4;       // IDG/IXZ500 sensitivity = 2mV/(deg/sec) 0.002
  }
  
  void initialize() 
  {
    // rollChannel = 1
    // pitchChannel = 2
    // yawChannel = 0
    this->_initialize(1, 2, 0);
    initialize_ArduCopter_ADC(); // this is needed for both gyros and accels, done once in this class
  }
  
  void measure() 
  {
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      _rawADC = analogRead_ArduCopter_ADC(_gyroChannel[axis]);
      if (_rawADC > 500) // Check if good measurement
      {
        _gyroADC[axis] =  _rawADC - _gyroZero[axis];
      }
      _gyroData[axis] = _gyroADC[axis]; // no smoothing needed
    }
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
        findZero[i] = analogRead_ArduCopter_ADC(_gyroChannel[calAxis]);
        delay(2);
      }
      _gyroZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }
  }
};

#endif