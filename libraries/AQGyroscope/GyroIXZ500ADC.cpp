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

#include <GyroIXZ500ADC.h>


GyroIXZ500ADC::GyroIXZ500ADC(AQADC aqAdc)
{
  _aqAdc = &aqAdc;
  // IDG500 Sensitivity (from datasheet) => 2.0mV/Ã‚Âº/s, 0.8mV/ADC step => 0.8/3.33 = 0.4
  // Tested values : 
  //#define Gyro_Gain_X 0.4 //X axis Gyro gain
  //#define Gyro_Gain_Y 0.41 //Y axis Gyro gain
  //#define Gyro_Gain_Z 0.41 //Z axis Gyro gain
  //#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
  //#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
  //#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second
  gyroFullScaleOutput = 500.0;   // IDG/IXZ500 full scale output = +/- 500 deg/sec
  gyroScaleFactor = 0.4;       // IDG/IXZ500 sensitivity = 2mV/(deg/sec) 0.002
}
  
void GyroIXZ500ADC::initialize(void) 
{
  // rollChannel = 1
  // pitchChannel = 2
  // yawChannel = 0
  this->_initialize(1, 2, 0);
  _aqAdc->initializeOilpanADC();
}
  
void GyroIXZ500ADC::measure(void) 
{
  for (byte axis = ROLL; axis < LASTAXIS; axis++) {
    rawADC = _aqAdc->analogReadOilpanADC(gyroChannel[axis]);
    if (rawADC > 500) // Check if good measurement
	{
      gyroADC[axis] =  rawADC - gyroZero[axis];
	}
    gyroData[axis] = gyroADC[axis]; // no smoothing needed
  }
}

const int GyroIXZ500ADC::getFlightData(byte axis) 
{
  return getRaw(axis);
}

void GyroIXZ500ADC::calibrate() 
{
  int findZero[FINDZERO];
  for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
  {
    for (int i=0; i<FINDZERO; i++) 
	{
      findZero[i] = _aqAdc->analogReadOilpanADC(gyroChannel[calAxis]);
      delay(5);
    }
    gyroZero[calAxis] = findModeInt(findZero, FINDZERO);
  }
  writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
  writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
  writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
}

