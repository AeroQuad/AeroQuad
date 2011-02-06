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

#include <AccelADXL335ADC.h>


AccelADXL335ADC::AccelADXL335ADC(AQADC aqAdc)
{
  _aqAdc = &aqAdc;
  // ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
  // ADXL335 Sensitivity(from datasheet) => 330mV/g, 0.8mV/ADC step => 330/0.8 = 412
  // Tested value : 414
  // #define GRAVITY 414 //this equivalent to 1G in the raw data coming from the accelerometer 
  // #define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square
  accelScaleFactor = 414.0 / 9.81;    
}
  
void AccelADXL335ADC::initialize(void) 
{
  // rollChannel = 5
  // pitchChannel = 4
  // zAxisChannel = 6
  this->_initialize(5, 4, 6);
}
  
void AccelADXL335ADC::measure(void) 
{
  for (byte axis = ROLL; axis < LASTAXIS; axis++) 
  {
    rawADC = _aqAdc->analogReadOilpanADC(accelChannel[axis]);
    if (rawADC > 500) // Check if measurement good
	{
      accelADC[axis] = rawADC - accelZero[axis];
	}
    accelData[axis] = accelADC[axis]; // no smoothing needed
  }
}

const int AccelADXL335ADC::getFlightData(byte axis) 
{
  return getRaw(axis);
}
  
// Allows user to zero accelerometers on command
void AccelADXL335ADC::calibrate(void) 
{
  for(byte calAxis = 0; calAxis < LASTAXIS; calAxis++) 
  {
    for (int i=0; i<FINDZERO; i++) 
	{
      findZero[i] = _aqAdc->analogReadOilpanADC(accelChannel[calAxis]);
      delay(2);
    }
    accelZero[calAxis] = findModeInt(findZero, FINDZERO);
  }

  // store accel value that represents 1g
//    accelOneG = accelZero[ZAXIS];
  accelOneG = getRaw(ZAXIS);
  // replace with estimated Z axis 0g value
  accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
   
  writeFloat(accelOneG, ACCEL1G_ADR);
  writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
  writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
  writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
}

void AccelADXL335ADC::calculateAltitude(void) 
{
  currentAccelTime = micros();
  if ((abs(getRaw(ROLL)) < 1500) && (abs(getRaw(PITCH)) < 1500)) 
  {
    rawAltitude += (getZaxis(currentAccelTime,previousAccelTime)) * ((currentAccelTime - previousAccelTime) / 1000000.0);
  }
  previousAccelTime = currentAccelTime;
} 

