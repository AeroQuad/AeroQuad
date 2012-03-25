/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

#ifndef _AQ_BAROMETRIC_SENSOR_BMP085_
#define _AQ_BAROMETRIC_SENSOR_BMP085_

#include "BarometricSensor.h"
#include "Device_I2C.h"
#include <AQMath.h>

// This sets up the BMP085 from Sparkfun
// Code from http://wiring.org.co/learning/libraries/bmp085.html
// Also made bug fixes based on BMP085 library from Jordi Munoz and Jose Julio

#define BMP085_I2C_ADDRESS 0x77

#define TEMPERATURE 0
#define PRESSURE 1
#define OVER_SAMPLING_SETTING 1 // use to be 3

byte overSamplingSetting = OVER_SAMPLING_SETTING;
int ac1 = 0, ac2 = 0, ac3 = 0;
unsigned int ac4 = 0, ac5 = 0, ac6 = 0;
int b1 = 0, b2 = 0, mb = 0, mc = 0, md = 0;
long pressure = 0;
long rawPressure = 0, rawTemperature = 0;
byte pressureCount = 0;
float pressureFactor = 1/5.255;
boolean isReadPressure = false;
float rawPressureSum = 0;
byte rawPressureSumCount = 0;
  
void requestRawPressure() {
  updateRegisterI2C(BMP085_I2C_ADDRESS, 0xF4, 0x34+(overSamplingSetting<<6));
}
  
long readRawPressure() {

  sendByteI2C(BMP085_I2C_ADDRESS, 0xF6);
  Wire.requestFrom(BMP085_I2C_ADDRESS, 3); // request three bytes
  return (((unsigned long)Wire.read() << 16) | ((unsigned long)Wire.read() << 8) | ((unsigned long)Wire.read())) >> (8-overSamplingSetting);
}

void requestRawTemperature() {
  updateRegisterI2C(BMP085_I2C_ADDRESS, 0xF4, 0x2E);
}
  
unsigned int readRawTemperature() {
  sendByteI2C(BMP085_I2C_ADDRESS, 0xF6);
  return readWordWaitI2C(BMP085_I2C_ADDRESS);
}

// ***********************************************************
// Define all the virtual functions declared in the main class
// ***********************************************************
void initializeBaro() {

  // oversampling setting
  // 0 = ultra low power
  // 1 = standard
  // 2 = high
  // 3 = ultra high resolution
  overSamplingSetting = OVER_SAMPLING_SETTING;
  pressure = 0;
  baroGroundAltitude = 0;
  pressureFactor = 1/5.255;
    
  if (readWhoI2C(BMP085_I2C_ADDRESS) == 0) {
	  vehicleState |= BARO_DETECTED;
  }
  
  sendByteI2C(BMP085_I2C_ADDRESS, 0xAA);
  Wire.requestFrom(BMP085_I2C_ADDRESS, 22);
  ac1 = readShortI2C();
  ac2 = readShortI2C();
  ac3 = readShortI2C();
  ac4 = readWordI2C();
  ac5 = readWordI2C();
  ac6 = readWordI2C();
  b1 = readShortI2C();
  b2 = readShortI2C();
  mb = readShortI2C();
  mc = readShortI2C();
  md = readShortI2C();
  requestRawTemperature(); // setup up next measure() for temperature
  isReadPressure = false;
  pressureCount = 0;
  measureBaro();
  delay(5); // delay for temperature
  measureBaro();
  delay(10); // delay for pressure
  measureGroundBaro();
  // check if measured ground altitude is valid
  while (abs(baroRawAltitude - baroGroundAltitude) > 10) {
    delay(26);
    measureGroundBaro();
  }
  baroAltitude = baroGroundAltitude;
}
  
void measureBaro() {
  measureBaroSum();
  evaluateBaroAltitude();
}

void measureBaroSum() {
  // switch between pressure and tempature measurements
  // each loop, since it's slow to measure pressure
  if (isReadPressure) {
    rawPressureSum += readRawPressure();
	rawPressureSumCount++;
    if (pressureCount == 4) {
      requestRawTemperature();
      pressureCount = 0;
      isReadPressure = false;
    }
    else {
      requestRawPressure();
	}
    pressureCount++;
  }
  else { // select must equal TEMPERATURE
    rawTemperature = (long)readRawTemperature();
    requestRawPressure();
    isReadPressure = true;
  }
}

void evaluateBaroAltitude() {
  long x1, x2, x3, b3, b5, b6, p;
  unsigned long b4, b7;
  int32_t tmp;

  //calculate true temperature
  x1 = ((long)rawTemperature - ac6) * ac5 >> 15;
  x2 = ((long) mc << 11) / (x1 + md);
  b5 = x1 + x2;

  if (rawPressureSumCount == 0) { // may occure at init time that no pressure have been read yet!
    return;
  }
  rawPressure = rawPressureSum / rawPressureSumCount;
  rawPressureSum = 0.0;
  rawPressureSumCount = 0;
  
  //calculate true pressure
  b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
 
  // Real Bosch formula - b3 = ((((int32_t)ac1 * 4 + x3) << overSamplingSetting) + 2) >> 2;
  // The version below is the same, but takes less program space
  tmp = ac1;
  tmp = (tmp * 4 + x3) << overSamplingSetting;
  b3 = (tmp + 2) >> 2;
 
  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) rawPressure - b3) * (50000 >> overSamplingSetting);
  p = b7 < 0x80000000 ? (b7 << 1) / b4 : (b7 / b4) << 1;
    
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
    pressure = (p + ((x1 + x2 + 3791) >> 4));
    
  baroRawAltitude = 44330 * (1 - pow(pressure/101325.0, pressureFactor)); // returns absolute baroAltitude in meters
  baroAltitude = filterSmooth(baroRawAltitude, baroAltitude, baroSmoothFactor);
}



#endif