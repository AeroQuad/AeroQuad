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

#ifndef _AEROQUAD_ACCELEROMETER_LSM303DLHC_H_
#define _AEROQUAD_ACCELEROMETER_LSM303DLHC_H_

#include <Accelerometer.h>
#include <SensorsStatus.h>

#define ACCEL_ADDRESS 0x19

void initializeAccel() {
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(0x20);
  byte last_status = Wire.endTransmission();
  Wire.requestFrom(ACCEL_ADDRESS, 1);
  byte out = Wire.read();
  if (out!=0) {
    vehicleState |= ACCEL_DETECTED;
    // Enable Accelerometer      
    // Normal power mode, all axes enabled
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(0x20);
    Wire.write(0b01100111); //200hz 0x67 //0x57  // 0x27 = 0b00100111
    Wire.endTransmission();   
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(0x23);
    Wire.write(0b00011000); // +/- 4G //0x08  // DLHC: enable high resolution mode
    Wire.endTransmission();
  }

  delay(10); 
}
  
void measureAccel() {
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(0x28 | (1 << 7)); // assert the MSB of the address to get the accelerometer to do slave-transmit subaddress updating.
  Wire.endTransmission();  
  Wire.requestFrom(ACCEL_ADDRESS, 6);
  byte xla = Wire.read();
  byte xha = Wire.read();
  byte yla = Wire.read();
  byte yha = Wire.read();
  byte zla = Wire.read();
  byte zha = Wire.read();

  // combine high and low bytes, then shift right to discard lowest 4 bits (which are meaningless)
  // GCC performs an arithmetic right shift for signed negative numbers, but this code will not work
  // if you port it to a compiler that does a logical right shift instead.
  long accnew0 = ((int16_t)(xha << 8 | xla)) >> 4;
  long accnew1 = ((int16_t)(yha << 8 | yla)) >> 4;
  long accnew2 = ((int16_t)(zha << 8 | zla)) >> 4;

  meterPerSecSec[XAXIS] = accnew0 * accelScaleFactor[XAXIS] + runTimeAccelBias[XAXIS];
  meterPerSecSec[YAXIS] = accnew1 * accelScaleFactor[YAXIS] + runTimeAccelBias[YAXIS];  
  meterPerSecSec[ZAXIS] = accnew2 * accelScaleFactor[ZAXIS] + runTimeAccelBias[ZAXIS];
}

void measureAccelSum() {

  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(0x28 | (1 << 7)); // assert the MSB of the address to get the accelerometer to do slave-transmit subaddress updating.
  Wire.endTransmission();  
  Wire.requestFrom(ACCEL_ADDRESS, 6);
  byte xla = Wire.read();
  byte xha = Wire.read();
  byte yla = Wire.read();
  byte yha = Wire.read();
  byte zla = Wire.read();
  byte zha = Wire.read();

  // combine high and low bytes, then shift right to discard lowest 4 bits (which are meaningless)
  // GCC performs an arithmetic right shift for signed negative numbers, but this code will not work
  // if you port it to a compiler that does a logical right shift instead.
  long accnew0 = ((int16_t)(xha << 8 | xla)) >> 4;
  long accnew1 = ((int16_t)(yha << 8 | yla)) >> 4;
  long accnew2 = ((int16_t)(zha << 8 | zla)) >> 4;

  
  
  accelSample[XAXIS] += accnew0;
  accelSample[YAXIS] += accnew1;  
  accelSample[ZAXIS] += accnew2;
  accelSampleCount++;
}

void evaluateMetersPerSec() {
	
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    meterPerSecSec[axis] = (accelSample[axis] / accelSampleCount) * accelScaleFactor[axis] + runTimeAccelBias[axis];
	accelSample[axis] = 0;
  }
  accelSampleCount = 0;		
}

void computeAccelBias() {
  
  for (int samples = 0; samples < SAMPLECOUNT; samples++) {
    measureAccelSum();
    delayMicroseconds(2500);
  }

  for (byte axis = 0; axis < 3; axis++) {
    meterPerSecSec[axis] = (float(accelSample[axis])/SAMPLECOUNT) * accelScaleFactor[axis];
    accelSample[axis] = 0;
  }
  accelSampleCount = 0;

  runTimeAccelBias[XAXIS] = -meterPerSecSec[XAXIS];
  runTimeAccelBias[YAXIS] = -meterPerSecSec[YAXIS];
  runTimeAccelBias[ZAXIS] = -9.8065 - meterPerSecSec[ZAXIS];

  accelOneG = fabs(meterPerSecSec[ZAXIS] + runTimeAccelBias[ZAXIS]);
}



#endif
