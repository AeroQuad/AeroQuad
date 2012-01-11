/*
  AeroQuad v3.0 - Januar 2012
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

#ifndef _AEROQUAD_ACCELEROMETER_BMA180_H_
#define _AEROQUAD_ACCELEROMETER_BMA180_H_

#include <Accelerometer.h>
#include <Device_I2C.h>
#include <SensorsStatus.h>

#ifdef BMA180_ADDRESS_ALTERNATE
  #define BMA180_ADDRESS 0x41   // Alternate address 41h
#else
  #define BMA180_ADDRESS 0x40
#endif
#define BMA180_IDENTITY 0x03
#define BMA180_RESET_REGISTER 0x10
#define BMA180_TRIGER_RESET_VALUE 0xB6
#define BMA180_ENABLE_WRITE_CONTROL_REGISTER 0x0D
#define BMA180_CONTROL_REGISTER 0x10
#define BMA180_BW_TCS 0x20
#define BMA180_LOW_PASS_FILTER_REGISTER 0x20
#define BMA180_10HZ_LOW_PASS_FILTER_VALUE 0x0F
#define BMA180_1200HZ_LOW_PASS_FILTER_VALUE 0X7F
#define BMA180_OFFSET_REGISTER 0x35
#define BMA180_READ_ROLL_ADDRESS 0x02
#define BMA180_READ_PITCH_ADDRESS 0x04
#define BMA180_READ_YAW_ADDRESS 0x06
#define BMA180_BUFFER_SIZE 6

void initializeAccel() {
  
  if (readWhoI2C(BMA180_ADDRESS) == BMA180_IDENTITY) {// page 52 of datasheet
    vehicleState |= ACCEL_DETECTED;
  }
	
  updateRegisterI2C(BMA180_ADDRESS, BMA180_RESET_REGISTER, BMA180_TRIGER_RESET_VALUE); 					//reset device
  delay(10);  																							//sleep 10 ms after reset (page 25)

  // In datasheet, summary register map is page 21
  // Low pass filter settings is page 27
  // Range settings is page 28
  updateRegisterI2C(BMA180_ADDRESS, BMA180_ENABLE_WRITE_CONTROL_REGISTER, BMA180_CONTROL_REGISTER); 		//enable writing to control registers
  sendByteI2C(BMA180_ADDRESS, BMA180_BW_TCS); 															// register bw_tcs (bits 4-7)
  byte data = readByteI2C(BMA180_ADDRESS); 																// get current register value
  updateRegisterI2C(BMA180_ADDRESS, BMA180_LOW_PASS_FILTER_REGISTER, data & BMA180_1200HZ_LOW_PASS_FILTER_VALUE); 	// set low pass filter to 1.2kHz (value = 0000xxxx)

  // From page 27 of BMA180 Datasheet
  //  1.0g = 0.13 mg/LSB
  //  1.5g = 0.19 mg/LSB
  //  2.0g = 0.25 mg/LSB
  //  3.0g = 0.38 mg/LSB
  //  4.0g = 0.50 mg/LSB
  //  8.0g = 0.99 mg/LSB
  // 16.0g = 1.98 mg/LSB
  sendByteI2C(BMA180_ADDRESS, BMA180_OFFSET_REGISTER); 													// register offset_lsb1 (bits 1-3)
  data = readByteI2C(BMA180_ADDRESS);
  data &= 0xF1;
  //data |= 0x04; // Set range select bits for +/-2g
  data |= 0x08; // set range select bits for +/-4g
  updateRegisterI2C(BMA180_ADDRESS, BMA180_OFFSET_REGISTER, data);	
}
  
void measureAccel() {

  sendByteI2C(BMA180_ADDRESS, BMA180_READ_ROLL_ADDRESS);
  Wire.requestFrom(BMA180_ADDRESS, BMA180_BUFFER_SIZE);
  
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    meterPerSecSec[axis] = (readReverseShortI2C() >> 2) * accelScaleFactor[axis] + runTimeAccelBias[axis];
  }  
}

void measureAccelSum() {

  sendByteI2C(BMA180_ADDRESS, BMA180_READ_ROLL_ADDRESS);
  Wire.requestFrom(BMA180_ADDRESS, BMA180_BUFFER_SIZE);
  
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    accelSample[axis] += (readReverseShortI2C() >> 2);
  }
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

  accelOneG = abs(meterPerSecSec[ZAXIS] + runTimeAccelBias[ZAXIS]);
}

#endif
