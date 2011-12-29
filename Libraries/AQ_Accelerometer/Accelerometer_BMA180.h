/*
  AeroQuad v3.0 - May 2011
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

#define ACCEL_ADDRESS 0x40
#define ACCEL_IDENTITY 0x03
#define ACCEL_RESET_REGISTER 0x10
#define ACCEL_TRIGER_RESET_VALUE 0xB6
#define ACCEL_ENABLE_WRITE_CONTROL_REGISTER 0x0D
#define ACCEL_CONTROL_REGISTER 0x10
#define ACCEL_BW_TCS 0x20
#define ACCEL_LOW_PASS_FILTER_REGISTER 0x20
#define ACCEL_10HZ_LOW_PASS_FILTER_VALUE 0x0F
#define ACCEL_1200HZ_LOW_PASS_FILTER_VALUE 0X7F
#define ACCEL_OFFSET_REGISTER 0x35
#define ACCEL_READ_ROLL_ADDRESS 0x02
#define ACCEL_READ_PITCH_ADDRESS 0x04
#define ACCEL_READ_YAW_ADDRESS 0x06

void initializeAccel() {
  
  //accelScale = G_2_MPS2(1.0/4096.0);  //  g per LSB @ +/- 2g range - checking with John if we can remove this
  
  if (readWhoI2C(ACCEL_ADDRESS) != ACCEL_IDENTITY) {// page 52 of datasheet
    vehicleState |= ACCEL_DETECTED;
  }
	
  updateRegisterI2C(ACCEL_ADDRESS, ACCEL_RESET_REGISTER, ACCEL_TRIGER_RESET_VALUE); 					//reset device
  delay(10);  																							//sleep 10 ms after reset (page 25)

  // In datasheet, summary register map is page 21
  // Low pass filter settings is page 27
  // Range settings is page 28
  updateRegisterI2C(ACCEL_ADDRESS, ACCEL_ENABLE_WRITE_CONTROL_REGISTER, ACCEL_CONTROL_REGISTER); 		//enable writing to control registers
  sendByteI2C(ACCEL_ADDRESS, ACCEL_BW_TCS); 															// register bw_tcs (bits 4-7)
  byte data = readByteI2C(ACCEL_ADDRESS); 																// get current register value
  updateRegisterI2C(ACCEL_ADDRESS, ACCEL_LOW_PASS_FILTER_REGISTER, data & ACCEL_1200HZ_LOW_PASS_FILTER_VALUE); 	// set low pass filter to 10Hz (value = 0000xxxx)

  // From page 27 of BMA180 Datasheet
  //  1.0g = 0.13 mg/LSB
  //  1.5g = 0.19 mg/LSB
  //  2.0g = 0.25 mg/LSB
  //  3.0g = 0.38 mg/LSB
  //  4.0g = 0.50 mg/LSB
  //  8.0g = 0.99 mg/LSB
  // 16.0g = 1.98 mg/LSB
  sendByteI2C(ACCEL_ADDRESS, ACCEL_OFFSET_REGISTER); 													// register offset_lsb1 (bits 1-3)
  data = readByteI2C(ACCEL_ADDRESS);
  data &= 0xF1;
  //data |= 0x04; // Set range select bits for +/-2g
  data |= 0x08;
  updateRegisterI2C(ACCEL_ADDRESS, ACCEL_OFFSET_REGISTER, data);	
}
  
void measureAccel() {

  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(ACCEL_READ_ROLL_ADDRESS);
  Wire.endTransmission();
  Wire.requestFrom(ACCEL_ADDRESS, 6);
  
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    meterPerSec[axis] = (readReverseShortI2C() >> 2) * accelScaleFactor[axis] + runTimeAccelBias[axis];
  }  
}

void measureAccelSum() {

  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(ACCEL_READ_ROLL_ADDRESS);
  Wire.endTransmission();
  Wire.requestFrom(ACCEL_ADDRESS, 6);
  
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    accelSample[axis] += (readReverseShortI2C() >> 2);
  }
  accelSampleCount++;  
}

void evaluateMetersPerSec() {

  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    meterPerSec[axis] = (accelSample[axis] / accelSampleCount) * accelScaleFactor[axis] + runTimeAccelBias[axis];
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
    meterPerSec[axis] = (float(accelSample[axis])/SAMPLECOUNT) * accelScaleFactor[axis];
    accelSample[axis] = 0;
  }
  accelSampleCount = 0;

  runTimeAccelBias[XAXIS] = -meterPerSec[XAXIS];
  runTimeAccelBias[YAXIS] = -meterPerSec[YAXIS];
  runTimeAccelBias[ZAXIS] = -9.8065 - meterPerSec[ZAXIS];

  accelOneG = abs(meterPerSec[ZAXIS] + runTimeAccelBias[ZAXIS]);
}

#endif
