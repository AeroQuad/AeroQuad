/*
  AeroQuad v3.0 - February 2011
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

#include "Accelerometer_BMA180.h"

Accelerometer_BMA180::Accelerometer_BMA180() {
  accelAddress = 0x40; // page 54 and 61 of datasheet
  // Accelerometer value if BMA180 setup for 1.0G
  // Page 27 of datasheet = 0.00013g/LSB
  accelScaleFactor = G_2_MPS2(1.0/4096.0);  //  g per LSB @ +/- 2g range
}

void Accelerometer_BMA180::initialize() {
  byte data;
  
   i2c.updateRegister(0x40, 0x10, 0xB6); // reset device
  delay(10); // sleep 10 ms after reset (page 25)

  i2c.updateRegister(0x40, 0x0D, 0x10);  // enable writing to control registers
  i2c.sendByte(0x40, 0x20); // register bw_tcs (bits 4-7)
  data = i2c.readByte(0x40); // get current register value
  i2c.updateRegister(0x40, 0x20, data & 0x0F); // set low pass filter to 10Hz (value = 0000xxxx)

  i2c.sendByte(0x40, 0x35); // Register offset_lsb1 (bits 1-3)
  data = i2c.readByte(0x40);
  data &= 0xF1;  // Clear range select bits
  data |= 0x04;  // Set range select bits for +/-2g
  i2c.updateRegister(0x40, 0x35, data);  // (value = xxxx010x)
}

void Accelerometer_BMA180::measure() {
  i2c.sendByte(0x40, 0x02);
  Wire.requestFrom(0x40, 6);

  // The following 3 lines read the accelerometer and assign it's data to accelVectorBits
  // in the correct order and phase to suit the standard shield installation
  // orientation.  See TBD for details.  If your shield is not installed in this
  // orientation, this is where you make the changes.
  accelRaw[XAXIS] = ((Wire.receive()|(Wire.receive() << 8)) >> 2) - accelZero[XAXIS];
  accelRaw[YAXIS] = accelZero[YAXIS] - ((Wire.receive()|(Wire.receive() << 8)) >> 2);
  accelRaw[ZAXIS] = accelZero[ZAXIS] - ((Wire.receive()|(Wire.receive() << 8)) >> 2);

  //for (byte axis = XAXIS; axis < LASTAXIS; axis++)
    //accelVector[axis] = smooth(accelRaw[axis] * accelScaleFactor, accelVector[axis], smoothFactor);
}

void Accelerometer_BMA180::calibrate() {
  int findZero[FINDZERO];
  int dataAddress;
  
  for (byte calAxis = XAXIS; calAxis < ZAXIS; calAxis++) {
    if (calAxis == XAXIS) dataAddress = 0x02;
    if (calAxis == YAXIS) dataAddress = 0x04;
    if (calAxis == ZAXIS) dataAddress = 0x06;
    for (int i=0; i<FINDZERO; i++) {
      i2c.sendByte(0x40, dataAddress);
      findZero[i] = i2c.readReverseWord(0x40) >> 2; // last two bits are not part of measurement
      delay(10);
    }
    accelZero[calAxis] = findMedian(findZero, FINDZERO);
  }

  // replace with estimated Z axis 0g value
  accelZero[ZAXIS] = (accelZero[XAXIS] + accelZero[YAXIS]) / 2;
  
  // store accel value that represents 1g
  measure();
  accelOneG = -accelVector[ZAXIS];
}