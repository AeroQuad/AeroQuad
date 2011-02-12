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

#ifndef _AQ_BMA180_ACCELEROMETER_H_
#define _AQ_BMA180_ACCELEROMETER_H_

#include "Accelerometer.h"

class BMA180Accelerometer : public Accelerometer 
{
private:
  int accelAddress;
  
public:
  BMA180Accelerometer() : Accelerometer()
  {
    accelAddress = 0x40; // page 54 and 61 of datasheet
    // Accelerometer value if BMA180 setup for 1.0G
    // Page 27 of datasheet = 0.00013g/LSB
    _accelScaleFactor = 0.00013;
  }
  
  void initialize() 
  {
    // Check if accel is connected
    if (readWhoI2C(accelAddress) != 0x03) // page 52 of datasheet
    {
      Serial.println("Accelerometer not found!");
    }

    // Thanks to SwiftingSpeed for updates on these settings
    // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11207&viewfull=1#post11207
    updateRegisterI2C(accelAddress, 0x10, 0xB6); //reset device
    delay(10);  //sleep 10 ms after reset (page 25)

    // In datasheet, summary register map is page 21
    // Low pass filter settings is page 27
    // Range settings is page 28
    updateRegisterI2C(accelAddress, 0x0D, 0x10); //enable writing to control registers
    sendByteI2C(accelAddress, 0x20); // register bw_tcs (bits 4-7)
    byte data = readByteI2C(accelAddress); // get current register value
    updateRegisterI2C(accelAddress, 0x20, data & 0x0F); // set low pass filter to 10Hz (value = 0000xxxx)

    // From page 27 of BMA180 Datasheet
    //  1.0g = 0.13 mg/LSB
    //  1.5g = 0.19 mg/LSB
    //  2.0g = 0.25 mg/LSB
    //  3.0g = 0.38 mg/LSB
    //  4.0g = 0.50 mg/LSB
    //  8.0g = 0.99 mg/LSB
    // 16.0g = 1.98 mg/LSB
    sendByteI2C(accelAddress, 0x35); // register offset_lsb1 (bits 1-3)
    data = readByteI2C(accelAddress);
    data &= 0xF1; // +/-1.0g (value = xxxx000x) // 0xF7;(3g)  //0xF5; (2g)
    updateRegisterI2C(accelAddress, 0x35, data);
  }
  
  void measure() 
  {
    int rawData[3];

    Wire.beginTransmission(accelAddress);
    Wire.send(0x02);
    Wire.endTransmission();
    Wire.requestFrom(accelAddress, 6);
    rawData[PITCH] = (Wire.receive()| (Wire.receive() << 8)) >> 2; // last 2 bits are not part of measurement
    rawData[ROLL] = (Wire.receive()| (Wire.receive() << 8)) >> 2; // last 2 bits are not part of measurement
    rawData[ZAXIS] = (Wire.receive()| (Wire.receive() << 8)) >> 2; // last 2 bits are not part of measurement
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      _accelADC[axis] = rawData[axis] - _accelZero[axis]; // center accel data around zero
      _accelData[axis] = filterSmooth(_accelADC[axis], _accelData[axis], _smoothFactor);
    }
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis) >> 4;
  }
  
  // Allows user to zero accelerometers on command
  void calibrate() 
  {  
    int findZero[FINDZERO];
    int dataAddress;
    
    for (byte calAxis = ROLL; calAxis < ZAXIS; calAxis++) 
    {
      if (calAxis == ROLL) dataAddress = 0x04;
      if (calAxis == PITCH) dataAddress = 0x02;
      if (calAxis == ZAXIS) dataAddress = 0x06;
      for (int i=0; i<FINDZERO; i++) 
      {
        sendByteI2C(accelAddress, dataAddress);
        findZero[i] = readReverseWordI2C(accelAddress) >> 2; // last two bits are not part of measurement
        delay(1);
      }
      _accelZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }

    // replace with estimated Z axis 0g value
    _accelZero[ZAXIS] = (_accelZero[ROLL] + _accelZero[PITCH]) / 2;
    // store accel value that represents 1g
    measure();
    _accelOneG = getRaw(ZAXIS);
    //accelOneG = 8274; // mesured value at flat level with configurator
  }
};

#endif