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

#include "Gyro_Null.h"
//#include <Wire.h>

void Gyro_Null::initialize() {
  for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
    data[axis] = 0;
    zero[axis] = 0;
  }
}

void Gyro_Null::measure(){
}

void Gyro_Null::calibrate(){
}

class RateGyro_AeroQuadMega_v2 : public RateGyro {
private:
  
public:
  RateGyro_AeroQuadMega_v2() : RateGyro() {
    gyroScaleFactor = DEG_2_RAD(1.0 / 14.375);  //  ITG3200 14.375 LSBs per °/sec
  }
  
////////////////////////////////////////////////////////////////////////////////
// Initialize AeroQuad v2.0 Gyro
////////////////////////////////////////////////////////////////////////////////

  void initialize(void) {
    updateRegisterI2C(0x69, 0x3E, 0x80); // send a reset to the device
    updateRegisterI2C(0x69, 0x16, 0x1D); // 10Hz low pass filter
    updateRegisterI2C(0x69, 0x3E, 0x01); // use internal oscillator 
  }
  
////////////////////////////////////////////////////////////////////////////////
// Measure AeroQuad v2.0 Gyro
////////////////////////////////////////////////////////////////////////////////

  void measure(void) {
    sendByteI2C(0x69, 0x1D);
    Wire.requestFrom(0x69, 6);
    
    // The following 3 lines read the gyro and assign it's data to gyroRaw
    // in the correct order and phase to suit the standard shield installation
    // orientation.  See TBD for details.  If your shield is not installed in this
    // orientation, this is where you make the changes.
    gyroRaw[ROLL]  = ((Wire.receive() << 8) | Wire.receive())  - gyroZero[ROLL];
    gyroRaw[PITCH] = gyroZero[PITCH] - ((Wire.receive() << 8) | Wire.receive());
    gyroRaw[YAW]   = gyroZero[YAW]   - ((Wire.receive() << 8) | Wire.receive());

    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      gyroVector[axis] = smooth(gyroRaw[axis] * gyroScaleFactor, gyroVector[axis], smoothFactor);
    }
  }
  
////////////////////////////////////////////////////////////////////////////////
// Calibrate AeroQuad v2.0 Gyro
////////////////////////////////////////////////////////////////////////////////

  void calibrate() {
    autoZero();
    writeFloat(gyroZero[ROLL],  GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW],   GYRO_YAW_ZERO_ADR);
  }
  
  void autoZero() {
    int findZero[FINDZERO];
    
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        sendByteI2C(0x69, (calAxis * 2) + 0x1D);
        findZero[i] = readWordI2C(0x69);
        delay(10);
      }
      gyroZero[calAxis] = findMode(findZero, FINDZERO);
    }
  }
  
////////////////////////////////////////////////////////////////////////////////
// Zero AeroQuad v2.0 Gyro
////////////////////////////////////////////////////////////////////////////////

  void zero() {
    // Not required for AeroQuad 2.0 Gyro
  }  
};