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

#include "Gyroscope_ITG3200.h"

#include <Wire.h>

#include "I2C.h"
#include "AQMath.h"


Gyroscope_ITG3200::Gyroscope_ITG3200() {
//  gyroScaleFactor = DEG_2_RAD(1.0 / 14.375);  //  ITG3200 14.375 LSBs per °/sec
  gyroScaleFactor = radians(1.0 / 14.375);  //  ITG3200 14.375 LSBs per °/sec
  measureDelay = 2;	// process or reading time for ITG3200 is 2ms
}
  

void Gyroscope_ITG3200::initialize(void) {
  smoothFactor = 1.0;
  updateRegisterI2C(0x69, 0x3E, 0x80); // send a reset to the device
  updateRegisterI2C(0x69, 0x16, 0x1D); // 10Hz low pass filter
  updateRegisterI2C(0x69, 0x3E, 0x01); // use internal oscillator 
}
  
void Gyroscope_ITG3200::measure(void) {

  unsigned long currentTime = millis();
  if ((currentTime - lastMeasuredTime) >= measureDelay) {
    float gyroRaw[3];
  
    sendByteI2C(0x69, 0x1D);
    Wire.requestFrom(0x69, 6);
    
    // The following 3 lines read the gyro and assign it's data to gyroRaw
    // in the correct order and phase to suit the standard shield installation
    // orientation.  See TBD for details.  If your shield is not installed in this
    // orientation, this is where you make the changes.
    gyroRaw[0]  = ((Wire.receive() << 8) | Wire.receive())  - zero[0];
    gyroRaw[1] = zero[1] - ((Wire.receive() << 8) | Wire.receive());
    gyroRaw[2]   = zero[2] - ((Wire.receive() << 8) | Wire.receive());

    for (byte axis = 0; axis < 3; axis++) {
      rate[axis] = filterSmooth(gyroRaw[axis] * gyroScaleFactor, rate[axis], smoothFactor);
    }
	lastMeasuredTime = currentTime;
  }
}
  
