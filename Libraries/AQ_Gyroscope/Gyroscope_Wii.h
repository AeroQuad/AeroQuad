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

#ifndef _AEROQUAD_GYROSCOPE_WII_H_
#define _AEROQUAD_GYROSCOPE_WII_H_

#include <Gyroscope.h>
#include "../AQ_Platform_Wii/Platform_Wii.h"

class Gyroscope_Wii : public Gyroscope {
private:
  float wmpLowRangeToRadPerSec;
  float wmpHighRangeToRadPerSec;
  
  Platform_Wii *platformWii;
  

public:
  Gyroscope_Wii() {
    // Wii Motion+ has a low range and high range. Scaling is thought to be as follows:
    //
    // Vref = 1.35 volts
    // At 0 rate, reading is approximately 8063 bits
    // Scaling is then 1.35/8063, or 0.00016743 volts/bit
    //
    // Low Range
    //    440 degrees per second at 2.7 millivolts/degree (from datasheet)
    //    degrees per bit = 0.00016743 / 2.7 mVolts = 0.06201166 degrees per second per bit
    //                                              = 0.00108231 radians per second per bit
    // High Range
    //   2000 degrees per second at 0.5 millivolts/degree (from datasheet)
    //    degrees per bit = 0.00016743 / 0.5 mVolts = 0.33486295 degrees per second per bit
    //                                              = 0.00584446 radians per second per bit
    wmpLowRangeToRadPerSec  = 0.001082308;
    wmpHighRangeToRadPerSec = 0.005844461;

    lastMesuredTime = micros();
  }

  void setPlatformWii(Platform_Wii *platformWii) {
    this->platformWii = platformWii;
  }

  void measure() {
	int gyroADC[3];
    gyroADC[ROLL] = zero[ROLL] - platformWii->getGyroADC(ROLL);
	gyroADC[PITCH] = platformWii->getGyroADC(PITCH) - zero[PITCH];
	gyroADC[YAW] = zero[YAW] - platformWii->getGyroADC(YAW);
	
	for (byte axis = ROLL; axis < LASTAXIS; axis++) { 
      float gyroScaleFactor = platformWii->getWmpSlow(axis) ? wmpLowRangeToRadPerSec : wmpHighRangeToRadPerSec ;  // if wmpSlow == 1, use low range conversion,
      rate[axis] = filterSmooth(gyroADC[axis] * gyroScaleFactor, rate[axis], smoothFactor); 
    }
  
    // Measure gyro heading
    long int currentTime = micros();
    if (rate[YAW] > radians(1.0) || rate[YAW] < radians(-1.0)) {
      heading += rate[YAW] * ((currentTime - lastMesuredTime) / 1000000.0);
    }
    lastMesuredTime = currentTime;
}

  void calibrate() {
    int findZero[FINDZERO];
    
    for (byte axis = ROLL; axis <= YAW; axis++) {
      for (int i=0; i<FINDZERO; i++) {
	    platformWii->measure();
        findZero[i] = platformWii->getGyroADC(axis);
        delay(5);
      }
      zero[axis] = findMedianInt(findZero, FINDZERO);
    }
  }
};
#endif