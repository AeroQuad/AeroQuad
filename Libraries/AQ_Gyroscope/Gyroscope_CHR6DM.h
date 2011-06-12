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

#ifndef _AEROQUAD_GYROSCOPE_CHR6DM_H_
#define _AEROQUAD_GYROSCOPE_CHR6DM_H_

#include <Gyroscope.h>
#include <Platform_CHR6DM.h>
#include <AQMath.h>

class Gyroscope_CHR6DM : public Gyroscope {
private:
  int gyroADC[3];
  CHR6DM *chr6dm;
  
public:
  Gyroscope_CHR6DM() {
  }

  void setChr6dm(CHR6DM *chr6dm) {
    this->chr6dm = chr6dm;
  }
  
  void measure(void) {
  
    gyroADC[ROLL] = chr6dm->data.rollRate - zero[ROLL]; //gx yawRate
    gyroADC[PITCH] = zero[PITCH] - chr6dm->data.pitchRate; //gy pitchRate
    gyroADC[YAW] = chr6dm->data.yawRate - zero[ZAXIS]; //gz rollRate

    rate[ROLL] = filterSmooth(gyroADC[ROLL], rate[ROLL], smoothFactor); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
    rate[PITCH] = filterSmooth(gyroADC[PITCH], rate[PITCH], smoothFactor); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
    rate[YAW] = filterSmooth(gyroADC[YAW], rate[YAW], smoothFactor); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1

    // Measure gyro heading
    long int currentTime = micros();
    if (rate[YAW] > radians(1.0) || rate[YAW] < radians(-1.0)) {
      heading += rate[YAW] * ((currentTime - lastMesuredTime) / 1000000.0);
    }
    lastMesuredTime = currentTime;
  }

  void calibrate() {
    float zeroXreads[FINDZERO];
    float zeroYreads[FINDZERO];
    float zeroZreads[FINDZERO];

    for (int i=0; i<FINDZERO; i++) {
        chr6dm->read();
        zeroXreads[i] = chr6dm->data.rollRate;
        zeroYreads[i] = chr6dm->data.pitchRate;
        zeroZreads[i] = chr6dm->data.yawRate;
    }

    zero[XAXIS] = findMedianFloat(zeroXreads, FINDZERO);
    zero[YAXIS] = findMedianFloat(zeroYreads, FINDZERO);
    zero[ZAXIS] = findMedianFloat(zeroZreads, FINDZERO);
  }
};

#endif  // #ifndef _AEROQUAD_GYROSCOPE_CHR6DM_H_

