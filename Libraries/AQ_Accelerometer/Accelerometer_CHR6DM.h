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

#ifndef _AEROQUAD_ACCELEROMETER_CHR6DM_H_
#define _AEROQUAD_ACCELEROMETER_CHR6DM_H_

#include <Accelerometer.h>
#include <Platform_CHR6DM.h>

class Accelerometer_CHR6DM : public Accelerometer {
private:
  CHR6DM *chr6dm;
  
public:
  Accelerometer_CHR6DM() {
    accelScaleFactor = 0;
    smoothFactor = 1.0;
  }

  void setChr6dm(CHR6DM *chr6dm) {
    this->chr6dm = chr6dm;
  }
  
  void measure(void) {
    float accelADC[3];
    accelADC[XAXIS] = chr6dm->data.ax - zero[XAXIS];
    accelADC[YAXIS] = chr6dm->data.ay - zero[YAXIS];
    accelADC[ZAXIS] = chr6dm->data.az - oneG;

    meterPerSec[XAXIS] = filterSmooth(accelADC[XAXIS], meterPerSec[XAXIS], smoothFactor); //to get around 1
    meterPerSec[YAXIS] = filterSmooth(accelADC[YAXIS], meterPerSec[YAXIS], smoothFactor);
    meterPerSec[ZAXIS] = filterSmooth(accelADC[ZAXIS], meterPerSec[ZAXIS], smoothFactor);
  }

  void calibrate() {
    float zeroXreads[FINDZERO];
    float zeroYreads[FINDZERO];
    float zeroZreads[FINDZERO];

    for (int i=0; i<FINDZERO; i++) {
        chr6dm->requestAndReadPacket();
        zeroXreads[i] = chr6dm->data.ax;
        zeroYreads[i] = chr6dm->data.ay;
        zeroZreads[i] = chr6dm->data.az;
    }

    zero[XAXIS] = findMedianFloat(zeroXreads, FINDZERO);
    zero[YAXIS] = findMedianFloat(zeroYreads, FINDZERO);
    zero[ZAXIS] = findMedianFloat(zeroZreads, FINDZERO);
   
    // store accel value that represents 1g
    oneG = zero[ZAXIS];
  }
};
#endif
