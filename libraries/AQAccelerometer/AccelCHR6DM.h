/*
  AeroQuad v2.1 - January 2011
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

#ifndef _ACCEL_CHR6DM_H
#define _ACCEL_CHR6DM_H

#include <Accelerometer.h>


class AccelCHR6DM : public Accelerometer {
public:
  AccelCHR6DM() : Accelerometer() {
    accelScaleFactor = 0;
  }

  void initialize(void) {
    smoothFactor = readFloat(ACCSMOOTH_ADR);
    accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
    accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
    accelOneG = readFloat(ACCEL1G_ADR);
    calibrate();
  }

  void measure(void) {
    currentTime = micros();
      accelADC[XAXIS] = chr6dm.data.ax - accelZero[XAXIS];
      accelADC[YAXIS] = chr6dm.data.ay - accelZero[YAXIS];
      accelADC[ZAXIS] = chr6dm.data.az - accelOneG;

      accelData[XAXIS] = filterSmoothWithTime(accelADC[XAXIS], accelData[XAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0)); //to get around 1
      accelData[YAXIS] = filterSmoothWithTime(accelADC[YAXIS], accelData[YAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
      accelData[ZAXIS] = filterSmoothWithTime(accelADC[ZAXIS], accelData[ZAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
    previousTime = currentTime;
  }    

  const int getFlightData(byte axis) {
    return getRaw(axis);
  }

  // Allows user to zero accelerometers on command
  void calibrate(void) {

    float zeroXreads[FINDZERO];
    float zeroYreads[FINDZERO];
    float zeroZreads[FINDZERO];

    for (int i=0; i<FINDZERO; i++) {
      chr6dm.requestAndReadPacket();
      zeroXreads[i] = chr6dm.data.ax;
      zeroYreads[i] = chr6dm.data.ay;
      zeroZreads[i] = chr6dm.data.az;
    }


    accelZero[XAXIS] = findMode(zeroXreads, FINDZERO);
    accelZero[YAXIS] = findMode(zeroYreads, FINDZERO);
    accelZero[ZAXIS] = findMode(zeroZreads, FINDZERO);
   
    // store accel value that represents 1g
    accelOneG = accelZero[ZAXIS];
    // replace with estimated Z axis 0g value
    //accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;

    writeFloat(accelOneG, ACCEL1G_ADR);
    writeFloat(accelZero[XAXIS], LEVELROLLCAL_ADR);
    writeFloat(accelZero[YAXIS], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }

  void calculateAltitude() {
    currentTime = micros();
    if ((abs(CHR_RollAngle) < 5) && (abs(CHR_PitchAngle) < 5)) 
      rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    previousTime = currentTime;
  } 
};

#endif