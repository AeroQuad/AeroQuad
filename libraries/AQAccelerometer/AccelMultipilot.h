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

#ifndef _ACCEL_MULTIPILOT_H
#define _ACCEL_MULTIPILOT_H

#include <Accelerometer.h>

class AccelMultipilot : public Accelerometer {
private:
  
public:
  AccelMultipilot() : Accelerometer(){
    // Accelerometer Values
    // Update these variables if using a different accel
    // Output is ratiometric for ADXL 335
    // Note: Vs is not AREF voltage
    // If Vs = 3.6V, then output sensitivity is 360mV/g
    // If Vs = 2V, then it's 195 mV/g
    // Then if Vs = 3.3V, then it's 329.062 mV/g
    // Accelerometer Values for LIS344ALH set fs to +- 2G
    // Vdd = 3.3 Volt
    // Zero = Vdd / 2
    // 3300 mV / 5  (+-2G ) = 660
    accelScaleFactor = 0.000660;
  }
  
  void initialize(void) {
    // rollChannel = 6
    // pitchChannel = 7
    // zAxisChannel = 5
    this->_initialize(6, 7, 5);
    smoothFactor = readFloat(ACCSMOOTH_ADR);
  }
  
  void measure(void) {
    currentTime = micros();
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
      accelData[axis] = filterSmoothWithTime(accelADC[axis], accelData[axis], smoothFactor, ((currentTime - previousTime) / 5000.0));
    }
    previousTime = currentTime;
  }
  
  const int getFlightData(byte axis) {
    return getRaw(axis);
  }
  
  // Allows user to zero accelerometers on command
  void calibrate(void) {
    int findZero[FINDZERO];
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++)
        findZero[i] = analogRead(accelChannel[calAxis]);
      accelZero[calAxis] = findMode(findZero, FINDZERO);
    }

    // store accel value that represents 1g
    accelOneG = accelZero[ZAXIS];
    // replace with estimated Z axis 0g value
    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    
    writeFloat(accelOneG, ACCEL1G_ADR);
    writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }

  void calculateAltitude() {
    currentTime = micros();
    if ((abs(getRaw(ROLL)) < 1500) && (abs(getRaw(PITCH)) < 1500)) 
      rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    previousTime = currentTime;
  } 
};

#endif