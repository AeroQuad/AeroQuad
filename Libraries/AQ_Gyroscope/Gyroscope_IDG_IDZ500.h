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

#ifndef _AEROQUAD_GYROSCOPE_IDG_IDZ500_H_
#define _AEROQUAD_GYROSCOPE_IDG_IDZ500_H_

#include <Gyroscope.h>

#define AZPIN 12 // Auto zero pin for IDG500 gyros

class Gyroscope_IDG_IDZ500 : public Gyroscope {
private:
  int gyroChannel[3];
  float aref;
  
public:
  Gyroscope_IDG_IDZ500() {
  }

  void setAref(float _aref) {
    aref = _aref;
    gyroScaleFactor = radians((aref/1024.0) / 0.002);  // IDG/IXZ500 sensitivity = 2mV/(deg/sec)
  } 

  void initialize(void) {
    analogReference(EXTERNAL);
    // Configure gyro auto zero pins
    pinMode (AZPIN, OUTPUT);
    digitalWrite(AZPIN, LOW);
    delay(1);

    // rollChannel = 4
    // pitchChannel = 3
    // yawChannel = 5
    gyroChannel[0] = 4;
    gyroChannel[1] = 3;
    gyroChannel[2] = 5;
  }
    
  void measure(void) {
    int gyroADC;
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      if (axis == PITCH)
        gyroADC = analogRead(gyroChannel[axis]) - zero[axis];
      else
        gyroADC = zero[axis] - analogRead(gyroChannel[axis]);
      rate[axis] = filterSmooth(gyroADC * gyroScaleFactor, rate[axis], smoothFactor);
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
    digitalWrite(AZPIN, HIGH);
    delayMicroseconds(750);
    digitalWrite(AZPIN, LOW);
    delay(8);

    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++)
        findZero[i] = analogRead(gyroChannel[calAxis]);
      zero[calAxis] = findMedianInt(findZero, FINDZERO);
    }
  }
};
#endif
