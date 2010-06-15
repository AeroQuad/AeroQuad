/*
  AeroQuad v1.8 - June 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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

/******************************************************/
/*************** Analog Accelerometer *****************/
/******************************************************/
class Accel_ADXL330 {
private:
  float accelScaleFactor; // defined in initialize()
  float smoothFactor;
  int accelChannel[3];
  int accelZero[3];
  int accelData[3];
  int accelADC[3];
  
public:
  Accel_ADXL330(void) {
    // Accelerometer Values
    // Update these variables if using a different accel
    // Output is ratiometric for ADXL 335
    // Note: Vs is not AREF voltage
    // If Vs = 3.6V, then output sensitivity is 360mV/g
    // If Vs = 2V, then it's 195 mV/g
    // Then if Vs = 3.3V, then it's 329.062 mV/g
    accelScaleFactor = 0.000329062;    
  }
  
  void initialize(byte rollChannel, byte pitchChannel, byte zAxisChannel) {
    accelChannel[ROLL] = rollChannel;
    accelChannel[PITCH] = pitchChannel;
    accelChannel[ZAXIS] = zAxisChannel;
    
    accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
    accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
    
    smoothFactor = readFloat(ACCSMOOTH_ADR);
  }
  
  int measure(byte axis) {
    accelADC[axis] = analogRead(accelChannel[axis] - accelZero[axis]);
    accelData[axis] = smooth(accelADC[axis], accelData[axis], smoothFactor);
    return accelData[axis];
  }
  
  int getRaw(byte axis) {
    return accelADC[axis];
  }
  
  int getData(byte axis) {
    return accelData[axis];
  }
  
  int invert(byte axis) {
    accelData[axis] = -accelData[axis];
    return accelData[axis];
  }
  
  int getZero(byte axis) {
    return accelZero[axis];
  }
  
  void setZero(byte axis, int value) {
    accelZero[axis] = value;
  }
  
  float getScaleFactor(void) {
    return accelScaleFactor;
  }
  
  float getSmoothFactor() {
    return smoothFactor;
  }
  
  void setSmoothFactor(float value) {
    smoothFactor = value;
  }
  
  // Allows user to zero accelerometers on command
  void calibrate(void) {
    for(axis = ROLL; axis > ZAXIS; axis++) {
      for (int i=0; i<FINDZERO; i++)
        findZero[i] = analogRead(accelChannel[axis]);
      accelZero[axis] = findMode(findZero, FINDZERO);
    }
    
    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
  
  float angleRad(byte axis) {
    if (axis == PITCH) return arctan2(accelData[PITCH], sqrt((long(accelData[ROLL]) * accelData[ROLL]) + (long(accelData[ZAXIS]) * accelData[ZAXIS])));
    if (axis == ROLL) return arctan2(accelData[ROLL], sqrt((long(accelData[PITCH]) * accelData[PITCH]) + (long(accelData[ZAXIS]) * accelData[ZAXIS])));
  }

  float angleDeg(byte axis) {
    return degrees(angleRad(axis));
  } 
};

