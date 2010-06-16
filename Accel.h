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

class Accel {
public:
  float accelScaleFactor;
  float smoothFactor;
  int accelChannel[3];
  int accelZero[3];
  int accelData[3];
  int accelADC[3];
  byte rollChannel, pitchChannel, zAxisChannel;
  Accel(void) {}

  // ******************************************************************
  // The following function calls must be defined in any new subclasses
  // ******************************************************************
  virtual void initialize(void) {
    this->_initialize(rollChannel, pitchChannel, zAxisChannel);
    smoothFactor = readFloat(ACCSMOOTH_ADR);
  }
  virtual int measure(byte axis);
  virtual void calibrate(void);  

  // **************************************************************
  // The following functions are common between all Gyro subclasses
  // **************************************************************
  void _initialize(byte rollChannel, byte pitchChannel, byte zAxisChannel) {
    accelChannel[ROLL] = rollChannel;
    accelChannel[PITCH] = pitchChannel;
    accelChannel[ZAXIS] = zAxisChannel;
    
    accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
    accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
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
  
  float angleRad(byte axis) {
    if (axis == PITCH) return arctan2(accelData[PITCH], sqrt((long(accelData[ROLL]) * accelData[ROLL]) + (long(accelData[ZAXIS]) * accelData[ZAXIS])));
    if (axis == ROLL) return arctan2(accelData[ROLL], sqrt((long(accelData[PITCH]) * accelData[PITCH]) + (long(accelData[ZAXIS]) * accelData[ZAXIS])));
  }

  float angleDeg(byte axis) {
    return degrees(angleRad(axis));
  }
};

/******************************************************/
/************** AeroQuad Accelerometer ****************/
/******************************************************/
#ifdef AeroQuad_v1
class Accel_AeroQuad_v1 : public Accel {
private:
  int findZero[FINDZERO];
  
public:
  Accel_AeroQuad_v1() : Accel(){
    // Accelerometer Values
    // Update these variables if using a different accel
    // Output is ratiometric for ADXL 335
    // Note: Vs is not AREF voltage
    // If Vs = 3.6V, then output sensitivity is 360mV/g
    // If Vs = 2V, then it's 195 mV/g
    // Then if Vs = 3.3V, then it's 329.062 mV/g
    accelScaleFactor = 0.000329062;    
  }
  
  void initialize(void) {
    // rollChannel = 1
    // pitchChannel = 0
    // zAxisChannel = 2
    this->_initialize(1, 0, 2);
    smoothFactor = readFloat(ACCSMOOTH_ADR);
  }
  
  int measure(byte axis) {
    accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
    accelData[axis] = smooth(accelADC[axis], accelData[axis], smoothFactor);
    return accelData[axis];
  }

  // Allows user to zero accelerometers on command
  void calibrate(void) {
    for (byte calAxis = ROLL; calAxis < ZAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++)
        findZero[i] = analogRead(accelChannel[calAxis]);
      accelZero[calAxis] = findMode(findZero, FINDZERO);
    }

    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
};
#endif

/******************************************************/
/*************** APM ADC Accelerometer ****************/
/******************************************************/
#ifdef APM
class Accel_APM : public Accel {
private:
  int findZero[FINDZERO];
  int rawADC;

public:
  Accel_APM() : Accel(){
    // ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
    // ADXL335 Sensitivity(from datasheet) => 330mV/g, 0.8mV/ADC step => 330/0.8 = 412
    // Tested value : 414
    // #define GRAVITY 414 //this equivalent to 1G in the raw data coming from the accelerometer 
    // #define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square
    accelScaleFactor = 414.0 / 9.81;    
  }
  
  void initialize(void) {
    // rollChannel = 5
    // pitchChannel = 4
    // zAxisChannel = 6
    this->_initialize(5, 4, 6);
  }
  
  int measure(byte axis) {
    rawADC = analogRead_APM_ADC(accelChannel[axis]);
    if (rawADC > 500) // Check if measurement good
      accelADC[axis] = rawADC - accelZero[axis];
    accelData[axis] = accelADC[axis]; // no smoothing needed
    return accelData[axis];
  }

  // Allows user to zero accelerometers on command
  void calibrate(void) {
    for(byte calAxis = 0; calAxis < ZAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        findZero[i] = analogRead_APM_ADC(accelChannel[calAxis]);
        delay(10);
      }
      accelZero[calAxis] = findMode(findZero, FINDZERO);
    }
    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
};
#endif

/******************************************************/
/*************** APM ADC Accelerometer ****************/
/******************************************************/
#ifdef AeroQuad_Wii
class Accel_Wii : public Accel {
private:
  int findZero[FINDZERO];

public:
  Accel_Wii() : Accel(){
    accelScaleFactor = 0;    
  }
  
  void initialize(void) {
    smoothFactor = readFloat(ACCSMOOTH_ADR);
  }
  
  int measure(byte axis) {
    accelADC[axis] = NWMP_acc[axis] - accelZero[axis];
    accelData[axis] = smooth(accelADC[axis], accelData[axis], smoothFactor);
    return accelData[axis];
  }

  // Allows user to zero accelerometers on command
  void calibrate(void) {
    for(byte calAxis = ROLL; calAxis < ZAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        updateControls();
        findZero[i] = NWMP_acc[calAxis];
      }
      accelZero[] = findMode(findZero, FINDZERO);
    }
    
    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
};
#endif
