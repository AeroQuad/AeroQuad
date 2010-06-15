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

class Gyro {
public:
  float gyroFullScaleOutput;
  float gyroScaleFactor;
  int gyroChannel[3];
  int gyroData[3];
  int gyroZero[3];
  int gyroADC[3];
  Gyro(void){}

  // The following function calls must be defined in any new subclasses
  virtual void initialize(byte rollChannel, byte pitchChannel, byte yawChannel) {
    this->_initialize(rollChannel, pitchChannel, yawChannel);
  }
  virtual int measure(byte axis);
  virtual void calibrate(void);  
  
  // The following functions are common between all Gyro subclasses
  void _initialize(byte rollChannel, byte pitchChannel, byte yawChannel) {
    gyroChannel[ROLL] = rollChannel;
    gyroChannel[PITCH] = pitchChannel;
    gyroChannel[ZAXIS] = yawChannel;
    
    gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
    gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
    gyroZero[ZAXIS] = readFloat(GYRO_YAW_ZERO_ADR);
  }
    
  int getRaw(byte axis) {
    return gyroADC[axis];
  }
  
  int getData(byte axis) {
    return gyroData[axis];
  }
  
  int setData(byte axis, int value) {
    gyroData[axis] = value;
    return gyroData[axis];
  }
  
  int invert(byte axis) {
    gyroData[axis] = -gyroData[axis];
    return gyroData[axis];
  }
  
  int getZero(byte axis) {
    return gyroZero[axis];
  }
  
  int setZero(byte axis, int value) {
    gyroZero[axis] = value;
    return gyroZero[axis];
  }    
  
  float getScaleFactor() {
    return gyroScaleFactor;
  }

  float rateDegPerSec(byte axis) {
    return (gyroADC[axis] / 1024.0) * aref / gyroScaleFactor;
  }

  float rateRadPerSec(byte axis) {
    return radians((gyroADC[axis] / 1024.0) * aref / gyroScaleFactor);
  }
};

/******************************************************/
/******************** Analog Gyro *********************/
/******************************************************/

class Gyro_IDG_IXZ_500 : public Gyro {
private:
  float smoothFactor;
  
public:
  Gyro_IDG_IXZ_500() : Gyro() {
    gyroFullScaleOutput = 500.0;   // IDG/IXZ500 full scale output = +/- 500 deg/sec
    gyroScaleFactor = 0.002;       // IDG/IXZ500 sensitivity = 2mV/(deg/sec)
  }
  
  void initialize(byte rollChannel, byte pitchChannel, byte yawChannel) {
    this->_initialize(rollChannel, pitchChannel, yawChannel);
    smoothFactor = readFloat(GYROSMOOTH_ADR);
  }
  
  int measure(byte axis) {
    gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
    gyroData[axis] = smooth(gyroADC[axis], gyroData[axis], smoothFactor) ;
    return gyroData[axis];
  }

  float getSmoothFactor(void) {
    return smoothFactor;
  }
  
  void setSmoothFactor(float value) {
    smoothFactor = value;
  }

  void calibrate() {
    autoZero();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      for (int i=0; i<FINDZERO; i++)
        findZero[i] = analogRead(gyroChannel[axis]);
      gyroZero[axis] = findMode(findZero, FINDZERO);
    }
    writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
  }
  
  void autoZero() {
    digitalWrite(AZPIN, HIGH);
    delayMicroseconds(750);
    digitalWrite(AZPIN, LOW);
    delay(8);
  }    
};

