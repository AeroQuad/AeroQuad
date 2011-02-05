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

#ifndef _GYRO_CHR6DM_H_
#define _GYRO_CHR6DM_H_

#include <Gyroscope.h>

class GyroCHR6DMFake : public Gyroscope {
public:
  float fakeGyroRoll;
  float fakeGyroPitch;
  float fakeGyroYaw;
  
  GyroCHR6DMFake() : Gyroscope() {
    gyroFullScaleOutput = 0;
    gyroScaleFactor = 0;
  }

  void initialize(void) {
    smoothFactor = readFloat(GYROSMOOTH_ADR);
    gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
    gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
    gyroZero[ZAXIS] = readFloat(GYRO_YAW_ZERO_ADR);
    gyroZero[ROLL] = 0;
    gyroZero[PITCH] = 0;
    gyroZero[ZAXIS] = 0;
  }

  void measure(void) {
    currentTime = micros();
    readFakeValues();
    gyroADC[ROLL] = fakeGyroRoll - gyroZero[ROLL]; //gx yawRate
    gyroADC[PITCH] = fakeGyroPitch - gyroZero[PITCH]; //gy pitchRate
    gyroADC[YAW] = fakeGyroYaw - gyroZero[ZAXIS]; //gz rollRate

    gyroData[ROLL] = filterSmooth(gyroADC[ROLL], gyroData[ROLL], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[PITCH] = filterSmooth(gyroADC[PITCH], gyroData[PITCH], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[YAW] = filterSmooth(gyroADC[YAW], gyroData[YAW], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    previousTime = currentTime;
  }

  const int getFlightData(byte axis) {
    return getRaw(axis);
  }

  void calibrate() {
    float zeroXreads[FINDZERO];
    float zeroYreads[FINDZERO];
    float zeroZreads[FINDZERO];
    for (int i=0; i<FINDZERO; i++) {
        readFakeValues();
        zeroXreads[i] = fakeGyroRoll;
        zeroYreads[i] = fakeGyroPitch;
        zeroZreads[i] = fakeGyroYaw;
    }

    gyroZero[XAXIS] = findModeFloat(zeroXreads, FINDZERO);
    gyroZero[YAXIS] = findModeFloat(zeroYreads, FINDZERO);
    gyroZero[ZAXIS] = findModeFloat(zeroZreads, FINDZERO);

    writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
  }

  void readFakeValues(){
    if (!syncToHeader()){
        return;
    }

    fakeGyroRoll = readInt();
    fakeGyroPitch = readInt();
    fakeGyroYaw = readInt();

    fakeAccelRoll = readInt();
    fakeAccelPitch = readInt();
    fakeAccelYaw = readInt();

    Serial2.print("fakeGyroRoll=");
    Serial2.println(fakeGyroRoll);
    Serial2.print("fakeGyroPitch=");
    Serial2.println(fakeGyroPitch);
    Serial2.print("fakeGyroYaw=");
    Serial2.println(fakeGyroYaw);

    Serial2.print("fakeAccelRoll=");
    Serial2.println(fakeAccelRoll);
    Serial2.print("fakeAccelPitch=");
    Serial2.println(fakeAccelPitch);
    Serial2.print("fakeAccelYaw=");
    Serial2.println(fakeAccelYaw);
  }

  int readInt() {
    return word(blockingRead(),blockingRead());
  }

  int blockingRead() {
    int read=-1;

    long starttime = millis();
    while(read==-1 && (millis()-starttime)<100) {
      read = Serial2.read();
    }
    return read;
  }

  bool syncToHeader() {
    while (Serial2.available()>0){
      if (blockingRead()=='a' && blockingRead()=='b' && blockingRead()=='c' ) return true;
    }
    return false;
  }
};

#endif