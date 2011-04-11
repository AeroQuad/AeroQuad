/*
  AeroQuad v2.4 - April 2011
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






/******************************************************/
/********************** CHR6DM Gyro **********************/
/******************************************************/
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
class Gyro_CHR6DM : public Gyro {

public:
  Gyro_CHR6DM() : Gyro() {
    gyroFullScaleOutput = 0;
    gyroScaleFactor = 0;
  }

  void initialize(void) {
    smoothFactor = readFloat(GYROSMOOTH_ADR);
    gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
    gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
    gyroZero[ZAXIS] = readFloat(GYRO_YAW_ZERO_ADR);
    initCHR6DM();
  }

  void measure(void) {
    //currentTime = micros();
    readCHR6DM();
    gyroADC[ROLL] = chr6dm.data.rollRate - gyroZero[ROLL]; //gx yawRate
    gyroADC[PITCH] = gyroZero[PITCH] - chr6dm.data.pitchRate; //gy pitchRate
    gyroADC[YAW] = chr6dm.data.yawRate - gyroZero[ZAXIS]; //gz rollRate

    //gyroData[ROLL] = filterSmoothWithTime(gyroADC[ROLL], gyroData[ROLL], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    //gyroData[PITCH] = filterSmoothWithTime(gyroADC[PITCH], gyroData[PITCH], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    //gyroData[YAW] = filterSmoothWithTime(gyroADC[YAW], gyroData[YAW], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[ROLL] = filterSmooth(gyroADC[ROLL], gyroData[ROLL], smoothFactor); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[PITCH] = filterSmooth(gyroADC[PITCH], gyroData[PITCH], smoothFactor); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[YAW] = filterSmooth(gyroADC[YAW], gyroData[YAW], smoothFactor); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    
    //previousTime = currentTime;
  }

  const int getFlightData(byte axis) {
    return getRaw(axis);
  }

  void calibrate() {

    float zeroXreads[FINDZERO];
    float zeroYreads[FINDZERO];
    float zeroZreads[FINDZERO];

    for (int i=0; i<FINDZERO; i++) {
        readCHR6DM();
        zeroXreads[i] = chr6dm.data.rollRate;
        zeroYreads[i] = chr6dm.data.pitchRate;
        zeroZreads[i] = chr6dm.data.yawRate;
    }

    gyroZero[XAXIS] = findMedian(zeroXreads, FINDZERO);
    gyroZero[YAXIS] = findMedian(zeroYreads, FINDZERO);
    gyroZero[ZAXIS] = findMedian(zeroZreads, FINDZERO);

    writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
  }
};
#endif

