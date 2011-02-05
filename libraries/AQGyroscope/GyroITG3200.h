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

#ifndef _GYRO_ITG3200_H_
#define _GYRO_ITG3200_H_

#include <Gyroscope.h>

/*
  10kOhm pull-ups on I2C lines.
  VDD & VIO = 3.3V
  SDA -> A4 (PC4)
  SCL -> A5 (PC5)
  INT -> D2 (PB2) (or no connection, not used here)
  CLK -> GND
*/
class GyroITG3200 : public Gyroscope {
private:
  int gyroAddress;
  long int previousGyroTime;
  
public:
  GyroITG3200() : Gyroscope() {
    gyroAddress = 0x69;
    gyroFullScaleOutput = 2000.0;   // ITG3200 full scale output = +/- 2000 deg/sec
    gyroScaleFactor = 1.0 / 14.375;       //  ITG3200 14.375 LSBs per °/sec
    
    lastReceiverYaw=0;
    yawAge=0;
    positiveGyroYawCount=1;
    negativeGyroYawCount=1;
    zeroGyroYawCount=1;
    previousGyroTime = micros();
  }
  
  void initialize(void) {
    this->_initialize(0,1,2);
    smoothFactor = readFloat(GYROSMOOTH_ADR);
    
    // Check if gyro is connected
    if (readWhoI2C(gyroAddress) != gyroAddress)
      Serial.println("Gyro not found!");
        
    // Thanks to SwiftingSpeed for updates on these settings
    // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11207&viewfull=1#post11207
    updateRegisterI2C(gyroAddress, 0x3E, 0x80); // send a reset to the device
    updateRegisterI2C(gyroAddress, 0x16, 0x1D); // 10Hz low pass filter
    updateRegisterI2C(gyroAddress, 0x3E, 0x01); // use internal oscillator 
  }
  
  void measure(void) {
    sendByteI2C(gyroAddress, 0x1D);
    Wire.requestFrom(gyroAddress, 6);

    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      gyroADC[axis] = ((Wire.receive() << 8) | Wire.receive()) - gyroZero[axis];
      gyroData[axis] = filterSmooth(gyroADC[axis], gyroData[axis], smoothFactor);
    }

    //calculateHeading();
    long int currentGyroTime = micros();
    rawHeading += -gyroADC[YAW] * gyroScaleFactor * ((currentGyroTime - previousGyroTime) / 1000000.0);
    //Serial.println(rawHeading);
    previousGyroTime = currentGyroTime;

    // ************ Correct for gyro drift by FabQuad **************
    // ************ http://aeroquad.com/entry.php?4-  **************
    // Modified FabQuad's approach to use yaw transmitter command instead of checking accelerometer
    if (abs(lastReceiverYaw - receiverYaw) < 15) {
      yawAge++;
      if (yawAge >= 4) {  // if gyro was the same long enough, we can assume that there is no (fast) rotation
        if (gyroData[YAW] < 0) {
          negativeGyroYawCount++; // if gyro still indicates negative rotation, that's additional signal that gyroZero is too low
        }
        else if (gyroData[YAW] > 0) {
          positiveGyroYawCount++;  // additional signal that gyroZero is too high
        }
        else {
          zeroGyroYawCount++; // additional signal that gyroZero is correct
        }
        yawAge = 0;
        if (zeroGyroYawCount + negativeGyroYawCount + positiveGyroYawCount > 50) {
          if (3*negativeGyroYawCount >= 4*(zeroGyroYawCount+positiveGyroYawCount)) gyroZero[YAW]--;  // enough signals the gyroZero is too low
          if (3*positiveGyroYawCount >= 4*(zeroGyroYawCount+negativeGyroYawCount)) gyroZero[YAW]++;  // enough signals the gyroZero is too high
          zeroGyroYawCount=0;
          negativeGyroYawCount=0;
          positiveGyroYawCount=0;
        }
      }
    }
    else { // gyro different, restart
      lastReceiverYaw = receiverYaw;
      yawAge = 0;
    }
  }
  
  const int getFlightData(byte axis) {
    int reducedData;
    
    reducedData = getRaw(axis) >> 3;
    //if ((reducedData < 5) && (reducedData > -5)) reducedData = 0;
    return reducedData;
  }

  void calibrate() {
    autoZero();
    writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
  }
  
  void autoZero() {
    int findZero[FINDZERO];
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        sendByteI2C(gyroAddress, (calAxis * 2) + 0x1D);
        findZero[i] = readWordI2C(gyroAddress);
        delay(10);
      }
      gyroZero[calAxis] = findModeInt(findZero, FINDZERO);
    }
  }
};

#endif