/*
  AeroQuad v2.2 - Feburary 2011
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

#ifndef _AQ_ITG3200_GYROSCOPE_H_
#define _AQ_ITG3200_GYROSCOPE_H_

#include "Gyroscope.h"

/*
  10kOhm pull-ups on I2C lines.
  VDD & VIO = 3.3V
  SDA -> A4 (PC4)
  SCL -> A5 (PC5)
  INT -> D2 (PB2) (or no connection, not used here)
  CLK -> GND
*/
class ITG3200Gyroscope : public Gyroscope 
{
private:
  int _gyroAddress;
  long int _previousGyroTime;
  
public:
  ITG3200Gyroscope() : Gyroscope() 
  {
    _gyroAddress = 0x69;
    _gyroFullScaleOutput = 2000.0;   // ITG3200 full scale output = +/- 2000 deg/sec
    _gyroScaleFactor = 1.0 / 14.375;       //  ITG3200 14.375 LSBs per °/sec
    
    _lastReceiverYaw=0;
    _yawAge=0;
    _positiveGyroYawCount=1;
    _negativeGyroYawCount=1;
    _zeroGyroYawCount=1;
    _previousGyroTime = micros();
  }
  
  void initialize() 
  {
    this->_initialize(0,1,2);
    
    // Check if gyro is connected
    if (readWhoI2C(_gyroAddress) != _gyroAddress)
    {
      Serial.println("Gyro not found!");
    }
        
    // Thanks to SwiftingSpeed for updates on these settings
    // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11207&viewfull=1#post11207
    updateRegisterI2C(_gyroAddress, 0x3E, 0x80); // send a reset to the device
    updateRegisterI2C(_gyroAddress, 0x16, 0x1D); // 10Hz low pass filter
    updateRegisterI2C(_gyroAddress, 0x3E, 0x01); // use internal oscillator 
  }
  
  void measure() 
  {
    sendByteI2C(_gyroAddress, 0x1D);
    Wire.requestFrom(_gyroAddress, 6);

    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      _gyroADC[axis] = ((Wire.receive() << 8) | Wire.receive()) - _gyroZero[axis];
      _gyroData[axis] = filterSmooth(_gyroADC[axis], _gyroData[axis], _smoothFactor);
    }

    //calculateHeading();
    long int currentGyroTime = micros();
    _rawHeading += -_gyroADC[YAW] * _gyroScaleFactor * ((currentGyroTime - _previousGyroTime) / 1000000.0);
    //Serial.println(rawHeading);
    _previousGyroTime = currentGyroTime;

    // ************ Correct for gyro drift by FabQuad **************
    // ************ http://aeroquad.com/entry.php?4-  **************
    // Modified FabQuad's approach to use yaw transmitter command instead of checking accelerometer
    if (abs(_lastReceiverYaw - _receiverYaw) < 15) 
    {
      _yawAge++;
      if (_yawAge >= 4) 
      {  // if gyro was the same long enough, we can assume that there is no (fast) rotation
        if (_gyroData[YAW] < 0) 
        {
          _negativeGyroYawCount++; // if gyro still indicates negative rotation, that's additional signal that gyroZero is too low
        }
        else if (_gyroData[YAW] > 0) 
        {
          _positiveGyroYawCount++;  // additional signal that gyroZero is too high
        }
        else 
        {
          _zeroGyroYawCount++; // additional signal that gyroZero is correct
        }
        _yawAge = 0;
        if (_zeroGyroYawCount + _negativeGyroYawCount + _positiveGyroYawCount > 50) 
        {
          if (3 * _negativeGyroYawCount >= 4 * (_zeroGyroYawCount + _positiveGyroYawCount)) 
          {
            _gyroZero[YAW]--;  // enough signals the gyroZero is too low
          }
          if (3 * _positiveGyroYawCount >= 4 * (_zeroGyroYawCount + _negativeGyroYawCount))
          {
            _gyroZero[YAW]++;  // enough signals the gyroZero is too high
          }
          _zeroGyroYawCount=0;
          _negativeGyroYawCount=0;
          _positiveGyroYawCount=0;
        }
      }
    }
    else 
    { // gyro different, restart
      _lastReceiverYaw = _receiverYaw;
      _yawAge = 0;
    }
  }
  
  const int getFlightData(byte axis) 
  {
    int reducedData = getRaw(axis) >> 3;
    //if ((reducedData < 5) && (reducedData > -5)) reducedData = 0;
    return reducedData;
  }

  void calibrate() 
  {
    autoZero();
  }
  
  void autoZero() 
  {
    int findZero[FINDZERO];
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++) 
      {
        sendByteI2C(_gyroAddress, (calAxis * 2) + 0x1D);
        findZero[i] = readWordI2C(_gyroAddress);
        delay(10);
      }
      _gyroZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }
  }
};

#endif