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

// Class to define sensors that can determine absolute heading

#include "MagnetometerHMC5843.h"

#include <AxisDefine.h>
#include <Wire.h>
#include <I2C.h>

MagnetometerHMC5843::MagnetometerHMC5843(Gyroscope gyroscope)
{
  _gyroscope = &gyroscope;
  compassAddress = 0x1E;
  // smoothFactor means time in seconds less than smoothFactor, depend on gyro more
  // time greater than smoothFactor depend on magnetometer more (mags are very noisy)
  smoothFactor = 1.0; 
  filter1 = smoothFactor / (smoothFactor);// + G_Dt);
  filter2 = 1 - filter1;
  gyroZero = _gyroscope->getZero(YAW);
}

// ***********************************************************
// Define all the virtual functions declared in the main class
// ***********************************************************
void MagnetometerHMC5843::initialize(void) 
{
  // Should do a WhoAmI to know if mag is present
  updateRegisterI2C(compassAddress, 0x01, 0x20);
  updateRegisterI2C(compassAddress, 0x02, 0x00); // continuous 10Hz mode
  measure(0,0);
  gyroStartHeading = getData();
  if (gyroStartHeading < 0) 
  {
    gyroStartHeading += 360;
  }
  _gyroscope->setStartHeading(gyroStartHeading);
}
  
const int MagnetometerHMC5843::getRawData(byte axis) 
{
  if (axis == XAXIS) 
  {
    return measuredMagX;
  }
  if (axis == YAXIS) 
  {
    return measuredMagY;
  }
  return measuredMagZ;
}
  
void MagnetometerHMC5843::measure(float angleRoll, float anglePitch) 
{
  sendByteI2C(compassAddress, 0x03);
  Wire.requestFrom(compassAddress, 6);
  measuredMagX = (Wire.receive() << 8) | Wire.receive();
  measuredMagY = (Wire.receive() << 8) | Wire.receive();
  measuredMagZ = (Wire.receive() << 8) | Wire.receive();
  Wire.endTransmission();
  // Heading calculation based on code written by FabQuad
  // http://aeroquad.com/showthread.php?691-Hold-your-heading-with-HMC5843-Magnetometer
  cosRoll = cos(radians(angleRoll));
  sinRoll = sin(radians(angleRoll));
  cosPitch = cos(radians(anglePitch));
  sinPitch = sin(radians(anglePitch));
  magX = ((float)measuredMagX * magScale[XAXIS] + magOffset[XAXIS]) * cosPitch + ((float)measuredMagY * magScale[YAXIS] + magOffset[YAXIS]) * sinRoll * sinPitch + ((float)measuredMagZ * magScale[ZAXIS] + magOffset[ZAXIS]) * cosRoll * sinPitch;
  magY = ((float)measuredMagY * magScale[YAXIS] + magOffset[YAXIS]) * cosRoll - ((float)measuredMagZ * magScale[ZAXIS] + magOffset[ZAXIS]) * sinRoll;
  //magX = measuredMagX * cosPitch + measuredMagY * sinRoll * sinPitch + measuredMagZ * cosRoll * sinPitch;
  //magY = measuredMagY * cosRoll - measuredMagZ * sinRoll;   
  compass = -degrees(atan2(-magY, magX));
    
  // Check if gyroZero adjusted, if it is, reset gyroHeading to compass value
  if (gyroZero != _gyroscope->getZero(YAW)) 
  {
    _gyroscope->setStartHeading(heading);
    gyroZero = _gyroscope->getZero(YAW);
  }
    
  adjustedGyroHeading = _gyroscope->getHeading();
  // if compass is positive while gyro is negative force gyro positive past 180
  if ((compass > 90) && adjustedGyroHeading < -90) 
  {
    adjustedGyroHeading += 360;
  }
  // if compass is negative whie gyro is positive force gyro negative past -180
  if ((compass < -90) && adjustedGyroHeading > 90) 
  {
    adjustedGyroHeading -= 360;
  }
  
  // Complementry filter from http://chiefdelphi.com/media/papers/2010
  heading = (filter1 * adjustedGyroHeading) + (filter2 * compass);
   
  // Change from +/-180 to 0-360
  if (heading < 0) 
  {
    absoluteHeading = 360 + heading;
  }
  else 
  {
    absoluteHeading = heading;
  }
}

