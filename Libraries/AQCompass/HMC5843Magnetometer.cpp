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


#include "HMC5843Magnetometer.h"

#include <I2C.h>


HMC5843Magnetometer::HMC5843Magnetometer()
{
  _compassAddress = 0x1E;
  // smoothFactor means time in seconds less than smoothFactor, depend on gyro more
  // time greater than smoothFactor depend on magnetometer more (mags are very noisy)
  _smoothFactor = 1.0; 
  _filter1 = _smoothFactor / (_smoothFactor + 0.002);
  _filter2 = 1 - _filter1;
  _gyroZero = _gyro->getZero(YAW);
}

// ***********************************************************
// Define all the virtual functions declared in the main class
// ***********************************************************
void HMC5843Magnetometer::initialize() 
{
  // Should do a WhoAmI to know if mag is present
  updateRegisterI2C(_compassAddress, 0x01, 0x20);
  updateRegisterI2C(_compassAddress, 0x02, 0x00); // continuous 10Hz mode
  measure(0.0,0.0);
  _gyroStartHeading = getData();
  if (_gyroStartHeading < 0) 
  {
    _gyroStartHeading += 360;
  }
  _gyro->setStartHeading(_gyroStartHeading);
}
  
const int HMC5843Magnetometer::getRawData(byte axis) 
{
  if (axis == XAXIS) 
  {
    return _measuredMagX;
  }
  if (axis == YAXIS) 
  {
    return _measuredMagY;
  }
  // then, it's zaxis
  return _measuredMagZ;
}
  
void HMC5843Magnetometer::measure(const float rollAngle, const float pitchAngle) 
{
  sendByteI2C(_compassAddress, 0x03);
  Wire.requestFrom(_compassAddress, 6);
  _measuredMagX = (Wire.receive() << 8) | Wire.receive();
  _measuredMagY = (Wire.receive() << 8) | Wire.receive();
  _measuredMagZ = (Wire.receive() << 8) | Wire.receive();
  Wire.endTransmission();
  // Heading calculation based on code written by FabQuad
  // http://aeroquad.com/showthread.php?691-Hold-your-heading-with-HMC5843-Magnetometer
  _cosRoll = cos(radians(rollAngle));
  _sinRoll = sin(radians(rollAngle));
  _cosPitch = cos(radians(pitchAngle));
  _sinPitch = sin(radians(pitchAngle));
  _magX = ((float)_measuredMagX * _magScale[XAXIS] + _magOffset[XAXIS]) * _cosPitch + 
          ((float)_measuredMagY * _magScale[YAXIS] + _magOffset[YAXIS]) * _sinRoll * _sinPitch + 
          ((float)_measuredMagZ * _magScale[ZAXIS] + _magOffset[ZAXIS]) * _cosRoll * _sinPitch;
  _magY = ((float)_measuredMagY * _magScale[YAXIS] + _magOffset[YAXIS]) * _cosRoll - 
          ((float)_measuredMagZ * _magScale[ZAXIS] + _magOffset[ZAXIS]) * _sinRoll;
  //magX = measuredMagX * cosPitch + measuredMagY * sinRoll * sinPitch + measuredMagZ * cosRoll * sinPitch;
  //magY = measuredMagY * cosRoll - measuredMagZ * sinRoll;   
  _compass = -degrees(atan2(-_magY, _magX));
   
  // Check if gyroZero adjusted, if it is, reset gyroHeading to compass value
  if (_gyroZero != _gyro->getZero(YAW)) 
  {
    _gyro->setStartHeading(_heading);
    _gyroZero = _gyro->getZero(YAW);
  }
    
  _adjustedGyroHeading = _gyro->getHeading();
  // if compass is positive while gyro is negative force gyro positive past 180
  if ((_compass > 90) && _adjustedGyroHeading < -90)
  {
    _adjustedGyroHeading += 360;
  }
  // if compass is negative whie gyro is positive force gyro negative past -180
  if ((_compass < -90) && _adjustedGyroHeading > 90)
  {
    _adjustedGyroHeading -= 360;
  }
    
  // Complementry filter from http://chiefdelphi.com/media/papers/2010
  _heading = (_filter1 * _adjustedGyroHeading) + (_filter2 * _compass);
    
  // Change from +/-180 to 0-360
  if (_heading < 0)
  { 
    _absoluteHeading = 360 + _heading;
  }
  else 
  {
    _absoluteHeading = _heading;
  }
}
