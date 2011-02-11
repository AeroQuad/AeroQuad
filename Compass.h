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

// Class to define sensors that can determine absolute heading

// ***********************************************************************
// ************************** Compass Class ******************************
// ***********************************************************************
class Compass 
{
private:  
  float _magMax[3];
  float _magMin[3];
  
protected:  
  float _absoluteHeading; 
  float _heading; 
  float _compass;
  int _compassAddress;
  float _gyroStartHeading;
  float _magScale[3];
  float _magOffset[3];
  
public: 

  Compass() {}

  // **********************************************************************
  // The following function calls must be defined inside any new subclasses
  // **********************************************************************
  virtual void initialize(); 
  virtual void measure();
  virtual const int getRawData(byte);
  
  // *********************************************************
  // The following functions are common between all subclasses
  // *********************************************************
  const float getData() 
  {
    return _compass;
  }
  
  const float getHeading() 
  {
    return _heading;
  }
  
  const float getAbsoluteHeading() 
  {
    return _absoluteHeading;
  }
  
  void setMagCal(byte axis, float maxValue, float minValue) 
  {
    _magMax[axis] = maxValue;
    _magMin[axis] = minValue;
    // Assume max/min is scaled to +1 and -1
    // y2 = 1, x2 = max; y1 = -1, x1 = min
    // m = (y2 - y1) / (x2 - x1)
    // m = 2 / (max - min)
    _magScale[axis] = 2.0 / (_magMax[axis] - _magMin[axis]);
    // b = y1 - mx1; b = -1 - (m * min)
    _magOffset[axis] = -(_magScale[axis] * _magMin[axis]) - 1;
  }
  
  const float getMagMax(byte axis) 
  {
    return _magMax[axis];
  }
  
  const float getMagMin(byte axis) 
  {
    return _magMin[axis];
  }
};

// ***********************************************************************
// ************************ HMC5843 Subclass *****************************
// ***********************************************************************
class HMC5843Magnetometer : public Compass 
{
// This sets up the HMC5843 from Sparkfun
private:
  float _cosRoll;
  float _sinRoll;
  float _cosPitch;
  float _sinPitch;
  float _magX;
  float _magY;
  int _measuredMagX;
  int _measuredMagY;
  int _measuredMagZ;
  float _smoothFactor; // time constant for complementary filter
  float _filter1;
  float _filter2; // coefficients for complementary filter
  float _adjustedGyroHeading;
  float _previousHead;
  int _gyroZero;
  
public: 
  HMC5843Magnetometer() : Compass() 
  {
    _compassAddress = 0x1E;
    // smoothFactor means time in seconds less than smoothFactor, depend on gyro more
    // time greater than smoothFactor depend on magnetometer more (mags are very noisy)
    _smoothFactor = 1.0; 
    _filter1 = _smoothFactor / (_smoothFactor + G_Dt);
    _filter2 = 1 - _filter1;
    _gyroZero = _gyro->getZero(YAW);
  }

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize() 
  {
    // Should do a WhoAmI to know if mag is present
    updateRegisterI2C(_compassAddress, 0x01, 0x20);
    updateRegisterI2C(_compassAddress, 0x02, 0x00); // continuous 10Hz mode
    measure();
    _gyroStartHeading = getData();
    if (_gyroStartHeading < 0) 
    {
      _gyroStartHeading += 360;
    }
    _gyro->setStartHeading(_gyroStartHeading);
  }
  
  const int getRawData(byte axis) 
  {
    if (axis == XAXIS) 
    {
      return _measuredMagX;
    }
    if (axis == YAXIS) 
    {
      return _measuredMagY;
    }
    if (axis == ZAXIS)
    {
      return _measuredMagZ;
    }
  }
  
  void measure() 
  {
    sendByteI2C(_compassAddress, 0x03);
    Wire.requestFrom(_compassAddress, 6);
    _measuredMagX = (Wire.receive() << 8) | Wire.receive();
    _measuredMagY = (Wire.receive() << 8) | Wire.receive();
    _measuredMagZ = (Wire.receive() << 8) | Wire.receive();
    Wire.endTransmission();
    // Heading calculation based on code written by FabQuad
    // http://aeroquad.com/showthread.php?691-Hold-your-heading-with-HMC5843-Magnetometer
    _cosRoll = cos(radians(_flightAngle->getData(ROLL)));
    _sinRoll = sin(radians(_flightAngle->getData(ROLL)));
    _cosPitch = cos(radians(_flightAngle->getData(PITCH)));
    _sinPitch = sin(radians(_flightAngle->getData(PITCH)));
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
};

// ***********************************************************************
// ************************* CHR6DM Subclass *****************************
// ***********************************************************************
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
class Compass_CHR6DM : public Compass 
{
public:
  Compass_CHR6DM() : Compass() {}
  
  void initialize() {}
  
  const int getRawData(byte) {}
  
  void measure() 
  {
    _heading = chr6dm.data.yaw; //this hardly needs any filtering :)
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
};

class Compass_CHR6DM_Fake : public Compass 
{
public:
  Compass_CHR6DM_Fake() : Compass() {}
  
  void initialize() {}
  
  const int getRawData(byte) {}
  
  void measure() 
  {
    _heading = 0;
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
};
#endif
