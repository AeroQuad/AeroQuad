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

#ifndef _AQ_ALTITUDE_PROVIDER_H
#define _AQ_ALTITUDE_PROVIDER_H

class AltitudeProvider 
{
protected:
  float _groundPressure; // remove later
  float _groundTemperature; // remove later
  float _groundAltitude;  
  float _smoothFactor;
  double _rawAltitude;
  double _altitude; 
  
public:
  
  AltitudeProvider () 
  { 
    _altitude = 0;
    _smoothFactor = 0.02;
  }

  // **********************************************************************
  // The following function calls must be defined inside any new subclasses
  // **********************************************************************
  virtual void initialize(); 
  virtual void measure();
  
  // *********************************************************
  // The following functions are common between all subclasses
  // *********************************************************
  const float getData() 
  {
    return _altitude - _groundAltitude;
  }
  
  const float getRawData() 
  {
    return _rawAltitude;
  }
  
  void setStartAltitude(float value) 
  {
    _altitude = value;
  }
  
  void measureGround() 
  {
    // measure initial ground pressure (multiple samples)
    _groundAltitude = 0;
    for (int i=0; i < 25; i++) 
    {
      measure();
      delay(26);
      _groundAltitude += _rawAltitude;
    }
    _groundAltitude = _groundAltitude / 25.0;
  }
  
  void setGroundAltitude(float value) 
  {
    _groundAltitude = value;
  }
  
  const float getGroundAltitude() 
  {
    return _groundAltitude;
  }
  
  void setSmoothFactor(float value) 
  {
    _smoothFactor = value;
  }
  
  const float getSmoothFactor() 
  {
    return _smoothFactor;
  }
};

#endif