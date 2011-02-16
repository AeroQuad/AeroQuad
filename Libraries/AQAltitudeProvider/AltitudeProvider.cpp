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

#include "AltitudeProvider.h"

AltitudeProvider::AltitudeProvider () 
{ 
  _altitude = 0;
  _smoothFactor = 0.02;
}

// **********************************************************************
// The following function calls must be defined inside any new subclasses
// **********************************************************************
void AltitudeProvider::initialize() {}
void AltitudeProvider::measure() {}
  
// *********************************************************
// The following functions are common between all subclasses
// *********************************************************
const float AltitudeProvider::getData() 
{
  return _altitude - _groundAltitude;
}
  
const float AltitudeProvider::getRawData() 
{
  return _rawAltitude;
}
  
void AltitudeProvider::setStartAltitude(float value) 
{
  _altitude = value;
}
  
void AltitudeProvider::measureGround() 
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
  
void AltitudeProvider::setGroundAltitude(float value) 
{
  _groundAltitude = value;
}
  
const float AltitudeProvider::getGroundAltitude() 
{
  return _groundAltitude;
}
  
void AltitudeProvider::setSmoothFactor(float value) 
{
  _smoothFactor = value;
}
  
const float AltitudeProvider::getSmoothFactor() 
{
  return _smoothFactor;
}
