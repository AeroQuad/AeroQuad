/*
  AeroQuad v3.0 - May 2011
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

#ifndef _AQ_BAROMETRIC_SENSOR_
#define _AQ_BAROMETRIC_SENSOR_


class BarometricSensor {
protected:
  double altitude;
  float smoothFactor;
  float groundAltitude;
  
public:
  
  BarometricSensor() { 
    altitude = 0;
    smoothFactor = 0.02;
  }

  // **********************************************************************
  // The following function calls must be defined inside any new subclasses
  // **********************************************************************
  virtual void initialize(); 
  virtual void measure();
  
  // *********************************************************
  // The following functions are common between all subclasses
  // *********************************************************
  const float getAltitude() {
    return altitude - groundAltitude;
  }
 
  void setSmoothFactor(float value) {
    smoothFactor = value;
  }
  
  const float getSmoothFactor() {
    return smoothFactor;
  }
};

#endif