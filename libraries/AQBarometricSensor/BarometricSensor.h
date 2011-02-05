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

// Class to define sensors that can determine altitude

// ***********************************************************************
// ************************** BarometricSensor Class *********************
// ***********************************************************************

#ifndef _BAROMETRIC_SENSOR_H
#define _BAROMETRIC_SENSOR_H

class BarometricSensor {
public:
  double altitude, rawAltitude;
  float groundTemperature; // remove later
  float groundPressure; // remove later
  float groundAltitude;
  float smoothFactor;
  
  BarometricSensor (void) { 
    altitude = 0;
    smoothFactor = 0.02;
  }

  // **********************************************************************
  // The following function calls must be defined inside any new subclasses
  // **********************************************************************
  virtual void initialize(void); 
  virtual void measure(void);
  
  // *********************************************************
  // The following functions are common between all subclasses
  // *********************************************************
  const float getData(void) {
    return altitude - groundAltitude;
  }
  
  const float getRawData(void) {
    return rawAltitude;
  }
  
  void setStartAltitude(float value) {
    altitude = value;
  }
  
  void measureGround(void) {
    // measure initial ground pressure (multiple samples)
    groundAltitude = 0;
    for (int i=0; i < 25; i++) {
      measure();
      delay(26);
      groundAltitude += rawAltitude;
    }
    groundAltitude = groundAltitude / 25.0;
  }
  
  void setGroundAltitude(float value) {
    groundAltitude = value;
  }
  
  const float getGroundAltitude(void) {
    return groundAltitude;
  }
  
  void setSmoothFactor(float value) {
    smoothFactor = value;
  }
  
  const float getSmoothFactor(void) {
    return smoothFactor;
  }
};

#endif