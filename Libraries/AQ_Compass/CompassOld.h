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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Compass {
public: 
  float magMax[3];
  float magMin[3];
  float magScale[3];
  float magOffset[3];
  float hdgX;
  float hdgY;
  int   measuredMagX;
  int   measuredMagY;
  int   measuredMagZ;
  
  Compass(void) {}

  const float getHdgXY(byte axis) {
    if (axis == XAXIS) return hdgX;
    if (axis == YAXIS) return hdgY;
  }

    const int getRawData(byte axis) {
    if (axis == XAXIS) return measuredMagX;
    if (axis == YAXIS) return measuredMagY;
    if (axis == ZAXIS) return measuredMagZ;
  }
  
  void setMagCal(byte axis, float maxValue, float minValue) {
    magMax[axis] = maxValue;
    magMin[axis] = minValue;
    // Assume max/min is scaled to +1 and -1
    // y2 = 1, x2 = max; y1 = -1, x1 = min
    // m = (y2 - y1) / (x2 - x1)
    // m = 2 / (max - min)
    magScale[axis] = 2.0 / (magMax[axis] - magMin[axis]);
    // b = y1 - mx1; b = -1 - (m * min)
    magOffset[axis] = -(magScale[axis] * magMin[axis]) - 1;
  }
  
  const float getMagMax(byte axis) {
    return magMax[axis];
  }
  
  const float getMagMin(byte axis) {
    return magMin[axis];
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Magnetometer (HMC5843)
////////////////////////////////////////////////////////////////////////////////

class Magnetometer_HMC5843 : public Compass {
private:
  float cosRoll;
  float sinRoll;
  float cosPitch;
  float sinPitch;

public: 
  Magnetometer_HMC5843() : Compass() {}

  ////////////////////////////////////////////////////////////////////////////////
  // Initialize AeroQuad Mega v2.0 Magnetometer
  ////////////////////////////////////////////////////////////////////////////////

 void initialize(void) {
    updateRegisterI2C(0x1E, 0x01, 0x20);
    updateRegisterI2C(0x1E, 0x02, 0x00); // continuous 10Hz mode
    
    measure(0.0, 0.0);  // Assume 1st measurement at 0 degrees roll and 0 degrees pitch
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  // Measure AeroQuad Mega v2.0 Magnetometer
  ////////////////////////////////////////////////////////////////////////////////

  void measure(float roll, float pitch) {
    float magX;
    float magY;
    float tmp;
    
    sendByteI2C(0x1E, 0x03);
    Wire.requestFrom(0x1E, 6);

    // The following 3 lines read the magnetometer and assign it's data to measuredMagX
    // Y and Z in the correct order and phase to suit the standard shield installation
    // orientation.  See TBD for details.  If your shield/sensor is not installed in this
    // orientation, this is where you make the changes.
    
    #ifdef AEROQUAD_MEGA_V2  // Sparkfun board on AeroQuad 2.0.6 shield
      measuredMagX = (Wire.receive() << 8) | Wire.receive();
      measuredMagY = (Wire.receive() << 8) | Wire.receive();
      measuredMagZ = (Wire.receive() << 8) | Wire.receive();
    #endif
    
    #ifdef APM  // DIY Drones board with pins facing aft, components facing up
      measuredMagX = (Wire.receive() << 8) | Wire.receive();
      measuredMagY = (Wire.receive() << 8) | Wire.receive();
      measuredMagZ = (Wire.receive() << 8) | Wire.receive();
    #endif
    
    #ifdef AEROQUAD_WII
      measuredMagX = (Wire.receive() << 8) | Wire.receive();
      measuredMagY = (Wire.receive() << 8) | Wire.receive();
      measuredMagZ = (Wire.receive() << 8) | Wire.receive();
    #endif
    
    Wire.endTransmission();

    cosRoll =  cos(roll);
    sinRoll =  sin(roll);
    cosPitch = cos(pitch);
    sinPitch = sin(pitch);

    magX = ((float)measuredMagX * magScale[XAXIS] + magOffset[XAXIS]) * cosPitch + \
           ((float)measuredMagY * magScale[YAXIS] + magOffset[YAXIS]) * sinRoll * sinPitch + \
           ((float)measuredMagZ * magScale[ZAXIS] + magOffset[ZAXIS]) * cosRoll * sinPitch;
           
    magY = ((float)measuredMagY * magScale[YAXIS] + magOffset[YAXIS]) * cosRoll - \
           ((float)measuredMagZ * magScale[ZAXIS] + magOffset[ZAXIS]) * sinRoll;

    tmp  = sqrt(magX * magX + magY * magY);
    
    hdgX = magX / tmp;
    hdgY = magY / tmp;
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

