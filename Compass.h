/*
  AeroQuad v2.5 Beta 1 - July 2011
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

// This class updated by jihlein

class Compass {
public:
  float magMax[3];
  float magMin[3];
  float magCalibration[3];
  float magScale[3];
  float magOffset[3];
  float hdgX;
  float hdgY;
  int   compassAddress;
  float measuredMagX;
  float measuredMagY;
  float measuredMagZ;
  
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

enum {
  typeHMC5843,
  typeHMC5883L,
  typeUnknown,
};

// The chip may be mounted with inverted axis
//#define INVERTED_XY_COMPASS_BREAKOUT_BOARD
   
// See HMC58x3 datasheet for more information on these values
#define NormalOperation             0x10
// Default DataOutputRate is 10hz on HMC5843 , 15hz on HMC5883L
#define DataOutputRate_Default      ( 0x04 << 2 )
#define HMC5883L_SampleAveraging_8  ( 0x03 << 5 )

class Magnetometer_HMC5843 : public Compass {
private:
  float cosRoll;
  float sinRoll;
  float cosPitch;
  float sinPitch;
  byte type;
public: 
  Magnetometer_HMC5843() : Compass() {
    compassAddress = 0x1E;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Initialize AeroQuad Mega v2.0 Magnetometer
  ////////////////////////////////////////////////////////////////////////////////

  void initialize(void) {
    byte numAttempts = 0;
    bool success = false;
    float expected_xy, expected_z;
    delay(10);                             // Power up delay **
   
    // determine if we are using 5843 or 5883L
    updateRegisterI2C(compassAddress, 0x00, HMC5883L_SampleAveraging_8 | DataOutputRate_Default | NormalOperation);
    sendByteI2C(compassAddress, 0x00);
    byte base_config = readByteI2C(compassAddress);
    if ( base_config == (HMC5883L_SampleAveraging_8 | DataOutputRate_Default | NormalOperation) ) {
        // HMC5883L supports the sample averaging config
        type = typeHMC5883L;
        expected_xy = 1264.4f; // xy - Gain 2 (0x20): 1090 LSB/Ga * 1.16 Ga - see HMC5883L specs
        expected_z  = 1177.2f; // z - Gain 2 (0x20): 1090 LSB/Ga * 1.08 Ga        
    } else if ( base_config == (DataOutputRate_Default | NormalOperation) ) {
        type = typeHMC5843;
        expected_xy = expected_z = 715.0;
    } else {
        // not behaving like either supported compass type
        type = typeUnknown;
        return;
    }

    magCalibration[XAXIS] = 1.0;
    magCalibration[YAXIS] = 1.0;
    magCalibration[ZAXIS] = 1.0;
    
    while (success == false && numAttempts < 5 ) {
      
      numAttempts++;
   
      updateRegisterI2C(compassAddress, 0x00, 0x11);  // Set positive bias configuration for sensor calibraiton
      delay(50);
   
      updateRegisterI2C(compassAddress, 0x01, 0x20); // Set +/- 1G gain
      delay(10);

      updateRegisterI2C(compassAddress, 0x02, 0x01);  // Perform single conversion
      delay(10);
   
      measure(0.0, 0.0);                    // Read calibration data
      delay(10);
   
      if ( fabs(measuredMagX) > 500.0 && fabs(measuredMagX) < (expected_xy + 300) \
          && fabs(measuredMagY) > 500.0 && fabs(measuredMagY) < (expected_xy + 300) \
          && fabs(measuredMagZ) > 500.0 && fabs(measuredMagZ) < (expected_z + 300)) {
        magCalibration[XAXIS] = fabs(expected_xy / measuredMagX);
        magCalibration[YAXIS] = fabs(expected_xy / measuredMagY);
        magCalibration[ZAXIS] = fabs(expected_z / measuredMagZ);
    
        success = true;
      }
   
      updateRegisterI2C(compassAddress, 0x00, base_config);  // Set default update rate (10hz/15hz) and normal operation
      delay(50);

      updateRegisterI2C(compassAddress, 0x02, 0x00); // Continuous Update mode
      delay(50);                           // Mode change delay (1/Update Rate) **
    }

    measure(0.0, 0.0);  // Assume 1st measurement at 0 degrees roll and 0 degrees pitch
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  // Measure AeroQuad Mega v2.0 Magnetometer
  ////////////////////////////////////////////////////////////////////////////////

  void measure(float roll, float pitch) {
    float magX;
    float magY;
    float tmp;
    
    sendByteI2C(compassAddress, 0x03);
    Wire.requestFrom(compassAddress, 6);

    measuredMagX =  ((Wire.receive() << 8) | Wire.receive()) * magCalibration[XAXIS];
    if(type == typeHMC5883L) {
        // the Z registers comes before the Y registers in the HMC5883L
      measuredMagZ = -((Wire.receive() << 8) | Wire.receive()) * magCalibration[ZAXIS];
      measuredMagY = -((Wire.receive() << 8) | Wire.receive()) * magCalibration[YAXIS];
    } else {   
      measuredMagY = -((Wire.receive() << 8) | Wire.receive()) * magCalibration[YAXIS];
      measuredMagZ = -((Wire.receive() << 8) | Wire.receive()) * magCalibration[ZAXIS];
    }
    
    Wire.endTransmission();

#ifdef INVERTED_XY_COMPASS_BREAKOUT_BOARD
    measuredMagX = -measuredMagX; 
    measuredMagY = -measuredMagY;
#endif

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
    hdgY = -magY / tmp;

  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ***********************************************************************
// ************************* CHR6DM Subclass *****************************
// ***********************************************************************
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
class Compass_CHR6DM : public Compass {
public:
  Compass_CHR6DM() : Compass() {}
  void initialize(void) {}
  const int getRawData(byte) {}
  void measure(float roll, float pitch) {
    heading = chr6dm.data.yaw; //this hardly needs any filtering :)
    // Change from +/-180 to 0-360
    if (heading < 0) absoluteHeading = 360 + heading;
    else absoluteHeading = heading;
  }
};

class Compass_CHR6DM_Fake : public Compass {
public:
  Compass_CHR6DM_Fake() : Compass() {}
  void initialize(void) {}
  const int getRawData(byte) {}
  void measure(float roll, float pitch) {
    heading = 0;
    // Change from +/-180 to 0-360
    if (heading < 0) absoluteHeading = 360 + heading;
    else absoluteHeading = heading;
  }
};
#endif
