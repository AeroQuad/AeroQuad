/*
  AeroQuad v2.5.1 - December 2011
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
  
  const float getData(byte axis)
  {
    return (float)getRawData(axis) * magScale[axis] + magOffset[axis];
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
  Magnetometer_HMC5843() : Compass() {
    compassAddress = 0x1E;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Initialize HMC5843 Magnetometer
  ////////////////////////////////////////////////////////////////////////////////

  void initialize(void) {
    byte numAttempts = 0;
    bool success = false;
    delay(10);                             // Power up delay **
   
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
   
      if ( fabs(measuredMagX) > 500.0 && fabs(measuredMagX) < 1000.0 \
          && fabs(measuredMagY) > 500.0 && fabs(measuredMagY) < 1000.0 \
          && fabs(measuredMagZ) > 500.0 && fabs(measuredMagZ) < 1000.0) {
        magCalibration[XAXIS] = fabs(715.0 / measuredMagX);
        magCalibration[YAXIS] = fabs(715.0 / measuredMagY);
        magCalibration[ZAXIS] = fabs(715.0 / measuredMagZ);
    
        success = true;
      }
   
      // JI - 12/11/11 - updateRegisterI2C(compassAddress, 0x00, 0x10);  // Set 10hz update rate and normal operaiton
      // JI - 12/11/11 - delay(50);

      // JI - 12/11/11 - updateRegisterI2C(compassAddress, 0x02, 0x00); // Continuous Update mode
      // JI - 12/11/11/- delay(50);                           // Mode change delay (1/Update Rate) **
    }

    updateRegisterI2C(compassAddress, 0x02, 0x02);  // JI - 12/11/11 - Set idle mode
    delay(25);                                      // JI - 12/11/11 - Allow time for mode change
    
    updateRegisterI2C(compassAddress, 0x00, 0x10);  // JI - 12/11/11 - Set 10hz update rate and normal operation
    delay(25);                                      // JI - 12/11/11 - Allow time for configuration change
    
    updateRegisterI2C(compassAddress, 0x02, 0x01);  // JI - 12/11/11 - Perform single conversion
    delay(25);                                      // JI - 12/11/11 - Allow time for conversion to complete
    
    measure(0.0, 0.0);  // Assume 1st measurement at 0 degrees roll and 0 degrees pitch
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  // Measure HMC5843 Magnetometer
  ////////////////////////////////////////////////////////////////////////////////

  void measure(float roll, float pitch)
  {
    byte  hmc5843ready;  // JI - 12/11/11
    float magX;
    float magY;
    float tmp;
    
    sendByteI2C(compassAddress, 0x09);                  // JI - 12/11/11 - Request status register
    hmc5843ready = 0x01 & readByteI2C(compassAddress);  // JI - 12/11/11 - Mask ready status
    
    if (hmc5843ready == 0x01)                           // JI - 12/11/11 - Read only if data ready
      {
      sendByteI2C(compassAddress, 0x03);
      Wire.requestFrom(compassAddress, 6);
  
      measuredMagX =  ((Wire.read() << 8) | Wire.read()) * magCalibration[XAXIS];
      measuredMagY = -((Wire.read() << 8) | Wire.read()) * magCalibration[YAXIS];
      measuredMagZ = -((Wire.read() << 8) | Wire.read()) * magCalibration[ZAXIS];
  
      Wire.endTransmission();
      
      updateRegisterI2C(compassAddress, 0x02, 0x01);  // JI - 12/11/11 - Perform single conversion
  
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
  }
};

#if !defined(SPARKFUN_5843_BOB) && !defined(ArduCopter) && !defined(APM_OP_CHR6DM)  &&!defined(AeroQuadMega_CHR6DM)  // JI - 12/13/11
////////////////////////////////////////////////////////////////////////////////
// Magnetometer (HMC5883L)
////////////////////////////////////////////////////////////////////////////////

// See HMC58x3 datasheet for more information on these values
#define NormalOperation             0x10
// Default DataOutputRate is 10hz on HMC5843 , 15hz on HMC5883L
#define DataOutputRate_Default      ( 0x04 << 2 )
#define HMC5883L_SampleAveraging_8  ( 0x03 << 5 )

class Magnetometer_HMC5883L : public Compass {
private:
  float cosRoll;
  float sinRoll;
  float cosPitch;
  float sinPitch;

public: 
  Magnetometer_HMC5883L() : Compass() {
    compassAddress = 0x1E;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Initialize HMC5883L Magnetometer
  ////////////////////////////////////////////////////////////////////////////////

  void initialize(void) {
    byte numAttempts = 0;
    bool success = false;
    delay(10);                             // Power up delay **
   
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
   
      if ( fabs(measuredMagX) > 500.0 && fabs(measuredMagX) < 1564.4f \
          && fabs(measuredMagY) > 500.0 && fabs(measuredMagY) < 1564.4f \
          && fabs(measuredMagZ) > 500.0 && fabs(measuredMagZ) < 1477.2f) {
        magCalibration[XAXIS] = fabs(1264.4f / measuredMagX);
        magCalibration[YAXIS] = fabs(1264.4f / measuredMagY);
        magCalibration[ZAXIS] = fabs(1177.2f / measuredMagZ); 
        success = true;
      }
   
      // JI - 12/11/11 - updateRegisterI2C(compassAddress, 0x00, HMC5883L_SampleAveraging_8 | DataOutputRate_Default | NormalOperation);
      // JI - 12/11/11 - delay(50);

      // JI - 12/11/11 - updateRegisterI2C(compassAddress, 0x02, 0x00); // Continuous Update mode
      // JI - 12/11/11 - delay(50);                           // Mode change delay (1/Update Rate) **
    }

    updateRegisterI2C(compassAddress, 0x02, 0x02);  // JI - 12/11/11 - Set idle mode
    delay(25);                                      // JI - 12/11/11 - Allow time for mode change
    
    updateRegisterI2C(compassAddress, 0x00, HMC5883L_SampleAveraging_8 | DataOutputRate_Default | NormalOperation);  // JI - 12/11/11 - Configure device
    delay(25);                                      // JI - 12/11/11 - Allow time for configuration change
    
    updateRegisterI2C(compassAddress, 0x02, 0x01);  // JI - 12/11/11 - Perform single conversion
    delay(25);                                      // JI - 12/11/11 - Allow time for conversion to complete
    
    measure(0.0, 0.0);  // Assume 1st measurement at 0 degrees roll and 0 degrees pitch
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  // Measure HMC5883L Magnetometer
  ////////////////////////////////////////////////////////////////////////////////

  void measure(float roll, float pitch) 
  {
    byte  hmc5883ready;  // JI - 12/11/11
    float magX;
    float magY;
    float tmp;
    
    sendByteI2C(compassAddress, 0x09);                  // JI - 12/11/11 - Request status register
    hmc5883ready = 0x01 & readByteI2C(compassAddress);  // JI - 12/11/11 - Mask ready status
    
    if (hmc5883ready == 0x01)                           // JI - 12/11/11 - Read only if data ready
    {
      sendByteI2C(compassAddress, 0x03);
      Wire.requestFrom(compassAddress, 6);
  
      #if defined(SPARKFUN_9DOF)
        // JI - 11/24/11 - SparkFun DOF on v2p1 Shield Configuration
        // JI - 11/24/11 - 5883L X axis points aft
        // JI - 11/24/11 - 5883L Sensor Orientation 3
        measuredMagX = -((Wire.read() << 8) | Wire.read()) * magCalibration[XAXIS];  // JI - 12/01/11 - magCalibration[XAXIS], was magCalibration[YAXIS]
        measuredMagZ = -((Wire.read() << 8) | Wire.read()) * magCalibration[ZAXIS];
        measuredMagY =  ((Wire.read() << 8) | Wire.read()) * magCalibration[YAXIS];  // JI - 12/01/11 - magCalibration[YAXIS], was magCalibration[XAXIS]
      #elif defined(SPARKFUN_5883L_BOB)
        // JI - 11/24/11 - Sparkfun 5883L Breakout Board Upside Down on v2p0 shield
        // JI - 11/24/11 - 5883L is upside down, X axis points forward
        // JI - 11/24/11 - 5883L Sensor Orientation 5
        measuredMagX =  ((Wire.read() << 8) | Wire.read()) * magCalibration[XAXIS];  // JI - 12/01/11 - magCalibration[XAXIS], was magCalibration[YAXIS]
        measuredMagZ =  ((Wire.read() << 8) | Wire.read()) * magCalibration[ZAXIS];
        measuredMagY =  ((Wire.read() << 8) | Wire.read()) * magCalibration[YAXIS];  // JI - 12/01/11 - magCalibration[YAXIS], was magCalibration[XAXIS]
      #else
        !! Define 5883L Orientation !!
      #endif
      
      Wire.endTransmission();
  
      updateRegisterI2C(compassAddress, 0x02, 0x01);  // JI - 12/11/11 - Perform single conversion
      
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
  }
};
#endif  // JI - 11/26/11

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


