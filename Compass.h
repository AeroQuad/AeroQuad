/*
  AeroQuad v2.1 - October 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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
class Compass {
public: 
  int compassAddress;
  float heading, absoluteHeading, gyroStartHeading;
  float compass;
  int magRangeX;
  int magOffsetX;
  int magRangeY;
  int magOffsetY;
  int magRangeZ;
  int magOffsetZ;

  
  Compass(void) { }

  // **********************************************************************
  // The following function calls must be defined inside any new subclasses
  // **********************************************************************
  virtual void initialize(void); 
  virtual void measure(void); 
  
  // *********************************************************
  // The following functions are common between all subclasses
  // *********************************************************
  const float getData(void) {
    return compass;
  }
  
  const float getHeading(void) {
    return heading;
  }
  
  const float getAbsoluteHeading(void) {
    return absoluteHeading;
  }
  
  void setRange(byte axis, int value) {
    if (axis == XAXIS) magRangeX = value;
    if (axis == YAXIS) magRangeY = value;
    if (axis == ZAXIS) magRangeZ = value;
  }
  
  void setOffset(byte axis, int value) {
    if (axis == XAXIS) magOffsetX = value;
    if (axis == YAXIS) magOffsetY = value;
    if (axis == ZAXIS) magOffsetZ = value;
  }    
  
  const int getRange(byte axis) {
    if (axis == XAXIS) return magRangeX;
    if (axis == YAXIS) return magRangeY;
    if (axis == ZAXIS) return magRangeZ;
  }
  
  const int getOffset(byte axis) {
    if (axis == XAXIS) return magOffsetX;
    if (axis == YAXIS) return magOffsetY;
    if (axis == ZAXIS) return magOffsetZ;
  }
};

// ***********************************************************************
// ************************ HMC5843 Subclass *****************************
// ***********************************************************************
class Compass_AeroQuad_v2 : public Compass {
// This sets up the HMC5843 from Sparkfun
private:
  float cosRoll;
  float sinRoll;
  float cosPitch;
  float sinPitch;
  float magX;
  float magY;
  int measuredMagX;
  int measuredMagY;
  int measuredMagZ;
  float smoothFactor; // time constant for complementary filter
  float filter1, filter2; // coefficients for complementary filter
  float adjustedGyroHeading, previousHead;
  int gyroZero;
  float magScaleXY, magScaleYX, magScaleXZ, magScaleYZ;
  
public: 
  Compass_AeroQuad_v2() : Compass(){
    compassAddress = 0x1E;
    // smoothFactor means time in seconds less than smoothFactor, depend on gyro more
    // time greater than smoothFactor depend on magnetometer more (mags are very noisy)
    smoothFactor = 1.0; 
    filter1 = smoothFactor / (smoothFactor + G_Dt);
    filter2 = 1 - filter1;
    gyroZero = gyro.getZero(YAW);
  }

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize(void) {
    // Should do a WhoAmI to know if mag is present
    updateRegisterI2C(compassAddress, 0x01, 0x20);
    updateRegisterI2C(compassAddress, 0x02, 0x00); // continuous 10Hz mode
    measure();
    gyroStartHeading = getData();
    if (gyroStartHeading < 0) gyroStartHeading += 360;
    gyro.setStartHeading(gyroStartHeading);
    magScaleXY = magRangeX / magRangeY;
    magScaleYX = magRangeY / magRangeX;
    magScaleXZ = magRangeX / magRangeZ;
    magScaleYZ = magRangeY / magRangeZ;
  }
  
  const int getRawData(byte axis) {
    if (axis == XAXIS) return measuredMagX;
    if (axis == YAXIS) return measuredMagY;
    if (axis == ZAXIS) return measuredMagZ;
  }
  
  void measure(void) {
    sendByteI2C(compassAddress, 0x03);
    Wire.requestFrom(compassAddress, 6);
    measuredMagX = (Wire.receive() << 8) | Wire.receive();
    measuredMagY = (Wire.receive() << 8) | Wire.receive();
    measuredMagZ = (Wire.receive() << 8) | Wire.receive();
    Wire.endTransmission();
    // Heading calculation based on code written by FabQuad
    // http://aeroquad.com/showthread.php?691-Hold-your-heading-with-HMC5843-Magnetometer
    cosRoll = cos(radians(flightAngle.getData(ROLL)));
    sinRoll = sin(radians(flightAngle.getData(ROLL)));
    cosPitch = cos(radians(flightAngle.getData(PITCH)));
    sinPitch = sin(radians(flightAngle.getData(PITCH)));
    magX = (measuredMagX * magScaleYX + magOffsetX) * cosPitch + (measuredMagY * magScaleXY + magOffsetY) * sinRoll * sinPitch + (measuredMagZ * magScaleXZ + magOffsetZ) * cosRoll * sinPitch;
    magY = (measuredMagY * magScaleXY + magOffsetY) * cosRoll - (measuredMagZ * magScaleYZ + magOffsetZ) * sinRoll;
    compass = -degrees(atan2(-magY, magX));
    
    // Check if gyroZero adjusted, if it is, reset gyroHeading to compass value
    if (gyroZero != gyro.getZero(YAW)) {
      gyro.setStartHeading(heading);
      gyroZero = gyro.getZero(YAW);
    }
    
    adjustedGyroHeading = gyro.getHeading();
    // if compass is positive while gyro is negative force gyro positive past 180
    if ((compass > 90) && adjustedGyroHeading < -90) adjustedGyroHeading += 360;
    // if compass is negative whie gyro is positive force gyro negative past -180
    if ((compass < -90) && adjustedGyroHeading > 90) adjustedGyroHeading -= 360;
    
    // Complementry filter from http://chiefdelphi.com/media/papers/2010
    heading = (filter1 * adjustedGyroHeading) + (filter2 * compass);
    
    // Change from +/-180 to 0-360
    if (heading < 0) absoluteHeading = 360 + heading;
    else absoluteHeading = heading;
  }
};
