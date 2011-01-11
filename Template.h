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

// User this as a template for new classes or subclasses

// ***********************************************************************
// ************************** Example Class ******************************
// ***********************************************************************
class exampleClass {
public: 
  int exampleVariable;
  float exampleData[3];
  exampleClass(void) { 
    // this is the constructor of the object and must have the same name 
    // can be used to initialize any of the variables declared above 
  }

  // **********************************************************************
  // The following function calls must be defined inside any new subclasses
  // **********************************************************************
  virtual void initialize(void); 
  virtual void exampleFunction(int); 
  virtual const int getExampleData(byte);
  
  // *********************************************************
  // The following functions are common between all subclasses
  // *********************************************************
  void examplePublicFunction(byte axis, int value) {
    // insert common code here 
  }
  const int getPublicData(byte axis) {
    return exampleData[axis];
  }
};

// ***********************************************************************
// ************************ Example Subclass *****************************
// ***********************************************************************
class exampleSubClass : public exampleClass { 
private:
  int exampleArray[3]; // only for use inside this subclass
  int examplePrivateData; // only for use inside this subclass
  void examplePrivateFunction(int functionVariable) {
    // it's possible to declare functions just for this subclass 
  }
  
public: 
  exampleSubClass() : exampleClass(){
    // this is the constructor of the object and must have the same name
    // can be used to initialize any of the variables declared above
  }

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize(void) {
    // insert code here 
  }
  void exampleFunction(int someVariable) {
    // insert code here 
    examplePrivateFunction(someVariable); 
  }
  const int getExampleData(byte axis) { 
    // insert code here 
    return exampleArray[axis]; 
  } 
};

// Example implementation of a class and subclass for a magnetometer
// ***********************************************************************
// ************************** Compass Class ******************************
// ***********************************************************************
class CompassExample {
private: // not found in the example above, but it's possible to declare private variables only seen in the main class
  float cosRoll;
  float sinRoll;
  float cosPitch;
  float sinPitch;
  float magX;
  float magY;
  
public: 
  int compassAddress;
  float heading;
  int measuredMagX;
  int measuredMagY;
  int measuredMagZ;
  
  CompassExample(void) { 
    // this is the constructor of the object and must have the same name 
    // can be used to initialize any of the variables declared above 
  }

  // **********************************************************************
  // The following function calls must be defined inside any new subclasses
  // **********************************************************************
  virtual void initialize(void); 
  virtual void measure(void); 
  
  // *********************************************************
  // The following functions are common between all subclasses
  // *********************************************************
  const float getHeading(void) {
    // Heading calculation based on code written by FabQuad
    // http://aeroquad.com/showthread.php?691-Hold-your-heading-with-HMC5843-Magnetometer
    cosRoll = cos(radians(flightAngle.getData(ROLL)));
    sinRoll = sin(radians(flightAngle.getData(ROLL)));
    cosPitch = cos(radians(flightAngle.getData(PITCH)));
    sinPitch = sin(radians(flightAngle.getData(PITCH)));
    magX = measuredMagX * cosPitch + measuredMagY * sinRoll * sinPitch + measuredMagZ * cosRoll * sinPitch;
    magY = measuredMagY * cosRoll - measuredMagZ * sinRoll;
    return degrees(atan2(-magY, magX));
  }
};

// ***********************************************************************
// ************************ Example Subclass *****************************
// ***********************************************************************
class Compass_AeroQuad_v2 : public CompassExample {
// This sets up the HMC5843 from Sparkfun
public: 
  Compass_AeroQuad_v2() : CompassExample(){
    compassAddress = 0x1E;
  }

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize(void) {
    updateRegisterI2C(compassAddress, 0x02, 0x00); // continuous 10Hz mode
    delay(100);
  }
  
  void measure(void) {
    sendByteI2C(compassAddress, 0x03);
    Wire.requestFrom(compassAddress, 6);
    measuredMagX = (Wire.receive() << 8) | Wire.receive();
    measuredMagY = (Wire.receive() << 8) | Wire.receive();
    measuredMagZ = (Wire.receive() << 8) | Wire.receive();
    Wire.endTransmission();
  }
};


