/*
  AeroQuad v2.1 - September 2010
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

// Class to define sensors that can determine altitude

// ***********************************************************************
// ************************** Altitude Class ******************************
// ***********************************************************************
class Altitude {
private: 
  
public:
  float altitude;
  Altitude (void) { 
    altitude = 0;
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
    return altitude;
  }
};

// ***********************************************************************
// ************************* BMP085 Subclass *****************************
// ***********************************************************************
class Altitude_AeroQuad_v2 : public Altitude {
// This sets up the BMP085 from Sparkfun
// From original code of Jordi Munoz and Jose Julio (DIYDrones.com)
private:
  unsigned int ac1, ac2, ac3;
  unsigned int ac4, ac5, ac6;
  unsigned int b1, b2, mb, mc, md;
  unsigned byte oss;
  long pressure, groundPressure;
  int temperature, groundTemperature;
  int groundAltitude;
  
  long readRawPressure(void) {
    updateRegisterI2C(altitudeAddress, 0xF4, 0x34+(oss<<6));
    sendByteI2C(altitudeAddress, 0xF6);
    Wire.requestFrom(altitudeAddress);
    return (Wire.available() << 16) | (Wire.available() << 8) | (Wire.available() << (8-oss));
  }

  long readRawTemperature(void) {
    updateRegisterI2C(altitudeAddress, 0xF4, 0x2E);
    sendByteI2C(altitudeAddress, 0xF6);
    return readWordI2C(altitudeAddress);
  }

public: 
  Altitude_AeroQuad_v2() : Altitude(){
    altitudeAddress = 0x77;
    oss = 3;
    pressure = 0;
    groundPressure = 0;
    temperature = 0
    groundTemperature = 0;
    groundAltitude = 0;
  }

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize(void) {
    sendByteI2C(altitudeAddress, 0xAA); // Read calibration data registers
    Wire.requestFrom(altitudeAddress, 22);
    ac1 = (Wire.receive() << 8) | Wire.receive();
    ac2 = (Wire.receive() << 8) | Wire.receive();
    ac3 = (Wire.receive() << 8) | Wire.receive();
    ac4 = (Wire.receive() << 8) | Wire.receive();
    ac5 = (Wire.receive() << 8) | Wire.receive();
    ac6 = (Wire.receive() << 8) | Wire.receive();
    b1 = (Wire.receive() << 8) | Wire.receive();
    b2 = (Wire.receive() << 8) | Wire.receive();
    mb = (Wire.receive() << 8) | Wire.receive();
    mc = (Wire.receive() << 8) | Wire.receive();
    md = (Wire.receive() << 8) | Wire.receive();
    Wire.endTransmission();
    delay(100);
    
    // measure initial ground pressure (multiple samples)
    for (int i=0; i < 100; i++) {
      measure();
      groundTemperature = smooth(temperature, groundTemperature, 0.5);
      groundPressure = smooth(pressure, groundPressure, 0.5);
      groundAltitude = smooth(altitude, groundAltitude, 0.5);
    }
  }
  
  void measure(void) {
    long x1, x2, x3, b3, b5, b6, p, tmp;
    unsigned long b4, b7;
    
    // See Datasheet page 13 for these formulas
    // Based also on Jee Labs BMP085 example code. Thanks for sharing.
    // Temperature calculations
    x1 = ((long)readRawTemperature() - ac6) * ac5 >> 15;
    x2 = ((long) mc << 11) / (x1 + md);
    b5 = x1 + x2;
    temperature = (b5 + 8) >> 4;
  
    // Pressure calculations
    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
    x2 = ac2 * b6 >> 11;
    x3 = x1 + x2;
    tmp = ac1;
    tmp = (tmp*4 + x3)<<oss;
    b3 = (tmp+2)/4;
    x1 = ac3 * b6 >> 13;
    x2 = (b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t) readRawPressure() - b3) * (50000 >> oss);
    p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
    
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    pressure = p + ((x1 + x2 + 3791) >> 4);
    
    altitude = (int)((log((double)groundPressure / (double)pressure) * ((float)groundTemperature / 10.f + 273.15f) * 29271.267f) / 10) + groundAltitude;
  }
  
  // For debug purposes
  const long getRawPressure(void) {
    return readRawPressure();
  }

  // For debug purposes
  const long getRawTemperature(void) {
    return readRawTemperature();
  }
};
