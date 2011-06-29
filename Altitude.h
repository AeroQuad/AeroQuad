/*
  AeroQuad v2.4.2 - June 2011
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
// ************************** Altitude Class *****************************
// ***********************************************************************

class Altitude {
public:
  double altitude, rawAltitude;
  float groundTemperature; // remove later
  float groundPressure; // remove later
  float groundAltitude;
  float smoothFactor;
  
  Altitude (void) { 
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

// ***********************************************************************
// ************************* BMP085 Subclass *****************************
// ***********************************************************************
class Altitude_AeroQuad_v2 : public Altitude {
// This sets up the BMP085 from Sparkfun
// Code from http://wiring.org.co/learning/libraries/bmp085.html
// Also made bug fixes based on BMP085 library from Jordi Munoz and Jose Julio
private:
  byte overSamplingSetting;
  int ac1, ac2, ac3;
  unsigned int ac4, ac5, ac6;
  int b1, b2, mb, mc, md;
  long pressure;
  long temperature;
  int altitudeAddress;
  long rawPressure, rawTemperature;
  byte select, pressureCount;
  float pressureFactor;
  
  void requestRawPressure(void) {
    updateRegisterI2C(altitudeAddress, 0xF4, 0x34+(overSamplingSetting<<6));
  }
  
  long readRawPressure(void) {
    unsigned char msb, lsb, xlsb;
    sendByteI2C(altitudeAddress, 0xF6);
    Wire.requestFrom(altitudeAddress, 3); // request three bytes
    while(!Wire.available()); // wait until data available
    msb = Wire.receive();
    while(!Wire.available()); // wait until data available
    lsb = Wire.receive();
    while(!Wire.available()); // wait until data available
    xlsb = Wire.receive();
    return (((long)msb<<16) | ((long)lsb<<8) | ((long)xlsb)) >>(8-overSamplingSetting);
  }

  void requestRawTemperature(void) {
    updateRegisterI2C(altitudeAddress, 0xF4, 0x2E);
  }
  
  unsigned int readRawTemperature(void) {
    sendByteI2C(altitudeAddress, 0xF6);
    return readWordWaitI2C(altitudeAddress);
  }

public: 
  Altitude_AeroQuad_v2() : Altitude(){
    altitudeAddress = 0x77;
    // oversampling setting
    // 0 = ultra low power
    // 1 = standard
    // 2 = high
    // 3 = ultra high resolution
    overSamplingSetting = 3;
    pressure = 0;
    groundPressure = 0;
    temperature = 0;
    groundTemperature = 0;
    groundAltitude = 0;
    pressureFactor = 1/5.255;
  }

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize(void) {
//    float verifyGroundAltitude;
    
    sendByteI2C(altitudeAddress, 0xAA);
    ac1 = readWordWaitI2C(altitudeAddress);
    sendByteI2C(altitudeAddress, 0xAC);
    ac2 = readWordWaitI2C(altitudeAddress);
    sendByteI2C(altitudeAddress, 0xAE);
    ac3 = readWordWaitI2C(altitudeAddress);
    sendByteI2C(altitudeAddress, 0xB0);
    ac4 = readWordWaitI2C(altitudeAddress);
    sendByteI2C(altitudeAddress, 0xB2);
    ac5 = readWordWaitI2C(altitudeAddress);
    sendByteI2C(altitudeAddress, 0xB4);
    ac6 = readWordWaitI2C(altitudeAddress);
    sendByteI2C(altitudeAddress, 0xB6);
    b1 = readWordWaitI2C(altitudeAddress);
    sendByteI2C(altitudeAddress, 0xB8);
    b2 = readWordWaitI2C(altitudeAddress);
    sendByteI2C(altitudeAddress, 0xBA);
    mb = readWordWaitI2C(altitudeAddress);
    sendByteI2C(altitudeAddress, 0xBC);
    mc = readWordWaitI2C(altitudeAddress);
    sendByteI2C(altitudeAddress, 0xBE);
    md = readWordWaitI2C(altitudeAddress);
    requestRawTemperature(); // setup up next measure() for temperature
    select = TEMPERATURE;
    pressureCount = 0;
    measure();
    delay(5); // delay for temperature
    measure();
    delay(26); // delay for pressure
    measureGround();
    // check if measured ground altitude is valid
    while (abs(getRawData() - getGroundAltitude()) > 10) {
      delay(26);
      measureGround();
    }
    setStartAltitude(getGroundAltitude());
  }
  
  void measure(void) {
    long x1, x2, x3, b3, b5, b6, p;
    unsigned long b4, b7;
    int32_t tmp;

    // switch between pressure and tempature measurements
    // each loop, since it's slow to measure pressure
    if (select == PRESSURE) {
      rawPressure = readRawPressure();
      if (pressureCount == 4) {
        requestRawTemperature();
        pressureCount = 0;
       select = TEMPERATURE;
      }
      else
        requestRawPressure();
      pressureCount++;
    }
    else { // select must equal TEMPERATURE
      rawTemperature = (long)readRawTemperature();
      requestRawPressure();
      select = PRESSURE;
    }
    
    //calculate true temperature
    x1 = ((long)rawTemperature - ac6) * ac5 >> 15;
    x2 = ((long) mc << 11) / (x1 + md);
    b5 = x1 + x2;
    temperature = ((b5 + 8) >> 4);
  
    //calculate true pressure
    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
    x2 = ac2 * b6 >> 11;
    x3 = x1 + x2;
 
    // Real Bosch formula - b3 = ((((int32_t)ac1 * 4 + x3) << overSamplingSetting) + 2) >> 2;
    // The version below is the same, but takes less program space
    tmp = ac1;
    tmp = (tmp * 4 + x3) << overSamplingSetting;
    b3 = (tmp + 2) >> 2;
 
    x1 = ac3 * b6 >> 13;
    x2 = (b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t) rawPressure - b3) * (50000 >> overSamplingSetting);
    p = b7 < 0x80000000 ? (b7 << 1) / b4 : (b7 / b4) << 1;
    
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    pressure = (p + ((x1 + x2 + 3791) >> 4));
    
    rawAltitude = 44330 * (1 - pow(pressure/101325.0, pressureFactor)); // returns absolute altitude in meters
    //rawAltitude = (101325.0-pressure)/4096*346;
    altitude = filterSmooth(rawAltitude, altitude, smoothFactor);
  }
};


