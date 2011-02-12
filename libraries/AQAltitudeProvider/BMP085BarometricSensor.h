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

#ifndef _AQ_BMP085_BAROMETRIC_SENSOR_H_
#define _AQ_BMP085_BAROMETRIC_SENSOR_H_

#include "AltitudeProvider.h"

class BMP085BarometricSensor : public AltitudeProvider 
{
// This sets up the BMP085 from Sparkfun
// Code from http://wiring.org.co/learning/libraries/bmp085.html
// Also made bug fixes based on BMP085 library from Jordi Munoz and Jose Julio
private:
  byte _select;
  byte _pressureCount;
  byte _overSamplingSetting;
  int _ac1;
  int _ac2;
  int _ac3;
  int _b1;
  int _b2;
  int _mc;
  int _md;
  int _altitudeAddress;
  unsigned int _ac4;
  unsigned int _ac5;
  unsigned int _ac6;
  long _pressure;
  long _temperature;
  long _rawPressure;
  long _rawTemperature;
  float _pressureFactor;
  
  void requestRawPressure() 
  {
    updateRegisterI2C(_altitudeAddress, 0xF4, 0x34+(_overSamplingSetting<<6));
  }
  
  long readRawPressure() 
  {
    unsigned char msb, lsb, xlsb;
    sendByteI2C(_altitudeAddress, 0xF6);
    Wire.requestFrom(_altitudeAddress, 3); // request three bytes
    while(!Wire.available()); // wait until data available
    msb = Wire.receive();
    while(!Wire.available()); // wait until data available
    lsb = Wire.receive();
    while(!Wire.available()); // wait until data available
    xlsb = Wire.receive();
    return (((long)msb<<16) | ((long)lsb<<8) | ((long)xlsb)) >>(8-_overSamplingSetting);
  }

  void requestRawTemperature() 
  {
    updateRegisterI2C(_altitudeAddress, 0xF4, 0x2E);
  }
  
  unsigned int readRawTemperature() 
  {
    sendByteI2C(_altitudeAddress, 0xF6);
    return readWordWaitI2C(_altitudeAddress);
  }

public: 
  BMP085BarometricSensor() : AltitudeProvider()
  {
    _altitudeAddress = 0x77;
    // oversampling setting
    // 0 = ultra low power
    // 1 = standard
    // 2 = high
    // 3 = ultra high resolution
    _overSamplingSetting = 3;
    _pressure = 0;
    _groundPressure = 0;
    _temperature = 0;
    _groundTemperature = 0;
    _groundAltitude = 0;
    _pressureFactor = 1/5.255;
  }

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize() 
  {
//    float verifyGroundAltitude;
    
    sendByteI2C(_altitudeAddress, 0xAA);
    _ac1 = readWordWaitI2C(_altitudeAddress);
    sendByteI2C(_altitudeAddress, 0xAC);
    _ac2 = readWordWaitI2C(_altitudeAddress);
    sendByteI2C(_altitudeAddress, 0xAE);
    _ac3 = readWordWaitI2C(_altitudeAddress);
    sendByteI2C(_altitudeAddress, 0xB0);
    _ac4 = readWordWaitI2C(_altitudeAddress);
    sendByteI2C(_altitudeAddress, 0xB2);
    _ac5 = readWordWaitI2C(_altitudeAddress);
    sendByteI2C(_altitudeAddress, 0xB4);
    _ac6 = readWordWaitI2C(_altitudeAddress);
    sendByteI2C(_altitudeAddress, 0xB6);
    _b1 = readWordWaitI2C(_altitudeAddress);
    sendByteI2C(_altitudeAddress, 0xB8);
    _b2 = readWordWaitI2C(_altitudeAddress);
    sendByteI2C(_altitudeAddress, 0xBA);
    int mb = readWordWaitI2C(_altitudeAddress);
    sendByteI2C(_altitudeAddress, 0xBC);
    _mc = readWordWaitI2C(_altitudeAddress);
    sendByteI2C(_altitudeAddress, 0xBE);
    _md = readWordWaitI2C(_altitudeAddress);
    requestRawTemperature(); // setup up next measure() for temperature
    _select = TEMPERATURE;
    _pressureCount = 0;
    measure();
    delay(5); // delay for temperature
    measure();
    delay(26); // delay for pressure
    measureGround();
    // check if measured ground altitude is valid
    while (abs(getRawData() - getGroundAltitude()) > 10) 
    {
      delay(26);
      measureGround();
    }
    setStartAltitude(getGroundAltitude());
  }
  
  void measure() 
  {
    long x1;
    long x2;
    long x3;
    long b3; 
    long b5; 
    long b6; 
    long p;
    unsigned long b4;
    unsigned long b7;
    int32_t tmp;

    // switch between pressure and tempature measurements
    // each loop, since it's slow to measure pressure
    if (_select == PRESSURE) 
    {
      _rawPressure = readRawPressure();
      if (_pressureCount == 1) 
      {
        requestRawTemperature();
        _pressureCount = 0;
       _select = TEMPERATURE;
      }
      else
      {
        requestRawPressure();
      }
      _pressureCount++;
    }
    else 
    { // _select must equal TEMPERATURE
      _rawTemperature = (long)readRawTemperature();
      requestRawPressure();
      _select = PRESSURE;
    }
    
    //calculate true temperature
    x1 = ((long) _rawTemperature - _ac6) * _ac5 >> 15;
    x2 = ((long) _mc << 11) / (x1 + _md);
    b5 = x1 + x2;
    _temperature = ((b5 + 8) >> 4);
  
    //calculate true pressure
    b6 = b5 - 4000;
    x1 = (_b2 * (b6 * b6 >> 12)) >> 11; 
    x2 = _ac2 * b6 >> 11;
    x3 = x1 + x2;
    tmp = _ac1;
    tmp = (tmp*4 + x3)<<_overSamplingSetting;
    b3 = (tmp+2)/4;
    x1 = _ac3 * b6 >> 13;
    x2 = (_b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (_ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t) _rawPressure - b3) * (50000 >> _overSamplingSetting);
    p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
    
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    _pressure = (p + ((x1 + x2 + 3791) >> 4));
    
    _rawAltitude = 44330 * (1 - pow(_pressure/101325.0, _pressureFactor)); // returns absolute altitude in meters
    //rawAltitude = (101325.0-pressure)/4096*346;
    //accel.calculateAltitude(); //cumulates onto rawAltitude from fast filtered accel Z reads
    _currentTime = micros();
    _altitude = filterSmooth(_rawAltitude, _altitude, _smoothFactor);
    _previousTime = _currentTime;
  }
};

#endif