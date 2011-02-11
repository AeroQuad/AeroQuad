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

class Accelerometer 
{
private:
  int _sign[3];
  byte _rollChannel;
  byte _pitchChannel;
  byte _zAxisChannel;
  unsigned long _currentAccelTime;
  unsigned long _previousAccelTime;
  
protected:  
  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    float _accelZero[3];
  #else
    int _accelZero[3];
  #endif
  int _accelChannel[3];
  int _accelADC[3];
  int _accelData[3];  
  float _accelScaleFactor;
  float _accelOneG;
  float _smoothFactor;
  float _rawAltitude;  


  
public:  
  Accelerometer() 
  {
    _sign[ROLL] = 1;
    _sign[PITCH] = 1;
    _sign[YAW] = 1;
  }

  // ******************************************************************
  // The following function calls must be defined in any new subclasses
  // ******************************************************************
  virtual void initialize() 
  {
    this->_initialize(_rollChannel, _pitchChannel, _zAxisChannel);
  }
  virtual void measure();
  virtual void calibrate();
  virtual const int getFlightData(byte);
  virtual void calculateAltitude();

  // **************************************************************
  // The following functions are common between all Gyro subclasses
  // **************************************************************
  void _initialize(byte rollChannel, byte pitchChannel, byte zAxisChannel) 
  {
    _accelChannel[ROLL] = rollChannel;
    _accelChannel[PITCH] = pitchChannel;
    _accelChannel[ZAXIS] = zAxisChannel;
    _currentAccelTime = micros();
    _previousAccelTime = _currentAccelTime;
  }
  
  const int getRaw(byte axis) 
  {
    return _accelADC[axis] * _sign[axis];
  }
  
  const int getData(byte axis) 
  {
    return _accelData[axis] * _sign[axis];
  }
  
  const int invert(byte axis) 
  {
    _sign[axis] = -_sign[axis];
    return _sign[axis];
  }
  
  const int getZero(byte axis) 
  {
    return _accelZero[axis];
  }
  
  void setZero(byte axis, int value) 
  {
    _accelZero[axis] = value;
  }
  
  const float getScaleFactor() 
  {
    return _accelScaleFactor;
  }
  
  const float getSmoothFactor() 
  {
    return _smoothFactor;
  }
  
  void setSmoothFactor(float value) 
  {
    _smoothFactor = value;
  }
  
  const float angleRad(byte axis) 
  {
    if (axis == PITCH) 
    {
      return arctan2(_accelData[PITCH] * _sign[PITCH], sqrt((long(_accelData[ROLL]) * _accelData[ROLL]) + (long(_accelData[ZAXIS]) * _accelData[ZAXIS])));
    }
    // then it have to be the ROLL axis
    return arctan2(_accelData[ROLL] * _sign[ROLL], sqrt((long(_accelData[PITCH]) * _accelData[PITCH]) + (long(_accelData[ZAXIS]) * _accelData[ZAXIS])));
  }

  const float angleDeg(byte axis) 
  {
    return degrees(angleRad(axis));
  }
  
  void setOneG(int value) 
  {
    _accelOneG = value;
  }
  
  const int getOneG() 
  {
    return _accelOneG;
  }
  
  const int getZaxis() 
  {
    //currentAccelTime = micros();
    //zAxis = filterSmoothWithTime(getFlightData(ZAXIS), zAxis, 0.25, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    //previousAccelTime = currentAccelTime;
    //return zAxis;
    return _accelOneG - getData(ZAXIS);
  }
  
  const float getAltitude() 
  {
    return _rawAltitude;
  }
  
  const float rateG(const byte axis) 
  {
    return getData(axis) / _accelOneG;
  }
};

/******************************************************/
/************ AeroQuad v1 Accelerometer ***************/
/******************************************************/
#if defined(AeroQuad_v1) || defined(AeroQuad_v1_IDG) || defined(AeroQuadMega_v1)
class ADXL335Accelerometer : public Accelerometer 
{
public:
  ADXL335Accelerometer() : Accelerometer()
  {
    // Accelerometer Values
    // Update these variables if using a different accel
    // Output is ratiometric for ADXL 335
    // Note: Vs is not AREF voltage
    // If Vs = 3.6V, then output sensitivity is 360mV/g
    // If Vs = 2V, then it's 195 mV/g
    // Then if Vs = 3.3V, then it's 329.062 mV/g
    _accelScaleFactor = 0.000329062;
  }
  
  void initialize() 
  {
    // rollChannel = 1
    // pitchChannel = 0
    // zAxisChannel = 2
    this->_initialize(1, 0, 2);
  }
  
  void measure() 
  {
    _currentTime = micros();
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      _accelADC[axis] = analogRead(_accelChannel[axis]) - _accelZero[axis];
      _accelData[axis] = filterSmooth(_accelADC[axis], _accelData[axis], _smoothFactor);
    }
    _previousTime = _currentTime;
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }
  
  // Allows user to zero accelerometers on command
  void calibrate() 
  {
    int findZero[FINDZERO];

    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++)
      {
        findZero[i] = analogRead(_accelChannel[calAxis]);
      }
      _accelZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }
    
    // store accel value that represents 1g
    _accelOneG = _accelZero[ZAXIS];
    // replace with estimated Z axis 0g value
    _accelZero[ZAXIS] = (_accelZero[ROLL] + _accelZero[PITCH]) / 2;
  }

  void calculateAltitude() 
  {
    _currentTime = micros();
    if ((abs(getRaw(ROLL)) < 1500) && (abs(getRaw(PITCH)) < 1500)) 
    {
      _rawAltitude += (getZaxis()) * ((_currentTime - _previousTime) / 1000000.0);
    }
    _previousTime = _currentTime;
  } 
};
#endif

/******************************************************/
/********* AeroQuad Mega v2 Accelerometer *************/
/******************************************************/
#if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
class BMA180Accelerometer : public Accelerometer 
{
private:
  int accelAddress;
  
public:
  BMA180Accelerometer() : Accelerometer()
  {
    accelAddress = 0x40; // page 54 and 61 of datasheet
    // Accelerometer value if BMA180 setup for 1.0G
    // Page 27 of datasheet = 0.00013g/LSB
    _accelScaleFactor = 0.00013;
  }
  
  void initialize() 
  {
    // Check if accel is connected
    if (readWhoI2C(accelAddress) != 0x03) // page 52 of datasheet
    {
      Serial.println("Accelerometer not found!");
    }

    // Thanks to SwiftingSpeed for updates on these settings
    // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11207&viewfull=1#post11207
    updateRegisterI2C(accelAddress, 0x10, 0xB6); //reset device
    delay(10);  //sleep 10 ms after reset (page 25)

    // In datasheet, summary register map is page 21
    // Low pass filter settings is page 27
    // Range settings is page 28
    updateRegisterI2C(accelAddress, 0x0D, 0x10); //enable writing to control registers
    sendByteI2C(accelAddress, 0x20); // register bw_tcs (bits 4-7)
    byte data = readByteI2C(accelAddress); // get current register value
    updateRegisterI2C(accelAddress, 0x20, data & 0x0F); // set low pass filter to 10Hz (value = 0000xxxx)

    // From page 27 of BMA180 Datasheet
    //  1.0g = 0.13 mg/LSB
    //  1.5g = 0.19 mg/LSB
    //  2.0g = 0.25 mg/LSB
    //  3.0g = 0.38 mg/LSB
    //  4.0g = 0.50 mg/LSB
    //  8.0g = 0.99 mg/LSB
    // 16.0g = 1.98 mg/LSB
    sendByteI2C(accelAddress, 0x35); // register offset_lsb1 (bits 1-3)
    data = readByteI2C(accelAddress);
    data &= 0xF1; // +/-1.0g (value = xxxx000x) // 0xF7;(3g)  //0xF5; (2g)
    updateRegisterI2C(accelAddress, 0x35, data);
  }
  
  void measure() 
  {
    int rawData[3];

    Wire.beginTransmission(accelAddress);
    Wire.send(0x02);
    Wire.endTransmission();
    Wire.requestFrom(accelAddress, 6);
    rawData[PITCH] = (Wire.receive()| (Wire.receive() << 8)) >> 2; // last 2 bits are not part of measurement
    rawData[ROLL] = (Wire.receive()| (Wire.receive() << 8)) >> 2; // last 2 bits are not part of measurement
    rawData[ZAXIS] = (Wire.receive()| (Wire.receive() << 8)) >> 2; // last 2 bits are not part of measurement
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      _accelADC[axis] = rawData[axis] - _accelZero[axis]; // center accel data around zero
      _accelData[axis] = filterSmooth(_accelADC[axis], _accelData[axis], _smoothFactor);
    }
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis) >> 4;
  }
  
  // Allows user to zero accelerometers on command
  void calibrate() 
  {  
    int findZero[FINDZERO];
    int dataAddress;
    
    for (byte calAxis = ROLL; calAxis < ZAXIS; calAxis++) 
    {
      if (calAxis == ROLL) dataAddress = 0x04;
      if (calAxis == PITCH) dataAddress = 0x02;
      if (calAxis == ZAXIS) dataAddress = 0x06;
      for (int i=0; i<FINDZERO; i++) 
      {
        sendByteI2C(accelAddress, dataAddress);
        findZero[i] = readReverseWordI2C(accelAddress) >> 2; // last two bits are not part of measurement
        delay(1);
      }
      _accelZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }

    // replace with estimated Z axis 0g value
    _accelZero[ZAXIS] = (_accelZero[ROLL] + _accelZero[PITCH]) / 2;
    // store accel value that represents 1g
    measure();
    _accelOneG = getRaw(ZAXIS);
    //accelOneG = 8274; // mesured value at flat level with configurator
  }

  void calculateAltitude() 
  {
    _currentTime = micros();
    if ((abs(getRaw(ROLL)) < 1500) && (abs(getRaw(PITCH)) < 1500)) 
    {
      _rawAltitude += (getZaxis()) * ((_currentTime - _previousTime) / 1000000.0);
    }
    _previousTime = _currentTime;
  } 
};
#endif

/******************************************************/
/*********** ArduCopter ADC Accelerometer *************/
/******************************************************/
#ifdef ArduCopter
class ADXL335_ADCAccelerometer : public Accelerometer 
{
private:
  int _findZero[FINDZERO];
  int _rawADC;

public:
  ADXL335_ADCAccelerometer() : Accelerometer()
  {
    // ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
    // ADXL335 Sensitivity(from datasheet) => 330mV/g, 0.8mV/ADC step => 330/0.8 = 412
    // Tested value : 414
    // #define GRAVITY 414 //this equivalent to 1G in the raw data coming from the accelerometer 
    // #define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square
    _accelScaleFactor = 414.0 / 9.81;    
  }
  
  void initialize() 
  {
    // rollChannel = 5
    // pitchChannel = 4
    // zAxisChannel = 6
    this->_initialize(5, 4, 6);
  }
  
  void measure() 
  {
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      _rawADC = analogRead_ArduCopter_ADC(_accelChannel[axis]);
      if (_rawADC > 500) // Check if measurement good
      {
        _accelADC[axis] = _rawADC - _accelZero[axis];
      }
      _accelData[axis] = _accelADC[axis]; // no smoothing needed
    }
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }
  
  // Allows user to zero accelerometers on command
  void calibrate() 
  {
    for(byte calAxis = 0; calAxis < LASTAXIS; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++) 
      {
        _findZero[i] = analogRead_ArduCopter_ADC(_accelChannel[calAxis]);
        delay(5);
      }
      _accelZero[calAxis] = findMedianInt(_findZero, FINDZERO);
    }

    // store accel value that represents 1g
    _accelOneG = _accelZero[ZAXIS];
    //accelOneG = 486;    // tested value with the configurator at flat level
    // replace with estimated Z axis 0g value
    _accelZero[ZAXIS] = (_accelZero[ROLL] + _accelZero[PITCH]) / 2;
  }

  void calculateAltitude() 
  {
    _currentTime = micros();
    if ((abs(getRaw(ROLL)) < 1500) && (abs(getRaw(PITCH)) < 1500)) 
    {
      _rawAltitude += (getZaxis()) * ((_currentTime - _previousTime) / 1000000.0);
    }
    _previousTime = _currentTime;
  } 
};
#endif

/******************************************************/
/****************** Wii Accelerometer *****************/
/******************************************************/
#if defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii)
class WiiAccelerometer : public Accelerometer 
{
public:
  WiiAccelerometer() : Accelerometer()
  {
    _accelScaleFactor = 0;    
  }
  
  void measure() 
  {
    _currentTime = micros();
    // Actual measurement performed in gyro class
    // We just update the appropriate variables here
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      _accelADC[axis] = _accelZero[axis] - NWMP_acc[axis];
      _accelData[axis] = filterSmoothWithTime(_accelADC[axis], _accelData[axis], _smoothFactor, ((_currentTime - _previousTime) / 5000.0));
    }
    _previousTime = _currentTime;
  }
  
  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }
 
  // Allows user to zero accelerometers on command
  void calibrate() 
  {
    int findZero[FINDZERO];

    for(byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++) 
      {
        updateControls();
        findZero[i] = NWMP_acc[calAxis];
      }
      _accelZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }
    
    // store accel value that represents 1g
    _accelOneG = getRaw(ZAXIS);
    // replace with estimated Z axis 0g value
    _accelZero[ZAXIS] = (_accelZero[ROLL] + _accelZero[PITCH]) / 2;
  }

  void calculateAltitude() 
  {
    _currentTime = micros();
    if ((abs(getRaw(ROLL)) < 1500) && (abs(getRaw(PITCH)) < 1500)) 
    {
      _rawAltitude += (getZaxis()) * ((_currentTime - _previousTime) / 1000000.0);
    }
    _previousTime = _currentTime;
  } 
};
#endif

/******************************************************/
/****************** CHR6DM Accelerometer **************/
/******************************************************/
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
class CHR6DMAccelerometer : public Accelerometer 
{
public:
  CHR6DMAccelerometer() : Accelerometer() 
  {
    _accelScaleFactor = 0;
  }

  void initialize() 
  {
    calibrate();
  }

  void measure() 
  {
    _currentTime = micros();
    _accelADC[XAXIS] = chr6dm.data.ax - _accelZero[XAXIS];
    _accelADC[YAXIS] = chr6dm.data.ay - _accelZero[YAXIS];
    _accelADC[ZAXIS] = chr6dm.data.az - _accelOneG;

    _accelData[XAXIS] = filterSmoothWithTime(_accelADC[XAXIS], _accelData[XAXIS], _smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //to get around 1
    _accelData[YAXIS] = filterSmoothWithTime(_accelADC[YAXIS], _accelData[YAXIS], _smoothFactor, ((_currentTime - _previousTime) / 5000.0));
    _accelData[ZAXIS] = filterSmoothWithTime(_accelADC[ZAXIS], _accelData[ZAXIS], _smoothFactor, ((_currentTime - _previousTime) / 5000.0));
    _previousTime = _currentTime;
  }    

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }

  // Allows user to zero accelerometers on command
  void calibrate() 
  {
   float zeroXreads[FINDZERO];
   float zeroYreads[FINDZERO];
   float zeroZreads[FINDZERO];

    for (int i=0; i<FINDZERO; i++) 
    {
        chr6dm.requestAndReadPacket();
        zeroXreads[i] = chr6dm.data.ax;
        zeroYreads[i] = chr6dm.data.ay;
        zeroZreads[i] = chr6dm.data.az;
    }

    _accelZero[XAXIS] = findMedianFloat(zeroXreads, FINDZERO);
    _accelZero[YAXIS] = findMedianFloat(zeroYreads, FINDZERO);
    _accelZero[ZAXIS] = findMedianFloat(zeroZreads, FINDZERO);
   
    // store accel value that represents 1g
    _accelOneG = _accelZero[ZAXIS];
    // replace with estimated Z axis 0g value
    //accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
  }

  void calculateAltitude() 
  {
    _currentTime = micros();
    if ((abs(CHR_RollAngle) < 5) && (abs(CHR_PitchAngle) < 5)) 
    {
      _rawAltitude += (getZaxis()) * ((_currentTime - _previousTime) / 1000000.0);
    }
    _previousTime = _currentTime;
  } 
};
#endif

/********************************************/
/******** CHR6DM Fake Accelerometer *********/
/********************************************/
#ifdef CHR6DM_FAKE_ACCEL
class CHR6DMFakeAccelerometer : public Accelerometer 
{
public:
  float fakeAccelRoll;
  float fakeAccelPitch;
  float fakeAccelYaw;
  
  CHR6DMFakeAccelerometer() : Accelerometer() 
  {
    accelScaleFactor = 0;
  }

  void initialize() 
  {
    accelZero[ROLL] = 0;
    accelZero[PITCH] = 0;
    accelZero[ZAXIS] = 0;
    calibrate();
  }

  void measure() 
  {
    currentTime = micros();
      //read done in gyro   //TODO
    accelADC[XAXIS] = fakeAccelRoll - accelZero[XAXIS];
    accelADC[YAXIS] = fakeAccelPitch - accelZero[YAXIS];
    accelADC[ZAXIS] = fakeAccelYaw - accelOneG;

    accelData[XAXIS] = smoothWithTime(accelADC[XAXIS], accelData[XAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
    accelData[YAXIS] = smoothWithTime(accelADC[YAXIS], accelData[YAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
    accelData[ZAXIS] = smoothWithTime(accelADC[ZAXIS], accelData[ZAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
    previousTime = currentTime;
  }
  
  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }

  // Allows user to zero accelerometers on command
  void calibrate() 
  {
   float zeroXreads[FINDZERO];
   float zeroYreads[FINDZERO];
   float zeroZreads[FINDZERO];

   for (int i=0; i<FINDZERO; i++) 
   {
        chr6dm.requestAndReadPacket();
        zeroXreads[i] = 0;
        zeroYreads[i] = 0;
        zeroZreads[i] = 0;
    }

    accelZero[XAXIS] = findMedian(zeroXreads, FINDZERO);
    accelZero[YAXIS] = findMedian(zeroYreads, FINDZERO);
    accelZero[ZAXIS] = findMedian(zeroZreads, FINDZERO);

    // store accel value that represents 1g
    accelOneG = accelZero[ZAXIS];
    // replace with estimated Z axis 0g value
    //accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
  }

  void calculateAltitude() 
  {
    currentTime = micros();
    if ((abs(CHR_RollAngle) < 5) && (abs(CHR_PitchAngle) < 5))
    { 
      rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    }
    previousTime = currentTime;
  } 
};
#endif

/******************************************************/
/************* MultiPilot Accelerometer ***************/
/******************************************************/
#if defined(Multipilot) || defined(MultipilotI2C)
class MultipilotAccelerometer : public Accelerometer 
{
public:

  MultipilotAccelerometer() : Accelerometer()
  {
    // Accelerometer Values
    // Update these variables if using a different accel
    // Output is ratiometric for ADXL 335
    // Note: Vs is not AREF voltage
    // If Vs = 3.6V, then output sensitivity is 360mV/g
    // If Vs = 2V, then it's 195 mV/g
    // Then if Vs = 3.3V, then it's 329.062 mV/g
    // Accelerometer Values for LIS344ALH set fs to +- 2G
    // Vdd = 3.3 Volt
    // Zero = Vdd / 2
    // 3300 mV / 5  (+-2G ) = 660
    accelScaleFactor = 0.000660;
  }
  
  void initialize() 
  {
    // rollChannel = 6
    // pitchChannel = 7
    // zAxisChannel = 5
    this->_initialize(6, 7, 5);
  }
  
  void measure() 
  {
    currentTime = micros();
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
      accelData[axis] = filterSmoothWithTime(accelADC[axis], accelData[axis], smoothFactor, ((currentTime - previousTime) / 5000.0));
    }
    previousTime = currentTime;
  }
  
  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }
  
  // Allows user to zero accelerometers on command
  void calibrate() 
  {
    int findZero[FINDZERO];
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++)
      {
        findZero[i] = analogRead(accelChannel[calAxis]);
      }
      accelZero[calAxis] = findMedian(findZero, FINDZERO);
    }

    // store accel value that represents 1g
    accelOneG = accelZero[ZAXIS];
    // replace with estimated Z axis 0g value
    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
  }

  void calculateAltitude() 
  {
    currentTime = micros();
    if ((abs(getRaw(ROLL)) < 1500) && (abs(getRaw(PITCH)) < 1500))
    { 
      rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    }
    previousTime = currentTime;
  } 
};
#endif
