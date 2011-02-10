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

class Accel 
{
private:
  int sign[3];
  float zAxis;
  byte rollChannel, pitchChannel, zAxisChannel;
  unsigned long currentAccelTime, previousAccelTime;
  
protected:  
  int accelChannel[3];
  float accelScaleFactor;
  float smoothFactor;
  int accelADC[3];
  int accelData[3];
  float accelOneG;
  float rawAltitude;
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
  float accelZero[3];
#else
  int accelZero[3];
#endif

public:  

  Accel(void) 
  {
    sign[0] = 1;
    sign[1] = 1;
    sign[2] = 1;
    zAxis = 0;
  }

  // ******************************************************************
  // The following function calls must be defined in any new subclasses
  // ******************************************************************
  virtual void initialize(void) 
  {
    this->_initialize(rollChannel, pitchChannel, zAxisChannel);
  }
  virtual void measure(void);
  virtual void calibrate(void);
  virtual const int getFlightData(byte);
  virtual void calculateAltitude(void);

  // **************************************************************
  // The following functions are common between all Gyro subclasses
  // **************************************************************
  void _initialize(byte rollChannel, byte pitchChannel, byte zAxisChannel) 
  {
    accelChannel[0] = rollChannel;
    accelChannel[1] = pitchChannel;
    accelChannel[2] = zAxisChannel;
    currentAccelTime = micros();
    previousAccelTime = currentAccelTime;
  }
  
  const int getRaw(byte axis) 
  {
    return accelADC[axis] * sign[axis];
  }
  
  const int getData(byte axis) 
  {
    return accelData[axis] * sign[axis];
  }
  
  const int invert(byte axis) 
  {
    sign[axis] = -sign[axis];
    return sign[axis];
  }
  
  const int getZero(byte axis) 
  {
    return accelZero[axis];
  }
  
  void setZero(byte axis, int value) 
  {
    accelZero[axis] = value;
  }
  
  const float getScaleFactor(void) 
  {
    return accelScaleFactor;
  }
  
  const float getSmoothFactor() 
  {
    return smoothFactor;
  }
  
  void setSmoothFactor(float value) 
  {
    smoothFactor = value;
  }
  
  const float angleRad(byte axis) 
  {
    if (axis == 1) return arctan2(accelData[1] * sign[1], sqrt((long(accelData[0]) * accelData[0]) + (long(accelData[2]) * accelData[2])));
    if (axis == 0) return arctan2(accelData[0] * sign[0], sqrt((long(accelData[1]) * accelData[1]) + (long(accelData[2]) * accelData[2])));
  }

  const float angleDeg(byte axis) 
  {
    return degrees(angleRad(axis));
  }
  
    void setOneG(int value) 
    {
    accelOneG = value;
  }
  
  const int getOneG(void) 
  {
    return accelOneG;
  }
  
  const int getZaxis() 
  {
    //currentAccelTime = micros();
    //zAxis = filterSmoothWithTime(getFlightData(2), zAxis, 0.25, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    //previousAccelTime = currentAccelTime;
    //return zAxis;
    return accelOneG - getData(2);
  }
  
  const float getAltitude(void) 
  {
    return rawAltitude;
  }
  
  const float rateG(const byte axis) 
  {
    return getData(axis) / accelOneG;
  }
};

/******************************************************/
/************ AeroQuad v1 Accelerometer ***************/
/******************************************************/
#if defined(AeroQuad_v1) || defined(AeroQuad_v1_IDG) || defined(AeroQuadMega_v1)
class Accel_AeroQuad_v1 : public Accel 
{
public:
  Accel_AeroQuad_v1() : Accel()
  {
    // Accelerometer Values
    // Update these variables if using a different accel
    // Output is ratiometric for ADXL 335
    // Note: Vs is not AREF voltage
    // If Vs = 3.6V, then output sensitivity is 360mV/g
    // If Vs = 2V, then it's 195 mV/g
    // Then if Vs = 3.3V, then it's 329.062 mV/g
    accelScaleFactor = 0.000329062;
  }
  
  void initialize(void) 
  {
    // rollChannel = 1
    // pitchChannel = 0
    // zAxisChannel = 2
    this->_initialize(1, 0, 2);
  }
  
  void measure(void) 
  {
    _currentTime = micros();
    for (byte axis = 0; axis < 3; axis++) 
    {
      accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
      accelData[axis] = filterSmooth(accelADC[axis], accelData[axis], smoothFactor);
    }
    _previousTime = _currentTime;
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }
  
  // Allows user to zero accelerometers on command
  void calibrate(void) 
  {
    int findZero[FINDZERO];

    for (byte calAxis = 0; calAxis < 3; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++)
      {
        findZero[i] = analogRead(accelChannel[calAxis]);
      }
      accelZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }
    
    // store accel value that represents 1g
    accelOneG = accelZero[2];
    // replace with estimated Z axis 0g value
    accelZero[2] = (accelZero[0] + accelZero[1]) / 2;
  }

  void calculateAltitude() 
  {
    _currentTime = micros();
    if ((abs(getRaw(0)) < 1500) && (abs(getRaw(1)) < 1500)) 
    {
      rawAltitude += (getZaxis()) * ((_currentTime - _previousTime) / 1000000.0);
    }
    _previousTime = _currentTime;
  } 
};
#endif

/******************************************************/
/********* AeroQuad Mega v2 Accelerometer *************/
/******************************************************/
#if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
class Accel_AeroQuadMega_v2 : public Accel {
private:
  int accelAddress;
  
public:
  Accel_AeroQuadMega_v2() : Accel()
  {
    accelAddress = 0x40; // page 54 and 61 of datasheet
    // Accelerometer value if BMA180 setup for 1.0G
    // Page 27 of datasheet = 0.00013g/LSB
    accelScaleFactor = 0.00013;
  }
  
  void initialize(void) 
  {
    byte data;
    
    // Check if accel is connected
    if (readWhoI2C(accelAddress) != 0x03) // page 52 of datasheet
      Serial.println("Accelerometer not found!");

    // Thanks to SwiftingSpeed for updates on these settings
    // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11207&viewfull=1#post11207
    updateRegisterI2C(accelAddress, 0x10, 0xB6); //reset device
    delay(10);  //sleep 10 ms after reset (page 25)

    // In datasheet, summary register map is page 21
    // Low pass filter settings is page 27
    // Range settings is page 28
    updateRegisterI2C(accelAddress, 0x0D, 0x10); //enable writing to control registers
    sendByteI2C(accelAddress, 0x20); // register bw_tcs (bits 4-7)
    data = readByteI2C(accelAddress); // get current register value
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
  
  void measure(void) 
  {
    int rawData[3];

    Wire.beginTransmission(accelAddress);
    Wire.send(0x02);
    Wire.endTransmission();
    Wire.requestFrom(accelAddress, 6);
    rawData[1] = (Wire.receive()| (Wire.receive() << 8)) >> 2; // last 2 bits are not part of measurement
    rawData[0] = (Wire.receive()| (Wire.receive() << 8)) >> 2; // last 2 bits are not part of measurement
    rawData[2] = (Wire.receive()| (Wire.receive() << 8)) >> 2; // last 2 bits are not part of measurement
    for (byte axis = 0; axis < 3; axis++) 
    {
      accelADC[axis] = rawData[axis] - accelZero[axis]; // center accel data around zero
      accelData[axis] = filterSmooth(accelADC[axis], accelData[axis], smoothFactor);
    }
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis) >> 4;
  }
  
  // Allows user to zero accelerometers on command
  void calibrate(void) 
  {  
    int findZero[FINDZERO];
    int dataAddress;
    
    for (byte calAxis = 0; calAxis < 2; calAxis++) 
    {
      if (calAxis == 0) dataAddress = 0x04;
      if (calAxis == 1) dataAddress = 0x02;
      if (calAxis == 2) dataAddress = 0x06;
      for (int i=0; i<FINDZERO; i++) 
      {
        sendByteI2C(accelAddress, dataAddress);
        findZero[i] = readReverseWordI2C(accelAddress) >> 2; // last two bits are not part of measurement
        delay(1);
      }
      accelZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }

    // replace with estimated Z axis 0g value
    accelZero[2] = (accelZero[0] + accelZero[1]) / 2;
    // store accel value that represents 1g
    measure();
    accelOneG = getRaw(2);
    //accelOneG = 8274; // mesured value at flat level with configurator
  }

  void calculateAltitude() 
  {
    _currentTime = micros();
    if ((abs(getRaw(0)) < 1500) && (abs(getRaw(1)) < 1500)) 
    {
      rawAltitude += (getZaxis()) * ((_currentTime - _previousTime) / 1000000.0);
    }
    _previousTime = _currentTime;
  } 
};
#endif

/******************************************************/
/*********** ArduCopter ADC Accelerometer *************/
/******************************************************/
#ifdef ArduCopter
class Accel_ArduCopter : public Accel {
private:
  int findZero[FINDZERO];
  int rawADC;

public:
  Accel_ArduCopter() : Accel()
  {
    // ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
    // ADXL335 Sensitivity(from datasheet) => 330mV/g, 0.8mV/ADC step => 330/0.8 = 412
    // Tested value : 414
    // #define GRAVITY 414 //this equivalent to 1G in the raw data coming from the accelerometer 
    // #define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square
    accelScaleFactor = 414.0 / 9.81;    
  }
  
  void initialize(void) 
  {
    // rollChannel = 5
    // pitchChannel = 4
    // zAxisChannel = 6
    this->_initialize(5, 4, 6);
  }
  
  void measure(void) {
    for (byte axis = 0; axis < 3; axis++)
    {
      rawADC = analogRead_ArduCopter_ADC(accelChannel[axis]);
      if (rawADC > 500) // Check if measurement good
      {
        accelADC[axis] = rawADC - accelZero[axis];
      }
      accelData[axis] = accelADC[axis]; // no smoothing needed
    }
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }
  
  // Allows user to zero accelerometers on command
  void calibrate(void) 
  {
    for(byte calAxis = 0; calAxis < 3; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++) 
      {
        findZero[i] = analogRead_ArduCopter_ADC(accelChannel[calAxis]);
        delay(2);
      }
      accelZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }

    // store accel value that represents 1g
    accelOneG = accelZero[2];
    //accelOneG = 486;    // tested value with the configurator at flat level
    // replace with estimated Z axis 0g value
    accelZero[2] = (accelZero[0] + accelZero[1]) / 2;
  }

  void calculateAltitude() 
  {
    _currentTime = micros();
    if ((abs(getRaw(0)) < 1500) && (abs(getRaw(1)) < 1500)) 
    {
      rawAltitude += (getZaxis()) * ((_currentTime - _previousTime) / 1000000.0);
    }
    _previousTime = _currentTime;
  } 
};
#endif

/******************************************************/
/****************** Wii Accelerometer *****************/
/******************************************************/
#if defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii)
class Accel_Wii : public Accel 
{
public:
  Accel_Wii() : Accel()
  {
    accelScaleFactor = 0;    
  }
  
  void measure(void) 
  {
    _currentTime = micros();
    // Actual measurement performed in gyro class
    // We just update the appropriate variables here
    for (byte axis = 0; axis < 3; axis++) 
    {
      accelADC[axis] = accelZero[axis] - NWMP_acc[axis];
      accelData[axis] = filterSmoothWithTime(accelADC[axis], accelData[axis], smoothFactor, ((_currentTime - _previousTime) / 5000.0));
    }
    _previousTime = _currentTime;
  }
  
  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }
 
  // Allows user to zero accelerometers on command
  void calibrate(void) 
  {
    int findZero[FINDZERO];

    for(byte calAxis = 0; calAxis < 3; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++)
      {
        updateControls();
        findZero[i] = NWMP_acc[calAxis];
      }
      accelZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }
    
    // store accel value that represents 1g
    accelOneG = getRaw(2);
    // replace with estimated Z axis 0g value
    accelZero[2] = (accelZero[0] + accelZero[1]) / 2;
  }

  void calculateAltitude() 
  {
    _currentTime = micros();
    if ((abs(getRaw(0)) < 1500) && (abs(getRaw(1)) < 1500))
    { 
      rawAltitude += (getZaxis()) * ((_currentTime - _previousTime) / 1000000.0);
    }
    _previousTime = _currentTime;
  } 
};
#endif

/******************************************************/
/****************** CHR6DM Accelerometer **************/
/******************************************************/
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
class Accel_CHR6DM : public Accel 
{
public:
  Accel_CHR6DM() : Accel() 
  {
    accelScaleFactor = 0;
  }

  void initialize(void) 
  {
    calibrate();
  }

  void measure(void) 
  {
    _currentTime = micros();
    accelADC[0] = chr6dm.data.ax - accelZero[0];
    accelADC[1] = chr6dm.data.ay - accelZero[1];
    accelADC[2] = chr6dm.data.az - accelOneG;

    accelData[0] = filterSmoothWithTime(accelADC[0], accelData[0], smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //to get around 1
    accelData[1] = filterSmoothWithTime(accelADC[1], accelData[1], smoothFactor, ((_currentTime - _previousTime) / 5000.0));
    accelData[2] = filterSmoothWithTime(accelADC[2], accelData[2], smoothFactor, ((_currentTime - _previousTime) / 5000.0));
    _previousTime = _currentTime;
  }    

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }

  // Allows user to zero accelerometers on command
  void calibrate(void)
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


    accelZero[0] = findMedianFloat(zeroXreads, FINDZERO);
    accelZero[1] = findMedianFloat(zeroYreads, FINDZERO);
    accelZero[2] = findMedianFloat(zeroZreads, FINDZERO);
   
    // store accel value that represents 1g
    accelOneG = accelZero[2];
    // replace with estimated Z axis 0g value
    //accelZero[2] = (accelZero[0] + accelZero[1]) / 2;
  }

  void calculateAltitude() 
  {
    _currentTime = micros();
    if ((abs(CHR_RollAngle) < 5) && (abs(CHR_PitchAngle) < 5)) 
    {
      rawAltitude += (getZaxis()) * ((_currentTime - _previousTime) / 1000000.0);
    }
    _previousTime = _currentTime;
  } 
};
#endif

/********************************************/
/******** CHR6DM Fake Accelerometer *********/
/********************************************/
#ifdef CHR6DM_FAKE_ACCEL
class Accel_CHR6DM_Fake : public Accel 
{
public:
  float fakeAccelRoll;
  float fakeAccelPitch;
  float fakeAccelYaw;
  Accel_CHR6DM_Fake() : Accel() 
  {
    accelScaleFactor = 0;
  }

  void initialize(void) 
  {
    accelZero[0] = 0;
    accelZero[1] = 0;
    accelZero[2] = 0;

    calibrate();
  }

  void measure(void) 
  {
    currentTime = micros();
    //read done in gyro   //TODO
    accelADC[0] = fakeAccelRoll - accelZero[0];
    accelADC[1] = fakeAccelPitch - accelZero[1];
    accelADC[2] = fakeAccelYaw - accelOneG;

    accelData[0] = smoothWithTime(accelADC[0], accelData[0], smoothFactor, ((currentTime - previousTime) / 5000.0));
    accelData[1] = smoothWithTime(accelADC[1], accelData[1], smoothFactor, ((currentTime - previousTime) / 5000.0));
    accelData[2] = smoothWithTime(accelADC[2], accelData[2], smoothFactor, ((currentTime - previousTime) / 5000.0));
    previousTime = currentTime;
  }
  
  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }

  // Allows user to zero accelerometers on command
  void calibrate(void) 
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

    accelZero[0] = findMedian(zeroXreads, FINDZERO);
    accelZero[1] = findMedian(zeroYreads, FINDZERO);
    accelZero[2] = findMedian(zeroZreads, FINDZERO);

    // store accel value that represents 1g
    accelOneG = accelZero[2];
    // replace with estimated Z axis 0g value
    //accelZero[2] = (accelZero[0] + accelZero[1]) / 2;
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
class Accel_Multipilot : public Accel 
{
public:
  Accel_Multipilot() : Accel()
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
  
  void initialize(void)
  {
    // rollChannel = 6
    // pitchChannel = 7
    // zAxisChannel = 5
    this->_initialize(6, 7, 5);
  }
  
  void measure(void) 
  {
    currentTime = micros();
    for (byte axis = 0; axis < 3; axis++)
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
  void calibrate(void) 
  {
    int findZero[FINDZERO];
    for (byte calAxis = 0; calAxis < 3; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++)
      {
        findZero[i] = analogRead(accelChannel[calAxis]);
      }
      accelZero[calAxis] = findMedian(findZero, FINDZERO);
    }

    // store accel value that represents 1g
    accelOneG = accelZero[2];
    // replace with estimated Z axis 0g value
    accelZero[2] = (accelZero[0] + accelZero[1]) / 2;
  }

  void calculateAltitude() 
  {
    currentTime = micros();
    if ((abs(getRaw(0)) < 1500) && (abs(getRaw(1)) < 1500))
    { 
      rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    }
    previousTime = currentTime;
  } 
};
#endif
