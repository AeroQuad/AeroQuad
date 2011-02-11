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

class Gyro 
{
private:
  byte _rollChannel; 
  byte _pitchChannel; 
  byte _yawChannel;
  int _sign[3];
  float _gyroHeading;

protected:    
  float _gyroFullScaleOutput;
  float _gyroScaleFactor;
  int _lastReceiverYaw;
  long _yawAge;
  int _positiveGyroYawCount;
  int _negativeGyroYawCount;
  int _zeroGyroYawCount;
  int _gyroADC[3];  
  int _gyroData[3];
  float _smoothFactor;  
  float _rawHeading;
  int _gyroChannel[3];

  // ************ Correct for gyro drift by FabQuad **************  
  // ************ http://aeroquad.com/entry.php?4-  **************     
  int _receiverYaw;
  unsigned long _currentTime; 
  unsigned long _previousTime;
  
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
  float _gyroZero[3];
#else
  int _gyroZero[3];
#endif
    
public:    
  Gyro(void)
  {
    _sign[0] = 1;
    _sign[1] = 1;
    _sign[2] = -1;
  }
  
  // The following function calls must be defined in any new subclasses
  virtual void initialize();
  virtual void initialize(byte rollChannel, byte pitchChannel, byte yawChannel) 
  {
    this->_initialize(rollChannel, pitchChannel, yawChannel);
  }
  virtual void measure(void);
  virtual void calibrate(void);
  virtual void autoZero(void){};
  virtual const int getFlightData(byte);

  // The following functions are common between all Gyro subclasses
  void _initialize(byte rollChannel, byte pitchChannel, byte yawChannel) 
  {
    _gyroChannel[0] = rollChannel;
    _gyroChannel[1] = pitchChannel;
    _gyroChannel[2] = yawChannel;
    
    _previousTime = micros();
  }
    
  const int getRaw(byte axis) 
  {
    return _gyroADC[axis] * _sign[axis];
  }
  
  const int getData(byte axis) 
  {
    return _gyroData[axis] * _sign[axis];
  }
  
  void setData(byte axis, int value) 
  {
    _gyroData[axis] = value;
  }
  
  const int invert(byte axis) 
  {
    _sign[axis] = -_sign[axis];
    return _sign[axis];
  }
  
  const int getZero(byte axis) 
  {
    return _gyroZero[axis];
  }
  
  void setZero(byte axis, int value) 
  {
    _gyroZero[axis] = value;
  }    
  
  const float getScaleFactor() 
  {
    return _gyroScaleFactor;
  }

  const float getSmoothFactor(void) 
  {
    return _smoothFactor;
  }
  
  void setSmoothFactor(float value) 
  {
    _smoothFactor = value;
  }

  const float rateDegPerSec(byte axis) 
  {
    return ((_gyroADC[axis] * _sign[axis])) * _gyroScaleFactor;
  }

  const float rateRadPerSec(byte axis) 
  {
    return radians(rateDegPerSec(axis));
  }
  
  // returns heading as +/- 180 degrees
  const float getHeading(void) 
  {
    div_t integerDivide;
    
    integerDivide = div(_rawHeading, 360);
    _gyroHeading = _rawHeading + (integerDivide.quot * -360);
    if (_gyroHeading > 180) 
    {
      _gyroHeading -= 360;
    }
    if (_gyroHeading < -180)
    {
      _gyroHeading += 360;
    }
    return _gyroHeading;
  }
  
  const float getRawHeading(void) 
  {
    return _rawHeading;
  }
  
  void setStartHeading(float value) 
  {
    // since a relative heading, get starting absolute heading from compass class
    _rawHeading = value;
  }
  
  void setReceiverYaw(int value) 
  {
    _receiverYaw = value;
  }
};

/******************************************************/
/****************** AeroQuad_v1 Gyro ******************/
/******************************************************/
#if defined(AeroQuad_v1) || defined(AeroQuad_v1_IDG) || defined(AeroQuadMega_v1)
class Gyro_AeroQuad_v1 : public Gyro 
{
public:
  Gyro_AeroQuad_v1() : Gyro() 
  {
    _gyroFullScaleOutput = 500.0;   // IDG/IXZ500 full scale output = +/- 500 deg/sec
    _gyroScaleFactor = 0.4;         // IDG/IXZ500 sensitivity = 2mV/(deg/sec)  2.0mV/Ã‚Âº/s, 0.8mV/ADC step => 0.8/3.33 = 0.4
  }
  
  void initialize(void) 
  {
    analogReference(EXTERNAL);
    // Configure gyro auto zero pins
    pinMode (AZPIN, OUTPUT);
    digitalWrite(AZPIN, LOW);
    delay(1);

    // rollChannel = 4
    // pitchChannel = 3
    // yawChannel = 5
    this->_initialize(4,3,5);
  }
  
  void measure(void) 
  {
    _currentTime = micros();
    for (byte axis = 0; axis < 3; axis++) 
    {
      _gyroADC[axis] = _gyroZero[axis] - analogRead(_gyroChannel[axis]);
      _gyroData[axis] = filterSmooth(_gyroADC[axis], _gyroData[axis], _smoothFactor);
    }
    _previousTime = _currentTime;
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }
  
 void calibrate() 
 {
    autoZero();
  }
  
  void autoZero() 
  {
    int findZero[FINDZERO];
    digitalWrite(AZPIN, HIGH);
    delayMicroseconds(750);
    digitalWrite(AZPIN, LOW);
    delay(8);

    for (byte calAxis = 0; calAxis < 3; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++)
      {
        findZero[i] = analogRead(_gyroChannel[calAxis]);
      }
      _gyroZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }
  }
};
#endif

/******************************************************/
/****************** AeroQuad_v2 Gyro ******************/
/******************************************************/
#if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
/*
  10kOhm pull-ups on I2C lines.
  VDD & VIO = 3.3V
  SDA -> A4 (PC4)
  SCL -> A5 (PC5)
  INT -> D2 (PB2) (or no connection, not used here)
  CLK -> GND
*/
class Gyro_AeroQuadMega_v2 : public Gyro 
{
private:
  int _gyroAddress;
  long int _previousGyroTime;
  
public:
  Gyro_AeroQuadMega_v2() : Gyro() 
  {
    _gyroAddress = 0x69;
    _gyroFullScaleOutput = 2000.0;   // ITG3200 full scale output = +/- 2000 deg/sec
    _gyroScaleFactor = 1.0 / 14.375;       //  ITG3200 14.375 LSBs per °/sec
    
    _lastReceiverYaw=0;
    _yawAge=0;
    _positiveGyroYawCount=1;
    _negativeGyroYawCount=1;
    _zeroGyroYawCount=1;
    _previousGyroTime = micros();
  }
  
  void initialize(void) 
  {
    this->_initialize(0,1,2);
    
    // Check if gyro is connected
    if (readWhoI2C(_gyroAddress) != _gyroAddress)
    {
      Serial.println("Gyro not found!");
    }
        
    // Thanks to SwiftingSpeed for updates on these settings
    // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11207&viewfull=1#post11207
    updateRegisterI2C(_gyroAddress, 0x3E, 0x80); // send a reset to the device
    updateRegisterI2C(_gyroAddress, 0x16, 0x1D); // 10Hz low pass filter
    updateRegisterI2C(_gyroAddress, 0x3E, 0x01); // use internal oscillator 
  }
  
  void measure(void) 
  {
    sendByteI2C(_gyroAddress, 0x1D);
    Wire.requestFrom(_gyroAddress, 6);

    for (byte axis = 0; axis < 3; axis++) 
    {
      _gyroADC[axis] = ((Wire.receive() << 8) | Wire.receive()) - _gyroZero[axis];
      _gyroData[axis] = filterSmooth(_gyroADC[axis], _gyroData[axis], _smoothFactor);
    }

    //calculateHeading();
    unsigned long currentGyroTime = micros();
    _rawHeading += -_gyroADC[2] * _gyroScaleFactor * ((currentGyroTime - _previousGyroTime) / 1000000.0);
    //Serial.println(rawHeading);
    _previousGyroTime = currentGyroTime;

    // ************ Correct for gyro drift by FabQuad **************
    // ************ http://aeroquad.com/entry.php?4-  **************
    // Modified FabQuad's approach to use yaw transmitter command instead of checking accelerometer
    if (abs(_lastReceiverYaw - _receiverYaw) < 15) 
    {
      _yawAge++;
      if (_yawAge >= 4) 
      {  // if gyro was the same long enough, we can assume that there is no (fast) rotation
        if (_gyroData[2] < 0) 
        {
          _negativeGyroYawCount++; // if gyro still indicates negative rotation, that's additional signal that gyroZero is too low
        }
        else if (_gyroData[2] > 0) 
        {
          _positiveGyroYawCount++;  // additional signal that gyroZero is too high
        }
        else 
        {
          _zeroGyroYawCount++; // additional signal that gyroZero is correct
        }
        _yawAge = 0;
        if (_zeroGyroYawCount + _negativeGyroYawCount + _positiveGyroYawCount > 50) 
        {
          if (3*_negativeGyroYawCount >= 4*(_zeroGyroYawCount + _positiveGyroYawCount)) 
          {
            _gyroZero[2]--;  // enough signals the gyroZero is too low
          }
          if (3 * _positiveGyroYawCount >= 4 * (_zeroGyroYawCount + _negativeGyroYawCount)) 
          {
            _gyroZero[2]++;  // enough signals the gyroZero is too high
          }
          _zeroGyroYawCount=0;
          _negativeGyroYawCount=0;
          _positiveGyroYawCount=0;
        }
      }
    }
    else 
    { // gyro different, restart
      _lastReceiverYaw = _receiverYaw;
      _yawAge = 0;
    }
  }
  
  const int getFlightData(byte axis) 
  {
    int reducedData;
    reducedData = getRaw(axis) >> 3;
    //if ((reducedData < 5) && (reducedData > -5)) reducedData = 0;
    return reducedData;
  }

  void calibrate() 
  {
    autoZero();
  }
  
  void autoZero() 
  {
    int findZero[FINDZERO];
    for (byte calAxis = 0; calAxis < 3; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++) 
      {
        sendByteI2C(_gyroAddress, (calAxis * 2) + 0x1D);
        findZero[i] = readWordI2C(_gyroAddress);
        delay(10);
      }
      _gyroZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }
  }
};
#endif

/******************************************************/
/**************** ArduCopter Gyro *********************/
/******************************************************/
#ifdef ArduCopter
class Gyro_ArduCopter : public Gyro 
{
private:
  int _rawADC;

public:
  Gyro_ArduCopter() : Gyro() 
  {
    // IDG500 Sensitivity (from datasheet) => 2.0mV/Ã‚Âº/s, 0.8mV/ADC step => 0.8/3.33 = 0.4
    // Tested values : 
    //#define Gyro_Gain_X 0.4 //X axis Gyro gain
    //#define Gyro_Gain_Y 0.41 //Y axis Gyro gain
    //#define Gyro_Gain_Z 0.41 //Z axis Gyro gain
    //#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
    //#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
    //#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second
    _gyroFullScaleOutput = 500.0;   // IDG/IXZ500 full scale output = +/- 500 deg/sec
    _gyroScaleFactor = 0.4;       // IDG/IXZ500 sensitivity = 2mV/(deg/sec) 0.002
  }
  
  void initialize(void) 
  {
    // rollChannel = 1
    // pitchChannel = 2
    // yawChannel = 0
    this->_initialize(1, 2, 0);
    initialize_ArduCopter_ADC(); // this is needed for both gyros and accels, done once in this class
  }
  
  void measure(void) 
  {
    for (byte axis = 0; axis < 3; axis++) 
    {
      _rawADC = analogRead_ArduCopter_ADC(_gyroChannel[axis]);
      if (_rawADC > 500) // Check if good measurement
      {
        _gyroADC[axis] = _rawADC - _gyroZero[axis];
      }
      _gyroData[axis] = _gyroADC[axis]; // no smoothing needed
    }
   }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }

  void calibrate() 
  {
    int findZero[FINDZERO];
    for (byte calAxis = 0; calAxis < 3; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++) 
      {
        findZero[i] = analogRead_ArduCopter_ADC(_gyroChannel[calAxis]);
        delay(2);
      }
      _gyroZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }
  }
};
#endif

/******************************************************/
/********************** Wii Gyro **********************/
/******************************************************/
#if defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii)
class Gyro_Wii : public Gyro 
{
public:
  Gyro_Wii() : Gyro() 
  {
    // 0.5mV/Ã‚Âº/s, 0.2mV/ADC step => 0.2/3.33 = around 0.069565217391304
    // @see http://invensense.com/mems/gyro/documents/PS-IDG-0650B-00-05.pdf and
    // @see http://invensense.com/mems/gyro/documents/ps-isz-0650b-00-05.pdf
    _gyroFullScaleOutput = 2000;
    _gyroScaleFactor = 0.069565217391304;
  }
  
  void initialize(void)
  {
    Init_Gyro_Acc(); // defined in DataAquisition.h
  }
  
  void measure(void) 
  {
    _currentTime = micros();
    updateControls(); // defined in DataAcquisition.h
    _gyroADC[0] = NWMP_gyro[0] - _gyroZero[0];
    _gyroData[0] = filterSmoothWithTime(_gyroADC[0], _gyroData[0], _smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    _gyroADC[1] = _gyroZero[1] - NWMP_gyro[1];
    _gyroData[1] = filterSmoothWithTime(_gyroADC[1], _gyroData[1], _smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    _gyroADC[2] =  _gyroZero[2] - NWMP_gyro[2];
    _gyroData[2] = filterSmoothWithTime(_gyroADC[2], _gyroData[2], _smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    _previousTime = _currentTime;
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }

  void calibrate() 
  {
    int findZero[FINDZERO];
  
    for (byte calAxis = 0; calAxis < 3; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++)
      {
        updateControls();
        findZero[i] = NWMP_gyro[calAxis];
      }
      _gyroZero[calAxis] = findMedianInt(findZero, FINDZERO);
    }
  }
};
#endif

/******************************************************/
/********************** CHR6DM Gyro **********************/
/******************************************************/
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
class Gyro_CHR6DM : public Gyro 
{
public:
  Gyro_CHR6DM() : Gyro() 
  {
    _gyroFullScaleOutput = 0;
    _gyroScaleFactor = 0;
  }

  void initialize(void) 
  {
    initCHR6DM();
  }

  void measure(void) 
  {
    _currentTime = micros();
    readCHR6DM();
    _gyroADC[0] = chr6dm.data.rollRate - _gyroZero[0]; //gx yawRate
    _gyroADC[1] = _gyroZero[1] - chr6dm.data.pitchRate; //gy pitchRate
    _gyroADC[2] = chr6dm.data.yawRate - _gyroZero[2]; //gz rollRate

    _gyroData[0] = filterSmoothWithTime(_gyroADC[0], _gyroData[0], _smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    _gyroData[1] = filterSmoothWithTime(_gyroADC[1], _gyroData[1], _smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    _gyroData[2] = filterSmoothWithTime(_gyroADC[2], _gyroData[2], _smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    _previousTime = _currentTime;
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }

  void calibrate() 
  {
    float zeroXreads[FINDZERO];
    float zeroYreads[FINDZERO];
    float zeroZreads[FINDZERO];
    for (int i=0; i<FINDZERO; i++) 
    {
        readCHR6DM();
        zeroXreads[i] = chr6dm.data.rollRate;
        zeroYreads[i] = chr6dm.data.pitchRate;
        zeroZreads[i] = chr6dm.data.yawRate;
    }
    _gyroZero[0] = findMedianFloat(zeroXreads, FINDZERO);
    _gyroZero[1] = findMedianFloat(zeroYreads, FINDZERO);
    _gyroZero[2] = findMedianFloat(zeroZreads, FINDZERO);
  }
};
#endif

/*************************************************/
/***************** CHR6DM FAKE Gyro **************/
/*************************************************/
#ifdef CHR6DM_FAKE_GYRO
class Gyro_CHR6DM_Fake : public Gyro 
{
public:
  float fakeGyroRoll;
  float fakeGyroPitch;
  float fakeGyroYaw;
  Gyro_CHR6DM_Fake() : Gyro() 
  {
    gyroFullScaleOutput = 0;
    gyroScaleFactor = 0;
  }

  void initialize(void) 
  {
    gyroZero[0] = 0;
    gyroZero[1] = 0;
    gyroZero[2] = 0;
  }

  void measure(void) 
  {
    currentTime = micros();
    readFakeValues();
    gyroADC[0] = fakeGyroRoll - gyroZero[0]; //gx yawRate
    gyroADC[1] = fakeGyroPitch - gyroZero[1]; //gy pitchRate
    gyroADC[2] = fakeGyroYaw - gyroZero[2]; //gz rollRate

    gyroData[0] = filterSmooth(gyroADC[0], gyroData[0], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[1] = filterSmooth(gyroADC[1], gyroData[1], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[2] = filterSmooth(gyroADC[2], gyroData[2], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    previousTime = currentTime;
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }

  void calibrate() 
  {
    float zeroXreads[FINDZERO];
    float zeroYreads[FINDZERO];
    float zeroZreads[FINDZERO];
    for (int i=0; i<FINDZERO; i++) 
    {
        readFakeValues();
        zeroXreads[i] = fakeGyroRoll;
        zeroYreads[i] = fakeGyroPitch;
        zeroZreads[i] = fakeGyroYaw;
    }

    gyroZero[0] = findMedian(zeroXreads, FINDZERO);
    gyroZero[1] = findMedian(zeroYreads, FINDZERO);
    gyroZero[2] = findMedian(zeroZreads, FINDZERO);
  }

  void readFakeValues()
  {
    if (!syncToHeader())
    {
        return;
    }

    fakeGyroRoll = readInt();
    fakeGyroPitch = readInt();
    fakeGyroYaw = readInt();

    fakeAccelRoll = readInt();
    fakeAccelPitch = readInt();
    fakeAccelYaw = readInt();

    Serial2.print("fakeGyroRoll=");
    Serial2.println(fakeGyroRoll);
    Serial2.print("fakeGyroPitch=");
    Serial2.println(fakeGyroPitch);
    Serial2.print("fakeGyroYaw=");
    Serial2.println(fakeGyroYaw);

    Serial2.print("fakeAccelRoll=");
    Serial2.println(fakeAccelRoll);
    Serial2.print("fakeAccelPitch=");
    Serial2.println(fakeAccelPitch);
    Serial2.print("fakeAccelYaw=");
    Serial2.println(fakeAccelYaw);
  }

  int readInt() 
  {
    return word(blockingRead(),blockingRead());
  }

  int blockingRead() 
  {
    int read=-1;

    long starttime = millis();
    while(read==-1 && (millis()-starttime)<100) 
    {
      read = Serial2.read();
    }
    return read;
  }

  bool syncToHeader() 
  {
    while (Serial2.available()>0)
    {
      if (blockingRead()=='a' && blockingRead()=='b' && blockingRead()=='c' ) return true;
    }
    return false;
  }
};
#endif

/******************************************************/
/******************* Multipilot Gyro ******************/
/******************************************************/
#if defined(Multipilot) || defined(MultipilotI2C)
class Gyro_Multipilot : public Gyro 
{
private:

public:
  Gyro_Multipilot() : Gyro() 
  {
    gyroFullScaleOutput = 300.0;        // ADXR610 full scale output = +/- 300 deg/sec
    gyroScaleFactor = aref / 0.006;     // ADXR610 sensitivity = 6mV/(deg/sec)
  }
  
  void initialize(void) 
  {
    analogReference(EXTERNAL);
    // Configure gyro auto zero pins
    pinMode (AZPIN, OUTPUT);
    digitalWrite(AZPIN, LOW);
    delay(1);

    // rollChannel = 1
    // pitchChannel = 2
    // yawChannel = 0
    this->_initialize(1,2,0);
  }
  
  void measure(void) 
  {
    currentTime = micros();
    for (byte axis = 0; axis < 3; axis++) 
    {
      gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
      gyroData[axis] = filterSmooth(gyroADC[axis], gyroData[axis], smoothFactor); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    }
    previousTime = currentTime;
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }

  void calibrate() 
  {
    autoZero();
  }
  
  void autoZero() 
  {
    int findZero[FINDZERO];
    digitalWrite(AZPIN, HIGH);
    delayMicroseconds(750);
    digitalWrite(AZPIN, LOW);
    delay(8);

    for (byte calAxis = 0; calAxis < 3; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++)
      {
        findZero[i] = analogRead(gyroChannel[calAxis]);
      }
      gyroZero[calAxis] = findMedian(findZero, FINDZERO);
    }
  }
};
#endif


