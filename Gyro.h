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
  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    float _gyroZero[3];
  #else
    int _gyroZero[3];
  #endif

  int _gyroADC[3];
  int _gyroChannel[3];  
  int _gyroData[3];  

  int _lastReceiverYaw;  
  int _positiveGyroYawCount; 
  int _negativeGyroYawCount;  
  int _zeroGyroYawCount;  
  int _receiverYaw;  
  // ************ Correct for gyro drift by FabQuad **************  
  // ************ http://aeroquad.com/entry.php?4-  **************     
  long _yawAge;
  float _gyroFullScaleOutput;
  float _gyroScaleFactor;
  float _smoothFactor;  
  float _rawHeading;  

  unsigned long _currentTime;
  unsigned long _previousTime;
  
public:    
  Gyro(void)
  {
    _sign[ROLL] = 1;
    _sign[PITCH] = 1;
    _sign[YAW] = -1;
  }
  
  // The following function calls must be defined in any new subclasses
  virtual void initialize(byte rollChannel, byte pitchChannel, byte yawChannel) 
  {
    this->_initialize(rollChannel, pitchChannel, yawChannel);
  }
  virtual void measure(void);
  virtual void calibrate(void);
  virtual void autoZero(void){};
  virtual const int getFlightData(byte);
  virtual void initialize();

  // The following functions are common between all Gyro subclasses
  void _initialize(byte rollChannel, byte pitchChannel, byte yawChannel) 
  {
    _gyroChannel[ROLL] = rollChannel;
    _gyroChannel[PITCH] = pitchChannel;
    _gyroChannel[ZAXIS] = yawChannel;
    
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
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
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

    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
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

    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      _gyroADC[axis] = ((Wire.receive() << 8) | Wire.receive()) - _gyroZero[axis];
      _gyroData[axis] = filterSmooth(_gyroADC[axis], _gyroData[axis], _smoothFactor);
    }

    //calculateHeading();
    long int currentGyroTime = micros();
    _rawHeading += -_gyroADC[YAW] * _gyroScaleFactor * ((currentGyroTime - _previousGyroTime) / 1000000.0);
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
        if (_gyroData[YAW] < 0) 
        {
          _negativeGyroYawCount++; // if gyro still indicates negative rotation, that's additional signal that gyroZero is too low
        }
        else if (_gyroData[YAW] > 0) 
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
          if (3 * _negativeGyroYawCount >= 4 * (_zeroGyroYawCount + _positiveGyroYawCount)) 
          {
            _gyroZero[YAW]--;  // enough signals the gyroZero is too low
          }
          if (3 * _positiveGyroYawCount >= 4 * (_zeroGyroYawCount + _negativeGyroYawCount))
          {
            _gyroZero[YAW]++;  // enough signals the gyroZero is too high
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
    int reducedData = getRaw(axis) >> 3;
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
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
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
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      _rawADC = analogRead_ArduCopter_ADC(_gyroChannel[axis]);
      if (_rawADC > 500) // Check if good measurement
      {
        _gyroADC[axis] =  _rawADC - _gyroZero[axis];
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
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++) 
      {
        findZero[i] = analogRead_ArduCopter_ADC(_gyroChannel[calAxis]);
        delay(5);
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
    _gyroADC[ROLL] = NWMP_gyro[ROLL] - _gyroZero[ROLL];
    _gyroData[ROLL] = filterSmoothWithTime(_gyroADC[ROLL], _gyroData[ROLL], _smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    _gyroADC[PITCH] = _gyroZero[PITCH] - NWMP_gyro[PITCH];
    _gyroData[PITCH] = filterSmoothWithTime(_gyroADC[PITCH], _gyroData[PITCH], _smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    _gyroADC[YAW] =  _gyroZero[YAW] - NWMP_gyro[YAW];
    _gyroData[YAW] = filterSmoothWithTime(_gyroADC[YAW], _gyroData[YAW], _smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    _previousTime = _currentTime;
  }

  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }

  void calibrate() 
  {
    int findZero[FINDZERO];
  
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
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
    _gyroADC[ROLL] = chr6dm.data.rollRate - _gyroZero[ROLL]; //gx yawRate
    _gyroADC[PITCH] = _gyroZero[PITCH] - chr6dm.data.pitchRate; //gy pitchRate
    _gyroADC[YAW] = chr6dm.data.yawRate - _gyroZero[ZAXIS]; //gz rollRate

    _gyroData[ROLL] = filterSmoothWithTime(_gyroADC[ROLL], _gyroData[ROLL], _smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    _gyroData[PITCH] = filterSmoothWithTime(_gyroADC[PITCH], _gyroData[PITCH], _smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    _gyroData[YAW] = filterSmoothWithTime(_gyroADC[YAW], _gyroData[YAW], _smoothFactor, ((_currentTime - _previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
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

    _gyroZero[XAXIS] = findMedianFloat(zeroXreads, FINDZERO);
    _gyroZero[YAXIS] = findMedianFloat(zeroYreads, FINDZERO);
    _gyroZero[ZAXIS] = findMedianFloat(zeroZreads, FINDZERO);
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
  Gyro_CHR6DM_Fake() : Gyro() {
    gyroFullScaleOutput = 0;
    gyroScaleFactor = 0;
  }

  void initialize(void) 
  {
    gyroZero[ROLL] = 0;
    gyroZero[PITCH] = 0;
    gyroZero[ZAXIS] = 0;
  }

  void measure(void) 
  {
    currentTime = micros();
    readFakeValues();
    gyroADC[ROLL] = fakeGyroRoll - gyroZero[ROLL]; //gx yawRate
    gyroADC[PITCH] = fakeGyroPitch - gyroZero[PITCH]; //gy pitchRate
    gyroADC[YAW] = fakeGyroYaw - gyroZero[ZAXIS]; //gz rollRate

    gyroData[ROLL] = filterSmooth(gyroADC[ROLL], gyroData[ROLL], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[PITCH] = filterSmooth(gyroADC[PITCH], gyroData[PITCH], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[YAW] = filterSmooth(gyroADC[YAW], gyroData[YAW], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
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

    gyroZero[XAXIS] = findMedian(zeroXreads, FINDZERO);
    gyroZero[YAXIS] = findMedian(zeroYreads, FINDZERO);
    gyroZero[ZAXIS] = findMedian(zeroZreads, FINDZERO);
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
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
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

    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
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


