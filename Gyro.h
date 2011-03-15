/*
  AeroQuad v2.3 - March 2011
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

class Gyro {
public:
  float gyroFullScaleOutput;
  float gyroScaleFactor;
  float smoothFactor;
  int gyroChannel[3];
  float gyroData[3];
  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    float gyroZero[3];
  #else
    int gyroZero[3];
  #endif
  int gyroADC[3];
  byte rollChannel, pitchChannel, yawChannel;
  int sign[3];
  float rawHeading, gyroHeading;
  //unsigned long currentTime, previousTime; // AKA - Changed to remove HONKS time smoothing
  
  // ************ Correct for gyro drift by FabQuad **************  
  // ************ http://aeroquad.com/entry.php?4-  **************     
  //int lastReceiverYaw, receiverYaw;
  //long yawAge;
  //int positiveGyroYawCount;
  //int negativeGyroYawCount;
  //int zeroGyroYawCount;
    
  Gyro(void){
    sign[ROLL] = 1;
    sign[PITCH] = 1;
    sign[YAW] = 1;
  }
  
  // The following function calls must be defined in any new subclasses
  virtual void initialize(byte rollChannel, byte pitchChannel, byte yawChannel) {
    this->_initialize(rollChannel, pitchChannel, yawChannel);
  }
  virtual void measure(void);
  virtual void calibrate(void);
  virtual void autoZero(void){};
  virtual const int getFlightData(byte);

  // The following functions are common between all Gyro subclasses
  void _initialize(byte rollChannel, byte pitchChannel, byte yawChannel) {
    gyroChannel[ROLL] = rollChannel;
    gyroChannel[PITCH] = pitchChannel;
    gyroChannel[ZAXIS] = yawChannel;
    
    gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
    gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
    gyroZero[ZAXIS] = readFloat(GYRO_YAW_ZERO_ADR);
    smoothFactor = readFloat(GYROSMOOTH_ADR);
    
    //previousTime = micros();
  }

  // returns the raw ADC value from the gyro, with sign change if needed, not smoothed or scaled to SI units    
  const int getRaw(byte axis) {
    return gyroADC[axis] * sign[axis];
  }
  
  // returns the smoothed and scaled to SI units value of the Gyro with sign change if needed
  // centered on zero radians +/-
  const float getData(byte axis) {
    return gyroData[axis] * sign[axis];
  }
  
  //  inverts, if needed the sign on the specific axis
  const int invert(byte axis) {
    sign[axis] = -sign[axis];
    return sign[axis];
  }
  
  const int getZero(byte axis) {
    return gyroZero[axis];
  }
  
  void setZero(byte axis, int value) {
    gyroZero[axis] = value;
  }    
  
  // returns the scale factor used for SI units on the gyro
  const float getScaleFactor() {
    return gyroScaleFactor;
  }

  // returns the smooth factor used on the gyro
  const float getSmoothFactor(void) {
    return smoothFactor;
  }
  
  void setSmoothFactor(float value) {
    smoothFactor = value;
  }

/*  AKA commented out, not used and not correct based upon SI unit conversion
  const float rateDegPerSec(byte axis) {
    return ((gyroADC[axis] * sign[axis])) * gyroScaleFactor;
  }

  const float rateRadPerSec(byte axis) {
    return radians(rateDegPerSec(axis));
  }
*/
  
  // returns gyro based heading as +/- PI in radians
  const float getHeading(void) {
    div_t integerDivide;
    
    integerDivide = div(rawHeading, 2*PI);
    gyroHeading = rawHeading + (integerDivide.quot * -(2*PI));
    if (gyroHeading > PI) gyroHeading -= (2*PI);
    if (gyroHeading < -PI) gyroHeading += (2*PI);
    return gyroHeading;
  }

/* AKA commeted out as not used  
  const float getRawHeading(void) {
    return rawHeading;
  }
  
  void setStartHeading(float value) {
    // since a relative heading, get starting absolute heading from compass class
    rawHeading = value;
  }
*/
/*  
  void setReceiverYaw(int value) {
    receiverYaw = value;
  }
*/  
};

/******************************************************/
/****************** AeroQuad_v1 Gyro ******************/
/******************************************************/
#if defined(AeroQuad_v1) || defined(AeroQuad_v1_IDG) || defined(AeroQuadMega_v1)
class Gyro_AeroQuad_v1 : public Gyro {
public:
  Gyro_AeroQuad_v1() : Gyro() {
  }
  
  void initialize(void) {
    analogReference(EXTERNAL);
    // Configure gyro auto zero pins
    pinMode (AZPIN, OUTPUT);
    digitalWrite(AZPIN, LOW);
    delay(1);

    // rollChannel = 4
    // pitchChannel = 3
    // yawChannel = 5
    this->_initialize(4,3,5);
    gyroFullScaleOutput = 500.0;   // IDG/IXZ500 full scale output = +/- 500 deg/sec
    gyroScaleFactor = radians((aref/1024.0) / 0.002);  // IDG/IXZ500 sensitivity = 2mV/(deg/sec)
  }
  
  void measure(void) {
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      if (axis == PITCH)
        gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
      else
        gyroADC[axis] = gyroZero[axis] - analogRead(gyroChannel[axis]);
      gyroData[axis] = filterSmooth(gyroADC[axis] * gyroScaleFactor, gyroData[axis], smoothFactor);
    }
  }

  const int getFlightData(byte axis) {
    return getRaw(axis);
  }
  
 void calibrate() {
    autoZero();
    writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
  }
  
  void autoZero() {
    int findZero[FINDZERO];
    digitalWrite(AZPIN, HIGH);
    delayMicroseconds(750);
    digitalWrite(AZPIN, LOW);
    delay(8);

    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++)
        findZero[i] = analogRead(gyroChannel[calAxis]);
      gyroZero[calAxis] = findMedian(findZero, FINDZERO);
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
class Gyro_AeroQuadMega_v2 : public Gyro {
private:
  int gyroAddress;
  long int previousGyroTime;
  int gyroLastADC;
  
public:
  Gyro_AeroQuadMega_v2() : Gyro() {
    gyroAddress = 0x69;
    gyroFullScaleOutput = 2000.0;   // ITG3200 full scale output = +/- 2000 deg/sec
    gyroScaleFactor = radians(1.0 / 14.375);  //  ITG3200 14.375 LSBs per °/sec
    
    /*
    lastReceiverYaw=0;
    yawAge=0;
    positiveGyroYawCount=1;
    negativeGyroYawCount=1;
    zeroGyroYawCount=1;
    */
    previousGyroTime = micros();
    
  }
  
  void initialize(void) {
    this->_initialize(0,1,2);
    
    gyroLastADC = 0;  // initalize for rawHeading, may be able to be removed in the future
    
    // Check if gyro is connected
    if (readWhoI2C(gyroAddress) != gyroAddress)
      Serial.println("Gyro not found!");
        
    // Thanks to SwiftingSpeed for updates on these settings
    // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11207&viewfull=1#post11207
    updateRegisterI2C(gyroAddress, 0x3E, 0x80); // send a reset to the device
    updateRegisterI2C(gyroAddress, 0x16, 0x1D); // 10Hz low pass filter
    updateRegisterI2C(gyroAddress, 0x3E, 0x01); // use internal oscillator 
  }
  
  void measure(void) {
    sendByteI2C(gyroAddress, 0x1D);
    Wire.requestFrom(gyroAddress, 6);

    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      if (axis == ROLL)
        gyroADC[axis] = ((Wire.receive() << 8) | Wire.receive()) - gyroZero[axis];
      else
        gyroADC[axis] = gyroZero[axis] - ((Wire.receive() << 8) | Wire.receive());
      gyroData[axis] = filterSmooth((float)gyroADC[axis] * gyroScaleFactor, gyroData[axis], smoothFactor);
    }

    //calculateHeading();
    // gyroLastADC can maybe replaced with Zero, but will leave as is for now
    // this provides a small guard band for the gyro on Yaw before it increments or decrements the rawHeading 
    
    long int currentGyroTime = micros();
    if ((gyroADC[YAW] - gyroLastADC) > 3 || (gyroADC[YAW] - gyroLastADC) < -3) {
      //Serial.print(gyroADC[YAW]);
      //Serial.print(",");
      //Serial.print(rawHeading);
      //Serial.print(",");
      //Serial.print(currentGyroTime - previousGyroTime);      
      //Serial.print(",");
      rawHeading += gyroADC[YAW] * gyroScaleFactor * ((currentGyroTime - previousGyroTime) / 1000000.0);
      //Serial.print(rawHeading);
      //Serial.println();
    }
    previousGyroTime = currentGyroTime;
      //gyroLastADC = gyroADC[YAW];

/*
    // ************ Correct for gyro drift by FabQuad **************
    // ************ http://aeroquad.com/entry.php?4-  **************
    // Modified FabQuad's approach to use yaw transmitter command instead of checking accelerometer
    if (abs(lastReceiverYaw - receiverYaw) < 15) {
      yawAge++;
      if (yawAge >= 4) {  // if gyro was the same long enough, we can assume that there is no (fast) rotation
        if (gyroData[YAW] < 0) {
          negativeGyroYawCount++; // if gyro still indicates negative rotation, that's additional signal that gyroZero is too low
        }
        else if (gyroData[YAW] > 0) {
          positiveGyroYawCount++;  // additional signal that gyroZero is too high
        }
        else {
          zeroGyroYawCount++; // additional signal that gyroZero is correct
        }
        yawAge = 0;
        if (zeroGyroYawCount + negativeGyroYawCount + positiveGyroYawCount > 50) {
          if (3*negativeGyroYawCount >= 4*(zeroGyroYawCount+positiveGyroYawCount)) 
            gyroZero[YAW]--;  // enough signals the gyroZero is too low
          if (3*positiveGyroYawCount >= 4*(zeroGyroYawCount+negativeGyroYawCount)) 
            gyroZero[YAW]++;  // enough signals the gyroZero is too high
          zeroGyroYawCount=0;
          negativeGyroYawCount=0;
          positiveGyroYawCount=0;
        }
      }
    }
    else { // gyro different, restart
      lastReceiverYaw = receiverYaw;
      yawAge = 0;
    }
    */
  }
  
  // returns raw ADC data from the Gyro centered on zero +/- values
  const int getFlightData(byte axis) {
    //int reducedData = getRaw(axis) >> 3;
    //if ((reducedData < 5) && (reducedData > -5)) reducedData = 0;
    if (axis == PITCH)
      return -(getRaw(axis) >> 3);
    else
      return (getRaw(axis) >> 3);
  }

  void calibrate() {
    autoZero();
    writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
  }
  
  void autoZero() {
    int findZero[FINDZERO];
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        sendByteI2C(gyroAddress, (calAxis * 2) + 0x1D);
        findZero[i] = readWordI2C(gyroAddress);
        delay(10);
      }
      gyroZero[calAxis] = findMedian(findZero, FINDZERO);
    }
  }
};
#endif

/******************************************************/
/**************** ArduCopter Gyro *********************/
/******************************************************/
#ifdef ArduCopter
class Gyro_ArduCopter : public Gyro {
private:
  int rawADC;

public:
  Gyro_ArduCopter() : Gyro() {
    gyroScaleFactor = radians((3.3/4096) / 0.002);  // IDG/IXZ500 sensitivity = 2mV/(deg/sec)
    gyroFullScaleOutput = 500.0;   // IDG/IXZ500 full scale output = +/- 500 deg/sec
  }
  
  void initialize(void) {
    // old AQ way
    // rollChannel = 1
    // pitchChannel = 2
    // yawChannel = 0
    // revised in 2.3 way
    // rollChannel = 0
    // pitchChannel = 1
    // yawChannel = 2
    this->_initialize(0, 1, 2);
    initialize_ArduCopter_ADC(); // this is needed for both gyros and accels, done once in this class
    smoothFactor = readFloat(GYROSMOOTH_ADR);
  }
  
  void measure(void) {
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      rawADC = analogRead_ArduCopter_ADC(gyroChannel[axis]);
      if (rawADC > 500) // Check if good measurement
        if (axis == ROLL)
          gyroADC[axis] =  rawADC - gyroZero[axis];
        else
          gyroADC[axis] =  gyroZero[axis] - rawADC;
      gyroData[axis] = filterSmooth(gyroADC[axis] * gyroScaleFactor, gyroData[axis], smoothFactor);
    }
   }

  const int getFlightData(byte axis) {
    if (axis == PITCH)
      return -getRaw(axis);
    else
      return getRaw(axis);
  }

  void calibrate() {
    autoZero();
    writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
  }
  
  void autoZero() {
    int findZero[FINDZERO];
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        findZero[i] = analogRead_ArduCopter_ADC(gyroChannel[calAxis]);
        delay(10);
      }
      gyroZero[calAxis] = findMedian(findZero, FINDZERO);
    }
  }
};
#endif

/******************************************************/
/********************** Wii Gyro **********************/
/******************************************************/
#if defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii)
class Gyro_Wii : public Gyro {
private:

public:
  Gyro_Wii() : Gyro() {
    // 0.5mV/Ã‚Âº/s, 0.2mV/ADC step => 0.2/3.33 = around 0.069565217391304
    // @see http://invensense.com/mems/gyro/documents/PS-IDG-0650B-00-05.pdf and
    // @see http://invensense.com/mems/gyro/documents/ps-isz-0650b-00-05.pdf
    gyroFullScaleOutput = 2000;
    gyroScaleFactor = radians(0.06201166);
  }
  
  void initialize(void) {
    Init_Gyro_Acc(); // defined in DataAquisition.h
    smoothFactor = readFloat(GYROSMOOTH_ADR);
    gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
    gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
    gyroZero[ZAXIS] = readFloat(GYRO_YAW_ZERO_ADR);
  }
  
  void measure(void) {
    updateControls(); // defined in DataAcquisition.h
    
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      gyroADC[axis] = NWMP_gyro[axis] - gyroZero[axis];
      gyroData[axis] = filterSmooth(gyroADC[axis] * gyroScaleFactor, gyroData[axis], smoothFactor); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
      //gyroADC[PITCH] = NWMP_gyro[PITCH] - gyroZero[PITCH];
      //gyroData[PITCH] = filterSmooth(gyroADC[PITCH] * gyroScaleFactor, gyroData[PITCH], smoothFactor); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
      //gyroADC[YAW] =  NWMP_gyro[YAW] - gyroZero[YAW];
      //gyroData[YAW] = filterSmooth(gyroADC[YAW] * gyroScaleFactor, gyroData[YAW], smoothFactor); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    }
  }

  const int getFlightData(byte axis) {
    if (axis == PITCH)
      return -getRaw(PITCH);
    else
      return getRaw(axis);
  }

  void calibrate() {
    int findZero[FINDZERO];
  
	  for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        updateControls();
        findZero[i] = NWMP_gyro[calAxis];
      }
      gyroZero[calAxis] = findMedian(findZero, FINDZERO);
    }
    writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
  }
};
#endif

/******************************************************/
/********************** CHR6DM Gyro **********************/
/******************************************************/
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
class Gyro_CHR6DM : public Gyro {

public:
  Gyro_CHR6DM() : Gyro() {
    gyroFullScaleOutput = 0;
    gyroScaleFactor = 0;
  }

  void initialize(void) {
    smoothFactor = readFloat(GYROSMOOTH_ADR);
    gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
    gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
    gyroZero[ZAXIS] = readFloat(GYRO_YAW_ZERO_ADR);
    initCHR6DM();
  }

  void measure(void) {
    //currentTime = micros();
    readCHR6DM();
    gyroADC[ROLL] = chr6dm.data.rollRate - gyroZero[ROLL]; //gx yawRate
    gyroADC[PITCH] = gyroZero[PITCH] - chr6dm.data.pitchRate; //gy pitchRate
    gyroADC[YAW] = chr6dm.data.yawRate - gyroZero[ZAXIS]; //gz rollRate

    //gyroData[ROLL] = filterSmoothWithTime(gyroADC[ROLL], gyroData[ROLL], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    //gyroData[PITCH] = filterSmoothWithTime(gyroADC[PITCH], gyroData[PITCH], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    //gyroData[YAW] = filterSmoothWithTime(gyroADC[YAW], gyroData[YAW], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[ROLL] = filterSmooth(gyroADC[ROLL], gyroData[ROLL], smoothFactor); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[PITCH] = filterSmooth(gyroADC[PITCH], gyroData[PITCH], smoothFactor); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[YAW] = filterSmooth(gyroADC[YAW], gyroData[YAW], smoothFactor); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    
    //previousTime = currentTime;
  }

  const int getFlightData(byte axis) {
    return getRaw(axis);
  }

  void calibrate() {

    float zeroXreads[FINDZERO];
    float zeroYreads[FINDZERO];
    float zeroZreads[FINDZERO];

    for (int i=0; i<FINDZERO; i++) {
        readCHR6DM();
        zeroXreads[i] = chr6dm.data.rollRate;
        zeroYreads[i] = chr6dm.data.pitchRate;
        zeroZreads[i] = chr6dm.data.yawRate;
    }

    gyroZero[XAXIS] = findMedian(zeroXreads, FINDZERO);
    gyroZero[YAXIS] = findMedian(zeroYreads, FINDZERO);
    gyroZero[ZAXIS] = findMedian(zeroZreads, FINDZERO);

    writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
  }
};
#endif

/*************************************************/
/***************** CHR6DM FAKE Gyro **************/
/*************************************************/
#ifdef CHR6DM_FAKE_GYRO
class Gyro_CHR6DM_Fake : public Gyro {
public:
  float fakeGyroRoll;
  float fakeGyroPitch;
  float fakeGyroYaw;
  Gyro_CHR6DM_Fake() : Gyro() {
    gyroFullScaleOutput = 0;
    gyroScaleFactor = 0;
  }

  void initialize(void) {
    smoothFactor = readFloat(GYROSMOOTH_ADR);
    gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
    gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
    gyroZero[ZAXIS] = readFloat(GYRO_YAW_ZERO_ADR);
    gyroZero[ROLL] = 0;
    gyroZero[PITCH] = 0;
    gyroZero[ZAXIS] = 0;
  }

  void measure(void) {
    //currentTime = micros();
    readFakeValues();
    gyroADC[ROLL] = fakeGyroRoll - gyroZero[ROLL]; //gx yawRate
    gyroADC[PITCH] = fakeGyroPitch - gyroZero[PITCH]; //gy pitchRate
    gyroADC[YAW] = fakeGyroYaw - gyroZero[ZAXIS]; //gz rollRate

    //gyroData[ROLL] = filterSmooth(gyroADC[ROLL], gyroData[ROLL], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    //gyroData[PITCH] = filterSmooth(gyroADC[PITCH], gyroData[PITCH], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    //gyroData[YAW] = filterSmooth(gyroADC[YAW], gyroData[YAW], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[ROLL] = filterSmooth(gyroADC[ROLL], gyroData[ROLL], smoothFactor); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[PITCH] = filterSmooth(gyroADC[PITCH], gyroData[PITCH], smoothFactor); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[YAW] = filterSmooth(gyroADC[YAW], gyroData[YAW], smoothFactor); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1

    //previousTime = currentTime;
  }

  const int getFlightData(byte axis) {
    return getRaw(axis);
  }

  void calibrate() {
    float zeroXreads[FINDZERO];
    float zeroYreads[FINDZERO];
    float zeroZreads[FINDZERO];
    for (int i=0; i<FINDZERO; i++) {
        readFakeValues();
        zeroXreads[i] = fakeGyroRoll;
        zeroYreads[i] = fakeGyroPitch;
        zeroZreads[i] = fakeGyroYaw;
    }

    gyroZero[XAXIS] = findMedian(zeroXreads, FINDZERO);
    gyroZero[YAXIS] = findMedian(zeroYreads, FINDZERO);
    gyroZero[ZAXIS] = findMedian(zeroZreads, FINDZERO);

    writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
  }

  void readFakeValues(){
    if (!syncToHeader()){
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

  int readInt() {
    return word(blockingRead(),blockingRead());
  }

  int blockingRead() {
    int read=-1;

    long starttime = millis();
    while(read==-1 && (millis()-starttime)<100) {
      read = Serial2.read();
    }
    return read;
  }

  bool syncToHeader() {
    while (Serial2.available()>0){
      if (blockingRead()=='a' && blockingRead()=='b' && blockingRead()=='c' ) return true;
    }
    return false;
  }
};
#endif
