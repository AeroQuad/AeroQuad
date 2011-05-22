/*
  AeroQuad v2.4 - April 2011
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
//  int sign[3];
  float rawHeading, gyroHeading;
  long int previousGyroTime;
  //unsigned long currentTime, previousTime; // AKA - Changed to remove HONKS time smoothing

  Gyro(void){
//    sign[ROLL] = 1;
//    sign[PITCH] = 1;
//    sign[YAW] = 1;
  }
  
  // The following function calls must be defined in any new subclasses
  virtual void initialize(void);
//  virtual void initialize(byte rollChannel, byte pitchChannel, byte yawChannel) {
//    this->_initialize(rollChannel, pitchChannel, yawChannel);
//  }
  virtual void measure(void);
  virtual void calibrate(void);
  virtual void autoZero(void);
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
  }

  // returns the raw ADC value from the gyro, with sign change if needed, not smoothed or scaled to SI units    
  const int getRaw(byte axis) {
    return gyroADC[axis]; // * sign[axis];
  }
  
  // returns the smoothed and scaled to SI units value of the Gyro with sign change if needed
  // centered on zero radians +/-
  const float getData(byte axis) {
    return gyroData[axis]; // * sign[axis];
  }
  
  //  inverts, if needed the sign on the specific axis
//  const int invert(byte axis) {
//    sign[axis] = -sign[axis];
//    return sign[axis];
//  }
  
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

  // returns gyro based heading as +/- PI in radians
  const float getHeading(void) {
    //div_t integerDivide;
    
    //integerDivide = div(rawHeading, 2*PI);
    gyroHeading = rawHeading; // + (integerDivide.quot * -(2*PI));
    if (gyroHeading > PI) gyroHeading -= (2*PI);
    if (gyroHeading < -PI) gyroHeading += (2*PI);
    //Serial.print(integerDivide.quot);Serial.print(",");Serial.print(integerDivide.rem);Serial.println();
    return gyroHeading;
  }
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
#if defined(AeroQuad_v18) || defined(AeroQuadMega_v2) || defined(AeroQuad_Mini)
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
  //float gyroLastData;
  
public:
  Gyro_AeroQuadMega_v2() : Gyro() {
#ifdef AeroQuad_Mini
    gyroAddress = 0x68;
#else        
    gyroAddress = 0x69;
#endif    
    gyroFullScaleOutput = 2000.0;   // ITG3200 full scale output = +/- 2000 deg/sec
    gyroScaleFactor = radians(1.0 / 14.375);  //  ITG3200 14.375 LSBs per °/sec
    
    previousGyroTime = micros();
  }
  
  void initialize(void) {
//    this->_initialize(0,1,2);
    gyroZero[XAXIS] = readFloat(GYRO_ROLL_ZERO_ADR);
    gyroZero[YAXIS] = readFloat(GYRO_PITCH_ZERO_ADR);
    gyroZero[ZAXIS] = readFloat(GYRO_YAW_ZERO_ADR);
    smoothFactor = readFloat(GYROSMOOTH_ADR);
    
    //gyroLastData = 0.0;  // initalize for rawHeading, may be able to be removed in the future
    
    // Check if gyro is connected
#ifdef AeroQuad_Mini    
    if (readWhoI2C(gyroAddress) != gyroAddress +1)  // hardcoded for +1 of address specific to sparkfun 6dof imu
#else    
    if (readWhoI2C(gyroAddress) != gyroAddress)  // hardcoded for +1 of address specific to sparkfun 6dof imu
#endif    
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
    if (gyroData[YAW] > radians(1.0) || gyroData[YAW] < radians(-1.0)) {
      rawHeading += gyroData[YAW] * ((currentGyroTime - previousGyroTime) / 1000000.0);
    }
    previousGyroTime = currentGyroTime;

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
    // gyroLastADC can maybe replaced with Zero, but will leave as is for now
    // this provides a small guard band for the gyro on Yaw before it increments or decrements the rawHeading 
    long int currentGyroTime = micros();
    if (gyroData[YAW] > radians(1.0) || gyroData[YAW] < radians(-1.0)) {
      rawHeading += gyroData[YAW] * ((currentGyroTime - previousGyroTime) / 1000000.0);
    }
    previousGyroTime = currentGyroTime;
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
  #if defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii)
    float wmpLowRangeToRadPerSec;
    float wmpHighRangeToRadPerSec;
  #endif

public:
  Gyro_Wii() : Gyro() {
    // Wii Motion+ has a low range and high range. Scaling is thought to be as follows:
    //
    // Vref = 1.35 volts
    // At 0 rate, reading is approximately 8063 bits
    // Scaling is then 1.35/8063, or 0.00016743 volts/bit
    //
    // Low Range
    //    440 degrees per second at 2.7 millivolts/degree (from datasheet)
    //    degrees per bit = 0.00016743 / 2.7 mVolts = 0.06201166 degrees per second per bit
    //                                              = 0.00108231 radians per second per bit
    // High Range
    //   2000 degrees per second at 0.5 millivolts/degree (from datasheet)
    //    degrees per bit = 0.00016743 / 0.5 mVolts = 0.33486295 degrees per second per bit
    //                                              = 0.00584446 radians per second per bit
    wmpLowRangeToRadPerSec  = 0.001082308;
    wmpHighRangeToRadPerSec = 0.005844461;

    previousGyroTime = micros();
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
    
    gyroADC[ROLL] =  gyroZero[ROLL]   - NWMP_gyro[ROLL];  // Configured for Paris MultiWii Board
    gyroADC[PITCH] = NWMP_gyro[PITCH] - gyroZero[PITCH];  // Configured for Paris MultiWii Board
    gyroADC[YAW] =   gyroZero[YAW]    - NWMP_gyro[YAW];   // Configured for Paris MultiWii Board
    
    for (byte axis = ROLL; axis < LASTAXIS; axis++) { 
      gyroScaleFactor = wmpSlow[axis] ? wmpLowRangeToRadPerSec : wmpHighRangeToRadPerSec ;  // if wmpSlow == 1, use low range conversion,
      gyroData[axis] = filterSmooth(gyroADC[axis] * gyroScaleFactor, gyroData[axis], smoothFactor); 
    }
    // gyroLastADC can maybe replaced with Zero, but will leave as is for now
    // this provides a small guard band for the gyro on Yaw before it increments or decrements the rawHeading 
    long int currentGyroTime = micros();
    if (gyroData[YAW] > radians(1.0) || gyroData[YAW] < radians(-1.0)) {
      rawHeading += gyroData[YAW] * ((currentGyroTime - previousGyroTime) / 1000000.0);
    }
    previousGyroTime = currentGyroTime;
  }

  const int getFlightData(byte axis) {
    if (axis == PITCH)
      return -getRaw(PITCH) / 18;
    else
      return getRaw(axis) / 18;
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
        updateControls();
        findZero[i] = NWMP_gyro[calAxis];
      }
      gyroZero[calAxis] = findMedian(findZero, FINDZERO);
    }
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
    autoZero();
    writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
  }

  void autoZero() {

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


  }
};
#endif
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
