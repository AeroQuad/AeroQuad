/*
  AeroQuad v2.1 Beta - December 2010
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

class Gyro {
public:
  float gyroFullScaleOutput;
  float gyroScaleFactor;
  float smoothFactor;
  int gyroChannel[3];
  int gyroData[3];
  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    float gyroZero[3];
  #else
    int gyroZero[3];
  #endif
  int gyroADC[3];
  byte rollChannel, pitchChannel, yawChannel;
  int sign[3];
  float rawHeading, gyroHeading;
  unsigned long currentTime, previousTime;
  
  // ************ Correct for gyro drift by FabQuad **************  
  // ************ http://aeroquad.com/entry.php?4-  **************     
  int lastReceiverYaw, receiverYaw;
  long yawAge;
  int positiveGyroYawCount;
  int negativeGyroYawCount;
  int zeroGyroYawCount;
    
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
    
    previousTime = micros(); //was millis();
  }
    
  const int getRaw(byte axis) {
    return gyroADC[axis] * sign[axis];
  }
  
  const int getData(byte axis) {
    return gyroData[axis] * sign[axis];
  }
  
  void setData(byte axis, int value) {
    gyroData[axis] = value;
  }
  
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
  
  const float getScaleFactor() {
    return gyroScaleFactor;
  }

  const float getSmoothFactor(void) {
    return smoothFactor;
  }
  
  void setSmoothFactor(float value) {
    smoothFactor = value;
  }

  const float rateDegPerSec(byte axis) {
    return ((gyroADC[axis] * sign[axis]) / 1024.0) * gyroScaleFactor;
  }

  const float rateRadPerSec(byte axis) {
    return radians(((gyroADC[axis] * sign[axis]) / 1024.0) * gyroScaleFactor);
  }
  
  /*void calculateHeading() {
    unsigned long currentHeadingTime, previousHeadingTime;
    
    currentHeadingTime = micros();
    rawHeading += getData(YAW) * gyroScaleFactor * ((currentHeadingTime - previousHeadingTime) / 1000000.0); //working in µs now
    previousHeadingTime = currentHeadingTime;
  }*/
 
  // returns heading as +/- 180 degrees
  const float getHeading(void) {
    div_t integerDivide;
    
    integerDivide = div(rawHeading, 360);
    gyroHeading = rawHeading + (integerDivide.quot * -360);
    if (gyroHeading > 180) gyroHeading -= 360;
    if (gyroHeading < -180) gyroHeading += 360;
    return gyroHeading;
  }
  
  const float getRawHeading(void) {
    return rawHeading;
  }
  
  void setStartHeading(float value) {
    // since a relative heading, get starting absolute heading from compass class
    rawHeading = value;
  }
  
  void setReceiverYaw(int value) {
    receiverYaw = value;
  }
};

/******************************************************/
/****************** AeroQuad_v1 Gyro ******************/
/******************************************************/
#if defined(AeroQuad_v1) || defined(AeroQuadMega_v1)
class Gyro_AeroQuad_v1 : public Gyro {
private:

public:
  Gyro_AeroQuad_v1() : Gyro() {
    gyroFullScaleOutput = 500.0;   // IDG/IXZ500 full scale output = +/- 500 deg/sec
    gyroScaleFactor = 0.002;       // IDG/IXZ500 sensitivity = 2mV/(deg/sec)
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
    smoothFactor = readFloat(GYROSMOOTH_ADR);
  }
  
  void measure(void) {
    currentTime = micros();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
      gyroData[axis] = smooth(gyroADC[axis], gyroData[axis], smoothFactor);
    }
    previousTime = currentTime;
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
      gyroZero[calAxis] = findMode(findZero, FINDZERO);
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
  
public:
  Gyro_AeroQuadMega_v2() : Gyro() {
    gyroAddress = 0x69;
    gyroFullScaleOutput = 2000.0;   // ITG3200 full scale output = +/- 2000 deg/sec
    gyroScaleFactor = 1.0 / 14.375;       //  ITG3200 14.375 LSBs per °/sec
    
    lastReceiverYaw=0;
    yawAge=0;
    positiveGyroYawCount=1;
    negativeGyroYawCount=1;
    zeroGyroYawCount=1;
    previousGyroTime = micros();
  }
  
  void initialize(void) {
    this->_initialize(0,1,2);
    smoothFactor = readFloat(GYROSMOOTH_ADR);
    
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
	  int rawData[3];

    sendByteI2C(gyroAddress, 0x1D);
    Wire.requestFrom(gyroAddress, 6);
    rawData[ROLL] = (Wire.receive() << 8) | Wire.receive();
    rawData[PITCH] = (Wire.receive() << 8) | Wire.receive();
    rawData[YAW] = (Wire.receive() << 8) | Wire.receive();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      gyroADC[axis] = rawData[axis] - gyroZero[axis];
      gyroData[axis] = smooth(gyroADC[axis], gyroData[axis], smoothFactor);
    }
    //calculateHeading();
    long int currentGyroTime = micros();
    rawHeading += -gyroADC[YAW] * gyroScaleFactor * ((currentGyroTime - previousGyroTime) / 1000000.0);
    //Serial.println(rawHeading);
    previousGyroTime = currentGyroTime;
    
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
          if (negativeGyroYawCount >= 1.3*(zeroGyroYawCount+positiveGyroYawCount)) gyroZero[YAW]--;  // enough signals the gyroZero is too low
          if (positiveGyroYawCount >= 1.3*(zeroGyroYawCount+negativeGyroYawCount)) gyroZero[YAW]++;  // enough signals the gyroZero is too high
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
  }
  
  const int getFlightData(byte axis) {
    int reducedData;
    
    reducedData = getRaw(axis) >> 3;
    //if ((reducedData < 5) && (reducedData > -5)) reducedData = 0;
    return reducedData;
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
      }
      gyroZero[calAxis] = findMode(findZero, FINDZERO);
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
    // IDG500 Sensitivity (from datasheet) => 2.0mV/Ã‚Âº/s, 0.8mV/ADC step => 0.8/3.33 = 0.4
    // Tested values : 
    //#define Gyro_Gain_X 0.4 //X axis Gyro gain
    //#define Gyro_Gain_Y 0.41 //Y axis Gyro gain
    //#define Gyro_Gain_Z 0.41 //Z axis Gyro gain
    //#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
    //#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
    //#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second
    gyroFullScaleOutput = 500.0;   // IDG/IXZ500 full scale output = +/- 500 deg/sec
    gyroScaleFactor = 0.002;       // IDG/IXZ500 sensitivity = 2mV/(deg/sec)
  }
  
  void initialize(void) {
    // rollChannel = 1
    // pitchChannel = 2
    // yawChannel = 0
    this->_initialize(1, 2, 0);
    initialize_ArduCopter_ADC(); // this is needed for both gyros and accels, done once in this class
  }
  
  void measure(void) {
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      rawADC = analogRead_ArduCopter_ADC(gyroChannel[axis]);
      if (rawADC > 500) // Check if good measurement
        gyroADC[axis] = gyroZero[axis] - rawADC;
      gyroData[axis] = gyroADC[axis]; // no smoothing needed
    }
   }

  const int getFlightData(byte axis) {
    return getRaw(axis);
  }

  void calibrate() {
    int findZero[FINDZERO];
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        findZero[i] = analogRead_ArduCopter_ADC(gyroChannel[calAxis]);
        delay(2);
      }
      gyroZero[calAxis] = findMode(findZero, FINDZERO);
    }
    writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
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
    gyroFullScaleOutput = 0;
    gyroScaleFactor = 0;
  }
  
  void initialize(void) {
    Init_Gyro_Acc(); // defined in DataAquisition.h
    smoothFactor = readFloat(GYROSMOOTH_ADR);
    gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
    gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
    gyroZero[ZAXIS] = readFloat(GYRO_YAW_ZERO_ADR);
  }
  
  void measure(void) {
   currentTime = micros();
    updateControls(); // defined in DataAcquisition.h
    gyroADC[ROLL] = NWMP_gyro[ROLL] - gyroZero[ROLL];
    gyroData[ROLL] = smoothWithTime(gyroADC[ROLL], gyroData[ROLL], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroADC[PITCH] = NWMP_gyro[PITCH] - gyroZero[PITCH];
    gyroData[PITCH] = smoothWithTime(gyroADC[PITCH], gyroData[PITCH], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroADC[YAW] = NWMP_gyro[YAW] - gyroZero[YAW];
    gyroData[YAW] = smoothWithTime(gyroADC[YAW], gyroData[YAW], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
   previousTime = currentTime;
  }

  const int getFlightData(byte axis) {
    return getRaw(axis);
  }

  void calibrate() {
    int findZero[FINDZERO];
  
	  for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        updateControls();
        findZero[i] = NWMP_gyro[calAxis];
      }
      gyroZero[calAxis] = findMode(findZero, FINDZERO);
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
    currentTime = micros();
    readCHR6DM();
    gyroADC[ROLL] = chr6dm.data.rollRate - gyroZero[ROLL]; //gx yawRate
    gyroADC[PITCH] = chr6dm.data.pitchRate - gyroZero[PITCH]; //gy pitchRate
    gyroADC[YAW] = chr6dm.data.yawRate - gyroZero[ZAXIS]; //gz rollRate

    gyroData[ROLL] = smoothWithTime(gyroADC[ROLL], gyroData[ROLL], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[PITCH] = smoothWithTime(gyroADC[PITCH], gyroData[PITCH], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[YAW] = smoothWithTime(gyroADC[YAW], gyroData[YAW], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    previousTime = currentTime;
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

    gyroZero[XAXIS] = findMode(zeroXreads, FINDZERO);
    gyroZero[YAXIS] = findMode(zeroYreads, FINDZERO);
    gyroZero[ZAXIS] = findMode(zeroZreads, FINDZERO);

    writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
  }
};
#endif
/******************************************************/
/********************** CHR6DM FAKE Gyro **************/
/******************************************************/
#ifdef CHR6DM_FAKE_GYRO
class Gyro_CHR6DM_Fake : public Gyro {

public:
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
currentTime = micros();
    readFakeValues();
    gyroADC[ROLL] = fakeGyroRoll - gyroZero[ROLL]; //gx yawRate
    gyroADC[PITCH] = fakeGyroPitch - gyroZero[PITCH]; //gy pitchRate
    gyroADC[YAW] = fakeGyroYaw - gyroZero[ZAXIS]; //gz rollRate

    gyroData[ROLL] = smooth(gyroADC[ROLL], gyroData[ROLL], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[PITCH] = smooth(gyroADC[PITCH], gyroData[PITCH], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    gyroData[YAW] = smooth(gyroADC[YAW], gyroData[YAW], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
previousTime = currentTime;
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

    gyroZero[XAXIS] = findMode(zeroXreads, FINDZERO);
    gyroZero[YAXIS] = findMode(zeroYreads, FINDZERO);
    gyroZero[ZAXIS] = findMode(zeroZreads, FINDZERO);

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

  int readInt(){
    return word(blockingRead(),blockingRead());
  }


  int blockingRead(){
        int read=-1;

         long starttime = millis();
         while(read==-1 && (millis()-starttime)<100){
            read = Serial2.read();
         }

         return read;
    }

    bool syncToHeader()  {
        while (Serial2.available()>0){
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
class Gyro_Multipilot : public Gyro {
private:

public:
  Gyro_Multipilot() : Gyro() {
    gyroFullScaleOutput = 300.0;        // ADXR610 full scale output = +/- 300 deg/sec
    gyroScaleFactor = aref / 0.006;     // ADXR610 sensitivity = 6mV/(deg/sec)
  }
  
  void initialize(void) {
    analogReference(EXTERNAL);
    // Configure gyro auto zero pins
    pinMode (AZPIN, OUTPUT);
    digitalWrite(AZPIN, LOW);
    delay(1);

    // rollChannel = 1
    // pitchChannel = 2
    // yawChannel = 0
    this->_initialize(1,2,0);
    smoothFactor = readFloat(GYROSMOOTH_ADR);
  }
  
  void measure(void) {
    currentTime = micros();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
      gyroData[axis] = smooth(gyroADC[axis], gyroData[axis], smoothFactor); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    }
    previousTime = currentTime;
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
      gyroZero[calAxis] = findMode(findZero, FINDZERO);
    }
  }
};
#endif


