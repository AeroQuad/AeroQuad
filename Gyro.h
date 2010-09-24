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

class Gyro {
public:
  float gyroFullScaleOutput;
  float gyroScaleFactor;
  float smoothFactor;
  int gyroChannel[3];
  int gyroData[3];
  int gyroZero[3];
  int gyroADC[3];
  byte rollChannel, pitchChannel, yawChannel;
  int sign[3];
  float currentHeading;
  
  Gyro(void){
    sign[ROLL] = 1;
    sign[PITCH] = 1;
    sign[YAW] = 1;
    currentHeading = 0;
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
  
  const float getHeading(void) {
    currentHeading += getData(YAW) * gyroScaleFactor * G_Dt;
    return currentHeading;
  }
};

/******************************************************/
/****************** AeroQuad_v1 Gyro ******************/
/******************************************************/
class Gyro_AeroQuad_v1 : public Gyro {
private:
  int findZero[FINDZERO];

public:
  Gyro_AeroQuad_v1() : Gyro() {
    gyroFullScaleOutput = 500.0;   // IDG/IXZ500 full scale output = +/- 500 deg/sec
    gyroScaleFactor = aref / 0.002;       // IDG/IXZ500 sensitivity = 2mV/(deg/sec)
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
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
      gyroData[axis] = smooth(gyroADC[axis], gyroData[axis], smoothFactor);
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

/******************************************************/
/****************** AeroQuad_v2 Gyro ******************/
/******************************************************/
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
  int findZero[FINDZERO];
  int gyroAddress;
  int data;
  int rawData[3];
  byte select;  // use to select which axis is being read
  
public:
  Gyro_AeroQuadMega_v2() : Gyro() {
    gyroAddress = 0x69;
    gyroFullScaleOutput = 2000.0;   // ITG3200 full scale output = +/- 2000 deg/sec
    gyroScaleFactor = 1.0 / 14.375;       //  ITG3200 14.375 LSBs per °/sec
  }
  
  void initialize(void) {
    this->_initialize(0,1,2);
    smoothFactor = readFloat(GYROSMOOTH_ADR);
    data =  0x0;
    select = ROLL;
    
    // Check if gyro is connected
    if (readWhoI2C(gyroAddress) != gyroAddress)
      Serial.println("Gyro not found!");
        
    updateRegisterI2C(gyroAddress, 0x3E, 0x80); // send a reset to the device
    updateRegisterI2C(gyroAddress, 0x15, 0x00); // 1kHz sample rate
    updateRegisterI2C(gyroAddress, 0x16, 0x1D); // 10Hz low pass filter
    updateRegisterI2C(gyroAddress, 0x17, 0x05); // enable send raw values
    updateRegisterI2C(gyroAddress, 0x3E, 0x00); // use internal oscillator    
  }
  
  void measure(void) {
    // round robin between each axis so that I2C blocking time is low
    if (select == ROLL) sendByteI2C(gyroAddress, 0x1D);
    if (select == PITCH) sendByteI2C(gyroAddress, 0x1F);
    if (select == YAW) sendByteI2C(gyroAddress, 0x21);
    rawData[select] = readWordI2C(gyroAddress);
    gyroADC[select] = rawData[select] - gyroZero[select];
    gyroData[select] = smooth(gyroADC[select], gyroData[select], smoothFactor);
    if (++select == LASTAXIS) select = ROLL; // go to next axis, reset to ROLL if past ZAXIS
  }

  const int getFlightData(byte axis) {
    return getRaw(axis) >> 3;
  }

  void calibrate() {
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        sendByteI2C(gyroAddress, (calAxis * 2) + 0x1D);
        findZero[i] = readWordI2C(gyroAddress);
      }
      gyroZero[calAxis] = findMode(findZero, FINDZERO);
    }
    writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
    writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
    writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
  }
  
  void autoZero() {
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        sendByteI2C(gyroAddress, (calAxis * 2) + 0x1D);
        findZero[i] = readWordI2C(gyroAddress);
      }
      gyroZero[calAxis] = findMode(findZero, FINDZERO);
    }
  }
};

/******************************************************/
/**************** ArduCopter Gyro *********************/
/******************************************************/
#ifdef ArduCopter
class Gyro_ArduCopter : public Gyro {
private:
  int findZero[FINDZERO];
  int rawADC;

public:
  Gyro_ArduCopter() : Gyro() {
    // IDG500 Sensitivity (from datasheet) => 2.0mV/º/s, 0.8mV/ADC step => 0.8/3.33 = 0.4
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
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        findZero[i] = analogRead_ArduCopter_ADC(gyroChannel[calAxis]);
        delay(1);
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
class Gyro_Wii : public Gyro {
private:
  int findZero[FINDZERO];

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
    updateControls(); // defined in DataAcquisition.h
    gyroADC[ROLL] = NWMP_gyro[ROLL] - gyroZero[ROLL];
    gyroData[ROLL] = smooth(gyroADC[ROLL], gyroData[ROLL], smoothFactor);
    gyroADC[PITCH] = NWMP_gyro[PITCH] - gyroZero[PITCH];
    gyroData[PITCH] = smooth(gyroADC[PITCH], gyroData[PITCH], smoothFactor);
    gyroADC[YAW] = NWMP_gyro[YAW] - gyroZero[YAW];
    gyroData[YAW] = smooth(gyroADC[YAW], gyroData[YAW], smoothFactor);
  }

  const int getFlightData(byte axis) {
    return getRaw(axis);
  }

  void calibrate() {
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

/******************************************************/
/******************* Multipilot Gyro ******************/
/******************************************************/
class Gyro_Multipilot : public Gyro {
private:
  int findZero[FINDZERO];

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
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
      gyroData[axis] = smooth(gyroADC[axis], gyroData[axis], smoothFactor);
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

