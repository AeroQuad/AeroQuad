/*
  AeroQuad v2.0 - July 2010
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
  Gyro(void){}
  
  // The following function calls must be defined in any new subclasses
  virtual void initialize(byte rollChannel, byte pitchChannel, byte yawChannel) {
    this->_initialize(rollChannel, pitchChannel, yawChannel);
  }
  virtual const int measure(byte axis);
  virtual void calibrate(void);
  virtual void autoZero(void){};
  
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
    return gyroADC[axis];
  }
  
  const int getData(byte axis) {
    return gyroData[axis];
  }
  
  void setData(byte axis, int value) {
    gyroData[axis] = value;
  }
  
  const int invert(byte axis) {
    gyroData[axis] = -gyroData[axis];
    return gyroData[axis];
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
    return (gyroADC[axis] / 1024.0) * gyroScaleFactor;
  }

  const float rateRadPerSec(byte axis) {
    return radians((gyroADC[axis] / 1024.0) * gyroScaleFactor);
  }
};

/******************************************************/
/****************** AeroQuad_v1 Gyro ******************/
/******************************************************/
#ifdef AeroQuad_v1
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
  
  const int measure(byte axis) {
    gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
    gyroData[axis] = smooth(gyroADC[axis], gyroData[axis], smoothFactor);
    return gyroData[axis];
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
#endif

/******************************************************/
/****************** AeroQuad_v2 Gyro ******************/
/******************************************************/
#ifdef AeroQuad_v2
/*
  Tested on a 3.3V 8MHz Arduino Pro
  10kOhm pull-ups on I2C lines.
  VDD & VIO = 3.3V
  SDA -> A4 (PC4)
  SCL -> A5 (PC5)
  INT -> D2 (PB2) (or no connection, not used here)
  CLK -> GND
*/
class Gyro_AeroQuad_v2 : public Gyro {
private:
  int findZero[FINDZERO];
  int gyroAddress;
  int data;
  int rawData[3];
  
public:
  Gyro_AeroQuad_v2() : Gyro() {
    gyroAddress = 0x69;
    gyroFullScaleOutput = 2000.0;   // IDG/IXZ500 full scale output = +/- 2000 deg/sec
    gyroScaleFactor = 0.06103515625;       // IDG/IXZ500 sensitivity (need to double check this)
  }
  
  void initialize(void) {
    this->_initialize(0,1,2);
    smoothFactor = readFloat(GYROSMOOTH_ADR);
    data =  0x0;
    
    Wire.begin();
    
    // Check if gyro is connected
    Wire.beginTransmission(0x69);
    Wire.send(0x00);
    Wire.endTransmission();
    delay(50);
    Wire.requestFrom(0x69, 1);
    data = Wire.receive();
    if (data != gyroAddress)
      Serial.println("Gyro not found!");
        
    Wire.beginTransmission(gyroAddress);
    Wire.send(0x3E);
    Wire.send(0x80);  //send a reset to the device
    Wire.endTransmission(); //end transmission
    
    Wire.beginTransmission(gyroAddress);
    Wire.send(0x15);
    Wire.send(0x00);   // 1kHz sample rate
    Wire.endTransmission(); //end transmission

    Wire.beginTransmission(gyroAddress);
    Wire.send(0x16);
    Wire.send(0x1D); // 10Hz low pass filter (0x1D)
    Wire.endTransmission(); //end transmission

    Wire.beginTransmission(gyroAddress);
    Wire.send(0x17);
    Wire.send(0x05);   // enable send raw values
    Wire.endTransmission(); //end transmission
    
    Wire.beginTransmission(gyroAddress);
    Wire.send(0x3E);
    Wire.send(0x00);  //use internal oscillator
    Wire.endTransmission(); //end transmission
    
    //Wire.onReceive(measureGyroData);
  }
  
  const int measure(byte axis) {
    gyroADC[axis] = rawData[axis] - gyroZero[axis];
    gyroData[axis] = smooth(gyroADC[axis], gyroData[axis], smoothFactor);
    return gyroData[axis];
  }
  
  void aquireData(void) {
    Wire.beginTransmission(gyroAddress);
    Wire.send(0x1D); // request high byte
    Wire.endTransmission();
    Wire.requestFrom(gyroAddress, 6);
    rawData[ROLL] = (Wire.receive()<<8) | Wire.receive();
    rawData[PITCH] = (Wire.receive()<<8) | Wire.receive();
    rawData[YAW] = (Wire.receive()<<8) | Wire.receive();
  }

  void calibrate() {
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        Wire.beginTransmission(gyroAddress);
        Wire.send((calAxis * 2) + 0x1D); // request high byte
        Wire.endTransmission();
        Wire.requestFrom(gyroAddress, 1);
        while (Wire.available() == 0) {/* wait for incoming data */};
        data = Wire.receive();  // receive high byte (overwrites previous reading)
        data = data << 8;    // shift high byte to be high 8 bits
        Wire.beginTransmission(gyroAddress);
        Wire.send((calAxis * 2) + 0x1E); // request low byte
        Wire.endTransmission();
        Wire.requestFrom(gyroAddress, 1);
        while (Wire.available() == 0) {/* wait for incoming data */};    
        data |= Wire.receive(); // receive low byte as lower 8 bits
        findZero[i] = data;
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
/********************** APM Gyro **********************/
/******************************************************/
#ifdef APM
class Gyro_APM : public Gyro {
private:
  int findZero[FINDZERO];
  int rawADC;

public:
  Gyro_APM() : Gyro() {
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
    initialize_APM_ADC(); // this is needed for both gyros and accels, done once in this class
  }
  
  const int measure(byte axis) {
    rawADC = analogRead_APM_ADC(gyroChannel[axis]);
    if (rawADC > 500) // Check if good measurement
      gyroADC[axis] = rawADC - gyroZero[axis];
    gyroData[axis] = gyroADC[axis]; // no smoothing needed
    return gyroData[axis];
  }

  void calibrate() {
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        findZero[i] = analogRead_APM_ADC(gyroChannel[calAxis]);
        delay(10);
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
#ifdef AeroQuad_Wii
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
  }
  
  const int measure(byte axis) {
    updateControls(); // defined in DataAcquisition.h
    gyroADC[axis] = NWMP_gyro[axis] - gyroZero[axis];
    gyroData[axis] = smooth(gyroADC[axis], gyroData[axis], smoothFactor) ;
    return gyroData[axis];
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
#endif
