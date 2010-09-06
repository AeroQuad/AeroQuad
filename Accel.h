/*
  AeroQuad v2.0 - September 2010
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

class Accel {
public:
  float accelScaleFactor;
  float smoothFactor;
  int accelChannel[3];
  int accelZero[3];
  int accelData[3];
  int accelADC[3];
  int sign[3];
  byte rollChannel, pitchChannel, zAxisChannel;
  Accel(void) {
    sign[ROLL] = 1;
    sign[PITCH] = 1;
    sign[YAW] = 1;
  }

  // ******************************************************************
  // The following function calls must be defined in any new subclasses
  // ******************************************************************
  virtual void initialize(void) {
    this->_initialize(rollChannel, pitchChannel, zAxisChannel);
    smoothFactor = readFloat(ACCSMOOTH_ADR);
  }
  virtual void measure(void);
  virtual void calibrate(void);
  virtual const int getFlightData(byte);

  // **************************************************************
  // The following functions are common between all Gyro subclasses
  // **************************************************************
  void _initialize(byte rollChannel, byte pitchChannel, byte zAxisChannel) {
    accelChannel[ROLL] = rollChannel;
    accelChannel[PITCH] = pitchChannel;
    accelChannel[ZAXIS] = zAxisChannel;
    
    accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
    accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
  }
  
  const int getRaw(byte axis) {
    return accelADC[axis] * sign[axis];
  }
  
  const int getData(byte axis) {
    return accelData[axis] * sign[axis];
  }
  
  const int invert(byte axis) {
    sign[axis] = -sign[axis];
    return sign[axis];
  }
  
  const int getZero(byte axis) {
    return accelZero[axis];
  }
  
  void setZero(byte axis, int value) {
    accelZero[axis] = value;
  }
  
  const float getScaleFactor(void) {
    return accelScaleFactor;
  }
  
  const float getSmoothFactor() {
    return smoothFactor;
  }
  
  void setSmoothFactor(float value) {
    smoothFactor = value;
  }
  
  const float angleRad(byte axis) {
    if (axis == PITCH) return arctan2(accelData[PITCH] * sign[PITCH], sqrt((long(accelData[ROLL]) * accelData[ROLL]) + (long(accelData[ZAXIS]) * accelData[ZAXIS])));
    if (axis == ROLL) return arctan2(accelData[ROLL] * sign[ROLL], sqrt((long(accelData[PITCH]) * accelData[PITCH]) + (long(accelData[ZAXIS]) * accelData[ZAXIS])));
  }

  const float angleDeg(byte axis) {
    return degrees(angleRad(axis));
  }
};

/******************************************************/
/************ AeroQuad v1 Accelerometer ***************/
/******************************************************/
#if defined(AeroQuad_v1) || defined(AeroQuadMega_v1) || defined(Multipilot) || defined(MultipilotI2C)
class Accel_AeroQuad_v1 : public Accel {
private:
  int findZero[FINDZERO];
  
public:
  Accel_AeroQuad_v1() : Accel(){
    // Accelerometer Values
    // If BMA180 setup for 1G
    // Page 27 of datasheet = 0.00013g/LSB
    accelScaleFactor = 0.00013;    
  }
  
  void initialize(void) {
    // rollChannel = 1
    // pitchChannel = 0
    // zAxisChannel = 2
    this->_initialize(1, 0, 2);
    smoothFactor = readFloat(ACCSMOOTH_ADR);
  }
  
  void measure(void) {
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
      accelData[axis] = smooth(accelADC[axis], accelData[axis], smoothFactor);
    }
  }

  const int getFlightData(byte axis) {
    return getRaw(axis);
  }
  
  // Allows user to zero accelerometers on command
  void calibrate(void) {
    for (byte calAxis = ROLL; calAxis < ZAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++)
        findZero[i] = analogRead(accelChannel[calAxis]);
      accelZero[calAxis] = findMode(findZero, FINDZERO);
    }

    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
};
#endif

/******************************************************/
/********* AeroQuad Mega v2 Accelerometer *************/
/******************************************************/
#if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
class Accel_AeroQuadMega_v2 : public Accel {
private:
  int findZero[FINDZERO];
  int accelAddress;
  int data[2];
  int rawData[3];
  byte select; // use to select which axis is being read
  
public:
  Accel_AeroQuadMega_v2() : Accel(){
    accelAddress = 0x40; // page 54 and 61 of datasheet
    // Accelerometer Values
    // Update these variables if using a different accel
    // Output is ratiometric for ADXL 335
    // Note: Vs is not AREF voltage
    // If Vs = 3.6V, then output sensitivity is 360mV/g
    // If Vs = 2V, then it's 195 mV/g
    // Then if Vs = 3.3V, then it's 329.062 mV/g
    accelScaleFactor = 0.000329062;
  }
  
  void initialize(void) {
    accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
    accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
    smoothFactor = readFloat(ACCSMOOTH_ADR);
    select = PITCH;
    
    // Check if accel is connected
    if (readWhoI2C(accelAddress) != 0x03) // page 52 of datasheet
      Serial.println("Accelerometer not found!");

    // In datasheet, summary register map is page 21
    // Low pass filter settings is page 27
    // Range settings is page 28
    updateRegisterI2C(accelAddress, 0x0D, 0x10); //enable writing to control registers
    sendByteI2C(accelAddress, 0x20); // register bw_tcs (bits 4-7)
    data[0] = readByteI2C(accelAddress); // get current register value
    updateRegisterI2C(accelAddress, 0x20, data[0] & 0x0F); // set low pass filter to 10Hz (value = 0000xxxx)

    // From page 27 of BMA180 Datasheet
    //  1.0g = 0.13 mg/LSB
    //  1.5g = 0.19 mg/LSB
    //  2.0g = 0.25 mg/LSB
    //  3.0g = 0.38 mg/LSB
    //  4.0g = 0.50 mg/LSB
    //  8.0g = 0.99 mg/LSB
    // 16.0g = 1.98 mg/LSB
    sendByteI2C(accelAddress, 0x35); // register offset_lsb1 (bits 1-3)
    data[0] = readByteI2C(accelAddress);
    updateRegisterI2C(accelAddress, 0x35, data[0] & ~0x0E); // set range to +/-1g (value = xxxx000x)
  }
  
  void measure(void) {
    // round robin between each axis so that I2C blocking time is low
    if (select == ROLL) sendByteI2C(accelAddress, 0x04);
    if (select == PITCH) sendByteI2C(accelAddress, 0x02);
    if (select == ZAXIS) sendByteI2C(accelAddress, 0x06);
    rawData[select] = readReverseWordI2C(accelAddress) >> 2; // last 2 bits are not part of measurement
    accelADC[select] = rawData[select] - accelZero[select]; // reduce ADC value
    accelData[select] = smooth(accelADC[select], accelData[select], smoothFactor);
    if (++select == LASTAXIS) select = ROLL; // go to next axis, reset to ROLL if past ZAXIS
  }

  const int getFlightData(byte axis) {
    return getRaw(axis) >> 4;
  }
  
  // Allows user to zero accelerometers on command
  void calibrate(void) {  
    int dataAddress;
    
    for (byte calAxis = ROLL; calAxis < ZAXIS; calAxis++) {
      if (calAxis == ROLL) dataAddress = 0x04;
      if (calAxis == PITCH) dataAddress = 0x02;
      if (calAxis == ZAXIS) dataAddress = 0x06;
      for (int i=0; i<FINDZERO; i++) {
        sendByteI2C(accelAddress, dataAddress);
        findZero[i] = readReverseWordI2C(accelAddress) >> 2; // last two bits are not part of measurement
      }
      accelZero[calAxis] = findMode(findZero, FINDZERO);
    }

    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
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
  Accel_ArduCopter() : Accel(){
    // ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
    // ADXL335 Sensitivity(from datasheet) => 330mV/g, 0.8mV/ADC step => 330/0.8 = 412
    // Tested value : 414
    // #define GRAVITY 414 //this equivalent to 1G in the raw data coming from the accelerometer 
    // #define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square
    accelScaleFactor = 414.0 / 9.81;    
  }
  
  void initialize(void) {
    // rollChannel = 5
    // pitchChannel = 4
    // zAxisChannel = 6
    this->_initialize(5, 4, 6);
  }
  
  void measure(void) {
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      rawADC = analogRead_ArduCopter_ADC(accelChannel[axis]);
      if (rawADC > 500) // Check if measurement good
        accelADC[axis] = rawADC - accelZero[axis];
      accelData[axis] = accelADC[axis]; // no smoothing needed
    }
  }

  const int getFlightData(byte axis) {
    return getRaw(axis);
  }
  
  // Allows user to zero accelerometers on command
  void calibrate(void) {
    for(byte calAxis = 0; calAxis < ZAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        findZero[i] = analogRead_ArduCopter_ADC(accelChannel[calAxis]);
        delay(10);
      }
      accelZero[calAxis] = findMode(findZero, FINDZERO);
    }
    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
};
#endif

/******************************************************/
/****************** Wii Accelerometer *****************/
/******************************************************/
#if defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii)
class Accel_Wii : public Accel {
private:
  int findZero[FINDZERO];

public:
  Accel_Wii() : Accel(){
    accelScaleFactor = 0;    
  }
  
  void initialize(void) {
    smoothFactor = readFloat(ACCSMOOTH_ADR);
    accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
    accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
  }
  
  void measure(void) {
    // Actual measurement performed in gyro class
    // We just update the appropriate variables here
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      accelADC[axis] = NWMP_acc[axis] - accelZero[axis];
      accelData[axis] = smooth(accelADC[axis], accelData[axis], smoothFactor);
    }
  }
  
  const int getFlightData(byte axis) {
    return getRaw(axis);
  }
 
  // Allows user to zero accelerometers on command
  void calibrate(void) {
    for(byte calAxis = ROLL; calAxis < ZAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        updateControls();
        findZero[i] = NWMP_acc[calAxis];
      }
      accelZero[calAxis] = findMode(findZero, FINDZERO);
    }
    
    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
};
#endif

/******************************************************/
/************* MultiPilot Accelerometer ***************/
/******************************************************/
#if defined(Multipilot) || defined(MultipilotI2C)
class Accel_Multipilot : public Accel {
private:
  int findZero[FINDZERO];
  
public:
  Accel_Multipilot() : Accel(){
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
  
  void initialize(void) {
    // rollChannel = 6
    // pitchChannel = 7
    // zAxisChannel = 5
    this->_initialize(6, 7, 5);
    smoothFactor = readFloat(ACCSMOOTH_ADR);
  }
  
  void measure(void) {
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
      accelData[axis] = smooth(accelADC[axis], accelData[axis], smoothFactor);
    }
  }
  
  const int getFlightData(byte axis) {
    return getRaw(axis);
  }
  
  // Allows user to zero accelerometers on command
  void calibrate(void) {
    for (byte calAxis = ROLL; calAxis < ZAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++)
        findZero[i] = analogRead(accelChannel[calAxis]);
      accelZero[calAxis] = findMode(findZero, FINDZERO);
    }

    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
};
#endif
