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

class Accel {
public:
  float accelScaleFactor;
  float smoothFactor;
  float rawAltitude;
  int accelChannel[3];
  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    float accelZero[3];
  #else
    int accelZero[3];
  #endif
  float accelData[3];
  int accelADC[3];
  int sign[3];
  float accelOneG, zAxis;
  byte rollChannel, pitchChannel, zAxisChannel;
  //unsigned long currentAccelTime, previousAccelTime;  // AKA changes to remove total Time from Honks smoothing changes
  
  Accel(void) {
    sign[ROLL] = 1;
    sign[PITCH] = 1;
    sign[YAW] = 1;
    zAxis = 0;
  }

  // ******************************************************************
  // The following function calls must be defined in any new subclasses
  // ******************************************************************
  virtual void initialize(void) {
    this->_initialize(rollChannel, pitchChannel, zAxisChannel);
  }
  virtual void measure(void);
  virtual void calibrate(void);
  virtual const int getFlightData(byte);
  //virtual void calculateAltitude(void); // AKA changes to remove total Time from Honks smoothing changes

  // **************************************************************
  // The following functions are common between all Gyro subclasses
  // **************************************************************
  void _initialize(byte rollChannel, byte pitchChannel, byte zAxisChannel) {
    accelChannel[ROLL] = rollChannel;
    accelChannel[PITCH] = pitchChannel;
    accelChannel[ZAXIS] = zAxisChannel;
    accelOneG        = readFloat(ACCEL1G_ADR);
    accelZero[XAXIS] = readFloat(LEVELPITCHCAL_ADR);
    accelZero[YAXIS] = readFloat(LEVELROLLCAL_ADR);
    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
    //smoothFactor     = readFloat(ACCSMOOTH_ADR);
    //currentAccelTime = micros(); // AKA changes to remove total Time from Honks smoothing changes
    //previousAccelTime = currentAccelTime;    // AKA changes to remove total Time from Honks smoothing changes
  }
  
  // return the raw ADC value from the accel, with sign change if need, not smoothed or scaled to SI units
  const int getRaw(byte axis) {
    return accelADC[axis] * sign[axis];
  }
  
  // return the smoothed and scaled to SI units value of the accel with sign change if needed
  const float getData(byte axis) {
    return accelData[axis] * sign[axis];
  }
  
  // invert the sign for a specifica accel axis
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
  
  // returns the SI scale factor
  const float getScaleFactor(void) {
    return accelScaleFactor;
  }
  
  // returns the smoothfactor
  const float getSmoothFactor() {
    return smoothFactor;
  }
  
  void setSmoothFactor(float value) {
    smoothFactor = value;
  }
  
/* AKA commented out and not used current, may not be correct with SI changes  
  const float angleRad(byte axis) {
    if (axis == PITCH) return arctan2(accelData[PITCH] * sign[PITCH], sqrt((long(accelData[ROLL]) * accelData[ROLL]) + (long(accelData[ZAXIS]) * accelData[ZAXIS])));
    if (axis == ROLL) return arctan2(accelData[ROLL] * sign[ROLL], sqrt((long(accelData[PITCH]) * accelData[PITCH]) + (long(accelData[ZAXIS]) * accelData[ZAXIS])));
  }

  const float angleDeg(byte axis) {
    return degrees(angleRad(axis));
  }
*/  
  
  void setOneG(float value) {
    accelOneG = value;
  }
  
  const float getOneG(void) {
    return accelOneG;
  }
  
  const float getZaxis() {
    //currentAccelTime = micros();
    //zAxis = filterSmoothWithTime(getFlightData(ZAXIS), zAxis, 0.25, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
    //previousAccelTime = currentAccelTime;
    //return zAxis;
    return accelOneG - getData(ZAXIS);
  }
/*  AKA not used
  const float getAltitude(void) {
    return rawAltitude;
  }
  
  const float rateG(const byte axis) {
    return getData(axis) / accelOneG;
  }
*/  
};

/******************************************************/
/************ AeroQuad v1 Accelerometer ***************/
/******************************************************/
#if defined(AeroQuad_v1) || defined(AeroQuad_v1_IDG) || defined(AeroQuadMega_v1)
class Accel_AeroQuad_v1 : public Accel {
private:
  
public:
  Accel_AeroQuad_v1() : Accel(){
    // Accelerometer Values
    // Update these variables if using a different accel
    // Output is ratiometric for ADXL 335
    // Note: Vs is not AREF voltage
    // If Vs = 3.6V, then output sensitivity is 360mV/g
    // If Vs = 2V, then it's 195 mV/g
    // Then if Vs = 3.3V, then it's 329.062 mV/g
    //accelScaleFactor = 0.000329062;
    accelScaleFactor = G_2_MPS2((3.3/4096) / 0.000329062);
  }
  
  void initialize(void) {
    // rollChannel = 1
    // pitchChannel = 0
    // zAxisChannel = 2
    this->_initialize(1, 0, 2);
    smoothFactor     = readFloat(ACCSMOOTH_ADR);
  }
  
  void measure(void) {
    //currentTime = micros(); // AKA changes to remove total Time from Honks smoothing changes
    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
      accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
      accelData[axis] = filterSmooth(accelADC[axis] * accelScaleFactor, accelData[axis], smoothFactor);
    }
  }

  const int getFlightData(byte axis) {
    return getRaw(axis);
  }
  
  // Allows user to zero accelerometers on command
  void calibrate(void) {
    int findZero[FINDZERO];

    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++)
        findZero[i] = analogRead(accelChannel[calAxis]);
      accelZero[calAxis] = findMedian(findZero, FINDZERO);
    }
    
    // store accel value that represents 1g
    accelOneG = accelZero[ZAXIS];
    // replace with estimated Z axis 0g value
    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    
    writeFloat(accelOneG, ACCEL1G_ADR);
    writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
    writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }

  /* // AKA - NOT USED
  void calculateAltitude() {
    //currentTime = micros();
    if ((abs(getRaw(ROLL)) < 1500) && (abs(getRaw(PITCH)) < 1500)) 
      rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    //previousTime = currentTime;
  } 
  */
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
  Accel_AeroQuadMega_v2() : Accel(){
    accelAddress = 0x40; // page 54 and 61 of datasheet
    accelScaleFactor = G_2_MPS2(1.0/4096.0);  //  g per LSB @ +/- 2g range
  }
  
  void initialize(void) {
    byte data;
    
    this->_initialize(0,1,2);  // AKA added for consistency
  
    accelOneG        = readFloat(ACCEL1G_ADR);
    accelZero[XAXIS] = readFloat(LEVELPITCHCAL_ADR);
    accelZero[YAXIS] = readFloat(LEVELROLLCAL_ADR);
    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
    smoothFactor     = readFloat(ACCSMOOTH_ADR);
    
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
    data &= 0xF1;
    data |= 0x04; // Set range select bits for +/-2g
    updateRegisterI2C(accelAddress, 0x35, data);
  }
  
  void measure(void) {
    //int rawData[3];

    Wire.beginTransmission(accelAddress);
    Wire.send(0x02);
    Wire.endTransmission();
    Wire.requestFrom(accelAddress, 6);
    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
      if (axis == XAXIS)
        accelADC[axis] = ((Wire.receive()|(Wire.receive() << 8)) >> 2) - accelZero[axis];
      else
        accelADC[axis] = accelZero[axis] - ((Wire.receive()|(Wire.receive() << 8)) >> 2);
      accelData[axis] = filterSmooth(accelADC[axis] * accelScaleFactor, accelData[axis], smoothFactor);
    }
  }

  const int getFlightData(byte axis) {
      return getRaw(axis) >> 3;
  }
  
  // Allows user to zero accelerometers on command
  void calibrate(void) {  
    int findZero[FINDZERO];
    int dataAddress;
    
    for (byte calAxis = XAXIS; calAxis < ZAXIS; calAxis++) {
      if (calAxis == XAXIS) dataAddress = 0x02;
      if (calAxis == YAXIS) dataAddress = 0x04;
      if (calAxis == ZAXIS) dataAddress = 0x06;
      for (int i=0; i<FINDZERO; i++) {
        sendByteI2C(accelAddress, dataAddress);
        findZero[i] = readReverseWordI2C(accelAddress) >> 2; // last two bits are not part of measurement
        delay(10);
      }
      accelZero[calAxis] = findMedian(findZero, FINDZERO);
    }

    // replace with estimated Z axis 0g value
    accelZero[ZAXIS] = (accelZero[XAXIS] + accelZero[PITCH]) / 2;
    // store accel value that represents 1g
    measure();
    accelOneG = -accelData[ZAXIS];
     
    writeFloat(accelOneG,        ACCEL1G_ADR);
    writeFloat(accelZero[XAXIS], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[YAXIS], LEVELROLLCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }

/* AKA - NOT USED
  void calculateAltitude() {
    currentTime = micros();
    if ((abs(getRaw(XAXIS)) < 1500) && (abs(getRaw(YAXIS)) < 1500)) 
      rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    previousTime = currentTime;
  } 
*/  
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
    accelScaleFactor = G_2_MPS2((3.3/4096) / 0.330);    
  }
  
  void initialize(void) {
    // old AQ way
    // rollChannel = 5
    // pitchChannel = 4
    // zAxisChannel = 6
    // new way in 2.3
    // rollChannel = 3
    // pitchChannel = 4
    // zAxisChannel = 5
    this->_initialize(3, 4, 5);
    smoothFactor     = readFloat(ACCSMOOTH_ADR);
  }
  
  void measure(void) {
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      rawADC = analogRead_ArduCopter_ADC(accelChannel[axis]);
      if (rawADC > 500) // Check if measurement good
        if (axis == ROLL)
          accelADC[axis] = rawADC - accelZero[axis];
       else
          accelADC[axis] = accelZero[axis] - rawADC;
      accelData[axis] = filterSmooth(accelADC[axis] * accelScaleFactor, accelData[axis], smoothFactor);
    }
  }

  const int getFlightData(byte axis) {
      return getRaw(axis);
  }
  
  // Allows user to zero accelerometers on command
  void calibrate(void) {
    for(byte calAxis = XAXIS; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        findZero[i] = analogRead_ArduCopter_ADC(accelChannel[calAxis]);
        delay(2);
      }
      accelZero[calAxis] = findMedian(findZero, FINDZERO);
    }

    //accelOneG = 486;    // tested value with the configurator at flat level
    // replace with estimated Z axis 0g value
    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
   
    // store accel value that represents 1g
    measure();
    accelOneG = -accelData[ZAXIS];

    writeFloat(accelOneG,        ACCEL1G_ADR);
    writeFloat(accelZero[XAXIS], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[YAXIS], LEVELROLLCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
/* AKA - NOT USED
  void calculateAltitude() {
    currentTime = micros();
    if ((abs(getRaw(XAXIS)) < 1500) && (abs(getRaw(YAXIS)) < 1500)) 
      rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    previousTime = currentTime;
  } 
*/  
};
#endif

/******************************************************/
/****************** Wii Accelerometer *****************/
/******************************************************/
#if defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii)
class Accel_Wii : public Accel {
public:
  Accel_Wii() : Accel(){
    accelScaleFactor = 0.09165093;  // Experimentally derived to produce meters/s^2    
  }
  
  void initialize(void) {
    accelOneG        = readFloat(ACCEL1G_ADR);
    accelZero[XAXIS] = readFloat(LEVELPITCHCAL_ADR);
    accelZero[YAXIS] = readFloat(LEVELROLLCAL_ADR);
    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
    smoothFactor     = readFloat(ACCSMOOTH_ADR);
  }
  
  void measure(void) {
    //currentTime = micros(); // AKA changes to remove total Time from Honks smoothing changes
    // Actual measurement performed in gyro class
    // We just update the appropriate variables here
    // Depending on how your accel is mounted, you can change X/Y axis to pitch/roll mapping here
    accelADC[XAXIS] = NWMP_acc[PITCH] - accelZero[PITCH];
    accelADC[YAXIS] = NWMP_acc[ROLL] - accelZero[ROLL];
    accelADC[ZAXIS] = accelZero[ZAXIS] - NWMP_acc[ZAXIS];
    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
      //accelData[axis] = filterSmoothWithTime(accelADC[axis] * accelScaleFactor, accelData[axis], smoothFactor, ((currentTime - previousTime) / 5000.0));  // AKA changes to remove total Time from Honks smoothing changes
      accelData[axis] = filterSmooth(accelADC[axis] * accelScaleFactor, accelData[axis], smoothFactor);
    }
    //previousTime = currentTime;  // AKA changes to remove total Time from Honks smoothing changes  
  }
  
  const int getFlightData(byte axis) {
    if (axis == ROLL)
      return -getRaw(YAXIS);
    if (axis == PITCH)
      return -getRaw(XAXIS);
    if (axis == ZAXIS)
      return -getRaw(ZAXIS);
  }
 
  // Allows user to zero accelerometers on command
  void calibrate(void) {
    int findZero[FINDZERO];

    for(byte calAxis = XAXIS; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        updateControls();
        findZero[i] = NWMP_acc[calAxis];
      }
      accelZero[calAxis] = findMedian(findZero, FINDZERO);
    }
    
    // store accel value that represents 1g
    accelOneG = -accelData[ZAXIS];
    // replace with estimated Z axis 0g value
    accelZero[ZAXIS] = (accelZero[XAXIS] + accelZero[YAXIS]) / 2;
    
    writeFloat(accelOneG, ACCEL1G_ADR);
    writeFloat(accelZero[XAXIS], LEVELROLLCAL_ADR);
    writeFloat(accelZero[YAXIS], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
/*  AKA - NOT USED
  void calculateAltitude() {
    currentTime = micros();
    if ((abs(getRaw(ROLL)) < 1500) && (abs(getRaw(PITCH)) < 1500)) 
      rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    previousTime = currentTime;
  } 
*/  
};
#endif

/******************************************************/
/****************** CHR6DM Accelerometer **************/
/******************************************************/
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
class Accel_CHR6DM : public Accel {
public:
  Accel_CHR6DM() : Accel() {
    accelScaleFactor = 0;
  }

  void initialize(void) {
    smoothFactor = readFloat(ACCSMOOTH_ADR);
    accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
    accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
    accelOneG = readFloat(ACCEL1G_ADR);
    calibrate();
  }

  void measure(void) {
    //currentTime = micros(); // AKA removed as a result of Honks original work, not needed further
      accelADC[XAXIS] = chr6dm.data.ax - accelZero[XAXIS];
      accelADC[YAXIS] = chr6dm.data.ay - accelZero[YAXIS];
      accelADC[ZAXIS] = chr6dm.data.az - accelOneG;

      //accelData[XAXIS] = filterSmoothWithTime(accelADC[XAXIS], accelData[XAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0)); //to get around 1
      //accelData[YAXIS] = filterSmoothWithTime(accelADC[YAXIS], accelData[YAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
      //accelData[ZAXIS] = filterSmoothWithTime(accelADC[ZAXIS], accelData[ZAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
      accelData[XAXIS] = filterSmooth(accelADC[XAXIS], accelData[XAXIS], smoothFactor); //to get around 1
      accelData[YAXIS] = filterSmooth(accelADC[YAXIS], accelData[YAXIS], smoothFactor);
      accelData[ZAXIS] = filterSmooth(accelADC[ZAXIS], accelData[ZAXIS], smoothFactor);

    //previousTime = currentTime; // AKA removed as a result of Honks original work, not needed further
  }    

  const int getFlightData(byte axis) {
    return getRaw(axis);
  }

  // Allows user to zero accelerometers on command
  void calibrate(void) {

   float zeroXreads[FINDZERO];
   float zeroYreads[FINDZERO];
   float zeroZreads[FINDZERO];


    for (int i=0; i<FINDZERO; i++) {
        chr6dm.requestAndReadPacket();
        zeroXreads[i] = chr6dm.data.ax;
        zeroYreads[i] = chr6dm.data.ay;
        zeroZreads[i] = chr6dm.data.az;
    }


    accelZero[XAXIS] = findMedian(zeroXreads, FINDZERO);
    accelZero[YAXIS] = findMedian(zeroYreads, FINDZERO);
    accelZero[ZAXIS] = findMedian(zeroZreads, FINDZERO);
   
    // store accel value that represents 1g
    accelOneG = accelZero[ZAXIS];
    // replace with estimated Z axis 0g value
    //accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;

    writeFloat(accelOneG, ACCEL1G_ADR);
    writeFloat(accelZero[XAXIS], LEVELROLLCAL_ADR);
    writeFloat(accelZero[YAXIS], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
/*  AKA - NOT USED
  void calculateAltitude() {
    currentTime = micros();
    if ((abs(CHR_RollAngle) < 5) && (abs(CHR_PitchAngle) < 5)) 
      rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    previousTime = currentTime;
  } 
*/  
};
#endif

/********************************************/
/******** CHR6DM Fake Accelerometer *********/
/********************************************/
#ifdef CHR6DM_FAKE_ACCEL
class Accel_CHR6DM_Fake : public Accel {
public:
  float fakeAccelRoll;
  float fakeAccelPitch;
  float fakeAccelYaw;
  Accel_CHR6DM_Fake() : Accel() {
    accelScaleFactor = 0;
  }

  void initialize(void) {
    smoothFactor = readFloat(ACCSMOOTH_ADR);
    accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
    accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
     accelZero[ROLL] = 0;
        accelZero[PITCH] = 0;
        accelZero[ZAXIS] = 0;

    accelOneG = readFloat(ACCEL1G_ADR);
    calibrate();
  }

  void measure(void) {
    //currentTime = micros(); // AKA removed as a result of Honks original work, not needed further
      //read done in gyro   //TODO
      accelADC[XAXIS] = fakeAccelRoll - accelZero[XAXIS];
      accelADC[YAXIS] = fakeAccelPitch - accelZero[YAXIS];
      accelADC[ZAXIS] = fakeAccelYaw - accelOneG;

      //accelData[XAXIS] = smoothWithTime(accelADC[XAXIS], accelData[XAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
      //accelData[YAXIS] = smoothWithTime(accelADC[YAXIS], accelData[YAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
      //accelData[ZAXIS] = smoothWithTime(accelADC[ZAXIS], accelData[ZAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
      accelData[XAXIS] = smooth(accelADC[XAXIS], accelData[XAXIS], smoothFactor);
      accelData[YAXIS] = smooth(accelADC[YAXIS], accelData[YAXIS], smoothFactor);
      accelData[ZAXIS] = smooth(accelADC[ZAXIS], accelData[ZAXIS], smoothFactor);

    //previousTime = currentTime; // AKA removed as a result of Honks original work, not needed further
  }
  
  const int getFlightData(byte axis) {
    return getRaw(axis);
  }

  // Allows user to zero accelerometers on command
  void calibrate(void) {

   float zeroXreads[FINDZERO];
   float zeroYreads[FINDZERO];
   float zeroZreads[FINDZERO];


    for (int i=0; i<FINDZERO; i++) {
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

    writeFloat(accelOneG, ACCEL1G_ADR);
    writeFloat(accelZero[XAXIS], LEVELROLLCAL_ADR);
    writeFloat(accelZero[YAXIS], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
/* AKA - NOT USED
  void calculateAltitude() {
    currentTime = micros();
    if ((abs(CHR_RollAngle) < 5) && (abs(CHR_PitchAngle) < 5)) 
      rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    previousTime = currentTime;
  } 
*/  
};
#endif
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               