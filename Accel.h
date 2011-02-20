/*
  AeroQuad v2.3 - February 2011
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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Accel {
public:
  float accelOneG;
  float accelScaleFactor;
  float accelVector[3];
  float smoothFactor;
  int   accelZero[3];
  int   accelRaw[3];
  
  Accel(void) {
    accelOneG        = readFloat(ACCEL1G_ADR);
    accelZero[XAXIS] = readFloat(LEVELPITCHCAL_ADR);
    accelZero[YAXIS] = readFloat(LEVELROLLCAL_ADR);
    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
    smoothFactor     = readFloat(ACCSMOOTH_ADR);
  }

  const float getData(byte axis) {
    return accelVector[axis];
  }
  
  const float getSmoothFactor() {
    return smoothFactor;
  }
  
  void setSmoothFactor(float value) {
    smoothFactor = value;
  }
  
  void setOneG(float value) {
    accelOneG = value;
  }
  
  const float getOneG(void) {
    return accelOneG;
  }
  
  const float getRaw(byte axis) {
    if (axis == XAXIS)
      return -accelRaw[PITCH] >> 3;
    if (axis == YAXIS)
      return accelRaw[ROLL] >> 3;
    if (axis == ZAXIS)
      return -accelRaw[YAW] >> 3;
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// AeroQuad Mega v2.0 Accelerometer (BMA180)
////////////////////////////////////////////////////////////////////////////////

class Accel_AeroQuadMega_v2 : public Accel {
private:
  
public:
  Accel_AeroQuadMega_v2() : Accel(){
    accelScaleFactor = G_2_MPS2(1.0/4096.0);  //  g per LSB @ +/- 2g range
  }
  
////////////////////////////////////////////////////////////////////////////////
// Initialize AeroQuad v2.0 Accelerometer
////////////////////////////////////////////////////////////////////////////////

  void initialize(void) {
    byte data;

    updateRegisterI2C(0x40, 0x10, 0xB6);         // reset device
    delay(10);                                   // sleep 10 ms after reset (page 25)

    updateRegisterI2C(0x40, 0x0D, 0x10);         // enable writing to control registers
    sendByteI2C(0x40, 0x20);                     // register bw_tcs (bits 4-7)
    data = readByteI2C(0x40);                    // get current register value
    updateRegisterI2C(0x40, 0x20, data & 0x0F);  // set low pass filter to 10Hz (value = 0000xxxx)

    sendByteI2C(0x40, 0x35);                     // Register offset_lsb1 (bits 1-3)
    data = readByteI2C(0x40);
    data &= 0xF1;                                // Clear range select bits
    data |= 0x04;                                // Set range select bits for +/-2g
    updateRegisterI2C(0x40, 0x35, data);         // (value = xxxx010x)
  }
  
////////////////////////////////////////////////////////////////////////////////
// Measure AeroQuad v2.0 Accelerometer
////////////////////////////////////////////////////////////////////////////////

  void measure(void) {

    Wire.beginTransmission(0x40);
    Wire.send(0x02);
    Wire.endTransmission();
    Wire.requestFrom(0x40, 6);

    // The following 3 lines read the accelerometer and assign it's data to accelVectorBits
    // in the correct order and phase to suit the standard shield installation
    // orientation.  See TBD for details.  If your shield is not installed in this
    // orientation, this is where you make the changes.
    accelRaw[XAXIS] = ((Wire.receive()|(Wire.receive() << 8)) >> 2) - accelZero[XAXIS];
    accelRaw[YAXIS] = accelZero[YAXIS] - ((Wire.receive()|(Wire.receive() << 8)) >> 2);
    accelRaw[ZAXIS] = accelZero[ZAXIS] - ((Wire.receive()|(Wire.receive() << 8)) >> 2);

    for (byte axis = XAXIS; axis < LASTAXIS; axis++)
      accelVector[axis] = filterSmooth(accelRaw[axis] * accelScaleFactor, accelVector[axis], smoothFactor);
  }

////////////////////////////////////////////////////////////////////////////////
// Calibrate AeroQuad v2.0 Accelerometer
////////////////////////////////////////////////////////////////////////////////

  void calibrate(void) {  
    int findZero[FINDZERO];
    int dataAddress;
    
    for (byte calAxis = XAXIS; calAxis < ZAXIS; calAxis++) {
      if (calAxis == XAXIS) dataAddress = 0x02;
      if (calAxis == YAXIS) dataAddress = 0x04;
      if (calAxis == ZAXIS) dataAddress = 0x06;
      for (int i=0; i<FINDZERO; i++) {
        sendByteI2C(0x40, dataAddress);
        findZero[i] = readReverseWordI2C(0x40) >> 2; // last two bits are not part of measurement
        delay(10);
      }
      accelZero[calAxis] = findMedian(findZero, FINDZERO);
    }

    // replace with estimated Z axis 0g value
    accelZero[ZAXIS] = (accelZero[XAXIS] + accelZero[YAXIS]) / 2;
    
    // store accel value that represents 1g
    measure();
    accelOneG = -accelVector[ZAXIS];
    
    writeFloat(accelOneG,        ACCEL1G_ADR);
    writeFloat(accelZero[XAXIS], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[YAXIS], LEVELROLLCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
  
////////////////////////////////////////////////////////////////////////////////
// Zero AeroQuad v2.0 Accelerometer
////////////////////////////////////////////////////////////////////////////////

  void zero() {
    // Not required for AeroQuad v2.0 Accelerometer
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(APM)

////////////////////////////////////////////////////////////////////////////////
// APM Accelerometer 
////////////////////////////////////////////////////////////////////////////////

class Accel_APM : public Accel {
private:

public:
  Accel_APM() : Accel(){
    accelScaleFactor = G_2_MPS2((3.3/4096) / 0.330);    
  }
  
////////////////////////////////////////////////////////////////////////////////
// Initialize APM Accelerometer
////////////////////////////////////////////////////////////////////////////////

  void initialize(void) {
    // No initialize procedure necessary, covered by APM gyro initialize
  }
  
////////////////////////////////////////////////////////////////////////////////
// Measure APM Accelerometer
////////////////////////////////////////////////////////////////////////////////

  void measure(void) {
    // The following 3 lines marked with ** read the accelerometer and assign it's 
    // data to accelRaw in the correct order and phase to suit the standard shield 
    // installation orientation.  See TBD for details.  If your shield is not
    //  installed in this orientation, this is where you make the changes.
    accelRaw[XAXIS] = readApmADC(XAXIS + 3);
    if (accelRaw[XAXIS] > 500)
      accelRaw[XAXIS] = accelRaw[XAXIS] - accelZero[XAXIS];  // **
      
    accelRaw[YAXIS] = readApmADC(YAXIS + 3);
    if (accelRaw[YAXIS] > 500)
      accelRaw[YAXIS] = accelZero[YAXIS] - accelRaw[YAXIS];  // **
      
    accelRaw[ZAXIS] = readApmADC(ZAXIS + 3);
    if (accelRaw[ZAXIS] > 500)
      accelRaw[ZAXIS] = accelZero[ZAXIS] - accelRaw[ZAXIS];  // **
    
    for (byte axis = XAXIS; axis < LASTAXIS; axis++)
      accelVector[axis] = smooth(accelRaw[axis] * accelScaleFactor, accelVector[axis], smoothFactor);
  }

////////////////////////////////////////////////////////////////////////////////
// Calibrate APM Accelerometer
////////////////////////////////////////////////////////////////////////////////

  void calibrate(void) {
    int findZero[FINDZERO];
    
    for(byte calAxis = 0; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        findZero[i] = readApmADC(calAxis + 3);
        delay(10);
      }
      accelZero[calAxis] = findMode(findZero, FINDZERO);
    }

    // replace with estimated Z axis 0g value
    accelZero[ZAXIS] = (accelZero[XAXIS] + accelZero[YAXIS]) / 2;
    
    // store accel value that represents 1g
    measure();
    accelOneG = -accelVector[ZAXIS];

    writeFloat(accelOneG,        ACCEL1G_ADR);
    writeFloat(accelZero[XAXIS], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[YAXIS], LEVELROLLCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
  
////////////////////////////////////////////////////////////////////////////////
// Zero APM Accelerometer
////////////////////////////////////////////////////////////////////////////////

  void zero() {
    for (byte n = 3; n < 6; n++) {
      adc_value[n] = 0;
      adc_counter[n] = 0;
    }
  }
};

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Wii Accelerometer
////////////////////////////////////////////////////////////////////////////////
#if defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii)
class Accel_Wii : public Accel {
private:

public:
  Accel_Wii() : Accel(){
    accelScaleFactor = 0.09165093;  // Experimentally derived to produce meters/s^2    
  }

////////////////////////////////////////////////////////////////////////////////
// Initialize Wii Accelerometer
////////////////////////////////////////////////////////////////////////////////

  void initialize(void) {
    // No initialize procedure necessary, covered by Wii gyro initialize
  };

////////////////////////////////////////////////////////////////////////////////
// Measure Wii Accelerometer
////////////////////////////////////////////////////////////////////////////////

  void measure(void) {
    // No measure procedure necessary, covered by Wii gyro measure
  };

////////////////////////////////////////////////////////////////////////////////
// Calibrate Wii Accelerometer
////////////////////////////////////////////////////////////////////////////////

  void calibrate(void) {
    int findZero[FINDZERO];

    for(byte calAxis = XAXIS; calAxis < LASTAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        readWii(0);
        findZero[i] = accelRaw[calAxis];
        delay(10);
      }
      accelZero[calAxis] = findMode(findZero, FINDZERO);
    }
    
    // replace with estimated Z axis 0g value
    accelZero[ZAXIS] = (accelZero[XAXIS] + accelZero[YAXIS]) / 2;
    
    // store accel value that represents 1g
    readWii(1);
    accelOneG = -accelVector[ZAXIS];
    
    writeFloat(accelOneG,        ACCEL1G_ADR);
    writeFloat(accelZero[XAXIS], LEVELPITCHCAL_ADR);
    writeFloat(accelZero[YAXIS], LEVELROLLCAL_ADR);
    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
  }
  
////////////////////////////////////////////////////////////////////////////////
// Zero Wi Accelerometer
////////////////////////////////////////////////////////////////////////////////

  void zero() {
    // Not required for Wii Accelerometer
  }
};
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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
    //currentTime = micros();
      accelADC[XAXIS] = chr6dm.data.ax - accelZero[XAXIS];
      accelADC[YAXIS] = chr6dm.data.ay - accelZero[YAXIS];
      accelADC[ZAXIS] = chr6dm.data.az - accelOneG;

      //accelData[XAXIS] = filterSmoothWithTime(accelADC[XAXIS], accelData[XAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0)); //to get around 1
      //accelData[YAXIS] = filterSmoothWithTime(accelADC[YAXIS], accelData[YAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
      //accelData[ZAXIS] = filterSmoothWithTime(accelADC[ZAXIS], accelData[ZAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
      accelData[XAXIS] = filterSmooth(accelADC[XAXIS], accelData[XAXIS], smoothFactor); //to get around 1
      accelData[YAXIS] = filterSmooth(accelADC[YAXIS], accelData[YAXIS], smoothFactor);
      accelData[ZAXIS] = filterSmooth(accelADC[ZAXIS], accelData[ZAXIS], smoothFactor);

    //previousTime = currentTime;
    
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

  //void calculateAltitude() {
    //currentTime = micros();
    //if ((abs(CHR_RollAngle) < 5) && (abs(CHR_PitchAngle) < 5)) 
      //rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    //previousTime = currentTime;
  //} 
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
    //currentTime = micros();
      //read done in gyro   //TODO
      accelADC[XAXIS] = fakeAccelRoll - accelZero[XAXIS];
      accelADC[YAXIS] = fakeAccelPitch - accelZero[YAXIS];
      accelADC[ZAXIS] = fakeAccelYaw - accelOneG;

      //accelData[XAXIS] = smoothWithTime(accelADC[XAXIS], accelData[XAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
      //accelData[YAXIS] = smoothWithTime(accelADC[YAXIS], accelData[YAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
      //accelData[ZAXIS] = smoothWithTime(accelADC[ZAXIS], accelData[ZAXIS], smoothFactor, ((currentTime - previousTime) / 5000.0));
      
      accelData[XAXIS] = filterSmooth(accelADC[XAXIS], accelData[XAXIS], smoothFactor);
      accelData[YAXIS] = filterSmooth(accelADC[YAXIS], accelData[YAXIS], smoothFactor);
      accelData[ZAXIS] = filterSmooth(accelADC[ZAXIS], accelData[ZAXIS], smoothFactor);
      
      
    //previousTime = currentTime;
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

  //void calculateAltitude() {
    //currentTime = micros();
    //if ((abs(CHR_RollAngle) < 5) && (abs(CHR_PitchAngle) < 5)) 
      //rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    //previousTime = currentTime;
  //} 
};
#endif

/******************************************************/
/************* MultiPilot Accelerometer ***************/
/******************************************************/
#if defined(Multipilot) || defined(MultipilotI2C)
class Accel_Multipilot : public Accel {
private:
  
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
    //currentTime = micros();
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
      //accelData[axis] = filterSmoothWithTime(accelADC[axis], accelData[axis], smoothFactor, ((currentTime - previousTime) / 5000.0));
      accelData[axis] = filterSmooth(accelADC[axis], accelData[axis], smoothFactor);
    }
    //previousTime = currentTime;
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

  //void calculateAltitude() {
    //currentTime = micros();
    //if ((abs(getRaw(ROLL)) < 1500) && (abs(getRaw(PITCH)) < 1500)) 
      //rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    //previousTime = currentTime;
  //} 
};
#endif
