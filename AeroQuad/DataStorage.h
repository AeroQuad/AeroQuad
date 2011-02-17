/*
  AeroQuad v2.1 - January 2011
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

// Special thanks for 1k space optimization update from Ala42
// http://aeroquad.com/showthread.php?1369-The-big-enhancement-addition-to-2.0-code&p=13359&viewfull=1#post13359

// Utilities for writing and reading from the EEPROM
float readFloat(int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatOut;

  for (int i = 0; i < 4; i++)
    floatOut.floatByte[i] = EEPROM.read(address + i);
  return floatOut.floatVal;
}

void writeFloat(float value, int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatIn;

  floatIn.floatVal = value;
  for (int i = 0; i < 4; i++)
    EEPROM.write(address + i, floatIn.floatByte[i]);
}

void readPID(unsigned char IDPid, unsigned int IDEeprom) {
  struct PIDdata* pid = &PID[IDPid];
  pid->P = readFloat(IDEeprom);
  pid->I = readFloat(IDEeprom+4);
  pid->D = readFloat(IDEeprom+8);
  pid->lastPosition = 0;
  pid->integratedError = 0;
}

void writePID(unsigned char IDPid, unsigned int IDEeprom) {
  struct PIDdata* pid = &PID[IDPid];
  writeFloat(pid->P, IDEeprom);
  writeFloat(pid->I, IDEeprom+4);
  writeFloat(pid->D, IDEeprom+8);
}

// contains all default values when re-writing EEPROM
void initializeEEPROM(void) {
  PID[ROLL].P = 1.20;
  PID[ROLL].I = 0.0;
  PID[ROLL].D = -0.05;
  PID[PITCH].P = 1.20;
  PID[PITCH].I = 0.0;
  PID[PITCH].D = -0.05;
  PID[YAW].P = 3.0;
  PID[YAW].I = 0.05;
  PID[YAW].D = 0.0;
  PID[LEVELROLL].P = 7.0;
  PID[LEVELROLL].I = 1.0;
  PID[LEVELROLL].D = 0.0;
  PID[LEVELPITCH].P = 7.0;
  PID[LEVELPITCH].I = 1.0;
  PID[LEVELPITCH].D = 0.0;
  PID[HEADING].P = 3.0;
  PID[HEADING].I = 0.1;
  PID[HEADING].D = 0.0;
  PID[LEVELGYROROLL].P = 1.2;
  PID[LEVELGYROROLL].I = 0.0;
  PID[LEVELGYROROLL].D = -0.05;
  PID[LEVELGYROPITCH].P = 1.2;
  PID[LEVELGYROPITCH].I = 0.0;
  PID[LEVELGYROPITCH].D = -0.05;
  #ifdef AltitudeHold
    PID[ALTITUDE].P = 25.0;
    PID[ALTITUDE].I = 0.1;
    PID[ALTITUDE].D = 0.0;
    PID[ALTITUDE].windupGuard = 25.0; //this prevents the 0.1 I term to rise too far
    PID[ZDAMPENING].P = 0.0;
    PID[ZDAMPENING].I = 0.0;
    PID[ZDAMPENING].D = 0.0;
    minThrottleAdjust = -50.0;
    maxThrottleAdjust = 50.0; //we don't want it to be able to take over totally
    altitude.setSmoothFactor(0.1);
  #endif
  #ifdef COMPASS_INSTALLED
    compass.setMagCal(XAXIS, 1, 0);
    compass.setMagCal(YAXIS, 1, 0);
    compass.setMagCal(ZAXIS, 1, 0);
  #endif
  windupGuard = 1000.0;
  receiver.setXmitFactor(0.50);
  levelLimit = 500.0;
  levelOff = 150.0;
  rateGyro.setSmoothFactor(1.0);
  accel.setSmoothFactor(1.0);
  accel.setOneG(9.80665);
  timeConstant = 7.0;
  for (byte channel = ROLL; channel < LASTCHANNEL; channel++) {
    receiver.setTransmitterSlope(channel, 1.0);
    receiver.setTransmitterOffset(channel, 0.0);
    receiver.setSmoothFactor(channel, 1.0);
  }
  receiver.setSmoothFactor(YAW, 0.5);

  smoothHeading = 1.0;
  flightMode = ACRO;
  headingHoldConfig = OFF;
  minAcro = 1300;
  aref = 5.0; // Use 3.0 if using a v1.7 shield or use 2.8 for an AeroQuad Shield < v1.7
  
  /*#ifdef Camera
    mCameraPitch = 11.11;   // scale angle to servo....  caculated as +/- 90 (ie 180) degrees maped to 1000-2000 
    mCameraRoll = 11.11;        
    mCameraYaw = 11.11;
    centerPitch = 1500;       // (bCamera) Center of stabilisation in mode 1,  point here in mode 2  
    centerRoll = 1500;        // 1000 - 2000 nornaly centered 1500
    centerYaw = 1500;  
    servoMinPitch = 1000;     // don't drive the servo past here  
    servoMinRoll = 1000;
    servoMinYaw = 1000;
    servoMaxPitch = 2000;
    servoMaxRoll = 2000;
    servoMaxYaw = 2000;
  #endif*/
}

void readEEPROM(void) {
  readPID(ROLL, ROLL_PID_GAIN_ADR);
  readPID(PITCH, PITCH_PID_GAIN_ADR);
  readPID(YAW, YAW_PID_GAIN_ADR);
  readPID(LEVELROLL, LEVELROLL_PID_GAIN_ADR);
  readPID(LEVELPITCH, LEVELPITCH_PID_GAIN_ADR);
  readPID(HEADING, HEADING_PID_GAIN_ADR);
  readPID(LEVELGYROROLL, LEVEL_GYRO_ROLL_PID_GAIN_ADR);
  readPID(LEVELGYROPITCH, LEVEL_GYRO_PITCH_PID_GAIN_ADR);

  #ifdef AltitudeHold
    // Leaving separate PID reads as commented for now
    // Previously had issue where EEPROM was not reading right data
    readPID(ALTITUDE, ALTITUDE_PGAIN_ADR);
    PID[ALTITUDE].windupGuard = readFloat(ALTITUDE_WINDUP_ADR);
    readPID(ZDAMPENING, ZDAMP_PGAIN_ADR);
    minThrottleAdjust = readFloat(ALTITUDE_MIN_THROTTLE_ADR);
    maxThrottleAdjust = readFloat(ALTITUDE_MAX_THROTTLE_ADR);
    altitude.setSmoothFactor(readFloat(ALTITUDE_SMOOTH_ADR));
  #endif

  #ifdef COMPASS_INSTALLED
    compass.setMagCal(XAXIS, readFloat(MAGXMAX_ADR), readFloat(MAGXMIN_ADR));
    compass.setMagCal(YAXIS, readFloat(MAGYMAX_ADR), readFloat(MAGYMIN_ADR));
    compass.setMagCal(ZAXIS, readFloat(MAGZMAX_ADR), readFloat(MAGZMIN_ADR));
  #endif

  windupGuard = readFloat(WINDUPGUARD_ADR);
  levelLimit = readFloat(LEVELLIMIT_ADR);
  levelOff = readFloat(LEVELOFF_ADR);
  timeConstant = readFloat(FILTERTERM_ADR);
  smoothHeading = readFloat(HEADINGSMOOTH_ADR);
  aref = readFloat(AREF_ADR);
  flightMode = readFloat(FLIGHTMODE_ADR);
  headingHoldConfig = readFloat(HEADINGHOLD_ADR);
  minAcro = readFloat(MINACRO_ADR);
  
  /*#ifdef Camera
  mCameraPitch = readFloat(MCAMERAPITCH_ADR);
  mCameraRoll = readFloat(MCAMERAROLL_ADR);
  mCameraYaw = readFloat(MCAMERAYAW_ADR);
  centerPitch = readFloat(CENTERPITCH_ADR);
  centerRoll = readFloat(CENTERROLL_ADR);
  centerYaw = readFloat(CENTERYAW_ADR);
  servoMinPitch = readFloat(SERVOMINPITCH_ADR);
  servoMinRoll = readFloat(SERVOMINROLL_ADR);
  servoMinYaw = readFloat(SERVOMINYAW_ADR);
  servoMaxPitch = readFloat(SERVOMAXPITCH_ADR);
  servoMaxRoll = readFloat(SERVOMAXROLL_ADR);
  servoMaxYaw = readFloat(SERVOMAXYAW_ADR);
  #endif*/
}

void writeEEPROM(void){
  cli(); // Needed so that APM sensor data doesn't overflow
  writePID(ROLL, ROLL_PID_GAIN_ADR);
  writePID(PITCH, PITCH_PID_GAIN_ADR);
  writePID(LEVELROLL, LEVELROLL_PID_GAIN_ADR);
  writePID(LEVELPITCH, LEVELPITCH_PID_GAIN_ADR);
  writePID(YAW, YAW_PID_GAIN_ADR);
  writePID(HEADING, HEADING_PID_GAIN_ADR);
  writePID(LEVELGYROROLL, LEVEL_GYRO_ROLL_PID_GAIN_ADR);
  writePID(LEVELGYROPITCH, LEVEL_GYRO_PITCH_PID_GAIN_ADR);
  #ifdef AltitudeHold
    writePID(ALTITUDE, ALTITUDE_PGAIN_ADR);
    writeFloat(PID[ALTITUDE].windupGuard, ALTITUDE_WINDUP_ADR);
    writePID(ZDAMPENING, ZDAMP_PGAIN_ADR);
    writeFloat(minThrottleAdjust, ALTITUDE_MIN_THROTTLE_ADR);
    writeFloat(maxThrottleAdjust, ALTITUDE_MAX_THROTTLE_ADR);
    writeFloat(altitude.getSmoothFactor(), ALTITUDE_SMOOTH_ADR);
  #endif
  #ifdef COMPASS_INSTALLED
    writeFloat(compass.getMagMax(XAXIS), MAGXMAX_ADR);
    writeFloat(compass.getMagMin(XAXIS), MAGXMIN_ADR);
    writeFloat(compass.getMagMax(YAXIS), MAGYMAX_ADR);
    writeFloat(compass.getMagMin(YAXIS), MAGYMIN_ADR);
    writeFloat(compass.getMagMax(ZAXIS), MAGZMAX_ADR);
    writeFloat(compass.getMagMin(ZAXIS), MAGZMIN_ADR);
  #endif
  writeFloat(windupGuard, WINDUPGUARD_ADR);
  writeFloat(levelLimit, LEVELLIMIT_ADR);
  writeFloat(levelOff, LEVELOFF_ADR);
  writeFloat(receiver.getXmitFactor(), XMITFACTOR_ADR);
  writeFloat(rateGyro.getSmoothFactor(), GYROSMOOTH_ADR);
  writeFloat(accel.getSmoothFactor(), ACCSMOOTH_ADR);
  writeFloat(timeConstant, FILTERTERM_ADR);

  for(byte channel = ROLL; channel < LASTCHANNEL; channel++) {
    byte offset = 12*channel + NVM_TRANSMITTER_SCALE_OFFSET_SMOOTH;
    writeFloat(receiver.getTransmitterSlope(channel),  offset+0);
    writeFloat(receiver.getTransmitterOffset(channel), offset+4);
    writeFloat(receiver.getSmoothFactor(channel),      offset+8);
  }

  writeFloat(smoothHeading, HEADINGSMOOTH_ADR);
  writeFloat(aref, AREF_ADR);
  writeFloat(flightMode, FLIGHTMODE_ADR);
  writeFloat(headingHoldConfig, HEADINGHOLD_ADR);
  writeFloat(minAcro, MINACRO_ADR);
    
  /*#ifdef Camera
  writeFloat(mCameraPitch, MCAMERAPITCH_ADR);
  writeFloat(mCameraRoll, MCAMERAROLL_ADR);
  writeFloat(mCameraYaw, MCAMERAYAW_ADR);
  writeFloat(centerPitch, CENTERPITCH_ADR);
  writeFloat(centerRoll, CENTERROLL_ADR);
  writeFloat(centerYaw, CENTERYAW_ADR);
  writeFloat(servoMinPitch, SERVOMINPITCH_ADR);
  writeFloat(servoMinRoll, SERVOMINROLL_ADR);
  writeFloat(servoMinYaw, SERVOMINYAW_ADR);
  writeFloat(servoMaxPitch, SERVOMAXPITCH_ADR);
  writeFloat(servoMaxRoll, SERVOMAXROLL_ADR);
  writeFloat(servoMaxYaw, SERVOMAXYAW_ADR);
  #endif*/
  
  sei(); // Restart interrupts
}
