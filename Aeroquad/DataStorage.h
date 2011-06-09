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

// Special thanks for 1k space optimization update from Ala42
// http://aeroquad.com/showthread.php?1369-The-big-enhancement-addition-to-2.0-code&p=13359&viewfull=1#post13359

// Utilities for writing and reading from the EEPROM
float nvrReadFloat(int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatOut;

  for (int i = 0; i < 4; i++)
    floatOut.floatByte[i] = EEPROM.read(address + i);
  return floatOut.floatVal;
}

void nvrWriteFloat(float value, int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatIn;

  floatIn.floatVal = value;
  for (int i = 0; i < 4; i++)
    EEPROM.write(address + i, floatIn.floatByte[i]);
}

void nvrReadPID(unsigned char IDPid, unsigned int IDEeprom) {
  struct PIDdata* pid = &PID[IDPid];
  pid->P = nvrReadFloat(IDEeprom);
  pid->I = nvrReadFloat(IDEeprom+4);
  pid->D = nvrReadFloat(IDEeprom+8);
  pid->lastPosition = 0;
  pid->integratedError = 0;
  // AKA experiements with PIDS
  pid->firstPass = true;
  if (IDPid == HEADING)
    pid->typePID = TYPEPI;
  else
    pid->typePID = NOTYPE;
}

void nvrWritePID(unsigned char IDPid, unsigned int IDEeprom) {
  struct PIDdata* pid = &PID[IDPid];
  nvrWriteFloat(pid->P, IDEeprom);
  nvrWriteFloat(pid->I, IDEeprom+4);
  nvrWriteFloat(pid->D, IDEeprom+8);
}

// contains all default values when re-writing EEPROM
void initializeEEPROM(void) {
  PID[ROLL].P = 100.0;
  PID[ROLL].I = 0.0;
  PID[ROLL].D = -300.0;
  PID[PITCH].P = 100.0;
  PID[PITCH].I = 0.0;
  PID[PITCH].D = -300.0;
  PID[YAW].P = 200.0;
  PID[YAW].I = 5.0;
  PID[YAW].D = 0.0;
  PID[LEVELROLL].P = 4.0;
  PID[LEVELROLL].I = 0.6;
  PID[LEVELROLL].D = 0.0;
  PID[LEVELPITCH].P = 4.0;
  PID[LEVELPITCH].I = 0.6;
  PID[LEVELPITCH].D = 0.0;
  PID[HEADING].P = 3.0;
  PID[HEADING].I = 0.1;
  PID[HEADING].D = 0.0;
  // AKA PID experiements
  PID[HEADING].typePID = TYPEPI;
  PID[LEVELGYROROLL].P = 100.0;
  PID[LEVELGYROROLL].I = 0.0;
  PID[LEVELGYROROLL].D = -300.0;
  PID[LEVELGYROPITCH].P = 100.0;
  PID[LEVELGYROPITCH].I = 0.0;
  PID[LEVELGYROPITCH].D = -300.0;
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
    barometricSensor->setSmoothFactor(0.1);
  #endif
  #ifdef HeadingMagHold
    compass->setMagCal(XAXIS, 1, 0);
    compass->setMagCal(YAXIS, 1, 0);
    compass->setMagCal(ZAXIS, 1, 0);
  #endif
  windupGuard = 1000.0;

  // AKA - added so that each PID has its own windupGuard, will need to be removed once each PID's range is established and put in the eeprom
  for (byte i = ROLL; i <= ZDAMPENING; i++ ) {
    if (i != ALTITUDE)
        PID[i].windupGuard = windupGuard;
  }
  // AKA added so that each PID has a type incase we need special cases like detecting +/- PI
  for (byte i = ROLL; i <= ZDAMPENING; i++ ) {
    if (i != HEADING)
        PID[i].typePID = NOTYPE;
  }
    
  receiver->setXmitFactor(1.0);
//  levelLimit = 500.0;
//  levelOff = 150.0;
  gyro->setSmoothFactor(1.0);
  accel->setSmoothFactor(1.0);
  // AKA - old setOneG not in SI - accel->setOneG(500);
  accel->setOneG(9.80665); // AKA set one G to 9.8 m/s^2
  timeConstant = 7.0;
  for (byte channel = ROLL; channel < LASTCHANNEL; channel++) {
    receiver->setTransmitterSlope(channel, 1.0);
    receiver->setTransmitterOffset(channel, 0.0);
    receiver->setSmoothFactor(channel, 1.0);
  }
  receiver->setSmoothFactor(YAW, 0.5);

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
    minThrottleAdjust = readFloat(ALTITUDE_MIN_THROTTLE_ADR);
    maxThrottleAdjust = readFloat(ALTITUDE_MAX_THROTTLE_ADR);
    barometricSensor->setSmoothFactor(readFloat(ALTITUDE_SMOOTH_ADR));
    readPID(ZDAMPENING, ZDAMP_PGAIN_ADR);
  #endif

  #ifdef HeadingMagHold
    compass->setMagCal(XAXIS, readFloat(MAGXMAX_ADR), readFloat(MAGXMIN_ADR));
    compass->setMagCal(YAXIS, readFloat(MAGYMAX_ADR), readFloat(MAGYMIN_ADR));
    compass->setMagCal(ZAXIS, readFloat(MAGZMAX_ADR), readFloat(MAGZMIN_ADR));
  #endif

  windupGuard = readFloat(WINDUPGUARD_ADR);

  // AKA - added so that each PID has its own windupGuard, will need to be removed once each PID's range is established and put in the eeprom
  for (byte i = ROLL; i <= ZDAMPENING; i++ ) {
    if (i != ALTITUDE)
        PID[i].windupGuard = windupGuard;
  }
    
//  levelLimit = readFloat(LEVELLIMIT_ADR);
//  levelOff = readFloat(LEVELOFF_ADR);
  timeConstant = readFloat(FILTERTERM_ADR);
  smoothHeading = readFloat(HEADINGSMOOTH_ADR);
  aref = readFloat(AREF_ADR);
  flightMode = readFloat(FLIGHTMODE_ADR);
  headingHoldConfig = readFloat(HEADINGHOLD_ADR);
  minAcro = readFloat(MINACRO_ADR);
  accel->setOneG(readFloat(ACCEL_1G_ADR));
  
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
    writeFloat(minThrottleAdjust, ALTITUDE_MIN_THROTTLE_ADR);
    writeFloat(maxThrottleAdjust, ALTITUDE_MAX_THROTTLE_ADR);
    writeFloat(barometricSensor->getSmoothFactor(), ALTITUDE_SMOOTH_ADR);
    writePID(ZDAMPENING, ZDAMP_PGAIN_ADR);
  #endif
  #ifdef HeadingMagHold
    writeFloat(compass->getMagMax(XAXIS), MAGXMAX_ADR);
    writeFloat(compass->getMagMin(XAXIS), MAGXMIN_ADR);
    writeFloat(compass->getMagMax(YAXIS), MAGYMAX_ADR);
    writeFloat(compass->getMagMin(YAXIS), MAGYMIN_ADR);
    writeFloat(compass->getMagMax(ZAXIS), MAGZMAX_ADR);
    writeFloat(compass->getMagMin(ZAXIS), MAGZMIN_ADR);
  #endif
  writeFloat(windupGuard, WINDUPGUARD_ADR);
//  writeFloat(levelLimit, LEVELLIMIT_ADR);
//  writeFloat(levelOff, LEVELOFF_ADR);
  writeFloat(receiver->getXmitFactor(), XMITFACTOR_ADR);
  writeFloat(gyro->getSmoothFactor(), GYROSMOOTH_ADR);
  writeFloat(accel->getSmoothFactor(), ACCSMOOTH_ADR);
  writeFloat(timeConstant, FILTERTERM_ADR);


  writeFloat(receiver->getTransmitterSlope(0),  RECEIVER_CHANNEL_0_SLOPE_ADR);
  writeFloat(receiver->getTransmitterOffset(0), RECEIVER_CHANNEL_0_OFFSET_ADR);
  writeFloat(receiver->getSmoothFactor(0),      RECEIVER_CHANNEL_0_SMOOTH_FACTOR_ADR);
  writeFloat(receiver->getTransmitterSlope(1),  RECEIVER_CHANNEL_1_SLOPE_ADR);
  writeFloat(receiver->getTransmitterOffset(1), RECEIVER_CHANNEL_1_OFFSET_ADR);
  writeFloat(receiver->getSmoothFactor(1),      RECEIVER_CHANNEL_1_SMOOTH_FACTOR_ADR);
  writeFloat(receiver->getTransmitterSlope(2),  RECEIVER_CHANNEL_2_SLOPE_ADR);
  writeFloat(receiver->getTransmitterOffset(2), RECEIVER_CHANNEL_2_OFFSET_ADR);
  writeFloat(receiver->getSmoothFactor(2),      RECEIVER_CHANNEL_2_SMOOTH_FACTOR_ADR);
  writeFloat(receiver->getTransmitterSlope(3),  RECEIVER_CHANNEL_3_SLOPE_ADR);
  writeFloat(receiver->getTransmitterOffset(3), RECEIVER_CHANNEL_3_OFFSET_ADR);
  writeFloat(receiver->getSmoothFactor(3),      RECEIVER_CHANNEL_3_SMOOTH_FACTOR_ADR);
  writeFloat(receiver->getTransmitterSlope(4),  RECEIVER_CHANNEL_4_SLOPE_ADR);
  writeFloat(receiver->getTransmitterOffset(4), RECEIVER_CHANNEL_4_OFFSET_ADR);
  writeFloat(receiver->getSmoothFactor(4),      RECEIVER_CHANNEL_4_SMOOTH_FACTOR_ADR);
  writeFloat(receiver->getTransmitterSlope(5),  RECEIVER_CHANNEL_5_SLOPE_ADR);
  writeFloat(receiver->getTransmitterOffset(5), RECEIVER_CHANNEL_5_OFFSET_ADR);
  writeFloat(receiver->getSmoothFactor(5),      RECEIVER_CHANNEL_5_SMOOTH_FACTOR_ADR);

  writeFloat(smoothHeading, HEADINGSMOOTH_ADR);
  writeFloat(aref, AREF_ADR);
  writeFloat(flightMode, FLIGHTMODE_ADR);
  writeFloat(headingHoldConfig, HEADINGHOLD_ADR);
  writeFloat(minAcro, MINACRO_ADR);
  writeFloat(accel->getOneG(), ACCEL_1G_ADR);
    
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

void initSensorsZeroFromEEPROM(void) {
  // Gyro initialization from EEPROM
  gyro->setZero(ROLL,readFloat(GYRO_ROLL_ZERO_ADR));
  gyro->setZero(PITCH,readFloat(GYRO_PITCH_ZERO_ADR));
  gyro->setZero(YAW,readFloat(GYRO_YAW_ZERO_ADR));
  gyro->setSmoothFactor(readFloat(GYROSMOOTH_ADR));
  
  // Accel initialization from EEPROM
  accel->setOneG(readFloat(ACCEL_1G_ADR));
//  Serial.print("read zero : ");  Serial.println(readFloat(ACCEL_XAXIS_ZERO_ADR));
  accel->setZero(XAXIS,readFloat(ACCEL_XAXIS_ZERO_ADR));
  accel->setZero(YAXIS,readFloat(ACCEL_YAXIS_ZERO_ADR));
  accel->setZero(ZAXIS,readFloat(ACCEL_ZAXIS_ZERO_ADR));
  accel->setSmoothFactor(readFloat(ACCSMOOTH_ADR));
}

void storeSensorsZeroToEEPROM(void) {
  // Store gyro data to EEPROM
  writeFloat(gyro->getZero(ROLL), GYRO_ROLL_ZERO_ADR);
  writeFloat(gyro->getZero(PITCH), GYRO_PITCH_ZERO_ADR);
  writeFloat(gyro->getZero(YAW), GYRO_YAW_ZERO_ADR);
  writeFloat(gyro->getSmoothFactor(), GYROSMOOTH_ADR);
  
  // Store accel data to EEPROM
  writeFloat(accel->getOneG(), ACCEL_1G_ADR);
//  Serial.print("write zero : ");  Serial.println(readFloat(ACCEL_XAXIS_ZERO_ADR));
  writeFloat(accel->getZero(XAXIS), ACCEL_XAXIS_ZERO_ADR);
  writeFloat(accel->getZero(YAXIS), ACCEL_YAXIS_ZERO_ADR);
  writeFloat(accel->getZero(ZAXIS), ACCEL_ZAXIS_ZERO_ADR);
  writeFloat(accel->getSmoothFactor(), ACCSMOOTH_ADR);
}

void initReceiverFromEEPROM(void) {
  receiver->setXmitFactor(readFloat(XMITFACTOR_ADR));

  receiver->setTransmitterSlope(0,readFloat(RECEIVER_CHANNEL_0_SLOPE_ADR));
  receiver->setTransmitterOffset(0,readFloat(RECEIVER_CHANNEL_0_OFFSET_ADR));
  receiver->setSmoothFactor(0,readFloat(RECEIVER_CHANNEL_0_SMOOTH_FACTOR_ADR));
  receiver->setTransmitterSlope(1,readFloat(RECEIVER_CHANNEL_1_SLOPE_ADR));
  receiver->setTransmitterOffset(1,readFloat(RECEIVER_CHANNEL_1_OFFSET_ADR));
  receiver->setSmoothFactor(1,readFloat(RECEIVER_CHANNEL_1_SMOOTH_FACTOR_ADR));
  receiver->setTransmitterSlope(2,readFloat(RECEIVER_CHANNEL_2_SLOPE_ADR));
  receiver->setTransmitterOffset(2,readFloat(RECEIVER_CHANNEL_2_OFFSET_ADR));
  receiver->setSmoothFactor(2,readFloat(RECEIVER_CHANNEL_2_SMOOTH_FACTOR_ADR));
  receiver->setTransmitterSlope(3,readFloat(RECEIVER_CHANNEL_3_SLOPE_ADR));
  receiver->setTransmitterOffset(3,readFloat(RECEIVER_CHANNEL_3_OFFSET_ADR));
  receiver->setSmoothFactor(3,readFloat(RECEIVER_CHANNEL_3_SMOOTH_FACTOR_ADR));
  receiver->setTransmitterSlope(4,readFloat(RECEIVER_CHANNEL_4_SLOPE_ADR));
  receiver->setTransmitterOffset(4,readFloat(RECEIVER_CHANNEL_4_OFFSET_ADR));
  receiver->setSmoothFactor(4,readFloat(RECEIVER_CHANNEL_4_SMOOTH_FACTOR_ADR));
  receiver->setTransmitterSlope(5,readFloat(RECEIVER_CHANNEL_5_SLOPE_ADR));
  receiver->setTransmitterOffset(5,readFloat(RECEIVER_CHANNEL_5_OFFSET_ADR));
  receiver->setSmoothFactor(5,readFloat(RECEIVER_CHANNEL_5_SMOOTH_FACTOR_ADR));
}

