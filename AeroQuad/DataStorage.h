/*
  AeroQuad v2.4 - December 2011
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

#ifndef _AQ_DATA_STORAGE_H_
#define _AQ_DATA_STORAGE_H_

// Utilities for writing and reading from the EEPROM
float nvrReadFloat(int address) {
  union floatStore {
    byte floatByte[4];
    unsigned short floatUShort[2];
    float floatVal;
  } floatOut;

#ifdef EEPROM_USES_16BIT_WORDS
  for (int i = 0; i < 2; i++)
    floatOut.floatUShort[i] = EEPROM.read(address + 2*i);
#else
  for (int i = 0; i < 4; i++)
    floatOut.floatByte[i] = EEPROM.read(address + i);
#endif

  return floatOut.floatVal;
}

void nvrWriteFloat(float value, int address) {
  union floatStore {
    byte floatByte[4];
    unsigned short floatUShort[2];
    float floatVal;
  } floatIn;

  floatIn.floatVal = value;
#ifdef EEPROM_USES_16BIT_WORDS
  for (int i = 0; i < 2; i++)
    EEPROM.write(address + 2*i, floatIn.floatUShort[i]);
#else
  for (int i = 0; i < 4; i++)
    EEPROM.write(address + i, floatIn.floatByte[i]);
#endif
}

void nvrReadPID(unsigned char IDPid, unsigned int IDEeprom) {
  struct PIDdata* pid = &PID[IDPid];
  pid->P = nvrReadFloat(IDEeprom);
  pid->I = nvrReadFloat(IDEeprom+4);
  pid->D = nvrReadFloat(IDEeprom+8);
  pid->lastPosition = 0;
  pid->integratedError = 0;
}

void nvrWritePID(unsigned char IDPid, unsigned int IDEeprom) {
  struct PIDdata* pid = &PID[IDPid];
  nvrWriteFloat(pid->P, IDEeprom);
  nvrWriteFloat(pid->I, IDEeprom+4);
  nvrWriteFloat(pid->D, IDEeprom+8);
}

// contains all default values when re-writing EEPROM
void initializeEEPROM() {
  PID[RATE_XAXIS_PID_IDX].P = 100.0;
  PID[RATE_XAXIS_PID_IDX].I = 0.0;
  PID[RATE_XAXIS_PID_IDX].D = -300.0;
  PID[RATE_YAXIS_PID_IDX].P = 100.0;
  PID[RATE_YAXIS_PID_IDX].I = 0.0;
  PID[RATE_YAXIS_PID_IDX].D = -300.0;
  PID[ZAXIS_PID_IDX].P = 200.0;
  PID[ZAXIS_PID_IDX].I = 5.0;
  PID[ZAXIS_PID_IDX].D = 0.0;
  PID[ATTITUDE_XAXIS_PID_IDX].P = 4.0;
  PID[ATTITUDE_XAXIS_PID_IDX].I = 0.0;
  PID[ATTITUDE_XAXIS_PID_IDX].D = 0.0;
  PID[ATTITUDE_YAXIS_PID_IDX].P = 4.0;
  PID[ATTITUDE_YAXIS_PID_IDX].I = 0.0;
  PID[ATTITUDE_YAXIS_PID_IDX].D = 0.0;
  PID[HEADING_HOLD_PID_IDX].P = 3.0;
  PID[HEADING_HOLD_PID_IDX].I = 0.1;
  PID[HEADING_HOLD_PID_IDX].D = 0.0;
  // AKA PID experiements
  PID[ATTITUDE_GYRO_XAXIS_PID_IDX].P = 100.0;
  PID[ATTITUDE_GYRO_XAXIS_PID_IDX].I = 0.0;
  PID[ATTITUDE_GYRO_XAXIS_PID_IDX].D = -300.0;
  PID[ATTITUDE_GYRO_YAXIS_PID_IDX].P = 100.0;
  PID[ATTITUDE_GYRO_YAXIS_PID_IDX].I = 0.0;
  PID[ATTITUDE_GYRO_YAXIS_PID_IDX].D = -300.0;

  PID[ALTITUDE_HOLD_PID_IDX].P = 25.0;
  PID[ALTITUDE_HOLD_PID_IDX].I = 0.6;
  PID[ALTITUDE_HOLD_PID_IDX].D = 0.0;
  PID[ALTITUDE_HOLD_PID_IDX].windupGuard = 25.0; //this prevents the 0.1 I term to rise too far
  PID[ZDAMPENING_PID_IDX].P = 0.0;
  PID[ZDAMPENING_PID_IDX].I = 0.0;
  PID[ZDAMPENING_PID_IDX].D = 0.0;
  
  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    minThrottleAdjust = -50.0;
    maxThrottleAdjust = 50.0; //we don't want it to be able to take over totally
    #if defined AltitudeHoldBaro
      baroSmoothFactor = 0.1;
    #endif
    altitudeHoldBump = 90;
    altitudeHoldPanicStickMovement = 250;
  #endif
  
  // Accel Cal
  accelScaleFactor[XAXIS] = 1.0;
  runTimeAccelBias[XAXIS] = 0.0;
  accelScaleFactor[YAXIS] = 1.0;
  runTimeAccelBias[YAXIS] = 0.0;
  accelScaleFactor[ZAXIS] = 1.0;
  runTimeAccelBias[ZAXIS] = 0.0;

  #ifdef HeadingMagHold
    magBias[XAXIS] = 0.0;
    magBias[YAXIS] = 0.0;
    magBias[ZAXIS] = 0.0;
  #endif
  windupGuard = 1000.0;

  // AKA - added so that each PID has its own windupGuard, will need to be removed once each PID's range is established and put in the eeprom
  for (byte i = XAXIS; i <= ZDAMPENING_PID_IDX; i++ ) {
    if (i != ALTITUDE_HOLD_PID_IDX) {
      PID[i].windupGuard = windupGuard;
    }
  }
    
  receiverXmitFactor = 1.0;
  gyroSmoothFactor = 1.0;
  accelSmoothFactor = 1.0;
  // AKA - old setOneG not in SI - accel->setOneG(500);
  accelOneG = -9.80665; // AKA set one G to 9.8 m/s^2
  for (byte channel = XAXIS; channel < LASTCHANNEL; channel++) {
    receiverSlope[channel] = 1.0;
    receiverOffset[channel] = 0.0;
    receiverSmoothFactor[channel] = 1.0;
  }
  receiverSmoothFactor[ZAXIS] = 0.5;

  flightMode = RATE_FLIGHT_MODE;
  headingHoldConfig = ON;
  aref = 5.0; // Use 3.0 if using a v1.7 shield or use 2.8 for an AeroQuad Shield < v1.7
  
  // Battery Monitor
  #ifdef BattMonitor
    batteryMonitorAlarmVoltage = 3.33;
    batteryMonitorThrottleTarget = 1450;
    batteryMonitorGoinDownTime = 60000;
  #endif
}

void readEEPROM() {
  readPID(XAXIS, ROLL_PID_GAIN_ADR);
  readPID(YAXIS, PITCH_PID_GAIN_ADR);
  readPID(ZAXIS, YAW_PID_GAIN_ADR);
  readPID(ATTITUDE_XAXIS_PID_IDX, LEVELROLL_PID_GAIN_ADR);
  readPID(ATTITUDE_YAXIS_PID_IDX, LEVELPITCH_PID_GAIN_ADR);
  readPID(HEADING_HOLD_PID_IDX, HEADING_PID_GAIN_ADR);
  readPID(ATTITUDE_GYRO_XAXIS_PID_IDX, LEVEL_GYRO_ROLL_PID_GAIN_ADR);
  readPID(ATTITUDE_GYRO_YAXIS_PID_IDX, LEVEL_GYRO_PITCH_PID_GAIN_ADR);

  // Leaving separate PID reads as commented for now
  // Previously had issue where EEPROM was not reading right data
  readPID(ALTITUDE_HOLD_PID_IDX, ALTITUDE_PID_GAIN_ADR);
  PID[ALTITUDE_HOLD_PID_IDX].windupGuard = readFloat(ALTITUDE_WINDUP_ADR);
  minThrottleAdjust = readFloat(ALTITUDE_MIN_THROTTLE_ADR);
  maxThrottleAdjust = readFloat(ALTITUDE_MAX_THROTTLE_ADR);
  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    #if defined AltitudeHoldBaro
      baroSmoothFactor = readFloat(ALTITUDE_SMOOTH_ADR);
    #endif  
    altitudeHoldBump = readFloat(ALTITUDE_BUMP_ADR);
    altitudeHoldPanicStickMovement = readFloat(ALTITUDE_PANIC_ADR);
  #endif
  readPID(ZDAMPENING_PID_IDX, ZDAMP_PID_GAIN_ADR);

  // Accel calibration
  accelScaleFactor[XAXIS] = readFloat(XAXIS_ACCEL_SCALE_FACTOR_ADR);
  runTimeAccelBias[XAXIS] = readFloat(XAXIS_ACCEL_BIAS_ADR);
  accelScaleFactor[YAXIS] = readFloat(YAXIS_ACCEL_SCALE_FACTOR_ADR);
  runTimeAccelBias[YAXIS] = readFloat(YAXIS_ACCEL_BIAS_ADR);
  accelScaleFactor[ZAXIS] = readFloat(ZAXIS_ACCEL_SCALE_FACTOR_ADR);
  runTimeAccelBias[ZAXIS] = readFloat(ZAXIS_ACCEL_BIAS_ADR);

  // Mag calibration
  #ifdef HeadingMagHold
    magBias[XAXIS] = readFloat(XAXIS_MAG_BIAS_ADR);
    magBias[YAXIS] = readFloat(YAXIS_MAG_BIAS_ADR);
    magBias[ZAXIS] = readFloat(ZAXIS_MAG_BIAS_ADR);
  #endif
  
  // Battery Monitor
  #ifdef BattMonitor
    batteryMonitorAlarmVoltage = readFloat(BATT_ALARM_VOLTAGE_ADR);
    batteryMonitorThrottleTarget = readFloat(BATT_THROTTLE_TARGET_ADR);
    batteryMonitorGoinDownTime = readFloat(BATT_DOWN_TIME_ADR);
  #endif
  
  windupGuard = readFloat(WINDUPGUARD_ADR);
  // AKA - added so that each PID has its own windupGuard, will need to be removed once each PID's range is established and put in the eeprom
  for (byte i = XAXIS; i <= ZDAMPENING_PID_IDX; i++ ) {
    if (i != ALTITUDE_HOLD_PID_IDX) {
      PID[i].windupGuard = windupGuard;
    }
  }
    
  aref = readFloat(AREF_ADR);
  flightMode = readFloat(FLIGHTMODE_ADR);
  accelOneG = readFloat(ACCEL_1G_ADR);
  headingHoldConfig = readFloat(HEADINGHOLD_ADR);
  if (headingHoldConfig) {
    vehicleState |= HEADINGHOLD_ENABLED;
  }
  else {
    vehicleState &= ~HEADINGHOLD_ENABLED;
  }  
}

void writeEEPROM(){
  cli(); // Needed so that APM sensor data doesn't overflow
  writePID(XAXIS, ROLL_PID_GAIN_ADR);
  writePID(YAXIS, PITCH_PID_GAIN_ADR);
  writePID(ATTITUDE_XAXIS_PID_IDX, LEVELROLL_PID_GAIN_ADR);
  writePID(ATTITUDE_YAXIS_PID_IDX, LEVELPITCH_PID_GAIN_ADR);
  writePID(ZAXIS, YAW_PID_GAIN_ADR);
  writePID(HEADING_HOLD_PID_IDX, HEADING_PID_GAIN_ADR);
  writePID(ATTITUDE_GYRO_XAXIS_PID_IDX, LEVEL_GYRO_ROLL_PID_GAIN_ADR);
  writePID(ATTITUDE_GYRO_YAXIS_PID_IDX, LEVEL_GYRO_PITCH_PID_GAIN_ADR);
  writePID(ALTITUDE_HOLD_PID_IDX, ALTITUDE_PID_GAIN_ADR);
  writeFloat(PID[ALTITUDE_HOLD_PID_IDX].windupGuard, ALTITUDE_WINDUP_ADR);
  writeFloat(minThrottleAdjust, ALTITUDE_MIN_THROTTLE_ADR);
  writeFloat(maxThrottleAdjust, ALTITUDE_MAX_THROTTLE_ADR);
  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    #if defined AltitudeHoldBaro
      writeFloat(baroSmoothFactor, ALTITUDE_SMOOTH_ADR);
    #else
      writeFloat(0, ALTITUDE_SMOOTH_ADR);
    #endif
    writeFloat(altitudeHoldBump, ALTITUDE_BUMP_ADR);
    writeFloat(altitudeHoldPanicStickMovement, ALTITUDE_PANIC_ADR);
  #else
    writeFloat(0.1, ALTITUDE_SMOOTH_ADR);
    writeFloat(90, ALTITUDE_BUMP_ADR);
    writeFloat(250, ALTITUDE_PANIC_ADR);
  #endif
  writePID(ZDAMPENING_PID_IDX, ZDAMP_PID_GAIN_ADR);
  // Accel Cal
  writeFloat(accelScaleFactor[XAXIS], XAXIS_ACCEL_SCALE_FACTOR_ADR);
  writeFloat(runTimeAccelBias[XAXIS], XAXIS_ACCEL_BIAS_ADR);
  writeFloat(accelScaleFactor[YAXIS], YAXIS_ACCEL_SCALE_FACTOR_ADR);
  writeFloat(runTimeAccelBias[YAXIS], YAXIS_ACCEL_BIAS_ADR);
  writeFloat(accelScaleFactor[ZAXIS], ZAXIS_ACCEL_SCALE_FACTOR_ADR);
  writeFloat(runTimeAccelBias[ZAXIS], ZAXIS_ACCEL_BIAS_ADR);
  #ifdef HeadingMagHold
    writeFloat(magBias[XAXIS], XAXIS_MAG_BIAS_ADR);
    writeFloat(magBias[YAXIS], YAXIS_MAG_BIAS_ADR);
    writeFloat(magBias[ZAXIS], ZAXIS_MAG_BIAS_ADR);
  #endif
  writeFloat(windupGuard, WINDUPGUARD_ADR);
  writeFloat(receiverXmitFactor, XMITFACTOR_ADR);
  writeFloat(gyroSmoothFactor, GYROSMOOTH_ADR);
  writeFloat(accelSmoothFactor, ACCSMOOTH_ADR);

  for(byte channel = XAXIS; channel < LASTCHANNEL; channel++) {
    writeFloat(receiverSlope[channel],  RECEIVER_DATA[channel].slope);
    writeFloat(receiverOffset[channel], RECEIVER_DATA[channel].offset);
    writeFloat(receiverSmoothFactor[channel], RECEIVER_DATA[channel].smooth_factor);
  }

  writeFloat(aref, AREF_ADR);
  writeFloat(flightMode, FLIGHTMODE_ADR);
  writeFloat(headingHoldConfig, HEADINGHOLD_ADR);
  writeFloat(accelOneG, ACCEL_1G_ADR);
  writeFloat(SOFTWARE_VERSION, SOFTWARE_VERSION_ADR);
  
  // Battery Monitor
  #ifdef BattMonitor
    writeFloat(batteryMonitorAlarmVoltage, BATT_ALARM_VOLTAGE_ADR);
    writeFloat(batteryMonitorThrottleTarget, BATT_THROTTLE_TARGET_ADR);
    writeFloat(batteryMonitorGoinDownTime, BATT_DOWN_TIME_ADR);
  #endif
  
  sei(); // Restart interrupts
}

void initSensorsZeroFromEEPROM() {
  // Gyro initialization from EEPROM
  gyroZero[XAXIS] = readFloat(GYRO_ROLL_ZERO_ADR);
  gyroZero[YAXIS] = readFloat(GYRO_PITCH_ZERO_ADR);
  gyroZero[ZAXIS] = readFloat(GYRO_YAW_ZERO_ADR);
  gyroSmoothFactor = readFloat(GYROSMOOTH_ADR);
 
  // Accel initialization from EEPROM
  accelOneG = readFloat(ACCEL_1G_ADR);
  accelSmoothFactor = readFloat(ACCSMOOTH_ADR);
}

void storeSensorsZeroToEEPROM() {
  // Store gyro data to EEPROM
  writeFloat(gyroZero[XAXIS], GYRO_ROLL_ZERO_ADR);
  writeFloat(gyroZero[YAXIS], GYRO_PITCH_ZERO_ADR);
  writeFloat(gyroZero[ZAXIS], GYRO_YAW_ZERO_ADR);
  writeFloat(gyroSmoothFactor, GYROSMOOTH_ADR);
  
  // Store accel data to EEPROM
  writeFloat(accelOneG, ACCEL_1G_ADR);
  writeFloat(accelSmoothFactor, ACCSMOOTH_ADR);
}

void initReceiverFromEEPROM() {
  receiverXmitFactor = readFloat(XMITFACTOR_ADR);
  
  for(byte channel = XAXIS; channel < LASTCHANNEL; channel++) {
    receiverSlope[channel] = readFloat(RECEIVER_DATA[channel].slope);
    receiverOffset[channel] = readFloat(RECEIVER_DATA[channel].offset);
    receiverSmoothFactor[channel] = readFloat(RECEIVER_DATA[channel].smooth_factor);
  }
}

#endif // _AQ_DATA_STORAGE_H_

