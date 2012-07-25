/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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
  for (int i = 0; i < 2; i++) {
    floatOut.floatUShort[i] = EEPROM.read(address + 2*i);
  }
#else
  for (int i = 0; i < 4; i++) {
    floatOut.floatByte[i] = EEPROM.read(address + i);
  }
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
  for (int i = 0; i < 2; i++) {
    EEPROM.write(address + 2*i, floatIn.floatUShort[i]);
  }
#else
  for (int i = 0; i < 4; i++) {
    EEPROM.write(address + i, floatIn.floatByte[i]);
  }
#endif
}

long nvrReadLong(int address) {
  union longStore {
    byte longByte[4];
    unsigned short longUShort[2];
    long longVal;
  } longOut;  

#ifdef EEPROM_USES_16BIT_WORDS
  for (int i = 0; i < 2; i++) {
    longOut.longUShort[i] = EEPROM.read(address + 2*i);
  }
#else
  for (byte i = 0; i < 4; i++) {
    longOut.longByte[i] = EEPROM.read(address + i);
  }
#endif
    
  return longOut.longVal;
}

void nvrWriteLong(long value, int address) {
  union longStore {
    byte longByte[4];
    unsigned short longUShort[2];
    long longVal;
  } longIn;  

  longIn.longVal = value;
  
#ifdef EEPROM_USES_16BIT_WORDS
  for (int i = 0; i < 2; i++) {
    EEPROM.write(address + 2*i, longIn.longUShort[i]);
  }
#else
  for (int i = 0; i < 4; i++) {
    EEPROM.write(address + i, longIn.longByte[i]);
  }
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
  PID[ATTITUDE_XAXIS_PID_IDX].P = 4.0;
  PID[ATTITUDE_XAXIS_PID_IDX].I = 0.0;
  PID[ATTITUDE_XAXIS_PID_IDX].D = 0.0;
  PID[ATTITUDE_YAXIS_PID_IDX].P = 4.0;
  PID[ATTITUDE_YAXIS_PID_IDX].I = 0.0;
  PID[ATTITUDE_YAXIS_PID_IDX].D = 0.0;
  PID[ZAXIS_PID_IDX].P = 200.0;
  PID[ZAXIS_PID_IDX].I = 5.0;
  PID[ZAXIS_PID_IDX].D = 0.0;
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

  PID[BARO_ALTITUDE_HOLD_PID_IDX].P = 25.0;
  PID[BARO_ALTITUDE_HOLD_PID_IDX].I = 0.6;
  PID[BARO_ALTITUDE_HOLD_PID_IDX].D = 0.0;
  PID[BARO_ALTITUDE_HOLD_PID_IDX].windupGuard = 25.0; //this prevents the 0.1 I term to rise too far
  PID[SONAR_ALTITUDE_HOLD_PID_IDX].P = 50.0;
  PID[SONAR_ALTITUDE_HOLD_PID_IDX].I = 0.6;
  PID[SONAR_ALTITUDE_HOLD_PID_IDX].D = 0.0;
  PID[SONAR_ALTITUDE_HOLD_PID_IDX].windupGuard = 25.0; //this prevents the 0.1 I term to rise too far

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
  
  // Gyro Cal
  gyroZero[XAXIS] = 0.0;
  gyroZero[YAXIS] = 0.0;
  gyroZero[ZAXIS] = 0.0;
  gyroSmoothFactor = 0.0;
  gyroTempBiasSlope[XAXIS] = 0.0;
  gyroTempBiasSlope[YAXIS] = 0.0;
  gyroTempBiasSlope[ZAXIS] = 0.0;
  gyroTempBiasIntercept[XAXIS] = 0.0;
  gyroTempBiasIntercept[YAXIS] = 0.0;
  gyroTempBiasIntercept[ZAXIS] = 0.0;
  
  // Accel Cal
  accelScaleFactor[XAXIS] = 1.0;
  runTimeAccelBias[XAXIS] = 0.0;
  accelScaleFactor[YAXIS] = 1.0;
  runTimeAccelBias[YAXIS] = 0.0;
  accelScaleFactor[ZAXIS] = 1.0;
  runTimeAccelBias[ZAXIS] = 0.0;

  #ifdef HeadingMagHold
    magScale[XAXIS] = 1.0;
    magBias[XAXIS] = 0.0;
    magScale[YAXIS] = 1.0;
    magBias[YAXIS] = 0.0;
    magScale[ZAXIS] = 1.0;
    magBias[ZAXIS] = 0.0;
  #endif
  windupGuard = 1000.0;

  // AKA - added so that each PID has its own windupGuard, will need to be removed once each PID's range is established and put in the eeprom
  for (byte i = XAXIS; i <= ZDAMPENING_PID_IDX; i++ ) {
    if (i != BARO_ALTITUDE_HOLD_PID_IDX) {
      PID[i].windupGuard = windupGuard;
    }
  }
    
  receiverXmitFactor = 1.0;
  minArmedThrottle = 1150;
  gyroSmoothFactor = 1.0;
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

  // Range Finder
  #if defined (AltitudeHoldRangeFinder)
    maxRangeFinderRange = 4.5;
    minRangeFinderRange = 0.0;
  #endif
  
  #if defined (UseGPSNavigator)
    missionNbPoint = 0;
    PID[GPSROLL_PID_IDX].P = 0.8;
    PID[GPSROLL_PID_IDX].I = 0.0;
    PID[GPSROLL_PID_IDX].D = 0.0;
    PID[GPSPITCH_PID_IDX].P = 0.8;
    PID[GPSPITCH_PID_IDX].I = 0.0;
    PID[GPSPITCH_PID_IDX].D = 0.0;
    PID[GPSYAW_PID_IDX].P = 50.0;
    PID[GPSYAW_PID_IDX].I = 0.0;
    PID[GPSYAW_PID_IDX].D = 0.0;

    for (byte location = 0; location < MAX_WAYPOINTS; location++) {
      waypoint[location].longitude = GPS_INVALID_ANGLE;
      waypoint[location].latitude = GPS_INVALID_ANGLE;
      waypoint[location].altitude = GPS_INVALID_ALTITUDE;
    }
  #endif

  // Camera Control
  #ifdef CameraControl
    cameraMode = 1;
    mCameraPitch = 1273.2;    
    mCameraRoll = 636.6;    
    mCameraYaw = 318.3;
    servoCenterPitch = 1500;
    servoCenterRoll = 1500;
    servoCenterYaw = 1500;
    servoMinPitch = 1000;
    servoMinRoll = 1000;
    servoMinYaw = 1000;
    servoMaxPitch = 2000;
    servoMaxRoll = 2000;
    servoMaxYaw = 2000;
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
  readPID(BARO_ALTITUDE_HOLD_PID_IDX, ALTITUDE_PID_GAIN_ADR);
  PID[BARO_ALTITUDE_HOLD_PID_IDX].windupGuard = readFloat(ALTITUDE_WINDUP_ADR);
  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    #if defined AltitudeHoldBaro
      baroSmoothFactor = readFloat(ALTITUDE_SMOOTH_ADR);
    #endif  
    altitudeHoldBump = readFloat(ALTITUDE_BUMP_ADR);
    altitudeHoldPanicStickMovement = readFloat(ALTITUDE_PANIC_ADR);
    minThrottleAdjust = readFloat(ALTITUDE_MIN_THROTTLE_ADR);
    maxThrottleAdjust = readFloat(ALTITUDE_MAX_THROTTLE_ADR);
  #endif
  readPID(ZDAMPENING_PID_IDX, ZDAMP_PID_GAIN_ADR);

  // Mag calibration
  #ifdef HeadingMagHold
    magBias[XAXIS]  = readFloat(XAXIS_MAG_BIAS_ADR);
    magScale[XAXIS] = readFloat(XAXIS_MAG_SCALE_FACTOR_ADR);
    magBias[YAXIS]  = readFloat(YAXIS_MAG_BIAS_ADR);
    magScale[YAXIS] = readFloat(YAXIS_MAG_SCALE_FACTOR_ADR);
    magBias[ZAXIS]  = readFloat(ZAXIS_MAG_BIAS_ADR);
    magScale[ZAXIS] = readFloat(ZAXIS_MAG_SCALE_FACTOR_ADR);
  #endif
  
  // Battery Monitor
  #ifdef BattMonitor
    batteryMonitorAlarmVoltage = readFloat(BATT_ALARM_VOLTAGE_ADR);
    batteryMonitorThrottleTarget = readFloat(BATT_THROTTLE_TARGET_ADR);
    batteryMonitorGoinDownTime = readFloat(BATT_DOWN_TIME_ADR);
  #endif
  
  windupGuard = readFloat(WINDUPGUARD_ADR);
  // AKA - added so that each PID has its own windupGuard, will need to be removed once each PID's range is established and put in the EEPROM
  for (byte i = XAXIS; i <= ZDAMPENING_PID_IDX; i++ ) {
    if (i != BARO_ALTITUDE_HOLD_PID_IDX) {
      PID[i].windupGuard = windupGuard;
    }
  }
    
  minArmedThrottle = readFloat(MINARMEDTHROTTLE_ADR);
  aref = readFloat(AREF_ADR);
  flightMode = readFloat(FLIGHTMODE_ADR);
  accelOneG = readFloat(ACCEL_1G_ADR);
  headingHoldConfig = readFloat(HEADINGHOLD_ADR);

  // Range Finder
  #if defined (AltitudeHoldRangeFinder)
//    maxRangeFinderRange = readFloat(RANGE_FINDER_MAX_ADR);  // @Kenny @todo, remove this!
//    minRangeFinderRange = readFloat(RANGE_FINDER_MIN_ADR);
  #endif     
  
  #if defined (UseGPSNavigator)
    missionNbPoint = readFloat(GPS_MISSION_NB_POINT_ADR);
    readPID(GPSROLL_PID_IDX, GPSROLL_PID_GAIN_ADR);
    readPID(GPSPITCH_PID_IDX, GPSPITCH_PID_GAIN_ADR);
    readPID(GPSYAW_PID_IDX, GPSYAW_PID_GAIN_ADR);
    
    for (byte location = 0; location < MAX_WAYPOINTS; location++) {
      waypoint[location].longitude = readLong(WAYPOINT_ADR[location].longitude);
      waypoint[location].latitude = readLong(WAYPOINT_ADR[location].latitude);
      waypoint[location].altitude = readLong(WAYPOINT_ADR[location].altitude);
    }    
  #endif

  // Camera Control
  #ifdef CameraControl
    cameraMode = readFloat(CAMERAMODE_ADR);
    mCameraPitch = readFloat(MCAMERAPITCH_ADR);
    mCameraRoll = readFloat(MCAMERAROLL_ADR);    
    mCameraYaw = readFloat(MCAMERAYAW_ADR);
    servoCenterPitch = readFloat(SERVOCENTERPITCH_ADR);
    servoCenterRoll = readFloat(SERVOCENTERROLL_ADR);
    servoCenterYaw = readFloat(SERVOCENTERYAW_ADR);
    servoMinPitch = readFloat(SERVOMINPITCH_ADR);
    servoMinRoll = readFloat(SERVOMINROLL_ADR);
    servoMinYaw = readFloat(SERVOMINYAW_ADR);
    servoMaxPitch = readFloat(SERVOMAXPITCH_ADR);
    servoMaxRoll = readFloat(SERVOMAXROLL_ADR);
    servoMaxYaw = readFloat(SERVOMAXYAW_ADR);
  #endif   
}

void writeEEPROM(){
  cli(); // Needed so that APM sensor data does not overflow
  writePID(XAXIS, ROLL_PID_GAIN_ADR);
  writePID(YAXIS, PITCH_PID_GAIN_ADR);
  writePID(ATTITUDE_XAXIS_PID_IDX, LEVELROLL_PID_GAIN_ADR);
  writePID(ATTITUDE_YAXIS_PID_IDX, LEVELPITCH_PID_GAIN_ADR);
  writePID(ZAXIS, YAW_PID_GAIN_ADR);
  writePID(HEADING_HOLD_PID_IDX, HEADING_PID_GAIN_ADR);
  writePID(ATTITUDE_GYRO_XAXIS_PID_IDX, LEVEL_GYRO_ROLL_PID_GAIN_ADR);
  writePID(ATTITUDE_GYRO_YAXIS_PID_IDX, LEVEL_GYRO_PITCH_PID_GAIN_ADR);
  writePID(BARO_ALTITUDE_HOLD_PID_IDX, ALTITUDE_PID_GAIN_ADR);
  writeFloat(PID[BARO_ALTITUDE_HOLD_PID_IDX].windupGuard, ALTITUDE_WINDUP_ADR);

  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    #if defined AltitudeHoldBaro
      writeFloat(baroSmoothFactor, ALTITUDE_SMOOTH_ADR);
    #else
      writeFloat(0.0, ALTITUDE_SMOOTH_ADR);
    #endif
    writeFloat(altitudeHoldBump, ALTITUDE_BUMP_ADR);
    writeFloat(altitudeHoldPanicStickMovement, ALTITUDE_PANIC_ADR);
    writeFloat(minThrottleAdjust, ALTITUDE_MIN_THROTTLE_ADR);
    writeFloat(maxThrottleAdjust, ALTITUDE_MAX_THROTTLE_ADR);
  #else
    writeFloat(0.1, ALTITUDE_SMOOTH_ADR);
    writeFloat(90, ALTITUDE_BUMP_ADR);
    writeFloat(250, ALTITUDE_PANIC_ADR);
    writeFloat(-50, ALTITUDE_MIN_THROTTLE_ADR);
    writeFloat(50, ALTITUDE_MAX_THROTTLE_ADR);
    writeFloat(0.1, ALTITUDE_SMOOTH_ADR);
  #endif
  writePID(ZDAMPENING_PID_IDX, ZDAMP_PID_GAIN_ADR);
  #ifdef HeadingMagHold
    writeFloat(magBias[XAXIS], XAXIS_MAG_BIAS_ADR);
    writeFloat(magScale[XAXIS], XAXIS_MAG_SCALE_FACTOR_ADR);
    writeFloat(magBias[YAXIS], YAXIS_MAG_BIAS_ADR);
    writeFloat(magScale[YAXIS], YAXIS_MAG_SCALE_FACTOR_ADR);
    writeFloat(magBias[ZAXIS], ZAXIS_MAG_BIAS_ADR);
    writeFloat(magScale[ZAXIS], ZAXIS_MAG_SCALE_FACTOR_ADR);
  #endif
  writeFloat(windupGuard, WINDUPGUARD_ADR);
  writeFloat(receiverXmitFactor, XMITFACTOR_ADR);
  writeFloat(gyroSmoothFactor, GYROSMOOTH_ADR);

  for(byte channel = XAXIS; channel < LASTCHANNEL; channel++) {
    writeFloat(receiverSlope[channel],  RECEIVER_DATA[channel].slope);
    writeFloat(receiverOffset[channel], RECEIVER_DATA[channel].offset);
    writeFloat(receiverSmoothFactor[channel], RECEIVER_DATA[channel].smooth_factor);
  }

  writeFloat(minArmedThrottle, MINARMEDTHROTTLE_ADR);
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

  // Range Finder
  #if defined (AltitudeHoldRangeFinder)
    writeFloat(maxRangeFinderRange, RANGE_FINDER_MAX_ADR);
    writeFloat(minRangeFinderRange, RANGE_FINDER_MIN_ADR);
  #else
    writeFloat(0, RANGE_FINDER_MAX_ADR);
    writeFloat(0, RANGE_FINDER_MIN_ADR);
  #endif
  
  #if defined (UseGPSNavigator)
    writeFloat(missionNbPoint, GPS_MISSION_NB_POINT_ADR);
    writePID(GPSROLL_PID_IDX, GPSROLL_PID_GAIN_ADR);
    writePID(GPSPITCH_PID_IDX, GPSPITCH_PID_GAIN_ADR);
    writePID(GPSYAW_PID_IDX, GPSYAW_PID_GAIN_ADR);
    
    for (byte location = 0; location < MAX_WAYPOINTS; location++) {
      writeLong(waypoint[location].longitude, WAYPOINT_ADR[location].longitude);
      writeLong(waypoint[location].latitude, WAYPOINT_ADR[location].latitude);
      writeLong(waypoint[location].altitude, WAYPOINT_ADR[location].altitude);
    }       
  #endif

    // Camera Control
  #ifdef CameraControl
    writeFloat(cameraMode, CAMERAMODE_ADR);
    writeFloat(mCameraPitch, MCAMERAPITCH_ADR);
    writeFloat(mCameraRoll, MCAMERAROLL_ADR);    
    writeFloat(mCameraYaw, MCAMERAYAW_ADR);
    writeFloat(servoCenterPitch, SERVOCENTERPITCH_ADR);
    writeFloat(servoCenterRoll, SERVOCENTERROLL_ADR);
    writeFloat(servoCenterYaw, SERVOCENTERYAW_ADR);
    writeFloat(servoMinPitch, SERVOMINPITCH_ADR);
    writeFloat(servoMinRoll, SERVOMINROLL_ADR);
    writeFloat(servoMinYaw, SERVOMINYAW_ADR);
    writeFloat(servoMaxPitch, SERVOMAXPITCH_ADR);
    writeFloat(servoMaxRoll, SERVOMAXROLL_ADR);
    writeFloat(servoMaxYaw, SERVOMAXYAW_ADR);
  #endif 
  sei(); // Restart interrupts
}

void initSensorsZeroFromEEPROM() {
  // Gyro initialization from EEPROM
  gyroZero[XAXIS] = readFloat(GYRO_ROLL_ZERO_ADR);
  gyroZero[YAXIS] = readFloat(GYRO_PITCH_ZERO_ADR);
  gyroZero[ZAXIS] = readFloat(GYRO_YAW_ZERO_ADR);
  gyroSmoothFactor = readFloat(GYROSMOOTH_ADR);
//  gyroTempBiasSlope[XAXIS] = readFloat(GYRO_ROLL_TEMP_BIAS_SLOPE_ADR);
//  gyroTempBiasSlope[YAXIS] = readFloat(GYRO_PITCH_TEMP_BIAS_SLOPE_ADR);
//  gyroTempBiasSlope[ZAXIS] = readFloat(GYRO_YAW_TEMP_BIAS_SLOPE_ADR);
//  gyroTempBiasIntercept[XAXIS] = readFloat(GYRO_ROLL_TEMP_BIAS_INTERCEPT_ADR);
//  gyroTempBiasIntercept[YAXIS] = readFloat(GYRO_PITCH_TEMP_BIAS_INTERCEPT_ADR);
//  gyroTempBiasIntercept[ZAXIS] = readFloat(GYRO_YAW_TEMP_BIAS_INTERCEPT_ADR);
 
  // Accel initialization from EEPROM
  accelOneG = readFloat(ACCEL_1G_ADR);
  // Accel calibration
  accelScaleFactor[XAXIS] = readFloat(XAXIS_ACCEL_SCALE_FACTOR_ADR);
  runTimeAccelBias[XAXIS] = readFloat(XAXIS_ACCEL_BIAS_ADR);
  accelScaleFactor[YAXIS] = readFloat(YAXIS_ACCEL_SCALE_FACTOR_ADR);
  runTimeAccelBias[YAXIS] = readFloat(YAXIS_ACCEL_BIAS_ADR);
  accelScaleFactor[ZAXIS] = readFloat(ZAXIS_ACCEL_SCALE_FACTOR_ADR);
  runTimeAccelBias[ZAXIS] = readFloat(ZAXIS_ACCEL_BIAS_ADR);
}

void storeSensorsZeroToEEPROM() {
  // Store gyro data to EEPROM
  writeFloat(gyroZero[XAXIS], GYRO_ROLL_ZERO_ADR);
  writeFloat(gyroZero[YAXIS], GYRO_PITCH_ZERO_ADR);
  writeFloat(gyroZero[ZAXIS], GYRO_YAW_ZERO_ADR);
  writeFloat(gyroSmoothFactor, GYROSMOOTH_ADR);
  
  writeFloat(gyroTempBiasSlope[XAXIS], GYRO_ROLL_TEMP_BIAS_SLOPE_ADR);
  writeFloat(gyroTempBiasSlope[YAXIS], GYRO_PITCH_TEMP_BIAS_SLOPE_ADR);
  writeFloat(gyroTempBiasSlope[ZAXIS], GYRO_YAW_TEMP_BIAS_SLOPE_ADR);
  writeFloat(gyroTempBiasIntercept[XAXIS], GYRO_ROLL_TEMP_BIAS_INTERCEPT_ADR);
  writeFloat(gyroTempBiasIntercept[YAXIS], GYRO_PITCH_TEMP_BIAS_INTERCEPT_ADR);
  writeFloat(gyroTempBiasIntercept[ZAXIS], GYRO_YAW_TEMP_BIAS_INTERCEPT_ADR);
  
  // Store accel data to EEPROM
  writeFloat(accelOneG, ACCEL_1G_ADR);
  // Accel Cal
  writeFloat(accelScaleFactor[XAXIS], XAXIS_ACCEL_SCALE_FACTOR_ADR);
  writeFloat(runTimeAccelBias[XAXIS], XAXIS_ACCEL_BIAS_ADR);
  writeFloat(accelScaleFactor[YAXIS], YAXIS_ACCEL_SCALE_FACTOR_ADR);
  writeFloat(runTimeAccelBias[YAXIS], YAXIS_ACCEL_BIAS_ADR);
  writeFloat(accelScaleFactor[ZAXIS], ZAXIS_ACCEL_SCALE_FACTOR_ADR);
  writeFloat(runTimeAccelBias[ZAXIS], ZAXIS_ACCEL_BIAS_ADR);
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

