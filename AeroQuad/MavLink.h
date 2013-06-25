/*
  AeroQuad v3.x - July 2012
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

#ifndef _AQ_MAVLINK_H_
#define _AQ_MAVLINK_H_

#define MAV_COMPONENT_ID MAV_COMP_ID_IMU

#ifndef MAV_SYSTEM_ID
  #define MAV_SYSTEM_ID 100
#endif

// MavLink 1.0 DKP
#include "../mavlink/include/mavlink/v1.0/common/mavlink.h"

#include "AeroQuad.h"

int systemType = MAV_TYPE_GENERIC;
int autopilotType = MAV_AUTOPILOT_GENERIC;
uint16_t len;
int systemMode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
int systemStatus = MAV_STATE_BOOT;

// Variables for writing and sending parameters

enum parameterTypeIndicator
{
  P,
  I,
  D,
  windUpGuard,
  NONE
};

int indexCounter = 0;
int paramListPartIndicator = -1;

int parameterChangeIndicator = -1;
int parameterMatch = 0;
mavlink_param_set_t set;
char* key;

int parameterType = MAVLINK_TYPE_FLOAT;
int parameterListSize;

const char* parameterNameRateRollP = "RateMode_Roll_P";
const char* parameterNameRateRollI = "RateMode_Roll_I";
const char* parameterNameRateRollD = "RateMode_Roll_D";
const char* parameterNameRatePitchP = "RateMode_Pitch_P";
const char* parameterNameRatePitchI = "RateMode_Pitch_I";
const char* parameterNameRatePitchD = "RateMode_Pitch_D";
const char* parameterNameRateRotationFactor = "RateMode_RotFact";
const char* parameterNameAttitudeRollP = "AttMode_Roll_P";
const char* parameterNameAttitudeRollI = "AttMode_Roll_I";
const char* parameterNameAttitudeRollD = "AttMode_Roll_D";
const char* parameterNameAttitudePitchP = "AttMode_Pitch_P";
const char* parameterNameAttitudePitchI = "AttMode_Pitch_I";
const char* parameterNameAttitudePitchD = "AttMode_Pitch_D";
const char* parameterNameAttitudeGyroRollP = "AttMode_GRoll_P";
const char* parameterNameAttitudeGyroRollI = "AttMode_GRoll_I";
const char* parameterNameAttitudeGyroRollD = "AttMode_GRoll_D";
const char* parameterNameAttitudeGyroPitchP = "AttMode_GPitch_P";
const char* parameterNameAttitudeGyroPitchI = "AttMode_GPitch_I";
const char* parameterNameAttitudeGyroPitchD = "AttMode_GPitch_D";
const char* parameterNameYawP = "Yaw_P";
const char* parameterNameYawI = "Yaw_I";
const char* parameterNameYawD = "Yaw_D";
const char* parameterNameHeadingConfig = "HeadingHold_Conf";
const char* parameterNameHeadingP = "HeadingHold_P";
const char* parameterNameHeadingI = "HeadingHold_I";
const char* parameterNameHeadingD = "HeadingHold_D";
const char* parameterNameMinThrottle = "Misc_Minimum Throttle";
#if defined(BattMonitor)
  const char* parameterNameBattMonAlarmVoltage = "BattMon_AlarmVol";
  const char* parameterNameBattMonThrottleTarget = "BattMon_ThrTarge";
  const char* parameterNameBattMonGoingDownTime = "BattMon_DownTime";
#endif
#if defined(CameraControl)
  const char* parameterNameCamMode = "Cam_Mode";
  const char* parameterNameCamPitchScale = "Cam_Scale Pitch";
  const char* parameterNameCamRollScale = "Cam_Scale Roll";
  const char* parameterNameCamYawScale = "Cam_Scale Yaw";
  const char* parameterNameCamRollServoMiddle = "Cam_ServoPitchCen";
  const char* parameterNameCamPitchServoMiddle = "Cam_ServoRollCe";
  const char* parameterNameCamYawServoMiddle = "Cam_ServoYawCent";
  const char* parameterNameCamPitchServoMin = "Cam_ServoPitchMi";
  const char* parameterNameCamRollServoMin = "Cam_ServoRollMin";
  const char* parameterNameCamYawServoMin = "Cam_ServoYawMin";
  const char* parameterNameCamPitchServoMax = "Cam_ServoPitchMa";
  const char* parameterNameCamRollServoMax = "Cam_ServoRollMax";
  const char* parameterNameCamYawServoMax = "Cam_ServoYawMax";
#endif
#if defined(AltitudeHoldBaro) || defined(AltitudeHoldRangeFinder)
  const char* parameterNameAHminThrottleAdjust = "AltHold_MinAdjus";
  const char* parameterNameAHmaxThrottleAdjust = "AltHold_MaxAdjus";
  const char* parameterNameAHBumpValue = "AltHold_BumpValu";
  const char* parameterNameAHPanicValue = "AltHold_PanicVal";
#endif
#if defined(AltitudeHoldBaro)
  const char* parameterNameAHBaroSmooth = "Barometer_Smooth";
  const char* parameterNameBaroP = "Barometer_P";
  const char* parameterNameBaroI = "Barometer_I";
  const char* parameterNameBaroD = "Barometer_D";
  const char* parameterNameBaroWindUpGuard = "Barometer_WindUp";
  const char* parameterNameZDampeningP = "Z Dampening_P";
  const char* parameterNameZDampeningI = "Z Dampening_I";
  const char* parameterNameZDampeningD = "Z Dampening_D";
#endif
#if defined(AltitudeHoldRangeFinder)
  const char* parameterNameRangeFinderP = "Rangefinder_P";
  const char* parameterNameRangeFinderI = "Rangefinder_I";
  const char* parameterNameRangeFinderD = "Rangefinder_D";
  const char* parameterNameRangeFinderWindUpGuard = "Rangefinder_Wind";
#endif
#if defined(UseGPSNavigator)
  const char* parameterNameGPSRollP = "GPS Roll_P";
  const char* parameterNameGPSRollI = "GPS Roll_I";
  const char* parameterNameGPSRollD = "GPS Roll_D";
  const char* parameterNameGPSPitchP = "GPS Pitch_P";
  const char* parameterNameGPSPitchI = "GPS Pitch_I";
  const char* parameterNameGPSPitchD = "GPS Pitch_D";
  const char* parameterNameGPSYawP = "GPS Yaw_P";
  const char* parameterNameGPSYawI = "GPS Yaw_I";
  const char* parameterNameGPSYawD = "GPS Yaw_D";
#endif

parameterTypeIndicator paramIndicator = NONE;
float *parameterToBeChangedFloat;
byte *parameterToBeChangedByte;
int *parameterToBeChangedInt;
unsigned long *parameterToBeChangedULong;

static uint16_t millisecondsSinceBoot = 0;
long system_dropped_packets = 0;

mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
mavlink_status_t status;


void evaluateParameterListSize() {
	parameterListSize = 27;

  #if defined(AltitudeHoldBaro) && defined(AltitudeHoldRangeFinder) && defined(UseGPSNavigator)
    parameterListSize += 25;
  #endif

  #if defined(AltitudeHoldBaro) && defined(AltitudeHoldRangeFinder) && !defined(UseGPSNavigator)
    parameterListSize += 16;
  #endif

  #if defined(AltitudeHoldBaro) && !defined(AltitudeHoldRangeFinder) && !defined(UseGPSNavigator)
    parameterListSize += 12;
  #endif

  #if !defined(AltitudeHoldBaro) && defined(AltitudeHoldRangeFinder) && !defined(UseGPSNavigator)
    parameterListSize += 8;
  #endif

  #if defined(AltitudeHoldBaro) && !defined(AltitudeHoldRangeFinder) && defined(UseGPSNavigator)
    parameterListSize += 21;
  #endif

  #if defined(BattMonitor)
    parameterListSize += 3;
  #endif

  #if defined(CameraControl)
    parameterListSize += 13;
  #endif
}

void evaluateCopterType() {
  #if defined(triConfig)
	systemType = MAV_TYPE_TRICOPTER;
  #endif

  #if defined(quadXConfig) || defined(quadPlusConfig) || defined(quadY4Config)
	systemType = MAV_TYPE_QUADROTOR;
  #endif

  #if defined(hexPlusConfig) || defined(hexXConfig) || defined(hexY6Config) 
	systemType = MAV_TYPE_HEXAROTOR;
  #endif

  #if defined(octoX8Config) || defined(octoPlusConfig) || defined(octoXConfig)
	systemType = MAV_TYPE_OCTOROTOR;
  #endif
}

void initCommunication() {
  evaluateParameterListSize();
  evaluateCopterType();
}

uint32_t previousFlightTimeUpdate = 0;
void updateFlightTime() {

  uint16_t timeDiff = millis() - previousFlightTimeUpdate;
  previousFlightTimeUpdate += timeDiff;

  if (motorArmed) {
    millisecondsSinceBoot += timeDiff;
  }
}

void sendSerialHeartbeat() {

  systemMode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

  if (flightMode == ATTITUDE_FLIGHT_MODE) {
    systemMode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
  }

  #if defined(UseGPSNavigator)
    if (navigationState == ON || positionHoldState == ON) {
      systemMode |= MAV_MODE_FLAG_GUIDED_ENABLED;
    }
  #endif

  if (motorArmed) {
    systemMode |= MAV_MODE_FLAG_SAFETY_ARMED;
    systemStatus = MAV_STATE_ACTIVE;
  }
  else {
    systemStatus = MAV_STATE_STANDBY;
  }

  mavlink_msg_heartbeat_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, systemType, autopilotType, systemMode, 0, systemStatus);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}


void sendSerialRawIMU() {
  #if defined(HeadingMagHold)
    mavlink_msg_raw_imu_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0, meterPerSecSec[XAXIS], meterPerSecSec[YAXIS], meterPerSecSec[ZAXIS], gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], getMagnetometerRawData(XAXIS), getMagnetometerRawData(YAXIS), getMagnetometerRawData(ZAXIS));
  #else
    mavlink_msg_raw_imu_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0, meterPerSecSec[XAXIS], meterPerSecSec[YAXIS], meterPerSecSec[ZAXIS], gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], 0, 0, 0);
  #endif
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}


void sendSerialAttitude() {
  mavlink_msg_attitude_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, kinematicsAngle[XAXIS], kinematicsAngle[YAXIS], kinematicsAngle[ZAXIS], 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}

void sendSerialHudData() {
  #if defined(HeadingMagHold)
    #if defined(AltitudeHoldBaro)
      mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360, (receiverCommand[receiverChannelMap[THROTTLE]]-1000)/10, getBaroAltitude(), 0.0);
    #else
      mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360, (receiverCommand[receiverChannelMap[THROTTLE]]-1000)/10, 0, 0.0);
    #endif
  #else
    #if defined(AltitudeHoldBaro)
      mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, 0, (receiverCommand[receiverChannelMap[THROTTLE]]-1000)/10, getBaroAltitude(), 0.0);
    #else
      mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, 0, (receiverCommand[receiverChannelMap[THROTTLE]]-1000)/10, 0, 0.0);
    #endif
  #endif
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}

void sendSerialGpsPostion() {
  #if defined(UseGPS)
    if (haveAGpsLock())
    {
      #if defined(AltitudeHoldBaro)
        mavlink_msg_global_position_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, currentPosition.latitude, currentPosition.longitude, getGpsAltitude() * 10, (getGpsAltitude() - baroGroundAltitude * 100) * 10 , 0, 0, 0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360);
      #else
        mavlink_msg_global_position_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, currentPosition.latitude, currentPosition.longitude, getGpsAltitude() * 10, getGpsAltitude() * 10 , 0, 0, 0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360);
      #endif
      len = mavlink_msg_to_send_buffer(buf, &msg);
      SERIAL_PORT.write(buf, len);
    }
  #endif
}

void sendSerialRawPressure() {
  #if defined(AltitudeHoldBaro)
    mavlink_msg_raw_pressure_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, readRawPressure(), 0,0, readRawTemperature());
    len = mavlink_msg_to_send_buffer(buf, &msg);
    SERIAL_PORT.write(buf, len);
  #endif
}

void sendSerialRcRaw() {
  #if defined(UseRSSIFaileSafe)
    mavlink_msg_rc_channels_raw_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, 0, receiverCommand[receiverChannelMap[THROTTLE]], receiverCommand[receiverChannelMap[XAXIS]], receiverCommand[receiverChannelMap[YAXIS]], receiverCommand[receiverChannelMap[ZAXIS]], receiverCommand[receiverChannelMap[MODE]], receiverCommand[receiverChannelMap[AUX1]], receiverCommand[receiverChannelMap[AUX2]], receiverCommand[receiverChannelMap[AUX3]], rssiRawValue * 2.55);
  #else
    mavlink_msg_rc_channels_raw_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, 0, receiverCommand[receiverChannelMap[THROTTLE]], receiverCommand[receiverChannelMap[XAXIS]], receiverCommand[receiverChannelMap[YAXIS]], receiverCommand[receiverChannelMap[ZAXIS]], receiverCommand[receiverChannelMap[MODE]], receiverCommand[receiverChannelMap[AUX1]], receiverCommand[receiverChannelMap[AUX2]], receiverCommand[receiverChannelMap[AUX3]], 0);
  #endif
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}

void sendSerialSysStatus() {
  uint32_t controlSensorsPresent = 0;
  uint32_t controlSensorEnabled;
  uint32_t controlSensorsHealthy;

  // first what sensors/controllers we have
  if (GYRO_DETECTED) {
    controlSensorsPresent |= (1<<0); // 3D gyro present
  }
  if (ACCEL_DETECTED) {
    controlSensorsPresent |= (1<<1); // 3D accelerometer present
  }
  #if defined(HeadingMagHold)
    if (MAG_DETECTED) {
      controlSensorsPresent |= (1<<2); // compass present
    }
  #endif
  #if defined(AltitudeHoldBaro)
    if (BARO_DETECTED) {
      controlSensorsPresent |= (1<<3); // absolute pressure sensor present
    }
  #endif
  #if defined(UseGPS)
    if (gpsData.state > 0) {
      controlSensorsPresent |= (1<<5); // GPS present
    }
  #endif

  controlSensorsPresent |= (1<<10); // 3D angular rate control
  if(ACCEL_DETECTED) {
	controlSensorsPresent |= (1<<11); // attitude stabilisation
  }
  controlSensorsPresent |= (1<<12); // yaw position
  #if defined(AltitudeHoldBaro) || defined(AltitudeHoldRangeFinder)
    controlSensorsPresent |= (1<<13); // altitude control
  #endif
  #if defined(UseGPSNavigator)
    controlSensorsPresent |= (1<<14); // X/Y position control
  #endif
  controlSensorsPresent |= (1<<15); // motor control

  // now what sensors/controllers are enabled
  // first the sensors
  controlSensorEnabled = controlSensorsPresent & 0x1FF;

  // now the controllers
  controlSensorEnabled = controlSensorsPresent & 0x1FF;

  controlSensorEnabled |= (1<<10); // 3D angular rate control
  if (flightMode == ATTITUDE_FLIGHT_MODE) {
    controlSensorEnabled |= (1<<11); // attitude stabilisation
  }
  if (headingHoldConfig == ON) {
	  controlSensorEnabled |= (1<<12); // yaw position
  }
  #if defined(AltitudeHoldBaro) || defined(AltitudeHoldRangeFinder)
    if (altitudeHoldState == ON) {
      controlSensorEnabled |= (1<<13); // altitude control
    }
  #endif
  #if defined(UseGPSNavigator)
	if (positionHoldState == ON || navigationState == ON) {
		controlSensorEnabled |= (1<<14); // X/Y position control
	}
  #endif
  controlSensorEnabled |= (1<<15); // motor control

  // at the moment all sensors/controllers are assumed healthy
  controlSensorsHealthy = controlSensorsPresent;

  #if defined(BattMonitor)
    mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, controlSensorsPresent, controlSensorEnabled, controlSensorsHealthy, 0, batteryData[0].voltage * 10, (int)(batteryData[0].current*1000), -1, system_dropped_packets, 0, 0, 0, 0, 0);
  #else
    mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, controlSensorsPresent, controlSensorEnabled, controlSensorsHealthy, 0, 0, 0, 0, system_dropped_packets, 0, 0, 0, 0, 0);
  #endif

  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}

void sendSerialPID(int IDPid, const char id_p[], const char id_i[], const char id_d[], const char id_windUp[], int listsize, int index) {

  int counter = 0;
  if (id_p != 0) {
    mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, id_p, PID[IDPid].P, parameterType, listsize, index);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    SERIAL_PORT.write(buf, len);
    counter++;
  }

  if (id_i != 0) {
    mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, id_i, PID[IDPid].I, parameterType, listsize, index + counter);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    SERIAL_PORT.write(buf, len);
    counter++;
  }

  if (id_d != 0) {
    mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, id_d, PID[IDPid].D, parameterType, listsize, index + counter);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    SERIAL_PORT.write(buf, len);
    counter++;
  }

  if (id_windUp != 0) {
    mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, id_windUp, PID[IDPid].windupGuard, parameterType, listsize, index + counter);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    SERIAL_PORT.write(buf, len);
  }
}

void sendSerialParameter(float parameterID, const char parameterName[], int listsize, int index) {
  mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)parameterName, parameterID, parameterType, listsize, index);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}

void sendSerialParameter(int parameterID, const char parameterName[], int listsize, int index) {
  mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)parameterName, parameterID, parameterType, listsize, index);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}

void sendSerialParameter(byte parameterID, const char parameterName[], int listsize, int index) {
  mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)parameterName, parameterID, parameterType, listsize, index);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}

void sendSerialParameter(unsigned long parameterID, const char parameterName[], int listsize, int index) {
  mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)parameterName, parameterID, parameterType, listsize, index);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}

void sendParameterListPart1() {
  sendSerialPID(RATE_XAXIS_PID_IDX, parameterNameRateRollP, parameterNameRateRollI, parameterNameRateRollD, 0, parameterListSize, indexCounter);
  indexCounter += 3;

  sendSerialPID(RATE_YAXIS_PID_IDX, parameterNameRatePitchP, parameterNameRatePitchI, parameterNameRatePitchD, 0, parameterListSize, indexCounter);
  indexCounter += 3;
  
  sendSerialParameter(rotationSpeedFactor, parameterNameRateRotationFactor, parameterListSize, indexCounter);
  indexCounter++;

  sendSerialPID(ATTITUDE_XAXIS_PID_IDX, parameterNameAttitudeRollP, parameterNameAttitudeRollI, parameterNameAttitudeRollD, 0, parameterListSize, indexCounter);
  indexCounter += 3;

  sendSerialPID(ATTITUDE_YAXIS_PID_IDX, parameterNameAttitudePitchP, parameterNameAttitudePitchI, parameterNameAttitudePitchD, 0, parameterListSize, indexCounter);
  indexCounter += 3;

  sendSerialPID(ATTITUDE_GYRO_XAXIS_PID_IDX, parameterNameAttitudeGyroRollP, parameterNameAttitudeGyroRollI, parameterNameAttitudeGyroRollD, 0, parameterListSize, indexCounter);
  indexCounter += 3;

  sendSerialPID(ATTITUDE_GYRO_YAXIS_PID_IDX, parameterNameAttitudeGyroPitchP, parameterNameAttitudeGyroPitchI, parameterNameAttitudeGyroPitchD, 0, parameterListSize, indexCounter);
  indexCounter += 3;

  sendSerialPID(ZAXIS_PID_IDX, parameterNameYawP, parameterNameYawI, parameterNameYawD, 0, parameterListSize, indexCounter);
  indexCounter += 3;
  
  sendSerialParameter(headingHoldConfig, parameterNameHeadingConfig, parameterListSize, indexCounter);
  indexCounter++;

  sendSerialPID(HEADING_HOLD_PID_IDX, parameterNameHeadingP, parameterNameHeadingI, parameterNameHeadingD, 0, parameterListSize, indexCounter);
  indexCounter += 3;
}

void sendParameterListPart2() {
  sendSerialParameter(minArmedThrottle, parameterNameMinThrottle, parameterListSize, indexCounter);
  indexCounter++;

  #if defined(BattMonitor)
    sendSerialParameter(batteryMonitorAlarmVoltage, parameterNameBattMonAlarmVoltage, parameterListSize, indexCounter);
    indexCounter++;

    sendSerialParameter(batteryMonitorThrottleTarget, parameterNameBattMonThrottleTarget, parameterListSize, indexCounter);
    indexCounter++;

    sendSerialParameter(batteryMonitorGoingDownTime, parameterNameBattMonGoingDownTime, parameterListSize, indexCounter);
    indexCounter++;
  #endif

  #if defined(CameraControl)
    sendSerialParameter(cameraMode, parameterNameCamMode, parameterListSize, indexCounter);
    indexCounter++;

    sendSerialParameter(mCameraPitch, parameterNameCamPitchScale , parameterListSize, indexCounter);
    indexCounter++;

    sendSerialParameter(mCameraRoll, parameterNameCamRollScale , parameterListSize, indexCounter);
    indexCounter++;

    sendSerialParameter(mCameraYaw, parameterNameCamYawScale, parameterListSize , indexCounter);
    indexCounter++;

    sendSerialParameter(servoCenterPitch, parameterNameCamPitchServoMiddle  , parameterListSize, indexCounter);
    indexCounter++;

    sendSerialParameter(servoCenterRoll, parameterNameCamRollServoMiddle , parameterListSize, indexCounter);
    indexCounter++;

    sendSerialParameter(servoCenterYaw, parameterNameCamYawServoMiddle , parameterListSize, indexCounter);
    indexCounter++;

    sendSerialParameter(servoMinPitch, parameterNameCamPitchServoMin , parameterListSize, indexCounter);
    indexCounter++;

    sendSerialParameter(servoMinRoll, parameterNameCamRollServoMin , parameterListSize, indexCounter);
    indexCounter++;

    sendSerialParameter(servoMinYaw, parameterNameCamYawServoMin , parameterListSize, indexCounter);
    indexCounter++;

    sendSerialParameter(servoMaxPitch, parameterNameCamPitchServoMax , parameterListSize, indexCounter);
    indexCounter++;

    sendSerialParameter(servoMaxRoll, parameterNameCamRollServoMax , parameterListSize, indexCounter);
    indexCounter++;

    sendSerialParameter(servoMaxYaw, parameterNameCamYawServoMax , parameterListSize, indexCounter);
    indexCounter++;
  #endif
}

void sendParameterListPart3() {
  #if defined(AltitudeHoldBaro) || defined(AltitudeHoldRangeFinder)
    sendSerialParameter(minThrottleAdjust, parameterNameAHminThrottleAdjust , parameterListSize, indexCounter);
    indexCounter++;

    sendSerialParameter(maxThrottleAdjust, parameterNameAHmaxThrottleAdjust , parameterListSize, indexCounter);
    indexCounter++;

    sendSerialParameter(altitudeHoldBump, parameterNameAHBumpValue , parameterListSize, indexCounter);
    indexCounter++;

    sendSerialParameter(altitudeHoldPanicStickMovement, parameterNameAHPanicValue , parameterListSize, indexCounter);
    indexCounter++;
  #endif

  #if defined(AltitudeHoldBaro)  && !defined(AltitudeHoldRangeFinder)
    sendSerialParameter(baroSmoothFactor, parameterNameAHBaroSmooth , parameterListSize, indexCounter);
    indexCounter++;

    sendSerialPID(BARO_ALTITUDE_HOLD_PID_IDX, parameterNameBaroP , parameterNameBaroI , parameterNameBaroD , parameterNameBaroWindUpGuard , parameterListSize, indexCounter);
    indexCounter += 4;

    sendSerialPID(ZDAMPENING_PID_IDX, parameterNameZDampeningP , parameterNameZDampeningI , parameterNameZDampeningD , 0, parameterListSize, indexCounter);
    indexCounter += 3;
  #endif

  #if defined(AltitudeHoldRangeFinder) && defined(AltitudeHoldBaro)
    sendSerialParameter(baroSmoothFactor, parameterNameAHBaroSmooth , parameterListSize, indexCounter);
    indexCounter++;

    sendSerialPID(BARO_ALTITUDE_HOLD_PID_IDX, parameterNameBaroP , parameterNameBaroI , parameterNameBaroD , parameterNameBaroWindUpGuard , parameterListSize, indexCounter);
    indexCounter += 4;

    sendSerialPID(ZDAMPENING_PID_IDX, parameterNameZDampeningP , parameterNameZDampeningI , parameterNameZDampeningD , 0, parameterListSize, indexCounter);
    indexCounter += 3;

    sendSerialPID(SONAR_ALTITUDE_HOLD_PID_IDX, parameterNameRangeFinderP , parameterNameRangeFinderI , parameterNameRangeFinderD , parameterNameRangeFinderWindUpGuard , parameterListSize, indexCounter);
    indexCounter += 4;
  #endif
  
  #if defined(AltitudeHoldRangeFinder) && !defined(AltitudeHoldBaro)
    sendSerialPID(SONAR_ALTITUDE_HOLD_PID_IDX, parameterNameRangeFinderP , parameterNameRangeFinderI , parameterNameRangeFinderD , parameterNameRangeFinderWindUpGuard , parameterListSize, indexCounter);
    indexCounter += 4;
  #endif
}

void sendParameterListPart4() {
  #if defined(UseGPSNavigator) && defined(AltitudeHoldBaro)
    sendSerialPID(GPSROLL_PID_IDX, parameterNameGPSRollP , parameterNameGPSRollI , parameterNameGPSRollD , 0, parameterListSize, indexCounter);
    indexCounter += 3;

    sendSerialPID(GPSPITCH_PID_IDX, parameterNameGPSPitchP , parameterNameGPSPitchI , parameterNameGPSPitchD , 0, parameterListSize, indexCounter);
    indexCounter += 3;

    sendSerialPID(GPSYAW_PID_IDX, parameterNameGPSYawP , parameterNameGPSYawI , parameterNameGPSYawD , 0, parameterListSize, indexCounter);
    indexCounter += 3;
  #endif
}

bool checkParameterMatch(const char* parameterName, char* key) {
  for (uint16_t j = 0; parameterName[j] != '\0'; j++) {
    if (((char) (parameterName[j])) != (char) (key[j]))	{
      return false;
    }
  }
  return true;
}

int findParameter(char* key) {
  paramIndicator = NONE;
  parameterToBeChangedFloat = NULL;
  parameterToBeChangedByte = NULL;
  parameterToBeChangedInt = NULL;
  parameterToBeChangedULong = NULL;

  if (checkParameterMatch(parameterNameRateRollP, key)) {
    paramIndicator = P;
    return RATE_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameRateRollI, key)) {
    paramIndicator = I;
    return RATE_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameRateRollD, key)) {
    paramIndicator = D;
    return RATE_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameRatePitchP, key)) {
    paramIndicator = P;
    return RATE_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameRatePitchI, key)) {
    paramIndicator = I;
    return RATE_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameRatePitchD, key)) {
    paramIndicator = D;
    return RATE_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameRateRotationFactor, key)) {
    paramIndicator = D;
    parameterToBeChangedFloat = &rotationSpeedFactor;
    return -1;
  }
  if (checkParameterMatch(parameterNameAttitudeRollP, key)) {
    paramIndicator = P;
    return ATTITUDE_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeRollI, key)) {
    paramIndicator = I;
    return ATTITUDE_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeRollD, key)) {
    paramIndicator = D;
    return ATTITUDE_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudePitchP, key)) {
    paramIndicator = P;
    return ATTITUDE_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudePitchI, key)) {
    paramIndicator = I;
    return ATTITUDE_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudePitchD, key)) {
    paramIndicator = D;
    return ATTITUDE_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeGyroRollP, key)) {
    paramIndicator = P;
    return ATTITUDE_GYRO_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeGyroRollI, key)) {
    paramIndicator = I;
    return ATTITUDE_GYRO_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeGyroRollD, key)) {
    paramIndicator = D;
    return ATTITUDE_GYRO_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeGyroPitchP, key)) {
    paramIndicator = P;
    return ATTITUDE_GYRO_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeGyroPitchI, key)) {
    paramIndicator = I;
    return ATTITUDE_GYRO_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeGyroPitchD, key)) {
    paramIndicator = D;
    return ATTITUDE_GYRO_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameHeadingP, key)) {
    paramIndicator = P;
    return HEADING_HOLD_PID_IDX;
  }
  if (checkParameterMatch(parameterNameHeadingI, key)) {
    paramIndicator = I;
    return HEADING_HOLD_PID_IDX;
  }
  if (checkParameterMatch(parameterNameHeadingD, key)) {
    paramIndicator = D;
    return HEADING_HOLD_PID_IDX;
  }
  if (checkParameterMatch(parameterNameHeadingConfig, key)) {
    paramIndicator = NONE;
    parameterToBeChangedByte = &headingHoldConfig;
    return -1;
  }
  if (checkParameterMatch(parameterNameMinThrottle, key)) {
    paramIndicator = NONE;
    parameterToBeChangedInt = &minArmedThrottle;
    return -1;
  }

  #if defined(BattMonitor)
    if (checkParameterMatch(parameterNameBattMonAlarmVoltage, key)) {
      paramIndicator = NONE;
      parameterToBeChangedFloat = &batteryMonitorAlarmVoltage;
	  setBatteryCellVoltageThreshold(set.param_value);
      return -1;
    }
    if (checkParameterMatch(parameterNameBattMonThrottleTarget, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &batteryMonitorThrottleTarget;
      return -1;
    }
    if (checkParameterMatch(parameterNameBattMonGoingDownTime, key)) {
      paramIndicator = NONE;
      parameterToBeChangedULong = &batteryMonitorGoingDownTime;
      return -1;
    }
  #endif

  #if defined(CameraControl)
    if (checkParameterMatch(parameterNameCamMode, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &cameraMode;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamPitchScale, key)) {
      paramIndicator = NONE;
      parameterToBeChangedFloat = &mCameraPitch;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamRollScale, key)) {
      paramIndicator = NONE;
      parameterToBeChangedFloat = &mCameraRoll;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamYawScale, key)) {
      paramIndicator = NONE;
      parameterToBeChangedFloat = &mCameraYaw;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamPitchServoMin, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &servoMinPitch;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamRollServoMin, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &servoMinRoll;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamYawServoMin, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &servoMinYaw;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamPitchServoMax, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &servoMaxPitch;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamRollServoMax, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &servoMaxRoll;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamYawServoMax, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &servoMaxYaw;
      return -1;
    }
  #endif

  #if defined(AltitudeHoldBaro) || defined(AltitudeHoldRangeFinder)
    if (checkParameterMatch(parameterNameAHminThrottleAdjust, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &minThrottleAdjust;
      return -1;
    }
    if (checkParameterMatch(parameterNameAHmaxThrottleAdjust, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &maxThrottleAdjust;
      return -1;
    }
    if (checkParameterMatch(parameterNameAHBumpValue, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &altitudeHoldBump;
      return -1;
    }
    if (checkParameterMatch(parameterNameAHPanicValue, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &altitudeHoldPanicStickMovement;
      return -1;
    }
  #endif

  #if defined(AltitudeHoldBaro)
    if (checkParameterMatch(parameterNameAHBaroSmooth, key)) {
      paramIndicator = NONE;
      parameterToBeChangedFloat = &baroSmoothFactor;
      return -1;
    }
    if (checkParameterMatch(parameterNameBaroP, key)) {
      paramIndicator = P;
      return BARO_ALTITUDE_HOLD_PID_IDX;
    }
    if (checkParameterMatch(parameterNameBaroI, key)) {
      paramIndicator = I;
      return BARO_ALTITUDE_HOLD_PID_IDX;
    }
    if (checkParameterMatch(parameterNameBaroD, key)) {
      paramIndicator = D;
      return BARO_ALTITUDE_HOLD_PID_IDX;
    }
    if (checkParameterMatch(parameterNameBaroWindUpGuard, key)) {
      paramIndicator = windUpGuard;
      return BARO_ALTITUDE_HOLD_PID_IDX;
    }
    if (checkParameterMatch(parameterNameZDampeningP, key)) {
      paramIndicator = P;
      return ZDAMPENING_PID_IDX;
    }
    if (checkParameterMatch(parameterNameZDampeningI, key)) {
      paramIndicator = I;
      return ZDAMPENING_PID_IDX;
    }
    if (checkParameterMatch(parameterNameZDampeningD, key)) {
      paramIndicator = D;
      return ZDAMPENING_PID_IDX;
    }
  #endif

  #if defined(AltitudeHoldRangeFinder)
    if (checkParameterMatch(parameterNameRangeFinderP, key)) {
      paramIndicator = P;
      return SONAR_ALTITUDE_HOLD_PID_IDX;
    }
    if (checkParameterMatch(parameterNameRangeFinderI, key)) {
      paramIndicator = I;
      return SONAR_ALTITUDE_HOLD_PID_IDX;
    }
    if (checkParameterMatch(parameterNameRangeFinderD, key)) {
      paramIndicator = D;
      return SONAR_ALTITUDE_HOLD_PID_IDX;
    }
    if (checkParameterMatch(parameterNameRangeFinderWindUpGuard, key)) {
      paramIndicator = windUpGuard;
      return SONAR_ALTITUDE_HOLD_PID_IDX;
    }
  #endif

  #if defined(UseGPSNavigator)
    if (checkParameterMatch(parameterNameGPSRollP, key)) {
      paramIndicator = P;
      return GPSROLL_PID_IDX;
    }
    if (checkParameterMatch(parameterNameGPSRollI, key)) {
      paramIndicator = I;
      return GPSROLL_PID_IDX;
    }
    if (checkParameterMatch(parameterNameGPSRollD, key)) {
      paramIndicator = D;
      return GPSROLL_PID_IDX;
    }
    if (checkParameterMatch(parameterNameGPSPitchP, key)) {
      paramIndicator = P;
      return GPSPITCH_PID_IDX;
    }
    if (checkParameterMatch(parameterNameGPSPitchI, key)) {
      paramIndicator = I;
      return GPSPITCH_PID_IDX;
    }
    if (checkParameterMatch(parameterNameGPSPitchD, key)) {
      paramIndicator = D;
      return GPSPITCH_PID_IDX;
    }
    if (checkParameterMatch(parameterNameGPSYawP, key)) {
      paramIndicator = P;
      return GPSYAW_PID_IDX;
    }
    if (checkParameterMatch(parameterNameGPSYawI, key)) {
      paramIndicator = I;
      return GPSYAW_PID_IDX;
    }
    if (checkParameterMatch(parameterNameGPSYawD, key)) {
      paramIndicator = D;
      return GPSYAW_PID_IDX;
    }
  #endif

  return 0;
}

void changeAndSendParameter() {
  if(parameterChangeIndicator == 0) {
    // Only write and emit changes if there is actually a difference AND only write if new value is NOT "not-a-number" AND is NOT infinit

    if(parameterMatch != 0) {
      if (paramIndicator == P) {
        if (PID[parameterMatch].P != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
          PID[parameterMatch].P = set.param_value;
          writeEEPROM();
          // Report back new value
          mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].P, parameterType, parameterListSize, -1);
          len = mavlink_msg_to_send_buffer(buf, &msg);
          SERIAL_PORT.write(buf, len);
        }
      }

      else if (paramIndicator == I) {
        if (PID[parameterMatch].I != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
          PID[parameterMatch].I = set.param_value;
          writeEEPROM();
          // Report back new value
          mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].I, parameterType, parameterListSize, -1);
          len = mavlink_msg_to_send_buffer(buf, &msg);
          SERIAL_PORT.write(buf, len);
        }
      }

      else if (paramIndicator == D) {
        if (PID[parameterMatch].D != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
          PID[parameterMatch].D = set.param_value;
          writeEEPROM();
          // Report back new value
          mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].D, parameterType, parameterListSize, -1);
          len = mavlink_msg_to_send_buffer(buf, &msg);
          SERIAL_PORT.write(buf, len);
        }
      }

      else if (paramIndicator == NONE) {
        if (parameterToBeChangedFloat != NULL) {
          if (*parameterToBeChangedFloat != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
            *parameterToBeChangedFloat = set.param_value;
            writeEEPROM();
            // Report back new value
            mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, *parameterToBeChangedFloat, parameterType, parameterListSize, -1);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            SERIAL_PORT.write(buf, len);
          }
        }
        else if (parameterToBeChangedByte != NULL) {
          if (*parameterToBeChangedByte != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
            *parameterToBeChangedByte = set.param_value;
            writeEEPROM();
            // Report back new value
            mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, *parameterToBeChangedByte, parameterType, parameterListSize, -1);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            SERIAL_PORT.write(buf, len);
          }
        }
        else if (parameterToBeChangedInt != NULL) {
          if (*parameterToBeChangedInt != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
            *parameterToBeChangedInt = set.param_value;
            writeEEPROM();
            // Report back new value
            mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, *parameterToBeChangedInt, parameterType, parameterListSize, -1);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            SERIAL_PORT.write(buf, len);
          }
        }
        else if (parameterToBeChangedULong != NULL) {
          if (*parameterToBeChangedULong != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
            *parameterToBeChangedULong = set.param_value;
            writeEEPROM();
            // Report back new value
            mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, *parameterToBeChangedULong, parameterType, parameterListSize, -1);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            SERIAL_PORT.write(buf, len);
          }
        }
        parameterChangeIndicator = -1;
      }
    }
  }
}

void readSerialCommand() {
  while(SERIAL_PORT.available() > 0) {

    uint8_t c = SERIAL_PORT.read();
    //try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG:  {
          uint8_t result = 0;
          uint8_t command = mavlink_msg_command_long_get_command(&msg);
		  
	  switch(command) {
	  // (yet) unsupported commands/features
	    case MAV_CMD_NAV_WAYPOINT: //16
	    case MAV_CMD_NAV_LOITER_UNLIM: //17
	    case MAV_CMD_NAV_LOITER_TURNS: //18
	    case MAV_CMD_NAV_LOITER_TIME: //19
	    case MAV_CMD_NAV_RETURN_TO_LAUNCH: //20
	    case MAV_CMD_NAV_LAND: //21
	    case MAV_CMD_NAV_TAKEOFF: //22
	    case MAV_CMD_NAV_ROI: //80
	    case MAV_CMD_NAV_PATHPLANNING: //81
	    case MAV_CMD_NAV_LAST: //95
	    case MAV_CMD_CONDITION_DELAY: //112
	    case MAV_CMD_CONDITION_CHANGE_ALT: //113
	    case MAV_CMD_CONDITION_DISTANCE: //114
	    case MAV_CMD_CONDITION_YAW: //115
	    case MAV_CMD_CONDITION_LAST: //159
	    case MAV_CMD_DO_SET_MODE: //176
	    case MAV_CMD_DO_JUMP: //177
	    case MAV_CMD_DO_CHANGE_SPEED: //178
	    case MAV_CMD_DO_SET_PARAMETER: //180
	    case MAV_CMD_DO_SET_RELAY: //181
	    case MAV_CMD_DO_REPEAT_RELAY: //182
	    case MAV_CMD_DO_SET_SERVO: //183
	    case MAV_CMD_DO_REPEAT_SERVO: //184
	    case MAV_CMD_DO_CONTROL_VIDEO: //200
	    case MAV_CMD_DO_LAST: //240
	    case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS: //242
	    case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: //246
	    case MAV_CMD_OVERRIDE_GOTO: //252
	    case MAV_CMD_MISSION_START: //300
	      result = MAV_RESULT_UNSUPPORTED;
	      break;
		  
	    case MAV_CMD_COMPONENT_ARM_DISARM: //400, needs some security checks to prevent accidential arming/disarming
			
	      //  if (mavlink_msg_command_long_get_param1(&msg) == 1.0) {
	      //	motorArmed = ON;
	      //  }
	      //  else if (mavlink_msg_command_long_get_param1(&msg) == 0.0) {
	      //    motorArmed = OFF;
	      //  }
	      //  result = MAV_RESULT_ACCEPTED;
	      result = MAV_RESULT_UNSUPPORTED;
	      break;
		
	    case MAV_CMD_DO_SET_HOME: //179
            #if defined(UseGPSNavigator)
              if (mavlink_msg_command_long_get_param1(&msg) == 1.0f) {
                homePosition = currentPosition;
              }
              else {
                homePosition.latitude = mavlink_msg_command_long_get_param5(&msg);
                homePosition.longitude = mavlink_msg_command_long_get_param6(&msg);
                homePosition.altitude = mavlink_msg_command_long_get_param7(&msg);
              }
              result = 	MAV_RESULT_ACCEPTED;
            #else
              result = 	MAV_RESULT_UNSUPPORTED;
            #endif
              break;
            
			
	    case MAV_CMD_PREFLIGHT_CALIBRATION: //241
              if (!motorArmed) {
                if (mavlink_msg_command_long_get_param1(&msg) == 1.0f) {
                  calibrateGyro();
                  storeSensorsZeroToEEPROM();
                  result = MAV_RESULT_ACCEPTED;
                }
                if (mavlink_msg_command_long_get_param2(&msg) == 1.0f) {
                  computeAccelBias();
                  storeSensorsZeroToEEPROM();
                  calibrateKinematics();
                  zeroIntegralError();
                  result = MAV_RESULT_ACCEPTED;
                }
              }
              else result = MAV_RESULT_TEMPORARILY_REJECTED;
	      break;
			  
            case MAV_CMD_PREFLIGHT_STORAGE: //245
              if (!motorArmed) {
                if (mavlink_msg_command_long_get_param1(&msg) == 0.0f) {
		  paramListPartIndicator = indexCounter = 0;
		}
		else if (mavlink_msg_command_long_get_param1(&msg) == 1.0f) {
		  mavlink_msg_param_set_decode(&msg, &set);
		  key = (char*) set.param_id;
		  parameterMatch = findParameter(key);
		  parameterChangeIndicator = 0;
		}
	      }	
	      break;
          }
		  
          mavlink_msg_command_ack_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, command, result);
          len = mavlink_msg_to_send_buffer(buf, &msg);
          SERIAL_PORT.write(buf, len);
        }
        break;

      case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
          paramListPartIndicator = indexCounter = 0;
        }
        break;

      case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
          mavlink_param_request_read_t read;
          mavlink_msg_param_request_read_decode(&msg, &read);

          key = (char*) read.param_id;

          int parameterMatch = findParameter(key);

          if(parameterMatch != 0) {
            if (paramIndicator == P) {
              mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].P, parameterType, parameterListSize, -1);
              len = mavlink_msg_to_send_buffer(buf, &msg);
              SERIAL_PORT.write(buf, len);
            }
            else if (paramIndicator == I) {
              mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].I, parameterType, parameterListSize, -1);
              len = mavlink_msg_to_send_buffer(buf, &msg);
              SERIAL_PORT.write(buf, len);
            }
            else if (paramIndicator == D) {
              mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].D, parameterType, parameterListSize, -1);
              len = mavlink_msg_to_send_buffer(buf, &msg);
              SERIAL_PORT.write(buf, len);
            }
            else if (paramIndicator == NONE) {
              mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, parameterMatch, parameterType, parameterListSize, -1);
              len = mavlink_msg_to_send_buffer(buf, &msg);
              SERIAL_PORT.write(buf, len);
            }
          }
        }
        break;

      case MAVLINK_MSG_ID_PARAM_SET:
        {
          if(!motorArmed) { // added for security reason, as the software is shortly blocked by this command
            mavlink_msg_param_set_decode(&msg, &set);
            key = (char*) set.param_id;
            parameterMatch = findParameter(key);
            parameterChangeIndicator = 0;
          }
        }
        break;

      case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: { //TODO needs to be tested
        #if defined(UseGPSNavigator)
          mavlink_msg_mission_count_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAX_WAYPOINTS);
          len = mavlink_msg_to_send_buffer(buf, &msg);
          SERIAL_PORT.write(buf, len);

          for (byte index = 0; index < MAX_WAYPOINTS; index++) {
            if (index != missionNbPoint) {
				mavlink_msg_mission_item_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SYSTEM_ID, MAV_COMPONENT_ID, index, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, 0, 1, 0, MIN_DISTANCE_TO_REACHED, 0, 0, waypoint[index].longitude, waypoint[index].latitude, waypoint[index].altitude);
			}
            else {
				mavlink_msg_mission_item_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SYSTEM_ID, MAV_COMPONENT_ID, index, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, 1, 1, 0, MIN_DISTANCE_TO_REACHED, 0, 0, waypoint[index].longitude, waypoint[index].latitude, waypoint[index].altitude);
			}
            len = mavlink_msg_to_send_buffer(buf, &msg);
            SERIAL_PORT.write(buf, len);
          }
        #endif
        }
        break;

      default:
        break;
      }
    }
  }
  system_dropped_packets += status.packet_rx_drop_count;
}

void sendQueuedParameters() {
  if(paramListPartIndicator >= 0 && paramListPartIndicator <= 3) {
    if(paramListPartIndicator == 0) {
      sendParameterListPart1();
    }
    else if(paramListPartIndicator == 1) {
      sendParameterListPart2();
    }
    else if(paramListPartIndicator == 2) {
      sendParameterListPart3();
    }
    else if(paramListPartIndicator == 3) {
      sendParameterListPart4();
    }
    paramListPartIndicator++;
  }
  else {
    paramListPartIndicator = -1;
  }
}

void sendSerialVehicleData() {
  sendSerialHudData();
  sendSerialAttitude();
  sendSerialRcRaw();
  sendSerialRawPressure();
  sendSerialRawIMU();
  sendSerialGpsPostion();
  sendSerialSysStatus();
}

void sendSerialTelemetry() {
  sendSerialVehicleData();
  updateFlightTime();
  sendQueuedParameters();
  changeAndSendParameter();
}

#endif //#define _AQ_MAVLINK_H_
