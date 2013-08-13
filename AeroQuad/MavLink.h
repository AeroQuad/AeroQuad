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

#define MAV_COMPONENT_ID MAV_COMP_ID_ALL

#ifndef MAV_SYSTEM_ID
#define MAV_SYSTEM_ID 100
#endif

// MavLink 1.0 DKP
#include "../mavlink/include/mavlink/v1.0/common/mavlink.h"

#include "AeroQuad.h"

const int autopilotType = MAV_AUTOPILOT_GENERIC;
int systemType = MAV_TYPE_GENERIC;
uint8_t baseMode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
uint32_t customMode = 0; // for future use
uint8_t systemStatus = MAV_STATE_BOOT;
uint16_t len;

// Variables for transmitter calibration
float tempReceiverSlope[MAX_NB_CHANNEL] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float tempReceiverOffset[MAX_NB_CHANNEL] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

// Variables for sending and receiving waypoints
bool waypointSending = false;
bool waypointReceiving = false;

unsigned long waypointTimeLastSent = 0;
unsigned long waypointTimeLastReceived = 0;
unsigned long waypointTimeLastRequested = 0;
unsigned long waypointSendTimeout = 1000; // 1 second
unsigned long waypointReceiveTimeout = 1000;

const uint8_t navFrame = MAV_FRAME_GLOBAL_RELATIVE_ALT;

int waypointsToBeRequested = 0; // Total number of waypoints to be requested
int waypointIndexToBeRequestedLast = -1; // Index of the last to be requested waypoint
int waypointIndexToBeRequested = -1; // Index of single waypoint to be requested
int waypointLastRequestedIndex = -1; // Index of the most recent requested waypoint
int waypointIndexToBeSent = -1; // Index of waypoint to be sent

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

const int parameterType = MAVLINK_TYPE_FLOAT;
int parameterListSize;

const char* parameterNameRateRollP = "RateM_Roll_P";
const char* parameterNameRateRollI = "RateM_Roll_I";
const char* parameterNameRateRollD = "RateM_Roll_D";
const char* parameterNameRatePitchP = "RateM_Pitch_P";
const char* parameterNameRatePitchI = "RateM_Pitch_I";
const char* parameterNameRatePitchD = "RateM_Pitch_D";
const char* parameterNameRateRotationFactor = "RateM_RotFactor";
const char* parameterNameAttitudeRollP = "AttM_Roll_P";
const char* parameterNameAttitudeRollI = "AttM_Roll_I";
const char* parameterNameAttitudeRollD = "AttM_Roll_D";
const char* parameterNameAttitudePitchP = "AttM_Pitch_P";
const char* parameterNameAttitudePitchI = "AttM_Pitch_I";
const char* parameterNameAttitudePitchD = "AttM_Pitch_D";
const char* parameterNameAttitudeGyroRollP = "AttM_GyrRoll_P";
const char* parameterNameAttitudeGyroRollI = "AttM_GyrRoll_I";
const char* parameterNameAttitudeGyroRollD = "AttM_GyrRoll_D";
const char* parameterNameAttitudeGyroPitchP = "AttM_GyrPitch_P";
const char* parameterNameAttitudeGyroPitchI = "AttM_GyrPitch_I";
const char* parameterNameAttitudeGyroPitchD = "AttM_GyrPitch_D";
const char* parameterNameYawP = "Yaw_P";
const char* parameterNameYawI = "Yaw_I";
const char* parameterNameYawD = "Yaw_D";
const char* parameterNameHeadingConfig = "HeadingHold_On";
const char* parameterNameHeadingP = "HeadingHold_P";
const char* parameterNameHeadingI = "HeadingHold_I";
const char* parameterNameHeadingD = "HeadingHold_D";
const char* parameterNameMinThrottle = "Misc_Min Thr";
#if defined(BattMonitor)
const char* parameterNameBattMonAlarmVoltage = "BattMon_AlarmV";
const char* parameterNameBattMonThrottleTarget = "BattMon_ThrTarg";
const char* parameterNameBattMonGoingDownTime = "BattMon_DownTim";
#endif
#if defined(CameraControl)
const char* parameterNameCamMode = "Cam_Mode";
const char* parameterNameCamPitchScale = "Cam_Scale Pitch";
const char* parameterNameCamRollScale = "Cam_Scale Roll";
const char* parameterNameCamYawScale = "Cam_Scale Yaw";
const char* parameterNameCamRollServoMiddle = "Cam_Pitch Mid";
const char* parameterNameCamPitchServoMiddle = "Cam_Roll Mid";
const char* parameterNameCamYawServoMiddle = "Cam_Yaw Mid";
const char* parameterNameCamPitchServoMin = "Cam_Pitch Min";
const char* parameterNameCamRollServoMin = "Cam_Roll Min";
const char* parameterNameCamYawServoMin = "Cam_Yaw Min";
const char* parameterNameCamPitchServoMax = "Cam_Pitch Max";
const char* parameterNameCamRollServoMax = "Cam_Roll Max";
const char* parameterNameCamYawServoMax = "Cam_Yaw Max";
#endif
#if defined(AltitudeHoldBaro) || defined(AltitudeHoldRangeFinder)
const char* parameterNameAHminThrottleAdjust = "AltHold_Min Adj";
const char* parameterNameAHmaxThrottleAdjust = "AltHold_Max Adj";
const char* parameterNameAHBumpValue = "AltHold_BumpVal";
const char* parameterNameAHPanicValue = "AltHold_PanicVa";
#endif
#if defined(AltitudeHoldBaro)
const char* parameterNameAHBaroSmooth = "Baro_Smooth";
const char* parameterNameBaroP = "Baro_P";
const char* parameterNameBaroI = "Baro_I";
const char* parameterNameBaroD = "Baro_D";
const char* parameterNameBaroWindUpGuard = "Baro_WindUp";
const char* parameterNameZDampeningP = "Z Dampening_P";
const char* parameterNameZDampeningI = "Z Dampening_I";
const char* parameterNameZDampeningD = "Z Dampening_D";
#endif
#if defined(AltitudeHoldRangeFinder)
const char* parameterNameRangeFinderP = "Sonar_P";
const char* parameterNameRangeFinderI = "Sonar_I";
const char* parameterNameRangeFinderD = "Sonar_D";
const char* parameterNameRangeFinderWindUpGuard = "Sonar_WindUp";
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

parameterTypeIndicator PIDIndicator = NONE;
float *parameterFloat;
byte *parameterByte;
int *parameterInt;
unsigned long *parameterULong;

static uint16_t millisecondsSinceBootWhileArmed = 0;
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

	if (motorArmed) {
		uint16_t timeDiff = millis() - previousFlightTimeUpdate;

		previousFlightTimeUpdate += timeDiff;
		millisecondsSinceBootWhileArmed += timeDiff;
	}
}

void sendSerialHeartbeat() {
	baseMode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

	if (flightMode == ATTITUDE_FLIGHT_MODE) {
		baseMode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
	}

#if defined(UseGPSNavigator)
	if (navigationState == ON || positionHoldState == ON) {
		baseMode |= MAV_MODE_FLAG_GUIDED_ENABLED;
	}
#endif

	if (motorArmed) {
		baseMode |= MAV_MODE_FLAG_SAFETY_ARMED;
		systemStatus = MAV_STATE_ACTIVE;
	}
	else {
		baseMode |= MAV_MODE_PREFLIGHT;
		systemStatus = MAV_STATE_STANDBY;
	}

	mavlink_msg_heartbeat_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, systemType, autopilotType, baseMode, 0, systemStatus);
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
	mavlink_msg_attitude_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBootWhileArmed, kinematicsAngle[XAXIS], kinematicsAngle[YAXIS], kinematicsAngle[ZAXIS], 0, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	SERIAL_PORT.write(buf, len);
}

void sendSerialHudData() {
	//TODO: getBaroAltitude() returns invalid value?
#if defined(HeadingMagHold)
#if defined(AltitudeHoldBaro)
#if defined (UseGPS)
	if(gpsData.state > GPS_NOFIX) {
		mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (float)getGpsSpeed() / 100.0f, (float)getGpsSpeed() / 100.0f, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360, (receiverCommand[THROTTLE]-1000)/10, getBaroAltitude(), 0.0);
	}
	else {
		mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0, 0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360, (receiverCommand[THROTTLE]-1000)/10, getBaroAltitude(), 0.0);
	}
#else
	mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360, (receiverCommand[THROTTLE]-1000)/10, getBaroAltitude(), 0.0);
#endif
#else
	mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360, (receiverCommand[THROTTLE]-1000)/10, 0, 0.0);
#endif
#else
#if defined(AltitudeHoldBaro)
	mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, 0, (receiverCommand[THROTTLE]-1000)/10, getBaroAltitude(), 0.0);
#else
	mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, 0, (receiverCommand[THROTTLE]-1000)/10, 0, 0.0);
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
		mavlink_msg_global_position_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBootWhileArmed, currentPosition.latitude, currentPosition.longitude, getGpsAltitude(), getBaroAltitude(), 65535, getGpsSpeed(), getCourse(), ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360);
#else
		mavlink_msg_global_position_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBootWhileArmed, currentPosition.latitude, currentPosition.longitude, getGpsAltitude(), getGpsAltitude(), 0, 0, 0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360);
#endif
		len = mavlink_msg_to_send_buffer(buf, &msg);
		SERIAL_PORT.write(buf, len);
	}
#endif
}

//TODO: verify output
void sendSerialNavControllerOutput() {
#if defined(UseGPSNavigator)
	if(waypointIndex > -1) {
		mavlink_msg_nav_controller_output_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, gpsRollAxisCorrection, gpsPitchAxisCorrection, gpsYawAxisCorrection, desiredHeading, distanceToNextWaypoint, (missionPositionToReach.altitude / 10 - getBaroAltitude()), 0, crossTrackError);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		SERIAL_PORT.write(buf, len);
	}
#endif
}

void sendSerialMotorOutput() {
	mavlink_msg_servo_output_raw_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBootWhileArmed, 0, motorCommand[MOTOR1], motorCommand[MOTOR2], motorCommand[MOTOR3], motorCommand[MOTOR4], motorCommand[MOTOR5], motorCommand[MOTOR6],motorCommand[MOTOR7], motorCommand[MOTOR8]);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	SERIAL_PORT.write(buf, len);
}

void sendSerialRawPressure() {
#if defined(AltitudeHoldBaro)
	mavlink_msg_raw_pressure_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBootWhileArmed, readRawPressure(), 0,0, readRawTemperature());
	len = mavlink_msg_to_send_buffer(buf, &msg);
	SERIAL_PORT.write(buf, len);
#endif
}

void sendSerialScaledPressure() {
#if defined(AltitudeHoldBaro)
	mavlink_msg_scaled_pressure_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBootWhileArmed, pressure, 0, readRawTemperature());
	len = mavlink_msg_to_send_buffer(buf, &msg);
	SERIAL_PORT.write(buf, len);
#endif
}

void sendSerialRcRaw() {
#if defined(UseRSSIFaileSafe)
	mavlink_msg_rc_channels_raw_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBootWhileArmed, 0, receiverCommand[XAXIS], receiverCommand[YAXIS], receiverCommand[THROTTLE], receiverCommand[ZAXIS], receiverCommand[MODE], receiverCommand[AUX1], receiverCommand[AUX2], receiverCommand[AUX3], rssiRawValue * 2.55);
#else
	mavlink_msg_rc_channels_raw_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBootWhileArmed, 0, receiverCommand[XAXIS], receiverCommand[YAXIS], receiverCommand[THROTTLE], receiverCommand[ZAXIS], receiverCommand[MODE], receiverCommand[AUX1], receiverCommand[AUX2], receiverCommand[AUX3], 255);
#endif
	len = mavlink_msg_to_send_buffer(buf, &msg);
	SERIAL_PORT.write(buf, len);
}

void sendSerialRcScaled() {
	// RC values scaled from "1000 - 2000" to "-10000 - 10000"
#if defined(UseRSSIFaileSafe)
	mavlink_msg_rc_channels_scaled_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBootWhileArmed, 0, (receiverCommand[XAXIS] - 1500) * 20, (receiverCommand[YAXIS] - 1500) * 20, (receiverCommand[THROTTLE] - 1500) * 20, (receiverCommand[ZAXIS] - 1500) * 20, (receiverCommand[MODE] - 1500) * 20, (receiverCommand[AUX1] - 1500) * 20, (receiverCommand[AUX2] - 1500) * 20, (receiverCommand[AUX3] - 1500) * 20, rssiRawValue * 2.55);
#else
	mavlink_msg_rc_channels_scaled_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBootWhileArmed, 0, (receiverCommand[XAXIS] - 1500) * 20, (receiverCommand[YAXIS] - 1500) * 20, (receiverCommand[THROTTLE] - 1500) * 20, (receiverCommand[ZAXIS] - 1500) * 20, (receiverCommand[MODE] - 1500) * 20, (receiverCommand[AUX1] - 1500) * 20, (receiverCommand[AUX2] - 1500) * 20, (receiverCommand[AUX3] - 1500) * 20, 255);
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
#if defined(BM_EXTENDED)
	mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, controlSensorsPresent, controlSensorEnabled, controlSensorsHealthy, 0, batteryData[0].voltage * 10, (int)(batteryData[0].current*1000), -1, system_dropped_packets, 0, 0, 0, 0, 0);
#else
	mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, controlSensorsPresent, controlSensorEnabled, controlSensorsHealthy, 0, batteryData[0].voltage * 10, -1, -1, system_dropped_packets, 0, 0, 0, 0, 0);
#endif
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

#if defined(AltitudeHoldBaro)
	sendSerialParameter(baroSmoothFactor, parameterNameAHBaroSmooth , parameterListSize, indexCounter);
	indexCounter++;

	sendSerialPID(BARO_ALTITUDE_HOLD_PID_IDX, parameterNameBaroP , parameterNameBaroI , parameterNameBaroD , parameterNameBaroWindUpGuard , parameterListSize, indexCounter);
	indexCounter += 4;

	sendSerialPID(ZDAMPENING_PID_IDX, parameterNameZDampeningP , parameterNameZDampeningI , parameterNameZDampeningD , 0, parameterListSize, indexCounter);
	indexCounter += 3;
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
		if (((char) (parameterName[j])) != (char) (key[j])) {
			return false;
		}
	}
	return true;
}

/// <param name="key">Name of parameter to be matched</param>
/// <returns>Index of matching parameter, -1 if parameter has no index, -2 if no parameter matched</returns>
int findParameter(char* key) {
	PIDIndicator = NONE;
	parameterFloat = NULL;
	parameterByte = NULL;
	parameterInt = NULL;
	parameterULong = NULL;

	if (checkParameterMatch(parameterNameRateRollP, key)) {
		PIDIndicator = P;
		return RATE_XAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameRateRollI, key)) {
		PIDIndicator = I;
		return RATE_XAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameRateRollD, key)) {
		PIDIndicator = D;
		return RATE_XAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameRatePitchP, key)) {
		PIDIndicator = P;
		return RATE_YAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameRatePitchI, key)) {
		PIDIndicator = I;
		return RATE_YAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameRatePitchD, key)) {
		PIDIndicator = D;
		return RATE_YAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameRateRotationFactor, key)) {
		PIDIndicator = NONE;
		parameterFloat = &rotationSpeedFactor;
		return -1;
	}
	if (checkParameterMatch(parameterNameAttitudeRollP, key)) {
		PIDIndicator = P;
		return ATTITUDE_XAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameAttitudeRollI, key)) {
		PIDIndicator = I;
		return ATTITUDE_XAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameAttitudeRollD, key)) {
		PIDIndicator = D;
		return ATTITUDE_XAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameAttitudePitchP, key)) {
		PIDIndicator = P;
		return ATTITUDE_YAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameAttitudePitchI, key)) {
		PIDIndicator = I;
		return ATTITUDE_YAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameAttitudePitchD, key)) {
		PIDIndicator = D;
		return ATTITUDE_YAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameAttitudeGyroRollP, key)) {
		PIDIndicator = P;
		return ATTITUDE_GYRO_XAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameAttitudeGyroRollI, key)) {
		PIDIndicator = I;
		return ATTITUDE_GYRO_XAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameAttitudeGyroRollD, key)) {
		PIDIndicator = D;
		return ATTITUDE_GYRO_XAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameAttitudeGyroPitchP, key)) {
		PIDIndicator = P;
		return ATTITUDE_GYRO_YAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameAttitudeGyroPitchI, key)) {
		PIDIndicator = I;
		return ATTITUDE_GYRO_YAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameAttitudeGyroPitchD, key)) {
		PIDIndicator = D;
		return ATTITUDE_GYRO_YAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameYawP, key)) {
		PIDIndicator = P;
		return ZAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameYawI, key)) {
		PIDIndicator = I;
		return ZAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameYawD, key)) {
		PIDIndicator = D;
		return ZAXIS_PID_IDX;
	}
	if (checkParameterMatch(parameterNameHeadingP, key)) {
		PIDIndicator = P;
		return HEADING_HOLD_PID_IDX;
	}
	if (checkParameterMatch(parameterNameHeadingI, key)) {
		PIDIndicator = I;
		return HEADING_HOLD_PID_IDX;
	}
	if (checkParameterMatch(parameterNameHeadingD, key)) {
		PIDIndicator = D;
		return HEADING_HOLD_PID_IDX;
	}
	if (checkParameterMatch(parameterNameHeadingConfig, key)) {
		PIDIndicator = NONE;
		parameterByte = &headingHoldConfig;
		return -1;
	}
	if (checkParameterMatch(parameterNameMinThrottle, key)) {
		PIDIndicator = NONE;
		parameterInt = &minArmedThrottle;
		return -1;
	}

#if defined(BattMonitor)
	if (checkParameterMatch(parameterNameBattMonAlarmVoltage, key)) {
		PIDIndicator = NONE;
		parameterFloat = &batteryMonitorAlarmVoltage;
		setBatteryCellVoltageThreshold(set.param_value);
		return -1;
	}
	if (checkParameterMatch(parameterNameBattMonThrottleTarget, key)) {
		PIDIndicator = NONE;
		parameterInt = &batteryMonitorThrottleTarget;
		return -1;
	}
	if (checkParameterMatch(parameterNameBattMonGoingDownTime, key)) {
		PIDIndicator = NONE;
		parameterULong = &batteryMonitorGoingDownTime;
		return -1;
	}
#endif

#if defined(CameraControl)
	if (checkParameterMatch(parameterNameCamMode, key)) {
		PIDIndicator = NONE;
		parameterInt = &cameraMode;
		return -1;
	}
	if (checkParameterMatch(parameterNameCamPitchScale, key)) {
		PIDIndicator = NONE;
		parameterFloat = &mCameraPitch;
		return -1;
	}
	if (checkParameterMatch(parameterNameCamRollScale, key)) {
		PIDIndicator = NONE;
		parameterFloat = &mCameraRoll;
		return -1;
	}
	if (checkParameterMatch(parameterNameCamYawScale, key)) {
		PIDIndicator = NONE;
		parameterFloat = &mCameraYaw;
		return -1;
	}
	if (checkParameterMatch(parameterNameCamPitchServoMiddle, key)) {
		PIDIndicator = NONE;
		parameterInt = &servoCenterPitch;
		return -1;
	}
	if (checkParameterMatch(parameterNameCamRollServoMiddle, key)) {
		PIDIndicator = NONE;
		parameterInt = &servoCenterRoll;
		return -1;
	}
	if (checkParameterMatch(parameterNameCamYawServoMiddle, key)) {
		PIDIndicator = NONE;
		parameterInt = &servoCenterYaw;
		return -1;
	}
	if (checkParameterMatch(parameterNameCamPitchServoMin, key)) {
		PIDIndicator = NONE;
		parameterInt = &servoMinPitch;
		return -1;
	}
	if (checkParameterMatch(parameterNameCamRollServoMin, key)) {
		PIDIndicator = NONE;
		parameterInt = &servoMinRoll;
		return -1;
	}
	if (checkParameterMatch(parameterNameCamYawServoMin, key)) {
		PIDIndicator = NONE;
		parameterInt = &servoMinYaw;
		return -1;
	}
	if (checkParameterMatch(parameterNameCamPitchServoMax, key)) {
		PIDIndicator = NONE;
		parameterInt = &servoMaxPitch;
		return -1;
	}
	if (checkParameterMatch(parameterNameCamRollServoMax, key)) {
		PIDIndicator = NONE;
		parameterInt = &servoMaxRoll;
		return -1;
	}
	if (checkParameterMatch(parameterNameCamYawServoMax, key)) {
		PIDIndicator = NONE;
		parameterInt = &servoMaxYaw;
		return -1;
	}
#endif

#if defined(AltitudeHoldBaro) || defined(AltitudeHoldRangeFinder)
	if (checkParameterMatch(parameterNameAHminThrottleAdjust, key)) {
		PIDIndicator = NONE;
		parameterInt = &minThrottleAdjust;
		return -1;
	}
	if (checkParameterMatch(parameterNameAHmaxThrottleAdjust, key)) {
		PIDIndicator = NONE;
		parameterInt = &maxThrottleAdjust;
		return -1;
	}
	if (checkParameterMatch(parameterNameAHBumpValue, key)) {
		PIDIndicator = NONE;
		parameterInt = &altitudeHoldBump;
		return -1;
	}
	if (checkParameterMatch(parameterNameAHPanicValue, key)) {
		PIDIndicator = NONE;
		parameterInt = &altitudeHoldPanicStickMovement;
		return -1;
	}
#endif

#if defined(AltitudeHoldBaro)
	if (checkParameterMatch(parameterNameAHBaroSmooth, key)) {
		PIDIndicator = NONE;
		parameterFloat = &baroSmoothFactor;
		return -1;
	}
	if (checkParameterMatch(parameterNameBaroP, key)) {
		PIDIndicator = P;
		return BARO_ALTITUDE_HOLD_PID_IDX;
	}
	if (checkParameterMatch(parameterNameBaroI, key)) {
		PIDIndicator = I;
		return BARO_ALTITUDE_HOLD_PID_IDX;
	}
	if (checkParameterMatch(parameterNameBaroD, key)) {
		PIDIndicator = D;
		return BARO_ALTITUDE_HOLD_PID_IDX;
	}
	if (checkParameterMatch(parameterNameBaroWindUpGuard, key)) {
		PIDIndicator = windUpGuard;
		return BARO_ALTITUDE_HOLD_PID_IDX;
	}
	if (checkParameterMatch(parameterNameZDampeningP, key)) {
		PIDIndicator = P;
		return ZDAMPENING_PID_IDX;
	}
	if (checkParameterMatch(parameterNameZDampeningI, key)) {
		PIDIndicator = I;
		return ZDAMPENING_PID_IDX;
	}
	if (checkParameterMatch(parameterNameZDampeningD, key)) {
		PIDIndicator = D;
		return ZDAMPENING_PID_IDX;
	}
#endif

#if defined(AltitudeHoldRangeFinder)
	if (checkParameterMatch(parameterNameRangeFinderP, key)) {
		PIDIndicator = P;
		return SONAR_ALTITUDE_HOLD_PID_IDX;
	}
	if (checkParameterMatch(parameterNameRangeFinderI, key)) {
		PIDIndicator = I;
		return SONAR_ALTITUDE_HOLD_PID_IDX;
	}
	if (checkParameterMatch(parameterNameRangeFinderD, key)) {
		PIDIndicator = D;
		return SONAR_ALTITUDE_HOLD_PID_IDX;
	}
	if (checkParameterMatch(parameterNameRangeFinderWindUpGuard, key)) {
		PIDIndicator = windUpGuard;
		return SONAR_ALTITUDE_HOLD_PID_IDX;
	}
#endif

#if defined(UseGPSNavigator)
	if (checkParameterMatch(parameterNameGPSRollP, key)) {
		PIDIndicator = P;
		return GPSROLL_PID_IDX;
	}
	if (checkParameterMatch(parameterNameGPSRollI, key)) {
		PIDIndicator = I;
		return GPSROLL_PID_IDX;
	}
	if (checkParameterMatch(parameterNameGPSRollD, key)) {
		PIDIndicator = D;
		return GPSROLL_PID_IDX;
	}
	if (checkParameterMatch(parameterNameGPSPitchP, key)) {
		PIDIndicator = P;
		return GPSPITCH_PID_IDX;
	}
	if (checkParameterMatch(parameterNameGPSPitchI, key)) {
		PIDIndicator = I;
		return GPSPITCH_PID_IDX;
	}
	if (checkParameterMatch(parameterNameGPSPitchD, key)) {
		PIDIndicator = D;
		return GPSPITCH_PID_IDX;
	}
	if (checkParameterMatch(parameterNameGPSYawP, key)) {
		PIDIndicator = P;
		return GPSYAW_PID_IDX;
	}
	if (checkParameterMatch(parameterNameGPSYawI, key)) {
		PIDIndicator = I;
		return GPSYAW_PID_IDX;
	}
	if (checkParameterMatch(parameterNameGPSYawD, key)) {
		PIDIndicator = D;
		return GPSYAW_PID_IDX;
	}
#endif
	return -2; // No parameter found, should not happen
}

void changeAndSendParameter() {
	if(parameterChangeIndicator == 0) {
		if(parameterMatch == -2) { // no parameter matched, should not happen
			mavlink_msg_statustext_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SEVERITY_ERROR, "Write failed");
			len = mavlink_msg_to_send_buffer(buf, &msg);
			SERIAL_PORT.write(buf, len);
			parameterChangeIndicator = -1;
			return;
		}

		// Only write and emit changes if there is actually a difference AND only write if new value is NOT "not-a-number" AND is NOT infinit
		if (PIDIndicator == P) {
			if (PID[parameterMatch].P != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
				PID[parameterMatch].P = set.param_value;
				writeEEPROM();
				// Report back new value
				mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].P, parameterType, parameterListSize, -1);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				SERIAL_PORT.write(buf, len);
			}
		}
		else if (PIDIndicator == I) {
			if (PID[parameterMatch].I != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
				PID[parameterMatch].I = set.param_value;
				writeEEPROM();
				// Report back new value
				mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].I, parameterType, parameterListSize, -1);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				SERIAL_PORT.write(buf, len);
			}
		}
		else if (PIDIndicator == D) {
			if (PID[parameterMatch].D != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
				PID[parameterMatch].D = set.param_value;
				writeEEPROM();
				// Report back new value
				mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].D, parameterType, parameterListSize, -1);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				SERIAL_PORT.write(buf, len);
			}
		}
		else if (PIDIndicator == windUpGuard) {
			if (PID[parameterMatch].windupGuard != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
				PID[parameterMatch].windupGuard = set.param_value;
				writeEEPROM();
				// Report back new value
				mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].windupGuard, parameterType, parameterListSize, -1);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				SERIAL_PORT.write(buf, len);
			}
		}
		else if (PIDIndicator == NONE) {
			if (parameterFloat != NULL) {
				if (*parameterFloat != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
					*parameterFloat = set.param_value;
					writeEEPROM();
					// Report back new value
					mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, *parameterFloat, parameterType, parameterListSize, -1);
					len = mavlink_msg_to_send_buffer(buf, &msg);
					SERIAL_PORT.write(buf, len);
				}
			}
			else if (parameterByte != NULL) {
				if (*parameterByte != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
					*parameterByte = set.param_value;
					writeEEPROM();
					// Report back new value
					mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, (float)*parameterByte, parameterType, parameterListSize, -1);
					len = mavlink_msg_to_send_buffer(buf, &msg);
					SERIAL_PORT.write(buf, len);
				}
			}
			else if (parameterInt != NULL) {
				if (*parameterInt != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
					*parameterInt = set.param_value;
					writeEEPROM();
					// Report back new value
					mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, (float)*parameterInt, parameterType, parameterListSize, -1);
					len = mavlink_msg_to_send_buffer(buf, &msg);
					SERIAL_PORT.write(buf, len);
				}
			}
			else if (parameterULong != NULL) {
				if (*parameterULong != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
					*parameterULong = set.param_value;
					writeEEPROM();
					// Report back new value
					mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, (float)*parameterULong, parameterType, parameterListSize, -1);
					len = mavlink_msg_to_send_buffer(buf, &msg);
					SERIAL_PORT.write(buf, len);
				}
			}
		}
		parameterChangeIndicator = -1;

		mavlink_msg_statustext_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SEVERITY_INFO, "Write successful");
		len = mavlink_msg_to_send_buffer(buf, &msg);
		SERIAL_PORT.write(buf, len);
	}
}

void sendQueuedParameters() {
	// sending complete parameter list divided into four parts to minimize delay
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

			mavlink_msg_statustext_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SEVERITY_INFO, "All parameters received");
			len = mavlink_msg_to_send_buffer(buf, &msg);
			SERIAL_PORT.write(buf, len);
		}
		paramListPartIndicator++;
	}
	else {
		paramListPartIndicator = -1;
	}
}

void receiveWaypoint() { // request waypoints one by one from GCS
	if (waypointLastRequestedIndex != waypointIndexToBeRequested) {
		waypointLastRequestedIndex = waypointIndexToBeRequested;

		mavlink_msg_mission_request_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SYSTEM_ID, MAV_COMPONENT_ID, waypointIndexToBeRequested);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		SERIAL_PORT.write(buf, len);
	}
}

//TODO not working yet
bool calculateTransmitterCalibrationValues() {
	return false;
}

void readSerialCommand() {
	while(SERIAL_PORT.available() > 0) {

		uint8_t c = SERIAL_PORT.read();
		//try to get a new message
		if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message
			float x = 0, y = 0, z = 0;
			uint8_t result = MAV_RESULT_UNSUPPORTED;
			uint8_t isCurrentWaypoint = 0;

			switch(msg.msgid) {

			case MAVLINK_MSG_ID_COMMAND_LONG:
				mavlink_command_long_t commandPacket;
				mavlink_msg_command_long_decode(&msg, &commandPacket);

				switch(commandPacket.command) {
					// (yet) unsupported commands/features
					//case MAV_CMD_NAV_WAYPOINT: //16
					//case MAV_CMD_NAV_LOITER_UNLIM: //17
					//case MAV_CMD_NAV_LOITER_TURNS: //18
					//case MAV_CMD_NAV_LOITER_TIME: //19
					//case MAV_CMD_NAV_RETURN_TO_LAUNCH: //20
					//case MAV_CMD_NAV_LAND: //21
					//case MAV_CMD_NAV_TAKEOFF: //22
					//case MAV_CMD_NAV_ROI: //80
					//case MAV_CMD_NAV_PATHPLANNING: //81
					//case MAV_CMD_NAV_LAST: //95
					//case MAV_CMD_CONDITION_DELAY: //112
					//case MAV_CMD_CONDITION_CHANGE_ALT: //113
					//case MAV_CMD_CONDITION_DISTANCE: //114
					//case MAV_CMD_CONDITION_YAW: //115
					//case MAV_CMD_CONDITION_LAST: //159
					//case MAV_CMD_DO_SET_MODE: //176
					//case MAV_CMD_DO_JUMP: //177
					//case MAV_CMD_DO_CHANGE_SPEED: //178
					//case MAV_CMD_DO_SET_PARAMETER: //180
					//case MAV_CMD_DO_SET_RELAY: //181
					//case MAV_CMD_DO_REPEAT_RELAY: //182
					//case MAV_CMD_DO_SET_SERVO: //183
					//case MAV_CMD_DO_REPEAT_SERVO: //184
					//case MAV_CMD_DO_CONTROL_VIDEO: //200
					//case MAV_CMD_DO_LAST: //240
					//case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS: //242
					//case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: //246
					//case MAV_CMD_OVERRIDE_GOTO: //252
					//case MAV_CMD_MISSION_START: //300
					//	result = MAV_RESULT_UNSUPPORTED;
					//	break;

				case MAV_CMD_COMPONENT_ARM_DISARM: //400, toggle between armed/disarmed
					if (commandPacket.param1 == 1.0f) {
						if(motorArmed) {
							armMotors();
							result = MAV_RESULT_ACCEPTED;
						}
						else {
							result = MAV_RESULT_TEMPORARILY_REJECTED;
						}
					}
					else if (commandPacket.param1 == 0.0f) {
						if(!motorArmed) {
							disarmMotors();
							result = MAV_RESULT_ACCEPTED;
						}
						else {
							result = MAV_RESULT_TEMPORARILY_REJECTED;
						}
					}
					else {
						result = MAV_RESULT_UNSUPPORTED;
					}
					break;

				case MAV_CMD_DO_SET_HOME: //179, resetting GPS home position
#if defined(UseGPSNavigator)
					if (commandPacket.param1 == 1.0f) {
						initHomeBase();
					}
					else {
						homePosition.latitude = 1.0e7f * commandPacket.param5;
						homePosition.longitude = 1.0e7f * commandPacket.param6;
						homePosition.altitude = 1.0e2f * commandPacket.param7;
					}
					result = MAV_RESULT_ACCEPTED;
#else
					result = MAV_RESULT_UNSUPPORTED;
#endif
					break;

					//TODO not working yet
				case MAV_CMD_PREFLIGHT_CALIBRATION: //241, calibration of acc/gyro/transmitter
					if (!motorArmed) {
						if (commandPacket.param1 == 1.0f) {
							// gyro calibration
							calibrateGyro();
							storeSensorsZeroToEEPROM();
							result = MAV_RESULT_ACCEPTED;
						}
						if(commandPacket.param3 == 1.0f) {
							// reset baro altitude
							measureGroundBaro();
							baroAltitude = baroGroundAltitude;
							result = MAV_RESULT_ACCEPTED;
						}
						if (commandPacket.param4 == 1.0f) {
							// transmitter calibration
							if(calculateTransmitterCalibrationValues()) {

								for (byte axis = XAXIS; axis < LASTCHANNEL; axis++) {
									receiverOffset[axis] = tempReceiverOffset[axis];
									receiverSlope[axis] = tempReceiverSlope[axis];
								}
								writeEEPROM();
								result = MAV_RESULT_ACCEPTED;
							}
							else {
								result = MAV_RESULT_FAILED;
							}
						}
						if (commandPacket.param5 == 1.0f) {
							// accel calibration
							computeAccelBias();
							storeSensorsZeroToEEPROM();
							calibrateKinematics();
							zeroIntegralError();
							result = MAV_RESULT_ACCEPTED;
						}
					}
					else result = MAV_RESULT_TEMPORARILY_REJECTED;
					break;

				case MAV_CMD_PREFLIGHT_STORAGE: //245, reading/writing of parameter list requested by GCS while in Preflight mode
					if (!motorArmed) {
						if (mavlink_msg_command_long_get_param1(&msg) == 0.0f) {
							paramListPartIndicator = indexCounter = 0;
						}
						else if (mavlink_msg_command_long_get_param1(&msg) == 1.0f) {
							mavlink_msg_param_set_decode(&msg, &set);
							key = (char*) set.param_id;
							parameterMatch = findParameter(key);
							parameterChangeIndicator = 0;
							changeAndSendParameter();
						}
					}	
					break;

				default:
					result = MAV_RESULT_UNSUPPORTED;
					break;
				}

				mavlink_msg_command_ack_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, commandPacket.command, result);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				SERIAL_PORT.write(buf, len);
				break;

			case MAVLINK_MSG_ID_SET_MODE: // set the base mode (only arming/disarming makes sense for now)
				__mavlink_set_mode_t modePacket;
				mavlink_msg_set_mode_decode(&msg, &modePacket);

				if(modePacket.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY) {
					if(!motorArmed) {
						armMotors();
					}
				}
				else if(!(modePacket.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY)) {
					if(motorArmed) {
						disarmMotors();
					}
				}
				break;

			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:	 // sending complete parameter list to GCS
				paramListPartIndicator = indexCounter = 0;
				break;

			case MAVLINK_MSG_ID_PARAM_REQUEST_READ:  // sending a specific parameter to GCS
				mavlink_param_request_read_t read;
				mavlink_msg_param_request_read_decode(&msg, &read);

				key = (char*) read.param_id;
				parameterMatch = findParameter(key);

				if(parameterMatch == -2) {
					mavlink_msg_statustext_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SEVERITY_ERROR, "Read failed");
					len = mavlink_msg_to_send_buffer(buf, &msg);
					SERIAL_PORT.write(buf, len);
					break;
				}
				if (PIDIndicator == P) {
					mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].P, parameterType, parameterListSize, -1);
					len = mavlink_msg_to_send_buffer(buf, &msg);
					SERIAL_PORT.write(buf, len);
				}
				else if (PIDIndicator == I) {
					mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].I, parameterType, parameterListSize, -1);
					len = mavlink_msg_to_send_buffer(buf, &msg);
					SERIAL_PORT.write(buf, len);
				}
				else if (PIDIndicator == D) {
					mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].D, parameterType, parameterListSize, -1);
					len = mavlink_msg_to_send_buffer(buf, &msg);
					SERIAL_PORT.write(buf, len);
				}
				else if (PIDIndicator == windUpGuard) {
					mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].windupGuard, parameterType, parameterListSize, -1);
					len = mavlink_msg_to_send_buffer(buf, &msg);
					SERIAL_PORT.write(buf, len);
				}
				else if (PIDIndicator == NONE) {
					if (parameterFloat != NULL) {
						mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, *parameterFloat, parameterType, parameterListSize, -1);
						len = mavlink_msg_to_send_buffer(buf, &msg);
						SERIAL_PORT.write(buf, len);
					}
					else if (parameterByte != NULL) {
						mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, (float)*parameterByte, parameterType, parameterListSize, -1);
						len = mavlink_msg_to_send_buffer(buf, &msg);
						SERIAL_PORT.write(buf, len);
					}
					else if (parameterInt != NULL) {
						mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, (float)*parameterInt, parameterType, parameterListSize, -1);
						len = mavlink_msg_to_send_buffer(buf, &msg);
						SERIAL_PORT.write(buf, len);
					}
					else if (parameterULong != NULL) {
						mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, (float)*parameterULong, parameterType, parameterListSize, -1);
						len = mavlink_msg_to_send_buffer(buf, &msg);
						SERIAL_PORT.write(buf, len);
					}
				}
				mavlink_msg_statustext_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SEVERITY_INFO, "Read successful");
				len = mavlink_msg_to_send_buffer(buf, &msg);
				SERIAL_PORT.write(buf, len);
				break;

			case MAVLINK_MSG_ID_PARAM_SET:  // set one specific onboard parameter
				if(!motorArmed) { // added for security reason, as the software is shortly blocked by this command
					mavlink_msg_param_set_decode(&msg, &set);
					key = (char*) set.param_id;
					parameterMatch = findParameter(key);
					parameterChangeIndicator = 0;
					changeAndSendParameter();
				}
				break;

			case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:  // requesting complete waypoint list from AQ
#if defined(UseGPSNavigator)
				mavlink_msg_mission_count_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SYSTEM_ID, MAV_COMPONENT_ID, missionNbPoint + 1);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				SERIAL_PORT.write(buf, len);

				waypointTimeLastSent = millis();
				waypointSending = true;
				waypointReceiving = false;
#endif
				break;

			case MAVLINK_MSG_ID_MISSION_REQUEST: // GCS requests a specific waypoint
#if defined(UseGPSNavigator)
				__mavlink_mission_request_t requestedWaypointPacket;
				mavlink_msg_mission_request_decode(&msg, &requestedWaypointPacket);

				waypointIndexToBeSent = requestedWaypointPacket.seq;

				isCurrentWaypoint = 0;
				if(waypointIndexToBeSent == waypointIndex) {
					isCurrentWaypoint = 1;
				}

				// command needs scaling
				x = waypoint[waypointIndexToBeSent].latitude / 1.0e7f;
				y = waypoint[waypointIndexToBeSent].longitude / 1.0e7f;
				z = waypoint[waypointIndexToBeSent].altitude / 1.0e2f;

				mavlink_msg_mission_item_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SYSTEM_ID, MAV_COMPONENT_ID, requestedWaypointPacket.seq,
					navFrame, MAV_CMD_NAV_WAYPOINT, isCurrentWaypoint, 1, 0, waypointCaptureDistance, 0, 0, x, y, z);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				SERIAL_PORT.write(buf, len);

				// update last waypoint comm stamp
				waypointTimeLastSent = millis();
#endif
				break;

			case MAVLINK_MSG_ID_MISSION_COUNT: // number of waypoints to be sent from GCS to AQ
#if defined(UseGPSNavigator)
				__mavlink_mission_count_t waypointListPacket;
				mavlink_msg_mission_count_decode(&msg, &waypointListPacket);

				if(waypointListPacket.count > MAX_WAYPOINTS) {
					mavlink_msg_statustext_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SEVERITY_WARNING, "Max. 16 waypoints allowed!");
					len = mavlink_msg_to_send_buffer(buf, &msg);
					SERIAL_PORT.write(buf, len);

					waypointListPacket.count = MAX_WAYPOINTS;
				}

				waypointsToBeRequested = waypointListPacket.count;
				waypointTimeLastReceived = millis();
				waypointReceiving = true;
				waypointSending = false;
				waypointIndexToBeRequested = 0;
				waypointIndexToBeRequestedLast = waypointsToBeRequested - 1;
				waypointLastRequestedIndex = -1;
				waypointTimeLastRequested = 0;
				missionNbPoint = -1; // reset number of waypoints
#endif
				break;

			case MAVLINK_MSG_ID_MISSION_ITEM: // add a waypoint from GCS to the waypoint list of AQ
#if defined(UseGPSNavigator)
				if (waypointIndexToBeRequested >= waypointsToBeRequested || waypointIndexToBeRequested > waypointIndexToBeRequestedLast) { // all waypoints received, send ACK message
					mavlink_msg_mission_ack_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SYSTEM_ID, MAV_COMPONENT_ID, result);
					len = mavlink_msg_to_send_buffer(buf, &msg);
					SERIAL_PORT.write(buf, len);

					waypointReceiving = false;
					isRouteInitialized = false;
					break;
				}     

				result = MAV_MISSION_ACCEPTED;

				__mavlink_mission_item_t waypointPacket;
				mavlink_msg_mission_item_decode(&msg, &waypointPacket);

				waypoint[waypointIndexToBeRequested].latitude = 1.0e7f * waypointPacket.x;
				waypoint[waypointIndexToBeRequested].longitude = 1.0e7f * waypointPacket.y;
				waypoint[waypointIndexToBeRequested].altitude = 1.0e2f * waypointPacket.z;
				missionNbPoint++;

				// Check if receiving waypoints (mission upload expected)
				if (!waypointReceiving) {
					result = MAV_MISSION_ERROR;
					mavlink_msg_mission_ack_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SYSTEM_ID, MAV_COMPONENT_ID, result);
					len = mavlink_msg_to_send_buffer(buf, &msg);
					SERIAL_PORT.write(buf, len);

					mavlink_msg_statustext_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SEVERITY_ERROR, "Error");
					len = mavlink_msg_to_send_buffer(buf, &msg);
					SERIAL_PORT.write(buf, len);
					break;
				}

				// Check if this is the requested waypoint
				if (waypointPacket.seq != waypointIndexToBeRequested) {
					result = MAV_MISSION_INVALID_SEQUENCE;
					mavlink_msg_mission_ack_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SYSTEM_ID, MAV_COMPONENT_ID, result);
					len = mavlink_msg_to_send_buffer(buf, &msg);
					SERIAL_PORT.write(buf, len);

					mavlink_msg_statustext_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SEVERITY_ERROR, "Incorrect waypoint sequence");
					len = mavlink_msg_to_send_buffer(buf, &msg);
					SERIAL_PORT.write(buf, len);
					break;
				}

				// Update waypoint receiving state machine
				waypointTimeLastReceived = millis();
				waypointTimeLastRequested = 0;
				waypointIndexToBeRequested++;
#endif
				break;

			case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
				mavlink_mission_write_partial_list_t waypointPartialListPacket;
				mavlink_msg_mission_write_partial_list_decode(&msg, &waypointPartialListPacket);

				if (waypointPartialListPacket.start_index > missionNbPoint ||
					waypointPartialListPacket.end_index > missionNbPoint ||
					waypointPartialListPacket.end_index < waypointPartialListPacket.start_index) {
						mavlink_msg_statustext_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SEVERITY_ERROR, "Mission update rejected");
						len = mavlink_msg_to_send_buffer(buf, &msg);
						SERIAL_PORT.write(buf, len);
						break;
				}

				waypointTimeLastReceived = millis();
				waypointTimeLastRequested = 0;
				waypointReceiving = true;
				waypointIndexToBeRequested = waypointPartialListPacket.start_index;
				waypointIndexToBeRequestedLast = waypointPartialListPacket.end_index;
				break;

			case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:  // delete all waypoints of AQ
#if defined(UseGPSNavigator)
				for (byte location = 0; location < MAX_WAYPOINTS; location++) {
					waypoint[location].longitude = GPS_INVALID_ANGLE;
					waypoint[location].latitude = GPS_INVALID_ANGLE;
					waypoint[location].altitude = GPS_INVALID_ALTITUDE;
				}
				isRouteInitialized = false; // reset mission status
				missionNbPoint = -1; // reset number of waypoints

				// Sending ACK message three times to make sure it's received
				//TODO: ACK is not received by GCS, but waypoints are deleted - maybe a bug in QGroundControl?
				for (int16_t i=0; i<3; i++) {
					mavlink_msg_mission_ack_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAV_MISSION_ACCEPTED);
					len = mavlink_msg_to_send_buffer(buf, &msg);
					SERIAL_PORT.write(buf, len);
				}

				mavlink_msg_statustext_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SEVERITY_INFO, "Mission deleted");
				len = mavlink_msg_to_send_buffer(buf, &msg);
				SERIAL_PORT.write(buf, len);
#endif
				break;

			case MAVLINK_MSG_ID_MISSION_ACK: 
#if defined(UseGPSNavigator)
				// turn off waypoint sending
				waypointSending = false;

				mavlink_msg_statustext_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SEVERITY_INFO, "Waypoint OK");
				len = mavlink_msg_to_send_buffer(buf, &msg);
				SERIAL_PORT.write(buf, len);
#endif
				break;
			}
		}
	}

	system_dropped_packets += status.packet_rx_drop_count;

	if (!waypointReceiving && !waypointSending) {
		return;
	}

	uint32_t tnow = millis();

	if (waypointReceiving && waypointIndexToBeRequested <= waypointsToBeRequested && tnow > waypointTimeLastRequested) {
		waypointTimeLastRequested = tnow;
		receiveWaypoint();
	}

	// stop waypoint sending if timeout
	if (waypointSending && (tnow - waypointTimeLastSent) > waypointSendTimeout) {
		waypointSending = false;
	}

	// stop waypoint receiving if timeout
	if (waypointReceiving && (tnow - waypointTimeLastReceived) > waypointReceiveTimeout) {
		waypointReceiving = false;
	}
}

void sendSerialVehicleData() {
	sendSerialHudData();
	sendSerialAttitude();
	sendSerialRcRaw();
	sendSerialRcScaled();
	sendSerialRawPressure();
	sendSerialScaledPressure();
	sendSerialRawIMU();
	sendSerialGpsPostion();
	sendSerialSysStatus();
	sendSerialNavControllerOutput();
	sendSerialMotorOutput();
}

void sendSerialTelemetry() {
	updateFlightTime();

	if (!waypointReceiving && !waypointSending) {
		// Don't interfere with mission transfer
		sendSerialVehicleData();
		sendQueuedParameters();
	}
}

#endif //#define _AQ_MAVLINK_H_