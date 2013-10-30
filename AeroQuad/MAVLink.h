/*
AeroQuad v3.x - 2013
www.AeroQuad.com
Copyright (c) 2013 Ted Carancho.  All rights reserved.
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
#include "../Libraries/AQ_MAVLink/include/mavlink/v1.0/common/mavlink.h"
#include "../Libraries/AQ_MAVLink/GCS_MAVLink.h"
#include "AeroQuad.h"

mavlink_channel_t chan = MAVLINK_COMM_0;

///* Variables for vehicle configuration and status *///
const int autopilotType = MAV_AUTOPILOT_GENERIC;
int systemType = MAV_TYPE_GENERIC;
uint8_t baseMode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
uint32_t customMode = 0; // for future use
uint8_t systemStatus = MAV_STATE_BOOT;

///* Variables for mag calibration *///
bool isCalibratingMag = false;

float measuredMagMin[3] = {0.0,0.0,0.0};
float measuredMagMax[3] = {0.0,0.0,0.0};

int magCalibrationTimeout = 40000; // 40 seconds

unsigned long magCalibrationTimeStarted = 0;

///* Variables for accel calibration *///
// Calibration needs to be performed in six different positions:
// 0 - right side up
// 1 - upside down
// 2 - left down
// 3 - right down
// 4 - rear down
// 5 - front down
bool isCalibratingAccel = false;
bool isStep1Calibrating = false;
bool isStep2Calibrating = false;
bool isStep3Calibrating = false;
bool isStep4Calibrating = false;
bool isStep5Calibrating = false;
bool isStep6Calibrating = false;
bool isAccelCalibrationStep1Done = false;
bool isAccelCalibrationStep2Done = false;
bool isAccelCalibrationStep3Done = false;
bool isAccelCalibrationStep4Done = false;
bool isAccelCalibrationStep5Done = false;
bool isAccelCalibrationStep6Done = false;
bool areAllStepsCompleted = false;

unsigned long accelCalibrationLastSampleTime = 0;
unsigned long accelCalibrationLastTimeRequested = 0;

#define NumberOfAccelSamples 100
#define AccelCalibrationSampleDelay 50 // 50ms

int accelRawData[3][NumberOfAccelSamples]; // Contains raw accel data (each of the 3 axes is measured 100 times)
int accelRawDataSum[3]; // Contains sum of raw accel data of each axis
float accelMeanData[3][6]; // Contains the mean values for each of the 3 axes in each of the 6 calibration posistions

int currentSampleCounter = 0;
int accelCalibrationTimeout = 40000; // 40 seconds

///* Variables for transmitter calibration *///
float tempReceiverSlope[MAX_NB_CHANNEL] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float tempReceiverOffset[MAX_NB_CHANNEL] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

///* Variables for sending and receiving waypoints *///
bool waypointSending = false;
bool waypointReceiving = false;

unsigned long waypointTimeLastSent = 0;
unsigned long waypointTimeLastReceived = 0;
unsigned long waypointTimeLastRequested = 0;
unsigned long waypointSendTimeout = 2000; // 2000ms = 2 seconds
unsigned long waypointReceiveTimeout = 2000;

const uint8_t navFrame = MAV_FRAME_GLOBAL_RELATIVE_ALT;

int waypointsToBeRequested = 0; // Total number of waypoints to be requested
int waypointIndexToBeRequestedLast = -1; // Index of the waypoint to be requested last
int waypointIndexToBeRequested = -1; // Index of single waypoint to be requested
int waypointLastRequestedIndex = -1; // Index of the most recent requested waypoint
int waypointIndexToBeSent = -1; // Index of waypoint to be sent

///* Variables for transmitter calibration *///
// default channel order in QGroundControl is:
// 1=roll, 2=pitch, 3=yaw, 4=throttle, 5=mode sw, 6-8=aux 1-3
const char* parameterNameTxMode = "RC_TYPE";
const char* parameterNameCh1Min = "RC1_MIN";
const char* parameterNameCh2Min = "RC2_MIN";
const char* parameterNameCh3Min = "RC3_MIN";
const char* parameterNameCh4Min = "RC4_MIN";
const char* parameterNameCh5Min = "RC5_MIN";
const char* parameterNameCh6Min = "RC6_MIN";
const char* parameterNameCh7Min = "RC7_MIN";
const char* parameterNameCh8Min = "RC8_MIN";
const char* parameterNameCh1Max = "RC1_MAX";
const char* parameterNameCh2Max = "RC2_MAX";
const char* parameterNameCh3Max = "RC3_MAX";
const char* parameterNameCh4Max = "RC4_MAX";
const char* parameterNameCh5Max = "RC5_MAX";
const char* parameterNameCh6Max = "RC6_MAX";
const char* parameterNameCh7Max = "RC7_MAX";
const char* parameterNameCh8Max = "RC8_MAX";
const char* parameterNameCh1Trim = "RC1_TRIM";
const char* parameterNameCh2Trim = "RC2_TRIM";
const char* parameterNameCh3Trim = "RC3_TRIM";
const char* parameterNameCh4Trim = "RC4_TRIM";
const char* parameterNameCh5Trim = "RC5_TRIM";
const char* parameterNameCh6Trim = "RC6_TRIM";
const char* parameterNameCh7Trim = "RC7_TRIM";
const char* parameterNameCh8Trim = "RC8_TRIM";
const char* parameterNameCh1Rev = "RC1_REV";
const char* parameterNameCh2Rev = "RC2_REV";
const char* parameterNameCh3Rev = "RC3_REV";
const char* parameterNameCh4Rev = "RC4_REV";
const char* parameterNameCh5Rev = "RC5_REV";
const char* parameterNameCh6Rev = "RC6_REV";
const char* parameterNameCh7Rev = "RC7_REV";
const char* parameterNameCh8Rev = "RC8_REV";
const char* parameterNameRcMapRoll = "RC_MAP_ROLL";
const char* parameterNameRcMapPitch = "RC_MAP_PITCH";
const char* parameterNameRcMapYaw = "RC_MAP_YAW";
const char* parameterNameRcMapThrottle = "RC_MAP_THROTTLE";
const char* parameterNameRcMapMode = "RC_MAP_MODE_SW";
const char* parameterNameRcMapAux1 = "RC_MAP_AUX1";
const char* parameterNameRcMapAux2 = "RC_MAP_AUX2";
const char* parameterNameRcMapAux3 = "RC_MAP_AUX3";

bool isRcCalibrationNeeded = false;

int rcTrim = 1500; // center postion of RC channels not needed at the moment, so use dummy value
int rcRev = 0; // for now channels can't be reversed, so use constant value for all channels
int txMode = 2; // setting TX mode does not have an effect in AQ now, so keep default (Mode 2)
int rcMap = 0; // mapping RC channels not available yet, so use dummy value

///* Variables for writing and sending parameters *///
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
#if defined(BattMonitorAutoDescent)
const char* parameterNameBattMonThrottleTarget = "BattMon_ThrTarg";
const char* parameterNameBattMonGoingDownTime = "BattMon_DownTim";
#endif
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

long system_dropped_packets = 0;

mavlink_message_t msg;
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
	parameterListSize += 1;

#if defined(BattMonitorAutoDescent)
	parameterListSize += 2;
#endif
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
	mavlink_system.sysid = MAV_SYSTEM_ID;
	mavlink_system.compid = MAV_COMPONENT_ID;

	evaluateParameterListSize();
	evaluateCopterType();
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

	mavlink_msg_heartbeat_send(chan, systemType, autopilotType, baseMode, customMode, systemStatus);
}

void sendSerialRawIMU() {
#if defined(HeadingMagHold)
	mavlink_msg_raw_imu_send(chan, 0, meterPerSecSec[XAXIS], meterPerSecSec[YAXIS], meterPerSecSec[ZAXIS], gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], getMagnetometerRawData(XAXIS), getMagnetometerRawData(YAXIS), getMagnetometerRawData(ZAXIS));
#else
	mavlink_msg_raw_imu_send(chan, 0, meterPerSecSec[XAXIS], meterPerSecSec[YAXIS], meterPerSecSec[ZAXIS], gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], 0, 0, 0);
#endif
}

void sendSerialAttitude() {
	mavlink_msg_attitude_send(chan, millis(), kinematicsAngle[XAXIS], kinematicsAngle[YAXIS], kinematicsAngle[ZAXIS], 0, 0, 0);
}

void sendSerialHudData() {
	//TODO: getBaroAltitude() returns invalid value?
#if defined(HeadingMagHold)
#if defined(AltitudeHoldBaro)
#if defined (UseGPS)
	if(gpsData.state > GPS_NOFIX) {
		mavlink_msg_vfr_hud_send(chan, (float)getGpsSpeed() / 100.0f, (float)getGpsSpeed() / 100.0f, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360, (receiverCommand[THROTTLE]-1000)/10, getBaroAltitude(), 0.0);
	}
	else {
		mavlink_msg_vfr_hud_send(chan, 0, 0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360, (receiverCommand[THROTTLE]-1000)/10, getBaroAltitude(), 0.0);
	}
#else
	mavlink_msg_vfr_hud_send(chan, 0.0, 0.0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360, (receiverCommand[THROTTLE]-1000)/10, getBaroAltitude(), 0.0);
#endif
#else
	mavlink_msg_vfr_hud_send(chan, 0.0, 0.0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360, (receiverCommand[THROTTLE]-1000)/10, 0, 0.0);
#endif
#else
#if defined(AltitudeHoldBaro)
	mavlink_msg_vfr_hud_send(chan, 0.0, 0.0, 0, (receiverCommand[THROTTLE]-1000)/10, getBaroAltitude(), 0.0);
#else
	mavlink_msg_vfr_hud_send(chan, 0.0, 0.0, 0, (receiverCommand[THROTTLE]-1000)/10, 0, 0.0);
#endif
#endif
}

void sendSerialGpsPostion() {
#if defined(UseGPS)
	if (haveAGpsLock())
	{
#if defined(AltitudeHoldBaro)
		mavlink_msg_global_position_int_send(chan, millis(), currentPosition.latitude, currentPosition.longitude, getGpsAltitude(), getBaroAltitude(), 65535, getGpsSpeed(), getCourse(), ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360);
#else
		mavlink_msg_global_position_int_send(chan, millis(), currentPosition.latitude, currentPosition.longitude, getGpsAltitude(), getGpsAltitude(), 0, 0, 0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360);
#endif
	}
#endif
}

void sendSerialNavControllerOutput() {
#if defined(UseGPSNavigator)
	if(waypointIndex > -1) {
		mavlink_msg_nav_controller_output_send(chan, gpsRollAxisCorrection, gpsPitchAxisCorrection, gpsYawAxisCorrection, desiredHeading, distanceToNextWaypoint, (missionPositionToReach.altitude / 10 - getBaroAltitude()), 0, crossTrackError);
	}
#endif
}

void sendSerialMotorOutput() {
	mavlink_msg_servo_output_raw_send(chan, millis(), 0, motorCommand[MOTOR1], motorCommand[MOTOR2], motorCommand[MOTOR3], motorCommand[MOTOR4], motorCommand[MOTOR5], motorCommand[MOTOR6],motorCommand[MOTOR7], motorCommand[MOTOR8]);
}

void sendSerialRawPressure() {
#if defined(AltitudeHoldBaro)
	mavlink_msg_raw_pressure_send(chan, millis(), readRawPressure(), 0,0, readRawTemperature());
#endif
}

void sendSerialScaledPressure() {
#if defined(AltitudeHoldBaro)
	mavlink_msg_scaled_pressure_send(chan, millis(), pressure, 0, readRawTemperature());
#endif
}

void sendSerialRcRaw() {
#if defined(UseRSSIFaileSafe)
	mavlink_msg_rc_channels_raw_send(chan, millis(), 0, receiverCommand[XAXIS], receiverCommand[YAXIS], receiverCommand[THROTTLE], receiverCommand[ZAXIS], receiverCommand[MODE], receiverCommand[AUX1], receiverCommand[AUX2], receiverCommand[AUX3], rssiRawValue * 2.55);
#else
	mavlink_msg_rc_channels_raw_send(chan, millis(), 0, receiverCommand[XAXIS], receiverCommand[YAXIS], receiverCommand[THROTTLE], receiverCommand[ZAXIS], receiverCommand[MODE], receiverCommand[AUX1], receiverCommand[AUX2], receiverCommand[AUX3], 255);
#endif
}

void sendSerialRcScaled() {
	// RC values scaled from "1000 - 2000" to "-10000 - 10000"
#if defined(UseRSSIFaileSafe)
	mavlink_msg_rc_channels_scaled_send(chan, millis(), 0, (receiverCommand[XAXIS] - 1500) * 20, (receiverCommand[YAXIS] - 1500) * 20, (receiverCommand[THROTTLE] - 1500) * 20, (receiverCommand[ZAXIS] - 1500) * 20, (receiverCommand[MODE] - 1500) * 20, (receiverCommand[AUX1] - 1500) * 20, (receiverCommand[AUX2] - 1500) * 20, (receiverCommand[AUX3] - 1500) * 20, rssiRawValue * 2.55);
#else
	mavlink_msg_rc_channels_scaled_send(chan, millis(), 0, (receiverCommand[XAXIS] - 1500) * 20, (receiverCommand[YAXIS] - 1500) * 20, (receiverCommand[THROTTLE] - 1500) * 20, (receiverCommand[ZAXIS] - 1500) * 20, (receiverCommand[MODE] - 1500) * 20, (receiverCommand[AUX1] - 1500) * 20, (receiverCommand[AUX2] - 1500) * 20, (receiverCommand[AUX3] - 1500) * 20, 255);
#endif
}

void sendSerialSysStatus() {
	uint32_t controlSensorsPresent = 0;
	uint32_t controlSensorsEnabled;
	uint32_t controlSensorsHealthy;
	
	// first what sensors/controllers we have
	if (GYRO_DETECTED) {
		controlSensorsPresent |= MAV_SYS_STATUS_SENSOR_3D_GYRO; // 3D gyro present
	}
	if (ACCEL_DETECTED) {
		controlSensorsPresent |= MAV_SYS_STATUS_SENSOR_3D_ACCEL; // 3D accelerometer present
	}
#if defined(HeadingMagHold)
	if (MAG_DETECTED) {
		controlSensorsPresent |= MAV_SYS_STATUS_SENSOR_3D_MAG; // compass present
	}
#endif
#if defined(AltitudeHoldBaro)
	if (BARO_DETECTED) {
		controlSensorsPresent |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE; // absolute pressure sensor present
	}
#endif
#if defined(UseGPS)
	if (gpsData.state > 0) {
		controlSensorsPresent |= MAV_SYS_STATUS_SENSOR_GPS; // GPS present
	}
#endif

	controlSensorsPresent |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
	if(ACCEL_DETECTED) {
		controlSensorsPresent |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
	}
	controlSensorsPresent |= MAV_SYS_STATUS_SENSOR_YAW_POSITION; // yaw position
#if defined(AltitudeHoldBaro) || defined(AltitudeHoldRangeFinder)
	controlSensorsPresent |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL; // altitude control
#endif
#if defined(UseGPSNavigator)
	controlSensorsPresent |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL; // X/Y position control
#endif
	controlSensorsPresent |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS; // motor control
	controlSensorsPresent |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER; // RC control

	// all present sensors enabled by default except attitude, altitude and X/Y- and Z-position control which we will set individually
    controlSensorsEnabled = controlSensorsPresent & (~MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION & ~MAV_SYS_STATUS_SENSOR_YAW_POSITION & ~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL & ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);


	controlSensorsEnabled |= (1<<10); // 3D angular rate control
	if (flightMode == ATTITUDE_FLIGHT_MODE) {
		controlSensorsEnabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
	}
	if (headingHoldConfig == ON) {
		controlSensorsEnabled |= MAV_SYS_STATUS_SENSOR_YAW_POSITION; // yaw position
	}
#if defined(AltitudeHoldBaro) || defined(AltitudeHoldRangeFinder)
	if (altitudeHoldState == ON) {
		controlSensorsEnabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL; // altitude control
	}
#endif
#if defined(UseGPSNavigator)
	if (positionHoldState == ON || navigationState == ON) {
		controlSensorsEnabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL; // X/Y position control
	}
#endif

	// at the moment all sensors/controllers are assumed healthy
	controlSensorsHealthy = controlSensorsPresent;

#if defined(BattMonitor)
#if defined(BM_EXTENDED)
	mavlink_msg_sys_status_send(chan, controlSensorsPresent, controlSensorsEnabled, controlSensorsHealthy, 0, batteryData[0].voltage * 10, (int)(batteryData[0].current*1000), -1, system_dropped_packets, 0, 0, 0, 0, 0);
#else
	mavlink_msg_sys_status_send(chan, &msg, controlSensorsPresent, controlSensorEnabled, controlSensorsHealthy, 0, batteryData[0].voltage * 10, -1, -1, system_dropped_packets, 0, 0, 0, 0, 0);
#endif
#else
	mavlink_msg_sys_status_send(chan, &msg, controlSensorsPresent, controlSensorEnabled, controlSensorsHealthy, 0, 0, 0, 0, system_dropped_packets, 0, 0, 0, 0, 0);
#endif
}

void sendSerialPID(int IDPid, const char id_p[], const char id_i[], const char id_d[], const char id_windUp[], int listsize, int index) {

	int counter = 0;
	if (id_p != 0) {
		mavlink_msg_param_value_send(chan, id_p, PID[IDPid].P, parameterType, listsize, index);
		counter++;
	}

	if (id_i != 0) {
		mavlink_msg_param_value_send(chan, id_i, PID[IDPid].I, parameterType, listsize, index + counter);
		counter++;
	}

	if (id_d != 0) {
		mavlink_msg_param_value_send(chan, id_d, PID[IDPid].D, parameterType, listsize, index + counter);
		counter++;
	}

	if (id_windUp != 0) {
		mavlink_msg_param_value_send(chan, id_windUp, PID[IDPid].windupGuard, parameterType, listsize, index + counter);
	}
}

void sendSerialParameter(float parameterID, const char parameterName[], int listsize, int index) {
	mavlink_msg_param_value_send(chan, (char*)parameterName, parameterID, parameterType, listsize, index);
}

void sendSerialParameter(int parameterID, const char parameterName[], int listsize, int index) {
	mavlink_msg_param_value_send(chan, (char*)parameterName, parameterID, parameterType, listsize, index);
}

void sendSerialParameter(byte parameterID, const char parameterName[], int listsize, int index) {
	mavlink_msg_param_value_send(chan, (char*)parameterName, parameterID, parameterType, listsize, index);
}

void sendSerialParameter(unsigned long parameterID, const char parameterName[], int listsize, int index) {
	mavlink_msg_param_value_send(chan, (char*)parameterName, parameterID, parameterType, listsize, index);
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

#if defined(BattMonitorAutoDescent)
	sendSerialParameter(batteryMonitorThrottleTarget, parameterNameBattMonThrottleTarget, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(batteryMonitorGoingDownTime, parameterNameBattMonGoingDownTime, parameterListSize, indexCounter);
	indexCounter++;
#endif
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

	sendSerialParameter(MINCOMMAND, parameterNameCh1Min, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MINCOMMAND, parameterNameCh2Min, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MINCOMMAND, parameterNameCh3Min, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MINCOMMAND, parameterNameCh4Min, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MINCOMMAND, parameterNameCh5Min, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MINCOMMAND, parameterNameCh6Min, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MINCOMMAND, parameterNameCh7Min, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MINCOMMAND, parameterNameCh8Min, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MAXCOMMAND, parameterNameCh1Max, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MAXCOMMAND, parameterNameCh2Max, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MAXCOMMAND, parameterNameCh3Max, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MAXCOMMAND, parameterNameCh4Max, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MAXCOMMAND, parameterNameCh5Max, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MAXCOMMAND, parameterNameCh6Max, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MAXCOMMAND, parameterNameCh7Max, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MAXCOMMAND, parameterNameCh8Max, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MIDCOMMAND, parameterNameCh1Trim, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MIDCOMMAND, parameterNameCh2Trim, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(1000, parameterNameCh3Trim, parameterListSize, indexCounter); // Throttle needs lowest position as trim value
	indexCounter++;

	sendSerialParameter(MIDCOMMAND, parameterNameCh4Trim, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MIDCOMMAND, parameterNameCh5Trim, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MIDCOMMAND, parameterNameCh6Trim, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MIDCOMMAND, parameterNameCh7Trim, parameterListSize, indexCounter);
	indexCounter++;

	sendSerialParameter(MIDCOMMAND, parameterNameCh8Trim, parameterListSize, indexCounter);
	indexCounter++;
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
	if (checkParameterMatch(parameterNameTxMode, key)) {
		PIDIndicator = NONE;
		parameterInt = &txMode;
		return -1;
	}
	if (checkParameterMatch(parameterNameCh1Min, key)) {
		PIDIndicator = NONE;
		parameterInt = &receiverRawMinValue[0];
		return -1;
	}
	if (checkParameterMatch(parameterNameCh2Min, key)) {
		PIDIndicator = NONE;
		parameterInt = &receiverRawMinValue[1];
		return -1;
	}
	if (checkParameterMatch(parameterNameCh3Min, key)) {
		PIDIndicator = NONE;
		parameterInt = &receiverRawMinValue[2];
		return -1;
	}
	if (checkParameterMatch(parameterNameCh4Min, key)) {
		PIDIndicator = NONE;
		parameterInt = &receiverRawMinValue[3];
		return -1;
	}
	if (checkParameterMatch(parameterNameCh5Min, key)) {
		PIDIndicator = NONE;
		parameterInt = &receiverRawMinValue[4];
		return -1;
	}
	if (checkParameterMatch(parameterNameCh6Min, key)) {
		PIDIndicator = NONE;
		parameterInt = &receiverRawMinValue[5];
		return -1;
	}
	if (checkParameterMatch(parameterNameCh7Min, key)) {
		PIDIndicator = NONE;
		parameterInt = &receiverRawMinValue[6];
		return -1;
	}
	if (checkParameterMatch(parameterNameCh8Min, key)) {
		PIDIndicator = NONE;
		parameterInt = &receiverRawMinValue[7];
		return -1;
	}
	if (checkParameterMatch(parameterNameCh1Max, key)) {
		PIDIndicator = NONE;
		parameterInt = &receiverRawMaxValue[0];
		return -1;
	}
	if (checkParameterMatch(parameterNameCh2Max, key)) {
		PIDIndicator = NONE;
		parameterInt = &receiverRawMaxValue[1];
		return -1;
	}
	if (checkParameterMatch(parameterNameCh3Max, key)) {
		PIDIndicator = NONE;
		parameterInt = &receiverRawMaxValue[2];
		return -1;
	}
	if (checkParameterMatch(parameterNameCh4Max, key)) {
		PIDIndicator = NONE;
		parameterInt = &receiverRawMaxValue[3];
		return -1;
	}
	if (checkParameterMatch(parameterNameCh5Max, key)) {
		PIDIndicator = NONE;
		parameterInt = &receiverRawMaxValue[4];
		return -1;
	}
	if (checkParameterMatch(parameterNameCh6Max, key)) {
		PIDIndicator = NONE;
		parameterInt = &receiverRawMaxValue[5];
		return -1;
	}
	if (checkParameterMatch(parameterNameCh7Max, key)) {
		PIDIndicator = NONE;
		parameterInt = &receiverRawMaxValue[6];
		return -1;
	}
	if (checkParameterMatch(parameterNameCh8Max, key)) {
		PIDIndicator = NONE;
		parameterInt = &receiverRawMaxValue[7];
		return -1;
	}
	if (checkParameterMatch(parameterNameCh1Trim, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcTrim;
		return -1;
	}
	if (checkParameterMatch(parameterNameCh2Trim, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcTrim;
		return -1;
	}
	if (checkParameterMatch(parameterNameCh3Trim, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcTrim;
		return -1;
	}
	if (checkParameterMatch(parameterNameCh4Trim, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcTrim;
		return -1;
	}
	if (checkParameterMatch(parameterNameCh5Trim, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcTrim;
		return -1;
	}
	if (checkParameterMatch(parameterNameCh6Trim, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcTrim;
		return -1;
	}
	if (checkParameterMatch(parameterNameCh7Trim, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcTrim;
		return -1;
	}
	if (checkParameterMatch(parameterNameCh8Trim, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcTrim;
		return -1;
	}
	if (checkParameterMatch(parameterNameCh1Rev, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcRev;
		return -1;
	}
	if (checkParameterMatch(parameterNameCh2Rev, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcRev;
		return -1;
	}
	if (checkParameterMatch(parameterNameCh3Rev, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcRev;
		return -1;
	}
	if (checkParameterMatch(parameterNameCh4Rev, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcRev;
		return -1;
	}
	if (checkParameterMatch(parameterNameCh5Rev, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcRev;
		return -1;
	}
	if (checkParameterMatch(parameterNameCh6Rev, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcRev;
		return -1;
	}
	if (checkParameterMatch(parameterNameCh7Rev, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcRev;
		return -1;
	}
	if (checkParameterMatch(parameterNameCh8Rev, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcRev;
		return -1;
	}
	if (checkParameterMatch(parameterNameRcMapRoll, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcMap;
		return -1;
	}
	if (checkParameterMatch(parameterNameRcMapPitch, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcMap;
		return -1;
	}
	if (checkParameterMatch(parameterNameRcMapYaw, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcMap;
		return -1;
	}
	if (checkParameterMatch(parameterNameRcMapThrottle, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcMap;
		return -1;
	}
	if (checkParameterMatch(parameterNameRcMapMode, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcMap;
		return -1;
	}	
	if (checkParameterMatch(parameterNameRcMapAux1, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcMap;
		return -1;
	}
	if (checkParameterMatch(parameterNameRcMapAux2, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcMap;
		return -1;
	}
	if (checkParameterMatch(parameterNameRcMapAux3, key)) {
		PIDIndicator = NONE;
		parameterInt = &rcMap;

		// Last parameter needed for RC calibration sent by QGroundControl
		isRcCalibrationNeeded = true;

		return -1;
	}

#if defined(BattMonitor)
	if (checkParameterMatch(parameterNameBattMonAlarmVoltage, key)) {
		PIDIndicator = NONE;
		parameterFloat = &batteryMonitorAlarmVoltage;
		setBatteryCellVoltageThreshold(set.param_value);
		return -1;
	}

#if defined(BattMonitorAutoDescent)
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
	// No parameter found, should not happen
	return -2;
}

void changeAndSendParameter() {
	if(parameterChangeIndicator == 0) {
		if(parameterMatch == -2) { // no parameter matched, should not happen
			parameterChangeIndicator = -1;

			mavlink_msg_statustext_send(chan, MAV_SEVERITY_ERROR, "Write failed");
			return;
		}

		// Only write and emit changes if there is actually a difference AND only write if new value is NOT "not-a-number" AND is NOT infinit
		if (PIDIndicator == P) {
			if (PID[parameterMatch].P != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
				PID[parameterMatch].P = set.param_value;
				writeEEPROM();
				// Report back new value
				mavlink_msg_param_value_send(chan, key, PID[parameterMatch].P, parameterType, parameterListSize, -1);
			}
		}
		else if (PIDIndicator == I) {
			if (PID[parameterMatch].I != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
				PID[parameterMatch].I = set.param_value;
				writeEEPROM();
				// Report back new value
				mavlink_msg_param_value_send(chan, key, PID[parameterMatch].I, parameterType, parameterListSize, -1);
			}
		}
		else if (PIDIndicator == D) {
			if (PID[parameterMatch].D != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
				PID[parameterMatch].D = set.param_value;
				writeEEPROM();
				// Report back new value
				mavlink_msg_param_value_send(chan, key, PID[parameterMatch].D, parameterType, parameterListSize, -1);
			}
		}
		else if (PIDIndicator == windUpGuard) {
			if (PID[parameterMatch].windupGuard != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
				PID[parameterMatch].windupGuard = set.param_value;
				writeEEPROM();
				// Report back new value
				mavlink_msg_param_value_send(chan, key, PID[parameterMatch].windupGuard, parameterType, parameterListSize, -1);
			}
		}
		else if (PIDIndicator == NONE) {
			if (parameterFloat != NULL) {
				if (*parameterFloat != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
					*parameterFloat = set.param_value;
					writeEEPROM();
					// Report back new value
					mavlink_msg_param_value_send(chan, key, *parameterFloat, parameterType, parameterListSize, -1);
				}
			}
			else if (parameterByte != NULL) {
				if (*parameterByte != (byte)set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
					*parameterByte = (byte)set.param_value;
					writeEEPROM();
					// Report back new value
					mavlink_msg_param_value_send(chan, key, (float)*parameterByte, parameterType, parameterListSize, -1);
				}
			}
			else if (parameterInt != NULL) {
				if (*parameterInt != (int)set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
					*parameterInt = (int)set.param_value;
					writeEEPROM();
					// Report back new value
					mavlink_msg_param_value_send(chan, key, (float)*parameterInt, parameterType, parameterListSize, -1);
				}
			}
			else if (parameterULong != NULL) {
				if (*parameterULong != (unsigned long)set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
					*parameterULong = (unsigned long)set.param_value;
					writeEEPROM();
					// Report back new value
					mavlink_msg_param_value_send(chan, key, (float)*parameterULong, parameterType, parameterListSize, -1);
				}
			}
		}
		parameterChangeIndicator = -1;

		mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Write successful");
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

			mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "All parameters received");
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

		mavlink_msg_mission_request_send(chan, MAV_SYSTEM_ID, MAV_COMPONENT_ID, waypointIndexToBeRequested);
	}
}

bool calculateTransmitterCalibrationValues() {
	for (byte channel = XAXIS; channel < 8; channel++) {
		int diff = receiverRawMaxValue[channel] - receiverRawMinValue[channel];

		if(diff < 100) return false;

		tempReceiverOffset[channel] = 1000 - receiverRawMinValue[channel] * (1000.0 / diff);
		tempReceiverSlope[channel] = 1000.0 / diff;
	}

	return true;
}

void resetAccelCalibrationStatus() {
	isAccelCalibrationStep1Done = false;
	isAccelCalibrationStep2Done = false;
	isAccelCalibrationStep3Done = false;
	isAccelCalibrationStep4Done = false;
	isAccelCalibrationStep5Done = false;
	isAccelCalibrationStep6Done = false;
	isStep1Calibrating = false;
	isStep2Calibrating = false;
	isStep3Calibrating = false;
	isStep4Calibrating = false;
	isStep5Calibrating = false;
	isStep6Calibrating = false;

	for (int16_t i = 0; i < 6; i++) {
		accelMeanData[XAXIS][i] = 0;
		accelMeanData[YAXIS][i] = 0;
		accelMeanData[ZAXIS][i] = 0;
	}

	for (int16_t i = 0; i < NumberOfAccelSamples; i++) {
		accelRawData[XAXIS][i] = 0;
		accelRawData[YAXIS][i] = 0;
		accelRawData[ZAXIS][i] = 0;
	}

	accelRawDataSum[XAXIS] = 0;
	accelRawDataSum[YAXIS] = 0;
	accelRawDataSum[ZAXIS] = 0;
}

void calculateAndStoreAccelCalibrationValues() {
	accelScaleFactor[XAXIS] = 9.8065 / (accelMeanData[XAXIS][4] - (accelMeanData[XAXIS][4] - (((accelMeanData[XAXIS][4] - accelMeanData[XAXIS][5]) / 19.613) * 9.8065))); 
	accelScaleFactor[YAXIS] = 9.8065 / (accelMeanData[YAXIS][2] - (accelMeanData[YAXIS][2] - (((accelMeanData[YAXIS][2] - accelMeanData[YAXIS][3]) / 19.613) * 9.8065)));
	accelScaleFactor[ZAXIS] = 9.8065 / (accelMeanData[ZAXIS][1] - (accelMeanData[ZAXIS][1] - (((accelMeanData[ZAXIS][1] - accelMeanData[ZAXIS][0]) / 19.613) * 9.8065)));

	computeAccelBias();    
	storeSensorsZeroToEEPROM();
	writeEEPROM();
	zeroIntegralError();
}

void resetMagCalibrationValues() {
	for (int16_t axis = XAXIS; axis <= ZAXIS; axis++) {
		measuredMagMax[axis] = 0;
		measuredMagMin[axis] = 0;
	}

}

void calculateAndStoreMagCalibrationValues() {
	magBias[XAXIS] = ((measuredMagMax[XAXIS] - measuredMagMin[XAXIS]) / 2) - measuredMagMax[XAXIS];
	magBias[YAXIS] = ((measuredMagMax[YAXIS] - measuredMagMin[YAXIS]) / 2) - measuredMagMax[YAXIS];
	magBias[ZAXIS] = ((measuredMagMax[ZAXIS] - measuredMagMin[ZAXIS]) / 2) - measuredMagMax[ZAXIS];

	writeEEPROM();
	zeroIntegralError();
}

void handleMessage(mavlink_message_t msg) {
	float x = 0, y = 0, z = 0;
	uint8_t result = MAV_RESULT_UNSUPPORTED;
	uint8_t isCurrentWaypoint = 0;
	bool suppressCommandAckMsg = false;

	switch(msg.msgid) {

	case MAVLINK_MSG_ID_COMMAND_LONG:
		mavlink_command_long_t commandsendet;
		mavlink_msg_command_long_decode(&msg, &commandsendet);

		switch(commandsendet.command) {
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
			if (commandsendet.param1 == 1.0f) {
				if(motorArmed) {
					armMotors();
					result = MAV_RESULT_ACCEPTED;
				}
				else {
					result = MAV_RESULT_TEMPORARILY_REJECTED;
				}
			}
			else if (commandsendet.param1 == 0.0f) {
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
			if (commandsendet.param1 == 1.0f) {
				initHomeBase();
			}
			else {
				homePosition.latitude = 1.0e7f * commandsendet.param5;
				homePosition.longitude = 1.0e7f * commandsendet.param6;
				homePosition.altitude = 1.0e2f * commandsendet.param7;
			}
			result = MAV_RESULT_ACCEPTED;
#else
			result = MAV_RESULT_UNSUPPORTED;
#endif
			break;

		case MAV_CMD_PREFLIGHT_CALIBRATION: //241, calibration of acc/gyro/mag/transmitter
			if (!motorArmed) {
				if (commandsendet.param1 == 1.0f) {
					// gyro calibration
					calibrateGyro();
					storeSensorsZeroToEEPROM();

					mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Gyro calibration successfully");

					// Suppress command ack message to prevent the statustext above from disappearing
					suppressCommandAckMsg = true;
				}
				if (commandsendet.param2 == 1.0f) {
					// start mag calibration
#if defined HeadingMagHold
					if(!isCalibratingMag) {
						isCalibratingMag = true;
						resetMagCalibrationValues();
						magCalibrationTimeStarted = millis();

						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Mag calibration started");
					}
					else {
						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Mag calibration already started");
					}

					// Suppress command ack message to prevent the statustext above from disappearing
					suppressCommandAckMsg = true;
#else
					result = MAV_RESULT_UNSUPPORTED;
#endif
				}
				if (commandsendet.param2 == 2.0f) {
					// cancel mag calibration
#if defined HeadingMagHold
					if(isCalibratingMag) {
						isCalibratingMag = false;

						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Mag calibration cancelled");
					}
					else {
						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Mag calibration not started yet");
					}

					// Suppress command ack message to prevent the statustext above from disappearing
					suppressCommandAckMsg = true;
#else
					result = MAV_RESULT_UNSUPPORTED;
#endif
				}
				if (commandsendet.param2 == 3.0f) {
					// finish mag calibration
#if defined HeadingMagHold
					if(isCalibratingMag) {
						isCalibratingMag = false;

						calculateAndStoreMagCalibrationValues();

						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Mag calibration successfully");
					}
					else {
						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Start mag calibration first");
					}

					// Suppress command ack message to prevent the statustext above from disappearing
					suppressCommandAckMsg = true;
#else
					result = MAV_RESULT_UNSUPPORTED;
#endif
				}
				if(commandsendet.param3 == 1.0f) {
#if defined(AltitudeHoldBaro) 
					// reset baro altitude
					measureGroundBaro();
					baroAltitude = baroGroundAltitude;

					mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Altitude reset successfully");

					// Suppress command ack message to prevent the statustext above from disappearing
					suppressCommandAckMsg = true;
#else
					result = MAV_RESULT_UNSUPPORTED;
#endif
				}
				if (commandsendet.param4 == 1.0f) {
					// Start accel calibration procedure
					if(!isCalibratingAccel) {
						accelCalibrationLastTimeRequested = millis();
						isCalibratingAccel = true;
						resetAccelCalibrationStatus();

						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Accel calibration started");
					}
					else {
						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Accel calibration already started");
					}

					// Suppress command ack message to prevent the statustext above from disappearing
					suppressCommandAckMsg = true;
				}
				if (commandsendet.param4 == 2.0f) {
					// Cancel accel calibration procedure
					if(isCalibratingAccel) {
						isCalibratingAccel = false;

						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Accel calibration cancelled");
					}
					else {
						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Accel calibration not started yet");
					}

					// Suppress command ack message to prevent the statustext above from disappearing
					suppressCommandAckMsg = true;
				}
				if (commandsendet.param4 == 3.0f) {
					// Finish accel calibration procedure
					if(isCalibratingAccel) {
						if(areAllStepsCompleted) {
							calculateAndStoreAccelCalibrationValues();

							areAllStepsCompleted = false;
							isCalibratingAccel = false;

							mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Accel calibration successfully");
						}
						else {
							mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Perform all steps first");
						}
					}
					else {
						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Accel calibration not started yet");
					}

					// Suppress command ack message to prevent the statustext above from disappearing
					suppressCommandAckMsg = true;
				}
				if (commandsendet.param5 == 1.0f) {
					// Perform accel calibration step 1
					if(isCalibratingAccel) {
						isStep1Calibrating = true;
					}
					else {
						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Start Accel calibration first");
					}
					// Suppress command ack message to prevent the statustext above from disappearing
					suppressCommandAckMsg = true;
				}
				if (commandsendet.param5 == 2.0f) {
					// Perform accel calibration step 2
					if(isCalibratingAccel) {
						isStep2Calibrating = true;
					}
					else {
						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Start Accel calibration first");
					}
					// Suppress command ack message to prevent the statustext above from disappearing
					suppressCommandAckMsg = true;
				}
				if (commandsendet.param5 == 3.0f) {
					// Perform accel calibration step 3
					if(isCalibratingAccel) {
						isStep3Calibrating = true;
					}
					else {
						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Start Accel calibration first");
					}
					// Suppress command ack message to prevent the statustext above from disappearing
					suppressCommandAckMsg = true;
				}
				if (commandsendet.param5 == 4.0f) {
					// Perform accel calibration step 4
					if(isCalibratingAccel) {
						isStep4Calibrating = true;
					}
					else {
						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Start Accel calibration first");
					}
					// Suppress command ack message to prevent the statustext above from disappearing
					suppressCommandAckMsg = true;
				}
				if (commandsendet.param5 == 5.0f) {
					// Perform accel calibration step 5
					if(isCalibratingAccel) {
						isStep5Calibrating = true;
					}
					else {
						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Start Accel calibration first");
					}
					// Suppress command ack message to prevent the statustext above from disappearing
					suppressCommandAckMsg = true;
				}
				if (commandsendet.param5 == 6.0f) {
					// Perform accel calibration step 6
					if(isCalibratingAccel) {
						isStep6Calibrating = true;
					}
					else {
						mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Start Accel calibration first");
					}
					// Suppress command ack message to prevent the statustext above from disappearing
					suppressCommandAckMsg = true;
				}
				if(commandsendet.param6 == 1.0f) {
					// Initalize EEPROM to default values
					initializeEEPROM();
					writeEEPROM();
					storeSensorsZeroToEEPROM();
					calibrateGyro();
					zeroIntegralError();
#if defined(HeadingMagHold)
					initializeMagnetometer();
#endif
#if defined(AltitudeHoldBaro)
					initializeBaro();
#endif
					result = MAV_RESULT_ACCEPTED;
				}
				if(commandsendet.param7 == 1.0f) {
					// Initalize transmitter calibration parameters to default values
					for (byte channel = XAXIS; channel < LASTCHANNEL; channel++) {
						receiverSlope[channel] = 1.0;
						receiverOffset[channel] = 0.0;
						receiverSmoothFactor[channel] = 1.0;
					}
					mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Transmitter calibration parameters initialized");

					// Suppress command ack message to prevent the statustext above from disappearing
					suppressCommandAckMsg = true;
				}
			}
			else {
				result = MAV_RESULT_TEMPORARILY_REJECTED;
			}
			break;

		case MAV_CMD_PREFLIGHT_STORAGE: //245, reading/writing of parameter list requested by GCS while in Preflight mode
			if (mavlink_msg_command_long_get_param1(&msg) == 0.0f) {
				// Read parameters from EEPROM
				paramListPartIndicator = indexCounter = 0;
				result = MAV_RESULT_ACCEPTED;
			}
			else if (mavlink_msg_command_long_get_param1(&msg) == 1.0f) {
				// Write all parameters to EEPROM
				writeEEPROM();
				result = MAV_RESULT_ACCEPTED;
			}
			break;

		default:
			result = MAV_RESULT_UNSUPPORTED;
			break;
		}

		if(!suppressCommandAckMsg) {
			mavlink_msg_command_ack_send(chan, commandsendet.command, result);
		}
		break;

	case MAVLINK_MSG_ID_SET_MODE: // set the base mode (only arming/disarming makes sense for now)
		__mavlink_set_mode_t modesendet;
		mavlink_msg_set_mode_decode(&msg, &modesendet);

		if(modesendet.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY) {
			if(!motorArmed) {
				armMotors();
			}
		}
		else if(!(modesendet.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY)) {
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
			// No parameter matched, should not happen
			break;
		}
		if (PIDIndicator == P) {
			mavlink_msg_param_value_send(chan, key, PID[parameterMatch].P, parameterType, parameterListSize, -1);
		}
		else if (PIDIndicator == I) {
			mavlink_msg_param_value_send(chan, key, PID[parameterMatch].I, parameterType, parameterListSize, -1);
		}
		else if (PIDIndicator == D) {
			mavlink_msg_param_value_send(chan, key, PID[parameterMatch].D, parameterType, parameterListSize, -1);
		}
		else if (PIDIndicator == windUpGuard) {
			mavlink_msg_param_value_send(chan, key, PID[parameterMatch].windupGuard, parameterType, parameterListSize, -1);
		}
		else if (PIDIndicator == NONE) {
			if (parameterFloat != NULL) {
				mavlink_msg_param_value_send(chan, key, *parameterFloat, parameterType, parameterListSize, -1);
			}
			else if (parameterByte != NULL) {
				mavlink_msg_param_value_send(chan, key, (float)*parameterByte, parameterType, parameterListSize, -1);
			}
			else if (parameterInt != NULL) {
				mavlink_msg_param_value_send(chan, key, (float)*parameterInt, parameterType, parameterListSize, -1);
			}
			else if (parameterULong != NULL) {
				mavlink_msg_param_value_send(chan, key, (float)*parameterULong, parameterType, parameterListSize, -1);
			}
		}
		mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Read successful");
		break;

	case MAVLINK_MSG_ID_PARAM_SET:  // set one specific onboard parameter
		if(!motorArmed) { // added for security reason, as the software is shortly blocked by this command
			mavlink_msg_param_set_decode(&msg, &set);
			key = (char*) set.param_id;
			parameterMatch = findParameter(key);
			parameterChangeIndicator = 0;
			changeAndSendParameter();

			// Check if we received the last parameter needed for transmitter calibration
			if(isRcCalibrationNeeded) {
				isRcCalibrationNeeded = false;

				if(calculateTransmitterCalibrationValues()) {
					for (byte channel = XAXIS; channel < 8; channel++) {
						receiverOffset[channel] = tempReceiverOffset[channel];
						receiverSlope[channel] = tempReceiverSlope[channel];
					}
					writeEEPROM();

					mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Transmitter calibration successfully");
				}
				else {
					mavlink_msg_statustext_send(chan, MAV_SEVERITY_ERROR, "Transmitter calibration failed");
				}
			}
		}
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:  // requesting complete waypoint list from AQ
#if defined(UseGPSNavigator)
		mavlink_msg_mission_count_send(chan, MAV_SYSTEM_ID, MAV_COMPONENT_ID, missionNbPoint + 1);

		waypointTimeLastSent = millis();
		waypointSending = true;
		waypointReceiving = false;
#endif
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST: // GCS requests a specific waypoint
#if defined(UseGPSNavigator)
		__mavlink_mission_request_t requestedWaypointsendet;
		mavlink_msg_mission_request_decode(&msg, &requestedWaypointsendet);

		waypointIndexToBeSent = requestedWaypointsendet.seq;

		isCurrentWaypoint = 0;
		if(waypointIndexToBeSent == waypointIndex) {
			isCurrentWaypoint = 1;
		}

		// command needs scaling
		x = waypoint[waypointIndexToBeSent].latitude / 1.0e7f;
		y = waypoint[waypointIndexToBeSent].longitude / 1.0e7f;
		z = waypoint[waypointIndexToBeSent].altitude / 1.0e2f;

		mavlink_msg_mission_item_send(chan, MAV_SYSTEM_ID, MAV_COMPONENT_ID, requestedWaypointsendet.seq,
			navFrame, MAV_CMD_NAV_WAYPOINT, isCurrentWaypoint, 1, 0, waypointCaptureDistance, 0, 0, x, y, z);

		// update last waypoint comm stamp
		waypointTimeLastSent = millis();
#endif
		break;

	case MAVLINK_MSG_ID_MISSION_COUNT: // number of waypoints to be sent from GCS to AQ
#if defined(UseGPSNavigator)
		__mavlink_mission_count_t waypointListsendet;
		mavlink_msg_mission_count_decode(&msg, &waypointListsendet);

		if(waypointListsendet.count > MAX_WAYPOINTS) {
			//If this happens, GCS tries to send more waypoints than allowed nevertheless and fails with time out error
			// but all allowed waypoints are correctly uploaded
			mavlink_msg_statustext_send(chan, MAV_SEVERITY_ERROR, "Max. 16 waypoints allowed!");

			waypointListsendet.count = MAX_WAYPOINTS;
		}

		waypointsToBeRequested = waypointListsendet.count;
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
		result = MAV_MISSION_ACCEPTED;

		__mavlink_mission_item_t waypointsendet;
		mavlink_msg_mission_item_decode(&msg, &waypointsendet);

		// Check if receiving waypoints (mission upload expected)
		if (!waypointReceiving) {
			result = MAV_MISSION_ERROR;
			mavlink_msg_mission_ack_send(chan, MAV_SYSTEM_ID, MAV_COMPONENT_ID, result);

			mavlink_msg_statustext_send(chan, MAV_SEVERITY_ERROR, "Error - Timeout");
			break;
		}

		// Check if this is the requested waypoint
		if (waypointsendet.seq != waypointIndexToBeRequested) {
			result = MAV_MISSION_INVALID_SEQUENCE;
			mavlink_msg_mission_ack_send(chan, MAV_SYSTEM_ID, MAV_COMPONENT_ID, result);

			mavlink_msg_statustext_send(chan, MAV_SEVERITY_ERROR, "Incorrect waypoint sequence");
			break;
		}

		waypoint[waypointIndexToBeRequested].latitude = 1.0e7f * waypointsendet.x;
		waypoint[waypointIndexToBeRequested].longitude = 1.0e7f * waypointsendet.y;
		waypoint[waypointIndexToBeRequested].altitude = 1.0e2f * waypointsendet.z;
		missionNbPoint++;

		// Update waypoint receiving state machine
		waypointTimeLastReceived = millis();
		waypointTimeLastRequested = 0;
		waypointIndexToBeRequested++;

		if (waypointIndexToBeRequested >= waypointsToBeRequested || waypointIndexToBeRequested > waypointIndexToBeRequestedLast) { // all waypoints received, send ACK message
			mavlink_msg_mission_ack_send(chan, MAV_SYSTEM_ID, MAV_COMPONENT_ID, result);

			waypointReceiving = false;
			isRouteInitialized = false;
			break;
		}     
#endif
		break;

	case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
#if defined(UseGPSNavigator)
		mavlink_mission_write_partial_list_t waypointPartialListsendet;
		mavlink_msg_mission_write_partial_list_decode(&msg, &waypointPartialListsendet);

		if (waypointPartialListsendet.start_index > missionNbPoint ||
			waypointPartialListsendet.end_index > missionNbPoint ||
			waypointPartialListsendet.end_index < waypointPartialListsendet.start_index) {
				mavlink_msg_statustext_send(chan, MAV_SEVERITY_ERROR, "Mission update rejected");
				break;
		}

		waypointTimeLastReceived = millis();
		waypointTimeLastRequested = 0;
		waypointReceiving = true;
		waypointIndexToBeRequested = waypointPartialListsendet.start_index;
		waypointIndexToBeRequestedLast = waypointPartialListsendet.end_index;
#endif
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
			mavlink_msg_mission_ack_send(chan, MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAV_MISSION_ACCEPTED);
		}

		mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Mission deleted");
#endif
		break;

	case MAVLINK_MSG_ID_MISSION_ACK: 
#if defined(UseGPSNavigator)
		// turn off waypoint sending
		waypointSending = false;

		mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Waypoint OK");
#endif
		break;
	}
}

void readSerialCommand() {
	while(SERIAL_PORT.available() > 0) {
		uint8_t c = comm_receive_ch(chan);
		//try to get a new message
		if (mavlink_parse_char(chan, c, &msg, &status)) {
			// Handle message
			handleMessage(msg);
		}
	}

	system_dropped_packets += status.packet_rx_drop_count;

	if (!isCalibratingMag && !isCalibratingAccel && !waypointReceiving && !waypointSending) {
		return;
	}

	if(isCalibratingAccel) { 
		if(isStep1Calibrating) {
			mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Calibrating (Step 1)...");

			accelCalibrationLastTimeRequested = millis();

			// Measure 100 samples of each axis
			if(currentSampleCounter < NumberOfAccelSamples) {
				if(currentSampleCounter == 0) {
					accelRawDataSum[XAXIS] = 0;
					accelRawDataSum[YAXIS] = 0;
					accelRawDataSum[ZAXIS] = 0;
				}

				unsigned long currentMillis = millis();

				if(currentMillis - accelCalibrationLastSampleTime > AccelCalibrationSampleDelay) {
					accelCalibrationLastSampleTime = currentMillis;  
					measureAccelSum();

					accelRawData[XAXIS][currentSampleCounter] = (int)(accelSample[XAXIS]/accelSampleCount);
					accelRawDataSum[XAXIS] += accelRawData[XAXIS][currentSampleCounter];
					accelSample[XAXIS] = 0;

					accelRawData[YAXIS][currentSampleCounter] = (int)(accelSample[YAXIS]/accelSampleCount);
					accelRawDataSum[YAXIS] += accelRawData[YAXIS][currentSampleCounter];
					accelSample[YAXIS] = 0;

					accelRawData[ZAXIS][currentSampleCounter] = (int)(accelSample[ZAXIS]/accelSampleCount);
					accelRawDataSum[ZAXIS] += accelRawData[ZAXIS][currentSampleCounter];
					accelSample[ZAXIS] = 0;

					accelSampleCount = 0;
					currentSampleCounter++;
				}
			}
			else {
				isAccelCalibrationStep1Done = true;
				isStep1Calibrating = false;
				currentSampleCounter = 0;

				// Calculate and store mean values for each axis
				accelMeanData[XAXIS][0] = accelRawDataSum[XAXIS] / (float)NumberOfAccelSamples;
				accelMeanData[YAXIS][0] = accelRawDataSum[YAXIS] / (float)NumberOfAccelSamples;
				accelMeanData[ZAXIS][0] = accelRawDataSum[ZAXIS] / (float)NumberOfAccelSamples;

				// Clear raw data
				for (int16_t i = 0; i < NumberOfAccelSamples; i++) {
					accelRawData[XAXIS][i] = 0;
					accelRawData[YAXIS][i] = 0;
					accelRawData[ZAXIS][i] = 0;
				}

				// Check if all steps completed
				if(isAccelCalibrationStep1Done && isAccelCalibrationStep2Done && isAccelCalibrationStep3Done && isAccelCalibrationStep4Done && isAccelCalibrationStep5Done && isAccelCalibrationStep6Done) {
					areAllStepsCompleted = true;
				}

				mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Completed Step 1");
			}
		}
		if(isStep2Calibrating) {
			mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Calibrating (Step 2)...");

			accelCalibrationLastTimeRequested = millis();

			// Measure 100 samples of each axis
			if(currentSampleCounter < NumberOfAccelSamples) {
				if(currentSampleCounter == 0) {
					accelRawDataSum[XAXIS] = 0;
					accelRawDataSum[YAXIS] = 0;
					accelRawDataSum[ZAXIS] = 0;
				}

				unsigned long currentMillis = millis();

				if(currentMillis - accelCalibrationLastSampleTime > AccelCalibrationSampleDelay) {
					accelCalibrationLastSampleTime = currentMillis;  
					measureAccelSum();

					accelRawData[XAXIS][currentSampleCounter] = (int)(accelSample[XAXIS]/accelSampleCount);
					accelRawDataSum[XAXIS] += accelRawData[XAXIS][currentSampleCounter];
					accelSample[XAXIS] = 0;

					accelRawData[YAXIS][currentSampleCounter] = (int)(accelSample[YAXIS]/accelSampleCount);
					accelRawDataSum[YAXIS] += accelRawData[YAXIS][currentSampleCounter];
					accelSample[YAXIS] = 0;

					accelRawData[ZAXIS][currentSampleCounter] = (int)(accelSample[ZAXIS]/accelSampleCount);
					accelRawDataSum[ZAXIS] += accelRawData[ZAXIS][currentSampleCounter];
					accelSample[ZAXIS] = 0;

					accelSampleCount = 0;
					currentSampleCounter++;
				}
			}
			else {
				isAccelCalibrationStep2Done = true;
				isStep2Calibrating = false;
				currentSampleCounter = 0;

				// Calculate and store mean values for each axis
				accelMeanData[XAXIS][1] = accelRawDataSum[XAXIS] / (float)NumberOfAccelSamples;
				accelMeanData[YAXIS][1] = accelRawDataSum[YAXIS] / (float)NumberOfAccelSamples;
				accelMeanData[ZAXIS][1] = accelRawDataSum[ZAXIS] / (float)NumberOfAccelSamples;

				// Clear raw data
				for (int16_t i = 0; i < NumberOfAccelSamples; i++) {
					accelRawData[XAXIS][i] = 0;
					accelRawData[YAXIS][i] = 0;
					accelRawData[ZAXIS][i] = 0;
				}

				// Check if all steps completed
				if(isAccelCalibrationStep1Done && isAccelCalibrationStep2Done && isAccelCalibrationStep3Done && isAccelCalibrationStep4Done && isAccelCalibrationStep5Done && isAccelCalibrationStep6Done) {
					areAllStepsCompleted = true;
				}

				mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Completed Step 2");
			}
		}
		if(isStep3Calibrating) {
			mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Calibrating (Step 3)...");

			accelCalibrationLastTimeRequested = millis();

			// Measure 100 samples of each axis
			if(currentSampleCounter < NumberOfAccelSamples) {
				if(currentSampleCounter == 0) {
					accelRawDataSum[XAXIS] = 0;
					accelRawDataSum[YAXIS] = 0;
					accelRawDataSum[ZAXIS] = 0;
				}

				unsigned long currentMillis = millis();

				if(currentMillis - accelCalibrationLastSampleTime > AccelCalibrationSampleDelay) {
					accelCalibrationLastSampleTime = currentMillis;  
					measureAccelSum();

					accelRawData[XAXIS][currentSampleCounter] = (int)(accelSample[XAXIS]/accelSampleCount);
					accelRawDataSum[XAXIS] += accelRawData[XAXIS][currentSampleCounter];
					accelSample[XAXIS] = 0;

					accelRawData[YAXIS][currentSampleCounter] = (int)(accelSample[YAXIS]/accelSampleCount);
					accelRawDataSum[YAXIS] += accelRawData[YAXIS][currentSampleCounter];
					accelSample[YAXIS] = 0;

					accelRawData[ZAXIS][currentSampleCounter] = (int)(accelSample[ZAXIS]/accelSampleCount);
					accelRawDataSum[ZAXIS] += accelRawData[ZAXIS][currentSampleCounter];
					accelSample[ZAXIS] = 0;

					accelSampleCount = 0;
					currentSampleCounter++;
				}
			}
			else {
				isAccelCalibrationStep3Done = true;
				isStep3Calibrating = false;
				currentSampleCounter = 0;

				// Calculate and store mean values for each axis
				accelMeanData[XAXIS][2] = accelRawDataSum[XAXIS] / (float)NumberOfAccelSamples;
				accelMeanData[YAXIS][2] = accelRawDataSum[YAXIS] / (float)NumberOfAccelSamples;
				accelMeanData[ZAXIS][2] = accelRawDataSum[ZAXIS] / (float)NumberOfAccelSamples;

				// Clear raw data
				for (int16_t i = 0; i < NumberOfAccelSamples; i++) {
					accelRawData[XAXIS][i] = 0;
					accelRawData[YAXIS][i] = 0;
					accelRawData[ZAXIS][i] = 0;
				}

				// Check if all steps completed
				if(isAccelCalibrationStep1Done && isAccelCalibrationStep2Done && isAccelCalibrationStep3Done && isAccelCalibrationStep4Done && isAccelCalibrationStep5Done && isAccelCalibrationStep6Done) {
					areAllStepsCompleted = true;
				}

				mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Completed Step 3");
			}
		}
		if(isStep4Calibrating) {
			mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Calibrating (Step 4)...");

			accelCalibrationLastTimeRequested = millis();

			// Measure 100 samples of each axis
			if(currentSampleCounter < NumberOfAccelSamples) {
				if(currentSampleCounter == 0) {
					accelRawDataSum[XAXIS] = 0;
					accelRawDataSum[YAXIS] = 0;
					accelRawDataSum[ZAXIS] = 0;
				}

				unsigned long currentMillis = millis();

				if(currentMillis - accelCalibrationLastSampleTime > AccelCalibrationSampleDelay) {
					accelCalibrationLastSampleTime = currentMillis;  
					measureAccelSum();

					accelRawData[XAXIS][currentSampleCounter] = (int)(accelSample[XAXIS]/accelSampleCount);
					accelRawDataSum[XAXIS] += accelRawData[XAXIS][currentSampleCounter];
					accelSample[XAXIS] = 0;

					accelRawData[YAXIS][currentSampleCounter] = (int)(accelSample[YAXIS]/accelSampleCount);
					accelRawDataSum[YAXIS] += accelRawData[YAXIS][currentSampleCounter];
					accelSample[YAXIS] = 0;

					accelRawData[ZAXIS][currentSampleCounter] = (int)(accelSample[ZAXIS]/accelSampleCount);
					accelRawDataSum[ZAXIS] += accelRawData[ZAXIS][currentSampleCounter];
					accelSample[ZAXIS] = 0;

					accelSampleCount = 0;
					currentSampleCounter++;
				}
			}
			else {
				isAccelCalibrationStep4Done = true;
				isStep4Calibrating = false;
				currentSampleCounter = 0;

				// Calculate and store mean values for each axis
				accelMeanData[XAXIS][3] = accelRawDataSum[XAXIS] / (float)NumberOfAccelSamples;
				accelMeanData[YAXIS][3] = accelRawDataSum[YAXIS] / (float)NumberOfAccelSamples;
				accelMeanData[ZAXIS][3] = accelRawDataSum[ZAXIS] / (float)NumberOfAccelSamples;

				// Clear raw data
				for (int16_t i = 0; i < NumberOfAccelSamples; i++) {
					accelRawData[XAXIS][i] = 0;
					accelRawData[YAXIS][i] = 0;
					accelRawData[ZAXIS][i] = 0;
				}

				// Check if all steps completed
				if(isAccelCalibrationStep1Done && isAccelCalibrationStep2Done && isAccelCalibrationStep3Done && isAccelCalibrationStep4Done && isAccelCalibrationStep5Done && isAccelCalibrationStep6Done) {
					areAllStepsCompleted = true;
				}

				mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Completed Step 4");
			}
		}
		if(isStep5Calibrating) {
			mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Calibrating (Step 5)...");

			accelCalibrationLastTimeRequested = millis();

			// Measure 100 samples of each axis
			if(currentSampleCounter < NumberOfAccelSamples) {
				if(currentSampleCounter == 0) {
					accelRawDataSum[XAXIS] = 0;
					accelRawDataSum[YAXIS] = 0;
					accelRawDataSum[ZAXIS] = 0;
				}

				unsigned long currentMillis = millis();

				if(currentMillis - accelCalibrationLastSampleTime > AccelCalibrationSampleDelay) {
					accelCalibrationLastSampleTime = currentMillis;  
					measureAccelSum();

					accelRawData[XAXIS][currentSampleCounter] = (int)(accelSample[XAXIS]/accelSampleCount);
					accelRawDataSum[XAXIS] += accelRawData[XAXIS][currentSampleCounter];
					accelSample[XAXIS] = 0;

					accelRawData[YAXIS][currentSampleCounter] = (int)(accelSample[YAXIS]/accelSampleCount);
					accelRawDataSum[YAXIS] += accelRawData[YAXIS][currentSampleCounter];
					accelSample[YAXIS] = 0;

					accelRawData[ZAXIS][currentSampleCounter] = (int)(accelSample[ZAXIS]/accelSampleCount);
					accelRawDataSum[ZAXIS] += accelRawData[ZAXIS][currentSampleCounter];
					accelSample[ZAXIS] = 0;

					accelSampleCount = 0;
					currentSampleCounter++;
				}
			}
			else {
				isAccelCalibrationStep5Done = true;
				isStep5Calibrating = false;
				currentSampleCounter = 0;

				// Calculate and store mean values for each axis
				accelMeanData[XAXIS][4] = accelRawDataSum[XAXIS] / (float)NumberOfAccelSamples;
				accelMeanData[YAXIS][4] = accelRawDataSum[YAXIS] / (float)NumberOfAccelSamples;
				accelMeanData[ZAXIS][4] = accelRawDataSum[ZAXIS] / (float)NumberOfAccelSamples;

				// Clear raw data
				for (int16_t i = 0; i < NumberOfAccelSamples; i++) {
					accelRawData[XAXIS][i] = 0;
					accelRawData[YAXIS][i] = 0;
					accelRawData[ZAXIS][i] = 0;
				}

				// Check if all steps completed
				if(isAccelCalibrationStep1Done && isAccelCalibrationStep2Done && isAccelCalibrationStep3Done && isAccelCalibrationStep4Done && isAccelCalibrationStep5Done && isAccelCalibrationStep6Done) {
					areAllStepsCompleted = true;
				}

				mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Completed Step 5");
			}
		}
		if(isStep6Calibrating) {
			mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Calibrating (Step 6)...");

			accelCalibrationLastTimeRequested = millis();

			// Measure 100 samples of each axis
			if(currentSampleCounter < NumberOfAccelSamples) {
				if(currentSampleCounter == 0) {
					accelRawDataSum[XAXIS] = 0;
					accelRawDataSum[YAXIS] = 0;
					accelRawDataSum[ZAXIS] = 0;
				}

				unsigned long currentMillis = millis();

				if(currentMillis - accelCalibrationLastSampleTime > AccelCalibrationSampleDelay) {
					accelCalibrationLastSampleTime = currentMillis;  
					measureAccelSum();

					accelRawData[XAXIS][currentSampleCounter] = (int)(accelSample[XAXIS]/accelSampleCount);
					accelRawDataSum[XAXIS] += accelRawData[XAXIS][currentSampleCounter];
					accelSample[XAXIS] = 0;

					accelRawData[YAXIS][currentSampleCounter] = (int)(accelSample[YAXIS]/accelSampleCount);
					accelRawDataSum[YAXIS] += accelRawData[YAXIS][currentSampleCounter];
					accelSample[YAXIS] = 0;

					accelRawData[ZAXIS][currentSampleCounter] = (int)(accelSample[ZAXIS]/accelSampleCount);
					accelRawDataSum[ZAXIS] += accelRawData[ZAXIS][currentSampleCounter];
					accelSample[ZAXIS] = 0;

					accelSampleCount = 0;
					currentSampleCounter++;
				}
			}
			else {
				isAccelCalibrationStep6Done = true;
				isStep6Calibrating = false;
				currentSampleCounter = 0;

				// Calculate and store mean values for each axis
				accelMeanData[XAXIS][5] = accelRawDataSum[XAXIS] / (float)NumberOfAccelSamples;
				accelMeanData[YAXIS][5] = accelRawDataSum[YAXIS] / (float)NumberOfAccelSamples;
				accelMeanData[ZAXIS][5] = accelRawDataSum[ZAXIS] / (float)NumberOfAccelSamples;

				// Clear raw data
				for (int16_t i = 0; i < NumberOfAccelSamples; i++) {
					accelRawData[XAXIS][i] = 0;
					accelRawData[YAXIS][i] = 0;
					accelRawData[ZAXIS][i] = 0;
				}

				// Check if all steps completed
				if(isAccelCalibrationStep1Done && isAccelCalibrationStep2Done && isAccelCalibrationStep3Done && isAccelCalibrationStep4Done && isAccelCalibrationStep5Done && isAccelCalibrationStep6Done) {
					areAllStepsCompleted = true;
				}

				mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Completed Step 6");
			}
		}
	}

	if(isCalibratingMag) {
		float rawXaxis = getMagnetometerRawData(XAXIS);
		float rawYaxis = getMagnetometerRawData(YAXIS);
		float rawZaxis = getMagnetometerRawData(ZAXIS);

		if (rawXaxis > measuredMagMax[XAXIS]) {
			measuredMagMax[XAXIS] = rawXaxis;
		}
		else if (rawXaxis < measuredMagMin[XAXIS]) {
			measuredMagMin[XAXIS] = rawXaxis; 
		}

		if (rawYaxis > measuredMagMax[YAXIS]) {
			measuredMagMax[YAXIS] = rawYaxis;
		}
		else if (rawYaxis < measuredMagMin[YAXIS]) {
			measuredMagMin[YAXIS] = rawYaxis; 
		}

		if (rawZaxis > measuredMagMax[ZAXIS]) {
			measuredMagMax[ZAXIS] = rawZaxis;
		}
		else if (rawZaxis < measuredMagMin[ZAXIS]) {
			measuredMagMin[ZAXIS] = rawZaxis; 
		}

		mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Calibrating magnetometer...");
	}

	uint32_t tnow = millis();

	// stop mag if timeout
	if (isCalibratingMag && (tnow - magCalibrationTimeStarted) > magCalibrationTimeout) {
		isCalibratingMag = false;

		mavlink_msg_statustext_send(chan, MAV_SEVERITY_ERROR, "Mag calibration Timeout - Please restart.");
	}

	// stop accel calibration if timeout
	if (isCalibratingAccel && (tnow - accelCalibrationLastTimeRequested) > accelCalibrationTimeout) {
		isCalibratingAccel = false;

		mavlink_msg_statustext_send(chan, MAV_SEVERITY_ERROR, "Accel calibration Timeout - Please restart.");
	}

	if (waypointReceiving && waypointIndexToBeRequested <= waypointsToBeRequested && tnow > waypointTimeLastRequested + 200) {
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
	if (!waypointReceiving && !waypointSending) {
		// Don't interfere with mission transfer
		sendSerialVehicleData();
		sendQueuedParameters();
	}
}

#endif //#define _AQ_MAVLINK_H_
