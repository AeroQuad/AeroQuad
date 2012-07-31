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

	// MavLink.pde is responsible for the serial communication for commands and telemetry from the AeroQuad
	// This comtains readSerialCommand() which listens for a serial command and it's arguments
	// This also contains readSerialTelemetry() which listens for a telemetry request and responds with the requested data

	#ifndef _AQ_MAVLINK_H_
	#define _AQ_MAVLINK_H_

	//***************************************************************************************************
	//********************************** Serial Commands ************************************************
	//***************************************************************************************************
	#ifdef MavLink
		#define PORT Serial //TODO Serial3
		#include "BatteryMonitor.h"
	#endif

	// MavLink 1.0 DKP
	#include "../mavlink/include/mavlink/v1.0/common/mavlink.h" 

	int systemType = MAV_TYPE_QUADROTOR;
	int autopilotType = MAV_AUTOPILOT_GENERIC;
	uint16_t len;
	int systemMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
	int systemStatus = MAV_STATE_UNINIT;
	int parameterType = MAVLINK_TYPE_FLOAT;
	int parameterListSize = 35;

	static uint16_t millisecondsSinceBoot = 0;
	long system_dropped_packets = 0;

	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_status_t status;


	static void updateFlightTime() {
		static uint32_t previousUpdate = 0;

		uint16_t timeDiff = millis() - previousUpdate;
		previousUpdate += timeDiff;

		if (motorArmed) {
			millisecondsSinceBoot += timeDiff;
		}
	}

	void sendSerialHeartbeat() {
		systemMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

		if (flightMode == ATTITUDE_FLIGHT_MODE) {
			systemMode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
		}

		#ifdef UseGPSNavigator
			if (navigationState == ON || positionHoldState == ON) {
				systemMode |= MAV_MODE_FLAG_GUIDED_ENABLED;
			}
		#endif

		systemMode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

		if (motorArmed) {
			systemMode |= MAV_MODE_FLAG_SAFETY_ARMED;
			systemStatus = MAV_STATE_ACTIVE;
		}
		else {
			systemStatus = MAV_STATE_STANDBY;
		}

		mavlink_msg_heartbeat_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, systemType, autopilotType, systemMode, 0, systemStatus);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		PORT.write(buf, len);
	}

	void sendSerialRawIMU() {
		#ifdef HeadingMagHold
			mavlink_msg_raw_imu_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0, meterPerSecSec[XAXIS], meterPerSecSec[YAXIS], meterPerSecSec[ZAXIS], gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], getMagnetometerRawData(XAXIS), getMagnetometerRawData(YAXIS), getMagnetometerRawData(ZAXIS));
		#else
			mavlink_msg_raw_imu_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0, meterPerSecSec[XAXIS], meterPerSecSec[YAXIS], meterPerSecSec[ZAXIS], gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], 0, 0, 0);
		#endif
		len = mavlink_msg_to_send_buffer(buf, &msg);
		PORT.write(buf, len);
	}

	void sendSerialAttitude() {
		mavlink_msg_attitude_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, kinematicsAngle[XAXIS], kinematicsAngle[YAXIS], kinematicsAngle[ZAXIS], 0, 0, 0);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		PORT.write(buf, len);
	}

	void sendSerialHudData() {
		#if defined HeadingMagHold
			#if defined AltitudeHoldBaro
 				mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360, (receiverData[THROTTLE]-1000)/10, getBaroAltitude(), 0.0);
			#else
				mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360, (receiverData[THROTTLE]-1000)/10, 0, 0.0);
			#endif
		#else
			#if defined AltitudeHoldBaro
				mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, 0, (receiverData[THROTTLE]-1000)/10, getBaroAltitude(), 0.0);
			#else
				mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, 0, (receiverData[THROTTLE]-1000)/10, 0, 0.0);
			#endif
		#endif
		len = mavlink_msg_to_send_buffer(buf, &msg);
		PORT.write(buf, len);   
	}

	void sendSerialGpsPostion() {
		#ifdef UseGPS
			if (haveAGpsLock())
			{
				#if defined AltitudeHoldBaro
					mavlink_msg_global_position_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, currentPosition.latitude, currentPosition.longitude, getGpsAltitude() * 10, (getGpsAltitude() - baroGroundAltitude * 100) * 10 , 0, 0, 0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360);
				#else
					mavlink_msg_global_position_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, currentPosition.latitude, currentPosition.longitude, getGpsAltitude() * 10, getGpsAltitude() * 10 , 0, 0, 0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360);
				#endif
				len = mavlink_msg_to_send_buffer(buf, &msg);
				PORT.write(buf, len);
			}
		#endif
	}
 
	void sendSerialRawPressure() {
		#ifdef AltitudeHoldBaro
			mavlink_msg_raw_pressure_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, readRawPressure(), 0,0, readRawTemperature());
			len = mavlink_msg_to_send_buffer(buf, &msg);
			PORT.write(buf, len);
		#endif
	}
 
	void sendSerialRcRaw() {
		#if defined UseRSSIFaileSafe
			mavlink_msg_rc_channels_raw_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, 0, receiverCommand[THROTTLE], receiverCommand[XAXIS], receiverCommand[YAXIS], receiverCommand[ZAXIS], receiverCommand[MODE], receiverCommand[AUX1], receiverCommand[AUX2], receiverCommand[AUX3], rssiRawValue * 2.55);
		#else 
			mavlink_msg_rc_channels_raw_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, 0, receiverCommand[THROTTLE], receiverCommand[XAXIS], receiverCommand[YAXIS], receiverCommand[ZAXIS], receiverCommand[MODE], receiverCommand[AUX1], receiverCommand[AUX2], receiverCommand[AUX3], 0);
		#endif
		len = mavlink_msg_to_send_buffer(buf, &msg);
		PORT.write(buf, len);
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
 

	void sendSerialPID(int IDPid, int8_t id_p[], int8_t id_i[], int8_t id_d[], int8_t id_windUp[], int listsize, int index) {
		int counter = 0;

		if(id_p != 0) {	
			mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)id_p, PID[IDPid].P, parameterType, listsize, index);
			len = mavlink_msg_to_send_buffer(buf, &msg);
			PORT.write(buf, len);
			counter++;
		}

		if(id_i != 0) {
			mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)id_i, PID[IDPid].I, parameterType, listsize, index + counter);
			len = mavlink_msg_to_send_buffer(buf, &msg);
			PORT.write(buf, len);
			counter++;
		}

		if(id_d != 0) {
			mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)id_d, PID[IDPid].D, parameterType, listsize, index + counter);
			len = mavlink_msg_to_send_buffer(buf, &msg);
			PORT.write(buf, len);
			counter++;
		}

		if(id_windUp != 0) {
			mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)id_windUp, PID[IDPid].windupGuard, parameterType, listsize, index + counter);
			len = mavlink_msg_to_send_buffer(buf, &msg);
			PORT.write(buf, len);
		}

	}

	void sendSerialParameter(int parameterID, int8_t parameterName[], int listsize, int index) {
		mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)parameterName, parameterID, parameterType, listsize, index);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		PORT.write(buf, len);
		}
	 
	void sendSerialParamValue(int8_t id[], float value, int listsize, int index) {
		mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)id, value, parameterType, listsize, index);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		PORT.write(buf, len);
	}
	
	void sendParameterList() {
		int indexCounter = 0;
		int8_t rateRoll_P[15] = "Rate Roll_P";
		int8_t rateRoll_I[15] = "Rate Roll_I";
		int8_t rateRoll_D[15] = "Rate Roll_D";
		sendSerialPID(RATE_XAXIS_PID_IDX, rateRoll_P, rateRoll_I, rateRoll_D, 0, parameterListSize, indexCounter);
		indexCounter += 3;

		int8_t ratePitch_P[15] = "Rate Pitch_P";
		int8_t ratePitch_I[15] = "Rate Pitch_I";
		int8_t ratePitch_D[15] = "Rate Pitch_D";
		sendSerialPID(RATE_YAXIS_PID_IDX, ratePitch_P, ratePitch_I, ratePitch_D, 0, parameterListSize, indexCounter);
		indexCounter += 3;

		int8_t attitudeRoll_P[16] = "Att Roll_P";
		int8_t attitudeRoll_I[16] = "Att Roll_I";
		int8_t attitudeRoll_D[16] = "Att Roll_D";
		sendSerialPID(ATTITUDE_XAXIS_PID_IDX, attitudeRoll_P, attitudeRoll_I, attitudeRoll_D, 0, parameterListSize, indexCounter);
		indexCounter += 3;

		int8_t attitudePitch_P[16] = "Att Pitch_P";
		int8_t attitudePitch_i[16] = "Att Pitch_I";
		int8_t attitudePitch_d[16] = "Att Pitch_D";
		sendSerialPID(ATTITUDE_YAXIS_PID_IDX, attitudePitch_P, attitudePitch_i, attitudePitch_d, 0, parameterListSize, indexCounter);
		indexCounter += 3;

		int8_t attitudeGyroRoll_P[16] = "Att Gyro Roll_P";
		int8_t attitudeGyroRoll_I[16] = "Att Gyro Roll_I";
		int8_t attitudeGyroRoll_D[16] = "Att Gyro Roll_D";
		sendSerialPID(ATTITUDE_GYRO_XAXIS_PID_IDX, attitudeGyroRoll_P, attitudeGyroRoll_I, attitudeGyroRoll_D, 0, parameterListSize, indexCounter);
		indexCounter += 3;

		int8_t attitudeGyroPitch_P[16] = "Att Gyro Pitc_P";
		int8_t attitudeGyroPitch_I[16] = "Att Gyro Pitc_I";
		int8_t attitudeGyroPitch_D[16] = "Att Gyro Pitc_D";
		sendSerialPID(ATTITUDE_GYRO_YAXIS_PID_IDX, attitudeGyroPitch_P, attitudeGyroPitch_I, attitudeGyroPitch_D, 0, parameterListSize, indexCounter);
		indexCounter += 3;

		int8_t yaw_p[15] = "Yaw_P";
		int8_t yaw_i[15] = "Yaw_I";
		int8_t yaw_d[15] = "Yaw_D";
		sendSerialPID(ZAXIS_PID_IDX, yaw_p, yaw_i, yaw_d, 0, parameterListSize, indexCounter);
		indexCounter += 3;

		int8_t heading_p[15] = "Heading_P";
		int8_t heading_i[15] = "Heading_I";
		int8_t heading_d[15] = "Heading_D";
		sendSerialPID(HEADING_HOLD_PID_IDX, heading_p, heading_i, heading_d, 0, parameterListSize, indexCounter);
		indexCounter += 3;

		int8_t heading_config[15] = "Heading_Config";
		sendSerialParameter(headingHoldConfig, heading_config, parameterListSize, indexCounter);
		indexCounter++;

		int8_t gyro_smooth_factor[16] = "Misc_GyroSmooth";
		sendSerialParameter(gyroSmoothFactor, gyro_smooth_factor, parameterListSize, indexCounter);
		indexCounter++;

		int8_t a_ref[16] = "Misc_AREF value";
		sendSerialParameter(aref, a_ref, parameterListSize, indexCounter);
		indexCounter++;

		int8_t min_armed_throttle[16] = "Misc_MinThrottl";
		sendSerialParameter(minArmedThrottle, min_armed_throttle, parameterListSize, indexCounter);
		indexCounter++;

		int8_t receiver_xmit_factor[16] = "TX_TX Factor";
		sendSerialParameter(receiverXmitFactor, receiver_xmit_factor, parameterListSize, indexCounter);
		indexCounter++;

		int8_t receiver_smooth_factor_roll[16] = "TX_Roll Smooth";
		sendSerialParameter(receiverSmoothFactor[XAXIS], receiver_smooth_factor_roll, parameterListSize, indexCounter);
		indexCounter++;

		int8_t receiver_smooth_factor_pitch[16] = "TX_Pitch Smooth";
		sendSerialParameter(receiverSmoothFactor[YAXIS], receiver_smooth_factor_pitch, parameterListSize, indexCounter);
		indexCounter++;

		int8_t receiver_smooth_factor_yaw[16] = "TX_Yaw Smooth";
		sendSerialParameter(receiverSmoothFactor[ZAXIS], receiver_smooth_factor_yaw, parameterListSize, indexCounter);
		indexCounter++;

		int8_t receiver_smooth_factor_throttle[16] = "TX_Thr Smooth";
		sendSerialParameter(receiverSmoothFactor[THROTTLE], receiver_smooth_factor_throttle, parameterListSize, indexCounter);
		indexCounter++;

		int8_t receiver_smooth_factor_mode[16] = "TX_Mode Smooth";
		sendSerialParameter(receiverSmoothFactor[MODE], receiver_smooth_factor_mode, parameterListSize, indexCounter);
		indexCounter++;

		int8_t receiver_smooth_factor_aux1[16] = "TX_AUX1 Smooth";
		sendSerialParameter(receiverSmoothFactor[AUX1], receiver_smooth_factor_aux1, parameterListSize, indexCounter);
		indexCounter++;

		if(LASTCHANNEL == 8) {
			int8_t receiver_smooth_factor_aux2[16] = "TX_AUX2 Smooth";
			sendSerialParameter(receiverSmoothFactor[AUX2], receiver_smooth_factor_aux2, parameterListSize, indexCounter);
			indexCounter++;

			int8_t receiver_smooth_factor_aux3[16] = "TX_AUX3 Smooth";
			sendSerialParameter(receiverSmoothFactor[AUX3], receiver_smooth_factor_aux3, parameterListSize, indexCounter);
			indexCounter++;
			}

		#if defined BattMonitor
			int8_t battery_monitor_alarm_voltage[16] = "BatMo_AlarmVolt";
			sendSerialParameter(batteryMonitorAlarmVoltage, battery_monitor_alarm_voltage, parameterListSize, indexCounter);
			indexCounter++;

			int8_t battery_monitor_throttle_target[16] = "BatMo_ThrTarget";
			sendSerialParameter(batteryMonitorThrottleTarget, battery_monitor_throttle_target, parameterListSize, indexCounter);
			indexCounter++;

			int8_t battery_monitor_going_down_time[16] = "BatMo_DownTime";  //TODO seems to produce an overflow, choosing unint32_t as data type doesn't help
			sendSerialParameter(batteryMonitorGoingDownTime, battery_monitor_going_down_time, parameterListSize, indexCounter);
			indexCounter++;
		#endif

		#ifdef CameraControl
			int8_t camera_mode[16] = "Cam_Mode";
			sendSerialParameter(cameraMode, camera_mode, parameterListSize, indexCounter);
			indexCounter++;

			int8_t m_camera_pitch[16] = "Cam_PitchMiddle";
			sendSerialParameter(mCameraPitch, m_camera_pitch, parameterListSize, indexCounter);
			indexCounter++;

			int8_t m_camera_roll[16] = "Cam_RollMiddle";
			sendSerialParameter(mCameraRoll, m_camera_roll, parameterListSize, indexCounter);
			indexCounter++;

			int8_t m_camera_yaw[16] = "Cam_YawMiddle";
			sendSerialParameter(mCameraYaw, m_camera_yaw, parameterListSize, indexCounter);
			indexCounter++;

			int8_t m_servo_pitch[16] = "Cam_ServoPitchM";
			sendSerialParameter(servoCenterPitch, m_servo_pitch, parameterListSize, indexCounter);
			indexCounter++;

			int8_t m_servo_roll[16] = "Cam_ServoRollM";
			sendSerialParameter(servoCenterRoll, m_servo_roll, parameterListSize, indexCounter);
			indexCounter++;

			int8_t m_servo_yaw[16] = "Cam_ServoYawM";
			sendSerialParameter(servoCenterYaw, m_servo_yaw, parameterListSize, indexCounter);
			indexCounter++;

			int8_t servo_min_pitch[16] = "Cam_ServoMinPit";
			sendSerialParameter(servoMinPitch, servo_min_pitch, parameterListSize, indexCounter);
			indexCounter++;

			int8_t servo_min_roll[16] = "Cam_ServoMinRol";
			sendSerialParameter(servoMinRoll, servo_min_roll, parameterListSize, indexCounter);
			indexCounter++;

			int8_t servo_min_yaw[16] = "Cam_ServoMinYaw";
			sendSerialParameter(servoMinYaw, servo_min_yaw, parameterListSize, indexCounter);
			indexCounter++;

			int8_t servo_max_pitch[16] = "Cam_ServoMaxPit";
			sendSerialParameter(servoMaxPitch, servo_max_pitch, parameterListSize, indexCounter);
			indexCounter++;

			int8_t servo_max_roll[16] = "Cam_ServoMaxRol";
			sendSerialParameter(servoMaxRoll, servo_max_roll, parameterListSize, indexCounter);
			indexCounter++;

			int8_t servo_max_yaw[16] = "Cam_ServoMaxYaw";
			sendSerialParameter(servoMaxYaw, servo_max_yaw, parameterListSize, indexCounter);
			indexCounter++;
		#endif

		#if defined (AltitudeHoldBaro) || defined AltitudeHoldRangeFinder
			int8_t min_throttle_adjust[16] = "AH_Min Adjust";
			sendSerialParameter(minThrottleAdjust, min_throttle_adjust, parameterListSize, indexCounter);
			indexCounter++;

			int8_t max_throttle_adjust[16] = "AH_Max Adjust";
			sendSerialParameter(maxThrottleAdjust, max_throttle_adjust, parameterListSize, indexCounter);
			indexCounter++;

			int8_t altitude_hold_bump[16] = "AH_Bump Value";
			sendSerialParameter(altitudeHoldBump, altitude_hold_bump, parameterListSize, indexCounter);
			indexCounter++;

			int8_t altitude_hold_panic_stick_movement[16] = "AH_Panic Value";
			sendSerialParameter(altitudeHoldPanicStickMovement, altitude_hold_panic_stick_movement, parameterListSize, indexCounter);
			indexCounter++;
		#endif 

		#if defined (AltitudeHoldBaro)  && !defined AltitudeHoldRangeFinder
			int8_t baro_smooth_factor[16] = "AH_SmoothFactor";
			sendSerialParameter(baroSmoothFactor, baro_smooth_factor, parameterListSize, indexCounter);
			indexCounter++;

			int8_t baro_p[15] = "Barometer_P";
			int8_t baro_i[15] = "Barometer_I";
			int8_t baro_d[15] = "Barometer_D";
			sendSerialPID(BARO_ALTITUDE_HOLD_PID_IDX, baro_p, baro_i, baro_d, 0, parameterListSize, indexCounter);
			indexCounter += 3;

			int8_t baro_windUpGuard[15] = "Baro_WindUp";
			sendSerialPID(BARO_ALTITUDE_HOLD_PID_IDX, 0, 0, 0, baro_windUpGuard, parameterListSize, indexCounter);
			indexCounter ++;

			int8_t zDampening_p[15] = "Z Dampening_P";
			int8_t zDampening_i[15] = "Z Dampening_I";
			int8_t zDampening_d[15] = "Z Dampening_D";
			sendSerialPID(ZDAMPENING_PID_IDX, zDampening_p, zDampening_i, zDampening_d, 0, parameterListSize, indexCounter);
			indexCounter += 3;
		#endif

		#if defined AltitudeHoldRangeFinder && defined AltitudeHoldBaro
			int8_t baro_smooth_factor[23] = "Altitude Smooth Factor";
			sendSerialParameter(baroSmoothFactor, baro_smooth_factor, parameterListSize, indexCounter);
			indexCounter++;

			int8_t baro_p[15] = "Barometer_P";
			int8_t baro_i[15] = "Barometer_I";
			int8_t baro_d[15] = "Barometer_D";
			sendSerialPID(BARO_ALTITUDE_HOLD_PID_IDX, baro_p, baro_i, baro_d, 0, parameterListSize, indexCounter);
			indexCounter += 3;

			int8_t baro_windUpGuard[16] = "Barometer_WindUp";
			sendSerialPID(BARO_ALTITUDE_HOLD_PID_IDX, 0, 0, 0, baro_windUpGuard, parameterListSize, indexCounter);
			indexCounter ++;

			int8_t zDampening_p[15] = "Z Dampening_P";
			int8_t zDampening_i[15] = "Z Dampening_I";
			int8_t zDampening_d[15] = "Z Dampening_D";
			sendSerialPID(ZDAMPENING_PID_IDX, zDampening_p, zDampening_i, zDampening_d, 0, parameterListSize, indexCounter);
			indexCounter += 3;

			int8_t range_p[15] = "Range_P";
			int8_t range_i[15] = "Range_I";
			int8_t range_d[15] = "Range_D";
			sendSerialPID(SONAR_ALTITUDE_HOLD_PID_IDX, range_p, range_i, range_d, 0, parameterListSize, indexCounter);
			indexCounter += 3;

			int8_t range_windUpGuard[15] = "Range_WindUp";
			sendSerialPID(SONAR_ALTITUDE_HOLD_PID_IDX, 0, 0, 0, range_windUpGuard, parameterListSize, indexCounter);
			indexCounter ++;
		#endif

		#if defined UseGPSNavigator && defined AltitudeHoldRangeFinder && defined AltitudeHoldBaro
			int8_t gps_roll_p[15] = "GPS Roll_P";
			int8_t gps_roll_i[15] = "GPS Roll_I";
			int8_t gps_roll_d[15] = "GPS Roll_D";
			sendSerialPID(GPSROLL_PID_IDX, gps_roll_p, gps_roll_i, gps_roll_d, 0, parameterListSize, indexCounter);
			indexCounter += 3;

			int8_t gps_pitch_p[15] = "GPS Pitch_P";
			int8_t gps_pitch_i[15] = "GPS Pitch_I";
			int8_t gps_pitch_d[15] = "GPS Pitch_D";
			sendSerialPID(GPSPITCH_PID_IDX, gps_pitch_p, gps_pitch_i, gps_pitch_d, 0, parameterListSize, indexCounter);
			indexCounter += 3;
			
			int8_t gps_yaw_p[15] = "GPS Yaw_P";
			int8_t gps_yaw_i[15] = "GPS Yaw_I";
			int8_t gps_yaw_d[15] = "GPS Yaw_D";
			sendSerialPID(GPSYAW_PID_IDX, gps_yaw_p, gps_yaw_i, gps_yaw_d, 0, parameterListSize, indexCounter);
			indexCounter += 3;
		#endif

		#if defined UseGPSNavigator && !defined AltitudeHoldRangeFinder && defined AltitudeHoldBaro
			int8_t gps_roll_p[15] = "GPS Roll_P";
			int8_t gps_roll_i[15] = "GPS Roll_I";
			int8_t gps_roll_d[15] = "GPS Roll_D";
			sendSerialPID(GPSROLL_PID_IDX, gps_roll_p, gps_roll_i, gps_roll_d, 0, parameterListSize, indexCounter);
			indexCounter += 3;

			int8_t gps_pitch_p[15] = "GPS Pitch_P";
			int8_t gps_pitch_i[15] = "GPS Pitch_I";
			int8_t gps_pitch_d[15] = "GPS Pitch_D";
			sendSerialPID(GPSPITCH_PID_IDX, gps_pitch_p, gps_pitch_i, gps_pitch_d, 0, parameterListSize, indexCounter);
			indexCounter += 3;

			int8_t gps_yaw_p[15] = "GPS Yaw_P";
			int8_t gps_yaw_i[15] = "GPS Yaw_I";
			int8_t gps_yaw_d[15] = "GPS Yaw_D";
			sendSerialPID(GPSYAW_PID_IDX, gps_yaw_p, gps_yaw_i, gps_yaw_d, 0, parameterListSize, indexCounter);
			indexCounter += 3;
		#endif

		#if defined AltitudeHoldRangeFinder && !defined AltitudeHoldBaro
			int8_t range_p[15] = "Range_P";
			int8_t range_i[15] = "Range_I";
			int8_t range_d[15] = "Range_D";
			sendSerialPID(SONAR_ALTITUDE_HOLD_PID_IDX, range_p, range_i, range_d, 0, parameterListSize, indexCounter);
			indexCounter += 3;

			int8_t range_windUpGuard[15] = "Range_WindUp";
			sendSerialPID(SONAR_ALTITUDE_HOLD_PID_IDX, 0, 0, 0, range_windUpGuard, parameterListSize, indexCounter);
			indexCounter ++;
		#endif 
	}

	void evaluateParameterListSize() {
		#if defined AltitudeHoldBaro && defined AltitudeHoldRangeFinder && defined UseGPSNavigator
			parameterListSize += 25;
		#endif

		#if defined AltitudeHoldBaro && defined AltitudeHoldRangeFinder && !defined UseGPSNavigator
			parameterListSize += 16;
		#endif

		#if defined AltitudeHoldBaro && !defined AltitudeHoldRangeFinder && !defined UseGPSNavigator
			parameterListSize += 12;
		#endif

		#if !defined AltitudeHoldBaro && defined AltitudeHoldRangeFinder && !defined UseGPSNavigator
			parameterListSize += 8;
		#endif

		#if defined AltitudeHoldBaro && !defined AltitudeHoldRangeFinder && defined UseGPSNavigator
			parameterListSize += 21;
		#endif

		#if defined BattMonitor
			parameterListSize += 3;
		#endif

		#ifdef CameraControl
			parameterListSize += 13;
		#endif

		if (LASTCHANNEL == 8) parameterListSize += 2;
	}

	void sendSerialSysStatus() {
		uint32_t controlSensorsPresent = 0;
		uint32_t controlSensorEnabled;
		uint32_t controlSensorsHealthy;
	
		// first what sensors/controllers we have
		if (GYRO_DETECTED)   controlSensorsPresent |= (1<<0); // 3D gyro present
		if (ACCEL_DETECTED)  controlSensorsPresent |= (1<<1); // 3D accelerometer present
		#if defined HeadingMagHold
			if (MAG_DETECTED)   controlSensorsPresent |= (1<<2); // compass present
		#endif
		#if defined AltitudeHoldBaro
			if (BARO_DETECTED)   controlSensorsPresent |= (1<<3); // absolute pressure sensor present
		#endif
		#if defined UseGPS
			if (gps->valid_read) controlSensorsPresent |= (1<<5); // GPS present
		#endif
		controlSensorsPresent |= (1<<10); // 3D angular rate control
		controlSensorsPresent |= (1<<11); // attitude stabilisation
		controlSensorsPresent |= (1<<12); // yaw position
		#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
			controlSensorsPresent |= (1<<13); // altitude control
		#endif
		#if defined UseGPSNavigator
			controlSensorsPresent |= (1<<14); // X/Y position control
		#endif
		controlSensorsPresent |= (1<<15); // motor control

		// now what sensors/controllers are enabled
		// first the sensors
		controlSensorEnabled = controlSensorsPresent & 0x1FF;
	
		// now the controllers
		controlSensorEnabled = controlSensorsPresent & 0x1FF;
	
		controlSensorEnabled |= (1<<10); // 3D angular rate control
		if (flightMode == ATTITUDE_FLIGHT_MODE) controlSensorEnabled |= (1<<11); // attitude stabilisation
		#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
			if (altitudeHoldState == ON) controlSensorEnabled |= (1<<13); // altitude control
		#endif
		controlSensorEnabled |= (1<<15); // motor control
		if (headingHoldConfig == ON) controlSensorEnabled |= (1<<12); // yaw position
		#if defined UseGPSNavigator
			if (positionHoldState == ON || navigationState == ON) controlSensorEnabled |= (1<<14); // X/Y position control
		#endif

		// at the moment all sensors/controllers are assumed healthy
		controlSensorsHealthy = controlSensorsPresent;

		#if defined BattMonitor
			mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, controlSensorsPresent, controlSensorEnabled, controlSensorsHealthy, 0, batteryData[0].voltage * 10, (int)(batteryData[0].current*1000), -1, system_dropped_packets, 0, 0, 0, 0, 0);
		#else
			mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, controlSensorsPresent, controlSensorEnabled, controlSensorsHealthy, 0, 0, 0, 0, system_dropped_packets, 0, 0, 0, 0, 0);  // system_dropped_packets
		#endif

		len = mavlink_msg_to_send_buffer(buf, &msg);
		PORT.write(buf, len);
	}

	int findParameter(char parameterID[]) { //TODO
		if(parameterID == "Rate Roll_P"){
			return RATE_XAXIS_PID_IDX;
			}
		return LAST_PID_IDX;
		}

	void readSerialMavLink() {
		while(PORT.available() > 0) { 
			uint8_t c = PORT.read();
			//try to get a new message 
			if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) { 
				// Handle message
				switch(msg.msgid) {

					case MAVLINK_MSG_ID_SET_MODE: {
						systemMode = mavlink_msg_set_mode_get_base_mode(&msg); // TODO check
					}
					break;

					case MAVLINK_MSG_ID_COMMAND_LONG: {
						uint8_t result = 0;
						uint8_t command = mavlink_msg_command_long_get_command(&msg);

						if (command == 	MAV_CMD_COMPONENT_ARM_DISARM) {
							if(mavlink_msg_command_long_get_param1(&msg) == 1) motorArmed = ON;
							else if (mavlink_msg_command_long_get_param1(&msg) == 0) motorArmed = OFF;
							result = 	MAV_RESULT_ACCEPTED;
						}

						else if (command == MAV_CMD_DO_SET_MODE) {
							systemMode = mavlink_msg_command_long_get_param1(&msg);
							result = 	MAV_RESULT_ACCEPTED;
						}

						else if (command ==	MAV_CMD_NAV_RETURN_TO_LAUNCH) {
							#if defined UseGPSNavigator 
							//TODO	add coming home
							result = 	MAV_RESULT_ACCEPTED;
							#else
							result = 	MAV_RESULT_UNSUPPORTED;
							#endif
						}

						else if (command == MAV_CMD_NAV_TAKEOFF) {
							#if defined UseGPSNavigator 
							//TODO	add gps takeoff
							result = 	MAV_RESULT_ACCEPTED;
							#else
							result = 	MAV_RESULT_UNSUPPORTED;	
							#endif
						}

						else if (command == MAV_CMD_DO_SET_HOME) {
							#if defined UseGPS
								if(mavlink_msg_command_long_get_param1(&msg) == 1) homePosition = currentPosition;
								else {
									homePosition.latitude = mavlink_msg_command_long_get_param5(&msg);
									homePosition.longitude = mavlink_msg_command_long_get_param6(&msg);
									homePosition.altitude = mavlink_msg_command_long_get_param7(&msg);
									}
							result = 	MAV_RESULT_ACCEPTED;
							#else
							result = 	MAV_RESULT_UNSUPPORTED;
							#endif
						}

						else if (command ==	MAV_CMD_PREFLIGHT_CALIBRATION) {
							if(systemMode == MAV_MODE_PREFLIGHT) {
								if(mavlink_msg_command_long_get_param1(&msg) == 1) calibrateGyro();
								if(mavlink_msg_command_long_get_param2(&msg) == 1) {
									computeAccelBias();
									storeSensorsZeroToEEPROM();
									calibrateKinematics();
									zeroIntegralError();
									}
								result = 	MAV_RESULT_ACCEPTED;
								}
							else result = 	MAV_RESULT_TEMPORARILY_REJECTED;
						}

						mavlink_msg_command_ack_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, command, result);
						len = mavlink_msg_to_send_buffer(buf, &msg);
						PORT.write(buf, len);
					}
					break;

					case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
						sendParameterList();
					}
					break;

					case MAVLINK_MSG_ID_PARAM_REQUEST_READ: { //TODO not working yet
						//uint16_t parameterID = mavlink_msg_param_request_read_get_param_id(&msg, 0);
						//int16_t parameterIndex = mavlink_msg_param_request_read_get_param_index(&msg);

// 						mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)parameterID, PID[parameterID].P, parameterType, 1, parameterIndex);
// 						len = mavlink_msg_to_send_buffer(buf, &msg);
// 						PORT.write(buf, len);
					}
					break;

					case MAVLINK_MSG_ID_PARAM_SET: { //TODO not working yet
						float parameterValue = mavlink_msg_param_set_get_param_value(&msg);
						char parameterID[16];
						int a = mavlink_msg_param_set_get_param_id(&msg, (char*)parameterID);

						int parameterToBeChanged = findParameter(parameterID);
						if(parameterToBeChanged != LAST_PID_IDX) {
								PID[parameterToBeChanged].P = parameterValue; //TODO check how to differ between P/I/D/windUpGuard
								mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)parameterID, PID[parameterToBeChanged].P, parameterType, parameterListSize, -1);
								len = mavlink_msg_to_send_buffer(buf, &msg);
								PORT.write(buf, len);
						}

					}
					break;

					case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: { //TODO needs to be tested
						#if defined UseGPSNavigator
							mavlink_msg_mission_count_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAX_WAYPOINTS);
							len = mavlink_msg_to_send_buffer(buf, &msg);
							PORT.write(buf, len);

							for (byte index = 0; index < MAX_WAYPOINTS; index++) {
								if(index != missionNbPoint) mavlink_msg_mission_item_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SYSTEM_ID, MAV_COMPONENT_ID, index, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, 0, 1, 0, MIN_DISTANCE_TO_REACHED, 0, 0, waypoint[index].longitude, waypoint[index].latitude, waypoint[index].altitude);
								else mavlink_msg_mission_item_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SYSTEM_ID, MAV_COMPONENT_ID, index, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, 1, 1, 0, MIN_DISTANCE_TO_REACHED, 0, 0, waypoint[index].longitude, waypoint[index].latitude, waypoint[index].altitude);
								len = mavlink_msg_to_send_buffer(buf, &msg);
								PORT.write(buf, len);
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

#endif //#define _AQ_MAVLINK_H_
