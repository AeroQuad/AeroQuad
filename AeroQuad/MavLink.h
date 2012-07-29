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

// MavLink 1.0 DKP
 #include "../mavlink/include/mavlink/v1.0/common/mavlink.h" 

int systemType = MAV_TYPE_QUADROTOR;
int autopilotType = MAV_AUTOPILOT_GENERIC;
uint16_t len;
int systemMode = 	MAV_MODE_PREFLIGHT;
int systemStatus = MAV_STATE_UNINIT;
int parameterType = MAVLINK_TYPE_FLOAT;

static uint16_t millisecondsSinceBoot = 0;
long system_dropped_packets = 0;

mavlink_message_t msg; 
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
mavlink_status_t status;


void readSerialMavLink() {
  while(PORT.available() > 0) { 
    uint8_t c = PORT.read();
    //try to get a new message 
      if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) { 
        // Handle message
        switch(msg.msgid) {

          case MAVLINK_MSG_ID_SET_MODE: {
            systemMode = mavlink_msg_set_mode_get_base_mode(&msg); // TODO check
            sendSerialSysStatus();
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
				if(systemMode = MAV_MODE_PREFLIGHT) {
					if(mavlink_msg_command_long_get_param1(&msg) == 1) calibrateGyro();
					if(mavlink_msg_command_long_get_param1(&msg) == 1) {
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

//           case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
//             int8_t rateRoll_P[15] = "Rate Roll_P";
//             int8_t rateRoll_I[15] = "Rate Roll_I";
//             int8_t rateRoll_D[15] = "Rate Roll_D";
//             sendSerialPID(RATE_XAXIS_PID_IDX, rateRoll_P, rateRoll_I, rateRoll_D, 1, 24);
//             
//             int8_t ratePitch_P[15] = "Rate Pitch_P";
//             int8_t ratePitch_I[15] = "Rate Pitch_I";
//             int8_t ratePitch_D[15] = "Rate Pitch_D";
//             sendSerialPID(RATE_YAXIS_PID_IDX, ratePitch_P, ratePitch_I, ratePitch_D, 4, 24);
//             
//             int8_t yaw_p[15] = "Yaw_P";
//             int8_t yaw_i[15] = "Yaw_I";
//             int8_t yaw_d[15] = "Yaw_D";
//             sendSerialPID(ZAXIS_PID_IDX, yaw_p, yaw_i, yaw_d, 7, 24);
//             
//             int8_t heading_p[15] = "Heading_P";
//             int8_t heading_i[15] = "Heading_I";
//             int8_t heading_d[15] = "Heading_D";
//             sendSerialPID(HEADING_HOLD_PID_IDX, heading_p, heading_i, heading_d, 10, 24);
//             
//             int8_t attitudeRoll_P[15] = "Stable Roll_P";
//             int8_t attitudeRoll_I[15] = "Stable Roll_I";
//             int8_t attitudeRoll_D[15] = "Stable Roll_D";
//             sendSerialPID(ATTITUDE_XAXIS_PID_IDX, attitudeRoll_P, attitudeRoll_I, attitudeRoll_D, 13, 24);
//                         
//             int8_t attitudePitch_P[15] = "Stable Pitch_P";
//             int8_t attitudePitch_i[15] = "Stable Pitch_I";
//             int8_t attitudePitch_d[15] = "Stable Pitch_D";
//             sendSerialPID(ATTITUDE_YAXIS_PID_IDX, attitudePitch_P, attitudePitch_i, attitudePitch_d, 16, 24);
//             
//             int8_t attitudeGyroRoll_P[15] = "Sta Gyro rol_P";
//             int8_t attitudeGyroRoll_I[15] = "Sta Gyro rol_I";
//             int8_t attitudeGyroRoll_D[15] = "Sta Gyro rol_D";
//             sendSerialPID(ATTITUDE_GYRO_XAXIS_PID_IDX, attitudeGyroRoll_P, attitudeGyroRoll_I, attitudeGyroRoll_D, 19, 24);
//             
//             int8_t attitudeGyroPitch_P[15] = "Sta Gyro pit_P";
//             int8_t attitudeGyroPitch_I[15] = "Sta Gyro pit_I";
//             int8_t attitudeGyroPitch_D[15] = "Sta Gyro pit_D";
//             sendSerialPID(ATTITUDE_GYRO_YAXIS_PID_IDX, attitudeGyroPitch_P, attitudeGyroPitch_I, attitudeGyroPitch_D, 22, 24);
//           }
//           break;

          default:
            //Do nothing
          break;
        }
      } 
      // And get the next one
    } 
    system_dropped_packets += status.packet_rx_drop_count;
}

static void updateFlightTime() {
	static uint32_t previousUpdate = 0;

	uint16_t timeDiff = millis() - previousUpdate;
	previousUpdate += timeDiff;

	if (motorArmed) {
		millisecondsSinceBoot += timeDiff;
		}
	}

void sendSerialHeartbeat() {
  mavlink_msg_heartbeat_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, systemType, autopilotType, systemMode, 0, systemStatus);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  PORT.write(buf, len);
}
// 
// void sendSerialRawIMU() {
//   mavlink_msg_raw_imu_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0, meterPerSecSec[XAXIS], meterPerSecSec[YAXIS], meterPerSecSec[ZAXIS], gyroADC[XAXIS], gyroADC[YAXIS], gyroADC[ZAXIS], getMagnetometerRawData(XAXIS), getMagnetometerRawData(YAXIS), getMagnetometerRawData(ZAXIS));
//   len = mavlink_msg_to_send_buffer(buf, &msg);
//   PORT.write(buf, len);
// }
// 
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
       mavlink_msg_global_position_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, currentPosition.latitude*100, currentPosition.longitude*100, baroAltitude, getBaroAltitude()*1000, 0, 0, 0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360);
       len = mavlink_msg_to_send_buffer(buf, &msg);
       PORT.write(buf, len);
     }
   #endif
 }
// 
//  void sendSerialAltitude() {
//    mavlink_msg_set_altitude_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_COMPONENT_ID, (int)(1234));
//    len = mavlink_msg_to_send_buffer(buf, &msg);
//    PORT.write(buf, len);
//  }
 
 void sendSerialRawPressure() {
   mavlink_msg_raw_pressure_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, readRawPressure(), 0,0, readRawTemperature());
   len = mavlink_msg_to_send_buffer(buf, &msg);
   PORT.write(buf, len);
 }
 
//  void sendSerialBoot() {
//     mavlink_msg_auth_key_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg);
//     len = mavlink_msg_to_send_buffer(buf, &msg);
//     PORT.write(buf, len);
//   }
 
 void sendSerialRcRaw() {
 #if defined UseRSSIFaileSafe
   mavlink_msg_rc_channels_raw_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, 0, receiverCommand[THROTTLE], receiverCommand[XAXIS], receiverCommand[YAXIS], receiverCommand[ZAXIS], receiverCommand[MODE], receiverCommand[AUX1], receiverCommand[AUX2], receiverCommand[AUX3], rssiRawValue * 2.55);
 #else 
   mavlink_msg_rc_channels_raw_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, 0, receiverCommand[THROTTLE], receiverCommand[XAXIS], receiverCommand[YAXIS], receiverCommand[ZAXIS], receiverCommand[MODE], receiverCommand[AUX1], receiverCommand[AUX2], receiverCommand[AUX3], 0);
 #endif
   len = mavlink_msg_to_send_buffer(buf, &msg);
   PORT.write(buf, len);
 }
 
 
// void sendSerialPID(int IDPid, int8_t id_p[], int8_t id_i[], int8_t id_d[],int index, int listsize) {
//   mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, id_p, PID[IDPid].P, parameterType, listsize, index);
//   len = mavlink_msg_to_send_buffer(buf, &msg);
//   PORT.write(buf, len);
//   mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, id_i, PID[IDPid].I, parameterType, listsize, index+1);
//   len = mavlink_msg_to_send_buffer(buf, &msg);
//   PORT.write(buf, len);
//   mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, id_d, PID[IDPid].D, parameterType, listsize, index+2);
//   len = mavlink_msg_to_send_buffer(buf, &msg);
//   PORT.write(buf, len);
// }
// 
// void sendSerialParamValue(int8_t id[], float value, int index, int listsize) {
//   mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, id, value, index,listsize);
//   len = mavlink_msg_to_send_buffer(buf, &msg);
//   PORT.write(buf, len);
// }
// 
// 
 void sendSerialSysStatus() {
   if (motorArmed == OFF)
   {
     systemMode = MAV_MODE_MANUAL_DISARMED;
     systemStatus = MAV_STATE_STANDBY;
   }
   else if (motorArmed == ON && flightMode == ATTITUDE_FLIGHT_MODE)
   {
     systemMode = MAV_MODE_STABILIZE_ARMED;
     systemStatus = MAV_STATE_ACTIVE;
   }
   else
   {
     systemMode = MAV_MODE_MANUAL_ARMED;
     systemStatus = MAV_STATE_ACTIVE;
   }

    uint32_t control_sensors_present = 0;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;
	
    // first what sensors/controllers we have
	if (GYRO_DETECTED)   control_sensors_present |= (1<<0); // 3D gyro present
    if (ACCEL_DETECTED)  control_sensors_present |= (1<<1); // 3D accelerometer present
#if defined HeadingMagHold
    if (MAG_DETECTED)   control_sensors_present |= (1<<2); // compass present
#endif
#if defined AltitudeHoldBaro
    if (BARO_DETECTED)   control_sensors_present |= (1<<3); // absolute pressure sensor present
#endif
#if defined UseGPS
	if (gps->valid_read) control_sensors_present |= (1<<5); // GPS present
#endif
    control_sensors_present |= (1<<10); // 3D angular rate control
    control_sensors_present |= (1<<11); // attitude stabilisation
    control_sensors_present |= (1<<12); // yaw position
#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    control_sensors_present |= (1<<13); // altitude control
#endif
#if defined UseGPS
	control_sensors_present |= (1<<14); // X/Y position control
#endif
    control_sensors_present |= (1<<15); // motor control

    // now what sensors/controllers are enabled
	// first the sensors
    control_sensors_enabled = control_sensors_present & 0x1FF;
	
    // now the controllers
    control_sensors_enabled = control_sensors_present & 0x1FF;
	
    control_sensors_enabled |= (1<<10); // 3D angular rate control
	if (flightMode == ATTITUDE_FLIGHT_MODE) control_sensors_enabled |= (1<<11); // attitude stabilisation
#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    if (altitudeHoldState == ON) control_sensors_enabled |= (1<<13); // altitude control
#endif
    control_sensors_enabled |= (1<<15); // motor control
    if (headingHoldConfig == ON) control_sensors_enabled |= (1<<12); // yaw position
#if defined UseGPS
    if (positionHoldState == ON || navigationState == ON) control_sensors_enabled |= (1<<14); // X/Y position control
#endif

    // at the moment all sensors/controllers are assumed healthy
    control_sensors_health = control_sensors_present;

#if defined BattMonitor
   mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, control_sensors_present, control_sensors_enabled, control_sensors_health, 0, batteryData[0].voltage * 10, (int)(batteryData[0].current*1000), -1, system_dropped_packets, 0, 0, 0, 0, 0);
#else
   mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, control_sensors_present, control_sensors_enabled, control_sensors_health, 0, 0, 0, 0, system_dropped_packets, 0, 0, 0, 0, 0);  // system_dropped_packets
#endif

   len = mavlink_msg_to_send_buffer(buf, &msg);
   PORT.write(buf, len);
 }
 #endif

#endif //#define _AQ_MAVLINK_H_
