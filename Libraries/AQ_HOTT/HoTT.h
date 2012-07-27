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

#ifndef _AEROQUAD_HOTT_H_
#define _AEROQUAD_HOTT_H_

#define HOTTV4_UPDATE_INTERVAL 2000

#if defined BattMonitor
#define HOTTV4BATT
#endif

#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
#define HOTTV4ALTITUDE
#endif

#if defined HeadingMagHold
#define HOTTV4DIR
#endif

/** ###### HoTT module specific settings ###### */

#define HOTTV4_GENERAL_AIR_SENSOR_ID 0xD0

#define HOTTV4_ELECTRICAL_AIR_SENSOR_ID 0xE0 // Electric Air Sensor ID
#define HOTTV4_ELECTRICAL_AIR_MODULE 0x8E // Electric Air Module ID
#define HOTTV4_ELECTRICAL_AIR_TEXTMODE 0x7F // Electrical Air Module Text Mode ID

#define HOTTV4_GPS_SENSOR_ID 0xA0 // GPS Sensor ID
#define HOTTV4_GPS_MODULE  0x8A  // GPS Module ID

#define HOTTV4_VARIO_SENSOR_ID 0x90 // Vario Sensor ID
#define HOTTV4_VARIO_MODULE 0x89 // Vario Sensor Module ID

#if !defined (HOTTV4_TX_DELAY) 
#define HOTTV4_TX_DELAY 600
#endif

/** ###### VARIO Text ###### */

#define HOTTV4_VARIO_ATTITUDE "AeroQuad (Attitude)"
#define HOTTV4_VARIO_RATE     "AeroQuad (Rate)"

/** ###### Common settings ###### */

#define VARIO_ASCIIS 21

typedef enum {
	HoTTGPSComingHome     = 'W',
	HoTTGPSPositionHold   = 'P',
	HoTTGPSFree           = '/'
	} HoTTGPSChar;

//most notifications not used yet, for future use
typedef enum {
	HoTTv4NotificationErrorCalibration     = 0x01,
	HoTTv4NotificationErrorReceiver        = 0x02,
	HoTTv4NotificationErrorDataBus         = 0x03,
	HoTTv4NotificationErrorNavigation      = 0x04,
	HoTTv4NotificationErrorError           = 0x05,
	HoTTv4NotificationErrorCompass         = 0x06,
	HoTTv4NotificationErrorSensor          = 0x07,
	HoTTv4NotificationErrorGPS             = 0x08,
	HoTTv4NotificationErrorMotor           = 0x09,

	HoTTv4NotificationMaxTemperature       = 0x0A,
	HoTTv4NotificationAltitudeReached      = 0x0B,
	HoTTv4NotificationWaypointReached      = 0x0C,
	HoTTv4NotificationNextWaypoint         = 0x0D,
	HoTTv4NotificationLanding              = 0x0E,
	HoTTv4NotificationGPSFix               = 0x0F,
	HoTTv4NotificationUndervoltage         = 0x10,
	HoTTv4NotificationGPSHold              = 0x11,
	HoTTv4NotificationGPSHome              = 0x12,
	HoTTv4NotificationGPSOff               = 0x13,
	HoTTv4NotificationBeep                 = 0x14,
	HoTTv4NotificationMicrocopter          = 0x15,
	HoTTv4NotificationCapacity             = 0x16,
	HoTTv4NotificationCareFreeOff          = 0x17,
	HoTTv4NotificationCalibrating          = 0x18,
	HoTTv4NotificationMaxRange             = 0x19,
	HoTTv4NotificationMaxAltitude          = 0x1A,

	HoTTv4Notification20Meter              = 0x25,
	HoTTv4NotificationMicrocopterOff       = 0x26,
	HoTTv4NotificationAltitudeOn           = 0x27,
	HoTTv4NotificationAltitudeOff          = 0x28,
	HoTTv4Notification100Meter             = 0x29,
	HoTTv4NotificationCareFreeOn           = 0x2E,
	HoTTv4NotificationDown                 = 0x2F,
	HoTTv4NotificationUp                   = 0x30,
	HoTTv4NotificationHold                 = 0x31,
	HoTTv4NotificationGPSOn                = 0x32,
	HoTTv4NotificationFollowing            = 0x33,
	HoTTv4NotificationStarting             = 0x34,
	HoTTv4NotificationReceiver             = 0x35,
} HoTTv4Notification;

// Function declarations
static void hottV4EnableReceiverMode();
static void hottV4EnableTransmitterMode();
static void hottV4SerialWrite(uint8_t data);
static void hottV4LoopUntilRegistersReady();
static void hottV4SendBinary(uint8_t *data);
static void hottV4UpdateDirection(uint8_t *data);
static void hottV4TriggerNotification(uint8_t *data, uint8_t notification);
static short hottv4UpdateBattery(uint8_t *data);
static short hottv4UpdateCurrent();
static long hottv4UpdateCapacity();
static int32_t hottv4UpdateAlt();
static unsigned int hottv4UpdateAltVario();
static void hottv4UpdateFlightTime(uint8_t *data);
void hottv4Init();
void hottv4Setup();
static void hottV4SendEAMTelemetry();
static void updatePosition(uint8_t *data, uint32_t value, uint8_t index);
static void hottV4SendGPSTelemetry();
static void hottV4SendVarioTelemetry();
static uint8_t hottV4SendChar(char c, uint8_t inverted);
uint8_t canSendTelemetry();
uint8_t hottV4Hook(uint8_t serialData);

#endif