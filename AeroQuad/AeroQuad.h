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

#ifndef _AQ_GLOBAL_HEADER_DEFINITION_H_
#define _AQ_GLOBAL_HEADER_DEFINITION_H_


#include <stdlib.h>
#include <math.h>
#include "Arduino.h"
#include "pins_arduino.h"
#include "GpsDataType.h"
#include "AQMath.h"
#include "Receiver_Base.h"

// Flight Software Version
#define SOFTWARE_VERSION 3.2

byte LASTMOTOR = 4;

#if defined USE_WIRELESS_COMMUNICATION
  #define BAUD 57600
#else
  #define BAUD 115200
#endif  

/**
 * ESC calibration process global declaration
 */
byte calibrateESC = 0;
int testCommand = 1000;
//////////////////////////////////////////////////////

/**
 * Flight control global declaration
 */
#define RATE_FLIGHT_MODE 0
#define HORIZON_FLIGHT_MODE 1
#define ATTITUDE_FLIGHT_MODE 2

#define TASK_100HZ 1
#define TASK_50HZ 2
#define TASK_10HZ 10
#define TASK_1HZ 100
#define THROTTLE_ADJUST_TASK_SPEED TASK_50HZ

byte flightMode = RATE_FLIGHT_MODE;
unsigned long frameCounter = 0; // main loop executive frame counter
int minArmedThrottle; // initial value configured by user

float G_Dt = 0.002; 
int throttle = 1000;
byte motorArmed = OFF;
byte safetyCheck = OFF;
byte maxLimit = OFF;
byte minLimit = OFF;
float filteredAccel[3] = {0.0,0.0,0.0};
boolean inFlight = false; // true when motor are armed and that the user pass one time the min throttle
float stickScalingFactor = 1.0;

float userRateRollP = 0.0;
float userRateRollD = 0.0;
float userRatePitchP = 0.0;
float userRatePitchD = 0.0;
int throttlePIDAdjustmentFactor = 0;

// main loop time variable
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;
// sub loop time variable
unsigned long oneHZpreviousTime = 0;
unsigned long tenHZpreviousTime = 0;
unsigned long lowPriorityTenHZpreviousTime = 0;
unsigned long lowPriorityTenHZpreviousTime2 = 0;
unsigned long fiftyHZpreviousTime = 0;
unsigned long hundredHZpreviousTime = 0;



//////////////////////////////////////////////////////

/**
 * Heading and heading hold global declaration section
 */
 
float headingHold         = 0; // calculated adjustment for quad to go to heading (PID output)
float heading             = 0; // measured heading from yaw gyro (process variable)
float relativeHeading     = 0; // current heading the quad is set to (set point)
byte  headingHoldState    = OFF;
void  processHeading();
//////////////////////////////////////////////////////


/**
 * Serial communication global declaration
 */
#define SERIAL_PRINT      SERIAL_PORT.print
#define SERIAL_PRINTLN    SERIAL_PORT.println
#define SERIAL_AVAILABLE  SERIAL_PORT.available
#define SERIAL_READ       SERIAL_PORT.read
#define SERIAL_FLUSH      SERIAL_PORT.flush
#define SERIAL_BEGIN      SERIAL_PORT.begin
 
//HardwareSerial *binaryPort;

void readSerialCommand();
void sendSerialTelemetry();
void printInt(int data);
float readFloatSerial();
long readIntegerSerial();
void sendBinaryFloat(float);
void sendBinaryuslong(unsigned long);
void comma();
void reportVehicleState();
//////////////////////////////////////////////////////

/**
 * battery monitor and battery monitor throttle correction global declaration section
 */
#if defined (BattMonitor)
  boolean isBatteryMonitorEnabled = false;
  #define BattMonitorAlarmVoltage 10.0  // required by battery monitor macro, this is overriden by readEEPROM()
  float batteryMonitorAlarmVoltage = 10.0;
  int batteryMonitorStartThrottle = 0;
  int batteryMonitorThrottleTarget = 1450;
  unsigned long batteryMonitorStartTime = 0;
  unsigned long batteryMonitorGoingDownTime = 60000; 

  
  #if defined BattMonitorAutoDescent
    #define BATTERY_MONITOR_MAX_ALARM_COUNT 50
    
    int batteryMonitorAlarmCounter = 0;
    int batteyMonitorThrottleCorrection = 0;
  #endif
#endif
//////////////////////////////////////////////////////




/**
 * Altitude control global declaration
 */
#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
  // special state that allows immediate turn off of Altitude hold if large throttle changesa are made at the TX
 
  #define ALTITUDE_HOLD_STATE 1
  #define VELOCITY_HOLD_STATE 2
  #define ALTPANIC 3
  byte altitudeHoldState = OFF;  // ON, OFF or ALTPANIC
  int altitudeHoldBump = 90;
  int altitudeHoldPanicStickMovement = 250;
  int altitudeHoldThrottle = 1000;
  boolean isAltitudeHoldInitialized = false;
  boolean isVelocityHoldInitialisez = false;
  
  float zVelocity = 0.0;
  float estimatedAltitude = 0.0;
  float previousBaroAltitude = 0.0;
  
  #if defined AltitudeHoldBaro
    float baroAltitudeToHoldTarget = 0.0;
  #endif  
  #if defined AltitudeHoldRangeFinder
    float sonarAltitudeToHoldTarget = 0.0;
  #endif
#endif
//////////////////////////////////////////////////////

/**
 * Auto landing feature variables
 */
#if defined (AutoLanding)
  #define BARO_AUTO_DESCENT_STATE 2
  #define SONAR_AUTO_DESCENT_STATE 3
  #define MOTOR_AUTO_DESCENT_STATE 4
  
  byte autoLandingState = OFF;
  boolean isAutoLandingInitialized = false;
  int autoLandingThrottleCorrection = 0;
#endif

/**
 * GPS navigation global declaration
 */
#define MAX_WAYPOINTS 16  // needed for EEPROM adr offset declarations
#if defined (UseGPS)

  #include <GpsAdapter.h>
  
  #define DEFAULT_HOME_ALTITUDE 5  // default home base altitude is equal to 5 meter
  GeodeticPosition homePosition = GPS_INVALID_POSITION; 
  GeodeticPosition missionPositionToReach = GPS_INVALID_POSITION;  // in case of no GPS navigator, indicate the home position into the OSD

  #if defined UseGPSNavigator
    byte navigationState = OFF;  // ON, OFF or ALTPANIC
    byte positionHoldState = OFF;  // ON, OFF or ALTPANIC

    int missionNbPoint = 0;
    int gpsRollAxisCorrection = 0;
    int gpsPitchAxisCorrection = 0;
    int gpsYawAxisCorrection = 0;
    boolean isStorePositionNeeded = false;
    boolean isInitNavigationNeeded = false;

    int waypointIndex = -1;    
    float gpsDistanceToDestination = 99999999.0;
    GeodeticPosition waypoint[MAX_WAYPOINTS] = {
      GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION,
      GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION,
      GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION,
      GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION};
      
    GeodeticPosition positionHoldPointToReach = GPS_INVALID_POSITION;
    
    void evaluateMissionPositionToReach();
    void processGpsNavigation();
  #endif
#endif
//////////////////////////////////////////////////////

/**
 * EEPROM global section
 */
typedef struct {
  float p;
  float i;
  float d;
} t_NVR_PID;

typedef struct {
  long minValue;
  long maxValue;
} t_NVR_Receiver;

typedef struct {    
  t_NVR_PID ROLL_PID_GAIN_ADR;
  t_NVR_PID LEVELROLL_PID_GAIN_ADR;
  t_NVR_PID YAW_PID_GAIN_ADR;
  t_NVR_PID PITCH_PID_GAIN_ADR;
  t_NVR_PID LEVELPITCH_PID_GAIN_ADR;
  t_NVR_PID HEADING_PID_GAIN_ADR;
  t_NVR_Receiver RECEIVER_DATA[MAX_NB_CHANNEL];
  
  float SOFTWARE_VERSION_ADR;
  float MINARMEDTHROTTLE_ADR;
  float FLIGHTMODE_ADR;
  float ACCEL_1G_ADR;
  
  // Gyro calibration
  float ROTATION_SPEED_FACTOR_ARD;
  long  THROTTLE_PID_ADJUSTMENT_ADR;
  // Accel Calibration
  float XAXIS_ACCEL_BIAS_ADR;
  float XAXIS_ACCEL_SCALE_FACTOR_ADR;
  float YAXIS_ACCEL_BIAS_ADR;
  float YAXIS_ACCEL_SCALE_FACTOR_ADR;
  float ZAXIS_ACCEL_BIAS_ADR;
  float ZAXIS_ACCEL_SCALE_FACTOR_ADR;
  
  float RECEIVER_CONFIG_TYPE_ADR;
  float RECEIVER_CHANNEL_MAP_ADR[MAX_NB_CHANNEL];
  long YAW_DIRECTION_ADR;
  long FLIGHT_CONFIG_TYPE;
  
  // Mag Calibration
  #if defined (HeadingMagHold)
    float XAXIS_MAG_BIAS_ADR;
    float YAXIS_MAG_BIAS_ADR;
    float ZAXIS_MAG_BIAS_ADR;
  #endif    
  
  // Battery Monitor
  #if defined (BattMonitor)
    float BATT_MONITOR_ENABLED_ADR;
    float BATT_ALARM_VOLTAGE_ADR;
    float BATT_THROTTLE_TARGET_ADR;
    float BATT_DOWN_TIME_ADR;
  #endif
  
  // Baro
  #if defined (AltitudeHoldBaro)
    float ALTITUDE_SMOOTH_ADR;
    float ALTITUDE_BUMP_ADR;
    float ALTITUDE_PANIC_ADR;
    t_NVR_PID ALTITUDE_PID_GAIN_ADR;
    t_NVR_PID ZDAMP_PID_GAIN_ADR;
  #endif
  
  // Range Finder
  #if defined (AltitudeHoldRangeFinder)
    float RANGE_FINDER_MAX_ADR;
    float RANGE_FINDER_MIN_ADR;
  #endif
  
  // Camera Control
  #if defined (CameraControl)
    float CAMERAMODE_ADR;
    float MCAMERAPITCH_ADR;
    float MCAMERAROLL_ADR;    
    float MCAMERAYAW_ADR;
    float SERVOCENTERPITCH_ADR;
    float SERVOCENTERROLL_ADR;
    float SERVOCENTERYAW_ADR;
    float SERVOMINPITCH_ADR;
    float SERVOMINROLL_ADR;
    float SERVOMINYAW_ADR;
    float SERVOMAXPITCH_ADR;
    float SERVOMAXROLL_ADR;
    float SERVOMAXYAW_ADR;
    float SERVOTXCHANNELS_ADR;
  #endif

  // GPS mission storing
  #if defined (UseGPS)
    t_NVR_PID GPSROLL_PID_GAIN_ADR;
    t_NVR_PID GPSPITCH_PID_GAIN_ADR;
    t_NVR_PID GPSYAW_PID_GAIN_ADR;
  
    float GPS_MISSION_NB_POINT_ADR;
    GeodeticPosition WAYPOINT_ADR[MAX_WAYPOINTS];
  #endif
} t_NVR_Data;  


void readEEPROM(); 
void initSensorsZeroFromEEPROM();
void storeSensorsZeroToEEPROM();
void initReceiverFromEEPROM();

float nvrReadFloat(int address); // defined in DataStorage.h
void nvrWriteFloat(float value, int address); // defined in DataStorage.h
long nvrReadLong(int address); // defined in DataStorage.h
void nvrWriteLong(long value, int address); // defined in DataStorage.h
void nvrReadPID(unsigned char IDPid, unsigned int IDEeprom);
void nvrWritePID(unsigned char IDPid, unsigned int IDEeprom);

#define GET_NVR_OFFSET(param) ((int)&(((t_NVR_Data*) 0)->param))
#define readFloat(addr) nvrReadFloat(GET_NVR_OFFSET(addr))
#define writeFloat(value, addr) nvrWriteFloat(value, GET_NVR_OFFSET(addr))
#define readLong(addr) nvrReadLong(GET_NVR_OFFSET(addr))
#define writeLong(value, addr) nvrWriteLong(value, GET_NVR_OFFSET(addr))
#define readPID(IDPid, addr) nvrReadPID(IDPid, GET_NVR_OFFSET(addr))
#define writePID(IDPid, addr) nvrWritePID(IDPid, GET_NVR_OFFSET(addr))

/**
 * Debug utility global declaration
 * Debug code should never be part of a release sofware
 * @see Kenny
 */
//#define DEBUG
byte fastTransfer = OFF; // Used for troubleshooting
//unsigned long fastTelemetryTime = 0;
//byte testSignal = LOW;
//////////////////////////////////////////////////////

#endif // _AQ_GLOBAL_HEADER_DEFINITION_H_
