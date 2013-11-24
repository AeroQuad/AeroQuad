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
#include "Receiver.h"

// Flight Software Version
#define SOFTWARE_VERSION 3.2

#if defined CONFIG_BAUDRATE
  #define BAUD CONFIG_BAUDRATE
#else
  #if defined WirelessTelemetry && !defined MavLink
    #define BAUD 111111 // use this to be compatible with USB and XBee connections
  #else
    #define BAUD 115200
  #endif
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
#define ATTITUDE_FLIGHT_MODE 1
#define SIMPLE_FLIGHT_MODE 2
byte previousFlightMode = ATTITUDE_FLIGHT_MODE;
#define TASK_100HZ 1
#define TASK_50HZ 2
#define TASK_10HZ 10
#define TASK_1HZ 100
#define THROTTLE_ADJUST_TASK_SPEED TASK_50HZ

byte flightMode = RATE_FLIGHT_MODE;
unsigned long frameCounter = 0; // main loop executive frame counter
int minArmedThrottle; // initial value configured by user

byte simpleModeInitialize = true;
float simpleModeStartHeading = 0.0;

float G_Dt = 0.002; 
int throttle = 1000;
byte motorArmed = OFF;
byte safetyCheck = OFF;
byte maxLimit = OFF;
byte minLimit = OFF;
float filteredAccel[3] = {0.0,0.0,0.0};
boolean inFlight = false; // true when motor are armed and that the user pass one time the min throttle
float rotationSpeedFactor = 1.0;

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


// Analog Reference Value
// This value provided from Configurator
// Use a DMM to measure the voltage between AREF and GND
// Enter the measured voltage below to define your value for aref
// If you don't have a DMM use the following:
// AeroQuad Shield v1.7, aref = 3.0
// AeroQuad Shield v1.6 or below, aref = 2.8
float aref; // Read in from EEPROM
//////////////////////////////////////////////////////

/**
 * Heading and heading hold global declaration section
 */
 
byte  headingHoldConfig   = 0;
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
void fastTelemetry();
void comma();
void reportVehicleState();
//////////////////////////////////////////////////////

/**
 * battery monitor and battery monitor throttle correction global declaration section
 */
#if defined (BattMonitor)
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
  byte altitudeHoldState = OFF;  // ON, OFF or ALTPANIC
  int altitudeHoldBump = 90;
  int altitudeHoldPanicStickMovement = 250;
  int minThrottleAdjust = -50;
  int maxThrottleAdjust = 50;
  int altitudeHoldThrottle = 1000;
  boolean isAltitudeHoldInitialized = false;
  
  
  float velocityCompFilter1 = 1.0 / (1.0 + 0.3);
  float velocityCompFilter2 = 1 - velocityCompFilter1;

  boolean runtimaZBiasInitialized = false;  
  float zVelocity = 0.0;
  float estimatedZVelocity = 0.0;
  float runtimeZBias = 0.0; 
  float zDampeningThrottleCorrection = 0.0;

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

    int missionNbPoint = -1; // Index of last waypoint, -1 if no waypoint present
    int gpsRollAxisCorrection = 0;
    int gpsPitchAxisCorrection = 0;
    int gpsYawAxisCorrection = 0;
    boolean isPositionHoldInitialized = false;
    boolean isGpsNavigationInitialized = false;

    int waypointIndex = -1;    
    float distanceToDestination = 99999999.0;
    GeodeticPosition waypoint[MAX_WAYPOINTS] = {
      GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION,
      GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION,
      GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION,
      GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION, GPS_INVALID_POSITION};
    GeodeticPosition positionHoldPointToReach = GPS_INVALID_POSITION;
    
    /* New updated GPS Navigation variables */
    double fromVector[3], toVector[3], presentPosition[3];
    double presentPositionEast[3], presentPositionNorth[3];
    double normalRangeVector[3], rangeVector[3];
    double zVector[3] = {0.0, 0.0, 1.0};
    double normalVector[3], normalPerpendicularVector[3], alongPathVector[3], negNormalVector[3];
    GeodeticPosition fromWaypoint, toWaypoint, currentLocation, followingWaypoint;
    double desiredHeading, currentHeading, groundTrackHeading;
    double trackAngleError, crossTrackError, crossTrack, alongPathDistance;
    int waypointCount;
    double distanceToNextWaypoint = 99999999.0;
    double distanceToFollowingWaypoint = 99999999.0;
    double testDistanceWaypoint = 99999999.0;
    const double earthRadius = 6378100.0; // meters
    double waypointCaptureDistance = 2.0; // meters
    float forwardSpeed = 15.0;
    byte navigatorSerialCommand = OFF; // TODO: remove when autopilot working
    bool isRouteInitialized = false;
    double distanceToGoAlongPath, distanceToGoPosition; // TODO: remove?
    float posRollCommand, posPitchCommand; // TODO: remove?
    long latDelta, lonDelta; // TODO: remove?

    double velocityVector[3];
    float velX = 0.0, velY = 0.0, velZ = 0.0;
    int velRollCommand = 0;
    int velPitchCommand = 0;
    double estSpeed, estCourse;
    double previousLat = 0.0;
    double previousLon = 0.0;
    double gpsVelocity[3], accVelocity[3];

    // make local when working
    float distanceFromStartToPosition;
    float bearingFromStartToPosition;
    float bearingFromStartToNextWP;


    bool evaluateMissionPositionToReach();
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
  float slope;
  float offset;
  float smooth_factor;
} t_NVR_Receiver;

typedef struct {    
  t_NVR_PID ROLL_PID_GAIN_ADR;
  t_NVR_PID LEVELROLL_PID_GAIN_ADR;
  t_NVR_PID YAW_PID_GAIN_ADR;
  t_NVR_PID PITCH_PID_GAIN_ADR;
  t_NVR_PID LEVELPITCH_PID_GAIN_ADR;
  t_NVR_PID HEADING_PID_GAIN_ADR;
  t_NVR_PID LEVEL_GYRO_ROLL_PID_GAIN_ADR;
  t_NVR_PID LEVEL_GYRO_PITCH_PID_GAIN_ADR;
  t_NVR_PID ALTITUDE_PID_GAIN_ADR;
  t_NVR_PID ZDAMP_PID_GAIN_ADR;
  t_NVR_PID GPSROLL_PID_GAIN_ADR;
  t_NVR_PID GPSPITCH_PID_GAIN_ADR;
  t_NVR_PID GPSYAW_PID_GAIN_ADR;
  t_NVR_Receiver RECEIVER_DATA[MAX_NB_CHANNEL];
  
  float SOFTWARE_VERSION_ADR;
  float WINDUPGUARD_ADR;
  float XMITFACTOR_ADR;
  float MINARMEDTHROTTLE_ADR;
  float AREF_ADR;
  float FLIGHTMODE_ADR;
  float HEADINGHOLD_ADR;
  float ACCEL_1G_ADR;
  float ALTITUDE_MAX_THROTTLE_ADR;
  float ALTITUDE_MIN_THROTTLE_ADR;
  float ALTITUDE_SMOOTH_ADR;
  float ALTITUDE_WINDUP_ADR;
  float ALTITUDE_BUMP_ADR;
  float ALTITUDE_PANIC_ADR;
  // Gyro calibration
  float ROTATION_SPEED_FACTOR_ARD;
  // Accel Calibration
  float XAXIS_ACCEL_BIAS_ADR;
  float XAXIS_ACCEL_SCALE_FACTOR_ADR;
  float YAXIS_ACCEL_BIAS_ADR;
  float YAXIS_ACCEL_SCALE_FACTOR_ADR;
  float ZAXIS_ACCEL_BIAS_ADR;
  float ZAXIS_ACCEL_SCALE_FACTOR_ADR;
  // Mag Calibration
  float XAXIS_MAG_BIAS_ADR;
  float YAXIS_MAG_BIAS_ADR;
  float ZAXIS_MAG_BIAS_ADR;
  // Battery Monitor
  float BATT_ALARM_VOLTAGE_ADR;
  float BATT_THROTTLE_TARGET_ADR;
  float BATT_DOWN_TIME_ADR;
  // Range Finder
  float RANGE_FINDER_MAX_ADR;
  float RANGE_FINDER_MIN_ADR;
  // Camera Control
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
  // GPS mission storing
  float GPS_MISSION_NB_POINT_ADR;
  GeodeticPosition WAYPOINT_ADR[MAX_WAYPOINTS];
  float WAYPOINT_CAPTURE_ADR;
  float FORWARD_SPEED_ADR;
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
