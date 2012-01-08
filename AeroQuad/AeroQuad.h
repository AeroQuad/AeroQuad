/*
  AeroQuad v3.0 - December 2011
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

#ifndef _AQ_GLOBAL_HEADER_DEFINITION_H_
#define _AQ_GLOBAL_HEADER_DEFINITION_H_


#include <stdlib.h>
#include <math.h>
#include "Arduino.h"
#include "pins_arduino.h"

// Flight Software Version
#define SOFTWARE_VERSION 3.0

#define BAUD 115200
//#define BAUD 111111 // use this to be compatible with USB and XBee connections
//#define BAUD 57600

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
//////////////////////////////////////////////////////

/**
 * battery monitor and battery monitor throttle correction global declaration section
 */
int batteyMonitorThrottleCorrection = 0;
#if defined (BattMonitor)
  #define BattMonitorAlarmVoltage 10.0  // required by battery monitor macro, this is overriden by readEEPROM()
  float batteryMonitorAlarmVoltage = 10.0;
  int batteryMonitorStartThrottle = 0;
  int batteryMonitorThrottleTarget = 1450;
  unsigned long batteryMonitorStartTime = 0;
  unsigned long batteryMonitorGoinDownTime = 60000; 
  
  #if defined BattMonitorAutoDescent
    int batteryMonitorAlarmCounter = 0;
    #define BATTERY_MONITOR_MAX_ALARM_COUNT 50
  #endif
#endif
//////////////////////////////////////////////////////

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
byte flightMode = RATE_FLIGHT_MODE;
unsigned long frameCounter = 0; // main loop executive frame counter
int minArmedThrottle = 1150;

float G_Dt = 0.002; 
int throttle = 1000;
byte motorArmed = OFF;
byte safetyCheck = OFF;

float filteredAccelRoll = 0.0;
float filteredAccelPitch = 0.0;
float filteredAccelYaw = 0.0;

// main loop time variable
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;
// sub loop time variable
unsigned long tenHZpreviousTime = 0;
unsigned long fiftyHZpreviousTime = 0;
unsigned long hundredHZpreviousTime = 0;

void readPilotCommands(); 
void calculateFlightError();
void processHeading();
void processAltitudeHold();
void processCalibrateESC();
void processFlightControl();
void processAltitudeHold();
//////////////////////////////////////////////////////

/**
 * Altitude control global declaration
 */
#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
 // special state that allows immediate turn off of Altitude hold if large throttle changesa are made at the TX
  int altitudeHoldBump = 90;
  int altitudeHoldPanicStickMovement = 250;

  float altitudeToHoldTarget = 0.0;
  int altitudeHoldThrottle = 1000;
  boolean isStoreAltitudeNeeded = false;
  boolean altitudeHoldState = OFF;  // ON, OFF or ALTPANIC
  
//  float estimatedAltitude = 0.0;
//  float estimatedZVelocity = 0.0;
//  float altitudeIntegratedError = 0.0;
//  float currentSensorAltitude = 0.0;
#endif
int minThrottleAdjust = -50;
int maxThrottleAdjust = 50;

float getAltitudeFromSensors();
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
 
HardwareSerial *binaryPort;

void readSerialCommand();
void sendSerialTelemetry();
void printInt(int data);
float readFloatSerial();
void sendBinaryFloat(float);
void sendBinaryuslong(unsigned long);
void fastTelemetry();
void comma();
void reportVehicleState();
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
  t_NVR_Receiver RECEIVER_DATA[LASTCHANNEL];
  
  float SOFTWARE_VERSION_ADR;
  float WINDUPGUARD_ADR;
  float XMITFACTOR_ADR;
  float MINARMEDTHROTTLE_ADR;
  float GYROSMOOTH_ADR;
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
  float SERVOMINPITCH_ADR;
  float SERVOMINROLL_ADR;
  float GYRO_ROLL_ZERO_ADR;
  float GYRO_PITCH_ZERO_ADR;
  float GYRO_YAW_ZERO_ADR;
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
} t_NVR_Data;  


void readEEPROM(); 
void initSensorsZeroFromEEPROM();
void storeSensorsZeroToEEPROM();
void initReceiverFromEEPROM();

float nvrReadFloat(int address); // defined in DataStorage.h
void nvrWriteFloat(float value, int address); // defined in DataStorage.h
void nvrReadPID(unsigned char IDPid, unsigned int IDEeprom);
void nvrWritePID(unsigned char IDPid, unsigned int IDEeprom);

#define GET_NVR_OFFSET(param) ((int)&(((t_NVR_Data*) 0)->param))
#define readFloat(addr) nvrReadFloat(GET_NVR_OFFSET(addr))
#define writeFloat(value, addr) nvrWriteFloat(value, GET_NVR_OFFSET(addr))
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

