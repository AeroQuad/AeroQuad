/*
  AeroQuad v2.1 - September 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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

/****************************************************************************
   Before flight, select the different user options for your AeroQuad below
   Also, consult the ReadMe.mht file for additional details
   If you need additional assitance go to http://AeroQuad.com/forum
*****************************************************************************/

/****************************************************************************
 ************************* Hardware Configuration ***************************
 ****************************************************************************/
// Select which hardware you wish to use with the AeroQuad Flight Software

//#define AeroQuad_v1         // Arduino 2009 with AeroQuad Shield v1.7 and below
//#define AeroQuad_v18        // Arduino 2009 with AeroQuad Shield v1.8
//#define AeroQuad_Wii        // Arduino 2009 with Wii Sensors and AeroQuad Shield v1.x
//#define AeroQuadMega_v1     // Arduino Mega with AeroQuad Shield v1.7 and below
#define AeroQuadMega_v2     // Arduino Mega with AeroQuad Shield v2.x
//#define AeroQuadMega_Wii    // Arduino Mega with Wii Sensors and AeroQuad Shield v2.x
//#define ArduCopter          // ArduPilot Mega (APM) with APM Sensor Board
//#define Multipilot          // Multipilot board with Lys344 and ADXL 610 Gyro (needs debug)
//#define MultipilotI2C       // Active Multipilot I2C and Mixertable (needs debug)

/****************************************************************************
 *********************** Define Flight Configuration ************************
 ****************************************************************************/
// Use only one of the following definitions

#define plusConfig
//#define XConfig
//#define HEXACOAXIAL
//#define HEXARADIAL

// 5DOF IMU Version
// Uncomment this if you have the version of the 5DOF IMU which uses the older IDG300 or IDG500 gyros
//#define OriginalIMU 

// Yaw Gyro Type
// Use only one of the following definitions
#define IXZ // IXZ-500 Flat Yaw Gyro or ITG-3200 Triple Axis Gyro
//#define IDG // IDG-300 or IDG-500 Dual Axis Gyro

// Camera Stabilization (experimental)
// Not yet fully tested and implemented
//#define Camera

/****************************************************************************
 ********************* End of User Definition Section ***********************
 ****************************************************************************/

#include <EEPROM.h>
//#include <Servo.h>
#include <Wire.h>
#include "AeroQuad.h"
#include "I2C.h"
#include "PID.h"
#include "Filter.h"
#include "Receiver.h"
#include "DataAcquisition.h"
#include "Accel.h"
#include "Gyro.h"
#include "Motors.h"

// Create objects defined from Configuration Section above
#ifdef AeroQuad_v1 
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Receiver_AeroQuad receiver;
  Motors_PWM motors;
  #include "DataStorage.h"
  #include "FlightAngle.h"
  FlightAngle_CompFilter flightAngle;
  //FlightAngle_MultiWii flightAngle;
#endif

#ifdef AeroQuad_v18
  Accel_AeroQuadMega_v2 accel;
  Gyro_AeroQuadMega_v2 gyro;
  Receiver_AeroQuad receiver;
  Motors_PWM motors;
  #include "DataStorage.h"
  #include "FlightAngle.h"
  FlightAngle_CompFilter flightAngle;
  //FlightAngle_DCM flightAngle;
#endif

#ifdef AeroQuadMega_v1 
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Receiver_AeroQuadMega receiver;
  Motors_PWM motors;
  #include "DataStorage.h"
  #include "FlightAngle.h"
  FlightAngle_DCM flightAngle;
#endif

#ifdef AeroQuadMega_v2
  Accel_AeroQuadMega_v2 accel;
  Gyro_AeroQuadMega_v2 gyro;
  Receiver_AeroQuadMega receiver;
  Motors_PWM motors;
  #include "DataStorage.h"
  #include "FlightAngle.h"
  FlightAngle_DCM flightAngle;
  #include "Compass.h"
  Compass_AeroQuad_v2 compass;
  #include "Altitude.h"
  Altitude_AeroQuad_v2 altitude;
#endif

#ifdef ArduCopter
  Gyro_ArduCopter gyro;
  Accel_ArduCopter accel;
  Receiver_ArduCopter receiver;
  Motors_ArduCopter motors;
  #include "FlightAngle.h"
  FlightAngle_DCM flightAngle;
  #include "DataStorage.h"
  #include "Altitude.h"
  Altitude_AeroQuad_v2 altitude;
#endif

#ifdef AeroQuad_Wii
  Accel_Wii accel;
  Gyro_Wii gyro;
  Receiver_AeroQuad receiver;
  Motors_PWM motors;
  #include "DataStorage.h"
  #include "FlightAngle.h"
  FlightAngle_CompFilter flightAngle;
  //FlightAngle_DCM flightAngle;
#endif

#ifdef AeroQuadMega_Wii
  Accel_Wii accel;
  Gyro_Wii gyro;
  Receiver_AeroQuadMega receiver;
  Motors_PWM motors;
  #include "DataStorage.h"
  #include "FlightAngle.h"
  FlightAngle_DCM flightAngle;
#endif

#ifdef Multipilot
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Receiver_Multipilot receiver;
  Motors_PWM motors;
  //#define PRINT_MIXERTABLE
  //#define TELEMETRY_DEBUG
  #include "DataStorage.h"
  #include "FlightAngle.h"
  FlightAngle_DCM flightAngle;
#endif

#ifdef MultipilotI2C  
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Receiver_Multipilot receiver;
  Motors_I2C motors;
  //#define PRINT_MIXERTABLE
  //#define TELEMETRY_DEBUG
  #include "DataStorage.h"
  #include "FlightAngle.h"
  FlightAngle_DCM flightAngle;
#endif

// Angle Estimation Objects
// Class definition for angle estimation found in FlightAngle.h
// Use only one of the following variable declarations
// Insert into the appropriate #idef's above
//#include "FlightAngle.h"
//FlightAngle_CompFilter flightAngle; // Use this for Complementary Filter
//FlightAngle_KalmanFilter flightAngle; // Use this for Kalman Filter
//FlightAngle_IMU flightAngle; // Use this for IMU filter (do not use, for experimentation only)
//FlightAngle_MultiWii flightAngle;

// DCM gyro null values are defined in flightAngle.initalize()
// Review those values is you add a new #define which uses FlightAngle_DCM
//FlightAngle_DCM flightAngle; // Use this for DCM (only for Arduino Mega)

// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
void setup() {
  Serial.begin(BAUD);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  
  #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
    pinMode(LED2PIN, OUTPUT);
    digitalWrite(LED2PIN, LOW);
    pinMode(LED3PIN, OUTPUT);
    digitalWrite(LED3PIN, LOW);
  #endif
  
  #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2) || defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii)
    Wire.begin();
  #endif

  // Read user values from EEPROM
  readEEPROM(); // defined in DataStorage.h
  
  // Configure motors
  motors.initialize(); // defined in Motors.h

  // Setup receiver pins for pin change interrupts
  if (receiverLoop == ON)
    receiver.initialize(); // defined in Received.h
       
  // Initialize sensors
  // If sensors have a common initialization routine
  // insert it into the gyro class because it executes first
  gyro.initialize(); // defined in Gyro.h
  accel.initialize(); // defined in Accel.h
  
  // Calibrate sensors
  gyro.autoZero(); // defined in Gyro.h
  zeroIntegralError();
  levelAdjust[ROLL] = 0;
  levelAdjust[PITCH] = 0;
  
  // Heading hold
  // aref is read in from EEPROM and originates from Configurator
  //headingScaleFactor = (aref / 1024.0) / gyro.getScaleFactor() * (PI/2.0);
  #if defined(AeroQuadMega_v2)
    compass.initialize();
    altitude.initialize();
  #endif
  #if defined(ArduCopter)
    altitude.initialize();
  #endif
  
  // Setup correct sensor orientation
  #ifdef AeroQuad_v1
    gyro.invert(PITCH);
    gyro.invert(ROLL);
  #endif 
  #ifdef AeroQuadMega_v1
    gyro.invert(PITCH);
    gyro.invert(ROLL);
  #endif 
  #ifdef OriginalIMU
    gyro.invert(PITCH);
    gyro.invert(ROLL);
  #endif
  #ifdef IXZ
    gyro.invert(YAW);
  #endif
  #ifdef ArduCopter
    gyro.invert(YAW);
    gyro.invert(PITCH);
    gyro.invert(ROLL);
  #endif
  #ifdef AeroQuad_Wii
    accel.invert(ROLL);
    gyro.invert(PITCH);
    gyro.invert(YAW);
  #endif
  #ifdef AeroQuadMega_Wii
    accel.invert(ROLL);
    gyro.invert(PITCH);
    gyro.invert(YAW);
  #endif
  #ifdef Multipilot
    accel.invert(PITCH);
    gyro.invert(ROLL);
  #endif
  
  // Flight angle estimiation
  flightAngle.initialize(); // defined in FlightAngle.h
    
  // Camera stabilization setup
  #ifdef Camera
    rollCamera.attach(ROLLCAMERAPIN);
    pitchCamera.attach(PITCHCAMERAPIN);
  #endif
  
  previousTime = millis();
  digitalWrite(LEDPIN, HIGH);
  safetyCheck = 0;
}

// ************************************************************
// ******************** Main AeroQuad Loop ********************
// ************************************************************
void loop () {
  // Measure loop rate
  currentTime = millis();
  deltaTime = currentTime - previousTime;
  G_Dt = deltaTime / 1000.0;
  previousTime = currentTime;
  #ifdef DEBUG
    if (testSignal == LOW) testSignal = HIGH;
    else testSignal = LOW;
    digitalWrite(LEDPIN, testSignal);
  #endif
  
  // Reads external pilot commands and performs functions based on stick configuration
  if ((currentTime > (receiverTime + RECEIVERLOOPTIME)) && (receiverLoop == ON)) {// 10Hz
    readPilotCommands(); // defined in FlightCommand.pde
    receiverTime = currentTime;
  }
  
  // Measures sensor data and calculates attitude
  if ((currentTime > (sensorTime + AILOOPTIME)) && (sensorLoop == ON)) { // 500Hz
    readSensors(); // defined in Sensors.pde
    sensorTime = currentTime;
  } 

  // Combines external pilot commands and measured sensor data to generate motor commands
  if ((currentTime > controlLoopTime + CONTROLLOOPTIME) && (controlLoop == ON)) { // 500Hz
    flightControl(); // defined in FlightControl.pde
    controlLoopTime = currentTime;
  } 
  
  // Listen for configuration commands and reports telemetry
  if ((currentTime > telemetryTime + TELEMETRYLOOPTIME) && (telemetryLoop == ON)) { // 10Hz    
    readSerialCommand(); // defined in SerialCom.pde
    sendSerialTelemetry(); // defined in SerialCom.pde
    telemetryTime = currentTime;
  }
  
#ifdef Camera // Experimental, not fully implemented yet
  // Command camera stabilization servos (requires #include <servo.h>)
  if ((currentTime > (cameraTime + CAMERALOOPTIME)) && (cameraLoop == ON)) { // 50Hz
    rollCamera.write((mCamera * flightAngle.get(ROLL)) + bCamera);
    pitchCamera.write((mCamera * flightAngle.get(PITCH)) + bCamera);
    cameraTime = currentTime;
  }
#endif
}
