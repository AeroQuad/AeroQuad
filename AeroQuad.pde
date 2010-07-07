/*
  AeroQuad v2.0 - July 2010
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

// Hardware Configuration
//#define AeroQuad_v1         // Arduino 2009 with AeroQuad Shield v1.x
#define AeroQuad_v2         // Arduino 2009 with AeroQuad Shield v2.x
//#define AeroQuadMega_v1     // Arduino Mega with AeroQuad Shield v1.x (needs debug)
//#define APM                 // ArduPilot Mega (APM) with APM Sensor Board
//#define AeroQuad_Wii        // Arduino 2009 with Wii Sensors (needs debug)
//#define AeroQuadMega_Wii    // Arduino Mega with Wii Sensors (needs debug)

// 5DOF IMU Version
//#define OriginalIMU // Use this if you have the 5DOF IMU which uses the IDG300 or IDG500      

// Yaw Gyro Type
// Use only one of the following definitions
#define IXZ // IXZ-500 Flat Yaw Gyro
//#define IDG // IDG-300 or IDG-500 Dual Axis Gyro

// Define Flight Configuration
// Use only one of the following definitions
//#define plusConfig
#define XConfig

// Receiver Input Configuration
// If you are using the Arduino Mega with an AeroQuad Shield v1.x, the receiver pins must be configured differently due to bug in Arduino core.
// Put a jumper wire between the Shield and Mega as indicated below
// For Roll (Aileron) Channel, place jumper between AQ Shield pin 2 and Mega AI13
// For Pitch (Elevator) Channel, place jumper between AQ Shield pin 5 and Mega AI11
// For Yaw (Rudder) Channel, place jumper between AQ Shield pin 6 and Mega AI10
// For Throttle Channel, place jumper between AQ Shield pin 4 and Mega AI12
// For Mode (Gear) Channel, place jumper between AQ Shield pin 7 and Mega AI9
// For Aux Channel, place jumper between AQ Shield 8 and Mega AI8

// Camera Stabilization (experimental)
// Will move development to Arduino Mega (needs Servo support for additional pins)
//#define Camera

/***************************************************************************/

#include <EEPROM.h>
#include <Servo.h>
#include <Wire.h>
#include "AeroQuad.h"
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
  Receiver_AeroQuad_v1 receiver;
  Motors_PWM motors;
#endif

#ifdef AeroQuad_v2
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v2 gyro;
  Receiver_AeroQuad_v1 receiver;
  Motors_PWM motors;
#endif

#ifdef AeroQuadMega_v1 
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Receiver_AeroQuadMega_v1 receiver;
  Motors_PWM motors;
#endif

#ifdef APM
  Accel_APM accel;
  Gyro_APM gyro;
  //Altimeter_APM altimeter;
  //Compass_APM compass;
  Receiver_APM receiver;
  Motors_APM motors;
#endif

#ifdef AeroQuad_Wii
  Accel_Wii accel;
  Gyro_Wii gyro;
  Receiver_AeroQuad_v1 receiver;
  Motors_PWM motors;
#endif

#ifdef AeroQuadMega_Wii
  Accel_Wii accel;
  Gyro_Wii gyro;
  Receiver_AeroQuadMega_v1 receiver;
  Motors_PWM motors;
#endif

// Class definition for angle estimation found in FlightAngle.h
// Use only one of the following variable declarations
#include "FlightAngle.h"
FlightAngle_CompFilter angle[2]; // Use this for Complementary Filter
//FlightAngle_KalmanFilter angle[2];  Use this for Kalman Filter
//FlightAngle_FabQuad angle[2]; // developed by FabQuad (http://aeroquad.com/showthread.php?p=3995#post3995)

#include "DataStorage.h"

// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
void setup() {
  Serial.begin(BAUD);
  pinMode (LEDPIN, OUTPUT);
  
  // Read user values from EEPROM
  readEEPROM();
  
  // Configure motors
  motors.initialize();

  // Setup receiver pins for pin change interrupts
  if (receiverLoop == ON)
    receiver.initialize();
     
  // Heading hold
  // aref is read in from EEPROM and originates from Configurator
  headingScaleFactor = (aref / 1024.0) / gyro.getScaleFactor() * (PI/2.0);
  
  // Initialize sensors
  gyro.initialize(); // If sensors have a common initialization routine, insert it into the corresponding gyro subclass
  accel.initialize();
  // Calibrate sensors
  gyro.autoZero();  
  zeroIntegralError();
  levelAdjust[ROLL] = 0;
  levelAdjust[PITCH] = 0;
  
  // Angle estimation intialization
  for (axis = ROLL; axis < YAW; axis++)
    //angle[axis].initialize(accelADC[ZAXIS]);
    angle[axis].initialize(axis);
    
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
  
// *************************************************************
// ******************* Camera Stailization *********************
// *************************************************************
#ifdef Camera // Development moved to Arduino Mega
  if ((currentTime > (cameraTime + CAMERALOOPTIME)) && (cameraLoop == ON)) { // 50Hz
    rollCamera.write((mCamera * flightAngle[ROLL]) + bCamera);
    pitchCamera.write((mCamera * flightAngle[PITCH]) + bCamera);
    cameraTime = currentTime;
  }
#endif
////////////////////////
// End of camera loop //
////////////////////////

// **************************************************************
// ***************** Fast Transfer Of Sensor Data ***************
// **************************************************************
  if ((currentTime > (fastTelemetryTime + FASTTELEMETRYTIME)) && (fastTransfer == ON)) { // 200Hz means up to 100Hz signal can be detected by FFT
    printInt(21845); // Start word of 0x5555
    for (axis = ROLL; axis < LASTAXIS; axis++) printInt(gyro.getRaw(axis));
    for (axis = ROLL; axis < LASTAXIS; axis++) printInt(accel.getRaw(axis));
    printInt(32767); // Stop word of 0x7FFF
    fastTelemetryTime = currentTime;
  }
////////////////////////////////
// End of fast telemetry loop //
////////////////////////////////
}
