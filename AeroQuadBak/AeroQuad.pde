// Mega 2560, Compass, PWM (timer) controllers
// 01/22/2011 - Baseline 2.1
//              31920 bytes
// 01/22/2011 - Move Matrix and Vector operations from FlightAngle.h to Matrix.pde and Vector.pde, create Matrix and Vector function declarations in AeroQuad.h
// 01/22/2011 - Move accel bias correction outside of loop in Accel.h, correct for proper rotation order, algebraic sign, replace ROLL, PITCH with XAXIS, YAXIS
// 01/22/2011 - Rework Accel initialization
// 01/24/2011 - Rename FlightAngle.h to AttitudeEstimation.h
// 01/24/2011 - Move gyro bias correction outside of loop in Gyro.h, correct for proper algebraic sign, rework gyro initialization
// 01/24/2011 - Rescale and reorder DCM gyro inputs, reorder DCM Euler Angle outputs, single set of Kp and Ki gains, delete unnecessary initializations
// 01/25/2011 - Restore accel weight function in DCM drift correction
// 01/25/2011 - Initialize DCM matrix with compass heading, correct yaw gyro drift with compass heading X and Y components
// 02/03/2011 - Change AttitudeEstimation to Kinematics, add conversions from body accels to earth accels
// 02/03/2011 - Made switch over to radians, opportunity exists to reduce code size when flight control PID gains are rescaled and configurator is updated
//              to read in radians and display them as degrees.  One step at a time!
// 02/04/2011 - Rework receiver class initialization
//              29696 bytes

// Mega 2560, Compass, I2C controllers
// 01/22/2011 - Baseline 2.1
//              32502 bytes
// 01/22/2011 - Rework I2C motor class to remove floating point operations
//              32002 bytes

/*
  AeroQuad v2.1 - January 2011
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

#include <EEPROM.h>
#include <Wire.h>
#include "AeroQuad.h"
#include "ApmAdc.h"
#include "Accel.h"
#include "Compass.h"
#include "Filter.h"
#include "I2C.h"
#include "Kinematics.h"
#include "Motors.h"
#include "PID.h"
#include "RateGyro.h"
#include "Receiver.h"

#include <EEPROM.h>
#include <Wire.h>
#include <I2C.h>
#include <AQMath.h>
#include <Axis.h>
#include <MotorsGlobalNames.h>
#include <ReceiverGlobalVariables.h>
#include "AeroQuad.h"
#include "PID.h"
// Include this last as it contains objects from above declarations
#include "DataStorage.h"

// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
void setup() {
  Serial.begin(BAUD);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  #if defined(AEROQUAD_V18) || defined(AEROQUAD_MEGA_V2)
    pinMode(LED2_PIN, OUTPUT);
    digitalWrite(LED2_PIN, LOW);
    pinMode(LED3_PIN, OUTPUT);
    digitalWrite(LED3_PIN, LOW);
  #endif

  #if defined(APM) 
    pinMode(LED_Red, OUTPUT);
    pinMode(LED_Yellow, OUTPUT);
    pinMode(LED_Green, OUTPUT);
  #endif
  
  #if defined(AEROQUAD_V18) || defined(AEROQUAD_MEGA_V2) || defined(AEROQUAD_WII) || defined(APM)
    Wire.begin();
  #endif
  
  #if defined(AEROQUAD_V18) || defined(AEROQUAD_MEGA_V2) || defined(APM)
    // Recommendation from Mgros to increase I2C speed to 400kHz
    // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11262&viewfull=1#post11262
    TWBR = 12;
  #endif

  // Read user values from EEPROM
  readEEPROM(); // defined in DataStorage.h
  
  // Configure motors
  motors.initialize(); // defined in Motors.h

  // Setup receiver pins for pin change interrupts
  if (receiverLoop == ON) receiver.initialize(); // defined in Received.h
       
  // Initialize sensors
  // If sensors have a common initialization routine
  // insert it into the gyro class because it executes first
  rateGyro.initialize();
  accel.initialize();
  
  // Calibrate sensors
  rateGyro.autoZero(); // defined in Gyro.h
  zeroIntegralError();
  levelAdjust[ROLL] = 0;
  levelAdjust[PITCH] = 0;
  
  // Optional Sensors
  #ifdef COMPASS_INSTALLED
    compass.initialize();
    kinematics.initialize(compass.getHdgXY(XAXIS), compass.getHdgXY(YAXIS));
  #else
    kinematics.initialize(1.0, 0.0);  // with no compass, DCM matrix initalizes to a heading of 0 degrees
  #endif
  
  previousTime = micros();
  digitalWrite(LED_PIN, HIGH);
  safetyCheck = 0;
}

// ************************************************************
// ******************** Main AeroQuad Loop ********************
// ************************************************************
void loop () {
  // Measure loop rate
  currentTime = micros();
  deltaTime = currentTime - previousTime;
  G_Dt = deltaTime / 1000000.0;
  previousTime = currentTime;
  #ifdef DEBUG
    if (testSignal == LOW) testSignal = HIGH;
    else testSignal = LOW;
    digitalWrite(LEDPIN, testSignal);
  #endif
  
  // Measures sensor data and calculates attitude
  if (sensorLoop == ON) {
    readSensors(); // defined in Sensors.pde
  } 

  // Combines external pilot commands and measured sensor data to generate motor commands
  if (controlLoop == ON) {
    flightControl(); // defined in FlightControl.pde
  } 
  
  // Reads external pilot commands and performs functions based on stick configuration
  if ((receiverLoop == ON) && (currentTime > receiverTime)) {// 50Hz
    readPilotCommands(); // defined in FlightCommand.pde
    receiverTime = currentTime + RECEIVERLOOPTIME;
  }
  
  // Listen for configuration commands and reports telemetry
  if ((telemetryLoop == ON) && (currentTime > telemetryTime)) { // 20Hz
    readSerialCommand(); // defined in SerialCom.pde
    sendSerialTelemetry(); // defined in SerialCom.pde
    telemetryTime = currentTime + TELEMETRYLOOPTIME;
  }

  #ifdef CameraControl // Experimental, not fully implemented yet
    if ((cameraLoop == ON) && (currentTime > cameraTime)) { // 50Hz
      camera.setPitch(flightAngle.getData(PITCH));
      camera.setRoll(flightAngle.getData(ROLL));
      camera.setYaw(flightAngle.getData(YAW));
      camera.move();
      cameraTime = currentTime + CAMERALOOPTIME;
    }
  #endif
}

// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
void setup() 
{
  Serial.begin(BAUD);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  
  // Init platform specific
  initPlatform();

  // Read user values from EEPROM
  readEEPROM(); // defined in DataStorage.h
  
  // Configure motors
  motors->initialize(); // defined in Motors.h

  // Setup receiver pins for pin change interrupts
  if (receiverLoop == ON) 
  {
    receiver->initialize(); // defined in Received.h
    initTransmitterFromEEPROM();
  }
       
  // Initialize sensors
  // If sensors have a common initialization routine
  // insert it into the gyro class because it executes first
  
  gyro->initialize(); // defined in Gyro.h
  accel->initialize(); // defined in Accel.h
  initSensorsFromEEPROM();
  
  // Calibrate sensors
  zeroIntegralError();
  levelAdjust[ROLL] = 0;
  levelAdjust[PITCH] = 0;
  
  // Setup correct sensor orientation
//  #ifdef Multipilot
//    accel->invert(PITCH);
//    gyro->invert(ROLL);
//  #endif
  
  // Flight angle estimiation
  flightAngle->setGyroscope(gyro);
  flightAngle->setAccelerometer(accel);
  flightAngle->initialize(); // defined in FlightAngle.h


  // Sub Process initialization
  // Optional Sensors
  #ifdef HeadingMagHold
    compass->initialize();
    setHeading = compass->getHeading();
  #endif
  #ifdef AltitudeHold
    altitudeProvider->initialize();
  #endif
  #ifdef BattMonitor
    batteryMonitor->initialize();
  #endif
  // Camera stabilization setup
  #ifdef CameraControl
    cameraStabilizer->initialize();
    cameraStabilizer->setmCameraRoll(11.11); // Need to figure out nice way to reverse servos
    cameraStabilizer->setCenterRoll(1500); // Need to figure out nice way to set center position
    cameraStabilizer->setmCameraPitch(11.11);
    cameraStabilizer->setCenterPitch(1300);
  #endif
  
  previousTime = micros();
  digitalWrite(LEDPIN, HIGH);
  safetyCheck = 0;
}

// ************************************************************
// ******************** Main AeroQuad Loop ********************
// ************************************************************
void loop () 
{
  // Measure loop rate
  currentTime = micros();
  deltaTime = currentTime - previousTime;
  G_Dt = deltaTime / 1000000.0;
  previousTime = currentTime;
  #ifdef DEBUG
    if (testSignal == LOW) testSignal = HIGH;
    else testSignal = LOW;
    digitalWrite(LEDPIN, testSignal);
  #endif
  
  // Measures sensor data and calculates attitude
  if (sensorLoop == ON) 
  {
    readSensors(); // defined in Sensors.pde
  } 

  // Combines external pilot commands and measured sensor data to generate motor commands
  if (controlLoop == ON) 
  {
    processFlightControl();
  } 
  
  // Reads external pilot commands and performs functions based on stick configuration
  if ((receiverLoop == ON) && (currentTime > receiverTime)) // 50Hz
  {
    readPilotCommands(); // defined in FlightCommand.pde
    receiverTime = currentTime + RECEIVERLOOPTIME;
  }
  
  // Listen for configuration commands and reports telemetry
  if ((telemetryLoop == ON) && (currentTime > telemetryTime)) // 20Hz 
  { 
    readSerialCommand(); // defined in SerialCom.pde
    sendSerialTelemetry(); // defined in SerialCom.pde
    telemetryTime = currentTime + TELEMETRYLOOPTIME;
  }

  #ifdef CameraControl // Experimental, not fully implemented yet
    if ((cameraLoop == ON) && (currentTime > cameraTime)) // 50Hz
    { 
      cameraStabilizer->setPitch(flightAngle->getData(PITCH));
      cameraStabilizer->setRoll(flightAngle->getData(ROLL));
      cameraStabilizer->setYaw(flightAngle->getData(YAW));
      cameraStabilizer->move();
      cameraTime = currentTime + CAMERALOOPTIME;
    }
  #endif
}
