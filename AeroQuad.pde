/*
  AeroQuad v2.2 - Feburary 2011
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

/****************************************************************************
   Before flight, select the different user options for your AeroQuad below
   If you need additional assitance go to http://AeroQuad.com/forum
*****************************************************************************/

/****************************************************************************
 *********************** Define Flight Configuration ************************
 ****************************************************************************/
// Use only one of the following definitions
#define XConfig
//#define plusConfig
//#define HEXACOAXIAL
//#define HEXARADIAL

// *******************************************************************************************************************************
// Optional Sensors
// Warning:  If you enable HeadingHold or AltitudeHold and do not have the correct sensors connected, the flight software may hang
// *******************************************************************************************************************************
#define UseArduPirateSuperStable // Enable the imported stable mode imported from ArduPirate (experimental, use at your own risk)
//#define HeadingMagHold // Enables HMC5843 Magnetometer, gets automatically selected if CHR6DM is defined
#define AltitudeHold // Enables BMP085 Barometer (experimental, use at your own risk)
//#define BattMonitor //define your personal specs in BatteryMonitor.h! Full documentation with schematic there
//#define WirelessTelemetry  // Enables Wireless telemetry on Serial3  // Wireless telemetry enable

// *******************************************************************************************************************************
// Camera Stabilization
// Servo output goes to D11(pitch), D12(roll), D13(yaw) on AeroQuad v1.8 shield
// If using v2.0 Shield place jumper between:
// D12 to D33 for roll, connect servo to SERVO1
// D11 to D34 for pitch, connect servo to SERVO2
// D13 to D35 for yaw, connect servo to SERVO3
// Please note that you will need to have battery connected to power on servos with v2.0 shield
// *******************************************************************************************************************************
//#define CameraControl

#include <EEPROM.h>
#include <Wire.h>
#include <I2C.h>
#include <AQMath.h>
#include <Axis.h>
#include <MotorsGlobalNames.h>
#include <ReceiverGlobalVariables.h>
#include "AeroQuad.h"
#include "PID.h"

/****************************************************************************
************************* Hardware Configuration ***************************
****************************************************************************/
// Select which hardware you wish to use with the AeroQuad Flight Software

//#define AeroQuad_v1         // Arduino 2009 with AeroQuad Shield v1.7 and below
//#define AeroQuad_v1_IDG     // Arduino 2009 with AeroQuad Shield v1.7 and below using IDG yaw gyro
//#define AeroQuad_v18        // Arduino 2009 with AeroQuad Shield v1.8
//#define AeroQuad_Wii        // Arduino 2009 with Wii Sensors and AeroQuad Shield v1.x
//#define AeroQuadMega_v1     // Arduino Mega with AeroQuad Shield v1.7 and below
//#define AeroQuadMega_v2     // Arduino Mega with AeroQuad Shield v2.x
//#define AeroQuadMega_Wii    // Arduino Mega with Wii Sensors and AeroQuad Shield v2.x
#define ArduCopter          // ArduPilot Mega (APM) with APM Sensor Board
//#define AeroQuadMega_CHR6DM // Clean Arduino Mega with CHR6DM as IMU/heading ref.
//#define APM_OP_CHR6DM       // ArduPilot Mega with CHR6DM as IMU/heading ref., 
                              // Oilpan for barometer (just uncomment AltitudeHold for baro), and voltage divider

//#define CustomConfig       // Make your own custom config in this file  SAMPLE PURPOSE ONLY                         
//#define DefaultConfig      // simplified v1.8 DO NOT USE, fix a compilation problem for other platform, probably arduino compiler problem! @see Kenny9999

/****************************************************************************
 ********************* End of User Definition Section ***********************
 ****************************************************************************/
 
//******************************************************
// Camera stabilization seem to not work on 328p cause
// of specific register used @see Kenny
// need to be fixed, CupOfTea, help please
//****************************************************** 



#ifdef AeroQuad_v1   
  #include <AeroQuadV1.h>
#elif defined(AeroQuad_v1_IDG) 
  #include <AeroQuadV1_IDG.h>
#elif defined(AeroQuad_v18)
  #include <AeroQuadV18.h>
#elif defined(AeroQuad_Wii)
  #include <AeroQuadWii.h>
#elif defined(AeroQuadMega_v1)
  #include <AeroQuadMegaV1.h>
#elif defined(AeroQuadMega_v2)
  #include <AeroQuadCameraStabilizer.h>
  #include <AeroQuadMegaV2.h>
#elif defined(AeroQuadMega_Wii)
  #include <AQWiiSensorAccessor.h>
  #include <AeroQuadMegaWii.h>
#elif defined(ArduCopter)
  #include <AQAPMADCSensorsAccessor.h>
  #include "Arducopter.h"
#elif defined(AeroQuadMega_CHR6DM)
  #include <CHR6DMSensorsAccessor.h>
  #include <AeroQuadMegaCHR6DM.h>
#elif defined(APM_OP_CHR6DM)  
  #include <CHR6DMSensorsAccessor.h>
  #include <APMOPCHR6DM.h>
#elif defined(CustomConfig)
  #include "CustomConfig.h"
#endif




#ifdef DefaultConfig  
  #include <BMA180Accelerometer.h>
  BMA180Accelerometer tempAccel;
  Accelerometer *accel = &tempAccel;
  #include <ITG3200Gyroscope.h>
  ITG3200Gyroscope tempGyro;
  Gyroscope *gyro = &tempGyro;
  #include <ReceiverFor328p.h>
  ReceiverFor328p tempReceiver;
  Receiver *receiver = &tempReceiver;
  #include <PWMTimedMotors.h>
  PWMTimedMotors tempMotors;
  Motors *motors = &tempMotors;
  //Motors_AeroQuadI2C motors; // Use for I2C based ESC's
  //#include "FlightAngleDCM.h"
  //FlightAngleDCM tempFlightAngle;
  //#include "FlightAngleCompFilter.h"
  //FlightAngleCompFilter tempFlightAngle;
  #include "FlightAngleKalmanFilter.h"
  FlightAngleKalmanFilter tempFlightAngle;
  FlightAngleProcessor *flightAngle = &tempFlightAngle;
  #ifdef HeadingMagHold
  #include <HMC5843Magnetometer.h>
    HMC5843Magnetometer tempCompass;
    Compass *_compass = &tempCompass;
  #endif
  #ifdef AltitudeHold
  #include <BMP085BarometricSensor.h>
    BMP085BarometricSensor tempAltitude;
    AltitudeProvider *altitudeProvider = &tempAltitude;
  #endif
  #ifdef BattMonitor
  #include <AeroQuadBatteryMonitor.h>
    AeroQuadBatteryMonitor tempBatteryMonitor;
    BatteryMonitor *batteryMonitor = &tempBatteryMonitor;
  #endif
#endif



#ifdef XConfig
  void (*processFlightControl)() = &processFlightControlXMode;
#else
  void (*processFlightControl)() = &processFlightControlPlusMode;
#endif

#ifdef UseArduPirateSuperStable
  void (*processStableMode)() = &processArdupirateSuperStableMode;
#else
  void (*processStableMode)() = &processAeroQuadStableMode;
#endif

// Include this last as it contains objects from above declarations
#include "DataStorage.h"

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
