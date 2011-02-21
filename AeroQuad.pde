/*
  AeroQuad v2.3 - February 2011
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
 ************************* Hardware Configuration ***************************
 ****************************************************************************/
// Select which hardware you wish to use with the AeroQuad Flight Software

//#define AeroQuad_v1         // Arduino 2009 with AeroQuad Shield v1.7 and below
//#define AeroQuad_v1_IDG     // Arduino 2009 with AeroQuad Shield v1.7 and below using IDG yaw gyro
#define AeroQuad_v18        // Arduino 2009 with AeroQuad Shield v1.8
//#define AeroQuad_Wii        // Arduino 2009 with Wii Sensors and AeroQuad Shield v1.x
//#define AeroQuadMega_v1     // Arduino Mega with AeroQuad Shield v1.7 and below
//#define AeroQuadMega_v2     // Arduino Mega with AeroQuad Shield v2.x
//#define AeroQuadMega_Wii    // Arduino Mega with Wii Sensors and AeroQuad Shield v2.x
//#define ArduCopter          // ArduPilot Mega (APM) with APM Sensor Board
//#define Multipilot          // Multipilot board with Lys344 and ADXL 610 Gyro (needs debug)
//#define MultipilotI2C       // Active Multipilot I2C and Mixertable (needs debug)
//#define AeroQuadMega_CHR6DM // Clean Arduino Mega with CHR6DM as IMU/heading ref.
//#define APM_OP_CHR6DM       // ArduPilot Mega with CHR6DM as IMU/heading ref., Oilpan for barometer (just uncomment AltitudeHold for baro), and voltage divider

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
//#define UseArduPirateSuperStable // Enable the imported stable mode imported from ArduPirate (experimental, use at your own risk)
//#define HeadingMagHold // Enables HMC5843 Magnetometer, gets automatically selected if CHR6DM is defined
//#define AltitudeHold // Enables BMP085 Barometer (experimental, use at your own risk)
#define BattMonitor //define your personal specs in BatteryMonitor.h! Full documentation with schematic there
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

/****************************************************************************
 ********************* End of User Definition Section ***********************
 ****************************************************************************/

#include <EEPROM.h>
#include <Wire.h>
#include "AeroQuad.h"
#include "I2C.h"
#include "PID.h"
#include "AQMath.h"
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
  #include "FlightAngle.h"
  FlightAngle_DCM tempFlightAngle;
  FlightAngle *_flightAngle = &tempFlightAngle;
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuad_v1_IDG
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Receiver_AeroQuad receiver;
  Motors_PWM motors;
  #include "FlightAngle.h"
  FlightAngle_DCM tempFlightAngle;
  FlightAngle *_flightAngle = &tempFlightAngle;
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuad_v18
  Accel_AeroQuadMega_v2 accel;
  //Gyro_AeroQuadMega_v2 gyro;
  #include "RateGyro.h"
  RateGyro_AeroQuadMega_v2 gyro;
  Receiver_AeroQuad receiver;
  Motors_PWMtimer motors;
  //Motors_AeroQuadI2C motors; // Use for I2C based ESC's
  //#include "FlightAngle.h"
  //FlightAngle_DCM tempFlightAngle;
  //FlightAngle *_flightAngle = &tempFlightAngle;
  #include "Kinematics.h"
  Kinematics_DCM kinematics;
  #ifdef HeadingMagHold
    #include "Compass.h"
    //Compass_AeroQuad_v2 compass;
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuadMega_v1
  // Special thanks to Wilafau for fixes for this setup
  // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11466&viewfull=1#post11466
  Receiver_AeroQuadMega receiver;
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Motors_PWM motors;
  #include "FlightAngle.h"
  FlightAngle_DCM tempFlightAngle;
  FlightAngle *_flightAngle = &tempFlightAngle;
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuadMega_v2
  Receiver_AeroQuadMega receiver;
  Motors_PWMtimer motors;
  //Motors_AeroQuadI2C motors; // Use for I2C based ESC's
  Accel_AeroQuadMega_v2 accel;
  //Gyro_AeroQuadMega_v2 gyro;
  #include "RateGyro.h"
  RateGyro_AeroQuadMega_v2 gyro;
  //#include "FlightAngle.h"
  //FlightAngle_DCM tempFlightAngle;
  //FlightAngle *_flightAngle = &tempFlightAngle;
  #include "Kinematics.h"
  Kinematics_DCM kinematics;
  #ifdef HeadingMagHold
    #include "Compass.h"
    //Compass_AeroQuad_v2 compass;
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef ArduCopter
  //Gyro_ArduCopter gyro;
  #include "RateGyro.h"
  Gyro_APM rateGyro;
  //Accel_ArduCopter accel;
  Accel_APM accel;
  Receiver_ArduCopter receiver;
  Motors_ArduCopter motors;
  //#include "FlightAngle.h"
  //FlightAngle_DCM tempFlightAngle;
  //FlightAngle *_flightAngle = &tempFlightAngle;
  #include "Kinematics.h"
  Kinematics_DCM kinematics;
  #ifdef HeadingMagHold
    #include "Compass.h"
    Compass_AeroQuad_v2 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_APM batteryMonitor;
  #endif
#endif

#ifdef AeroQuad_Wii
  Accel_Wii accel;
  Gyro_Wii gyro;
  Receiver_AeroQuad receiver;
  Motors_PWM motors;
  //#include "FlightAngle.h"
  //FlightAngle_DCM tempFlightAngle;
  //FlightAngle *_flightAngle = &tempFlightAngle;
  #include "Kinematics.h"
  Kinematics_DCM kinematics; 
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuadMega_Wii
  Accel_Wii accel;
  Gyro_Wii gyro;
  Receiver_AeroQuadMega receiver;
  Motors_PWM motors;
  //#include "FlightAngle.h"
  //FlightAngle_DCM tempFlightAngle;
  //FlightAngle *_flightAngle = &tempFlightAngle;
  #include "Kinematics.h"
  Kinematics_DCM kinematics; 
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuadMega_CHR6DM
  Accel_CHR6DM accel;
  Gyro_CHR6DM gyro;
  Receiver_AeroQuadMega receiver;
  Motors_PWM motors;
  #include "FlightAngle.h"
  FlightAngle_CHR6DM tempFlightAngle;
  FlightAngle *_flightAngle = &tempFlightAngle;
  #include "Compass.h"
  Compass_CHR6DM compass;
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_APM batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef APM_OP_CHR6DM
  Accel_CHR6DM accel;
  Gyro_CHR6DM gyro;
  Receiver_ArduCopter receiver;
  Motors_ArduCopter motors;
  #include "FlightAngle.h"
  FlightAngle_CHR6DM tempFlightAngle;
  FlightAngle *_flightAngle = &tempFlightAngle;
  #include "Compass.h"
  Compass_CHR6DM compass;
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_APM batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef Multipilot
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Receiver_Multipilot receiver;
  Motors_PWM motors;
  //#define PRINT_MIXERTABLE
  //#define TELEMETRY_DEBUG
  #include "FlightAngle.h"
  FlightAngle_DCM tempFlightAngle;
  FlightAngle *_flightAngle = &tempFlightAngle;
#endif

#ifdef MultipilotI2C  
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Receiver_Multipilot receiver;
  Motors_I2C motors;
  //#define PRINT_MIXERTABLE
  //#define TELEMETRY_DEBUG
  #include "FlightAngle.h"
  FlightAngle_DCM tempFlightAngle;
  FlightAngle *_flightAngle = &tempFlightAngle;
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
void setup() {
  Serial.begin(BAUD);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);

  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    Serial1.begin(BAUD);
    PORTD = B00000100;
  #endif
  #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
    pinMode(LED2PIN, OUTPUT);
    digitalWrite(LED2PIN, LOW);
    pinMode(LED3PIN, OUTPUT);
    digitalWrite(LED3PIN, LOW);
  #endif
  #ifdef AeroQuadMega_v2
    // pins set to INPUT for camera stabilization so won't interfere with new camera class
    pinMode(33, INPUT);
    pinMode(34, INPUT);
    pinMode(35, INPUT);
  #endif
  #if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
    pinMode(LED_Red, OUTPUT);
    pinMode(LED_Yellow, OUTPUT);
    pinMode(LED_Green, OUTPUT);
  #endif
  
  #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2) || defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM) || defined(ArduCopter)
    Wire.begin();
  #endif
  #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
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
  gyro.initialize(); // defined in Gyro.h
  accel.initialize(); // defined in Accel.h
  
  // Calibrate sensors
  gyro.autoZero(); // defined in Gyro.h
  zeroIntegralError();
  levelAdjust[ROLL] = 0;
  levelAdjust[PITCH] = 0;
  
  // Setup correct sensor orientation
  #ifdef AeroQuad_v1
    gyro.invert(YAW);
  #endif
  #if defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii)
    accel.invert(PITCH);
    accel.invert(ZAXIS);
  #endif
  #ifdef Multipilot
    accel.invert(PITCH);
    gyro.invert(ROLL);
  #endif
  
  // Flight angle estimiation
  //_flightAngle->initialize(); // defined in FlightAngle.h

  // Optional Sensors
  #ifdef HeadingMagHold
    compass.initialize();
    kinematics.initialize(compass.getHdgXY(XAXIS), compass.getHdgXY(YAXIS));
    //setHeading = compass.getHeading();
  #else
    kinematics.initialize(1.0, 0.0);  // with no compass, DCM matrix initalizes to a heading of 0 degrees
  #endif
  #ifdef AltitudeHold
    altitude.initialize();
  #endif
  
  // Battery Monitor
  #ifdef BattMonitor
    batteryMonitor.initialize();
  #endif
  
  // Camera stabilization setup
  #ifdef CameraControl
    camera.initialize();
    camera.setmCameraRoll(11.11); // Need to figure out nice way to reverse servos
    camera.setCenterRoll(1500); // Need to figure out nice way to set center position
    camera.setmCameraPitch(11.11);
    camera.setCenterPitch(1300);
  #endif
  
  previousTime = micros();
  digitalWrite(LEDPIN, HIGH);
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
    //flightControl(); // defined in FlightControl.pde
    processFlightControl();
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
      camera.setPitch(_flightAngle->getData(PITCH));
      camera.setRoll(_flightAngle->getData(ROLL));
      camera.setYaw(_flightAngle->getData(YAW));
      camera.move();
      cameraTime = currentTime + CAMERALOOPTIME;
    }
  #endif
}
