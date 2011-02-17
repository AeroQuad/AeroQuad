// AeroQuad Configurator will auto generate this config for you
// You can also modify this by hand to customize to your needs

///////////////////////////////////////////////////////////
/// Hardware Platform
///////////////////////////////////////////////////////////
//#define MINI
#define UNO
//#define MEGA
//#define APM
//#define WII

///////////////////////////////////////////////////////////
/// Accelerometers 
///////////////////////////////////////////////////////////
#include <Accel_Null.h>
Accel_Null tempAccel;

//#include <Accel_BMA180.h>
//Accel_BMA180 tempAccel;

//#include <Accel_APM.h>
//Accel_APM tempAccel;

//#include <Accel_Wii.h>
//Accel_Wii tempAccel;

//#include <Accel_CHR6DM.h>
//Accel_CHR6DM tempAccel;

///////////////////////////////////////////////////////////
/// Gyros 
///////////////////////////////////////////////////////////

#include <Gyro_Null.h>
Gyro_Null tempGyro;

//#include<Gyro_ITG3200.h>
//Gyro_ITG3200 tempGyro;

//#include <Gyro_Wii.h>
//Gyro_Wii tempGyro;

//#include <Gyro_APM.h>
//Gyro_APM tempGyro;

//#include<Gyro_CHR6DM.h>
//Gyro_CHR6DM tempGyro;

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
#define HeadingMagHold // Enables HMC5843 Magnetometer, gets automatically selected if CHR6DM is defined
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

/****************************************************************************
   Before flight, select the different user options for your AeroQuad below
   If you need additional assitance go to http://AeroQuad.com/forum
*****************************************************************************/

/****************************************************************************
 ************************* Hardware Configuration ***************************
 ****************************************************************************/
// Select which hardware you wish to use with the AeroQuad Flight Software

//#define AEROQUAD_V18        // Arduino 2009/Uno with AeroQuad Shield v1.8
//#define AEROQUAD_WII        // Arduino 2009/Uno with Wii Sensors and AeroQuad Shield v1.x
#define AEROQUAD_MEGA_V2    // Arduino Mega 1280/2650 with AeroQuad Shield v2.x
//#define APM                 // ArduPilot Mega (APM) with APM Sensor Board

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
#define COMPASS_INSTALLED               //  Uncomment this line if you have an HMC5843 compass installed
//#define WIRELESS_TELEMETRY_INSTALLED    //  Enables Wireless telemetry on Serial3  // jihlein: Wireless telemetry enable

/****************************************************************************
 ********************* End of User Definition Section ***********************
 ****************************************************************************/

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
/******************************************************
*************** SAMPLE OF CUSTOM CONFIG ***************
******************* DON'T USE *************************
******************************************************/
#ifndef _AQ_CURSOM_CONFIG_H_
#define _AQ_CURSOM_CONFIG_H_

#include <ADXL335Accelerometer.h>
ADXL335Accelerometer tempAccel;
Accelerometer *accel = &tempAccel;
#include <ITG3200Gyroscope.h>
ITG3200Gyroscope tempGyro;
Gyroscope *gyro = &tempGyro;
#include <ReceiverFor328p.h>
ReceiverFor328p tempReceiver;
Receiver *receiver = &tempReceiver;
#include <I2CMotors.h>
I2CMotors tempMotors;
Motors *motors = &tempMotors;
//Motors_AeroQuadI2C motors; // Use for I2C based ESC's
#include "FlightAngleDCM.h"
FlightAngleDCM tempFlightAngle;
//  #include "FlightAngleCompFilter.h"
//  FlightAngleCompFilter tempFlightAngle;
//  #include "FlightAngleKalmanFilter.h"
//  FlightAngleKalmanFilter tempFlightAngle;
FlightAngleProcessor *flightAngle = &tempFlightAngle;
#ifdef HeadingMagHold
  #include <HMC5843Magnetometer.h>
  HMC5843Magnetometer tempCompass(_gyro);
  Compass *compass = &tempCompass;
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

void initPlatform()
{
}


#endif
