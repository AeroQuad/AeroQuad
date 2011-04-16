/*
  AeroQuad v3.0 - June 2011
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

#include <stdlib.h>
#include <math.h>
#include "WProgram.h"
#include "pins_arduino.h"

// Flight Software Version
#define VERSION 2.1

#define DEG_2_RAD(d) (d * 0.01745329252)
#define RAD_2_DEG(r) (r * 57.2957795131)

#define G_2_MPS2(g) (g * 9.80665)
#define MPS2_2_G(m) (m * 0.10197162)

#define BAUD 111111
//#define BAUD 115200
#define ON 1
#define OFF 0

#include "Configuration.h"
#include <Wire.h>
#include <I2C.h>          // I2C Utility Functions
#include <Accel.h>        // Accelerometer Sensor
#include <Gyro.h>         // Gyro Sensor
//#include <Compass.h>      // Compass Sensor
//#include <Altitude.h>     // Altitude Sensor
//#include <Filter.h>       // Low Pass Filters
//#include <Kinematics.h>   // Kinematics
//#include <Motors.h>       // Motor Control
//#include <Control.h>      // Control Law (PID, Fuzzy Logic, etc)
//#include <Receiver.h>     // Receive External Commands

#if defined(APM)
  #include <APM_ADC.h>
#elif defined(WII)
  #include <Wii.h>
#endif

// Common declaration of all objects from parent class
// Forces all future objects to conform to parent class
Accel *accel = &tempAccel;
Gyro  *gyro  = &tempGyro;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef AEROQUAD_V18

  Receiver_AeroQuad         receiver;
  Motors_PWMtimer           motors;
  //Motors_AeroQuadI2C        motors;  // Use for I2C based ESC's
  Accel_AeroQuadMega_v2     accel;
  RateGyro_AeroQuadMega_v2  rateGyro;
  Kinematics_DCM            kinematics;

  #ifdef COMPASS_INSTALLED
    Magnetometer_HMC5843    compass;
  #endif
  
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef AEROQUAD_MEGA_V2

  Receiver_AeroQuadMega     receiver;
  Motors_PWMtimer           motors;
  //Motors_AeroQuadI2C        motors;  // Use for I2C based ESC's
  Accel_AeroQuadMega_v2     accel;
  RateGyro_AeroQuadMega_v2  rateGyro;
  Kinematics_DCM            kinematics;

  #ifdef COMPASS_INSTALLED
    Magnetometer_HMC5843    compass;
  #endif
  
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef APM

  Receiver_APM              receiver;
  Motors_APM                motors;
  //Motors_AeroQuadI2C        motors;  // Use for I2C based ESC's
  Accel_APM                 accel;
  Gyro_APM                  rateGyro;
  Kinematics_DCM            kinematics;

  #ifdef COMPASS_INSTALLED
    Magnetometer_HMC5843    compass;
  #endif

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef AEROQUAD_WII

  Receiver_AeroQuad         receiver;
  Motors_PWMtimer           motors;
  //Motors_AeroQuadI2C        motors;  // Use for I2C based ESC's
  Accel_Wii                 accel;
  Gyro_Wii                  rateGyro;
  Kinematics_DCM            kinematics;
  
  #ifdef COMPASS_INSTALLED
    Magnetometer_HMC5843    compass;
  #endif
  
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

// FUNCTION: return the number of bytes currently free in RAM      
extern int  __bss_end; // used by freemem 
extern int  *__brkval; // used by freemem
int freemem(){
    int free_memory;
    if((int)__brkval == 0)
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
        free_memory = ((int)&free_memory) - ((int)__brkval);
    return free_memory;
}

