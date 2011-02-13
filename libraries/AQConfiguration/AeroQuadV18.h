

#define AeroQuad_v18
  #include <BMA180Accelerometer.h>
  BMA180Accelerometer tempAccel;
  Accelerometer *_accel = &tempAccel;
  #include <ITG3200Gyroscope.h>
  ITG3200Gyroscope tempGyro;
  Gyroscope *_gyro = &tempGyro;
  #include <ReceiverFor328p.h>
  ReceiverFor328p tempReceiver;
  Receiver *_receiver = &tempReceiver;
  #include <PWMTimedMotors.h>
  PWMTimedMotors tempMotors;
  Motors *_motors = &tempMotors;
  //Motors_AeroQuadI2C motors; // Use for I2C based ESC's
  #include "FlightAngleDCM.h"
  FlightAngleDCM tempFlightAngle;
//  #include "FlightAngleCompFilter.h"
//  FlightAngleCompFilter tempFlightAngle;
//  #include "FlightAngleKalmanFilter.h"
//  FlightAngleKalmanFilter tempFlightAngle;
  FlightAngleProcessor *_flightAngle = &tempFlightAngle;
  #ifdef HeadingMagHold
    #include <HMC5843Magnetometer.h>
    HMC5843Magnetometer tempCompass(_gyro);
    Compass *_compass = &tempCompass;
  #endif
  #ifdef AltitudeHold
    #include <BMP085BarometricSensor.h>
    BMP085BarometricSensor tempAltitude;
    AltitudeProvider *_altitudeProvider = &tempAltitude;
  #endif
  #ifdef BattMonitor
    #include <AeroQuadBatteryMonitor.h>
    AeroQuadBatteryMonitor tempBatteryMonitor;
    BatteryMonitor *_batteryMonitor = &tempBatteryMonitor;
  #endif
//  #ifdef CameraControl
//    #include "AeroQuadCameraStabilizer.h"
//    AeroQuadCameraStabilizer tempCamera;
//    CameraStabilizer *_cameraStabilizer = &tempCamera;
//  #endif
//#endif