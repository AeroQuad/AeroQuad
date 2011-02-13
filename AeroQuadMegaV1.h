

#define AeroQuadMega_v1
  // Special thanks to Wilafau for fixes for this setup
  // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11466&viewfull=1#post11466
  #include <ReceiverForMega.h>
  ReceiverForMega tempReceiver;
  Receiver *_receiver = &tempReceiver;
  #include <ADXL335Accelerometer.h>
  ADXL335Accelerometer tempAccel;
  Accelerometer *_accel = &tempAccel;
  #include <IDGIXZ500Gyroscope.h>
  IDGIXZ500Gyroscope tempGyro;
  Gyroscope *_gyro = &tempGyro;
  #include <PWMMotors.h>
  PWMMotors tempMotors;
  Motors *_motors = &tempMotors;
  #include "FlightAngle.h"
  FlightAngleDCM tempFlightAngle;
  FlightAngleProcessor *_flightAngle = &tempFlightAngle;
  #ifdef CameraControl
    #include <AeroQuadCameraStabilizer.h>
    AeroQuadCameraStabilizer tempCamera;
    CameraStabilizer *_cameraStabilizer = &tempCamera;
  #endif
//#endif
