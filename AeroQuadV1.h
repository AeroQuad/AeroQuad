


#define AeroQuad_v1
  #include <ADXL335Accelerometer.h>
  ADXL335Accelerometer tempAccel;
  Accelerometer *_accel = &tempAccel;
  #include <IDGIXZ500Gyroscope.h>
  IDGIXZ500Gyroscope tempGyro;
  Gyroscope *_gyro = &tempGyro;
  #include <ReceiverFor328p.h>
  ReceiverFor328p tempReceiver;
  Receiver *_receiver = &tempReceiver;
  #include <PWMMotors.h>
  PWMMotors tempMotors;
  Motors *_motors = &tempMotors;
  #include "FlightAngle.h"
  FlightAngleDCM tempFlightAngle;
  FlightAngleProcessor *_flightAngle = &tempFlightAngle;
//  #ifdef CameraControl
//    #include "AeroQuadCameraStabilizer.h"
//    AeroQuadCameraStabilizer tempCamera;
//    CameraStabilizer *_cameraStabilizer = &tempCamera;
//  #endif
//#endif
