

#define AeroQuad_Wii
  #include <AQWiiSensorAccessor.h>
  AQWiiSensorAccessor _wiiSensorAccessor;
  #include <WiiAccelerometer.h>
  WiiAccelerometer tempAccel(_wiiSensorAccessor);
  Accelerometer *_accel = &tempAccel;
  #include <WiiGyroscope.h>
  WiiGyroscope tempGyro(_wiiSensorAccessor);
  Gyroscope *_gyro = &tempGyro;
  #include <ReceiverFor328p.h>
  ReceiverFor328p tempReceiver;
  Receiver *_receiver = &tempReceiver;
  #include <PWMMotors.h>
  PWMMotors tempMotors;
  Motors *_motors = &tempMotors;
  #include "FlightAngle.h"
  FlightAngleDCM tempFlightAngle;
//  FlightAngle_CompFilter tempFlightAngle;  
  FlightAngleProcessor *_flightAngle = &tempFlightAngle;
  #ifdef CameraControl
    #include <AeroQuadCameraStabilizer.h>
    AeroQuadCameraStabilizer tempCamera;
    CameraStabilizer *_cameraStabilizer = &tempCamera;
  #endif
//#endif
