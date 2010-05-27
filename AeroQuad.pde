/*
  AeroQuad v1.8 - May 2010
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

#include <EEPROM.h>
//#include <NewSoftSerial.h>
//#include <TinyGPS.h>
//#include <Servo.h>
#include "GPS.h"
#include "AeroQuad.h"
#include "Filter.h"
#include "Eeprom.h"
#include "Sensors.h"
#include "PID.h"

/**************************************************************************** 
   Before flight, select the different user options for your AeroQuad below
   Also, consult the ReadMe.mht file for additional details
   If you need additional assitance go to http://forum.AeroQuad.info
*****************************************************************************/

// Define Flight Configuration
// Use only one of the following definitions
//#define plusConfig
#define XConfig

// 5DOF IMU Version
//#define OriginalIMU // Use this if you have the 5DOF IMU which uses the IDG300 or IDG500      

// Yaw Gyro Type
// Use only one of the following definitions
#define IXZ // IXZ-500 Flat Yaw Gyro
//#define IDG // IDG-300 or IDG-500 Dual Axis Gyro

// Arduino Mega with AeroQuad Shield v1.x
// If you are using the Arduino Mega with an AeroQuad Shield v1.x, the receiver pins must be configured differently due to bug in Arduino core.
// Put a jumper wire between the Shield and Mega as indicated below
// For Roll (Aileron) Channel, place jumper between AQ Shield pin 2 and Mega AI13
// For Pitch (Elevator) Channel, place jumper between AQ Shield pin 5 and Mega AI11
// For Yaw (Rudder) Channel, place jumper between AQ Shield pin 6 and Mega AI10
// For Throttle Channel, place jumper between AQ Shield pin 4 and Mega AI12
// For Mode (Gear) Channel, place jumper between AQ Shield pin 7 and Mega AI9
// For Aux Channel, place jumper between AQ Shield 8 and Mega AI8
//#define Mega_AQ1x
#define Duemilanove_AQ1x
//#define AeroQuadAPM

// Heading Hold (experimental)
// Currently uses yaw gyro which drifts over time, for Mega development will use magnetometer
//#define HeadingHold

// Camera Stabilization (experimental)
// Will move development to Arduino Mega (needs Servo support for additional pins)
//#define Camera

// GPS
//#define GPS

// Class definition for angle estimation found in FlightAngle.h
// Use only one of the following variable declarations
#include "FlightAngle.h"
FlightAngle_CompFilter angle[2]; // Use this for Complementary Filter
//FlightAngle_KalmanFilter angle[2];  Use this for Kalman Filter
//FlightAngle_FabQuad angle[2]; // developed by FabQuad (http://aeroquad.com/showthread.php?p=3995#post3995)

// Class definition for motor control found in Motors.h
// Use only one of the following variable declarations
#include "Motors.h"
Motors_PWM motors; // Use this for PWM ESC's
//Motors_APM motors; // Use this for AMP compatability (connect OUT0-3)
//Motors_I2C motors; // Future capability under construction

#include "Receiver.h" // This needs to be here to correctly define Mega pins for #define Mega_AQ1x

// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
void setup() {
  Serial.begin(BAUD);
  analogReference(EXTERNAL);
  pinMode (LEDPIN, OUTPUT);
  // Configure gyro auto zero pins
  pinMode (AZPIN, OUTPUT);
  digitalWrite(AZPIN, LOW);
  delay(1);
  
  // Configure motors
  motors.initialize();

  // Read user values from EEPROM
  readEEPROM();
  
  // Setup receiver pins for pin change interrupts
  if (receiverLoop == ON)
     configureReceiver();
  
  //  Auto Zero Gyros
  autoZeroGyros();
  
  // Heading hold
  // aref is read in from EEPROM and originates from Configurator
  headingScaleFactor = (aref / 1024.0) / gyroScaleFactor * (PI/2.0);
  
  // Calibrate sensors
  zeroGyros();
  zeroIntegralError();
  levelAdjust[ROLL] = 0;
  levelAdjust[PITCH] = 0;
  //accelADC[ZAXIS] = analogRead(accelChannel[ZAXIS]) - accelZero[ZAXIS];
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

  // GPS (Experimental)
  // To make this work remember to disable LED pin 13
  // Also disable PCINT0_vect
  //nss.begin(9600);
  //newdata = false;
  //start = millis();
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
  
// ************************************************************************
// ****************** Transmitter/Receiver Command Loop *******************
// ************************************************************************
  if ((currentTime > (receiverTime + RECEIVERLOOPTIME)) && (receiverLoop == ON)) { // 10Hz
    // Buffer receiver values read from pin change interrupt handler
    for (channel = ROLL; channel < LASTCHANNEL; channel++)
      receiverData[channel] = (mTransmitter[channel] * readReceiver(receiverPin[channel])) + bTransmitter[channel];
    // Smooth the flight control transmitter inputs (roll, pitch, yaw, throttle)
    for (channel = ROLL; channel < LASTCHANNEL; channel++)
      transmitterCommandSmooth[channel] = smooth(receiverData[channel], transmitterCommandSmooth[channel], smoothTransmitter[channel]);
    // Reduce transmitter commands using xmitFactor and center around 1500
    for (channel = ROLL; channel < LASTAXIS; channel++)
      transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
    // No xmitFactor reduction applied for throttle, mode and AUX
    for (channel = THROTTLE; channel < LASTCHANNEL; channel++)
      transmitterCommand[channel] = transmitterCommandSmooth[channel];

    // Read quad configuration commands from transmitter when throttle down
    if (receiverData[THROTTLE] < MINCHECK) {
      zeroIntegralError();
      // Disarm motors (left stick lower left corner)
      if (receiverData[YAW] < MINCHECK && armed == 1) {
        armed = 0;
        motors.commandAllMotors(MINCOMMAND);
      }    
      // Zero sensors (left stick lower left, right stick lower right corner)
      if ((receiverData[YAW] < MINCHECK) && (receiverData[ROLL] > MAXCHECK) && (receiverData[PITCH] < MINCHECK)) {
        autoZeroGyros();
        zeroGyros();
        zeroAccelerometers();
        zeroIntegralError();
        motors.pulseMotors(3);
      }   
      // Arm motors (left stick lower right corner)
      if (receiverData[YAW] > MAXCHECK && armed == 0 && safetyCheck == 1) {
        armed = 1;
        zeroIntegralError();
        transmitterCenter[PITCH] = receiverData[PITCH];
        transmitterCenter[ROLL] = receiverData[ROLL];
        for (motor=FRONT; motor < LASTMOTOR; motor++)
          minCommand[motor] = MINTHROTTLE;
      }
      // Prevents accidental arming of motor output if no transmitter command received
      if (receiverData[YAW] > MINCHECK) safetyCheck = 1; 
    }
    receiverTime = currentTime;
  } 
/////////////////////////////
// End of transmitter loop //
/////////////////////////////
  
// ***********************************************************
// ********************* Analog Input Loop *******************
// ***********************************************************
  if ((currentTime > (analogInputTime + AILOOPTIME)) && (analogInputLoop == ON)) { // 500Hz
    // *********************** Read Sensors **********************
    // Apply low pass filter to sensor values and center around zero
    // Did not convert to engineering units, since will experiment to find P gain anyway
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
      accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
    }

    #ifndef OriginalIMU
      gyroADC[ROLL] = -gyroADC[ROLL];
      gyroADC[PITCH] = -gyroADC[PITCH];
    #endif
    #ifdef IXZ
      gyroADC[YAW] = -gyroADC[YAW];
    #endif
  
    // Compiler seems to like calculating this in separate loop better
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      gyroData[axis] = smooth(gyroADC[axis], gyroData[axis], smoothFactor[GYRO]);
      accelData[axis] = smooth(accelADC[axis], accelData[axis], smoothFactor[ACCEL]);
    }

    // ****************** Calculate Absolute Angle *****************
    // angle[axis].calculate() defined in FlightAngle.h
    flightAngle[ROLL] = angle[ROLL].calculate(angleDeg(ROLL), rateDegPerSec(ROLL));
    //flightAngle[ROLL] = angle[ROLL].calculate(accelADC[ROLL], gyroData[ROLL]);
    flightAngle[PITCH] = angle[PITCH].calculate(angleDeg(PITCH), rateDegPerSec(PITCH));
    //flightAngle[PITCH] = angle[PITCH].calculate(accelADC[PITCH], -gyroData[PITCH]);
    analogInputTime = currentTime;
  } 
//////////////////////////////
// End of analog input loop //
//////////////////////////////
  
// ********************************************************************
// *********************** Flight Control Loop ************************
// ********************************************************************
  if ((currentTime > controlLoopTime + CONTROLLOOPTIME) && (controlLoop == ON)) { // 500Hz

  // ********************* Check Flight Mode *********************
      if (flightMode == ACRO) {
        // Acrobatic Mode
        // ************************** Update Roll/Pitch ***********************
        // updatePID(target, measured, PIDsettings);
        // measured = rate data from gyros scaled to PWM (1000-2000), since PID settings are found experimentally
        motorAxisCommand[ROLL] = updatePID(transmitterCommand[ROLL], gyroData[ROLL] + 1500, &PID[ROLL]);
        motorAxisCommand[PITCH] = updatePID(transmitterCommand[PITCH], gyroData[PITCH] + 1500, &PID[PITCH]);
      }
      if (flightMode == STABLE) {
        // Stable Mode
        // ************************** Update Roll/Pitch ***********************
        // updatePID(target, measured, PIDsettings);
        // measured = flight angle calculated from angle object
        motorAxisCommand[ROLL] = updatePIDangle(transmitterCommandSmooth[ROLL] * mLevelTransmitter + bLevelTransmitter, flightAngle[ROLL], updatePID(0, gyroData[ROLL], &PID[LEVELGYROROLL]), &PID[LEVELROLL]);
        motorAxisCommand[PITCH] = updatePIDangle(transmitterCommandSmooth[PITCH] * mLevelTransmitter + bLevelTransmitter, -flightAngle[PITCH], updatePID(0, gyroData[PITCH], &PID[LEVELGYROPITCH]), &PID[LEVELPITCH]);
      }
      
    // ***************************** Update Yaw ***************************
    // Note: gyro tends to drift over time, this will be better implemented when determining heading with magnetometer
    // Current method of calculating heading with gyro does not give an absolute heading, but rather is just used relatively to get a number to lock heading when no yaw input applied
    if (headingHoldConfig == ON) {
      if (transmitterCommandSmooth[AUX] < 1800) {
        currentHeading += gyroData[YAW] * headingScaleFactor * controldT;
        if (transmitterCommand[THROTTLE] > MINCHECK ) { // apply heading hold only when throttle high enough to start flight
          if ((transmitterCommand[YAW] > (MIDCOMMAND + 25)) || (transmitterCommand[YAW] < (MIDCOMMAND - 25))) { // if commanding yaw, turn off heading hold
            headingHold = 0;
            heading = currentHeading;
          }
          else // no yaw input, calculate current heading vs. desired heading heading hold
            headingHold = updatePID(heading, currentHeading, &PID[HEADING]);
        }
        else {
          heading = 0;
          currentHeading = 0;
          headingHold = 0;
          PID[HEADING].integratedError = 0;
        }
      }
    }   
    motorAxisCommand[YAW] = updatePID(transmitterCommand[YAW] + headingHold, gyroData[YAW] + 1500, &PID[YAW]);
      
    // *********************** Calculate Motor Commands **********************
    if (armed && safetyCheck) {
      #ifdef plusConfig
        motorCommand[FRONT] = transmitterCommand[THROTTLE] - motorAxisCommand[PITCH] - motorAxisCommand[YAW];
        motorCommand[REAR] = transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] - motorAxisCommand[YAW];
        motorCommand[RIGHT] = transmitterCommand[THROTTLE] - motorAxisCommand[ROLL] + motorAxisCommand[YAW];
        motorCommand[LEFT] = transmitterCommand[THROTTLE] + motorAxisCommand[ROLL] + motorAxisCommand[YAW];
      #endif
      #ifdef XConfig
        // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
        motorCommand[FRONT] = transmitterCommand[THROTTLE] - motorAxisCommand[PITCH] + motorAxisCommand[ROLL] - motorAxisCommand[YAW];
        motorCommand[RIGHT] = transmitterCommand[THROTTLE] - motorAxisCommand[PITCH] - motorAxisCommand[ROLL] + motorAxisCommand[YAW];
        motorCommand[LEFT] = transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] + motorAxisCommand[ROLL] + motorAxisCommand[YAW];
        motorCommand[REAR] = transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] - motorAxisCommand[ROLL] - motorAxisCommand[YAW];
      #endif
    }
    
    // ****************************** Altitude Adjust *************************
    // s = (at^2)/2, t = 0.002
    //zAccelHover += ((accelData[ZAXIS] * accelScaleFactor) * 0.000004) * 0.5;
    /*zAccelHover = accelADC[ROLL] / tan(angleRad(ROLL));
    throttleAdjust = limitRange((zAccelHover - accelADC[ZAXIS]) * throttleAdjustGain, minThrottleAdjust, maxThrottleAdjust);
    for (motor = FRONT; motor < LASTMOTOR; motor++)
      motorCommand[motor] += throttleAdjust;*/

    // Prevents too little power applied to motors during hard manuevers
    // Also provides even motor power on both sides if limit encountered
    if ((motorCommand[FRONT] <= MINTHROTTLE) || (motorCommand[REAR] <= MINTHROTTLE)){
      delta = transmitterCommand[THROTTLE] - 1100;
      maxCommand[RIGHT] = limitRange(transmitterCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
      maxCommand[LEFT] = limitRange(transmitterCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    }
    else if ((motorCommand[FRONT] >= MAXCOMMAND) || (motorCommand[REAR] >= MAXCOMMAND)) {
      delta = MAXCOMMAND - transmitterCommand[THROTTLE];
      minCommand[RIGHT] = limitRange(transmitterCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
      minCommand[LEFT] = limitRange(transmitterCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    }     
    else {
      maxCommand[RIGHT] = MAXCOMMAND;
      maxCommand[LEFT] = MAXCOMMAND;
      minCommand[RIGHT] = MINTHROTTLE;
      minCommand[LEFT] = MINTHROTTLE;
    }
    
    if ((motorCommand[LEFT] <= MINTHROTTLE) || (motorCommand[RIGHT] <= MINTHROTTLE)){
      delta = transmitterCommand[THROTTLE] - 1100;
      maxCommand[FRONT] = limitRange(transmitterCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
      maxCommand[REAR] = limitRange(transmitterCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    }
    else if ((motorCommand[LEFT] >= MAXCOMMAND) || (motorCommand[RIGHT] >= MAXCOMMAND)) {
      delta = MAXCOMMAND - transmitterCommand[THROTTLE];
      minCommand[FRONT] = limitRange(transmitterCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
      minCommand[REAR] = limitRange(transmitterCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    }     
    else {
      maxCommand[FRONT] = MAXCOMMAND;
      maxCommand[REAR] = MAXCOMMAND;
      minCommand[FRONT] = MINTHROTTLE;
      minCommand[REAR] = MINTHROTTLE;
    }
    
    // Apply limits to motor commands
    for (motor = FRONT; motor < LASTMOTOR; motor++)
      motorCommand[motor] = limitRange(motorCommand[motor], minCommand[motor], maxCommand[motor]);
 
    // Allows quad to do acrobatics by turning off opposite motors during hard manuevers
    if (flightMode == ACRO) {
      #ifdef plusConfig
      if (receiverData[ROLL] < MINCHECK) {
        motorCommand[LEFT] = MINCOMMAND;
        motorCommand[RIGHT] = MAXCOMMAND;
      }
      if (receiverData[ROLL] > MAXCHECK) {
        motorCommand[LEFT] = MAXCOMMAND;
        motorCommand[RIGHT] = MINCOMMAND;
      }
      if (receiverData[PITCH] < MINCHECK) {
       motorCommand[FRONT] = MAXCOMMAND;
       motorCommand[REAR] = MINCOMMAND;
      }
      if (receiverData[PITCH] > MAXCHECK) {
       motorCommand[FRONT] = MINCOMMAND;
       motorCommand[REAR] = MAXCOMMAND;
      }
      #endif
      #ifdef XConfig
      if (receiverData[ROLL] < MINCHECK) {
        motorCommand[FRONT] = MINCOMMAND;
        motorCommand[REAR] = MAXCOMMAND;
        motorCommand[LEFT] = MINCOMMAND;
        motorCommand[RIGHT] = MAXCOMMAND;
      }
      if (receiverData[ROLL] > MAXCHECK) {
        motorCommand[FRONT] = MAXCOMMAND;
        motorCommand[REAR] = MINCOMMAND;
        motorCommand[LEFT] = MAXCOMMAND;
        motorCommand[RIGHT] = MINCOMMAND;
      }
      if (receiverData[PITCH] < MINCHECK) {
        motorCommand[FRONT] = MAXCOMMAND;
        motorCommand[REAR] = MINCOMMAND;
        motorCommand[LEFT] = MINCOMMAND;
        motorCommand[RIGHT] = MAXCOMMAND;
      }
      if (receiverData[PITCH] > MAXCHECK) {
        motorCommand[FRONT] = MINCOMMAND;
        motorCommand[REAR] = MAXCOMMAND;
        motorCommand[LEFT] = MAXCOMMAND;
        motorCommand[RIGHT] = MINCOMMAND;
      }
      #endif
    }

    // If throttle in minimum position, don't apply yaw
    if (transmitterCommand[THROTTLE] < MINCHECK) {
      for (motor = FRONT; motor < LASTMOTOR; motor++)
        motorCommand[motor] = MINTHROTTLE;
    }
    // If motor output disarmed, force motor output to minimum
    if (armed == 0) {
      switch (calibrateESC) { // used for calibrating ESC's
      case 1:
        for (motor = FRONT; motor < LASTMOTOR; motor++)
          motorCommand[motor] = MAXCOMMAND;
        break;
      case 3:
        for (motor = FRONT; motor < LASTMOTOR; motor++)
          motorCommand[motor] = limitRange(testCommand, 1000, 1200);
        break;
      case 5:
        for (motor = FRONT; motor < LASTMOTOR; motor++)
          motorCommand[motor] = limitRange(remoteCommand[motor], 1000, 1200);
        safetyCheck = 1;
        break;
      default:
        for (motor = FRONT; motor < LASTMOTOR; motor++)
          motorCommand[motor] = MINCOMMAND;
      }
    }
    
    // *********************** Command Motors **********************
    motors.write(motorCommand); // Defined in Motors.h
    controlLoopTime = currentTime;
  } 
/////////////////////////
// End of control loop //
/////////////////////////
  
// *************************************************************
// **************** Command & Telemetry Functions **************
// *************************************************************
  if ((currentTime > telemetryTime + TELEMETRYLOOPTIME) && (telemetryLoop == ON)) { // 10Hz    
    readSerialCommand();
    sendSerialTelemetry();
    telemetryTime = currentTime;
      // Every 5 seconds we print an update
  }
///////////////////////////
// End of telemetry loop //
///////////////////////////
  
// *************************************************************
// ******************* Camera Stailization *********************
// *************************************************************
#ifdef Camera // Development moved to Arduino Mega
  if ((currentTime > (cameraTime + CAMERALOOPTIME)) && (cameraLoop == ON)) { // 50Hz
    rollCamera.write((mCamera * flightAngle[ROLL]) + bCamera);
    pitchCamera.write(-(mCamera * flightAngle[PITCH]) + bCamera);
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
    for (axis = ROLL; axis < LASTAXIS; axis++) printInt(gyroADC[axis]);
    for (axis = ROLL; axis < LASTAXIS; axis++) printInt(accelADC[axis]);
    printInt(32767); // Stop word of 0x7FFF
    fastTelemetryTime = currentTime;
  }
////////////////////////////////
// End of fast telemetry loop //
////////////////////////////////

// **************************************************************
// ************************ GPS Prototype ***********************
// **************************************************************
/*
  // Every 5 seconds we print an update
  while (millis() - start < 5000)
  {
    if (feedgps())
      newdata = true;
      //Serial.println("Check");
  }
  
  if (newdata)
  {
    Serial.println("Acquired Data");
    Serial.println("-------------");
    gpsdump(gps);
    Serial.println("-------------");
    Serial.println();
  }*/
}
