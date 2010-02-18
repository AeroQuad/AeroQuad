/*
  AeroQuad v1.6 - February 2010
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

/**************************************************************************** 
   Before flight, select the different user options for your AeroQuad below
   Also, consult the ReadMe.html file for additional details
   If you need additional assitance go to http://forum.AeroQuad.info
*****************************************************************************/

// Define Flight Configuration
#define plusConfig
//#define XConfig

// 5DOF IMU Version
//#define OriginalIMU // Use this if you have the 5DOF IMU which uses the IDG300

// Yaw Gyro Type
#define IXZ // IXZ-500 Flat Yaw Gyro
//#define IDG // IDG-500 Dual Axis Gyro

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

// Sensor Filter
// The Kalman Filter implementation is here for comparison against the Complementary Filter
// To adjust the KF parameters, look at initGyro1DKalman() found inside ConfigureFilter() in Filter.pde
//#define KalmanFilter

// Heading Hold (experimental)
// Currently uses yaw gyro which drifts over time, for Mega development will use magnetometer
//#define HeadingHold

// Auto Level (experimental)
//#define AutoLevel

// Camera Stabilization (experimental)
// Will move development to Arduino Mega (needs analogWrite support for additional pins)
//#define Camera

// *************************************************************

#include <stdlib.h>
#include <math.h>
#include <EEPROM.h>
#include <Servo.h>
#define byte uint8_t
#include "Eeprom.h"
#include "PID.h"
#include "Receiver.h"
#include "Sensors.h"
#include "Motors.h"
#include "AeroQuad.h"
#include "Filter.h"

// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
void setup() {
  Serial.begin(BAUD);
  analogReference(EXTERNAL); // Current external ref is connected to 3.3V
  pinMode (LEDPIN, OUTPUT);
  pinMode (AZPIN, OUTPUT);
  digitalWrite(AZPIN, LOW);
  delay(1);
  
  // Configure motors
  configureMotors();
  commandAllMotors(MINCOMMAND);

  // Read user values from EEPROM
  readEEPROM();
  
  // Setup receiver pins for pin change interrupts
  if (receiverLoop == ON)
     configureReceiver();
  
  //  Auto Zero Gyros
  autoZeroGyros();
  
  // Calibrate sensors
  zeroGyros();
  zeroIntegralError();
  levelAdjust[ROLL] = 0;
  levelAdjust[PITCH] = 0;
  
  // Camera stabilization setup
  #ifdef Camera
    rollCamera.attach(ROLLCAMERAPIN);
    pitchCamera.attach(PITCHCAMERAPIN);
  #endif
  
  // Complementary filter setup
  configureFilter(timeConstant);
  
  previousTime = millis();
  digitalWrite(LEDPIN, HIGH);
  safetyCheck = 0;
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
        commandAllMotors(MINCOMMAND);
      }    
      // Zero sensors (left stick lower left, right stick lower right corner)
      if ((receiverData[YAW] < MINCHECK) && (receiverData[ROLL] > MAXCHECK) && (receiverData[PITCH] < MINCHECK)) {
        autoZeroGyros();
        zeroGyros();
        zeroAccelerometers();
        zeroIntegralError();
        pulseMotors(3);
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
    // Allows quad to do acrobatics by turning off opposite motors during hard manuevers
    //if ((receiverData[ROLL] < MINCHECK) || (receiverData[ROLL] > MAXCHECK) || (receiverData[PITCH] < MINCHECK) || (receiverData[PITCH] > MAXCHECK))
      //minCommand = MINTHROTTLE;
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
    #ifndef KalmanFilter
      //filterData(previousAngle, newAngle, rate, *filterTerm, dt)
      flightAngle[ROLL] = filterData(flightAngle[ROLL], angleDeg(ROLL), rateDegPerSec(ROLL), filterTermRoll, AIdT);
      flightAngle[PITCH] = filterData(flightAngle[PITCH], angleDeg(PITCH), rateDegPerSec(PITCH), filterTermPitch, AIdT);
    #endif
      
    #ifdef KalmanFilter
      predictKalman(&rollFilter, rateDegPerSec(ROLL), AIdT);
      flightAngle[ROLL] = updateKalman(&rollFilter, angleDeg(ROLL));
      predictKalman(&pitchFilter, rateDegPerSec(PITCH), AIdT);
      flightAngle[PITCH] = updateKalman(&pitchFilter, angleDeg(PITCH));
    #endif
    
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
    #ifdef AutoLevel
      if (transmitterCommandSmooth[MODE] < 1500) {
        // Acrobatic Mode
        levelAdjust[ROLL] = 0;
        levelAdjust[PITCH] = 0;
      }
      else {
        // Stable Mode
        for (axis = ROLL; axis < YAW; axis++)
          levelAdjust[axis] = limitRange(updatePID(0, flightAngle[axis], &PID[LEVELROLL + axis]), -levelLimit, levelLimit);
        // Turn off Stable Mode if transmitter stick applied
        if ((abs(receiverData[ROLL] - transmitterCenter[ROLL]) > levelOff)) {
          levelAdjust[ROLL] = 0;
          PID[axis].integratedError = 0;
        }
        if ((abs(receiverData[PITCH] - transmitterCenter[PITCH]) > levelOff)) {
          levelAdjust[PITCH] = 0;
          PID[PITCH].integratedError = 0;
        }
      }
    #endif
    
    // ************************** Update Roll/Pitch ***********************
    // updatedPID(target, measured, PIDsettings);
    // measured = rate data from gyros scaled to PWM (1000-2000), since PID settings are found experimentally
    motorAxisCommand[ROLL] = updatePID(transmitterCommand[ROLL] + levelAdjust[ROLL], (gyroData[ROLL] * mMotorRate) + bMotorRate, &PID[ROLL]);
    motorAxisCommand[PITCH] = updatePID(transmitterCommand[PITCH] - levelAdjust[PITCH], (gyroData[PITCH] * mMotorRate) + bMotorRate, &PID[PITCH]);

    // ***************************** Update Yaw ***************************
    // Note: gyro tends to drift over time, this will be better implemented when determining heading with magnetometer
    // Current method of calculating heading with gyro does not give an absolute heading, but rather is just used relatively to get a number to lock heading when no yaw input applied
    #ifdef HeadingHold
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
      motorAxisCommand[YAW] = updatePID(transmitterCommand[YAW] + headingHold, (gyroData[YAW] * mMotorRate) + bMotorRate, &PID[YAW]);
    #endif
    
    #ifndef HeadingHold
      motorAxisCommand[YAW] = updatePID(transmitterCommand[YAW], (gyroData[YAW] * mMotorRate) + bMotorRate, &PID[YAW]);
    #endif
  
    // ****************** Calculate Motor Commands *****************
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
        motorCommand[RIGHT] = transmitterCommand[THROTTLE] - motorAxisCommand[PITCH] - motorAxisCommand[ROLL] + motorAxisCommand[YAW]);
        motorCommand[LEFT] = transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] + motorAxisCommand[ROLL] + motorAxisCommand[YAW];
        motorCommand[REAR] = transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] - motorAxisCommand[ROLL] - motorAxisCommand[YAW]);
      #endif
    }
  
    // Prevents too little power applied to motors during hard manuevers
    // Also even motor power on both sides if limit enabled
    if ((motorCommand[FRONT] <= MINTHROTTLE) || (motorCommand[REAR] <= MINTHROTTLE)){
      delta = transmitterCommand[THROTTLE] - 1100;
      maxCommand[RIGHT] = limitRange(transmitterCommand[THROTTLE] + delta, MINTHROTTLE, MAXCOMMAND);
      maxCommand[LEFT] = limitRange(transmitterCommand[THROTTLE] + delta, MINTHROTTLE, MAXCOMMAND);
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
      maxCommand[FRONT] = limitRange(transmitterCommand[THROTTLE] + delta, MINTHROTTLE, MAXCOMMAND);
      maxCommand[REAR] = limitRange(transmitterCommand[THROTTLE] + delta, MINTHROTTLE, MAXCOMMAND);
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
    
    for (motor = FRONT; motor < LASTMOTOR; motor++)
      motorCommand[motor] = limitRange(motorCommand[motor], minCommand[motor], maxCommand[motor]);

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
    commandMotors();
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

}
