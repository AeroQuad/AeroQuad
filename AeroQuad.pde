/*
  AeroQuad v1.3 - September 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Open Source Arduino based quadrocopter.
 
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

// ************************ User Options ***********************

// Define Flight Configuration
//#define plusConfig
#define XConfig

// Calibration At Powerup
//#define CalibrationAtPower

// Define Motor PWM Approach
#define AnalogWrite
//#define ServoTimerTwo

// Camera Stabilization
//#define Camera

// Experimental Auto Level (still under development)
//#define AutoLevel

// *************************************************************

#include <stdlib.h>
#include <math.h>
#include <EEPROM.h>
#ifdef ServoTimerTwo
  #include <Servo.h>
#endif

#define BAUD 115200
#define LEDPIN 13

// Sensor pin assignments
#define PITCHACCELPIN 0
#define ROLLACCELPIN 1
#define ZACCELPIN 2
#define PITCHRATEPIN 3
#define ROLLRATEPIN 4
#define YAWRATEPIN 5
int gyroChannel[3] = {ROLLRATEPIN, PITCHRATEPIN, YAWRATEPIN};
int accelChannel[3] = {ROLLACCELPIN, PITCHACCELPIN, ZACCELPIN};

// EEPROM storage
#define PGAIN_ADR 0
#define IGAIN_ADR 4
#define DGAIN_ADR 8
#define LEVEL_PGAIN_ADR 12
#define LEVEL_IGAIN_ADR 16
#define LEVEL_DGAIN_ADR 20
#define YAW_PGAIN_ADR 24
#define YAW_IGAIN_ADR 28
#define YAW_DGAIN_ADR 32
#define WINDUPGUARD_ADR 36
#define LEVELLIMIT_ADR 40
#define LEVELOFF_ADR 44
#define XMITFACTOR_ADR 48
#define GYROSMOOTH_ADR 52
#define ACCSMOOTH_ADR 56
#define LEVELPITCHCAL_ADR 60
#define LEVELROLLCAL_ADR 64
#define LEVELZCAL_ADR 68
#define FILTERTERM_ADR 72
#define MODESMOOTH_ADR 76
#define ROLLSMOOTH_ADR 80
#define PITCHSMOOTH_ADR 84
#define YAWSMOOTH_ADR 88
#define THROTTLESMOOTH_ADR 92
#define GYRO_ROLL_ZERO_ADR 96
#define GYRO_PITCH_ZERO_ADR 100
#define GYRO_YAW_ZERO_ADR 104
#define PITCH_PGAIN_ADR 124
#define PITCH_IGAIN_ADR 128
#define PITCH_DGAIN_ADR 132
#define LEVEL_PITCH_PGAIN_ADR 136
#define LEVEL_PITCH_IGAIN_ADR 140
#define LEVEL_PITCH_DGAIN_ADR 144
#define THROTTLESCALE_ADR 148
#define THROTTLEOFFSET_ADR 152
#define ROLLSCALE_ADR 156
#define ROLLOFFSET_ADR 160
#define PITCHSCALE_ADR 164
#define PITCHOFFSET_ADR 168
#define YAWSCALE_ADR 172
#define YAWOFFSET_ADR 176
#define MODESCALE_ADR 180
#define MODEOFFSET_ADR 184
#define AUXSCALE_ADR 188
#define AUXOFFSET_ADR 192
#define AUXSMOOTH_ADR 196

// Motor control variables
#define FRONTMOTORPIN 3
#define REARMOTORPIN 9
#define RIGHTMOTORPIN 10
#define LEFTMOTORPIN 11
#define LASTMOTORPIN 12
#define FRONT 0
#define REAR 1
#define RIGHT 2
#define LEFT 3
#define LASTMOTOR 4
#ifdef ServoTimerTwo
  Servo frontMotor;
  Servo rearMotor;
  Servo rightMotor;
  Servo leftMotor;
#endif
int motorCommand[4] = {1000,1000,1000,1000};
int motorAxisCommand[3] = {0,0,0};
int motor, minCommand = 0;
// If AREF = 3.3V, then A/D is 931 at 3V and 465 = 1.5V 
// Scale gyro output (-465 to +465) to motor commands (1000 to 2000) 
// use y = mx + b 
float mMotorRate = 1.0753; // m = (y2 - y1) / (x2 - x1) = (2000 - 1000) / (465 - (-465)) 
float bMotorRate = 1500;   // b = y1 - m * x1
// Scale motor commands to analogWrite
// m = (250-126)/(2000-1000) = 0.124
// b = y1 - (m * x1) = 126 - (0.124 * 1000) = 2
float mMotorCommand = 0.124;
float bMotorCommand = 2;

// Receiver variables
#define TIMEOUT 25000
#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000
#define MINDELTA 200
#define MINCHECK MINCOMMAND + 100
#define MAXCHECK MAXCOMMAND - 100
#define MINTHROTTLE MINCOMMAND + 100
#define LEVELOFF 100
#define ROLLPIN 2
#define THROTTLEPIN 4
#define PITCHPIN 5
#define YAWPIN 6
#define MODEPIN 7
#define AUXPIN 8
#define ROLL 0
#define PITCH 1
#define YAW 2
#define THROTTLE 3
#define MODE 4
#define AUX 5
#define LASTCHANNEL 6
#ifndef Camera
  int receiverChannel[6] = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, MODEPIN, AUXPIN}; // defines Arduino pins
  int receiverPin[6] = {18, 21, 22, 20, 23, 0}; // defines ATmega328P pins (Arduino pins converted to ATmega328P pinouts)
#endif
#ifdef Camera
  int receiverChannel[6] = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, MODEPIN, MODEPIN}; // defines Arduino pins
  int receiverPin[6] = {18, 21, 22, 20, 23, 23}; // defines ATmega328P pins (Arduino pins converted to ATmega328P pinouts)
#endif
int receiverData[6];
int transmitterCommand[6] = {1500,1500,1500,1000,1000,1000};
int transmitterCommandSmooth[6] = {0,0,0,0,0,0};
int transmitterZero[3] = {1500,1500,1500};
int transmitterCenter[2] = {1500,1500};
byte channel;
// Controls the strength of the commands sent from the transmitter
// xmitFactor ranges from 0.01 - 1.0 (0.01 = weakest, 1.0 - strongest)
float xmitFactor; // Read in from EEPROM
float mTransmitter[6] = {1,1,1,1,1,1};
float bTransmitter[6] = {0,0,0,0,0,0};

// These A/D values depend on how well the sensors are mounted
// change these values to your unique configuration
// #define XMIN 405
// #define XMAX 607
// #define YMIN 409
// #define YMAX 618
#define ZMIN 454
#define ZMAX 687
#define ZAXIS 2
#define ZEROLIMIT 2
int axis;

// Accelerometer setup
int accelData[3] = {0,0,0};
int accelZero[3] = {0,0,0};
int accelADC[3] = {0,0,0};

// Auto level setup
int levelAdjust[2] = {0,0};
int levelLimit; // Read in from EEPROM
int levelOff; // Read in from EEPROM

// Gyro setup
int gyroData[3] = {0,0,0};
int gyroZero[3] = {0,0,0};
int gyroADC[3] = {0,0,0};

// Complementary roll/pitch angle
float flightAngle[2] = {0,0};
float filterTermRoll[4] = {0,0,0,0};
float filterTermPitch[4] = {0,0,0,0};
float timeConstant; // Read in from EEPROM

// Camera stabilization variables
#define ROLLCAMERAPIN 12
#define PITCHCAMERAPIN 13
// map +/-90 degrees to 1000-2000
float mCamera = 5.556;
float bCamera = 1500;
#ifdef Camera
  Servo rollCamera;
  Servo pitchCamera;
#endif

// Calibration parameters
#define FINDZERO 50
int findZero[FINDZERO];

// Low pass filter parameters
#define GYRO 0
#define ACCEL 1
float smoothFactor[2]; // Read in from EEPROM
float smoothTransmitter[6]; // Read in from EEPROM

// PID Values
#define LASTAXIS 3
#define LEVELROLL 3
#define LEVELPITCH 4
#define LASTLEVELAXIS 5
struct PIDdata {
  float P, I, D;
  float lastPosition;
  float integratedError;
} PID[5];
float windupGuard; // Read in from EEPROM

// Communication
char queryType = 'X';
byte tlmType = 0;
char string[32];
byte armed = 0;
byte safetyCheck = 0;
byte update = 0;

// Interrupt handler variables
byte timeSlot;

// Timing
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;
unsigned long receiverTime =0;
unsigned long telemetryTime = 50; // make telemetry output 50ms offset from receiver check
float dt = 0;

// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
void setup() {
  Serial.begin(BAUD);
  analogReference(EXTERNAL); // Current external ref is connected to 3.3V
  pinMode (LEDPIN, OUTPUT);
  
  // Configure motors
  configureMotors();
  commandAllMotors(MINCOMMAND);

  // Read user values from EEPROM
  readEEPROM();
  
  // Setup receiver pins for pin change interrupts
  configureReceiver();
  
  #ifdef CalibrationAtStartup
    // Calibrate sensors
    zeroGyros();
    zeroAccelerometers();
    zeroIntegralError();
  #endif
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
  
// ******************* Transmitter Commands *******************
  if (currentTime > (receiverTime + 100)) {
    // Buffer receiver values read from pin change interrupt handler
    for (channel = ROLL; channel < LASTCHANNEL; channel++)
      receiverData[channel] = (mTransmitter[channel] * readReceiver(receiverPin[channel])) + bTransmitter[channel];
    // Smooth the flight control transmitter inputs (roll, pitch, yaw, throttle)
    for (axis = ROLL; axis < LASTCHANNEL; axis++)
      transmitterCommandSmooth[axis] = smooth(receiverData[axis], transmitterCommandSmooth[axis], smoothTransmitter[axis]);
      //transmitterCommandSmooth[axis] = limitRange(smooth(receiverData[axis], transmitterCommandSmooth[axis], smoothTransmitter[axis]), MINCOMMAND, MAXCOMMAND);
    // Reduce transmitter commands using xmitFactor and center around 1500
    for (axis = ROLL; axis < LASTAXIS; axis++)
      transmitterCommand[axis] = ((transmitterCommandSmooth[axis] - transmitterZero[axis]) * xmitFactor) + transmitterZero[axis];
    // No xmitFactor reduction applied for throttle, mode and AUX
    for (axis = THROTTLE; axis < LASTCHANNEL; axis++)
      transmitterCommand[axis] = transmitterCommandSmooth[axis];
    // Read quad configuration commands from transmitter when throttle down
    if (receiverData[THROTTLE] < MINCHECK) {
      zeroIntegralError();
      // Disarm motors (left stick lower left corner)
      if (receiverData[YAW] < MINCHECK && armed == 1) {
        armed = 0;
        commandAllMotors(MINCOMMAND);
      }    
      // Zero sensors (left stick lower left, right stick lower right corner)
      if (receiverData[YAW] < MINCHECK && receiverData[ROLL] > MAXCHECK && receiverData[PITCH] < MINCHECK) {
        zeroGyros();
        zeroAccelerometers();
        zeroIntegralError();
        pulseMotors(3);
      }   
      // Arm motors (left stick lower right corner)
      if (receiverData[YAW] > MAXCHECK && armed == 0 && safetyCheck == 1) {
        armed = 1;
        zeroIntegralError();
        minCommand = MINTHROTTLE;
        transmitterCenter[PITCH] = receiverData[PITCH];
        transmitterCenter[ROLL] = receiverData[ROLL];
      }
      // Prevents accidental arming of motor output if no transmitter command received
      if (receiverData[YAW] > MINCHECK) safetyCheck = 1; 
    }
    // Prevents too little power applied to motors during hard manuevers
    if (receiverData[THROTTLE] > (MIDCOMMAND - MINDELTA)) minCommand = receiverData[THROTTLE] - MINDELTA;
    if (receiverData[THROTTLE] < MINTHROTTLE) minCommand = MINTHROTTLE;
    receiverTime = currentTime;
  }
  
// *********************** Read Sensors **********************
  // Apply low pass filter to sensor values and center around zero
  // Did not convert to engineering units, since will experiment to find P gain anyway
  for (axis = ROLL; axis < LASTAXIS; axis++) {
    gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
    accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
  }
  // Compiler seems to like calculating this in separate loop better
  for (axis = ROLL; axis < LASTAXIS; axis++) {
    gyroData[axis] = smooth(gyroADC[axis], gyroData[axis], smoothFactor[GYRO]);
    accelData[axis] = smooth(accelADC[axis], accelData[axis], smoothFactor[ACCEL]);
  }

// ****************** Calculate Absolute Angle *****************
  dt = deltaTime / 1000.0; // Convert to seconds from milliseconds for complementary filter
  flightAngle[ROLL] = filterData(flightAngle[ROLL], gyroADC[ROLL], atan2(accelADC[ROLL], accelADC[ZAXIS]), filterTermRoll, dt);
  flightAngle[PITCH] = filterData(flightAngle[PITCH], gyroADC[PITCH], atan2(accelADC[PITCH], accelADC[ZAXIS]), filterTermPitch, dt);  

// ********************* Check Flight Mode *********************
  #ifdef AutoLevel
    if (receiverData[MODE] < 1200) {
      // Acrobatic Mode
      levelAdjust[ROLL] = 0;
      levelAdjust[PITCH] = 0;
    }
    else {
      // Stable Mode
      for (axis = ROLL; axis < YAW; axis++)
        levelAdjust[axis] = limitRange(updatePID(0, flightAngle[axis], &PID[LEVELROLL + axis]), -levelLimit, levelLimit);
      // Turn off Stable Mode if transmitter stick applied
      if ((abs(receiverData[PITCH] - transmitterCenter[PITCH]) > levelOff))
        levelAdjust[PITCH] = 0;
      if ((abs(receiverData[ROLL] - transmitterCenter[ROLL]) > levelOff))
        levelAdjust[ROLL] = 0;
    }
  #endif
  
// ************************* Update PID ************************
  motorAxisCommand[ROLL] = updatePID(transmitterCommand[ROLL] + levelAdjust[ROLL], (gyroData[ROLL] * mMotorRate) + bMotorRate, &PID[ROLL]);
  motorAxisCommand[PITCH] = updatePID(transmitterCommand[PITCH] - levelAdjust[PITCH], (gyroData[PITCH] * mMotorRate) + bMotorRate, &PID[PITCH]);
  motorAxisCommand[YAW] = updatePID(transmitterCommand[YAW], (gyroData[YAW] * mMotorRate) + bMotorRate, &PID[YAW]);
  
// ****************** Calculate Motor Commands *****************
  if (armed && safetyCheck) {
    #ifdef plusConfig
      motorCommand[FRONT] = limitRange(transmitterCommand[THROTTLE] - motorAxisCommand[PITCH] - motorAxisCommand[YAW], minCommand, MAXCOMMAND);
      motorCommand[REAR] = limitRange(transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] - motorAxisCommand[YAW], minCommand, MAXCOMMAND);
      motorCommand[RIGHT] = limitRange(transmitterCommand[THROTTLE] - motorAxisCommand[ROLL] + motorAxisCommand[YAW], minCommand, MAXCOMMAND);
      motorCommand[LEFT] = limitRange(transmitterCommand[THROTTLE] + motorAxisCommand[ROLL] + motorAxisCommand[YAW], minCommand, MAXCOMMAND);
    #endif
    #ifdef XConfig
      // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
      motorCommand[FRONT] = limitRange(transmitterCommand[THROTTLE] - motorAxisCommand[PITCH] + motorAxisCommand[ROLL] - motorAxisCommand[YAW], minCommand, MAXCOMMAND);
      motorCommand[RIGHT] = limitRange(transmitterCommand[THROTTLE] - motorAxisCommand[PITCH] - motorAxisCommand[ROLL] + motorAxisCommand[YAW], minCommand, MAXCOMMAND);
      motorCommand[LEFT] = limitRange(transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] + motorAxisCommand[ROLL] + motorAxisCommand[YAW], minCommand, MAXCOMMAND);
      motorCommand[REAR] = limitRange(transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] - motorAxisCommand[ROLL] - motorAxisCommand[YAW], minCommand, MAXCOMMAND);
    #endif
  }
  
  // If throttle in minimum position, don't apply yaw
  if (transmitterCommand[THROTTLE] < MINCHECK) {
    for (motor = FRONT; motor < LASTMOTOR; motor++)
      motorCommand[motor] = minCommand;
  }
  // If motor output disarmed, force motor output to minimum
  if (armed == 0) {
    for (motor = FRONT; motor < LASTMOTOR; motor++)
      motorCommand[motor] = MINCOMMAND;
  }

// *********************** Command Motors **********************
  commandMotors();
  
// **************** Command & Telemetry Functions **************
  readSerialCommand();
  if (currentTime > (telemetryTime + 100)) {
    sendSerialTelemetry();
    telemetryTime = currentTime;
  }
  
// ******************* Camera Stailization *********************
#ifdef Camera
  rollCamera.write((mCamera * flightAngle[ROLL]) + bCamera);
  pitchCamera.write(-(mCamera * flightAngle[PITCH]) + bCamera);
#endif
}
