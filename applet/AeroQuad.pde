/*
  AeroQuad v1.0 - March 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Arduino based quadrocopter using the Sparkfun 5DOF IMU and IDG300 Dual Axis Gyro
  This version will be able to use gyros for stability (acrobatic mode) or accelerometers (experimental stable mode).
 
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

// To Do:
// Create automated calibration for Z axis

#include <stdlib.h>
#include <math.h>
#include <EEPROM.h>
#include <ServoTimer2.h>

// ******************** Initialize Variables ********************
#define BAUD 115200
#define LEDPIN 13

// Sensor pin assignments
/* Original orientation
#define XACCELPIN 0
#define YACCELPIN 1
#define ZACCELPIN 2
#define PITCHRATEPIN 3
#define ROLLRATEPIN 4
#define YAWRATEPIN 5*/
// New orientation
#define PITCHACCELPIN 4
#define ROLLACCELPIN 5
#define ZACCELPIN 3
#define PITCHRATEPIN 0
#define ROLLRATEPIN 1
#define YAWRATEPIN 2
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
#define LEVELINTERVAL_ADR 44
#define XMITFACTOR_ADR 48
#define GYROSMOOTH_ADR 52
#define ACCSMOOTH_ADR 56
#define LEVELPITCHCAL_ADR 60
#define LEVELROLLCAL_ADR 64
#define LEVELZCAL_ADR 68
#define FILTERTERM_ADR 72
/*#define ROLLCH_ADR 76
#define PITCHCH_ADR 80
#define YAWCH_ADR 84
#define THROTTLECH_ADR 88
#define MODECH_ADR 92
#define AUXCH_ADR 96
#define ROLLPIN_ADR 100
#define PITCHPIN_ADR 104
#define YAWPIN_ADR 108
#define THROTTLEPIN_ADR 112
#define MODEPIN_ADR 116
#define AUXPIN_ADR 120*/

// Motor control variables
#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000
#define MINDELTA 200
#define MINCHECK MINCOMMAND + 100
#define MAXCHECK MAXCOMMAND - 100
#define MINTHROTTLE MINCOMMAND + 50
#define FRONTMOTORPIN 8
#define REARMOTORPIN 9
#define RIGHTMOTORPIN 10
#define LEFTMOTORPIN 11
#define FRONT 0
#define REAR 1
#define RIGHT 2
#define LEFT 3
#define LASTMOTOR 4
ServoTimer2 frontMotor;
ServoTimer2 rearMotor;
ServoTimer2 rightMotor;
ServoTimer2 leftMotor;
int motorCommand[4] = {1000,1000,1000,1000};
int motorAxisCommand[3] = {0,0,0};
int motor, minCommand = 0;
// If AREF = 3.3V, then A/D is 931 at 3V and 465 = 1.5V 
// Scale gyro output (-465 to +465) to motor commands (1000 to 2000) 
// use y = mx + b 
float mMotorRate = 1.0753; // m = (y2 - y1) / (x2 - x1) = (2000 - 1000) / (465 - (-465)) 
float bMotorRate = 1500;   // b = y1 - m * x1 

// Transmitter variables
#define TIMEOUT 25000
#define THROTTLEPIN 4
#define ROLLPIN 2
#define PITCHPIN 3
#define YAWPIN 6
#define MODEPIN 7
#define AUXPIN 5
#define ROLL 0
#define PITCH 1
#define YAW 2
#define THROTTLE 3
#define MODE 4
#define AUX 5
#define LASTCHANNEL 6
volatile int transmitterData[6];
int orderCh[6] = {ROLL,AUX,MODE,PITCH,THROTTLE,YAW}; // order the channels are output from receiver
int xmitCh[6] = {2,5,7,3,4,6}; // digital pin assignments for each channel
int transmitterCommand[4] = {1500,1500,1500,1000};
int transmitterZero[3] = {1500,1500,1500};
int channel = 0;
// Controls the strength of the commands sent from the transmitter
// xmitFactor ranges from 0.01 - 1.0 (0.01 = weakest, 1.0 - strongest)
float xmitFactor; // Read in from EEPROM

// These A/D values depend on how well the sensors are mounted
// change these values to your unique configuration
// #define XMIN 405
// #define XMAX 607
// #define YMIN 409
// #define YMAX 618
#define ZMIN 403
#define ZMAX 611
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
int levelInterval; // Read in from EEPROM

// Gyro setup
int gyroData[3] = {0,0,0};
int gyroZero[3] = {0,0,0};
int gyroADC[3] = {0,0,0};

// Complementary roll/pitch angle
float flightAngle[2] = {0,0};
float filterTerm[2] = {0,0};
float timeConstant; // Read in from EEPROM

// Calibration parameters
#define FINDZERO 50
int findZero[FINDZERO];

// Low pass filter parameters
#define GYRO 0
#define ACCEL 1
float smoothFactor[2]; // Read in from EEPROM

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
volatile byte update = 0;

// Interrupt handler variables
volatile byte timeSlot;

// Timing
long previousTime = 0;
long currentTime = 0;
long deltaTime = 0;
long enableAcc = 0;
float dt = 0;

// ******************** Setup AeroQuadAero ********************
void setup() {
  Serial.begin(BAUD);
  analogReference(EXTERNAL); // Current external ref is connected to 3.3V
  pinMode (LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  
  // Configure motors
  configureMotors();
  commandAllMotors(MINCOMMAND);

  // Read user values from EEPROM
  readEEPROM();
  
  // Setup interrupt to trigger on pin D2
  configureTransmitter();
  
  // Wait for battery to be plugged in before ESC's are armed
  //armESC(3000);
  
  // Calibrate sensors
  zeroGyros();
  
  // Complementary filter setup
  configureFilter(timeConstant);
  
  previousTime = millis();
  digitalWrite(LEDPIN, HIGH);
  safetyCheck = 0;
}

// ******************** Main AeroQuad Loop ********************
void loop () {
  // Measure loop rate
  currentTime = millis();
  deltaTime = currentTime - previousTime;
  previousTime = currentTime;
  enableAcc++;
  
  // Send configuration commands from transmitter
  if (transmitterData[THROTTLE] < 1010) {
    zeroIntegralError();
    // Disarm motors (throttle down, yaw left)
    //if (yaw < MINCHECK && armed == 1) {
    if (transmitterData[YAW] > MAXCHECK && armed == 1) {
      armed = 0;
      commandAllMotors(MINCOMMAND);
    }    
    // Zero sensors (throttle down, yaw left, roll left)
    //if (yaw < MINCHECK && roll > MAXCHECK) {
    if (transmitterData[YAW] > MAXCHECK && transmitterData[ROLL] < MINCHECK) {
      zeroGyros();
      zeroAccelerometers();
      zeroIntegralError();
      pulseMotors(3);
    }   
    // Arm motors (throttle down, yaw right)  
    //if (yaw > MAXCHECK && armed == 0) {
    if (transmitterData[YAW] < MINCHECK && armed == 0 && safetyCheck == 1) {
      armed = 1;
      zeroIntegralError();
      minCommand = MINTHROTTLE;
    }
    if (transmitterData[YAW] > MINCHECK) safetyCheck = 1; 
  }
  else if (transmitterData[THROTTLE] > (MIDCOMMAND - MINDELTA)) minCommand = transmitterData[THROTTLE] - MINDELTA;
  if (transmitterData[THROTTLE] < MINTHROTTLE) minCommand = MINTHROTTLE;

  // Read Sensors
  // Apply low pass filter to sensor values and center around zero
  // Did not convert to engineering units, since will apply P gain anyway
  for (axis = ROLL; axis < LASTAXIS; axis++) {
    gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
    accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
  }
  
  for (axis = ROLL; axis < LASTAXIS; axis++) {
    gyroData[axis] = smooth(gyroADC[axis], gyroData[axis], smoothFactor[GYRO]);
    accelData[axis] = smooth(accelADC[axis], accelData[axis], smoothFactor[ACCEL]);
  }
  
  dt = deltaTime / 1000.0;
  for (axis = ROLL; axis < YAW; axis++)
    flightAngle[axis] = filterData(flightAngle[axis], gyroADC[axis], accelADC[axis], filterTerm, dt);
  
  if (transmitterData[MODE] < 1500) {
    // Acrobatic Mode
    levelAdjust[ROLL] = 0;
    levelAdjust[PITCH] = 0;
  }
  else {
    // Stable Mode
    if ((enableAcc % levelInterval) == 0) {
      for (axis = ROLL; axis < YAW; axis++)
        levelAdjust[axis] = limitRange(updatePID(0, flightAngle[axis], &PID[LEVELROLL + axis]), -levelLimit, levelLimit);
    }
  }
  
  // Transmitter Commands
  // Calculations are performed in Transmitter.pde when receiver channels are read
  // Commands calculated are transmitterCommand[ROLL], transmitterCommand[PITCH], transmitterCommand[YAW]
  // Saves processing time, since values only change when reading receiver
  
  // Update PID
  motorAxisCommand[ROLL] = updatePID(transmitterCommand[ROLL] - levelAdjust[ROLL], (gyroData[ROLL] * mMotorRate) + bMotorRate, &PID[ROLL]);
  motorAxisCommand[PITCH] = updatePID(transmitterCommand[PITCH] + levelAdjust[PITCH], (gyroData[PITCH] * mMotorRate) + bMotorRate, &PID[PITCH]);
  motorAxisCommand[YAW] = updatePID(transmitterCommand[YAW], (gyroData[YAW] * mMotorRate) + bMotorRate, &PID[YAW]);
    
  // Calculate motor commands
  // Original orientation
  // frontCommand =limitRange(throttle + pitchMotorCommand - yawMotorCommand - pitchLevelAdjust, MINCOMMAND, MAXCOMMAND);
  // rearCommand = limitRange(throttle - pitchMotorCommand - yawMotorCommand + pitchLevelAdjust, MINCOMMAND, MAXCOMMAND);
  // rightCommand = limitRange(throttle + rollMotorCommand + yawMotorCommand + rollLevelAdjust, MINCOMMAND, MAXCOMMAND);
  // leftCommand = limitRange(throttle - rollMotorCommand + yawMotorCommand - rollLevelAdjust, MINCOMMAND, MAXCOMMAND);

  if (armed && safetyCheck) {
    motorCommand[FRONT] =limitRange(transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] + motorAxisCommand[YAW], minCommand, MAXCOMMAND);
    motorCommand[REAR] = limitRange(transmitterCommand[THROTTLE] - motorAxisCommand[PITCH] + motorAxisCommand[YAW], minCommand, MAXCOMMAND);
    motorCommand[RIGHT] = limitRange(transmitterCommand[THROTTLE] - motorAxisCommand[ROLL] - motorAxisCommand[YAW], minCommand, MAXCOMMAND);
    motorCommand[LEFT] = limitRange(transmitterCommand[THROTTLE] + motorAxisCommand[ROLL] - motorAxisCommand[YAW], minCommand, MAXCOMMAND);
  }
  else {
    for (motor = FRONT; motor < LASTMOTOR; motor++)
      motorCommand[motor] = MINCOMMAND;
  }
  
  // Command motors
  commandMotors();
  
  // Check for remote commands and send requested telemetry
  readSerialCommand();
  if (update) sendSerialTelemetry();
}
