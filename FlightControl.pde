/*
  AeroQuad v2.2 - Feburary 2011
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

// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)
// Special thanks to Keny9999 for suggesting a more readable format for FlightControl.pde and for
// porting over the ArduPirates Stable Mode (please note this is still experimental, use at your own risk)

#define MAX_CONTROL_OUTPUT 500

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// ArduPirateSuperStableProcessor ///////////////////
//////////////////////////////////////////////////////////////////////////////
void processArdupirateSuperStableMode(void)
{
  // ArduPirate adaptation
  // default value are P = 4, I = 0.15, P (gyro) = 1.2
  // ROLL
  float errorRoll = (_receiver->getAngle(ROLL) - _flightAngle->getData(ROLL));     
  errorRoll = constrain(errorRoll,-50,50);
  if (_receiver->getAngle(ROLL) < 30) 
  {
    PID[LEVELROLL].integratedError += errorRoll*G_Dt;                            
    PID[LEVELROLL].integratedError = constrain(PID[LEVELROLL].integratedError,-20,20);
  }
  else
  {
    PID[LEVELROLL].integratedError = 0;
  }
  const float stableRoll = PID[LEVELROLL].P * errorRoll + PID[LEVELROLL].I * PID[LEVELROLL].integratedError;
  errorRoll = stableRoll - _flightAngle->getGyroUnbias(ROLL);
  _motors->setMotorAxisCommand(ROLL,constrain(PID[LEVELGYROROLL].P*errorRoll,-MAX_CONTROL_OUTPUT,MAX_CONTROL_OUTPUT));

  // PITCH
  float errorPitch = (_receiver->getAngle(PITCH) + _flightAngle->getData(PITCH));     
  errorPitch = constrain(errorPitch,-50,50);                    
  if (_receiver->getAngle(PITCH) < 30) 
  {
    PID[LEVELPITCH].integratedError += errorPitch*G_Dt;                            
    PID[LEVELPITCH].integratedError = constrain(PID[LEVELPITCH].integratedError,-20,20);
  }
  else
  {
    PID[LEVELPITCH].integratedError = 0;
  }
  const float stablePitch = PID[LEVELPITCH].P * errorPitch + PID[LEVELPITCH].I * PID[LEVELPITCH].integratedError;
  errorPitch = stablePitch - _flightAngle->getGyroUnbias(PITCH);
  _motors->setMotorAxisCommand(PITCH,constrain(PID[LEVELGYROPITCH].P*errorPitch,-MAX_CONTROL_OUTPUT,MAX_CONTROL_OUTPUT));
}


//////////////////////////////////////////////////////////////////////////////
/////////////////////////// AQ Original Stable Mode //////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processAeroQuadStableMode(void)
{
  _levelAdjust[ROLL] = (_receiver->getAngle(ROLL) - _flightAngle->getData(ROLL)) * PID[LEVELROLL].P;
  _levelAdjust[PITCH] = (_receiver->getAngle(PITCH) + _flightAngle->getData(PITCH)) * PID[LEVELPITCH].P;
  // Check if pilot commands are not in hover, don't auto trim
  if ((abs(_receiver->getTrimData(ROLL)) > _levelOff) || (abs(_receiver->getTrimData(PITCH)) > _levelOff)) 
  {
    zeroIntegralError();
    #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
      digitalWrite(LED2PIN, LOW);
    #endif
    #ifdef APM_OP_CHR
      digitalWrite(LED_Green, LOW);
    #endif
  }
  else 
  {
    PID[LEVELROLL].integratedError = constrain(PID[LEVELROLL].integratedError + (((_receiver->getAngle(ROLL) - _flightAngle->getData(ROLL)) * G_Dt) * PID[LEVELROLL].I), -_levelLimit, _levelLimit);
    PID[LEVELPITCH].integratedError = constrain(PID[LEVELPITCH].integratedError + (((_receiver->getAngle(PITCH) + _flightAngle->getData(PITCH)) * G_Dt) * PID[LEVELROLL].I), -_levelLimit, _levelLimit);
    #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
      digitalWrite(LED2PIN, HIGH);
    #endif
    #ifdef APM_OP_CHR
      digitalWrite(LED_Green, HIGH);
    #endif
  }
  _motors->setMotorAxisCommand(ROLL, updatePID(_receiver->getData(ROLL) + _levelAdjust[ROLL], _gyro->getFlightData(ROLL) + 1500, &PID[LEVELGYROROLL]) + PID[LEVELROLL].integratedError);
  _motors->setMotorAxisCommand(PITCH, updatePID(_receiver->getData(PITCH) + _levelAdjust[PITCH], _gyro->getFlightData(PITCH) + 1500, &PID[LEVELGYROPITCH]) + PID[LEVELPITCH].integratedError);
}


//////////////////////////////////////////////////////////////////////////////
/////////////////////////// calculateFlightError /////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void calculateFlightError(void)
{
  if (_flightMode == ACRO) 
  {
    // Acrobatic Mode
    // updatePID(target, measured, PIDsettings);
    // measured = rate data from gyros scaled to PWM (1000-2000), since PID settings are found experimentally
    // updatePID() is defined in PID.h
    _motors->setMotorAxisCommand(ROLL, updatePID(_receiver->getData(ROLL), _gyro->getFlightData(ROLL) + 1500, &PID[ROLL]));
    _motors->setMotorAxisCommand(PITCH, updatePID(_receiver->getData(PITCH), _gyro->getFlightData(PITCH) + 1500, &PID[PITCH]));
    zeroIntegralError();
  }
  else 
  {
    processStableMode();
  }
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processCalibrateESC //////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processCalibrateESC(void)
{
  switch (_calibrateESC) // used for calibrating ESC's
  { 
  case 1:
    for (byte motor = FRONT; motor < LASTMOTOR; motor++)
      _motors->setMotorCommand(motor, MAXCOMMAND);
    break;
  case 3:
    for (byte motor = FRONT; motor < LASTMOTOR; motor++)
      _motors->setMotorCommand(motor, constrain(_testCommand, 1000, 1200));
    break;
  case 5:
    for (byte motor = FRONT; motor < LASTMOTOR; motor++)
      _motors->setMotorCommand(motor, constrain(_motors->getRemoteCommand(motor), 1000, 1200));
    _safetyCheck = ON;
    break;
  default:
    for (byte motor = FRONT; motor < LASTMOTOR; motor++)
      _motors->setMotorCommand(motor, MINCOMMAND);
  }
  // Send calibration commands to motors
  _motors->write(); // Defined in Motors.h
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processHeadingHold ///////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processHeading(void)
{
  if (_headingHoldConfig == ON) 
  {
    //gyro.calculateHeading();

#if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    _heading = _compass->getHeading();
#else
    _heading = _gyro->getHeading();
#endif

    // Always center relative heading around absolute heading chosen during yaw command
    // This assumes that an incorrect yaw can't be forced on the AeroQuad >180 or <-180 degrees
    // This is done so that AeroQuad does not accidentally hit transition between 0 and 360 or -180 and 180
    _relativeHeading = _heading - _setHeading;
    if (_heading <= (_setHeading - 180)) 
    {
      _relativeHeading += 360;
    }
    if (_heading >= (_setHeading + 180)) 
    {
      _relativeHeading -= 360;
    }

    // Apply heading hold only when throttle high enough to start flight
    if (_receiver->getData(THROTTLE) > MINCHECK ) 
    { 
      if ((_receiver->getData(YAW) > (MIDCOMMAND + 25)) || (_receiver->getData(YAW) < (MIDCOMMAND - 25))) 
      {
        // If commanding yaw, turn off heading hold and store latest heading
        _setHeading = _heading;
        _headingHold = 0;
        PID[HEADING].integratedError = 0;
      }
      else 
      {
        // No new yaw input, calculate current heading vs. desired heading heading hold
        // Relative heading is always centered around zero
        _headingHold = updatePID(0, _relativeHeading, &PID[HEADING]);
      }
    }
    else 
    {
      // minimum throttle not reached, use off settings
      _setHeading = _heading;
      _headingHold = 0;
      PID[HEADING].integratedError = 0;
    }
  }
  _commandedYaw = constrain(_receiver->getData(YAW) + _headingHold, 1000, 2000);
  _motors->setMotorAxisCommand(YAW, updatePID(_commandedYaw, _gyro->getFlightData(YAW) + 1500, &PID[YAW]));
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processAltitudeHold //////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processAltitudeHold(void)
{
  // ****************************** Altitude Adjust *************************
  // Thanks to Honk for his work with altitude hold
  // http://aeroquad.com/showthread.php?792-Problems-with-BMP085-I2C-barometer
  // Thanks to Sherbakov for his work in Z Axis dampening
  // http://aeroquad.com/showthread.php?359-Stable-flight-logic...&p=10325&viewfull=1#post10325
#ifdef AltitudeHold
  if (_altitudeHold == ON) 
  {
    _throttleAdjust = updatePID(_holdAltitude, _altitude->getData(), &PID[ALTITUDE]);
    _zDampening = updatePID(0, _accel->getZaxis(), &PID[ZDAMPENING]); // This is stil under development - do not use (set PID=0)
    if((abs(_flightAngle->getData(ROLL)) > 5) ||  (abs(_flightAngle->getData(PITCH)) > 5)) 
    { 
      PID[ZDAMPENING].integratedError = 0; 
    }
    _throttleAdjust = constrain((_holdAltitude - _altitude->getData()) * PID[ALTITUDE].P, _minThrottleAdjust, _maxThrottleAdjust);
    if (_receiver->getData(THROTTLE) > MAXCHECK) //above 1900
    {
      _holdAltitude += 0.1;
    }
    if (_receiver->getData(THROTTLE) <= MINCHECK) //below 1100
    {
      _holdAltitude -= 0.1;
    }
  }
  else 
  {
    // Altitude hold is off, get throttle from receiver
    _holdThrottle = _receiver->getData(THROTTLE);
    _throttleAdjust = _autoDescent; // autoDescent is lowered from BatteryMonitor.h during battery alarm
  }
  // holdThrottle set in FlightCommand.pde if altitude hold is on
  _throttle = _holdThrottle + _throttleAdjust; // holdThrottle is also adjust by BatteryMonitor.h during battery alarm
#else
  //zDampening = updatePID(0, accel.getZaxis(), &PID[ZDAMPENING]); // This is stil under development - do not use (set PID=0)
  //throttle = _receiver->getData(THROTTLE) - zDampening + autoDescent; 
  // If altitude hold not enabled in AeroQuad.pde, get throttle from receiver
  _throttle = _receiver->getData(THROTTLE) + _autoDescent; //autoDescent is lowered from BatteryMonitor.h while battery critical, otherwise kept 0
#endif
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processMinMaxMotorCommand ////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processMinMaxMotorCommand(void)
{
  // Prevents too little power applied to motors during hard manuevers
  // Also provides even motor power on both sides if limit encountered
  if ((_motors->getMotorCommand(FRONT) <= MINTHROTTLE) || (_motors->getMotorCommand(REAR) <= MINTHROTTLE))
  {
    _delta = _receiver->getData(THROTTLE) - MINTHROTTLE;
    _motors->setMaxCommand(RIGHT, constrain(_receiver->getData(THROTTLE) + _delta, MINTHROTTLE, MAXCHECK));
    _motors->setMaxCommand(LEFT, constrain(_receiver->getData(THROTTLE) + _delta, MINTHROTTLE, MAXCHECK));
  }
  else if ((_motors->getMotorCommand(FRONT) >= MAXCOMMAND) || (_motors->getMotorCommand(REAR) >= MAXCOMMAND)) 
  {
    _delta = MAXCOMMAND - _receiver->getData(THROTTLE);
    _motors->setMinCommand(RIGHT, constrain(_receiver->getData(THROTTLE) - _delta, MINTHROTTLE, MAXCOMMAND));
    _motors->setMinCommand(LEFT, constrain(_receiver->getData(THROTTLE) - _delta, MINTHROTTLE, MAXCOMMAND));
  }     
  else 
  {
    _motors->setMaxCommand(RIGHT, MAXCOMMAND);
    _motors->setMaxCommand(LEFT, MAXCOMMAND);
    _motors->setMinCommand(RIGHT, MINTHROTTLE);
    _motors->setMinCommand(LEFT, MINTHROTTLE);
  }

  if ((_motors->getMotorCommand(LEFT) <= MINTHROTTLE) || (_motors->getMotorCommand(RIGHT) <= MINTHROTTLE))
  {
    _delta = _receiver->getData(THROTTLE) - MINTHROTTLE;
    _motors->setMaxCommand(FRONT, constrain(_receiver->getData(THROTTLE) + _delta, MINTHROTTLE, MAXCHECK));
    _motors->setMaxCommand(REAR, constrain(_receiver->getData(THROTTLE) + _delta, MINTHROTTLE, MAXCHECK));
  }
  else if ((_motors->getMotorCommand(LEFT) >= MAXCOMMAND) || (_motors->getMotorCommand(RIGHT) >= MAXCOMMAND)) 
  {
    _delta = MAXCOMMAND - _receiver->getData(THROTTLE);
    _motors->setMinCommand(FRONT, constrain(_receiver->getData(THROTTLE) - _delta, MINTHROTTLE, MAXCOMMAND));
    _motors->setMinCommand(REAR, constrain(_receiver->getData(THROTTLE) - _delta, MINTHROTTLE, MAXCOMMAND));
  }     
  else 
  {
    _motors->setMaxCommand(FRONT, MAXCOMMAND);
    _motors->setMaxCommand(REAR, MAXCOMMAND);
    _motors->setMinCommand(FRONT, MINTHROTTLE);
    _motors->setMinCommand(REAR, MINTHROTTLE);
  }
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////// processHardManuevers ////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processHardManuevers()
{
  if (_receiver->getRaw(ROLL) < MINCHECK) 
  {
    _motors->setMaxCommand(FRONT, _minAcro);
    _motors->setMaxCommand(REAR, MAXCOMMAND);
    _motors->setMaxCommand(LEFT, _minAcro);
    _motors->setMaxCommand(RIGHT, MAXCOMMAND);
  }
  else if (_receiver->getRaw(ROLL) > MAXCHECK) 
  {
    _motors->setMaxCommand(FRONT, MAXCOMMAND);
    _motors->setMaxCommand(REAR, _minAcro);
    _motors->setMaxCommand(LEFT, MAXCOMMAND);
    _motors->setMaxCommand(RIGHT, _minAcro);
  }
  else if (_receiver->getRaw(PITCH) < MINCHECK) 
  {
    _motors->setMaxCommand(FRONT, MAXCOMMAND);
    _motors->setMaxCommand(REAR, _minAcro);
    _motors->setMaxCommand(LEFT, _minAcro);
    _motors->setMaxCommand(RIGHT, MAXCOMMAND);
  }
  else if (_receiver->getRaw(PITCH) > MAXCHECK) 
  {
    _motors->setMaxCommand(FRONT, _minAcro);
    _motors->setMaxCommand(REAR, MAXCOMMAND);
    _motors->setMaxCommand(LEFT, MAXCOMMAND);
    _motors->setMaxCommand(RIGHT, _minAcro);
  }
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// X MODE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processFlightControlXMode(void) 
{
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (_armed && _safetyCheck) 
  {
    // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
    _motors->setMotorCommand(FRONT, _throttle - _motors->getMotorAxisCommand(PITCH) + _motors->getMotorAxisCommand(ROLL) - _motors->getMotorAxisCommand(YAW));
    _motors->setMotorCommand(RIGHT, _throttle - _motors->getMotorAxisCommand(PITCH) - _motors->getMotorAxisCommand(ROLL) + _motors->getMotorAxisCommand(YAW));
    _motors->setMotorCommand(LEFT, _throttle + _motors->getMotorAxisCommand(PITCH) + _motors->getMotorAxisCommand(ROLL) + _motors->getMotorAxisCommand(YAW));
    _motors->setMotorCommand(REAR, _throttle + _motors->getMotorAxisCommand(PITCH) - _motors->getMotorAxisCommand(ROLL) - _motors->getMotorAxisCommand(YAW));
#ifdef MultipilotI2C
    // if using Mixertable need only Throttle MotorAxixCommand Roll,Pitch,Yaw Yet set
    _motors->setThrottle(_receiver->getData(THROTTLE));
#endif
  } 

  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
  if (_flightMode == ACRO) 
  {
    processHardManuevers();
  }

  // Apply limits to motor commands
  for (byte motor = FRONT; motor < LASTMOTOR; motor++) 
  {
    _motors->setMotorCommand(motor, constrain(_motors->getMotorCommand(motor), _motors->getMinCommand(motor), _motors->getMaxCommand(motor)));
  }

  // If throttle in minimum position, don't apply yaw
  if (_receiver->getData(THROTTLE) < MINCHECK) 
  {
    for (byte motor = FRONT; motor < LASTMOTOR; motor++) 
    {
      _motors->setMotorCommand(motor, MINTHROTTLE);
    }
  }

  // ESC Calibration
  if (_armed == OFF) 
  {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (_armed == ON && _safetyCheck == ON) 
  {
    _motors->write(); // Defined in Motors.h
  }
}

//////////////////////////////////////////////////////////////////////////////
///////////////////////////////// PLUS MODE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processFlightControlPlusMode(void) 
{
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (_armed && _safetyCheck) 
  {
    _motors->setMotorCommand(FRONT, _throttle - _motors->getMotorAxisCommand(PITCH) - _motors->getMotorAxisCommand(YAW));
    _motors->setMotorCommand(REAR, _throttle + _motors->getMotorAxisCommand(PITCH) - _motors->getMotorAxisCommand(YAW));
    _motors->setMotorCommand(RIGHT, _throttle - _motors->getMotorAxisCommand(ROLL) + _motors->getMotorAxisCommand(YAW));
    _motors->setMotorCommand(LEFT, _throttle + _motors->getMotorAxisCommand(ROLL) + _motors->getMotorAxisCommand(YAW));
#ifdef MultipilotI2C
    // if using Mixertable need only Throttle MotorAxixCommand Roll,Pitch,Yaw Yet set
    _motors->setThrottle(_receiver->getData(THROTTLE));
#endif
  } 

  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
  if (_flightMode == ACRO) 
  {
    processHardManuevers();
  }

  // Apply limits to motor commands
  for (byte motor = FRONT; motor < LASTMOTOR; motor++) 
  {
    _motors->setMotorCommand(motor, constrain(_motors->getMotorCommand(motor), _motors->getMinCommand(motor), _motors->getMaxCommand(motor)));
  }

  // If throttle in minimum position, don't apply yaw
  if (_receiver->getData(THROTTLE) < MINCHECK) 
  {
    for (byte motor = FRONT; motor < LASTMOTOR; motor++) 
    {
      _motors->setMotorCommand(motor, MINTHROTTLE);
    }
  }

  // ESC Calibration
  if (_armed == OFF) 
  {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (_armed == ON && _safetyCheck == ON) 
  {
    _motors->write(); // Defined in Motors.h
  }
}


