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

// FlightCommand.pde is responsible for decoding transmitter stick combinations
// for setting up AeroQuad modes such as motor arming and disarming

void readPilotCommands() {
  _receiver->read();
  // Read quad configuration commands from transmitter when throttle down
  if (_receiver->getRaw(THROTTLE) < MINCHECK) {
    zeroIntegralError();
    _throttleAdjust = 0;
    //_receiver->adjustThrottle(throttleAdjust);
    // Disarm motors (left stick lower left corner)
    if (_receiver->getRaw(YAW) < MINCHECK && _armed == ON) {
      _armed = OFF;
      _motors->commandAllMotors(MINCOMMAND);
      #if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
      digitalWrite(LED_Red, LOW);
      #endif
    }    
    // Zero Gyro and Accel sensors (left stick lower left, right stick lower right corner)
    if ((_receiver->getRaw(YAW) < MINCHECK) && (_receiver->getRaw(ROLL) > MAXCHECK) && (_receiver->getRaw(PITCH) < MINCHECK)) {
      _gyro->calibrate(); // defined in Gyro.h
      _accel->calibrate(); // defined in Accel.h
      storeSensorsToEEPROM();
      //accel.setOneG(accel.getFlightData(ZAXIS));
      #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
        _flightAngle->calibrate();
      #endif
      zeroIntegralError();
      _motors->pulseMotors(3);
      // ledCW() is currently a private method in BatteryMonitor.h, fix and agree on this behavior in next revision
      //#if defined(BattMonitor) && defined(ArduCopter)
      //  ledCW(); ledCW(); ledCW();
      //#endif
      #ifdef ArduCopter
        zero_ArduCopter_ADC();
      #endif
    }   
    // Multipilot Zero Gyro sensors (left stick no throttle, right stick upper right corner)
    if ((_receiver->getRaw(ROLL) > MAXCHECK) && (_receiver->getRaw(PITCH) > MAXCHECK)) {
      _accel->calibrate(); // defined in Accel.h
      storeSensorsToEEPROM();
      zeroIntegralError();
      _motors->pulseMotors(3);
      #ifdef ArduCopter
        zero_ArduCopter_ADC();
      #endif
    }   
    // Multipilot Zero Gyros (left stick no throttle, right stick upper left corner)
    if ((_receiver->getRaw(ROLL) < MINCHECK) && (_receiver->getRaw(PITCH) > MAXCHECK)) {
      _gyro->calibrate();
      zeroIntegralError();
      _motors->pulseMotors(4);
      #ifdef ArduCopter
        zero_ArduCopter_ADC();
      #endif
    }
    // Arm motors (left stick lower right corner)
    if (_receiver->getRaw(YAW) > MAXCHECK && _armed == OFF && _safetyCheck == ON) {
      zeroIntegralError();
      _armed = ON;
      #if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
      digitalWrite(LED_Red, HIGH);
      #endif
      for (byte motor = FRONT; motor < LASTMOTOR; motor++)
        _motors->setMinCommand(motor, MINTHROTTLE);
      //   delay(100);
      //_altitude->measureGround();
    }
    // Prevents accidental arming of motor output if no transmitter command received
    if (_receiver->getRaw(YAW) > MINCHECK) 
    {
      _safetyCheck = ON; 
    }
  }
  
  // Get center value of roll/pitch/yaw channels when enough throttle to lift off
  if (_receiver->getRaw(THROTTLE) < 1300) {
    _receiver->setTransmitterTrim(ROLL, _receiver->getRaw(ROLL));
    _receiver->setTransmitterTrim(PITCH, _receiver->getRaw(PITCH));
    _receiver->setTransmitterTrim(YAW, _receiver->getRaw(YAW));
  }
  
  // Check Mode switch for Acro or Stable
  if (_receiver->getRaw(MODE) > 1500) {
    #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
      if (_flightMode == ACRO)
      {
        digitalWrite(LED2PIN, HIGH);
      }
    #endif
    _flightMode = STABLE;
 }
  else {
    #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
      if (_flightMode == STABLE)
      {
        digitalWrite(LED2PIN, LOW);
      }
    #endif
    _flightMode = ACRO;
  }
  
   #if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
      if (_flightMode == ACRO)
      {
        digitalWrite(LED_Yellow, HIGH);
        digitalWrite(LED_Green, LOW);
      }
      else if (_flightMode == STABLE) 
      {
        digitalWrite(LED_Green, HIGH);
        digitalWrite(LED_Yellow, LOW); 
      }
   #endif
  
  #ifdef AltitudeHold
    
   if (_receiver->getRaw(AUX) < 1750) 
   {
      if (_storeAltitude == ON) 
      {
        _holdAltitude = _altitude->getData();
        _holdThrottle = _receiver->getData(THROTTLE);
        PID[ALTITUDE].integratedError = 0;
        _accel->setOneG(_accel->getFlightData(ZAXIS));
        _storeAltitude = OFF;
      }
      _altitudeHold = ON;
    }
    else 
    {
      _storeAltitude = ON;
      _altitudeHold = OFF;
    }
  #endif
  
  // Use for correcting gyro drift with v2.0 Shield
  //gyro.setReceiverYaw(_receiver->getData(YAW));
}




