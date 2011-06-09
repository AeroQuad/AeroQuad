/*
  AeroQuad v2.4 - April 2011
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
  receiver->read();
  // Read quad configuration commands from transmitter when throttle down
  if (receiver->getData(THROTTLE) < MINCHECK) {
    zeroIntegralError();
    throttleAdjust = 0;
    //receiver->adjustThrottle(throttleAdjust);
    // Disarm motors (left stick lower left corner)
    if (receiver->getData(YAW) < MINCHECK && armed == ON) {
      armed = OFF;
      motors->commandAllMotors(MINCOMMAND);
      #if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
      digitalWrite(LED_Red, LOW);
      #endif
    }    
    // Zero Gyro and Accel sensors (left stick lower left, right stick lower right corner)
    if ((receiver->getData(YAW) < MINCHECK) && (receiver->getData(ROLL) > MAXCHECK) && (receiver->getData(PITCH) < MINCHECK)) {
      gyro->calibrate(); // defined in Gyro.h
      accel->calibrate(); // defined in Accel.h
      storeSensorsZeroToEEPROM();
      //accel.setOneG(accel.getFlightData(ZAXIS));
      #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
        _flightAngle->calibrate();
      #endif
      zeroIntegralError();
      motors->pulseMotors(3);
      // ledCW() is currently a private method in BatteryMonitor.h, fix and agree on this behavior in next revision
      //#if defined(BattMonitor) && defined(ArduCopter)
      //  ledCW(); ledCW(); ledCW();
      //#endif
    }   
    // Arm motors (left stick lower right corner)
    if (receiver->getData(YAW) > MAXCHECK && armed == OFF && safetyCheck == ON) {
      zeroIntegralError();
      armed = ON;
      #if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
      digitalWrite(LED_Red, HIGH);
      #endif
      for (byte motor = 0; motor < LASTMOTOR; motor++) {
        motorMinCommand[motor] = MINTHROTTLE;
      }
      //   delay(100);
      //altitude.measureGround();
    }
    // Prevents accidental arming of motor output if no transmitter command received
    if (receiver->getData(YAW) > MINCHECK) safetyCheck = ON; 
  }
  
  // Get center value of roll/pitch/yaw channels when enough throttle to lift off
  if (receiver->getData(THROTTLE) < 1300) {
    receiver->setTransmitterTrim(ROLL, receiver->getData(ROLL));
    receiver->setTransmitterTrim(PITCH, receiver->getData(PITCH));
    receiver->setTransmitterTrim(YAW, receiver->getData(YAW));
    //receiver->setZero(ROLL, receiver->getData(ROLL));
    //receiver->setZero(PITCH, receiver->getData(PITCH));
    //receiver->setZero(YAW, receiver->getData(YAW));
  }
  
  // Check Mode switch for Acro or Stable
  if (receiver->getData(MODE) > 1500) {
    if (flightMode == ACRO) {
      #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
        digitalWrite(LED2PIN, HIGH);
      #endif
      zeroIntegralError();
    }
    flightMode = STABLE;
 }
  else {
    #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
      if (flightMode == STABLE)
        digitalWrite(LED2PIN, LOW);
    #endif
    flightMode = ACRO;
  }
  
   #if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
      if (flightMode == ACRO) {
        digitalWrite(LED_Yellow, HIGH);
        digitalWrite(LED_Green, LOW);
       }
     else if (flightMode == STABLE) {
        digitalWrite(LED_Green, HIGH);
        digitalWrite(LED_Yellow, LOW); 
     }
   #endif
  
  #ifdef AltitudeHold
   if (receiver->getData(AUX) < 1750) {
     if (altitudeHold != ALTPANIC ) {  // check for special condition with manditory override of Altitude hold
       if (storeAltitude == ON) {
         holdAltitude = barometricSensor->getAltitude();
         holdThrottle = receiver->getData(THROTTLE);
         PID[ALTITUDE].integratedError = 0;
         PID[ALTITUDE].lastPosition = holdAltitude;  // add to initialize hold position on switch turn on.
         //accel.setOneG(accel.getFlightData(ZAXIS));  // AKA need to fix this
         storeAltitude = OFF;
       }
       altitudeHold = ON;
     }
     // note, Panic will stay set until Althold is toggled off/on
   } 
   else {
     storeAltitude = ON;
     altitudeHold = OFF;
   }
  #endif
}




