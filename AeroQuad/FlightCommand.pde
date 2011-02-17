/*
  AeroQuad v2.1 - January 2011
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
  receiver.read();
  // Read quad configuration commands from transmitter when throttle down
  if (receiver.getRaw(THROTTLE) < MINCHECK) {
    zeroIntegralError();
    throttleAdjust = 0;
    //receiver.adjustThrottle(throttleAdjust);
    // Disarm motors (left stick lower left corner)
    if (receiver.getRaw(YAW) < MINCHECK && armed == ON) {
      armed = OFF;
      motors.commandAllMotors(MINCOMMAND);
      #if defined(APM_OP_CHR6DM) || defined(APM) 
      digitalWrite(LED_Red, LOW);
      #endif
    }    
    // Zero Gyro and Accel sensors (left stick lower left, right stick lower right corner)
    if ((receiver.getRaw(YAW) < MINCHECK) && (receiver.getRaw(ROLL) > MAXCHECK) && (receiver.getRaw(PITCH) < MINCHECK)) {
      rateGyro.calibrate(); // defined in Gyro.h
      accel.calibrate(); // defined in Accel.h
      //accel.setOneG(accel.getFlightData(ZAXIS));
       #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      flightAngle.calibrate();
       #endif
      zeroIntegralError();
      #ifndef BatteryMonitor
      motors.pulseMotors(3);
      #endif
      #ifdef BatteryMonitor
      ledCW(); ledCW(); ledCW();
      #endif     
      rateGyro.zero();
      accel.zero();
    }   
    // Multipilot Zero Gyro sensors (left stick no throttle, right stick upper right corner)
    if ((receiver.getRaw(ROLL) > MAXCHECK) && (receiver.getRaw(PITCH) > MAXCHECK)) {
      accel.calibrate(); // defined in Accel.h
      zeroIntegralError();
      motors.pulseMotors(3);
      rateGyro.zero();
      accel.zero();
    }   
    // Multipilot Zero Gyros (left stick no throttle, right stick upper left corner)
    if ((receiver.getRaw(ROLL) < MINCHECK) && (receiver.getRaw(PITCH) > MAXCHECK)) {
      rateGyro.calibrate();
      zeroIntegralError();
      motors.pulseMotors(4);
      rateGyro.zero();
      accel.zero();
    }
    // Arm motors (left stick lower right corner)
    if (receiver.getRaw(YAW) > MAXCHECK && armed == OFF && safetyCheck == ON) {
      zeroIntegralError();
      armed = ON;
      #if defined(APM_OP_CHR6DM) || defined(APM) 
      digitalWrite(LED_Red, HIGH);
      #endif
      for (byte motor = FRONT; motor < LASTMOTOR; motor++)
        motors.setMinCommand(motor, MINTHROTTLE);
      //   delay(100);
      //altitude.measureGround();
    }
    // Prevents accidental arming of motor output if no transmitter command received
    if (receiver.getRaw(YAW) > MINCHECK) safetyCheck = ON; 
  }
  
  // Get center value of roll/pitch/yaw channels when enough throttle to lift off
  if (receiver.getRaw(THROTTLE) < 1300) {
    receiver.setTransmitterTrim(ROLL, receiver.getRaw(ROLL));
    receiver.setTransmitterTrim(PITCH, receiver.getRaw(PITCH));
    receiver.setTransmitterTrim(YAW, receiver.getRaw(YAW));
  }
  
  // Check Mode switch for Acro or Stable
  if (receiver.getRaw(MODE) > 1500) {
    #if defined(AEROQUAD_V18) || defined(AEROQUAD_MEGA_V2)
      if (flightMode == ACRO)
        digitalWrite(LED2_PIN, HIGH);
    #endif
    flightMode = STABLE;
 }
  else {
    #if defined(AEROQUAD_V18) || defined(AEROQUAD_MEGA_V2)
      if (flightMode == STABLE)
        digitalWrite(LED2_PIN, LOW);
    #endif
    flightMode = ACRO;
  }
  
   #if defined(APM_OP_CHR6DM) || defined(APM) 
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
    
   if (receiver.getRaw(AUX) < 1750) {
      if (storeAltitude == ON) {
        holdAltitude = altitude.getData();
        holdThrottle = receiver.getData(THROTTLE);
        PID[ALTITUDE].integratedError = 0;
        accel.setOneG(accel.getFlightData(ZAXIS));
        storeAltitude = OFF;
      }
      altitudeHold = ON;
    }
    else {
      storeAltitude = ON;
      altitudeHold = OFF;
    }
  #endif
  
  // Use for correcting gyro drift with v2.0 Shield
  //gyro.setReceiverYaw(receiver.getData(YAW));
}




