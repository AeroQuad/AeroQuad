/*
  AeroQuad v2.1 - October 2010
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
    }    
    // Zero Gyro and Accel sensors (left stick lower left, right stick lower right corner)
    if ((receiver.getRaw(YAW) < MINCHECK) && (receiver.getRaw(ROLL) > MAXCHECK) && (receiver.getRaw(PITCH) < MINCHECK)) {
      gyro.calibrate(); // defined in Gyro.h
      accel.calibrate(); // defined in Accel.h
      zeroIntegralError();
      motors.pulseMotors(3);
      #ifdef ArduCopter
        zero_ArduCopter_ADC();
      #endif
    }   
    // Multipilot Zero Gyro sensors (left stick no throttle, right stick upper right corner)
    if ((receiver.getRaw(ROLL) > MAXCHECK) && (receiver.getRaw(PITCH) > MAXCHECK)) {
      accel.calibrate(); // defined in Accel.h
      zeroIntegralError();
      motors.pulseMotors(3);
      #ifdef ArduCopter
        zero_ArduCopter_ADC();
      #endif
    }   
    // Multipilot Zero Gyros (left stick no throttle, right stick upper left corner)
    if ((receiver.getRaw(ROLL) < MINCHECK) && (receiver.getRaw(PITCH) > MAXCHECK)) {
      gyro.calibrate();
      zeroIntegralError();
      motors.pulseMotors(4);
      #ifdef ArduCopter
        zero_ArduCopter_ADC();
      #endif
    }
    // Arm motors (left stick lower right corner)
    if (receiver.getRaw(YAW) > MAXCHECK && armed == OFF && safetyCheck == ON) {
      zeroIntegralError();
      armed = ON;
      for (motor=FRONT; motor < LASTMOTOR; motor++)
        motors.setMinCommand(motor, MINTHROTTLE);
      delay(50);
      accel.setOneG(accel.getData(ZAXIS));
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
    #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
      if (flightMode == ACRO)
        digitalWrite(LED2PIN, HIGH);
    #endif
    flightMode = STABLE;
 }
  else {
    #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
      if (flightMode == STABLE)
        digitalWrite(LED2PIN, LOW);
    #endif
    flightMode = ACRO;
  }
  
  #ifdef AltitudeHold
    // Check if altitude hold is enabled
    if (receiver.getRaw(AUX) < 1400) {
      // return to preset altitude or land?
    }
    else if (receiver.getRaw(AUX) < 1700) {
      if (storeAltitude == ON) {
        holdAltitude = altitude.getData();
        PID[ALTITUDE].integratedError = 0;
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
  gyro.setReceiverYaw(receiver.getData(YAW));
}


