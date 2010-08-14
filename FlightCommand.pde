/*
  AeroQuad v2.0 - July 2010
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
    // Disarm motors (left stick lower left corner)
    if (receiver.getRaw(YAW) < MINCHECK && armed == 1) {
      armed = 0;
      motors.commandAllMotors(MINCOMMAND);
    }    
    // Zero Gyro and Accel sensors (left stick lower left, right stick lower right corner)
    if ((receiver.getRaw(YAW) < MINCHECK) && (receiver.getRaw(ROLL) > MAXCHECK) && (receiver.getRaw(PITCH) < MINCHECK)) {
      gyro.calibrate();
      accel.calibrate(); // defined in Accel.h
      zeroIntegralError();
      motors.pulseMotors(3);
    }   
    // Multipilot Zero Gyro sensors (left stick no throttle, right stick upper right corner)
    if ((receiver.getRaw(ROLL) > MAXCHECK) && (receiver.getRaw(PITCH) > MAXCHECK)) {
      accel.calibrate(); // defined in Accel.h
      zeroIntegralError();
      motors.pulseMotors(3);
      #ifdef TELEMETRY_DEBUG
        Serial.println("ZeroG Accel");
      #endif
    }   
    // Multipilot Zero Gyros (left stick no throttle, right stick upper left corner)
    if ((receiver.getRaw(ROLL) < MINCHECK) && (receiver.getRaw(PITCH) > MAXCHECK)) {
      gyro.calibrate();
      zeroIntegralError();
      motors.pulseMotors(4);
      #ifdef TELEMETRY_DEBUG
        Serial.println("ZeroG Gyro");
      #endif 
  }
    // Arm motors (left stick lower right corner)
    if (receiver.getRaw(YAW) > MAXCHECK && armed == 0 && safetyCheck == 1) {
      armed = 1;
      zeroIntegralError();
      for (motor=FRONT; motor < LASTMOTOR; motor++)
        motors.setMinCommand(motor, MINTHROTTLE);
    }
    // Prevents accidental arming of motor output if no transmitter command received
    if (receiver.getRaw(YAW) > MINCHECK) safetyCheck = 1; 
  }
  
  if (receiver.getRaw(MODE) > 1500)
    flightMode = STABLE;
  else
    flightMode = ACRO;
}


