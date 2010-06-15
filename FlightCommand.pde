/*
  AeroQuad v1.8 - June 2010
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

void readPilotCommands() {
  // Buffer receiver values read from pin change interrupt handler
  for (channel = ROLL; channel < LASTCHANNEL; channel++)
    receiverData[channel] = (mTransmitter[channel] * readReceiver(receiverPin[channel])) + bTransmitter[channel];
  // Smooth the flight control transmitter inputs (roll, pitch, yaw, throttle)
  for (channel = ROLL; channel < LASTCHANNEL; channel++)
    transmitterCommandSmooth[channel] = smooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
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
      gyro.calibrate();
      accel.calibrate(); // defined in Accel.h
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
}


