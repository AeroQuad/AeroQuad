/*
  MikroQuad v3.0 - March 2009
  www.MikroQuad.com
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

// Interrupts are generated every 20ms because
// Roll pin from transmitter triggers interrupt (50MHz, PWM)
// Modulo division by 5 causes each time slot to happen every 100ms

void configureTransmitter() {
  attachInterrupt(0, readTransmitter, RISING);
}

void readTransmitter() {
  if (((timeSlot += 1) % 5) == 0) {
    // Read Receiver
    // This has been tested with AR6100 and AR6200 Spektrum receivers
    for (channel = 0; channel < 6; channel++)
      transmitterData[orderCh[channel]] = pulseIn(xmitCh[channel], HIGH, TIMEOUT);
    // Calculate Commanded Angles
    // Reduce transmitter commands using xmitFactor and center around 1500
    // rollCommand, pitchCommand and yawCommand used in main loop of MikroQuad.pde to control quad
    for (axis = ROLL; axis < LASTAXIS; axis++)
      transmitterCommand[axis] = ((transmitterData[axis] - transmitterZero[axis]) * xmitFactor) + transmitterZero[axis];
    transmitterCommand[THROTTLE] = transmitterData[THROTTLE];
  }
  if ((timeSlot % 5) == 2) update = 1;
}
