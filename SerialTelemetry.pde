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

void sendSerialTelemetry() {
  update = 0;
  switch (queryType) {
  case 'X': // send no debug messages
    break;
  case 'A': // send all data
    Serial.print(deltaTime);
    comma();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(gyroData[axis]);
      comma();
    }
    Serial.print(transmitterData[THROTTLE]);
    comma();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(motorAxisCommand[axis]);
      comma();
    }
    for (motor = FRONT; motor < LASTMOTOR; motor++) {
      Serial.print(motorCommand[motor]);
      comma();
    }
    Serial.print(armed, BIN);
    comma();
    Serial.println(transmitterData[MODE]);
    break;
  case 'S': // send sensor data
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(gyroADC[axis]);
      comma();
    }
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(accelADC[axis]);
      comma();
    }
    for (axis = ROLL; axis < YAW; axis++) {
      Serial.print(levelAdjust[axis]);
      comma();
    }
    Serial.print(flightAngle[ROLL]);
    comma();
    Serial.print(flightAngle[PITCH]);
    Serial.println();
    break;
  case 'D': // raw read sensor data
    Serial.print(analogRead(ROLLRATEPIN));
    comma();
    Serial.print(analogRead(PITCHRATEPIN));
    comma();
    Serial.print(analogRead(YAWRATEPIN));
    comma();
    Serial.print(analogRead(ROLLACCELPIN));
    comma();
    Serial.print(analogRead(PITCHACCELPIN));
    comma();
    Serial.println(analogRead(ZACCELPIN));
    break;
  case 'U': // send user defined values
    //Serial.print(dtostrf(PID[ROLL].P, 1, 2, string));
    Serial.print(PID[ROLL].P);
    comma();
    Serial.print(PID[ROLL].I);
    comma();
    Serial.print(PID[ROLL].D);
    comma();
    Serial.print(PID[LEVELROLL].P);
    comma();
    Serial.print(PID[LEVELROLL].I);
    comma();
    Serial.print(PID[LEVELROLL].D);
    comma();
    Serial.print(PID[YAW].P);
    comma();
    Serial.print(PID[YAW].I);
    comma();
    Serial.print(PID[YAW].D);
    comma();
    Serial.print(windupGuard);
    comma();
    Serial.print(levelLimit);
    comma();
    Serial.print(levelInterval);
    comma();
    Serial.print(xmitFactor);
    comma();
    Serial.print(smoothFactor[GYRO]);
    comma();
    Serial.print(smoothFactor[ACCEL]);
    comma();
    Serial.print(timeConstant);
    Serial.println(); // will probably add more responses in the future
    queryType = 'X';
    break;
   case 'T': // read processed transmitter values
    Serial.print(xmitFactor);
    comma();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(transmitterCommand[axis]);
      comma();
    }
    for (axis = ROLL; axis < YAW; axis++) {
      Serial.print(levelAdjust[axis]);
      comma();
    }
    Serial.print(motorAxisCommand[ROLL]);
    comma();
    Serial.print(motorAxisCommand[PITCH]);
    comma();
    Serial.println(motorAxisCommand[YAW]);
    break;
  case 'R': // send receiver values
    for (channel = ROLL; channel < AUX; channel++) {
      Serial.print(transmitterData[channel]);
      comma();
    }
    Serial.println(transmitterData[AUX]);
    break;
  /*case 'N': // send receiver channel order
    for (channel = ROLL; channel < LASTCHANNEL; channel++) {
      Serial.print(orderCh[channel]);
      comma();
    }
    for (channel = ROLL; channel < AUX; channel++) {
      Serial.print(xmitCh[channel]);
      comma();
    }
    Serial.println(xmitCh[AUX]);
    queryType = 'X';
    break;*/
  }
}

void comma() {
  Serial.print(',');
}
