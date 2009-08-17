/*
  AeroQuad v1.2 - August 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Open Source Arduino based quadrocopter.
 
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
  case 'B': // Send roll and pitch gyro PID values
    Serial.print(PID[ROLL].P);
    comma();
    Serial.print(PID[ROLL].I);
    comma();
    Serial.print(PID[ROLL].D);
    comma();
    Serial.print(PID[PITCH].P);
    comma();
    Serial.print(PID[PITCH].I);
    comma();
    Serial.println(PID[PITCH].D);
    queryType = 'X';
    break;
  case 'D': // Send yaw PID values
    Serial.print(PID[YAW].P);
    comma();
    Serial.print(PID[YAW].I);
    comma();
    Serial.println(PID[YAW].D);
    queryType = 'X';
    break;
  case 'F': // Send roll and pitch auto level PID values
    Serial.print(PID[LEVELROLL].P);
    comma();
    Serial.print(PID[LEVELROLL].I);
    comma();
    Serial.print(PID[LEVELROLL].D);
    comma();
    Serial.print(PID[LEVELPITCH].P);
    comma();
    Serial.print(PID[LEVELPITCH].I);
    comma();
    Serial.println(PID[LEVELPITCH].D);
    queryType = 'X';
    break;
  case 'H': // Send auto level configuration values
    Serial.print(levelLimit);
    comma();
    Serial.println(levelOff);
    queryType = 'X';
    break;
  case 'J': // Send flight control configuration values
    Serial.print(windupGuard);
    comma();
    Serial.println(xmitFactor);
    queryType = 'X';
    break;
  case 'L': // Send data filtering values
    Serial.print(smoothFactor[GYRO]);
    comma();
    Serial.print(smoothFactor[ACCEL]);
    comma();
    Serial.println(timeConstant);
    queryType = 'X';
    break;
  case 'N': // Send motor smoothing values
    for (axis= ROLL; axis < LASTAXIS; axis++) {
      Serial.print(smoothTransmitter[axis]);
      comma();
    }
    Serial.println(smoothTransmitter[THROTTLE]);
    queryType = 'X';
    break;
  case 'Q': // Send sensor data
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
  case 'R': // Send raw sensor data
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
  case 'S': // Send all flight data
    Serial.print(deltaTime);
    comma();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(gyroData[axis]);
      comma();
    }
    Serial.print(transmitterCommand[THROTTLE]);
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
    #ifdef AutoLevel
      Serial.println(receiverData[MODE]);
    #endif
    #ifndef AutoLevel
      Serial.println(1000);
    #endif
    break;
   case 'T': // Send processed transmitter values
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
  case 'U': // Send receiver values
    for (channel = ROLL; channel < AUX; channel++) {
      Serial.print(receiverData[channel]);
      comma();
    }
    Serial.println(receiverData[AUX]);
    break;
  case 'V': // Send receiver status
    for (channel = ROLL; channel < AUX; channel++) {
      Serial.print(receiverData[channel]);
      comma();
    }
    Serial.println(receiverData[AUX]);
    break;
  case 'X': // Stop sending messages
    break;
  }
}

void comma() {
  Serial.print(',');
}
