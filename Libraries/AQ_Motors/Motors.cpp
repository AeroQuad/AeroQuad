/*
  AeroQuad v3.0 - April 2011
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


#include "Motors.h"


Motors::Motors() {
}

void Motors::pulseMotors(byte nbPulse) {
  for (byte i = 0; i < nbPulse; i++) {
    commandAllMotors(MINCOMMAND + 100);
    delay(250);
    commandAllMotors(MINCOMMAND);
    delay(250);
  }
}

void Motors::setMinCommand(byte motor, int command) {
  minCommand[motor] = command;
}

int Motors::getMinCommand(byte motor) {
  return minCommand[motor];
}

void Motors::setMaxCommand(byte motor, int command) {
  maxCommand[motor] = command;
}

int Motors::getMaxCommand(byte motor) {
  return maxCommand[motor];
}

void Motors::setMotorCommand(byte motor, int command) {
  motorCommand[motor] = command;
}

int Motors::getMotorCommand(byte motor) {
  return motorCommand[motor];
}











