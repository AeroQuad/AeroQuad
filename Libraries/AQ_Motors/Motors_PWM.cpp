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



#include "Motors_PWM.h"

Motors_PWM::Motors_PWM() {
}

void Motors_PWM::initialize() {
  commandAllMotors(1000);
}

void Motors_PWM::write() {
  analogWrite(FRONTMOTORPIN, motorCommand[FRONT] / 8);
  analogWrite(REARMOTORPIN,  motorCommand[REAR]  / 8);
  analogWrite(RIGHTMOTORPIN, motorCommand[RIGHT] / 8);
  analogWrite(LEFTMOTORPIN,  motorCommand[LEFT]  / 8);
}

void Motors_PWM::commandAllMotors(int command) {
  analogWrite(FRONTMOTORPIN, command / 8);
  analogWrite(REARMOTORPIN,  command / 8);
  analogWrite(RIGHTMOTORPIN, command / 8);
  analogWrite(LEFTMOTORPIN,  command / 8);
}

