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


#ifndef _AEROQUAD_MOTORS_APM_H_
#define _AEROQUAD_MOTORS_APM_H_

#include "Arduino.h"
#include <APM_RC.h>
#include "Motors.h"

void initializeMotors(NB_Motors numbers) {
  commandAllMotors(1000);
}

void writeMotors() {
  writeMotorCommand(0,motorCommand[MOTOR1]);
  writeMotorCommand(1,motorCommand[MOTOR2]);
  writeMotorCommand(2,motorCommand[MOTOR3]);
  writeMotorCommand(3,motorCommand[MOTOR4]);
  writeMotorCommand(6,motorCommand[MOTOR5]);
  writeMotorCommand(7,motorCommand[MOTOR6]);
  writeMotorCommand(9,motorCommand[MOTOR7]);
  writeMotorCommand(10,motorCommand[MOTOR8]);
  force_Out0_Out1();
  force_Out2_Out3();
  force_Out6_Out7();
}

void commandAllMotors(int command) {
  writeMotorCommand(0,command);
  writeMotorCommand(1,command);
  writeMotorCommand(2,command);
  writeMotorCommand(3,command);
  writeMotorCommand(6,command);
  writeMotorCommand(6,command);
  writeMotorCommand(7,command);
  writeMotorCommand(9,command);
  writeMotorCommand(10,command);
  force_Out0_Out1();
  force_Out2_Out3();
  force_Out6_Out7();
}
  


#endif