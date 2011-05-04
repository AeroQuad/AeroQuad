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


#ifndef _AEROQUAD_MOTORS_H_
#define _AEROQUAD_MOTORS_H_

#include <WProgram.h>


#define FRONT 0
#define REAR 1
#define RIGHT 2
#define LEFT 3
#define RIGHT2 4
#define LEFT2 5
#define MOTORID1 0		
#define MOTORID2 1		
#define MOTORID3 2		
#define MOTORID4 3		
#define MOTORID5 4		
#define MOTORID6 5
#define MINCOMMAND 1000
#define MAXCOMMAND 2000

#define LASTMOTOR 4  // 6 or 8 motors are not supported yet

enum NB_Motors{
  FOUR_Motors,
  SIX_Motors,
  HEIGHT_Motors
};


class Motors {
private:
  int minCommand[LASTMOTOR];
  int maxCommand[LASTMOTOR];

protected:
  int motorCommand[LASTMOTOR];
  
public:

  Motors();
	
  virtual void initialize(NB_Motors numbers = FOUR_Motors) {}
  virtual void write() {}
  virtual void commandAllMotors(int command) {}
  
  void pulseMotors(byte nbPulse);
  void setMinCommand(byte motor, int command);
  int getMinCommand(byte motor);
  void setMaxCommand(byte motor, int command);
  int getMaxCommand(byte motor);
  void setMotorCommand(byte motor, int command);
  int getMotorCommand(byte motor);
};



#endif