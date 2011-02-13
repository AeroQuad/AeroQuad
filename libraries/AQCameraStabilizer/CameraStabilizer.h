/*
  AeroQuad v2.2 - Feburary 2011
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


// Written by CupOfTea:
// http://aeroquad.com/showthread.php?1484-Camera-Stablisation
// http://aeroquad.com/showthread.php?1491-Camera-Stablisation-continued

// ***********************************************************************
// *********************** Camera Control ***************************
// ***********************************************************************
/*Some basics about the 16 bit timer:
- The timer counts clock ticks derived from the CPU clock. Using 16MHz CPU clock
  and a prescaler of 8 gives a timer clock of 2MHz, one tick every 0.5?s. This
  is also called timer resolution.
- The timer is used as cyclic upwards counter, the counter periode is set in the
  ICRx register. IIRC periode-1 has to be set in the ICRx register.
- When the counter reaches 0, the outputs are set
- When the counter reaches OCRxy, the corresponding output is cleared.
In the code below, the periode shall be 20ms, so the ICRx register is set to 
 40000 ticks of 0.5?s/tick. It probably should be 39999, but who cares about
 this 0.5?s for the periode.
The high time shall be 1500?s, so the OCRxy register is set to 3000. A change of
 the timer periode does not change this setting, as the clock rate is still one
 tick every 0.5?s. If the prescaler was changed, the OCRxy register value would
 be different. 
*/

#ifndef _AQ_CAMERA_STABILIZER_H_
#define _AQ_CAMERA_STABILIZER_H_


#include "WProgram.h"

class CameraStabilizer 
{
private:
  float _mCameraPitch;
  float _mCameraRoll;    
  float _mCameraYaw;
  int _centerPitch;
  int _centerRoll;
  int _centerYaw;
  int _servoMinPitch;
  int _servoMinRoll;
  int _servoMinYaw;
  int _servoMaxPitch;
  int _servoMaxRoll;
  int _servoMaxYaw;
  
protected:  
  int _mode;
  int _servoPitch;                // 1000 - 2000 where we are or will move to next  
  int _servoRoll;
  int _servoYaw;

public:  
  CameraStabilizer();
  
  virtual void _initialize();
  virtual void move();

  void initialize();
  void setPitch (float angle);
  void setRoll (float angle);
  void setYaw (float angle);
  void setmCameraPitch(float gear);
  void setmCameraRoll(float gear);
  void setmCameraYaw(float gear);
  void setCenterPitch(int servoCenter);
  void setCenterRoll(int servoCenter);
  void setCenterYaw(int servoCenter);
  void setServoMinPitch (int servoMin);
  void setServoMinRoll (int servoMin);
  void setServoMinYaw (int servoMin);
  void setServoMaxPitch (int servoMax);
  void setServoMaxRoll (int servoMax);
  void setServoMaxYaw (int servoMax);
  void setMode (int camMode);
  int getMode();
  int getPitch();
  int getRoll();
  int getYaw();
  float getmCameraPitch();
  float getmCameraRoll();
  float getmCameraYaw();
  int getCenterPitch();
  int getCenterRoll();
  int getCenterYaw();
  int getServoMinPitch();
  int getServoMinRoll();
  int getServoMinYaw();
  int getServoMaxPitch();
  int getServoMaxRoll();
  int getServoMaxYaw();
};

#endif
