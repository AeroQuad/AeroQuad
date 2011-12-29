/*
  AeroQuad v3.0 - June 2011
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

#ifndef _AEROQUAD_CAMERA_STABILIZER_H_
#define _AEROQUAD_CAMERA_STABILIZER_H_


#if defined(CameraControl)

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
int mode = 0;
float mCameraPitch = 0.0;
float mCameraRoll = 0.0;    
float mCameraYaw = 0.0;
int centerPitch = 0;
int centerRoll = 0;
int centerYaw = 0;
int servoMinPitch = 0;
int servoMinRoll = 0;
int servoMinYaw = 0;
int servoMaxPitch = 0;
int servoMaxRoll = 0;
int servoMaxYaw = 0;
int servoPitch = 0;                // 1000 - 2000 where we are or will move to next  
int servoRoll = 0;
int servoYaw = 0;
  
void initializeCameraControl();
void cameraControlMove ();


  
void cameraControlSetPitch (float angle) {

  if (mode == 1) {
    servoPitch = constrain((mCameraPitch * angle) + centerPitch , servoMinPitch , servoMaxPitch);
  }
  else {
    servoPitch = constrain(centerPitch , servoMinPitch , servoMaxPitch);
  }
}
  
void cameraControlSetRoll (float angle) {

  if (mode == 1) {
    servoRoll = constrain((mCameraRoll * angle) + centerRoll , servoMinRoll , servoMaxRoll);
  }
  else {
    servoRoll = constrain(centerRoll , servoMinRoll , servoMaxRoll);
  }
}
  
void cameraControlSetYaw (float angle) {

  if (mode == 1) {
    servoYaw = constrain((mCameraYaw * angle) + centerYaw , servoMinYaw , servoMaxYaw);
  }
  else {
    servoYaw = constrain(centerYaw , servoMinYaw , servoMaxYaw);
  }	
}
  
void setmCameraPitch(float gear) {
  mCameraPitch = gear;
}  
  
void setmCameraRoll(float gear) {
  mCameraRoll = gear;        
}
  
void setmCameraYaw(float gear) {
  mCameraYaw = gear;
}
  
void setCenterPitch(int servoCenter) {
  centerPitch = servoCenter;
}
  
void setCenterRoll(int servoCenter) {
  centerRoll = servoCenter;
}
  
void setCenterYaw(int servoCenter) {
  centerYaw = servoCenter;
}
  
void setServoMinPitch (int servoMin) {
  servoMinPitch = servoMin;
}
  
void setServoMinRoll (int servoMin) {
  servoMinRoll = servoMin;
}
  
void setServoMinYaw (int servoMin) {
  servoMinYaw = servoMin;
}
 
void setServoMaxPitch (int servoMax) {
  servoMaxPitch = servoMax;
}
  
void setServoMaxRoll (int servoMax) {
  servoMaxRoll = servoMax;
}
  
void setServoMaxYaw (int servoMax) {
  servoMaxYaw = servoMax;
}
  
void setMode (int camMode) {
  mode = camMode;
}
  
int getMode (void) {
  return mode;
} 
  
int getPitch (void) {
  return servoPitch;
}
  
int getRoll (void) {
  return servoRoll;
}
  
int getYaw (void) {
  return servoYaw;
}
  
float getmCameraPitch(void) {
  return mCameraPitch;
}  
  
float getmCameraRoll(void) {
  return mCameraRoll;        
}
  
float getmCameraYaw(void) {
  return mCameraYaw;
}
  
int getCenterPitch(void) {
  return centerPitch;
}
  
int getCenterRoll(void) {
  return centerRoll;
}
  
int getCenterYaw(void) {
  return centerYaw;
}
  
int getServoMinPitch(void) {
  return servoMinPitch;
}
  
int getServoMinRoll(void) {
  return servoMinRoll;
}
  
int getServoMinYaw(void) {
  return servoMinYaw;
}
 
int getServoMaxPitch(void) {
  return servoMaxPitch;
}
  
int getServoMaxRoll(void) {
  return servoMaxRoll;
}
 
int getServoMaxYaw(void) {
  return servoMaxYaw;
}

void initializeCameraStabilization() {
  mode = 1;                 // 0 = off,  1 = onboard stabilisation, 2 = serialCom/debug/adjust center 
  mCameraPitch = 318.3;   // scale angle to servo....  caculated as +/- 90 (ie 180) degrees maped to 1000-2000 
  mCameraRoll = 318.3;        
  mCameraYaw = 318.3;
  centerPitch = 1500;       // (bCamera) Center of stabilisation in mode 1,  point here in mode 2  
  centerRoll = 1500;        // 1000 - 2000 nornaly centered 1500
  centerYaw = 1500;  
  servoMinPitch = 1000;     // don't drive the servo past here  
  servoMinRoll = 1000;
  servoMinYaw = 1000;
  servoMaxPitch = 2000;
  servoMaxRoll = 2000;
  servoMaxYaw = 2000;
    
  initializeCameraControl(); // specific init for timer chosen
  cameraControlSetPitch(0);
  cameraControlSetRoll(0);
  cameraControlSetYaw(0);
  cameraControlMove();
}

#endif  // #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#endif  // #define _AEROQUAD_CAMERA_STABILIZER_H_
