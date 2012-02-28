/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

int cameraMode;         // 0 = off,  1 = onboard stabilisation, 2 = serialCom/debug/adjust center 
float mCameraPitch;     // scale angle...  the default 318.3 assumes that a rotation of
float mCameraRoll;      //  +/- 90deg ie pi radians will move your hardware in the same ratio
float mCameraYaw;
int servoCenterPitch;   // (bCamera) Center of stabilisation in mode 1,  point here in mode 2
int servoCenterRoll;
int servoCenterYaw;
int servoMinPitch;      // limit servo movement
int servoMinRoll;
int servoMinYaw;
int servoMaxPitch;
int servoMaxRoll;
int servoMaxYaw;
  
void initializeCameraControl();
void cameraControlMove (int, int, int);

void moveCamera(float anglePitch, float angleRoll, float angleYaw) {
  if (cameraMode == 1) {
    cameraControlMove(constrain((mCameraPitch * anglePitch) + servoCenterPitch , servoMinPitch , servoMaxPitch),
                      constrain((mCameraRoll * angleRoll) + servoCenterRoll , servoMinRoll , servoMaxRoll),
                      constrain((mCameraYaw * angleYaw) + servoCenterYaw , servoMinYaw , servoMaxYaw));
  }
  else {
    cameraControlMove(constrain(servoCenterPitch , servoMinPitch , servoMaxPitch),
                      constrain(servoCenterRoll , servoMinRoll , servoMaxRoll),
                      constrain(servoCenterYaw , servoMinYaw , servoMaxYaw));
  }
}

void initializeCameraStabilization() {
  initializeCameraControl(); // specific init for timer chosen
  moveCamera(0,0,0); 
}

#endif  // #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#endif  // #define _AEROQUAD_CAMERA_STABILIZER_H_
