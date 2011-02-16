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

#include "CameraStabilizer.h"


CameraStabilizer::CameraStabilizer() 
{
}

void CameraStabilizer::_initialize() {}
void CameraStabilizer::move() {}

void CameraStabilizer::initialize() 
{
  _mode = 1;                 // 0 = off,  1 = onboard stabilisation, 2 = serialCom/debug/adjust center 
  _mCameraPitch = 11.11;   // scale angle to servo....  caculated as +/- 90 (ie 180) degrees maped to 1000-2000 
  _mCameraRoll = 11.11;        
  _mCameraYaw = 11.11;
  _centerPitch = 1500;       // (bCamera) Center of stabilisation in mode 1,  point here in mode 2  
  _centerRoll = 1500;        // 1000 - 2000 nornaly centered 1500
  _centerYaw = 1500;  
  _servoMinPitch = 1000;     // don't drive the servo past here  
  _servoMinRoll = 1000;
  _servoMinYaw = 1000;
  _servoMaxPitch = 2000;
  _servoMaxRoll = 2000;
  _servoMaxYaw = 2000;
    
  _initialize(); // specific init for timer chosen
  setPitch(0);
  setRoll(0);
  setYaw(0);
  move();
}
  
void CameraStabilizer::setPitch (float angle) 
{
  if (_mode == 1) 
  {
    _servoPitch = constrain((_mCameraPitch * angle) + _centerPitch , _servoMinPitch , _servoMaxPitch);
  }
  else 
  {
    _servoPitch = constrain(_centerPitch , _servoMinPitch , _servoMaxPitch);
  }
}
  
void CameraStabilizer::setRoll (float angle) 
{
  if (_mode == 1) 
  {
    _servoRoll = constrain((_mCameraRoll * angle) + _centerRoll , _servoMinRoll , _servoMaxRoll);
  }
  else 
  {
    _servoRoll = constrain(_centerRoll , _servoMinRoll , _servoMaxRoll);
  }
}
  
void CameraStabilizer::setYaw (float angle) 
{
  if (_mode == 1) 
  {
    _servoYaw = constrain((_mCameraYaw * angle) + _centerYaw , _servoMinYaw , _servoMaxYaw);
  }
  else 
  {
    _servoYaw = constrain(_centerYaw , _servoMinYaw , _servoMaxYaw);
  }
}
  
void CameraStabilizer::setmCameraPitch(float gear) 
{
  _mCameraPitch = gear;
}  
  
void CameraStabilizer::setmCameraRoll(float gear) 
{
  _mCameraRoll = gear;        
}
  
void CameraStabilizer::setmCameraYaw(float gear) 
{
  _mCameraYaw = gear;
}
  
void CameraStabilizer::setCenterPitch(int servoCenter) 
{
  _centerPitch = servoCenter;
}
  
void CameraStabilizer::setCenterRoll(int servoCenter) 
{
  _centerRoll = servoCenter;
}
  
void CameraStabilizer::setCenterYaw(int servoCenter)
{
  _centerYaw = servoCenter;
}
  
void CameraStabilizer::setServoMinPitch (int servoMin) 
{
  _servoMinPitch = servoMin;
}
  
void CameraStabilizer::setServoMinRoll (int servoMin) 
{
  _servoMinRoll = servoMin;
}
 
void CameraStabilizer::setServoMinYaw (int servoMin) 
{
  _servoMinYaw = servoMin;
}
  
void CameraStabilizer::setServoMaxPitch (int servoMax) 
{
  _servoMaxPitch = servoMax;
}
  
void CameraStabilizer::setServoMaxRoll (int servoMax) 
{
  _servoMaxRoll = servoMax;
}
  
void CameraStabilizer::setServoMaxYaw (int servoMax) 
{
  _servoMaxYaw = servoMax;
}
  
void CameraStabilizer::setMode (int camMode) 
{
  _mode = camMode;
}
  
int CameraStabilizer::getMode () 
{
  return _mode;
} 
  
int CameraStabilizer::getPitch () 
{
  return _servoPitch;
}
  
int CameraStabilizer::getRoll () 
{
  return _servoRoll;
}
  
int CameraStabilizer::getYaw () 
{
  return _servoYaw;
}
  
float CameraStabilizer::getmCameraPitch() 
{
  return _mCameraPitch;
}  
  
float CameraStabilizer::getmCameraRoll() 
{
  return _mCameraRoll;        
}
  
float CameraStabilizer::getmCameraYaw() 
{
  return _mCameraYaw;
}
  
int CameraStabilizer::getCenterPitch() 
{
  return _centerPitch;
}
  
int CameraStabilizer::getCenterRoll() 
{
  return _centerRoll;
}
  
int CameraStabilizer::getCenterYaw() 
{
  return _centerYaw;
}
  
int CameraStabilizer::getServoMinPitch() 
{
  return _servoMinPitch;
}
  
int CameraStabilizer::getServoMinRoll() 
{
  return _servoMinRoll;
}
  
int CameraStabilizer::getServoMinYaw() 
{
  return _servoMinYaw;
}
  
int CameraStabilizer::getServoMaxPitch() 
{
  return _servoMaxPitch;
}
  
int CameraStabilizer::getServoMaxRoll() 
{
  return _servoMaxRoll;
}
 
int CameraStabilizer::getServoMaxYaw() 
{
  return _servoMaxYaw;
}
