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

#ifndef _AQ_CAMERA_STABILIZER_H_
#define _AQ_CAMERA_STABILIZER_H_

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
  CameraStabilizer() {}
  virtual void _initialize();
  virtual void move();

  void initialize() 
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
  
  void setPitch (float angle) 
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
  
  void setRoll (float angle) 
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
  
  void setYaw (float angle) 
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
  
  void setmCameraPitch(float gear) 
  {
    _mCameraPitch = gear;
  }  
  
  void setmCameraRoll(float gear) 
  {
    _mCameraRoll = gear;        
  }
  
  void setmCameraYaw(float gear) 
  {
    _mCameraYaw = gear;
  }
  
  void setCenterPitch(int servoCenter) 
  {
    _centerPitch = servoCenter;
  }
  
  void setCenterRoll(int servoCenter) 
  {
    _centerRoll = servoCenter;
  }
  
  void setCenterYaw(int servoCenter)
  {
    _centerYaw = servoCenter;
  }
  
  void setServoMinPitch (int servoMin) 
  {
    _servoMinPitch = servoMin;
  }
  
  void setServoMinRoll (int servoMin) 
  {
    _servoMinRoll = servoMin;
  }
  
  void setServoMinYaw (int servoMin) 
  {
    _servoMinYaw = servoMin;
  }
  
  void setServoMaxPitch (int servoMax) 
  {
    _servoMaxPitch = servoMax;
  }
  
  void setServoMaxRoll (int servoMax) 
  {
    _servoMaxRoll = servoMax;
  }
  
  void setServoMaxYaw (int servoMax) 
  {
    _servoMaxYaw = servoMax;
  }
  
  void setMode (int camMode) 
  {
    _mode = camMode;
  }
  
  int getMode () 
  {
    return _mode;
  } 
  
  int getPitch () 
  {
    return _servoPitch;
  }
  
  int getRoll () 
  {
    return _servoRoll;
  }
  
  int getYaw () 
  {
    return _servoYaw;
  }
  
  float getmCameraPitch() 
  {
    return _mCameraPitch;
  }  
  
  float getmCameraRoll() 
  {
    return _mCameraRoll;        
  }
  
  float getmCameraYaw() 
  {
    return _mCameraYaw;
  }
  
  int getCenterPitch() 
  {
    return _centerPitch;
  }
  
  int getCenterRoll() 
  {
    return _centerRoll;
  }
  
  int getCenterYaw() 
  {
    return _centerYaw;
  }
  
  int getServoMinPitch() 
  {
    return _servoMinPitch;
  }
  
  int getServoMinRoll() 
  {
    return _servoMinRoll;
  }
  
  int getServoMinYaw() 
  {
    return _servoMinYaw;
  }
  
  int getServoMaxPitch() 
  {
    return _servoMaxPitch;
  }
  
  int getServoMaxRoll() 
  {
    return _servoMaxRoll;
  }
  
  int getServoMaxYaw() 
  {
    return _servoMaxYaw;
  }
};

#endif
