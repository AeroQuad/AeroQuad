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
  and a prescaler of 8 gives a timer clock of 2MHz, one tick every 0.5�s. This
  is also called timer resolution.
- The timer is used as cyclic upwards counter, the counter periode is set in the
  ICRx register. IIRC periode-1 has to be set in the ICRx register.
- When the counter reaches 0, the outputs are set
- When the counter reaches OCRxy, the corresponding output is cleared.
In the code below, the periode shall be 20ms, so the ICRx register is set to 
 40000 ticks of 0.5�s/tick. It probably should be 39999, but who cares about
 this 0.5�s for the periode.
The high time shall be 1500�s, so the OCRxy register is set to 3000. A change of
 the timer periode does not change this setting, as the clock rate is still one
 tick every 0.5�s. If the prescaler was changed, the OCRxy register value would
 be different. 
*/
class Camera 
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
  Camera() {}
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

class Camera_AeroQuad : public Camera 
{
public:
  Camera_AeroQuad() : Camera() {}
  
  void _initialize() 
  {
     // Init PWM Timer 1      Probable conflict with Arducopter Motor
    DDRB = DDRB | B11100000;                                  //Set to Output Mega Port-Pin PB5-11, PB6-12, PB7-13
                                                              //WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTTOM 
    TCCR1A =((1<<WGM11)|(1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1)); //Clear OCRnA/OCRnB/OCRnC outputs on compare match, set OCRnA/OCRnB/OCRnC outputs at BOTTOM (non-inverting mode).      
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);                 //Prescaler set to 8, that give us a resolution of 0.5us
    ICR1 = 39999;    //50hz freq (standard servos) 20ms = 40000 * 0.5us
  }

  void move() 
  {
    if (_mode > 0) 
    {
      OCR1A = _servoPitch * 2;
      OCR1B = _servoRoll * 2;
      OCR1C = _servoYaw * 2;
    }
  }
};

class Camera_Pins_2_3_5 : public Camera 
{
public:
  Camera_Pins_2_3_5() : Camera() {}
  
  void _initialize() 
  {
    // Init PWM Timer 3    Probable conflict with AeroQuad Motor
    DDRE = DDRE | B00111000;                                  //Set to Output Mega Port-Pin PE4-2, PE5-3, PE3-5
    TCCR3A =((1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1));
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31); 
    ICR3 = 39999; //50hz freq (standard servos)
  }
  
  void move() 
  {
    if (_mode > 0) 
    {
      OCR3A = _servoPitch * 2;
      OCR3B = _servoRoll * 2;
      OCR3C = _servoYaw * 2;
    }
  }
};

class Camera_Pins_6_7_8 : public Camera 
{
public:
  Camera_Pins_6_7_8() : Camera() {}
  
  void _initialize() 
  {
    // Init PWM Timer 4    Probable conflict with AeroQuad Motor or Arducopter PPM
    DDRH = DDRH | B00111000;                                  //Set to Output Mega Port-Pin PH3-8, PE4-7, PE5-6
    TCCR4A =((1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1)); 
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    ICR4 = 39999; //50hz freq (standard servos)
  }
  
  void move() 
  {
    if (_mode > 0) {
      OCR4A = _servoPitch * 2;
      OCR4B = _servoRoll * 2;
      OCR4C = _servoYaw * 2;
    }
  }
};

class Camera_Pins_44_45_46 : public Camera 
{
public:
  Camera_Pins_44_45_46() : Camera() {}
  
  void _initialize() 
  {
    // Init PWM Timer 5   Probable conflict with Arducopter Motor
    DDRL = DDRL | B00111000;                                  //Set to Output Mega Port-Pin PL3-46, PE4-45, PE5-44
    TCCR5A =((1<<WGM51)|(1<<COM5A1)|(1<<COM5B1)|(1<<COM5C1)); 
    TCCR5B = (1<<WGM53)|(1<<WGM52)|(1<<CS51);
    ICR5 = 39999; //50hz freq (standard servos)
  }
  
  void move() 
  {
    if (_mode > 0) 
    {
      OCR5A = _servoPitch * 2;
      OCR5B = _servoRoll * 2;
      OCR5C = _servoYaw * 2;      
    }
  }
};

