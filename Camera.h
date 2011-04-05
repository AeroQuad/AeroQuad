/*
  AeroQuad v2.4 - April 2011
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
class Camera {
public:
  int mode;
  float mCameraPitch;
  float mCameraRoll;    
  float mCameraYaw;
  int centerPitch;
  int centerRoll;
  int centerYaw;
  int servoMinPitch;
  int servoMinRoll;
  int servoMinYaw;
  int servoMaxPitch;
  int servoMaxRoll;
  int servoMaxYaw;
  int servoPitch;                // 1000 - 2000 where we are or will move to next  
  int servoRoll;
  int servoYaw;
  
  Camera(void) {}
  virtual void _initialize(void);
  virtual void move (void);

  void initialize(void) {
    mode = 1;                 // 0 = off,  1 = onboard stabilisation, 2 = serialCom/debug/adjust center 
    mCameraPitch = 11.11;   // scale angle to servo....  caculated as +/- 90 (ie 180) degrees maped to 1000-2000 
    mCameraRoll = 11.11;        
    mCameraYaw = 11.11;
    centerPitch = 1500;       // (bCamera) Center of stabilisation in mode 1,  point here in mode 2  
    centerRoll = 1500;        // 1000 - 2000 nornaly centered 1500
    centerYaw = 1500;  
    servoMinPitch = 1000;     // don't drive the servo past here  
    servoMinRoll = 1000;
    servoMinYaw = 1000;
    servoMaxPitch = 2000;
    servoMaxRoll = 2000;
    servoMaxYaw = 2000;
    
    _initialize(); // specific init for timer chosen
    setPitch(0);
    setRoll(0);
    setYaw(0);
    move();
  }
  
  void setPitch (float angle) {
    if (mode == 1) servoPitch = constrain((mCameraPitch * angle) + centerPitch , servoMinPitch , servoMaxPitch);
    else servoPitch = constrain(centerPitch , servoMinPitch , servoMaxPitch);
  }
  
  void setRoll (float angle) {
    if (mode == 1) servoRoll = constrain((mCameraRoll * angle) + centerRoll , servoMinRoll , servoMaxRoll);
    else servoRoll = constrain(centerRoll , servoMinRoll , servoMaxRoll);
  }
  
  void setYaw (float angle) {
    if (mode == 1) servoYaw = constrain((mCameraYaw * angle) + centerYaw , servoMinYaw , servoMaxYaw);
    else servoYaw = constrain(centerYaw , servoMinYaw , servoMaxYaw);
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
};

class Camera_AeroQuad : public Camera {
public:
  Camera_AeroQuad() : Camera() {}
  void _initialize(void) {
     // Init PWM Timer 1      Probable conflict with Arducopter Motor
    DDRB = DDRB | B11100000;                                  //Set to Output Mega Port-Pin PB5-11, PB6-12, PB7-13
                                                              //WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTTOM 
    TCCR1A =((1<<WGM11)|(1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1)); //Clear OCRnA/OCRnB/OCRnC outputs on compare match, set OCRnA/OCRnB/OCRnC outputs at BOTTOM (non-inverting mode).      
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);                 //Prescaler set to 8, that give us a resolution of 0.5us
    ICR1 = 39999;    //50hz freq (standard servos) 20ms = 40000 * 0.5us
  }

  void move(void) {
    if (mode > 0) {
      OCR1A = servoPitch * 2;
      OCR1B = servoRoll * 2;
      OCR1C = servoYaw * 2;
    }
  }
};

class Camera_Pins_2_3_5 : public Camera {
public:
  Camera_Pins_2_3_5() : Camera() {}
  void _initialize(void) {
    // Init PWM Timer 3    Probable conflict with AeroQuad Motor
    DDRE = DDRE | B00111000;                                  //Set to Output Mega Port-Pin PE4-2, PE5-3, PE3-5
    TCCR3A =((1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1));
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31); 
    ICR3 = 39999; //50hz freq (standard servos)
  }
  
  void move(void) {
    if (mode > 0) {
      OCR3A = servoPitch * 2;
      OCR3B = servoRoll * 2;
      OCR3C = servoYaw * 2;
    }
  }
};

class Camera_Pins_6_7_8 : public Camera {
public:
  Camera_Pins_6_7_8() : Camera() {}
  void _initialize(void) {
    // Init PWM Timer 4    Probable conflict with AeroQuad Motor or Arducopter PPM
    DDRH = DDRH | B00111000;                                  //Set to Output Mega Port-Pin PH3-8, PE4-7, PE5-6
    TCCR4A =((1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1)); 
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    ICR4 = 39999; //50hz freq (standard servos)
  }
  
  void move(void) {
    if (mode > 0) {
      OCR4A = servoPitch * 2;
      OCR4B = servoRoll * 2;
      OCR4C = servoYaw * 2;
    }
  }
};

class Camera_Pins_44_45_46 : public Camera {
public:
  Camera_Pins_44_45_46() : Camera() {}
  void _initialize(void) {
    // Init PWM Timer 5   Probable conflict with Arducopter Motor
    DDRL = DDRL | B00111000;                                  //Set to Output Mega Port-Pin PL3-46, PE4-45, PE5-44
    TCCR5A =((1<<WGM51)|(1<<COM5A1)|(1<<COM5B1)|(1<<COM5C1)); 
    TCCR5B = (1<<WGM53)|(1<<WGM52)|(1<<CS51);
    ICR5 = 39999; //50hz freq (standard servos)
  }
  
  void move(void) {
    if (mode > 0) {
      OCR5A = servoPitch * 2;
      OCR5B = servoRoll * 2;
      OCR5C = servoYaw * 2;      
    }
  }
};

