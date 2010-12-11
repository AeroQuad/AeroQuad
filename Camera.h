//Camera.h
// ***********************************************************************
// *********************** Camera Control ***************************
// ***********************************************************************
class camera {
public:
  int mode;                      // 0 = off,  1 = onboard stabilisation, 2 = serialCom/debug/adjust center
  float mCameraPitch;       // scale angle to servo....  caculated as +/- 90 (ie 180) degrees maped to 1000-2000 
  float mCameraRoll;        
  float mCameraYaw;
  int centerPitch;               // (bCamera) Center of stabilisation in mode 1,  point here in mode 2  
  int centerRoll;                // 1000 - 2000 nornaly centered 1500
  int centerYaw;  
  int servoPitch;                // 1000 - 2000 where we are or will move to next  
  int servoRoll;
  int servoYaw;
  int servoMinPitch;                // don't drive the servo past here  
  int servoMinRoll;
  int servoMinYaw;
  int servoMaxPitch;
  int servoMaxRoll;
  int servoMaxYaw;

  camera(void) {
     mode = 1;
     mCameraPitch = 11.11;
     mCameraRoll = 11.11;        
     mCameraYaw = 11.11;
     centerPitch = 1500;
     centerRoll = 1500;
     centerYaw = 1500;  
     servoPitch = 1500;
     servoRoll = 1500;
     servoYaw = 1500;
     servoMinPitch = 1000;
     servoMinRoll = 1000;
     servoMinYaw = 1000;  
     servoMaxPitch = 2000;
     servoMaxRoll = 2000;
     servoMaxYaw = 2000; 
  }
  void initialize(void) {

#ifdef CameraTimer1
   // Init PWM Timer 1
    pinMode(11,OUTPUT); // Pitch (PB5/OC1A)
    pinMode(12,OUTPUT); // Roll  (PB6/OC1B)
    pinMode(13,OUTPUT); // Yaw   (PB7/OC1C)
                                                              // WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTTOM 
    TCCR1A =((1<<WGM11)|(1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1)); // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode).      
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);                 //Prescaler set to 8, that give us a resolution of 0.5us
    OCR1B = 3000;                                             //init each servo to center
    OCR1C = 3000;   
    OCR1A = 3000;   
    ICR1 = 39999;    //50hz freq (standard servos) 20ms = 40000 * 0.5us
#endif
#ifdef CameraTimer3
    // Init PWM Timer 3
    pinMode(2,OUTPUT); //OUT7 (PE4/OC3B)
    pinMode(3,OUTPUT); //OUT6 (PE5/OC3C)
    pinMode(5,OUTPUT); //     (PE3/OC3A)
    TCCR3A =((1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1));
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31); 
    OCR3A = 3000; //PE3, NONE
    OCR3B = 3000; //PE4, OUT7
    OCR3C = 3000; //PE5, OUT6
    ICR3 = 39999; //50hz freq (standard servos)
#endif
#ifdef CameraTimer4
    // Init PWM Timer 5
    pinMode(6,OUTPUT); // (PL5/OC4A)
    pinMode(7,OUTPUT);  // (PL4/OC4B)
    pinMode(8,OUTPUT);  // (PL3/OC4C)
    TCCR4A =((1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1)); 
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    OCR4A = 3000;  
    OCR4B = 3000; 
    OCR4C = 3000; 
    ICR4 = 39999; //50hz freq (standard servos)
#endif    
#ifdef CameraTimer5
    // Init PWM Timer 5
    pinMode(44,OUTPUT);  //OUT1 (PL5/OC5C)
    pinMode(45,OUTPUT);  //OUT0 (PL4/OC5B)
    pinMode(46,OUTPUT);  //     (PL3/OC5A)
    TCCR5A =((1<<WGM51)|(1<<COM5A1)|(1<<COM5B1)|(1<<COM5C1)); 
    TCCR5B = (1<<WGM53)|(1<<WGM52)|(1<<CS51);
    OCR5A = 3000; //PL3, 
    OCR5B = 3000; //PL4, OUT0
    OCR5C = 3000; //PL5, OUT1
    ICR5 = 39999; //50hz freq (standard servos)
#endif
  }
  
  void setPitch (int angle) {
    if (mode == 1) }
      servoPitch = constrain((mCameraPitch * angle) + centerPitch , servoMinPitch , servoMaxPitch);
    } else {
      servoPitch = constrain(centerPitch , servoMinPitch , servoMaxPitch);
    }  
  }
  void setRoll (int angle) {
    if (mode == 1) }
      servoRoll = constrain((mCameraRoll * angle) + centerRoll , servoMinRoll , servoMaxRoll);
    } else {
      servoRoll = constrain(centerRoll , servoMinRoll , servoMaxRoll);
    }  
  }
  void setYaw (int angle) {
    if (mode == 1) }
      servoYaw = constrain((mCameraYaw * angle) + centerYaw , servoMinYaw , servoMaxYaw);
    } else {
      servoYaw = constrain(centerYaw , servoMinYaw , servoMaxYaw);
    }  
  }
  void move (void) {
    if (mode > 0) {
#ifdef CameraTimer1
      OCR1A = servoPitch * 2;
      OCR1B = servoRoll * 2;
      OCR1C = servoYaw * 2;
#endif
#ifdef CameraTimer3
      OCR3A = servoPitch * 2 ;
      OCR3B = servoRoll * 2;
      OCR3C = servoYaw * 2;
#endif
#ifdef CameraTimer4
      OCR4A = servoPitch * 2 ;
      OCR4B = servoRoll * 2;
      OCR4C = servoYaw * 2;
#endif
#ifdef CameraTimer5
      OCR5A = servoPitch * 2 ;
      OCR5B = servoRoll * 2;
      OCR5C = servoYaw * 2;      
#endif
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
};
  
