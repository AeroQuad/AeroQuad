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

class Motors {
public:
  // Assume maximum number of motors is 8, leave array indexes unused if lower number
  int motorAxisCommand[3];
  //int motorAxisCommandRoll[LASTMOTOR];
  //int motorAxisCommandPitch[LASTMOTOR];
  //int motorAxisCommandYaw[LASTMOTOR];
  //int motorMixerSettingRoll[LASTMOTOR];
  //int motorMixerSettingPitch[LASTMOTOR];
  //int motorMixerSettingYaw[LASTMOTOR];
  int motorCommand[LASTMOTOR];
  int minCommand[LASTMOTOR];
  int maxCommand[LASTMOTOR];
  float throttle;
  float timerDebug;
  int delta;
  byte axis;
  // Ground station control
  int remoteCommand[LASTMOTOR];
  float mMotorCommand;
  float bMotorCommand;


  Motors(void){
    throttle = 0;
    motorAxisCommand[ROLL] = 0;
    motorAxisCommand[PITCH] = 0;
    motorAxisCommand[YAW] = 0;
    for (byte motor = 0; motor < LASTMOTOR; motor++) {
      //motorAxisCommandRoll[motor] = 0;
      //motorAxisCommandPitch[motor] = 0;
      //motorAxisCommandYaw[motor] = 0;
      //motorMixerSettingRoll[motor] = 0;
      //motorMixerSettingPitch[motor] = 0;
      //motorMixerSettingYaw[motor] = 0;
      motorCommand[motor] = 1000;
      minCommand[motor] = MINCOMMAND;
      maxCommand[motor] = MAXCOMMAND;
      remoteCommand[motor] = 1000;
    }
    delta = 0;
  };

  // The following function calls must be defined in any new subclasses
  virtual void initialize(void);
  virtual void write (void);
  virtual void commandAllMotors(int motorCommand);

  //Any number of optional methods can be configured as needed by the SubSystem to expose functionality externally
  void pulseMotors(byte quantity) {
    for (byte i = 0; i < quantity; i++) {
      commandAllMotors(MINCOMMAND + 100);
      delay(250);
      commandAllMotors(MINCOMMAND);
      delay(250);
    }
  }

  void setRemoteCommand(byte motor, int value) {
    remoteCommand[motor] = value;
  }

  const int getRemoteCommand(byte motor) {
    return remoteCommand[motor];
  }

  const float getMotorSlope(void) {
    return mMotorCommand;
  }

  const float getMotorOffset(void) {
    return bMotorCommand;
  }

  void setMinCommand(byte motor, int value) {
    minCommand[motor] = value;
  }

  const int getMinCommand(byte motor) {
    return minCommand[motor];
  }

  void setMaxCommand(byte motor, int value) {
    maxCommand[motor] = value;
  }

  const int getMaxCommand(byte motor) {
    return maxCommand[motor];
  }

  void setMotorAxisCommand(byte motor, int value) {
    motorAxisCommand[motor] = value;
  }

  const int getMotorAxisCommand(byte motor) {
    return motorAxisCommand[motor];
  }

  void setMotorCommand(byte motor, int value) {
    motorCommand[motor] = value;
  }

  const int getMotorCommand(byte motor) {
    return motorCommand[motor];
  }
  void setThrottle(float value) {
    throttle = value;
  }
  const float getThrottle() {
    return throttle;
  }
};


/******************************************************/
/********************* PWM Motors *********************/
/******************************************************/
class Motors_PWM : public Motors {
private:
  #if defined(AeroQuadMega_v2) || defined(AeroQuadMega_Wii) || defined (AeroQuadMega_CHR6DM)
    #define FRONTMOTORPIN  2
    #define REARMOTORPIN   3
    #define RIGHTMOTORPIN  5
    #define LEFTMOTORPIN   6
    #define LASTMOTORPIN   7
  #else
    #define FRONTMOTORPIN  3
    #define REARMOTORPIN   9
    #define RIGHTMOTORPIN 10
    #define LEFTMOTORPIN  11
    #define LASTMOTORPIN  12
  #endif
  int minCommand;
  byte pin;

 public:
  Motors_PWM() : Motors(){
   // Analog write supports commands from 0-255 => 0 - 100% duty cycle
   // Using 125-250 for motor setting 1000-2000
  }

  void initialize(void) {
    commandAllMotors(1000);
  }

  void write(void) {
    analogWrite(FRONTMOTORPIN, motorCommand[FRONT] / 8);
    analogWrite(REARMOTORPIN,  motorCommand[REAR]  / 8);
    analogWrite(RIGHTMOTORPIN, motorCommand[RIGHT] / 8);
    analogWrite(LEFTMOTORPIN,  motorCommand[LEFT]  / 8);

  }

  void commandAllMotors(int _motorCommand) {   // Sends commands to all motors
    analogWrite(FRONTMOTORPIN, _motorCommand / 8);
    analogWrite(REARMOTORPIN,  _motorCommand / 8);
    analogWrite(RIGHTMOTORPIN, _motorCommand / 8);
    analogWrite(LEFTMOTORPIN,  _motorCommand / 8);
  }
};

/***********************************************************/
/********************* PWMtimer Motors *********************/
/***********************************************************/
// Special thanks to CupOfTea for authorting this class
// http://aeroquad.com/showthread.php?1553-Timed-Motors_PWM
// Uses system timers directly instead of analogWrite
/*Some basics about the 16 bit timer:
- The timer counts clock ticks derived from the CPU clock. Using 16MHz CPU clock
  and a prescaler of 8 gives a timer clock of 2MHz, one tick every 0.5us. This
  is also called timer resolution.
- The timer is used as cyclic upwards counter, the counter period is set in the
  ICRx register. IIRC period-1 has to be set in the ICRx register.
- When the counter reaches 0, the outputs are set
- When the counter reaches OCRxy, the corresponding output is cleared.
In the code below, the period shall be 3.3ms (300hz), so the ICRx register is
 set to 6600 ticks of 0.5us/tick. It probably should be 6599, but who cares about
 this 0.5us for the period. This value is #define TOP
The high time shall be 1000us, so the OCRxy register is set to 2000. In the code
 below this can be seen in the line "commandAllMotors(1000);"  A change of
 the timer period does not change this setting, as the clock rate is still one
 tick every 0.5us. If the prescaler was changed, the OCRxy register value would
 be different.
*/
class Motors_PWMtimer : public Motors {
private:
/*  Motor   Mega Pin Port        Uno Pin Port          HEXA Mega Pin Port
    FRONT         2  PE4              3  PD3
    REAR          3  PE5              9  PB1
    RIGHT         5  PE3             10  PB2                      7  PH4
    LEFT          6  PH3             11  PB3                      8  PH5
*/

#define PWM_FREQUENCY 300   // in Hz
#define PWM_PRESCALER 8
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)

public:
  Motors_PWMtimer() : Motors(){
  }

  void initialize(void) {
#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    DDRE = DDRE | B00111000;                                  // Set ports to output PE3-5
  #if defined(plusConfig) || defined(XConfig)
    DDRH = DDRH | B00001000;                                  // Set port to output PH3
  #endif
  #if defined(HEXACOAXIAL) || defined(HEXARADIAL)
    DDRH = DDRH | B00111000;                                  // Set ports to output PH3-5
  #endif
//#endif
//#if defined (__AVR_ATmega328P__)
#else
    DDRB = DDRB | B00001110;                                  // Set ports to output PB1-3
    DDRD = DDRD | B00001000;                                  // Set port to output PD3
#endif

    commandAllMotors(1000);                                   // Initialise motors to 1000us (stopped)

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // Init PWM Timer 3                                       // WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTTOM
    TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);                 // Prescaler set to 8, that gives us a resolution of 0.5us
    ICR3 = PWM_COUNTER_PERIOD;                               // Clock_speed / ( Prescaler * desired_PWM_Frequency) #defined above.
  #if defined(plusConfig) || defined(XConfig)
    // Init PWM Timer 4
    TCCR4A = (1<<WGM41)|(1<<COM4A1);
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    ICR4 = PWM_COUNTER_PERIOD;
  #endif
  #if defined(HEXACOAXIAL) || defined(HEXARADIAL)
    // Init PWM Timer 4
    TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    ICR4 = PWM_COUNTER_PERIOD;
  #endif
#else
    // Init PWM Timer 1  16 bit
    TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
    ICR1 = PWM_COUNTER_PERIOD;
    // Init PWM Timer 2   8bit                               // WGMn1 WGMn2 = Mode ? Fast PWM, TOP = 0xFF ,Update of OCRnx at BOTTOM
    TCCR2A = (1<<WGM20)|(1<<WGM21)|(1<<COM2A1)|(1<<COM2B1);  // Clear OCnA/OCnB on compare match, set OCnA/OCnB at BOTTOM (non-inverting mode)
    TCCR2B = (1<<CS22)|(1<<CS21);                            // Prescaler set to 256, that gives us a resolution of 16us
    // TOP is fixed at 255                                   // Output_PWM_Frequency = 244hz = 16000000/(256*(1+255)) = Clock_Speed / (Prescaler * (1 + TOP))
#endif
}

  void write(void) {
#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    OCR3B = motorCommand[FRONT] * 2 ;
    OCR3C = motorCommand[REAR]  * 2 ;
    OCR3A = motorCommand[RIGHT] * 2 ;
    OCR4A = motorCommand[LEFT]  * 2 ;
  #if defined(HEXACOAXIAL) || defined(HEXARADIAL)
    OCR4B = motorCommand[RIGHT2] * 2 ;
    OCR4C = motorCommand[LEFT2]  * 2 ;
  #endif
//#endif
//#if defined (__AVR_ATmega328P__)
#else
    OCR2B = motorCommand[FRONT] / 16 ;                       // 1000-2000 to 128-256
    OCR1A = motorCommand[REAR]  * 2 ;
    OCR1B = motorCommand[RIGHT] * 2 ;
    OCR2A = motorCommand[LEFT]  / 16 ;
#endif
  }

 void commandAllMotors(int _motorCommand) {   // Sends commands to all motors
#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    OCR3B = _motorCommand * 2 ;
    OCR3C = _motorCommand * 2 ;
    OCR3A = _motorCommand * 2 ;
    OCR4A = _motorCommand * 2 ;
  #if defined(HEXACOAXIAL) || defined(HEXARADIAL)
    OCR4B = _motorCommand * 2 ;
    OCR4C = _motorCommand * 2 ;
  #endif
//#endif
//#if defined (__AVR_ATmega328P__)
#else
    OCR2B = _motorCommand / 16 ;
    OCR1A = _motorCommand * 2 ;
    OCR1B = _motorCommand * 2 ;
    OCR2A = _motorCommand / 16 ;
#endif
  }
};

/******************************************************/
/********************* Fake PWM Motors ****************/
/******************************************************/
#ifdef CHR6DM_FAKE_MOTORS
class Motors_PWM_Fake : public Motors {
private:
  #if defined(AeroQuadMega_v2) || defined(AeroQuadMega_Wii) || defined (AeroQuadMega_CHR6DM)
    #define FRONTMOTORPIN 2
    #define REARMOTORPIN 3
    #define RIGHTMOTORPIN 5
    #define LEFTMOTORPIN 6
    #define LASTMOTORPIN 7
  #else
    #define FRONTMOTORPIN 3
    #define REARMOTORPIN 9
    #define RIGHTMOTORPIN 10
    #define LEFTMOTORPIN 11
    #define LASTMOTORPIN 12
  #endif
  int minCommand;
  byte pin;

 public:
  Motors_PWM_Fake() : Motors(){
    // Scale motor commands to analogWrite
    // Only supports commands from 0-255 => 0 - 100% duty cycle
    // Usable pulsewith from approximately 1000-2000 us = 126 - 250
    // m = (250-126)/(2000-1000) = 0.124
    // b = y1 - (m * x1) = 126 - (0.124 * 1000) = 2
    mMotorCommand = 0.124;
    bMotorCommand = 2.0;
  }

  void initialize(void) {
    pinMode(FRONTMOTORPIN, OUTPUT);
    fake_analogWrite(FRONTMOTORPIN, 124);
    pinMode(REARMOTORPIN, OUTPUT);
    fake_analogWrite(REARMOTORPIN, 124);
    pinMode(RIGHTMOTORPIN, OUTPUT);
    fake_analogWrite(RIGHTMOTORPIN, 124);
    pinMode(LEFTMOTORPIN, OUTPUT);
  }

  void write(void) {
    fake_analogWrite(FRONTMOTORPIN, (motorCommand[FRONT] * mMotorCommand) + bMotorCommand);
    fake_analogWrite(REARMOTORPIN, (motorCommand[REAR] * mMotorCommand) + bMotorCommand);
    fake_analogWrite(RIGHTMOTORPIN, (motorCommand[RIGHT] * mMotorCommand) + bMotorCommand);
    fake_analogWrite(LEFTMOTORPIN, (motorCommand[LEFT] * mMotorCommand) + bMotorCommand);
  }

  void commandAllMotors(int _motorCommand) {   // Sends commands to all motors
    fake_analogWrite(FRONTMOTORPIN, (_motorCommand * mMotorCommand) + bMotorCommand);
    fake_analogWrite(REARMOTORPIN, (_motorCommand * mMotorCommand) + bMotorCommand);
    fake_analogWrite(RIGHTMOTORPIN, (_motorCommand * mMotorCommand) + bMotorCommand);
    fake_analogWrite(LEFTMOTORPIN, (_motorCommand * mMotorCommand) + bMotorCommand);
  }

  void fake_analogWrite(int pin, int value){
    Serial2.print("analogWrite:");
    Serial2.print(pin);
    Serial2.print(",");
    Serial2.println(value);
  }
};
#endif

/******************************************************/
/***************** ArduCopter Motors ******************/
/******************************************************/
#if defined(ArduCopter) || defined(APM_OP_CHR6DM)
class Motors_ArduCopter : public Motors {
public:
  Motors_ArduCopter() : Motors() {}

  void initialize(void) {
    // Init PWM Timer 1
    //pinMode(11,OUTPUT); //     (PB5/OC1A)
    pinMode(12,OUTPUT); //OUT2 (PB6/OC1B)
    pinMode(13,OUTPUT); //OUT3 (PB7/OC1C)

    //Remember the registers not declared here remains zero by default...
    TCCR1A =((1<<WGM11)|(1<<COM1B1)|(1<<COM1C1)); //Please read page 131 of DataSheet, we are changing the registers settings of WGM11,COM1B1,COM1A1 to 1 thats all...
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11); //Prescaler set to 8, that give us a resolution of 0.5us, read page 134 of data sheet
    //OCR1A = 3000; //PB5, none
    OCR1B = 2000; //PB6, OUT2
    OCR1C = 2000; //PB7  OUT3
    ICR1 = 6600; //300hz freq...

    // Init PWM Timer 3
    pinMode(2,OUTPUT); //OUT7 (PE4/OC3B)
    pinMode(3,OUTPUT); //OUT6 (PE5/OC3C)
    //pinMode(5,OUTPUT); //     (PE3/OC3A)
    TCCR3A =((1<<WGM31)|(1<<COM3B1)|(1<<COM3C1));
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
    //OCR3A = 3000; //PE3, NONE
    OCR3B = 2000; //PE4, OUT7
    OCR3C = 2000; //PE5, OUT6
    ICR3 = 40000; //50hz freq (standard servos)

    // Init PWM Timer 5
    pinMode(44,OUTPUT);  //OUT1 (PL5/OC5C)
    pinMode(45,OUTPUT);  //OUT0 (PL4/OC5B)
    //pinMode(46,OUTPUT);  //     (PL3/OC5A)

    TCCR5A =((1<<WGM51)|(1<<COM5B1)|(1<<COM5C1));
    TCCR5B = (1<<WGM53)|(1<<WGM52)|(1<<CS51);
    //OCR5A = 3000; //PL3,
    OCR5B = 2000; //PL4, OUT0
    OCR5C = 2000; //PL5, OUT1
    ICR5 = 6600; //300hz freq

    // Init PPM input and PWM Timer 4
    pinMode(49, INPUT);  // ICP4 pin (PL0) (PPM input)
    pinMode(7,OUTPUT);   //OUT5 (PH4/OC4B)
    pinMode(8,OUTPUT);   //OUT4 (PH5/OC4C)

    TCCR4A =((1<<WGM40)|(1<<WGM41)|(1<<COM4C1)|(1<<COM4B1)|(1<<COM4A1));
    //Prescaler set to 8, that give us a resolution of 0.5us
    // Input Capture rising edge
    TCCR4B = ((1<<WGM43)|(1<<WGM42)|(1<<CS41)|(1<<ICES4));

    OCR4A = 40000; ///50hz freq. (standard servos)
    OCR4B = 2000; //PH4, OUT5
    OCR4C = 2000; //PH5, OUT4

    //TCCR4B |=(1<<ICES4); //Changing edge detector (rising edge).
    //TCCR4B &=(~(1<<ICES4)); //Changing edge detector. (falling edge)
    TIMSK4 |= (1<<ICIE4); // Enable Input Capture interrupt. Timer interrupt mask
    
    commandAllMotors(1000);
  }

  void write (void) {
    OCR1B = motorCommand[FRONT] * 2;
    OCR1C = motorCommand[REAR] * 2;
    OCR5B = motorCommand[RIGHT] * 2;
    OCR5C = motorCommand[LEFT] * 2;
  }

  void commandAllMotors(int _motorCommand) {   // Sends commands to all motors
    OCR1B = _motorCommand * 2;
    OCR1C = _motorCommand * 2;
    OCR5B = _motorCommand * 2;
    OCR5C = _motorCommand * 2;
  }
};
#endif

/*************************************************************/
/********************* Multipilot Motors *********************/
/*************************************************************/
#ifdef MultipilotI2C
class Motors_I2C : public Motors {
public:


/* Mixertable VAR */
  float MotorGas[LASTMOTOR];
  float MotorPitch[LASTMOTOR];
  float MotorRoll[LASTMOTOR];
  float MotorYaw[LASTMOTOR];
  float motorAxisCommandPitch[LASTMOTOR];
  float motorAxisCommandRoll[LASTMOTOR];
  float motorAxisCommandYaw[LASTMOTOR];

  unsigned char MotorI2C[LASTMOTOR];
  Motors_I2C() : Motors() {}

  void initialize(void) {
  char Motor[LASTMOTOR];
  // Scale motor commands to analogWrite
  // m = (250-126)/(2000-1000) = 0.124
  // b = y1 - (m * x1) = 126 - (0.124 * 1000) = 2
  //float mMotorCommand = 0.124;
  //float bMotorCommand = 2 ;

  mMotorCommand = 0.255;
  bMotorCommand = -255;
  timer_debug=0;

  //motorCommand[8] = {1000,1000,1000,1000,1000,1000,1000,1000};

  //int subtrim[4] = {1500,1500,1500,1500};    //SUBTRIM li esprimo in millisecondi come i servi per standardizzazione.
  //motorAxisCommand[3] = {0,0,0};

  // If AREF = 3.3V, then A/D is 931 at 3V and 465 = 1.5V
  // Scale gyro output (-465 to +465) to motor commands (1000 to 2000)
  // use y = mx + b
  //mMotorRate = 1.0753; // m = (y2 - y1) / (x2 - x1) = (2000 - 1000) / (465 - (-465))
  //bMotorRate = 1500;   // b = y1 - m * x1
  init_mixer_table(); // Init MixerTable
  Wire.begin(0x29);
 }

// C'e' im
#ifdef HEXARADIAL
void init_mixer_table()
{
// Example for Hexa configuration
MotorGas[0] = 100;
MotorPitch[0] = -100;
MotorRoll[0] =  0;
MotorYaw[0] =  100;

MotorGas[1] = 100;
MotorPitch[1] = -50;
MotorRoll[1] =  -100;
MotorYaw[1] =  -100;

MotorGas[2] = 100;
MotorPitch[2] = +50  ;
MotorRoll[2] =  -100;
MotorYaw[2] =  100;

MotorGas[3] = 100;
MotorPitch[3] =  +100;
MotorRoll[3] =  0;
MotorYaw[3] =  -100;

MotorGas[4] = 100;
MotorPitch[4] = +50;
MotorRoll[4] =  100;
MotorYaw[4] =  100;

MotorGas[5] = 100;
MotorPitch[5] =  -50;
MotorRoll[5] =  100;
MotorYaw[5] =  -100;
}
#endif

// Example of Hexa Coaxial.
//Configurazione Motori Hexa Coaxial

//Dietro 			GAS		NICK		        ROLL		        YAW
//Sopra 1         		64		-64			0			-64
//Sotto 2			76		-64			0			64

//Guardando l'Hexafox da dietro braccio  Sinistro
//Sopra 3			64		32			64			64
//Sotto 4			76		32			64			-64

//Guardando l'Hexafox da dietro braccio  Destro.

//Sopra 5			64		32			-64			64
//Sotto 6			76		32			-64			-64


#ifdef HEXACOAXIAL
void init_mixer_table()
{
// Example for Hexa configuration
MotorGas[0] = 95;
MotorPitch[0] = 100;
MotorRoll[0] =  0;
MotorYaw[0] =  100;

MotorGas[1] = 100;
MotorPitch[1] = 100;
MotorRoll[1] =  0;
MotorYaw[1] =  -100;

MotorGas[2] = 95;
MotorPitch[2] = -50  ;
MotorRoll[2] =  100;
MotorYaw[2] =  -100;

MotorGas[3] = 100;
MotorPitch[3] =  -50;
MotorRoll[3] =  100;
MotorYaw[3] =  100;

MotorGas[4] = 95;
MotorPitch[4] = -50;
MotorRoll[4] =  -100;
MotorYaw[4] =  -100;

MotorGas[5] = 100;
MotorPitch[5] =  -50;
MotorRoll[5] =  -100;
MotorYaw[5] =  100;
}
#endif


void motor_axis_correction()
{
//int i;
for (byte i=0;i<LASTMOTOR;i++)
  {
  motorAxisCommandPitch[i] = (motorAxisCommand[PITCH] / 100.0) * MotorPitch[i];
  motorAxisCommandRoll[i] = (motorAxisCommand[ROLL] / 100.0) * MotorRoll[i];
  motorAxisCommandYaw[i] = (motorAxisCommand[YAW] / 100.0) * MotorYaw[i];
  }
}

//After that we can mix them together:
void motor_matrix_command()
{
//int i;
float valuemotor;
for (byte i=0;i<LASTMOTOR;i++)
  {
   valuemotor = ((Throttle* MotorGas[i])/100) + motorAxisCommandPitch[i] + motorAxisCommandYaw[i] + motorAxisCommandRoll[i];
   //valuemotor = Throttle + motorAxisCommandPitch[i] + motorAxisCommandYaw[i] + motorAxisCommandRoll[i]; // OLD VERSION WITHOUT GAS CONTROL ON Mixertable
   valuemotor = constrain(valuemotor, minAcro, MAXCOMMAND);
   motorCommand[i]=valuemotor;
  }
}


void matrix_debug()
{
#ifdef PRINT_MIXERTABLE
     Serial.println();
     Serial.println("--------------------------");
     Serial.println("        Motori Mixertable " );
     Serial.println("--------------------------");
     Serial.println();

/*
     Serial.print("AIL:");
     Serial.print(ch1);
     Serial.print(" ELE:");
     Serial.print(ch2);
*/
     Serial.print(" THR:");
     Serial.print(Throttle);
/*
     Serial.print(" YAW:");
     Serial.print(ch4);
     Serial.print(" AUX:");
     Serial.print(ch_aux);
     Serial.print(" AUX2:");
     Serial.print(ch_aux2);
  */
     Serial.println();
     Serial.print("CONTROL_ROLL:");
     Serial.print(motorAxisCommand[ROLL]);
     Serial.print(" CONTROL_PITCH:");
     Serial.print(motorAxisCommand[PITCH]);
     Serial.print(" CONTROL_YAW:");
     Serial.print(motorAxisCommand[YAW]);
//     Serial.print(" SONAR_VALUE:");
//     Serial.print(sonar_value);
//     Serial.print(" TARGET_SONAR_VALUE:");
//     Serial.print(target_sonar_altitude);
//     Serial.print(" ERR_SONAR_VALUE:");
//     Serial.print(err_altitude);
//     Serial.println();
//     Serial.print("latitude:");
//     Serial.print(GPS_np.Lattitude);
//     Serial.print(" longitude:");
//     Serial.print(GPS_np.Longitude);
//     Serial.print(" command gps roll:");
//     Serial.print(command_gps_roll);
//     Serial.print(" command gps pitch:");
//     Serial.print(command_gps_pitch);
//     Serial.print(" Lon_diff:");
//     Serial.print(Lon_diff);
//     Serial.print(" Lon_diff");
//     Serial.print(command_gps_pitch);

//     Serial.println();

//     Serial.print("AP MODE:");Serial.print((int)AP_mode);

#ifdef ARDUCOPTER
     Serial.print("AIL:");
     Serial.print(ch1);
     Serial.print(" ELE:");
     Serial.print(ch2);
     Serial.print(" THR:");
     Serial.print(ch3);
     Serial.print(" YAW:");
     Serial.print(ch4);
     Serial.print(" AUX:");
     Serial.print(ch_aux);
     Serial.print(" AUX2:");
     Serial.print(ch_aux2);
     Serial.println();
     Serial.print("CONTROL_ROLL:");
     Serial.print(control_roll);
     Serial.print(" CONTROL_PITCH:");
     Serial.print(control_pitch);
     Serial.print(" CONTROL_YAW:");
     Serial.print(control_yaw);
     Serial.print(" SONAR_VALUE:");
     Serial.print(sonar_value);
     Serial.print(" TARGET_SONAR_VALUE:");
     Serial.print(target_sonar_altitude);
     Serial.print(" ERR_SONAR_VALUE:");
     Serial.print(err_altitude);
     Serial.println();
     Serial.print("latitude:");
     Serial.print(GPS_np.Lattitude);
     Serial.print(" longitude:");
     Serial.print(GPS_np.Longitude);
     Serial.print(" command gps roll:");
     Serial.print(command_gps_roll);
     Serial.print(" command gps pitch:");
     Serial.print(command_gps_pitch);
     Serial.print(" Lon_diff:");
     Serial.print(Lon_diff);
     Serial.print(" Lon_diff");
     Serial.print(command_gps_pitch);

     Serial.println();

     Serial.print("AP MODE:");Serial.print((int)AP_mode);
#endif

#ifdef HEXARADIAL
     Serial.println();
     Serial.print((unsigned int)MotorI2C[5]);
     comma();
     Serial.print((unsigned int)MotorI2C[0]);
     comma();
     Serial.print((unsigned int)MotorI2C[1]);
     comma();
     Serial.println();
     Serial.print((unsigned int)MotorI2C[4]);
     comma();
     Serial.print((unsigned int)MotorI2C[3]);
     comma();
     Serial.println((unsigned int)MotorI2C[2]);
     Serial.println("---------------");
     Serial.println();
#endif

// Example of Hexa Coaxial.
//Configurazione Motori Hexa Coaxial

//Dietro 			GAS		NICK		        ROLL		        YAW
//Sopra 1         		64		-64			0			-64
//Sotto 2			76		-64			0			64

//Guardando l'Hexafox da dietro braccio  Sinistro
//Sopra 3			64		32			64			64
//Sotto 4			76		32			64			-64

//Guardando l'Hexafox da dietro braccio  Destro.

//Sopra 5			64		32			-64			64
//Sotto 6			76		32			-64			-64


#ifdef HEXACOAXIAL
     Serial.println();
     Serial.print((unsigned int)MotorI2C[2]);
     comma();
     Serial.print((unsigned int)MotorI2C[4]);
     Serial.println();
     //comma();
     Serial.print((unsigned int)MotorI2C[3]);
     comma();
     Serial.print((unsigned int)MotorI2C[5]);
     Serial.println();
     Serial.print ("   ");
     //comma();
     Serial.print((unsigned int)MotorI2C[0]);
     Serial.println();
     Serial.print ("   ");
     //comma();
     Serial.println((unsigned int)MotorI2C[1]);
     Serial.println("---------------");
     Serial.println();
#endif

#endif
}

void WireMotorWrite()
{
int i = 0;
//int nmotor=0;
int index=0;
int tout=0;
char buff_i2c[10];

Wire.endTransmission(); //end transmission
for(byte nmotor=0;nmotor<6;nmotor++)
  {
  index=0x29+nmotor;
  Wire.beginTransmission(index);
  Wire.send(MotorI2C[nmotor]);
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(index, 1);    // request 6 bytes from device
  i=0;
  while(1)
  //while((Wire.available())&&(i<6))
  {
    buff_i2c[i] = Wire.receive();  // receive one byte
    i++;
    if (i>6)break;
    //Serial.print(i);
    if (Wire.available()==0)break;
  }

  }

}
  void write (void) {
    // Matrix transformation.
    motor_axis_correction();
    // Matrix Command.
    motor_matrix_command();
   // Matrix Assignment.
    MotorI2C[MOTORID1]=(char)((motorCommand[0] * mMotorCommand) + bMotorCommand);
    MotorI2C[MOTORID2]=(char)((motorCommand[1] * mMotorCommand) + bMotorCommand);
    MotorI2C[MOTORID3]=(char)((motorCommand[2] * mMotorCommand) + bMotorCommand);
    MotorI2C[MOTORID4]=(char)((motorCommand[3] * mMotorCommand) + bMotorCommand);
    MotorI2C[MOTORID5]=(char)((motorCommand[4] * mMotorCommand) + bMotorCommand);
    MotorI2C[MOTORID6]=(char)((motorCommand[5] * mMotorCommand) + bMotorCommand);

   if((millis()-timer_debug)>1000)   // 100ms => 10 Hz loop rate
  {
    timer_debug=millis();
    #ifdef TELEMETRY_DEBUG
    matrix_debug();
    #endif
}


    WireMotorWrite();
}

  void commandAllMotors(int _motorCommand) {   // Sends commands to all motors

    // Matrix transformation.
    motor_axis_correction();
    // Matrix Command.
    motor_matrix_command();
   // Matrix Assignment.
    MotorI2C[MOTORID1]=(char)((_motorCommand * mMotorCommand) + bMotorCommand);
    MotorI2C[MOTORID2]=(char)((_motorCommand * mMotorCommand) + bMotorCommand);
    MotorI2C[MOTORID3]=(char)((_motorCommand * mMotorCommand) + bMotorCommand);
    MotorI2C[MOTORID4]=(char)((_motorCommand * mMotorCommand) + bMotorCommand);
    MotorI2C[MOTORID5]=(char)((_motorCommand * mMotorCommand) + bMotorCommand);
    MotorI2C[MOTORID6]=(char)((_motorCommand * mMotorCommand) + bMotorCommand);
    matrix_debug();
    WireMotorWrite();
  }
};
#endif

/******************************************************/
/********************* I2C Motors *********************/
/******************************************************/
// Tested AeroQuad I2C class authored by jihlein
// http://code.google.com/p/aeroquad/issues/detail?id=67
class Motors_AeroQuadI2C : public Motors {
private:
  #define MOTORBASE 0x28            // I2C controller base address

  #define FRONTMOTORID MOTORBASE + 1  // define I2C controller addresses per your configuration
  #define REARMOTORID  MOTORBASE + 3  // these addresses are for Phifun controllers
  #define RIGHTMOTORID MOTORBASE + 2  // as installed on jihlein's homebrew AeroQuad 3.0
  #define LEFTMOTORID  MOTORBASE + 4  // inspired frame

  public:
  Motors_AeroQuadI2C() : Motors(){
    // Scale motor commands to 0 to 255
    // for I2C commands
    // m = (255 - 0)/(2000-1000) = 0.255
    // b = y1 - (m * x1) = 0 - (0.255 * 1000) = -255
    mMotorCommand = 0.255;
    bMotorCommand = -255.0;
  }

  void initialize(void)
  {
    sendByteI2C(FRONTMOTORID, 0);
    sendByteI2C(REARMOTORID,  0);
    sendByteI2C(RIGHTMOTORID, 0);
    sendByteI2C(LEFTMOTORID,  0);
  }

  void write(void)
  {
    sendByteI2C(FRONTMOTORID, constrain((motorCommand[FRONT] * mMotorCommand) + bMotorCommand, 0, 255));
    sendByteI2C(REARMOTORID,  constrain((motorCommand[REAR]  * mMotorCommand) + bMotorCommand, 0, 255));
    sendByteI2C(RIGHTMOTORID, constrain((motorCommand[RIGHT] * mMotorCommand) + bMotorCommand, 0, 255));
    sendByteI2C(LEFTMOTORID,  constrain((motorCommand[LEFT]  * mMotorCommand) + bMotorCommand, 0, 255));
  }

  void commandAllMotors(int motorCommand)
  {
    sendByteI2C(FRONTMOTORID, constrain((motorCommand * mMotorCommand) + bMotorCommand, 0, 255));
    sendByteI2C(REARMOTORID,  constrain((motorCommand * mMotorCommand) + bMotorCommand, 0, 255));
    sendByteI2C(RIGHTMOTORID, constrain((motorCommand * mMotorCommand) + bMotorCommand, 0, 255));
    sendByteI2C(LEFTMOTORID,  constrain((motorCommand * mMotorCommand) + bMotorCommand, 0, 255));
  }
};

