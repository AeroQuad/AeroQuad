/*
  AeroQuad v2.4.1 - June 2011
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

class  Receiver {
public:
  int receiverData[LASTCHANNEL];
  int transmitterCommand[LASTCHANNEL];
  int transmitterCommandSmooth[LASTCHANNEL];
  int transmitterZero[3];
  int transmitterTrim[3];
  // Controls the strength of the commands sent from the transmitter
  // xmitFactor ranges from 0.01 - 1.0 (0.01 = weakest, 1.0 - strongest)
  float xmitFactor; // Read in from EEPROM
  float transmitterSmooth[LASTCHANNEL];
  float mTransmitter[LASTCHANNEL];
  float bTransmitter[LASTCHANNEL];
  //unsigned long currentTime, previousTime;

  Receiver(void) {
    transmitterCommand[ROLL] = 1500;
    transmitterCommand[PITCH] = 1500;
    transmitterCommand[YAW] = 1500;
    transmitterCommand[THROTTLE] = 1000;
    transmitterCommand[MODE] = 1000;
    transmitterCommand[AUX] = 1000;

    for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
      transmitterCommandSmooth[channel] = 1.0;
    for (byte channel = ROLL; channel < THROTTLE; channel++)
      transmitterZero[channel] = 1500;
  }

  // ******************************************************************
  // The following function calls must be defined in any new subclasses
  // ******************************************************************
  virtual void initialize(void);
  virtual void read(void);

  // **************************************************************
  // The following functions are common between all Gyro subclasses
  // **************************************************************

  void _initialize(void) {
    xmitFactor = readFloat(XMITFACTOR_ADR);
    
    mTransmitter[0] = readFloat(RECEIVER_CHANNEL_0_SLOPE_ADR);
    bTransmitter[0] = readFloat(RECEIVER_CHANNEL_0_OFFSET_ADR);
    transmitterSmooth[0] = readFloat(RECEIVER_CHANNEL_0_SMOOTH_FACTOR_ADR);
    mTransmitter[1] = readFloat(RECEIVER_CHANNEL_1_SLOPE_ADR);
    bTransmitter[1] = readFloat(RECEIVER_CHANNEL_1_OFFSET_ADR);
    transmitterSmooth[1] = readFloat(RECEIVER_CHANNEL_1_SMOOTH_FACTOR_ADR);
    mTransmitter[2] = readFloat(RECEIVER_CHANNEL_2_SLOPE_ADR);
    bTransmitter[2] = readFloat(RECEIVER_CHANNEL_2_OFFSET_ADR);
    transmitterSmooth[2] = readFloat(RECEIVER_CHANNEL_2_SMOOTH_FACTOR_ADR);
    mTransmitter[3] = readFloat(RECEIVER_CHANNEL_3_SLOPE_ADR);
    bTransmitter[3] = readFloat(RECEIVER_CHANNEL_3_OFFSET_ADR);
    transmitterSmooth[3] = readFloat(RECEIVER_CHANNEL_3_SMOOTH_FACTOR_ADR);
    mTransmitter[4] = readFloat(RECEIVER_CHANNEL_4_SLOPE_ADR);
    bTransmitter[4] = readFloat(RECEIVER_CHANNEL_4_OFFSET_ADR);
    transmitterSmooth[4] = readFloat(RECEIVER_CHANNEL_4_SMOOTH_FACTOR_ADR);
    mTransmitter[5] = readFloat(RECEIVER_CHANNEL_5_SLOPE_ADR);
    bTransmitter[5] = readFloat(RECEIVER_CHANNEL_5_OFFSET_ADR);
    transmitterSmooth[5] = readFloat(RECEIVER_CHANNEL_5_SMOOTH_FACTOR_ADR);
  }

  // returns non-smoothed non-scaled ADC data in PWM full range 1000-2000 values
  const int getRaw(byte channel) {
    return receiverData[channel];
  }
  
  // returns raw but smoothed receiver(channel) in PWM
  const int getRawSmoothed(byte channel) {
    return transmitterCommandSmooth[channel];
  }
 
   // returns smoothed & scaled receiver(channel) in PWM values, zero centered
  const int getData(byte channel) {
    return transmitterCommand[channel];
  }
  
  // return the smoothed & scaled number of radians/sec in stick movement - zero centered
  const float getSIData(byte channel) {
    // 2.3 Original
    return ((transmitterCommand[channel] - transmitterZero[channel]) * (2.5 * PWM2RAD));  // +/- 2.5RPS 50% of full rate
    // 2.3 Stable
    //return ((transmitterCommand[channel] - transmitterZero[channel]) * (5.0 * PWM2RAD));  // +/- 5RPS factored by xmitfactor of full rate
  }

  const int getTrimData(byte channel) {
    return receiverData[channel] - transmitterTrim[channel];
  }

  // returns Zero value of channel in PWM
  const int getZero(byte channel) {
    return transmitterZero[channel];
  }
  // sets zero value of channel in PWM
  void setZero(byte channel, int value) {
    transmitterZero[channel] = value;
  }

  const int getTransmitterTrim(byte channel) {
    return transmitterTrim[channel];
  }

  void setTransmitterTrim(byte channel, int value) {
    transmitterTrim[channel] = value;
  }

  const float getSmoothFactor(byte channel) {
    return transmitterSmooth[channel];
  }

  void setSmoothFactor(byte channel, float value) {
    transmitterSmooth[channel] = value;
  }

  const float getXmitFactor(void) {
    return xmitFactor;
  }

  void setXmitFactor(float value) {
    xmitFactor = value;
  }

  const float getTransmitterSlope(byte channel) {
    return mTransmitter[channel];
  }

  void setTransmitterSlope(byte channel, float value) {
    mTransmitter[channel] = value;
  }

  const float getTransmitterOffset(byte channel) {
    return bTransmitter[channel];
  }

  void setTransmitterOffset(byte channel, float value) {
    bTransmitter[channel] = value;
  }

  const float getAngle(byte channel) {
    // Scale 1000-2000 usecs to -45 to 45 degrees
    // m = 0.09, b = -135
    return (0.09 * transmitterCommand[channel]) - 135;
    //return (0.09 * receiverData[channel]) - 135;
  }
};

/*************************************************/
/*************** AeroQuad PCINT ******************/
/*************************************************/
#if defined(AeroQuad_v1) || defined(AeroQuad_v18) || defined(AeroQuad_Mini) || defined(AeroQuad_Wii) || defined(AeroQuad_v1_IDG)
volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};
volatile static uint8_t PCintLast[3];
// Channel data
typedef struct {
  byte edge;
  unsigned long riseTime;
  unsigned long fallTime;
  unsigned int  lastGoodWidth;
} tPinTimingData;
volatile static tPinTimingData pinData[9];

// Attaches PCINT to Arduino Pin
void attachPinChangeInterrupt(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  }
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }
  // set the mask
  *pcmask |= bit;
  // enable the interrupt
  PCICR |= 0x01 << port;
}

// ISR which records time of rising or falling edge of signal
static void measurePulseWidthISR(uint8_t port, uint8_t pinoffset) {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;
  uint32_t currentTime;
  uint32_t time;

  // get the pin states for the indicated port.
  curr = *portInputRegister(port+2);
  mask = curr ^ PCintLast[port];
  PCintLast[port] = curr;
  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= *port_to_pcmask[port]) == 0) {
    return;
  }
  currentTime = micros();
  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = pinoffset + i;
      // for each pin changed, record time of change
      if (bit & PCintLast[port]) {
        time = currentTime - pinData[pin].fallTime;
        pinData[pin].riseTime = currentTime;
        if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
          pinData[pin].edge = RISING_EDGE;
        else
          pinData[pin].edge = FALLING_EDGE; // invalid rising edge detected
      }
      else {
        time = currentTime - pinData[pin].riseTime;
        pinData[pin].fallTime = currentTime;
        if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[pin].edge == RISING_EDGE)) {
          pinData[pin].lastGoodWidth = time;
          pinData[pin].edge = FALLING_EDGE;
        }
      }
    }
  }
}

SIGNAL(PCINT0_vect) {
  measurePulseWidthISR(0, 8); // PORT B
}

SIGNAL(PCINT2_vect) {
  measurePulseWidthISR(2, 0); // PORT D
}

#ifdef AeroQuad_Paris_v3
// defines arduino pins used for receiver in arduino pin numbering schema
static byte receiverPin[6] = {4, 5, 6, 2, 7, 8}; // pins used for ROLL, PITCH, YAW, THROTTLE, MODE, AUX
#else
// defines arduino pins used for receiver in arduino pin numbering schema
static byte receiverPin[6] = {2, 5, 6, 4, 7, 8}; // pins used for ROLL, PITCH, YAW, THROTTLE, MODE, AUX
#endif

class Receiver_AeroQuad : public Receiver {
public:
  // Configure each receiver pin for PCINT
  void initialize() {
    this->_initialize(); // load in calibration xmitFactor from EEPROM
    for (byte channel = ROLL; channel < LASTCHANNEL; channel++) {
      pinMode(receiverPin[channel], INPUT);
      pinData[receiverPin[channel]].edge = FALLING_EDGE;
      attachPinChangeInterrupt(receiverPin[channel]);
    }
  }

  // Calculate PWM pulse width of receiver data
  // If invalid PWM measured, use last known good time
  void read(void) {
    for(byte channel = ROLL; channel < LASTCHANNEL; channel++) {
      byte pin = receiverPin[channel];
      uint8_t oldSREG = SREG;
      cli();
      // Get receiver value read by pin change interrupt handler
      uint16_t lastGoodWidth = pinData[pin].lastGoodWidth;
      SREG = oldSREG;

      // Apply transmitter calibration adjustment
      receiverData[channel] = (mTransmitter[channel] * lastGoodWidth) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs
      transmitterCommandSmooth[channel] = filterSmooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
    }

    // Reduce transmitter commands using xmitFactor and center around 1500
    for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
      if (channel < THROTTLE)
        transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
      else
    // No xmitFactor reduction applied for throttle, mode and
    //for (byte channel = THROTTLE; channel < LASTCHANNEL; channel++)
        transmitterCommand[channel] = transmitterCommandSmooth[channel];
  }
};
#endif

/******************************************************/
/*************** AeroQuad Mega PCINT ******************/
/******************************************************/
#if defined(AeroQuadMega_v1) || defined(AeroQuadMega_v2) || defined(AeroQuadMega_Wii) || defined(AeroQuadMega_CHR6DM)
volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};
volatile static uint8_t PCintLast[3];
// Channel data
typedef struct {
  byte edge;
  unsigned long riseTime;
  unsigned long fallTime;
  unsigned int lastGoodWidth;
} tPinTimingData;
volatile static tPinTimingData pinData[LASTCHANNEL];

static void MegaPcIntISR() {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;
  uint32_t currentTime;
  uint32_t time;

  curr = *portInputRegister(11);
  mask = curr ^ PCintLast[0];
  PCintLast[0] = curr;

  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= PCMSK2) == 0) {
    return;
  }

  currentTime = micros();

  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = i;
      // for each pin changed, record time of change
      if (bit & PCintLast[0]) {
        time = currentTime - pinData[pin].fallTime;
        pinData[pin].riseTime = currentTime;
        if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
          pinData[pin].edge = RISING_EDGE;
        else
          pinData[pin].edge = FALLING_EDGE; // invalid rising edge detected
      }
      else {
        time = currentTime - pinData[pin].riseTime;
        pinData[pin].fallTime = currentTime;
        if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[pin].edge == RISING_EDGE)) {
          pinData[pin].lastGoodWidth = time;
          pinData[pin].edge = FALLING_EDGE;
        }
      }
    }
  }
}

SIGNAL(PCINT2_vect) {
  MegaPcIntISR();
}

#ifdef AeroQuadMega_v1
  // arduino pins 67, 65, 64, 66, 63, 62
  static byte receiverPin[6] = {5, 3, 2, 4, 1, 0}; // bit number of PORTK used for ROLL, PITCH, YAW, THROTTLE, MODE, AUX
#else
 //arduino pins 63, 64, 65, 62, 66, 67
  static byte receiverPin[6] = {1, 2, 3, 0, 4, 5}; // bit number of PORTK used for ROLL, PITCH, YAW, THROTTLE, MODE, AUX
#endif

class Receiver_AeroQuadMega : public Receiver {
public:
  void initialize() {
    this->_initialize(); // load in calibration xmitFactor from EEPROM
    DDRK = 0;
    PORTK = 0;
    PCMSK2 |= 0x3F;
    PCICR |= 0x1 << 2;

  for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
      pinData[receiverPin[channel]].edge = FALLING_EDGE;
  }

  // Calculate PWM pulse width of receiver data
  // If invalid PWM measured, use last known good time
  void read(void) {
    for(byte channel = ROLL; channel < LASTCHANNEL; channel++) {
      byte pin = receiverPin[channel];
      uint8_t oldSREG = SREG;
      cli();
      // Get receiver value read by pin change interrupt handler
      uint16_t lastGoodWidth = pinData[pin].lastGoodWidth;
      SREG = oldSREG;

      // Apply transmitter calibration adjustment
      receiverData[channel] = (mTransmitter[channel] * lastGoodWidth) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs
      transmitterCommandSmooth[channel] = filterSmooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
    }

    // Reduce transmitter commands using xmitFactor and center around 1500
    for (byte channel = ROLL; channel < THROTTLE; channel++)
      transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
    // No xmitFactor reduction applied for throttle, mode and AUX
    for (byte channel = THROTTLE; channel < LASTCHANNEL; channel++)
      transmitterCommand[channel] = transmitterCommandSmooth[channel];
  }
};

class Receiver_AeroQuadMega_Fake :
public Receiver {
private:

public:
  Receiver_AeroQuadMega_Fake() :
  Receiver(){
  }

  void initialize() {
    this->_initialize(); // load in calibration xmitFactor from EEPROM
    DDRK = 0;
    PORTK = 0;
    PCMSK2 |= 0x3F;
    PCICR |= 0x1 << 2;
  }

  // Calculate PWM pulse width of receiver data
  // If invalid PWM measured, use last known good time
  void read(void) {
    uint16_t data[6];
    uint8_t oldSREG;

    oldSREG = SREG;
    cli();
    // Buffer receiver values read from pin change interrupt handler
    for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
      data[channel] = 1500;
    SREG = oldSREG;

    for(byte channel = ROLL; channel < LASTCHANNEL; channel++) {
      //currentTime = micros();
      // Apply transmitter calibration adjustment
      receiverData[channel] = (mTransmitter[channel] * data[channel]) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs
      transmitterCommandSmooth[channel] = filterSmooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
      //transmitterCommandSmooth[channel] = transmitterFilter[channel].filter(receiverData[channel]);
      //previousTime = currentTime;
    }

    // Reduce transmitter commands using xmitFactor and center around 1500
    for (byte channel = ROLL; channel < THROTTLE; channel++)
      transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
    // No xmitFactor reduction applied for throttle, mode and AUX
    for (byte channel = THROTTLE; channel < LASTCHANNEL; channel++)
      transmitterCommand[channel] = transmitterCommandSmooth[channel];
  }
};
#endif

/*********************************************/
/********** ArduCopter PPM Input *************/
/*********************************************/
#if defined(ArduCopter) || defined(APM_OP_CHR6DM)
#include <avr/interrupt.h>
volatile unsigned int Start_Pulse = 0;
volatile unsigned int Stop_Pulse = 0;
volatile unsigned int Pulse_Width = 0;
volatile byte PPM_Counter=0;
volatile int PWM_RAW[8] = {
  2400,2400,2400,2400,2400,2400,2400,2400};

/****************************************************
 * Interrupt Vector
 ****************************************************/
ISR(TIMER4_CAPT_vect)//interrupt.
{
  if(((1<<ICES4)&TCCR4B) >= 0x01)
  {

    if(Start_Pulse>Stop_Pulse) //Checking if the Stop Pulse overflow the register, if yes i normalize it.
    {
      Stop_Pulse+=40000; //Nomarlizing the stop pulse.
    }
    Pulse_Width=Stop_Pulse-Start_Pulse; //Calculating pulse
    if(Pulse_Width>5000) //Verify if this is the sync pulse
    {
      PPM_Counter=0; //If yes restart the counter
    }
    else
    {
      //PWM_RAW[PPM_Counter]=Pulse_Width; //Saving pulse.
      PWM_RAW[PPM_Counter & 0x07]=Pulse_Width; //Saving pulse.
      PPM_Counter++;
    }
    Start_Pulse=ICR4;
    TCCR4B &=(~(1<<ICES4)); //Changing edge detector.
  }
  else
  {
    Stop_Pulse=ICR4; //Capturing time stop of the drop edge
    TCCR4B |=(1<<ICES4); //Changing edge detector.
    //TCCR4B &=(~(1<<ICES4));
  }
  //Counter++;
}
//#endif
class Receiver_ArduCopter : public Receiver {
private:
  int receiverPin[6];

public:
  Receiver_ArduCopter() :
  Receiver(){
    receiverPin[ROLL] = 0;
    receiverPin[PITCH] = 1;
    receiverPin[YAW] = 3;
    receiverPin[THROTTLE] = 2;
    receiverPin[MODE] = 4;
    receiverPin[AUX] = 5;
  }

  void initialize(void) {
    this->_initialize(); // load in calibration and xmitFactor from EEPROM
    /*Note that timer4 is configured to used the Input capture for PPM decoding and to pulse two servos
     OCR4A is used as the top counter*/
    pinMode(49, INPUT);
    pinMode(7,OUTPUT);
    pinMode(8,OUTPUT);
    //Remember the registers not declared here remains zero by default...
    TCCR4A =((1<<WGM40)|(1<<WGM41)|(1<<COM4C1)|(1<<COM4B1)|(1<<COM4A1));
    TCCR4B = ((1<<WGM43)|(1<<WGM42)|(1<<CS41)|(1<<ICES4)); //Prescaler set to 8, that give us a resolution of 2us, read page 134 of data sheet
    OCR4A = 40000; ///50hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000,
    //must be 50hz because is the servo standard (every 20 ms, and 1hz = 1sec) 1000ms/20ms=50hz, elementary school stuff...
    OCR4B = 3000; //PH4, OUT5
    OCR4C = 3000; //PH5, OUT4

    TIMSK4 |= (1<<ICIE4); //Timer interrupt mask
    sei();
  }

  void read(void) {
    for(byte channel = ROLL; channel < LASTCHANNEL; channel++) {
      //currentTime = micros();
      // Apply transmitter calibration adjustment
      receiverData[channel] = (mTransmitter[channel] * ((PWM_RAW[receiverPin[channel]]+600)/2)) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs
      transmitterCommandSmooth[channel] = filterSmooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
      //previousTime = currentTime;
    }

    // Reduce transmitter commands using xmitFactor and center around 1500
    for (byte channel = ROLL; channel < THROTTLE; channel++)
      transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
    // No xmitFactor reduction applied for throttle, mode and
    for (byte channel = THROTTLE; channel < LASTCHANNEL; channel++)
      transmitterCommand[channel] = transmitterCommandSmooth[channel];
  }
};
#endif

/*************************************************/
/*************** Multipilot PCINT ****************/
/*************************************************/
#if defined(Multipilot) || defined(MultipilotI2C)

#define ROLLCH 4
#define PITCHCH 3
#define YAWCH 1
#define THROTTLECH 2
#define MODECH 8
#define AUXCH  7
#define CAMERAROLLCH 5
#define CAMERAPITCHCH 6

class Receiver_Multipilot :
public Receiver {
private:
  int receiverChannel[LASTCHANNEL];

public:
  Receiver_Multipilot() :
  Receiver(){
    receiverChannel[ROLL] = ROLLCH;
    receiverChannel[PITCH] = PITCHCH;
    receiverChannel[YAW] = YAWCH;
    receiverChannel[THROTTLE] = THROTTLECH;
    receiverChannel[MODE] = MODECH;
    receiverChannel[AUX] = AUXCH;

  }

  // Configure each receiver pin for PCINT
  void initialize() {
    this->_initialize(); // load in calibration xmitFactor from EEPROM
    ServoDecode.begin();
    ServoDecode.setFailsafe(3,1234); // set channel 3 failsafe pulse  width
  }

  // Calculate PWM pulse width of receiver data
  // If invalid PWM measured, use last known good time
  void read(void) {
    uint16_t data[6];

    if(ServoDecode.getState()!= READY_state)
    {
      for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
      {
        safetyCheck=0;
        data[channel]=5000;
      }
    }
    else
    {
      data[ROLL] = ServoDecode.GetChannelPulseWidth((int)receiverChannel[ROLL]);
      data[PITCH] = ServoDecode.GetChannelPulseWidth((int)receiverChannel[PITCH]);
      data[THROTTLE] = ServoDecode.GetChannelPulseWidth((int)receiverChannel[THROTTLE]);
      data[YAW] = ServoDecode.GetChannelPulseWidth((int)receiverChannel[YAW]);
      data[MODE] = ServoDecode.GetChannelPulseWidth((int)receiverChannel[MODE]);
      data[AUX] = ServoDecode.GetChannelPulseWidth((int)receiverChannel[AUX]);
      safetyCheck=1;
    }


    for(byte channel = ROLL; channel < LASTCHANNEL; channel++) {
      //currentTime = micros();
      // Apply transmitter calibration adjustment
      receiverData[channel] = (mTransmitter[channel] * data[channel]) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs
      transmitterCommandSmooth[channel] = filterSmooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
      //previousTime = currentTime;
    }

    // Reduce transmitter commands using xmitFactor and center around 1500
    for (byte channel = ROLL; channel < THROTTLE; channel++)
      transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
    //transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel])) + transmitterZero[channel];
    // No xmitFactor reduction applied for throttle, mode and
    for (byte channel = THROTTLE; channel < LASTCHANNEL; channel++)
      transmitterCommand[channel] = transmitterCommandSmooth[channel];

  }
};
#endif



