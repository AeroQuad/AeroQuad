/*
  AeroQuad v1.8 - June 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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

#ifndef RECEIVER_H
#define RECEIVER_H

class  Receiver {
public:
  int receiverData[6];
  int transmitterCommand[6];
  int transmitterCommandSmooth[6];
  int transmitterZero[3];
  // Controls the strength of the commands sent from the transmitter
  // xmitFactor ranges from 0.01 - 1.0 (0.01 = weakest, 1.0 - strongest)
  float xmitFactor; // Read in from EEPROM
  float transmitterSmooth[6];
  float mTransmitter[6];
  float bTransmitter[6];

  Receiver(void) { 
    transmitterCommand[ROLL] = 1500;
    transmitterCommand[PITCH] = 1500;
    transmitterCommand[YAW] = 1500;
    transmitterCommand[THROTTLE] = 1000;
    transmitterCommand[MODE] = 1000;
    transmitterCommand[AUX] = 1000;
    
    for (channel = ROLL; channel < LASTCHANNEL; channel++)
      transmitterCommandSmooth[channel] = 0;
    for (channel = ROLL; channel < THROTTLE; channel++)
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
    mTransmitter[ROLL] = readFloat(ROLLSCALE_ADR);
    bTransmitter[ROLL] = readFloat(ROLLOFFSET_ADR);
    mTransmitter[PITCH] = readFloat(PITCHSCALE_ADR);
    bTransmitter[PITCH] = readFloat(PITCHOFFSET_ADR);
    mTransmitter[YAW] = readFloat(YAWSCALE_ADR);
    bTransmitter[YAW] = readFloat(YAWOFFSET_ADR);
    mTransmitter[THROTTLE] = readFloat(THROTTLESCALE_ADR);
    bTransmitter[THROTTLE] = readFloat(THROTTLEOFFSET_ADR);
    mTransmitter[MODE] = readFloat(MODESCALE_ADR);
    bTransmitter[MODE] = readFloat(MODEOFFSET_ADR);
    mTransmitter[AUX] = readFloat(AUXSCALE_ADR);
    bTransmitter[AUX] = readFloat(AUXOFFSET_ADR);
    
    transmitterSmooth[THROTTLE] = readFloat(THROTTLESMOOTH_ADR);
    transmitterSmooth[ROLL] = readFloat(ROLLSMOOTH_ADR);
    transmitterSmooth[PITCH] = readFloat(PITCHSMOOTH_ADR);
    transmitterSmooth[YAW] = readFloat(YAWSMOOTH_ADR);
    transmitterSmooth[MODE] = readFloat(MODESMOOTH_ADR);
    transmitterSmooth[AUX] = readFloat(AUXSMOOTH_ADR);
  }
  
  const int getRaw(byte channel) {
    return receiverData[channel];
  }
  
  const int getData(byte channel) {
    return transmitterCommand[channel];
  }
    
  const int getZero(byte channel) {
    return transmitterZero[channel];
  }
  
  void setZero(byte channel, int value) {
    transmitterZero[channel] = value;
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
};

/*************************************************/
/************** AeroQuad v1 PCINT ****************/
/*************************************************/
#ifdef AeroQuad_v1

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
  unsigned long lastGoodWidth;
} pinTimingData;  
volatile static pinTimingData pinData[24]; 

// Attaches PCINT to Arduino Pin
void attachPinChangeInterrupt(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t slot;
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
static void measurePulseWidthISR(uint8_t port) {
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
      pin = port * 8 + i;
      // for each pin changed, record time of change
      if (bit & PCintLast[port]) {
        time = currentTime - pinData[pin].fallTime;
        pinData[pin].riseTime = currentTime;        
        if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
          pinData[pin].edge = RISING_EDGE;
        else
          pinData[pin].edge == FALLING_EDGE; // invalid rising edge detected
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
  measurePulseWidthISR(0);
}
SIGNAL(PCINT1_vect) {
  measurePulseWidthISR(1);
}
SIGNAL(PCINT2_vect) {
  measurePulseWidthISR(2);
}

class Receiver_AeroQuad_v1 : public Receiver {
private:
  int receiverChannel[6];
  int receiverPin[6];

public:
  Receiver_AeroQuad_v1() : Receiver(){
    // Receiver pin definitions
    // To pick your own PCINT pins look at page 2 of Atmega 328 data sheet and the Duemilanove data sheet and match the PCINT pin with the Arduino pinout
    // These pins need to correspond to the ROLL/PITCH/YAW/THROTTLE/MODE/AUXPIN below
    // Pin 2=18, Pin 3=19, Pin 4=20, Pin 5=21, Pin 6=22, Pin 7=23
    receiverChannel[ROLL] = 2;
    receiverChannel[PITCH] = 5;
    receiverChannel[YAW] = 6;
    receiverChannel[THROTTLE] = 4;
    receiverChannel[MODE] = 7;
    receiverChannel[AUX] = 8;
    
    // defines ATmega328P pins (Arduino pins converted to ATmega328P pinouts)
    receiverPin[ROLL] = 18;
    receiverPin[PITCH] = 21;
    receiverPin[YAW] = 22;
    receiverPin[THROTTLE] = 20;
    receiverPin[MODE] = 23;
    receiverPin[AUX] = 0;
  }

  // Configure each receiver pin for PCINT
  void initialize() {
    this->_initialize(); // load in calibration xmitFactor from EEPROM
    for (channel = ROLL; channel < LASTCHANNEL; channel++) {
      pinMode(receiverChannel[channel], INPUT);
      pinData[receiverChannel[channel]].edge = FALLING_EDGE;
      attachPinChangeInterrupt(receiverChannel[channel]);
    }
  }

  // Calculate PWM pulse width of receiver data
  // If invalid PWM measured, use last known good time
  void read(void) {
    uint16_t data[6];
    uint8_t oldSREG;
      
    oldSREG = SREG;
    cli();
    // Buffer receiver values read from pin change interrupt handler
    //for (channel = ROLL; channel < LASTCHANNEL; channel++) {
    //  data[channel] = pinData[receiverPin[channel]].lastGoodWidth;
    //}
    data[ROLL] = pinData[receiverPin[ROLL]].lastGoodWidth;
    data[PITCH] = pinData[receiverPin[PITCH]].lastGoodWidth;
    data[THROTTLE] = pinData[receiverPin[THROTTLE]].lastGoodWidth;
    data[YAW] = pinData[receiverPin[YAW]].lastGoodWidth;
    data[MODE] = pinData[receiverPin[MODE]].lastGoodWidth;
    data[AUX] = pinData[receiverPin[AUX]].lastGoodWidth;
    SREG = oldSREG;  
    
    for(channel = ROLL; channel < LASTCHANNEL; channel++) {
      // Apply transmitter calibration adjustment
      receiverData[channel] = (mTransmitter[channel] * data[channel]) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs 
      transmitterCommandSmooth[channel] = smooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
    }
    // Reduce transmitter commands using xmitFactor and center around 1500
    for (channel = ROLL; channel < THROTTLE; channel++)
      transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
    // No xmitFactor reduction applied for throttle, mode and 
    for (channel = THROTTLE; channel < LASTCHANNEL; channel++)
      transmitterCommand[channel] = transmitterCommandSmooth[channel];
  }
};
#endif

/******************************************************/
/************** AeroQuad Mega v1 PCINT ****************/
/******************************************************/
#ifdef AeroQuadMega_v1

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
    unsigned long lastGoodWidth;
  } pinTimingData;  
  volatile static pinTimingData pinData[24]; 

  static void MegaPcIntISR() {
    uint8_t bit;
    uint8_t curr;
    uint8_t mask;
    uint8_t pin;
    uint32_t currentTime;
    uint32_t time;

    //curr = PORTK;
    curr = *portInputRegister(11);
    mask = curr ^ PCintLast[0];
    PCintLast[0] = curr;  

    //Serial.println(curr,DEC);

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
            //Serial.println(pinData[4].lastGoodWidth);
            pinData[pin].edge = FALLING_EDGE;
          } 
        }
      }
    }
  }

  SIGNAL(PCINT2_vect) {
    MegaPcIntISR();
  }
  
class Receiver_AeroQuadMega_v1 : public Receiver {
private:
  //Receiver pin assignments for the Arduino Mega using an AeroQuad v1.x Shield
  //The defines below are for documentation only of the Mega receiver input
  //The real pin assignments happen in initializeMegaPcInt2()
  //If you are using an AQ 1.x Shield, put a jumper wire between the Shield and Mega as indicated in the comments below
  #define ROLLPIN 67 // AI13, Place jumper between AQ Shield pin 2 and Mega AI13
  #define PITCHPIN 65 // AI11, Place jumper between AQ Shield pin 5 and Mega AI11
  #define YAWPIN 64 // AI10, Place jumper between AQ Shield pin 6 and Mega AI10
  #define THROTTLEPIN 66 // AI12, Place jumper between AQ Shield pin 4 and Mega AI12
  #define MODEPIN 63 // AI9, Place jumper between AQ Shield pin 7 and Mega AI9
  #define AUXPIN 62 // AI8, Place jumper between AQ Shield 8 and Mega AI8
  int receiverPin[6] = {5,3,2,4,1,0};  // Attaches PCINT to Arduino Pin
  
  public:
  Receiver_AeroQuadMega_v1() : Receiver(){}

  void initialize() {
    DDRK = 0;
    PORTK = 0;
    PCMSK2 |= 0x3F;
    PCICR |= 0x1 << 2;

    for (channel = ROLL; channel < LASTCHANNEL; channel++)
      pinData[receiverChannel[channel]].edge = FALLING_EDGE;
  }
  
  // Calculate PWM pulse width of receiver data
  // If invalid PWM measured, use last known good time
  void read(void) {
    uint16_t data[6];
    uint8_t oldSREG;
      
    oldSREG = SREG;
    cli();
    // Buffer receiver values read from pin change interrupt handler
    for (channel = ROLL; channel < LASTCHANNEL; channel++)
      data[channel] = pinData[receiverPin[channel]].lastGoodWidth;
    SREG = oldSREG;  
    
    for(channel = ROLL; channel < LASTCHANNEL; channel++) {
      // Apply transmitter calibration adjustment
      receiverData[channel] = (mTransmitter[channel] * data[channel]) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs 
      transmitterCommandSmooth[channel] = smooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
    }
    // Reduce transmitter commands using xmitFactor and center around 1500
    for (channel = ROLL; channel < THROTTLE; channel ++)
      transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
    // No xmitFactor reduction applied for throttle, mode and 
    for (channel = THROTTLE; channel < LASTCHANNEL; channel++)
      transmitterCommand[channel] = transmitterCommandSmooth[channel];
  }
};
#endif

/*********************************************/
/************** APM PPM Input ****************/
/*********************************************/
#ifdef APM

  #include <avr/interrupt.h>
  volatile unsigned int Start_Pulse = 0;
  volatile unsigned int Stop_Pulse = 0;
  volatile unsigned int Pulse_Width = 0;
  volatile byte PPM_Counter=0;
  volatile int PWM_RAW[8] = {2400,2400,2400,2400,2400,2400,2400,2400};
  
  /****************************************************
    Interrupt Vector
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
          PWM_RAW[PPM_Counter]=Pulse_Width; //Saving pulse. 
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

class Receiver_APM : public Receiver {
private:
  int receiverPin[6];

public:
  Receiver_APM() : Receiver(){
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
    for(channel = ROLL; channel < LASTCHANNEL; channel++) {
      // Apply transmitter calibration adjustment
      receiverData[channel] = (mTransmitter[channel] * ((PWM_RAW[receiverPin[channel]]+600)/2)) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs 
      transmitterCommandSmooth[channel] = smooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
    }
    // Reduce transmitter commands using xmitFactor and center around 1500
    for (channel = ROLL; channel < THROTTLE; channel++)
      transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
    // No xmitFactor reduction applied for throttle, mode and 
    for (channel = THROTTLE; channel < LASTCHANNEL; channel++)
      transmitterCommand[channel] = transmitterCommandSmooth[channel];
  }
};
#endif
#endif
