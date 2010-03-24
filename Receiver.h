/*
  AeroQuad v1.7.1 - March 2010
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

#include "pins_arduino.h"

// Receiver pin definitions
// To pick your own PCINT pins look at page 2 of Atmega 328 data sheet and the Duemilanove data sheet and match the PCINT pin with the Arduino pinout
// These pins need to correspond to the ROLL/PITCH/YAW/THROTTLE/MODE/AUXPIN below
// Pin 2=18, Pin 3=19, Pin 4=20, Pin 5=21, Pin 6=22, Pin 7=23
#ifndef Mega_AQ1x
  #define ROLLPIN 2
  #define PITCHPIN 5
  #define YAWPIN 6
  #define THROTTLEPIN 4
  #define MODEPIN 7
  #define AUXPIN 8
  int receiverPin[6] = {18, 21, 22, 20, 23, 0}; // defines ATmega328P pins (Arduino pins converted to ATmega328P pinouts)
#endif
#ifdef Mega_AQ1x //Receiver pin assignments for the Arduino Mega using an AeroQuad v1.x Shield
  //The defines below are for documentation only of the Mega receiver input
  //The real pin assignments happen in initializeMegaPcInt2()
  //If you are using an AQ 1.x Shield, put a jumper wire between the Shield and Mega as indicated in the comments below
  #define ROLLPIN 67 // AI13, Place jumper between AQ Shield pin 2 and Mega AI13
  #define PITCHPIN 65 // AI11, Place jumper between AQ Shield pin 5 and Mega AI11
  #define YAWPIN 64 // AI10, Place jumper between AQ Shield pin 6 and Mega AI10
  #define THROTTLEPIN 66 // AI12, Place jumper between AQ Shield pin 4 and Mega AI12
  #define MODEPIN 63 // AI9, Place jumper between AQ Shield pin 7 and Mega AI9
  #define AUXPIN 62 // AI8, Place jumper between AQ Shield 8 and Mega AI8
  int receiverPin[6] = {5,3,2,4,1,0};
#endif

int receiverChannel[6] = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, MODEPIN, AUXPIN}; // defines Arduino pins

// Receiver variables
#define TIMEOUT 25000
#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000
#define MINDELTA 200
#define MINCHECK MINCOMMAND + 100
#define MAXCHECK MAXCOMMAND - 100
#define MINTHROTTLE MINCOMMAND + 100
#define LEVELOFF 100
#define ROLL 0
#define PITCH 1
#define YAW 2
#define THROTTLE 3
#define MODE 4
#define AUX 5
#define LASTCHANNEL 6

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000

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

int receiverData[6];
int transmitterCommand[6] = {1500,1500,1500,1000,1000,1000};
int transmitterCommandSmooth[6] = {0,0,0,0,0,0};
int transmitterZero[3] = {1500,1500,1500};
int transmitterCenter[3] = {1500,1500,1500};
byte channel;
// Controls the strength of the commands sent from the transmitter
// xmitFactor ranges from 0.01 - 1.0 (0.01 = weakest, 1.0 - strongest)
float xmitFactor; // Read in from EEPROM
// This scale not fully implemented, kept for future use
float mTransmitter[6] = {1,1,1,1,1,1};
float bTransmitter[6] = {0,0,0,0,0,0};

#endif
