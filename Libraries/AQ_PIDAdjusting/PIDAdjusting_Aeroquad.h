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

#ifndef _AEROQUAD_PIDADJUSTING_H_
#define _AEROQUAD_PIDADJUSTING_H_


#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // used only on mega for now

#if defined PIDAdjusting && defined PIDAdjustingSwitchChannel && defined PIDAdjustingPot1PID1
	int use1 = 1;
#else
	int use1 = 0;
#endif

#if defined PIDAdjusting && defined PIDAdjustingSwitchChannel && defined PIDAdjustingPot2PID1
	int use2 = 1;
#else
	int use2 = 0;
#endif

// PID Variables
struct PIDold {
  float P, I, D;
  float lastPosition;
  // AKA experiments with PID
  float previousPIDTime;
  float integratedError;
  float windupGuard; // Thinking about having individual wind up guards for each PID
} PIDold[10];

float PIDAdj1 = 0.00;
float PIDAdj2 = 0.00;


int mapping(float var , float in_min, float in_max, float out_min, float out_max) {
 return float (var - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void PIDAdjusting_store() {
  for (byte axis = 0; axis <= 9; axis++) {				//store original PID-Data
    PIDold[axis].P = PID[axis].P;
	PIDold[axis].I = PID[axis].I;
	PIDold[axis].D = PID[axis].D;
	PIDold[axis].lastPosition = PID[axis].lastPosition;
    PIDold[axis].previousPIDTime = PID[axis].previousPIDTime;
    PIDold[axis].integratedError = PID[axis].integratedError;
    PIDold[axis].windupGuard = PID[axis].windupGuard;
  }
}

void PIDAdjusting_restore() {
  for (byte axis = 0; axis <= 9; axis++) {				//store original PID-Data
    PID[axis].P = PIDold[axis].P;
	PID[axis].I = PIDold[axis].I;
	PID[axis].D = PIDold[axis].D;
	PID[axis].lastPosition = PIDold[axis].lastPosition;
    PID[axis].previousPIDTime = PIDold[axis].previousPIDTime;
    PID[axis].integratedError = PIDold[axis].integratedError;
    PID[axis].windupGuard = PIDold[axis].windupGuard;
  }
}

void initializePIDADJUSTING() {
  PIDAdjusting_store();
}

void PIDAdjusting_update() {
	if (receiverCommand[PIDAdjustingSwitchChannel] > PIDAdjustingSwitchMin
		&& receiverCommand[PIDAdjustingSwitchChannel] < PIDAdjustingSwitchMax) {							//Switched on?
		//Map Receiver input to potis
	    PIDAdj1 = mapping(receiverCommand[PIDAdjustingPot1Channel] , 1000.0, 2000.0 ,PIDAdjustingPot1Min * pow(10,PIDAdjustingPot1Decimals)  , PIDAdjustingPot1Max * pow(10,PIDAdjustingPot1Decimals));
		PIDAdj1 = PIDAdj1  / pow(10,PIDAdjustingPot1Decimals);
		#if defined PIDAdjustingPot2Channel
	    PIDAdj2 = mapping(receiverCommand[PIDAdjustingPot2Channel], 1000.0, 2000.0 , PIDAdjustingPot2Min * pow(10,PIDAdjustingPot2Decimals) , PIDAdjustingPot2Max *  pow(10,PIDAdjustingPot2Decimals));
		PIDAdj2 = PIDAdj2 / pow(10,PIDAdjustingPot2Decimals);
		#endif
		//channel 1
		if (use1 == 1) {
			PID[PIDAdjustingPot1PID1].PIDAdjustingPot1PID1Type = PIDAdj1;
			#if defined PIDAdjustingPot1PID2
				PID[PIDAdjustingPot1PID2].PIDAdjustingPot1PID2Type = PIDAdj1;
			#endif
			}
		//channel 2
		#if defined PIDAdjustingPot2Channel
		if (use2 == 1) {
			PID[PIDAdjustingPot2PID1].PIDAdjustingPot2PID1Type = PIDAdj2;
			#if defined PIDAdjustingPot2PID2
				PID[PIDAdjustingPot2PID2].PIDAdjustingPot2PID2Type = PIDAdj2;
			#endif
		}
		#endif
	} else {
	  PIDAdjusting_restore();     //restore original data
	}

}


#endif  // #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#endif  // #define _AEROQUAD_PIDADJUSTING_H_
