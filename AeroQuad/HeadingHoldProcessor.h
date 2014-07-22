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


#ifndef _AQ_HEADING_CONTROL_PROCESSOR_H_
#define _AQ_HEADING_CONTROL_PROCESSOR_H_


//float setHeading = 0;

void processHeading()
{
//  heading = degrees(gyroHeading);
//
//  relativeHeading = heading - setHeading;
//  if (heading <= (setHeading - 180)) {
//    relativeHeading += 360;
//  }
//  else if (heading >= (setHeading + 180)) {
//    relativeHeading -= 360;
//  }
//  
//  if (inFlight) { 
//    
//    if ((receiverCommand[receiverChannelMap[ZAXIS]] > (MIDCOMMAND + 25)) || 
//        (receiverCommand[receiverChannelMap[ZAXIS]] < (MIDCOMMAND - 25))) {
//      
//      setHeading = heading;
//      headingHold = 0;
//      PID[HEADING_HOLD_PID_IDX].integratedError = 0;
//    }
//    else {
//      headingHold = updatePID(0, relativeHeading, &PID[HEADING_HOLD_PID_IDX]);
//    }
//  }
//  else {
//    setHeading = heading;
//    headingHold = 0;
//    PID[HEADING_HOLD_PID_IDX].integratedError = 0;
//  }
//
//  const float receiverSiData = (receiverCommand[receiverChannelMap[ZAXIS]] - 1500) * (2.5 * PWM2RAD);
//  const float commandedYaw = constrain(receiverSiData + radians(headingHold), -PI, PI);
////  Serial.println(headingHold);
//  motorAxisCommandYaw = updatePID(commandedYaw, gyroRate[ZAXIS], &PID[ZAXIS_PID_IDX]);
//  Serial.println(motorAxisCommandYaw);


  if (!inFlight) {
    PID[ZAXIS_PID_IDX].integratedError = 0;
  }  

  int userYawCommand = 0.0;
  if ((receiverCommand[receiverChannelMap[ZAXIS]] > (MIDCOMMAND + 15)) || 
      (receiverCommand[receiverChannelMap[ZAXIS]] < (MIDCOMMAND - 15))) {
        userYawCommand = map(receiverCommand[receiverChannelMap[ZAXIS]] - 1500, -500 , 500, -gyroOneMeterSecADCFactor, gyroOneMeterSecADCFactor) * 
                         map((abs(receiverCommand[receiverChannelMap[ZAXIS]] - 1500)), 0 , 500, 100, yawSpeedFactor*100) / 100.0;
  }
  
  motorAxisCommandYaw = updatePID(userYawCommand, gyroADC[ZAXIS], &PID[ZAXIS_PID_IDX]);
}

#endif






