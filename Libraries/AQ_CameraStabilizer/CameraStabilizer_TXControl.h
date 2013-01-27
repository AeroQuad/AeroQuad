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

#ifndef _AEROQUAD_CAMERA_STABILIZER_TXCONTROL_H_
#define _AEROQUAD_CAMERA_STABILIZER_TXCONTROL_H_

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined (AeroQuadSTM32)

#include "CameraStabilizer.h"

int servoCenterPitchDiff = 0;
int servoCenterPitchDesired = 0;
int servoTXChannels = 0;
int servoActualCenter = 0;

void processCameraTXControl()
{
    if (servoTXChannels == 1)
    {
        if (receiverCommand[AUX3] == 1500) // channel is centered -> move back to center
        {
            servoCenterPitchDiff = (servoActualCenter - servoCenterPitch) / 25;
            if ((servoCenterPitchDiff > 0) && (servoCenterPitchDiff < 1))
            {
                servoCenterPitchDiff = 1;
            }
            servoCenterPitch += servoCenterPitchDiff;
        }
        else
        {
            servoCenterPitchDesired = map(receiverCommand[AUX3],1000,2000,servoMaxPitch,servoMinPitch);
            if (servoCenterPitch != servoCenterPitchDesired)
            {
                servoCenterPitchDiff = (servoCenterPitchDesired - servoCenterPitch) / 25;
                if ((servoCenterPitchDiff > 0) && (servoCenterPitchDiff < 1))
                {
                    servoCenterPitchDiff = 1;
                }
                servoCenterPitch += servoCenterPitchDiff;
            }
        }
    }
    else if (servoTXChannels == 2)
    {
        if (receiverCommand[AUX4] == 1500) // override back to center
        {
            servoCenterPitchDiff = (servoActualCenter - servoCenterPitch) / 25;
            if ((servoCenterPitchDiff > 0) && (servoCenterPitchDiff < 1))
            {
                servoCenterPitchDiff = 1;
            }
            servoCenterPitch += servoCenterPitchDiff;
        }
        else // move up or down at a nice rate
        {
            if (receiverCommand[AUX3] != 1500) // map to up or down movement
            {
                servoCenterPitch += (1500 - receiverCommand[AUX3]) / 50;
            }
        }
    }
}
#endif
#endif
