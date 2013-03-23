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


#ifndef _AEROQUAD_FLIGHT_CONFIG_MEGA_H_
#define _AEROQUAD_FLIGHT_CONFIG_MEGA_H_

#include "Arduino.h"

#include "FlightControlQuadX.h"
#include "FlightControlQuadPlus.h"
#include "FlightControlHexPlus.h"
#include "FlightControlHexX.h"
#include "FlightControlTri.h"
#include "FlightControlQuadY4.h"
#include "FlightControlHexY6.h"
#include "FlightControlOctoX8.h"
#include "FlightControlOctoPlus.h"
#include "FlightControlOctoX.h"

functionPtr applyMotorCommand[] = {applyMotorCommandQuadX,
                                   applyMotorCommandQuadPlus,
                                   applyMotorCommandHexPlus,
                                   applyMotorCommandHexX,
                                   applyMotorCommandTri,
                                   applyMotorCommandY4,
                                   applyMotorCommandY6,
								   applyMotorCommandX8,
								   applyMotorCommandOctoPlus,
								   applyMotorCommandOctoX};

#endif








