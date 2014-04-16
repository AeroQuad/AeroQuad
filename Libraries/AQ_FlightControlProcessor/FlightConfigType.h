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


#ifndef _AEROQUAD_FLIGHT_CONFIG_H_
#define _AEROQUAD_FLIGHT_CONFIG_H_

#include "Arduino.h"

#define QUAD_X     0
#define QUAD_PLUS  1  
#define HEX_PLUS   2   
#define HEX_X      3      
#define TRI        4
#define QUAD_Y4    5    
#define HEX_Y6     6
#define OCTO_X8    7 
#define OCTO_PLUS  8	
#define OCTO_X	   9

volatile int8_t flightConfigType = QUAD_X;

#endif