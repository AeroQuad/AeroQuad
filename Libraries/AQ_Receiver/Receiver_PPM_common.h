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

#ifndef _AEROQUAD_RECEIVER_PPM_COMMON_H_
#define _AEROQUAD_RECEIVER_PPM_COMMON_H_

#define PPM_CHANNELS 10

#define SERIAL_SUM_PPM_1         1,2,3,0,4,5,6,7,8,9 // PITCH,YAW,THR,ROLL... For Graupner/Spektrum
#define SERIAL_SUM_PPM_2         0,1,3,2,4,5,6,7,8,9 // ROLL,PITCH,THR,YAW... For Robe/Hitec/Futaba/Turnigy9xFrsky
#define SERIAL_SUM_PPM_3         1,0,3,2,4,5,6,7,8,9 // PITCH,ROLL,THR,YAW... For some Hitec/Sanwa/Others

#if defined (SKETCH_SERIAL_SUM_PPM)
  #define SERIAL_SUM_PPM SKETCH_SERIAL_SUM_PPM
#else
  #define SERIAL_SUM_PPM SERIAL_SUM_PPM_1
#endif

#endif

