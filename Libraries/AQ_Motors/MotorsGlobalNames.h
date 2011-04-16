/*
  AeroQuad v2.2 - Feburary 2011
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

#ifndef _AQ_MOTORS_GLOBAL_NAMES_H_
#define _AQ_MOTORS_GLOBAL_NAMES_H_

#if defined(HEXACOAXIAL) || defined(HEXARADIAL)
  #define LASTMOTOR 6
#else
  #define LASTMOTOR 4
#endif

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
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


#define FRONT 0
#define REAR 1
#define RIGHT 2
#define LEFT 3
#define MOTORID1 0		
#define MOTORID2 1		
#define MOTORID3 2		
#define MOTORID4 3		
#define MOTORID5 4		
#define MOTORID6 5
#define MINCOMMAND 1000
#define MAXCOMMAND 2000


#endif