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

#ifndef _AQ_CAMERA_STABILIZER_ON_PIN_6_7_8_H_
#define _AQ_CAMERA_STABILIZER_ON_PIN_6_7_8_H_

#include "CameraStabilizer.h"

class CameraStabilizerOnPins_6_7_8 : public CameraStabilizer 
{
public:
  CameraStabilizerOnPins_6_7_8();
  void _initialize();
  void move();
};

#endif