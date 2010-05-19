/*
  AeroQuad v1.8 - May 2010
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

#ifndef EEPROM_H
#define EEPROM_H

// EEPROM storage addresses
#define PGAIN_ADR 0
#define IGAIN_ADR 4
#define DGAIN_ADR 8
#define LEVEL_PGAIN_ADR 12
#define LEVEL_IGAIN_ADR 16
#define LEVEL_DGAIN_ADR 20
#define YAW_PGAIN_ADR 24
#define YAW_IGAIN_ADR 28
#define YAW_DGAIN_ADR 32
#define WINDUPGUARD_ADR 36
#define LEVELLIMIT_ADR 40
#define LEVELOFF_ADR 44
#define XMITFACTOR_ADR 48
#define GYROSMOOTH_ADR 52
#define ACCSMOOTH_ADR 56
#define LEVELPITCHCAL_ADR 60
#define LEVELROLLCAL_ADR 64
#define LEVELZCAL_ADR 68
#define FILTERTERM_ADR 72
#define MODESMOOTH_ADR 76
#define ROLLSMOOTH_ADR 80
#define PITCHSMOOTH_ADR 84
#define YAWSMOOTH_ADR 88
#define THROTTLESMOOTH_ADR 92
#define GYRO_ROLL_ZERO_ADR 96
#define GYRO_PITCH_ZERO_ADR 100
#define GYRO_YAW_ZERO_ADR 104
#define PITCH_PGAIN_ADR 124
#define PITCH_IGAIN_ADR 128
#define PITCH_DGAIN_ADR 132
#define LEVEL_PITCH_PGAIN_ADR 136
#define LEVEL_PITCH_IGAIN_ADR 140
#define LEVEL_PITCH_DGAIN_ADR 144
#define THROTTLESCALE_ADR 148
#define THROTTLEOFFSET_ADR 152
#define ROLLSCALE_ADR 156
#define ROLLOFFSET_ADR 160
#define PITCHSCALE_ADR 164
#define PITCHOFFSET_ADR 168
#define YAWSCALE_ADR 172
#define YAWOFFSET_ADR 176
#define MODESCALE_ADR 180
#define MODEOFFSET_ADR 184
#define AUXSCALE_ADR 188
#define AUXOFFSET_ADR 192
#define AUXSMOOTH_ADR 196
#define HEADINGSMOOTH_ADR 200
#define HEADING_PGAIN_ADR 204
#define HEADING_IGAIN_ADR 208
#define HEADING_DGAIN_ADR 212
#define AREF_ADR 216
#define FLIGHTMODE 220

float readFloat(int address);
void writeFloat(float value, int address);
void readEEPROM(void);

#endif
