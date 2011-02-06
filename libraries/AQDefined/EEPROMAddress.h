/*
  AeroQuad v2.1 - January 2011
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

#ifndef _EEPROM_ADDRESS_H_
#define _EEPROM_ADDRESS_H_

// **************************************************************
// *************************** EEPROM ***************************
// **************************************************************
// EEPROM storage addresses
#define ROLL_PID_GAIN_ADR 0
#define LEVELROLL_PID_GAIN_ADR 12
#define YAW_PID_GAIN_ADR 24
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
#define NVM_TRANSMITTER_SCALE_OFFSET_SMOOTH 76  // needs 8 channel with 3 entries of float (4 byte) -> 96 byte
#define PITCH_PID_GAIN_ADR 172
#define LEVELPITCH_PID_GAIN_ADR 184
#define HEADINGSMOOTH_ADR 200
#define HEADING_PID_GAIN_ADR 204
#define AREF_ADR 216
#define FLIGHTMODE_ADR 220
#define LEVEL_GYRO_ROLL_PID_GAIN_ADR 224
#define LEVEL_GYRO_PITCH_PID_GAIN_ADR 236
#define HEADINGHOLD_ADR 248
#define MINACRO_ADR 252
#define ACCEL1G_ADR 256
#define ALTITUDE_PGAIN_ADR 260
#define ALTITUDE_IGAIN_ADR 264
#define ALTITUDE_DGAIN_ADR 268
#define ALTITUDE_MAX_THROTTLE_ADR 272
#define ALTITUDE_MIN_THROTTLE_ADR 276
#define ALTITUDE_SMOOTH_ADR 280
#define ZDAMP_PGAIN_ADR 284
#define ZDAMP_IGAIN_ADR 288
#define ZDAMP_DGAIN_ADR 292
#define ALTITUDE_WINDUP_ADR 296
#define MAGXMAX_ADR 300
#define MAGXMIN_ADR 304
#define MAGYMAX_ADR 308
#define MAGYMIN_ADR 312
#define MAGZMAX_ADR 316
#define MAGZMIN_ADR 320
#define MCAMERAPITCH_ADR 324
#define MCAMERAROLL_ADR 328
#define MCAMERAYAW_ADR 332
#define CENTERPITCH_ADR 336
#define CENTERROLL_ADR 340
#define CENTERYAW_ADR 344
#define SERVOMINPITCH_ADR 348
#define SERVOMINROLL_ADR 352
#define SERVOMINYAW_ADR 356
#define SERVOMAXPITCH_ADR 360
#define SERVOMAXROLL_ADR 364
#define SERVOMAXYAW_ADR 368
#define GYRO_ROLL_ZERO_ADR 372
#define GYRO_PITCH_ZERO_ADR 376
#define GYRO_YAW_ZERO_ADR 380

#endif