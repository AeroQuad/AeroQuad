/*
  AeroQuad v3.0 - Febuary 2012
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

// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)


#ifndef _AQ_Navigator_H_
#define _AQ_Navigator_H_


//GPS_waypoint GPS_waypoints[10];

//int GPS_waypointCount = 0;
//int GPS_currWpt = 0;
//GPS_waypoint * GPS_currentWaypoint = &GPS_waypoints[0];

unsigned long GPS_fixAge = GPS_INVALID_FIX_TIME;

unsigned long GPS_course = GPS_INVALID_ANGLE;
float GPS_speed = GPS_INVALID_SPEED;

float GPS_laggedSpeed = 0;
long GPS_laggedCourse = 0;

int GPS_speedSmoothValue = 0.5;
int GPS_coarseSmoothValue = 0.5;

int GPS_satelitesInUse = 0;

unsigned long GPS_chars; //from tinyGPS? needed? 
unsigned short GPS_sentences, GPS_failed_checksum;


const int GPS_minSatelitesNum = 6; //minimum amount of satelites for gpsfix. - to be moved to eeprom
const int GPS_maxAngle = 4; //+/- 5 degrees. - how much quad can lean - to be moved to eeprom

const int GPS_maxFixAge = 300; //maximum gpsfix age - to be moved to eeprom
const int GPS_maxHeadingSpeed = 400; // m/s * 100 // 3 m/s = 10.8km/h - to be moved to eeprom
long GPS_latitudeToHold = 999999999;
long GPS_longtitudeToHold = 999999999;
byte GPS_isGpsStorePositionNeeded = OFF;
byte GPS_holdStatus = OFF;
byte GPS_fix = OFF;

int GPS_rollAxis = 0; //values added to roll axis
int GPS_pitchAxis = 0; //values added to pitch axis


// ******** Metods implemented for individual type of GPS ***** 
int GPS_refreshRate;


boolean haveMission() {
  return missionNbPoint != 0;
}

void updateGPSRollPitchSpeedAlg(GPS_waypoint wpt) {
	float YDist = 0, XDist = 0, Dist = 0;
    
	float DerivateDistX = 0, DerivateDistY = 0, DerivateDist = 0;
    
	float CurrentSpeedCmPerSec = 0, CurrentSpeedCmPerSecRoll = 0, CurrentSpeedCmPerSecPitch = 0;
	float MaxSpeedRoll = 0, MaxSpeedPitch = 0, AngleToWaypoint = 0;
	float azimuth = getAbsoluteHeading();
	//float CourseRads = radians(LaggedCourse/100); //Get current GPS heading in radians
	float CourseRads = 0; //Get current GPS heading in radians
    
	float angle = 0;
	float tmpsin = 0, tmpcos = 0;

	if (GPS_holdStatus == ON && GPS_fix == ON) {
		//Distance calculations
		
		DerivateDistX = (GPS_curr_longitude - GPS_prev_longitude)*0.649876;
		DerivateDistY = (GPS_curr_latitude - GPS_prev_latitude)*1.113195;
		DerivateDist = sqrt(sq(DerivateDistY) + sq(DerivateDistX));
 
		XDist = (wpt.longitude - GPS_curr_longitude)*0.649876; //centimiters
		YDist = (wpt.latitude - GPS_curr_latitude)*1.113195; //centimiters
		Dist = sqrt(sq(XDist) + sq(YDist));
      
		GPS_laggedSpeed = GPS_laggedSpeed * (GPS_speedSmoothValue) + DerivateDist * GPS_refreshRate * (1-GPS_speedSmoothValue);
		float tmp = 0;
		if (DerivateDistX != 0 || DerivateDistY != 0) {
			tmp = degrees(atan2(DerivateDistX, DerivateDistY));
			if (tmp < 0) tmp += radians(360);
				GPS_laggedCourse = (int)((float)GPS_laggedCourse*(GPS_coarseSmoothValue) + tmp*100*(1-GPS_coarseSmoothValue));// + Course*0.1);      
		}
      
		AngleToWaypoint = atan2(XDist, YDist);
      
		CurrentSpeedCmPerSec = GPS_laggedSpeed; //LaggedSpeed in cm/s
		CourseRads = radians(GPS_laggedCourse/100);
      
		CurrentSpeedCmPerSecRoll = sin(CourseRads-azimuth)*CurrentSpeedCmPerSec; 
		CurrentSpeedCmPerSecPitch = cos(CourseRads-azimuth)*CurrentSpeedCmPerSec;
      
		angle = AngleToWaypoint-azimuth;
      
		if (Dist != 0) {
			tmpsin = sin(angle);
			tmpcos = cos(angle);
        
			if (Dist > 300) //if distance is over 20m, use max speed
			{
				//Vector of speed is constant.
				//Speed calculations
				MaxSpeedRoll = GPS_maxHeadingSpeed*tmpsin; //max speed on roll
				MaxSpeedPitch = GPS_maxHeadingSpeed*tmpcos; //max speed on pitch
			} 
			else 
			{ //its less then 20m
				//scale max speed to 2 
				MaxSpeedRoll = (int)(GPS_maxHeadingSpeed*tmpsin*((float)Dist/300)); //roll
				MaxSpeedPitch = (int)(GPS_maxHeadingSpeed*tmpcos*((float)Dist/300)); //pitch
			}
    
			if (Dist > 300) {
				GPS_pitchAxis = updatePID(MaxSpeedPitch, CurrentSpeedCmPerSecPitch , &PID[GPSPITCH_PID_IDX]);
				GPS_rollAxis = updatePID(MaxSpeedRoll, CurrentSpeedCmPerSecRoll, &PID[GPSROLL_PID_IDX]);
			}
		}
	} else {
		//if there is no fix then what?  gpshold reset pids?
		GPS_pitchAxis = 0;
		GPS_rollAxis = 0;
	}
}  
  
void gpsdump()
{    
  Serial.print("lat: ");  Serial.print(GPS_curr_latitude);  Serial.print(" ");      
  Serial.print("lon: ");  Serial.print(GPS_curr_longitude);  Serial.print(" ");
  Serial.print("sat num: ");  Serial.print(GPS_satelitesInUse);  Serial.print(" ");
  Serial.print("fix age: ");  Serial.print(GPS_fixAge);  Serial.print(" ");
  Serial.print("speed: ");  Serial.print(GPS_speed);  Serial.print(" ");
  Serial.print("course: ");  Serial.print(GPS_course); Serial.print(" ");
  
  Serial.print("chars: ");  Serial.print(GPS_chars);  Serial.print(" ");    
  Serial.print("sentences: ");  Serial.print(GPS_sentences);  Serial.print(" ");
  Serial.print("failed_checksum: ");  Serial.print(GPS_failed_checksum); Serial.print("\n");
}
#endif

