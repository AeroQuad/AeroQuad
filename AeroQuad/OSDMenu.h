/*
 AeroQuad v3.0 - December 2011
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

/* Menu system implementation, usable with OSD or SerialLCD */

#ifndef _AQ_OSD_MENU_
#define _AQ_OSD_MENU_

//#define MENU_GOPRO // enable GoPro controls... not usable atm.

struct MenuItem {
  const byte level;            // menu level the item is on
  const char *text;            // text to show
  void (*function)(byte,byte); // handler func on leaf level
  const byte mode;             // data to give for handler function
};

extern const struct MenuItem menuData[];

#define MENU_INIT     0 // initial call to handler
#define MENU_UP       1 // stick up action
#define MENU_DOWN     2 // stick down action
#define MENU_SELECT   3 // stick right action
#define MENU_EXIT     4 // stick left action
#define MENU_CALLBACK 5 // timed callback
#define MENU_ABORT    6 // cleanup now (motors armed), only needs to be handled if cleanups are needed
#define MENU_HIJACK   7 // handler should interrept sticks completely !!

#define MENU_NOFUNC   0

#define MENU_SYM_BOTH '\015'
#define MENU_SYM_UP   '\016'
#define MENU_SYM_DOWN '\017'

#define MENU_STICK_CENTER  1500  // center value
#define MENU_STICK_NEUTRAL 100   // less than this from center is neutral
#define MENU_STICK_ACTIVE  200   // over this is select
#define MENU_STICK_REPEAT  400   // autorepeat at extreme values

byte  menuInFunc = 0;       // tells if a handler func is active
                            // 0 - we're in base menu
                            // 1 - call handler on stick actions
                            // 2-255 countdown and callback when it gets to 1 - no sticks
byte  menuEntry  = 255;     // Active menu entry
byte  menuAtExit = 0;       // are we at the exit at the top
byte  stickWaitNeutral = 1; // wait for stick to center
byte  menuOwnsSticks = 0;   // menu code will handle stick input (prevents arming)

// DATA that menu functions can freely use to store state
byte  menuFuncData[10];  // 10 bytes of data for funcs to use as they wish...
float menuFuncDataFloat; // float for menufuncs use

// menuHandleSimple - handle trivial menu actions launched from menu
//
//    This function will do action on MENU_INIT and optionally display a message.
//
//    To display message add following two lines after doing the action
//      notifyOSD(OSD_NOCLEAR|OSD_CENTER, "message");
//      menuInFunc = 10; // display time in 100ms increments

void menuHandleSimple(byte mode, byte action) {

  menuInFunc = 0; // default to no callback
  if (action == MENU_INIT) {
    switch (mode) {
    case 0:
#ifdef OSD
      armedTime = 0;
#endif
      break;
#ifdef BattMonitor
    case 1:
      for (int i=0; i<numberOfBatteries; i++) {
        resetBattery(i);
      }
      notifyOSD(OSD_NOCLEAR|OSD_CENTER, "Battery state reset!");
      menuInFunc = 10;
      break;
#endif
#ifdef UseGPS
    case 2:
      homePosition.latitude   = GPS_INVALID_ANGLE;
      homePosition.longitude  = GPS_INVALID_ANGLE;
      break;
#endif
/* TEMPLATE CODE FOR NEW ACTION:
    case XX: // Choose a free number here (0-255)
      doTask();
      notifyOSD(OSD_NOCLEAR|OSD_CENTER, "TASK done"); // optional message
      menuInFunc = 10;                                // optional message
      break;
*/
    }
  }
}

void writeEEPROM();
void initializeEEPROM();

// menuHandleConfirm - handle confirmed menu actions launched from menu
//
//    This function will ask confirmation before executing action and optionally display a message.
//
//    To display message add following two lines after doing the action
//      notifyOSD(OSD_NOCLEAR|OSD_CENTER, "message");
//      menuInFunc = 10; // display time in 100ms increments

void menuHandleConfirm(byte mode, byte action) {

  switch (action) {
  case MENU_ABORT:
  case MENU_EXIT:
  case MENU_CALLBACK:
    menuInFunc=0; // exit to menu
    break;

  case MENU_INIT:
  case MENU_UP:
  case MENU_DOWN:
    {
      menuFuncData[0] = 0;
      if (action == MENU_UP) {
        menuFuncData[0] = 1;
      }
      byte cpos = strlen(menuData[menuEntry].text) + 4;
      notifyOSDmenu(OSD_CURSOR|OSD_NOCLEAR, cpos, cpos, "%c%s? %c",
        menuFuncData[0] ? MENU_SYM_DOWN : MENU_SYM_UP,
        menuData[menuEntry].text,menuFuncData[0] ? 'Y' : 'N');
    }
    break;

  case MENU_SELECT:
    menuInFunc = 0;
    if (menuFuncData[0] == 1) {
      switch (mode) {
      case 0:
        writeEEPROM(); // defined in DataStorage.h
        zeroIntegralError();
        notifyOSD(OSD_NOCLEAR|OSD_CENTER, "EEPROM data saved");
        menuInFunc = 10;
        break;

      case 1:
        // Initialize EEPROM with default values
        notifyOSD(OSD_NOCLEAR|OSD_CENTER, "initializing EEPROM");
        initializeEEPROM(); // defined in DataStorage.h
        writeEEPROM(); // defined in DataStorage.h
        calibrateGyro();
        computeAccelBias();
        zeroIntegralError();
#ifdef HeadingMagHold
        initializeMagnetometer();
#endif
#if defined AltitudeHoldBaro
        initializeBaro();
#endif
        notifyOSD(OSD_NOCLEAR|OSD_CENTER, "EEPROM reinitialized");
        menuInFunc = 10; // callback after 1s
        break;

/* TEMPLATE CODE FOR NEW ACTION:
      case XX: // Choose a free number here (0-255)
        doTask();
        notifyOSD(OSD_NOCLEAR|OSD_CENTER, "TASK done"); // optional message
        menuInFunc = 10;                                // optional message
        break;
*/
      }
    }
    break;
  }
}

#ifdef CameraControl
void menuHandleCam(byte mode, byte action) {

  switch (action) {
  case MENU_ABORT:
    return; // no cleanup needed
  case MENU_INIT:
    menuFuncData[0]=0;
    menuFuncData[1]=0;
    break;

  case MENU_EXIT:
    if (menuFuncData[0]==0) {
      menuInFunc=0;
      return;
    }
    else {
      menuFuncData[0]--;
    }
    break;

  case MENU_SELECT:
    if (menuFuncData[0]<1) {
      menuFuncData[0]++;
    }
    break;
  case MENU_UP:
  case MENU_DOWN:
    if (menuFuncData[0]==0) {
      if (action==MENU_UP) {
        if (menuFuncData[1] < 3) {
          menuFuncData[1]++;
        }
      }
      else {
        if (menuFuncData[1]>0) {
          menuFuncData[1]--;
        }
      }
    }
    else if (menuFuncData[0]==1) {
      int val = (action == MENU_UP) ? 10 : -10;
      switch (menuFuncData[1]) {
      case 0:
        cameraMode = (action==MENU_UP)?1:0;
        break;
      case 1:
        servoCenterPitch = constrain(servoCenterPitch + val, servoMinPitch, servoMaxPitch);
        break;
      case 2:
        servoCenterRoll = constrain(servoCenterRoll + val, servoMinRoll, servoMaxRoll);
        break;
      case 3:
        servoCenterYaw = constrain(servoCenterYaw + val, servoMinYaw, servoMaxYaw);
        break;
      }
    }
    break;
  }

  if (menuFuncData[1] == 0) {
    notifyOSDmenu(OSD_NOCLEAR | OSD_CURSOR, menuFuncData[0] ? 18 : 1, menuFuncData[0] ? 18 : 16, "%cStabilizer mode: %1d", MENU_SYM_BOTH, cameraMode);
  }
  else {
    notifyOSDmenu(OSD_NOCLEAR|OSD_CURSOR,
      menuFuncData[0] ? 15 : 8, menuFuncData[0] ? 18 : 12,
      "%cCenter %s: %04d", MENU_SYM_BOTH,
      (menuFuncData[1] == 1)?"Pitch":
      (menuFuncData[1] == 2)?"Roll ":
      "Yaw  ",
      (menuFuncData[1] == 1) ? servoCenterPitch:
      (menuFuncData[1] == 2) ? servoCenterRoll:
      servoCenterYaw);
  }
}
#endif

// GoPro handling skeleton, not complete...
#ifdef MENU_GOPRO
const char *gopro_b_txt[3] = { "Shutter", "Mode", "Power" };

void menuHandleGoPro(byte mode, byte action) {

  switch (action) {
  case MENU_ABORT: // depress I/O line immediately to avoid letting it on
  case MENU_CALLBACK:
    // depress I/O line...
    menuInFunc=0; // exit to menu
    break;
  case MENU_INIT:
    // activate I/O
    notifyOSD(OSD_NOCLEAR|OSD_CENTER, "%s pressed", gopro_b_txt[mode]);
    if (mode == 2) {
      menuInFunc = 35; //3.5 sec
    }
    else {
      menuInFunc = 10; // 1 sec
    }
    break;
  default:
    break;
  }
}
#endif

// edit digit  with of form [+/-][i*#].[d*#]
// pos: 0=sign, 1-(i)=intpart, (i+1)-(i+d+1)=decimals
void menuEditFloat(float *f, byte i, byte d, byte pos, byte action, float min, float max) {

  if (pos > (1 + i + d)) {
    return;
  }
  // if ((action!=MENU_UP)&&(action!=MENU_DOWN)) return;
  if (pos == 0 && min * max < 0) { // change sign only if min/max are different sign
    *f = -*f;
  }
  else {
    *f += (action == MENU_UP ? 1.0 : -1.0) * pow(10.0, i-pos);
  }
  *f = constrain(*f, min, max);
}

const char *pidNames[] = {
  "RRoll", "RPitc", "RYaw ", "ARoll", "APitc",
  "Headi", "AGRol", "AGPit", "B_Alt", "S_Alt",
  "ZDamp",
#ifdef UseGPS
  "GPS_P", "GPS_R", "GPS_Y"
#endif
};

#define PIDCOUNT  (sizeof(pidNames)/sizeof(char*))

void menuHandlePidTune(byte mode, byte action) {

  switch (action) {
  case MENU_INIT:
    menuFuncData[0]=0; //level 0-select PID;1-select P/I/D;>=2-edit value
    menuFuncData[1]=0; // PIDno
    menuFuncData[2]=0; // 0=P/1=I/2=D
    break;
  case MENU_ABORT:
    return; // nocleanup needed
  case MENU_EXIT:
    if (menuFuncData[0]>0) {
      menuFuncData[0]--;
    }
    else {
      menuInFunc=0;
      return;
    }
    menuFuncData[3]=0;
    break;

  case MENU_SELECT:
    if (menuFuncData[0] < 9) {
      menuFuncData[0]++;
    }
    else {
      if (menuFuncData[3]) {
        switch (menuFuncData[2]) {
        case 0:
          PID[menuFuncData[1]].P = menuFuncDataFloat;
          break;
        case 1:
          PID[menuFuncData[1]].I = menuFuncDataFloat;
          break;
        case 2:
          PID[menuFuncData[1]].D = menuFuncDataFloat;
          break;
        }
        notifyOSD(OSD_NOCLEAR, "PID value saved!!");
      }
      else {
        notifyOSD(OSD_NOCLEAR, "PID value not saved!!");
      }
      menuInFunc = 10;     // callback after 1 second
      menuFuncData[0] = 1; //return to P/I/D selection
      return;
    }
    break;

  case MENU_UP:
    if (menuFuncData[0] == 0) {
      if (menuFuncData[1] < (PIDCOUNT-1)) menuFuncData[1]++;
    }
    else if (menuFuncData[0] == 1) {
      if (menuFuncData[2] < 2) menuFuncData[2]++;
    }
    else if (menuFuncData[0] == 9) {
      menuFuncData[3] = 1;
    }
    else {
      menuEditFloat(&menuFuncDataFloat, 4, 2, menuFuncData[0]-2, MENU_UP, -1000, 1000);
    }
    break;

  case MENU_DOWN:
    if (menuFuncData[0] < 2) {
      if (menuFuncData[menuFuncData[0] +1 ] > 0) menuFuncData[menuFuncData[0] + 1]--;
    }
    else if (menuFuncData[0] == 9) {
      menuFuncData[3] = 0;
    }
    else {
      menuEditFloat(&menuFuncDataFloat, 4, 2, menuFuncData[0]-2, MENU_DOWN, -1000, 1000);
    }
    break;
  }

  float old = (menuFuncData[2] == 0 ? PID[menuFuncData[1]].P:
               menuFuncData[2] == 1 ? PID[menuFuncData[1]].I:
               PID[menuFuncData[1]].D);

  if (menuFuncData[0] < 2) {
    // update value if edited value might have changed
    menuFuncDataFloat = old;
  }

  if (menuFuncData[0] < 9) {
    // determine 'cursor' position
    // assume we are editing the number
    byte cl   = ((menuFuncData[0] > 6) ? 12 : 11) + menuFuncData[0];
    byte cr   = cl;
    byte updn = MENU_SYM_BOTH;
    // check if it is something else
    if (menuFuncData[0] == 0) {
      // selecting PID
      cl = 5;
      cr = 9;
      if (menuFuncData[1] == 0) updn = MENU_SYM_UP;
      if (menuFuncData[1] == (PIDCOUNT-1)) updn = MENU_SYM_DOWN;
    }
    else if (menuFuncData[0] == 1) {
      // selecting P/I/D
      cl = 11;
      cr = cl;
      if (menuFuncData[2] == 0) updn = MENU_SYM_UP;
      if (menuFuncData[2] == 2) updn = MENU_SYM_DOWN;
    }
    notifyOSDmenu(OSD_CURSOR | OSD_NOCLEAR, cl, cr, "%cPID %s:%c=%c%04d.%02d",
      updn, pidNames[menuFuncData[1]], menuFuncData[2]==0 ? 'P' : menuFuncData[2]==1 ? 'I' : 'D',
      (menuFuncDataFloat >= 0) ? '+' : '-',
      (int)abs(menuFuncDataFloat), (int)abs((menuFuncDataFloat - (int)menuFuncDataFloat) * 100.0));
  }
  else {
    // asking confirmation for changing the value
    notifyOSDmenu(OSD_CURSOR|OSD_NOCLEAR,21,21,"C %c%04d.%02d->%c%04d.%02d?%c",
      (old>=0)?'+':'-',(int)abs(old),(int)abs((old-(int)old)*100.0),
      (menuFuncDataFloat>=0)?'+':'-',(int)abs(menuFuncDataFloat),(int)abs((menuFuncDataFloat-(int)menuFuncDataFloat)*100.0),
      menuFuncData[3] ? 'Y' : 'N');
  }
}


// this define is used to convert a float into (char) sign, (int) integerpart, (int) per100parts
#define PRFLOAT(x) ((x<0.0)?'-':' '),((int)abs(x)),(abs((int)(100*x))%100)

void menuSensorInfo(byte mode, byte action){
  switch (action) {
  case MENU_EXIT:
  case MENU_ABORT:
    menuInFunc=0;
    break;
  case MENU_CALLBACK:
  case MENU_INIT:
    switch (mode) {
      case 0: // Accel
        notifyOSD(OSD_NOCLEAR,"Acc: X%c%d.%02d Y%c%d.%02d Z%c%d.%02d",
                  PRFLOAT(meterPerSecSec[XAXIS]),PRFLOAT(meterPerSecSec[YAXIS]),PRFLOAT(meterPerSecSec[ZAXIS]));
        break;
      case 1: // Gyro
        notifyOSD(OSD_NOCLEAR,"Gyr: X%c%d.%02d Y%c%d.%02dZ %c%d.%02d",
                  PRFLOAT(gyroRate[XAXIS]),PRFLOAT(gyroRate[YAXIS]),PRFLOAT(gyroRate[ZAXIS]));
        break;
      #if defined(HeadingMagHold)
        case 2: // Mag
          notifyOSD(OSD_NOCLEAR,"Mag: X%5d Y%5d Z%5d",
		  getMagnetometerData(XAXIS),getMagnetometerData(YAXIS),getMagnetometerData(ZAXIS));
          break;
      #endif
      #if defined(AltitudeHoldRangeFinder)
        case 3: // Rangers
          notifyOSD(OSD_NOCLEAR,"US:a%df%dr%dr%dl%d cm",
                  (int)(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]*100),
                  (int)(rangeFinderRange[FRONT_RANGE_FINDER_INDEX]*100),
                  (int)(rangeFinderRange[RIGHT_RANGE_FINDER_INDEX]*100),
                  (int)(rangeFinderRange[REAR_RANGE_FINDER_INDEX]*100),
                  (int)(rangeFinderRange[LEFT_RANGE_FINDER_INDEX]*100));
          break;
      #endif
    }
    menuInFunc=3;
    break;
  default:
    menuInFunc=3;
  }
}

void menuHideOSD(byte mode, byte action){

  menuInFunc=0;
  menuEntry=255;
  notifyOSD(OSD_NOCLEAR,NULL);
  hideOSD();
}

#ifdef CameraControl
short savedCenterYaw, savedCenterPitch, savedCenterRoll;
byte  savedCameraMode;

#define POWERSAVE 10 // enable to shut off servos after idle
#if defined (POWERSAVE)
  byte idleCounter = POWERSAVE;
#endif

#define ZOOMPIN 24


void menuCameraPTZ(byte mode, byte action){

  if (action == MENU_INIT) {
    hideOSD();
    menuOwnsSticks = 1;
    savedCenterYaw   = servoCenterYaw;
    savedCenterPitch = servoCenterPitch;
    savedCenterRoll  = servoCenterRoll;
    savedCameraMode  = cameraMode;
    cameraMode       = 0; // disable stabilizer
    menuFuncDataFloat = 0.0;
		#if defined (POWERSAVE)
		idleCounter = POWERSAVE;
		#endif
  }
  else if ((action == MENU_CALLBACK) || (action == MENU_ABORT)) {
    digitalWrite(ZOOMPIN, LOW); // Zoom off
    pinMode(ZOOMPIN, INPUT);
		#if defined (POWERSAVE)
			TCCR1A |= ((1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1)); // make sure servos are enabled
	  #endif
    menuInFunc = 0;    
  }
  else if (action == MENU_HIJACK) {
    const short roll  = receiverCommand[XAXIS] - MENU_STICK_CENTER;  // adjust all to -500 - +500
    const short pitch = receiverCommand[YAXIS] - MENU_STICK_CENTER;
    short yaw   = receiverCommand[ZAXIS] - MENU_STICK_CENTER;

    if (roll < -MENU_STICK_REPEAT) {
      unhideOSD();
      menuOwnsSticks = 0;
      menuInFunc  = 40; // Callback after 4sec

      pinMode(ZOOMPIN, OUTPUT);
      digitalWrite(ZOOMPIN, LOW); // Zoom out

      // restore position
//      servoCenterYaw   = savedCenterYaw;
//      servoCenterPitch = savedCenterPitch;
//      servoCenterRoll = savedCenterRoll;
      cameraMode  = savedCameraMode;

      notifyOSD(OSD_NOCLEAR|OSD_CENTER, "exiting PTZ mode");
      return;
    }

    if (abs(yaw) > 50) {
      if (yaw>0) { 
        yaw-=50;
      }
      else {
        yaw+=50;
      }
      servoCenterYaw = constrain(servoCenterYaw + (yaw/10), servoMinYaw, servoMaxYaw);
  		#if defined (POWERSAVE)
    		idleCounter = POWERSAVE;
  		#endif
    }

    if (abs(receiverCommand[THROTTLE] - menuFuncDataFloat) > 2) {
      menuFuncDataFloat = receiverCommand[THROTTLE];
      servoCenterPitch = constrain(3000 - menuFuncDataFloat, servoMinPitch, servoMaxPitch);
  		#if defined (POWERSAVE)
    		idleCounter = POWERSAVE;
  		#endif
    }

    if (roll > MENU_STICK_REPEAT) {
      // elevator
      if (pitch < -MENU_STICK_REPEAT) {
        // DOWN
        servoCenterRoll = servoMinRoll;
      }
      else if (pitch > MENU_STICK_REPEAT) {
        // UP
        servoCenterRoll = servoMaxRoll;
      } else {
        // STOP
        servoCenterRoll = savedCenterRoll;
      }
  		#if defined (POWERSAVE)
    		idleCounter = POWERSAVE;
  		#endif

    } 
    else {
      // zoom
      if (pitch < -MENU_STICK_REPEAT) {
        // zoom out
        pinMode(ZOOMPIN, OUTPUT);
        digitalWrite(ZOOMPIN, LOW);
      }
      else if (pitch > MENU_STICK_REPEAT) {
        // zoom in
        pinMode(ZOOMPIN, OUTPUT);
        digitalWrite(ZOOMPIN, HIGH);
      } else {
        // release zoom
        digitalWrite(ZOOMPIN, LOW); // this is needed to remove the 'internal pullup'
        pinMode(ZOOMPIN, INPUT);
      }
    }
    #if defined (POWERSAVE)
    	if (idleCounter == POWERSAVE) {
    	    idleCounter--;
    					TCCR1A |= ((1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1)); // Enable servos
    	} else if (idleCounter>0) {
    		idleCounter--;
    		if (idleCounter == 0) {
    			TCCR1A &= ~((1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1)); // shut off servo ppm
    		}
    	}
  	#endif

  }
}
#endif

// MENU STRUCTURE TABLE
//
// One line in this table corresponds to one entry on the menu.
//
// Entry format:
//  { LEVEL, TEXT, HANDLER, MODE},
// Where
//  LEVEL   - this defines the tree structure 0 == root level
//  TEXT    - displayed text should be no more than ~16 characters
//  HADNLER - handler function to be called for this item (or MENU_NOFUNC for submenu)
//  MODE    - data passed to handler function to allow sharing them

const struct MenuItem menuData[] = {
#if 0
  {0, "Waypoints",            MENU_NOFUNC,       0},
  {1,   "Select dest.",       menuHandleWpt,     0},
  {1,   "Add wpt",            MENU_NOFUNC,       0},
  {2,     "Add wpt here",     menuHandleWpt,     1},
  {2,     "Add wpt by coord", menuHandleWpt,     2},
  {1,   "Delete wpt",         menuHandleWpt,     3},
#endif
#ifdef MENU_GOPRO
  {0, "GoPro",                MENU_NOFUNC,       0},
  {1,   "Shutter",            menuHandleGoPro,   0},
  {1,   "Mode",               menuHandleGoPro,   1},
  {1,   "Pwr",                menuHandleGoPro,   2},
#endif
  {0, "Config",               MENU_NOFUNC,       0},
#ifdef CameraControl
  {1,   "Camera stabilizer",  menuHandleCam,     0},
#endif
#ifdef BattMonitor
  {1,   "Reset battery stats",menuHandleSimple,  1},
#endif
#ifdef UseGPS
  {1,   "Reset Home",         menuHandleSimple,  2},
#endif
  {1,   "OSD",                MENU_NOFUNC,       0},
  {2,     "Reset flightime",  menuHandleSimple,  0},
  {0, "Setup",                MENU_NOFUNC,       0},
  {1,   "Edit PIDs",          menuHandlePidTune, 0},
  {1,   "Save to EEPROM",     menuHandleConfirm, 0},
  {1,   "Reinit EEPROM",      menuHandleConfirm, 1},
  {0, "Debug",                MENU_NOFUNC,       0},
  {1,   "Sensors",            MENU_NOFUNC,       0},
  {2,     "Accel Data",       menuSensorInfo,    0},
  {2,     "Gyro Data",        menuSensorInfo,    1},
#if defined(HeadingMagHold)
  {2,     "Mag Data",         menuSensorInfo,    2},
#endif
#if defined(AltitudeHoldRangeFinder)
  {2,     "Ranger Data",      menuSensorInfo,    3},
#endif
#ifdef CameraControl
  {0, "Camera PTZ mode",      menuCameraPTZ,     0},
#endif
  {0, "Exit and hide OSD",    menuHideOSD,       0},
  };

#define menuNumEntries (sizeof(menuData) / sizeof(MenuItem))

// This function will tell if the entry is the last at this level
byte  menuIsLast(byte entry) {

  for (byte i=entry+1; i<menuNumEntries; i++) {
    if (menuData[i].level==menuData[entry].level) {
      // found an entry at same level
      return 0;
    }
    if (menuData[i].level<menuData[entry].level) {
      // found an entry at lower level
      return 1;
    }
  }
  // bottom of the whole menu structure
  return 1;
}

#define MENU_CALLFUNC(entry, action) menuData[entry].function(menuData[entry].mode,action)

void menuShow(byte entry) {

  if (255 != menuEntry) {
    if (menuAtExit) {
      notifyOSD(OSD_NOCLEAR, "%cEXIT", MENU_SYM_DOWN);
    }
    else {
      notifyOSD(OSD_NOCLEAR,"%c%s",
        menuIsLast(entry) ? MENU_SYM_UP : MENU_SYM_BOTH,
        menuData[entry].text);
    }
  }
  else {
    notifyOSD(OSD_NOCLEAR,NULL);
  }
}

void menuUp() {

  if (255==menuEntry) {
    return;
  }

  if (menuInFunc) {
    MENU_CALLFUNC(menuEntry,MENU_UP);
    return;
  }

  if ((0 == menuEntry) || (menuData[menuEntry].level > menuData[menuEntry-1].level)) {
    menuAtExit=1;
  }
  else {
    for (byte i=menuEntry-1; i>=0; i--) {
      if (menuData[menuEntry].level == menuData[i].level) {
        menuEntry = i;
        break;
      }
    }
  }
  menuShow(menuEntry);
}

void menuDown() {

  if (255 == menuEntry) {
    return;
  }
  if (menuInFunc) {
    MENU_CALLFUNC(menuEntry, MENU_DOWN);
    return;
  }
  if (menuAtExit) {
    menuAtExit = 0;
  }
  else {
    if (menuIsLast(menuEntry)) return;
    for (byte i = menuEntry + 1; i < menuNumEntries; i++) {
      if (menuData[menuEntry].level == menuData[i].level) {
        menuEntry = i;
        break;
      }
    }
  }
  menuShow(menuEntry);
}

void menuSelect() {

  if (255==menuEntry) {
    // enable menu
    unhideOSD(); // make sure OSD is visible
    menuAtExit=0;
    menuEntry=0;
  }
  else if (menuInFunc) {
    MENU_CALLFUNC(menuEntry,MENU_SELECT);
    if (menuInFunc) return; // redisplay menu if we exited from handler
  }
  else if (menuAtExit) {
    if (0==menuEntry) {
      menuEntry=255;
    }
    else {
      // leave submenu
      menuEntry--;
      menuAtExit=0;
    }
  }
  else if (menuData[menuEntry].function != MENU_NOFUNC) {
    menuInFunc = 1;
    MENU_CALLFUNC(menuEntry, MENU_INIT);
    return;
  }
  else if (menuData[menuEntry].level < menuData[menuEntry + 1].level) {
    // enter submenu
    menuEntry++;
  }
  menuShow(menuEntry);
}

void menuExit() {

  if (255 == menuEntry)
    return;

  if (menuInFunc) {
    MENU_CALLFUNC(menuEntry, MENU_EXIT);
    if (menuInFunc)
      return;
  }
  else if ((0 == menuEntry) || (0 == menuData[menuEntry].level)) {
    menuEntry = 255;
  }
  else {
    // leave submenu
    for (byte i = menuEntry-1; i>=0; i--) {
       if (menuData[i].level < menuData[menuEntry].level) {
         menuEntry = i;
         menuAtExit = 0;
         break;
       }
    }
  }
  menuShow(menuEntry);
}

void updateOSDMenu() {

  // check for special HiJack mode
  if ((menuEntry!=255) && menuOwnsSticks  && menuInFunc) {

    MENU_CALLFUNC(menuEntry, MENU_HIJACK);
    if (menuInFunc == 0) {
      menuShow(menuEntry);
    }
    return;
  }

  // check if armed, menu is only operational when not armed
  if (motorArmed == true) {
    if (menuEntry != 255) {
      // BAIL OUT of menu if armed
      if (menuInFunc) {
        MENU_CALLFUNC(menuEntry, MENU_ABORT);
      }
      notifyOSD(0, NULL); // clear menuline
      menuInFunc = 0;
      menuEntry  = 255;
    }
    return;
  }

  // check if we're waiting for callback to happen
  if (menuInFunc > 1) {
    menuInFunc--;
    if (menuInFunc == 1) {
      // call the callback when counter hits 1
      MENU_CALLFUNC(menuEntry, MENU_CALLBACK);
      // show the menu entry if the handler function 'exited'
      if (menuInFunc == 0) {
        menuShow(menuEntry);
      }
      return;
    }
  }

  const short roll  = receiverCommand[XAXIS]  - MENU_STICK_CENTER;  // pitch/roll should be -500 - +500
  const short pitch = receiverCommand[YAXIS] - MENU_STICK_CENTER;

  if (abs(roll) < MENU_STICK_NEUTRAL) {
    if (abs(pitch) < MENU_STICK_NEUTRAL) {
      // stick at center
      stickWaitNeutral = 0;
      return;
    }
    else {
      // roll at neutral, pitch not
      if (abs(pitch) > MENU_STICK_REPEAT) {
        // above repeat threshold so allow redoing action
        stickWaitNeutral = 0;
      }

      // check if we are waiting for neutral (after single action)
      if (stickWaitNeutral) {
        return;
      }

      // call action function if stick active
      if (abs(pitch) > MENU_STICK_ACTIVE) {
        if (pitch > 0) {
          menuUp();
        }
        else {
          menuDown();
        }

        // action done, further actions only after centering (or repeat)
        stickWaitNeutral = 1;
      }
    }
  }
  else {
    if (abs(pitch) >= MENU_STICK_NEUTRAL) {
      return; // both ways active - no action
    }
    if (abs(roll) > MENU_STICK_ACTIVE) {
        if (roll > 0) {
          if (!stickWaitNeutral) {
            menuSelect();
          }
        }
        else {
          if ((!stickWaitNeutral) || (abs(roll) >= MENU_STICK_REPEAT)) {
            menuExit();
          }
        }
        stickWaitNeutral = 1;
      }
  }
  return;
}

#endif // Menu_h
