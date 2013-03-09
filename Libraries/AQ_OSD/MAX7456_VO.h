/*
 AeroQuad v3.0 - Nov 2011
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

/*
 This module is an effort to create a meter for AeroQuad OSD to assist in monitoring
 landing speed, it in no way is intended to be the only means of landing your craft 
 properly or safely re-read above. composed by RDavis and WDavis
 */
 
#ifndef _AQ_OSD_MAX7456_VO_H_
#define _AQ_OSD_MAX7456_VO_H_

// 012345678901234567890123456789
//
//
//                              
//                              V
//         - - - RR - - -      >V
//                              V
//
//
//

#ifdef AQ_CPU_IS_FAST  													                         // slices barometer is sampled
  byte VO_timer = 25;
#else
  byte VO_timer = 5;
#endif

#define VO_DISPLAY_RECT_HEIGHT 9  										                   // height of rectangle bounding AI. Should be odd so that there is an equal space above/below the centre reticle
#define VO_MAX_PITCH_ANGLE (PI/8)  										                   // bounds of scale used for displaying variometer. When variometer is >= |this number|, the variometer lines will be at top or bottom of bounding box
#define VO_RETICLE_ROW (MAX_screen_rows/2)  								             // centre row - don't change this
#define VO_RETICLE_COL 27  												                       // reticle will be in this column
#define VO_CENTRE (VO_RETICLE_ROW*18+10)  								               // row, in pixels, corresponding to zero vario.
#define VO_TOP_PIXEL ((VO_RETICLE_ROW - VO_DISPLAY_RECT_HEIGHT/2)*18)  	 // top of bounding box of variometer display 
#define VO_BOTTOM_PIXEL ((VO_RETICLE_ROW + VO_DISPLAY_RECT_HEIGHT/2)*18) // bottom of bounding box of variometer display 

byte ii = 0;  															                             // holds current time slice (times into routine)

static char VO_reticle[1] = {228};  							                       // right pointing arrow character and others

static const byte VO_COLUMN = 28;  										                   // column where the variometer line is printed
static byte VO_AHoldline = 0;

static boolean displayedArrow = false;  									               // has the "needle" center arrow been displayed
static float deltaAltitude = 0.0;  										                   // initialize deltaAltitude our lead actor
static float velocitySafe = 0.6;  										                   // what appears to be a safe rate to land ??
static float attitudeSafe = 0.05;  										                   // what appears to be a safe attitude to land ??

//////////////////////////////////////////////////////////////////////////////
///////////////////////// Landing Assist Display /////////////////////////////
//////////////////////////////////////////////////////////////////////////////

void displayVariometer(byte flightMode) {  								               // called VO_timer times a second
  static float lastAltitude=0.0;  										                   // stored for comparison to current baroAltitude

  if (displayedArrow == false) {  										                   // only need to do this one time
    writeChars( VO_reticle, 1, 0, VO_RETICLE_ROW, VO_RETICLE_COL );  		 // display variometer reticle (right pointing arrow)
    displayedArrow = true;  												                     // reticle has been displayed
  }

  ii = (ii + 1) % VO_timer;  												                     // increment counter and roll over when necc. -  % (modulo operator) rolls over variable

  if (ii == 0){  												                                 // sample barometer once a second to get current altitude
    deltaAltitude = ( baroAltitude - lastAltitude )*3.81;                // not a magic number just something visible
    lastAltitude = baroAltitude;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // I'm sure this assumption is not the best way to decide if were not moving horizontally, but if we're at an angle we are!
  // I don't want to be falling at .3 meters per sec moving forward at any serious velocity, no skid landing ;)
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  

  if ( ((kinematicsAngle[XAXIS] < attitudeSafe) && (kinematicsAngle[XAXIS] > -attitudeSafe))  &&
    ((kinematicsAngle[YAXIS] < attitudeSafe) && (kinematicsAngle[YAXIS] > -attitudeSafe)) ){  																		// are we mostly level ?

    if (deltaAltitude < velocitySafe && deltaAltitude > -velocitySafe){
      if (VO_reticle[0] != 13){
        VO_reticle[0] = 13;
        writeChars( VO_reticle, 1, 0, VO_RETICLE_ROW, VO_RETICLE_COL );  	// display variometer reticle up/down
      }
    }

    if (deltaAltitude > velocitySafe){
      if (VO_reticle[0] != 14){
        VO_reticle[0] = 14;
        writeChars( VO_reticle, 1, 0, VO_RETICLE_ROW, VO_RETICLE_COL );  	// display variometer reticle up
      }
    }

    if (deltaAltitude < -velocitySafe){
      if (VO_reticle[0] != 15){
        VO_reticle[0] = 15;
        writeChars( VO_reticle, 1, 0, VO_RETICLE_ROW, VO_RETICLE_COL );  	// display variometer reticle down
      }
    }
  }
  else
  { 
    if (VO_reticle[0] != 227){
      VO_reticle[0] = 227;  												// we're not level enough to land ie probably moving ??
      writeChars( VO_reticle, 1, 1, VO_RETICLE_ROW, VO_RETICLE_COL );     // display reticle left pointing flashing at artificial horizon
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  ////////////////////////////// Draw Difference ///////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  if ( lastAltitude != baroAltitude ){

    if (deltaAltitude <= velocitySafe && deltaAltitude >= -velocitySafe)
      deltaAltitude = 0.0;

    byte row = constrain(
    AH_CENTRE + (14.5 - (float)VO_COLUMN) * 12 * 1.4 * ( deltaAltitude ) +
      (( deltaAltitude )/VO_MAX_PITCH_ANGLE*(VO_CENTRE-VO_TOP_PIXEL)),VO_TOP_PIXEL, VO_BOTTOM_PIXEL)-1;

    if ((row/18) != VO_AHoldline)  										                    // erase existing character
    {
      writeChars( NULL, 1, 0, VO_AHoldline, VO_COLUMN );
      VO_AHoldline = row/18;
    }

    char VO_RollLine = LINE_ROW_0 + (row % 18);
    writeChars( &VO_RollLine, 1, 0, VO_AHoldline, VO_COLUMN );  			    // display "variometer" value as a bar graph line
  }
}

#endif  // #define 
_AQ_OSD_MAX7456_VO_H_


