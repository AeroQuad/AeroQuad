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
 properly or safely re-read above.
 
 composed by Robert Davis and William Davis
 */

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

#ifdef AQ_CPU_IS_FAST														// slices barometer is sampled
  byte VO_timer = 25;
  float delta_Factor= 25*3.81;												// samples per second times feet per meter used to obtain feet per second
#endif
#ifndef AQ_CPU_IS_FAST
  byte VO_timer = 5;
  float delta_Factor= 5*3.81;												// samples per second times feet per meter used to obtain feet per second
#endif

#define VO_DISPLAY_RECT_HEIGHT 9											// height of rectangle bounding AI. Should be odd so that there is an equal space above/below the centre reticle
#define VO_MAX_PITCH_ANGLE (PI/8)											// bounds of scale used for displaying variometer. When variometer is >= |this number|, the variometer lines will be at top or bottom of bounding box
#define VO_RETICLE_ROW (MAX_screen_rows/2)									// centre row - don't change this
#define VO_RETICLE_COL 27													// reticle will be in this column
#define VO_CENTRE       (VO_RETICLE_ROW*18+10)								// row, in pixels, corresponding to zero vario.
#define VO_TOP_PIXEL    ((VO_RETICLE_ROW - VO_DISPLAY_RECT_HEIGHT/2)*18)	// top of bounding box of variometer display 
#define VO_BOTTOM_PIXEL ((VO_RETICLE_ROW + VO_DISPLAY_RECT_HEIGHT/2)*18)	// bottom of bounding box of variometer display 

static char VO_reticle[4] = {228};											// right pointing arrow character

static const byte VO_COLUMN = 28;											// column where the variometer line is printed
static byte VO_AHoldline = 0;

static boolean displayedArrow = false;										// has the "needle" center arrow been displayed
static float deltaAltitude = 0.0;

static float safeVelocity = 2.00;											// reasonable velocity to land
static float safeAttitude = 0.03;											// how level to assume not moving much
static float visualScale = 5.00;											// scale variometer needle

//////////////////////////////////////////////////////////////////////////////
///////////////////////// Landing Assist Display /////////////////////////////
//////////////////////////////////////////////////////////////////////////////

void displayVariometer(byte flightMode) {									// called 25 times a second
  static float lastAltitude=0.0;											// stored for comparison to current baroAltitude

  deltaAltitude = ( baroAltitude - lastAltitude )*delta_Factor;				// feet per second
  lastAltitude = baroAltitude;

  if (displayedArrow == false) {											// only need to do this one time
   writeChars( VO_reticle, 1, 0, VO_RETICLE_ROW, VO_RETICLE_COL );			// display variometer reticle (right pointing arrow)
   displayedArrow = true;													// reticle has been displayed
   }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // I'm sure this is not the best way to decide if we're moving horizontally, but if we're at an angle we're moving!  //
  //                Falling at .3 meters per sec moving forward 10 meters per second would be bad;)                    //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  

  if (((kinematicsAngle[XAXIS] > safeAttitude)||(kinematicsAngle[XAXIS] < -safeAttitude))||
      ((kinematicsAngle[YAXIS] > safeAttitude)||(kinematicsAngle[YAXIS] < -safeAttitude)))
  { 
    VO_reticle[0] = 227;													// we're not level enough to land ie probably moving ??
    writeChars( VO_reticle, 1, 1, VO_RETICLE_ROW, VO_RETICLE_COL );			// display reticle left pointing flashing at artificial horizon
  }
  else
  {
    if (deltaAltitude < safeVelocity && deltaAltitude > -safeVelocity){		// mostly level we're probably not moving x or y 
      VO_reticle[0] = 13;
      writeChars( VO_reticle, 1, 0, VO_RETICLE_ROW, VO_RETICLE_COL );		// display variometer reticle center
    }

    if (deltaAltitude > safeVelocity) {
      VO_reticle[0] = 14;
      writeChars( VO_reticle, 1, 0, VO_RETICLE_ROW, VO_RETICLE_COL );		// display variometer reticle up
    }

    if (deltaAltitude < -safeVelocity) {
      VO_reticle[0] = 15;
      writeChars( VO_reticle, 1, 0, VO_RETICLE_ROW, VO_RETICLE_COL );		// display variometer reticle down
    }
  }


  //////////////////////////////////////////////////////////////////////////////
  ////////////////////////////// Draw Difference ///////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  byte row = constrain(
  AH_CENTRE + (14.5 - (float)VO_COLUMN) * 12 * 1.4 * ( deltaAltitude/visualScale ) +
    (( deltaAltitude/visualScale )/VO_MAX_PITCH_ANGLE*(VO_CENTRE-VO_TOP_PIXEL)),VO_TOP_PIXEL, VO_BOTTOM_PIXEL)-1;

  if ((row/18) != VO_AHoldline) {											// erase existing character
    writeChars( NULL, 1, 0, VO_AHoldline, VO_COLUMN );
    VO_AHoldline = row/18;
  }

  char VO_RollLine = LINE_ROW_0 + (row % 18);
  writeChars( &VO_RollLine, 1, 0, VO_AHoldline, VO_COLUMN );				// display variometer value as a bar graph line
}

#endif  // #define_AQ_OSD_MAX7456_VO_H_
