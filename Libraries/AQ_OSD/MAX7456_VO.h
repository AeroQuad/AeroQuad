/*
  AeroQuad v3.2 - March 2013
  www.AeroQuad.com
  Copyright (c) 2013 Ted Carancho.  All rights reserved.
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
 
  composed by norem and his Dad
 */

#ifndef _AQ_OSD_MAX7456_VO_H_
#define _AQ_OSD_MAX7456_VO_H_

/*
 012345678901234567890123456789


                              V
         - - - RR - - -     <>V
                              V



*/

#define VO_DISPLAY_RECT_HEIGHT 9						// height of rectangle bounding AI. Should be odd so that there is an equal space 
#define VO_MAX_PITCH_ANGLE (PI/8)						// above/below the centre reticle bounds of scale used for displaying variometer.
										// When variometer is >= |this number|, the variometer lines will be at top or bottom of bounding box
#define VO_RETICLE_ROW (MAX_screen_rows/2)					// center row - don't change this
#define VO_RETICLE_COL 27							// reticle will be in this column
#define VO_CENTRE (VO_RETICLE_ROW*18+10)					// row, in pixels, corresponding to zero vario.
#define VO_TOP_PIXEL ((VO_RETICLE_ROW - VO_DISPLAY_RECT_HEIGHT/2)*18)		// top of bounding box of variometer display 
#define VO_BOTTOM_PIXEL ((VO_RETICLE_ROW + VO_DISPLAY_RECT_HEIGHT/2)*18)	// bottom of bounding box of variometer display 

static char VO_reticle[1] = {228};						// right pointing arrow character

static const byte VO_COLUMN = 28;						// column where the variometer line is printed
static byte VO_AHoldline = 0;

static float safeclimbFallRate = 0.3048;					// reasonable change in altitude in meters per second to land softly
static float safeattitudeAngle = 0.12;						// how level to assume we're not moving excessively
static float scaleVariometer = 1.00;						// scale to top and bottom of center pointer

/**********************************************************
 ************* Landing Assist DisplayLanding **************
 **********************************************************/

void displayVariometer(float climbFallRate) {
  if (((kinematicsAngle[XAXIS] > safeattitudeAngle)||(kinematicsAngle[XAXIS] < -safeattitudeAngle))||
      ((kinematicsAngle[YAXIS] > safeattitudeAngle)||(kinematicsAngle[YAXIS] < -safeattitudeAngle)))
  { 
    VO_reticle[0] = 227;							// we're not level enough to land ie probably moving ??
    writeChars( VO_reticle, 1, 1, VO_RETICLE_ROW, VO_RETICLE_COL -1);		// display center pointer flashing at artificial horizon
  }
  else {  
    writeChars( NULL, 1, 1, VO_RETICLE_ROW, VO_RETICLE_COL -1);			// erase center pointer flashing at artificial horizon

    VO_reticle[0] = 13;								// set center pointer up-down arrow icon
	 
    if (climbFallRate > safeclimbFallRate) {
      VO_reticle[0] = 14;							// set center pointer to up arrow icon
    }

    if (climbFallRate < -safeclimbFallRate) {
      VO_reticle[0] = 15;							// set center pointer to down arrow icon
    }

    writeChars( VO_reticle, 1, 0, VO_RETICLE_ROW, VO_RETICLE_COL );		// display variometer center pointer
  }

/**********************************************************
 ****************** Draw Altitude Difference **************
 **********************************************************/

  byte row = constrain(								// resolution .10  -4.5 to +4.5 meters per second
  AH_CENTRE + (14.5 - (float)VO_COLUMN) * 12 * 1.4 * ( climbFallRate/scaleVariometer ) +
    (( climbFallRate/scaleVariometer )/VO_MAX_PITCH_ANGLE*(VO_CENTRE-VO_TOP_PIXEL)),VO_TOP_PIXEL, VO_BOTTOM_PIXEL)-1;

  if ((row/18) != VO_AHoldline) {						// erase existing character if not on center row
    writeChars( NULL, 1, 0, VO_AHoldline, VO_COLUMN );
    VO_AHoldline = row/18;
  }

  char VO_RollLine = LINE_ROW_0 + (row % 18);
  writeChars( &VO_RollLine, 1, 0, VO_AHoldline, VO_COLUMN );			// display variometer value as a bar graph line
}

#endif  // #define_AQ_OSD_MAX7456_VO_H_
