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

/* changelog
  rev 1.1.0 corrected some comments for clarity
  rev 1.1.1 changed safeVelocity to safedeltaVelocity
  rev 1.1.2 separated displayVariometer() and deltaAltitudeSeconds() routines
*/

#ifndef _AQ_OSD_MAX7456_VO_H_
#define _AQ_OSD_MAX7456_VO_H_

/*
 012345678901234567890123456789


                              V
         - - - RR - - -     <>V
                              V



*/

#define VODISPLAY_RECT_HEIGHT 9											// height of rectangle bounding AI. Should be odd so that there is an equal space above/below the centre reticle
#define VOMAX_PITCH_ANGLE (PI/8)											// bounds of scale used for displaying variometer. When variometer is >= |this number|, the variometer lines will be at top or bottom of bounding box
#define VORETICLE_ROW (MAX_screen_rows/2)									// center row - don't change this
#define VORETICLE_COL 27													// reticle will be in this column
#define VOCENTRE (VORETICLE_ROW*18+10)									// row, in pixels, corresponding to zero vario.
#define VOTOP_PIXEL ((VORETICLE_ROW - VODISPLAY_RECT_HEIGHT/2)*18)		// top of bounding box of variometer display 
#define VOBOTTOM_PIXEL ((VORETICLE_ROW + VODISPLAY_RECT_HEIGHT/2)*18)	// bottom of bounding box of variometer display 

static char VOreticle[1] = {228};											// right pointing arrow character

static const byte VOCOLUMN = 28;											// column where the variometer line is printed
static byte VOAHoldline = 0;

static float safedeltaAltitudeSeconds = 1.75;								// reasonable change in altitude in feet per second to land softly
static float safeattitudeAngle = 0.12;										// how level to assume we're not moving excessively
static float scaleVariometer = 3.00;										// scale to top and bottom of center pointer

/**********************************************************
 ************* Landing Assist DisplayLanding **************
 **********************************************************/

void displayVariometer(float climbFallRate) {
  if (((kinematicsAngle[XAXIS] > safeattitudeAngle)||(kinematicsAngle[XAXIS] < -safeattitudeAngle))||
      ((kinematicsAngle[YAXIS] > safeattitudeAngle)||(kinematicsAngle[YAXIS] < -safeattitudeAngle)))
  { 
    VOreticle[0] = 227;													// we're not level enough to land ie probably moving ??
    writeChars( VOreticle, 1, 1, VORETICLE_ROW, VORETICLE_COL -1);		// display center pointer flashing at artificial horizon
  }
  else {  
    writeChars( NULL, 1, 1, VORETICLE_ROW, VORETICLE_COL -1);				// erase center pointer flashing at artificial horizon

    VOreticle[0] = 13;														// set center pointer up-down arrow icon
	 
    if (climbFallRate > safedeltaAltitudeSeconds) {
      VOreticle[0] = 14;													// set center pointer to up arrow icon
    }

    if (climbFallRate < -safedeltaAltitudeSeconds) {
      VOreticle[0] = 15;													// set center pointer to down arrow icon
    }

    writeChars( VOreticle, 1, 0, VORETICLE_ROW, VORETICLE_COL );			// display variometer center pointer
  }

/**********************************************************
 ****************** Draw Altitude Difference **************
 **********************************************************/

  byte row = constrain(														// resolution .10  -4.5 to +4.5
  AH_CENTRE + (14.5 - (float)VOCOLUMN) * 12 * 1.4 * ( climbFallRate/scaleVariometer ) +
    (( climbFallRate/scaleVariometer )/VOMAX_PITCH_ANGLE*(VOCENTRE-VOTOP_PIXEL)),VOTOP_PIXEL, VOBOTTOM_PIXEL)-1;

  if ((row/18) != VOAHoldline) {											// erase existing character if not on center row
    writeChars( NULL, 1, 0, VOAHoldline, VOCOLUMN );
    VOAHoldline = row/18;
  }

  char VORollLine = LINE_ROW_0 + (row % 18);
  writeChars( &VORollLine, 1, 0, VOAHoldline, VOCOLUMN );				// display variometer value as a bar graph line
}

#endif  // #define_AQ_OSD_MAX7456_VO_H_

