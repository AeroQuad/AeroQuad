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

#ifndef _AQ_OSD_MAX7456_AH_H_
#define _AQ_OSD_MAX7456_AH_H_

// 012345678901234567890123456789
//
//         - - - RR - - -


#define LINE_ROW_0 0x80                             // character address of a character with a horizontal line in row 0. Other rows follow this one
static const byte AH_COLUMNS[6] = {8,10,12,17,19,21};  // columns where the roll line is printed
#define AH_DISPLAY_RECT_HEIGHT 9                    // Height of rectangle bounding AI. Should be odd so that there is an equal space above/below the centre reticle
#define AH_MAX_PITCH_ANGLE (PI/8)                   // bounds of scale used for displaying pitch. When pitch is >= |this number|, the pitch lines will be at top or bottom of bounding box

#define RETICLE_ROW (MAX_screen_rows/2)             // centre row - don't change this
#define RETICLE_COL 14                              // reticle will be in this col, and col to the right

#define AH_TOP_PIXEL ((RETICLE_ROW - AH_DISPLAY_RECT_HEIGHT/2)*18)
#define AH_BOTTOM_PIXEL ((RETICLE_ROW + AH_DISPLAY_RECT_HEIGHT/2)*18)
#define AH_CENTRE (RETICLE_ROW*18+10)               // row, in pixels, corresponding to zero pitch/roll.

//////////////////////////////////////////////////////////////////////////////
////////////////////// Artificial Horizon Display ////////////////////////////
//////////////////////////////////////////////////////////////////////////////

byte AHoldline[6] = {0,0,0,0,0,0};
byte lastFlightMode = 9;

void displayArtificialHorizon(float roll, float pitch, byte flightMode) {

  for (byte i=0; i<6; i++) {
    byte row = constrain(
			 AH_CENTRE + 
			 (14.5 - (float)AH_COLUMNS[i]) * 12 * 1.4 * roll +
			 (pitch/AH_MAX_PITCH_ANGLE*(AH_CENTRE-AH_TOP_PIXEL)),
			 AH_TOP_PIXEL, AH_BOTTOM_PIXEL);
    if ((row/18) != AHoldline[i]) {
      writeChars( NULL, 1, 0, AHoldline[i], AH_COLUMNS[i] );
      AHoldline[i] = row/18;
    }
    char RollLine = LINE_ROW_0 + (row % 18);
    writeChars( &RollLine, 1, 0, AHoldline[i], AH_COLUMNS[i] );
  }

  // Reticle on the center of the screen
  // 0 - rate mode (no letter)
  // 1 - Attitude 'S'
  // 2 - GPS position hold 'P'
  // 3 - GPS navigation 'N'
  if (lastFlightMode != flightMode) {
    char reticle[2];
    reticle[0] = flightMode * 2 + 1;
    reticle[1] = reticle[0] + 1;
    writeChars( reticle, 2, 0, RETICLE_ROW, RETICLE_COL ); //write 2 chars to row (middle), column 14
    lastFlightMode = flightMode;
  }
}

#endif  // #define _AQ_OSD_MAX7456_AI_H_


