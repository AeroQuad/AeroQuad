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

#ifndef _AQ_OSD_MAX7456_AI_H_
#define _AQ_OSD_MAX7456_AI_H_

//configuration for AI
#define LINE_ROW_0 0x80                             // character address of a character with a horizontal line in row 0. Other rows follow this one
#define AI_MAX_PITCH_ANGLE (PI/4)                   // bounds of scale used for displaying pitch. When pitch is >= |this number|, the pitch lines will be at top or bottom of bounding box
static const byte ROLL_COLUMNS[4] = {10,12,17,19};  // columns where the roll line is printed
#define PITCH_L_COL 7
#define PITCH_R_COL 22
#define AI_DISPLAY_RECT_HEIGHT 9                    // Height of rectangle bounding AI. Should be odd so that there is an equal space above/below the centre reticle

#define RETICLE_ROW (MAX_screen_rows/2)             // centre row - don't change this
#define RETICLE_COL 14                              // reticle will be in this col, and col to the right

#define AI_TOP_PIXEL ((RETICLE_ROW - AI_DISPLAY_RECT_HEIGHT/2)*18)
#define AI_BOTTOM_PIXEL ((RETICLE_ROW + AI_DISPLAY_RECT_HEIGHT/2)*18)
#define AI_CENTRE (RETICLE_ROW*18+10)               // row, in pixels, corresponding to zero pitch/roll.

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// ATTITUDE Display /////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

byte AIoldline[5] = {0,0,0,0,0};
byte lastFlightMode = 9;

void displayArtificialHorizon(float roll, float pitch, byte flightMode) {

  short AIrows[5]   = {0,0,0,0,0};  //Holds the row, in pixels, of AI elements: pitch then roll from left to right.
  //Calculate row of new pitch lines
  AIrows[0] = constrain( (int)AI_CENTRE + (int)((pitch/AI_MAX_PITCH_ANGLE)*(AI_CENTRE-AI_TOP_PIXEL) ), AI_TOP_PIXEL, AI_BOTTOM_PIXEL );  //centre + proportion of full scale
  char pitchLine = LINE_ROW_0 + (AIrows[0] % 18);

  if (AIoldline[0] != AIrows[0]/18) {
    //Remove old pitch lines if not overwritten by new ones
    writeChars( NULL, 1, 0, AIoldline[0], PITCH_L_COL );
    writeChars( NULL, 1, 0, AIoldline[0], PITCH_R_COL );
    AIoldline[0] = AIrows[0]/18;
  }

  //Write new pitch lines
  writeChars( &pitchLine, 1, 0, AIoldline[0], PITCH_L_COL );
  writeChars( &pitchLine, 1, 0, AIoldline[0], PITCH_R_COL );

  //Calculate row (in pixels) of new roll lines
  int distFar  = (ROLL_COLUMNS[3] - (RETICLE_COL + 1))*12 + 6; //horizontal pixels between centre of reticle and centre of far angle line
  int distNear = (ROLL_COLUMNS[2] - (RETICLE_COL + 1))*12 + 6;
  float gradient = 1.4 * roll; // was "tan(roll)", yes rude but damn fast !!
  AIrows[4] = constrain( AI_CENTRE - (int)(((float)distFar)*gradient), AI_TOP_PIXEL, AI_BOTTOM_PIXEL );
  AIrows[3] = constrain( AI_CENTRE - (int)(((float)distNear)*gradient), AI_TOP_PIXEL, AI_BOTTOM_PIXEL );
  AIrows[1] = constrain( 2*AI_CENTRE - AIrows[4], AI_TOP_PIXEL, AI_BOTTOM_PIXEL );
  AIrows[2] = constrain( 2*AI_CENTRE - AIrows[3], AI_TOP_PIXEL, AI_BOTTOM_PIXEL );

  //writing new roll lines to screen
  for (byte i=1; i<5; i++ ) {
    // clear previous roll lines if not going to overwrite
    if (AIoldline[i] != AIrows[i]/18) {
      writeChars( NULL, 1, 0, AIoldline[i], ROLL_COLUMNS[i-1] );
      AIoldline[i] =  AIrows[i]/18;
    }
    //converting rows (in pixels) to character addresses used for the 'lines'
    char RollLine = LINE_ROW_0 + (AIrows[i] % 18);
    writeChars( &RollLine, 1, 0, AIoldline[i], ROLL_COLUMNS[i-1] );
  }

  // Reticle on the center of the screen
  // We have two reticles empty one for RATE_FLIGHT_MODE and one with (s) for 'ATTITUDE_FLIGHT_MODE' mode
  if (lastFlightMode != flightMode) {
    writeChars( (flightMode == 0) ? "\1\2" : "\3\4", 2, 0, RETICLE_ROW, RETICLE_COL ); //write 2 chars to row (middle), column 14
    lastFlightMode = flightMode;
  }
}

#endif  // #define _AQ_OSD_MAX7456_AI_H_


