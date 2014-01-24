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

#ifndef _AQ_OSD_MAX7456_HEADING_H_
#define _AQ_OSD_MAX7456_HEADING_H_

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// HeadingMagHold Display ///////////////////////////
//////////////////////////////////////////////////////////////////////////////

int lastHeadingDeg = 361; 	// bogus to force update
static char buf_show[12];	// compass bar buffer

// N=0x4e; E=0x45; S=0x53; W=0x57; -=0x2d; |=0x7c;
const char buf_Rule[36] = {0x4e,0x2d,0x2d,0x7c,0x2d,0x2d,0x7c,0x2d,0x2d,
                           0x45,0x2d,0x2d,0x7c,0x2d,0x2d,0x7c,0x2d,0x2d,
                           0x53,0x2d,0x2d,0x7c,0x2d,0x2d,0x7c,0x2d,0x2d,
                           0x57,0x2d,0x2d,0x7c,0x2d,0x2d,0x7c,0x2d,0x2d};

void displayHeading(float currentHeading) 
{
	float deg = degrees(currentHeading);
  	int currentHeadingDeg = (int)( 0.5 + (deg<-0.5 ? deg+360.0 : deg));

  	if (currentHeadingDeg != lastHeadingDeg) 
	{
    	char buf[6];
		
		snprintf(buf,6,"\026%3d\027", currentHeadingDeg); // \026 is compass \027 is degree symbol
		writeChars( buf, 5, 0, COMPASS_ROW, COMPASS_COL );
    	
		int lastPos = round((lastHeadingDeg * 36)/360);		
		int currentPos = round((currentHeadingDeg * 36)/360);
		
		// Only execute this code when the pos is changed.
		if(currentPos != lastPos)
		{
			currentPos -= 5;
			
			if(currentPos < 0)
			{
				currentPos += 36;
			}
			
			for(int x=0; x <= 10; ++x)
			{
				buf_show[x] = buf_Rule[currentPos];
				
				if(++currentPos > 35)
				{
					currentPos = 0;
				}
			}
			
			buf_show[11] = '\0';
			writeChars( buf_show, 11, 0, COMPASS_BAR_ROW, COMPASS_BAR_COL );
		}
		
		lastHeadingDeg = currentHeadingDeg;
	}
}

#endif  // #define _AQ_OSD_MAX7456_HEADING_H_


