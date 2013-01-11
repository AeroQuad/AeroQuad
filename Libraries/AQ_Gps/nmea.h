/*
  AeroQuad v3.x - Sep 2012
  www.AeroQuad.com
  Copyright (c) 2012 AeroQuad developers.  All rights reserved.
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

/* simple NMEA parser */

#ifndef _AQ_nmea_H_
#define _AQ_nmea_H_

#define SENTENCESIZE 80 // Maximum size of sentence (80 in NMEA)
char sentenceBuffer[SENTENCESIZE+1];
unsigned char sentenceLength=0;
unsigned char sentenceCalculatedXOR;
unsigned char sentenceXOR;

enum gpsProcessDataState { WAIT_START, READ, READ_CS1, READ_CS2 } gpsProcessDataState = WAIT_START;

static const char nib2hex[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

// initialize parser
void nmeaInit() {

  gpsProcessDataState = WAIT_START;
}

// read a optionally decimal number from string
// - value returned is always integer as multiplied to include wanted decimal count
// - this will also consume the leading comma (required)
// - out can be NULL to ignore read value
int nmeaGetScaledInt( char **s, long *out, int decimals ) {

  long val=0;
  int  ret=0;

  // read whole numbers (prior to dot)
  while (((**s)>='0') && ((**s) <= '9')) {
    val *= 10;
    val = val + (*((*s)++) - '0');
    ret=1;
  }

  if ((**s)=='.') {
    // we have decimals
    (*s)++;
    while  (decimals--) {
      val *= 10;
      if (((**s)>='0') && ((**s) <= '9')) {
        val += (*((*s)++) - '0');
        ret = 1;
      }
    }
  }
  else {
    while  (decimals--) {
      val *= 10;
    }
  }

  // take off the decimals we did not care about
  while (((**s)>='0') && ((**s) <= '9')) {
    (*s)++;
  }

  if ((**s) == ',') {
    (*s)++;
  }
  else {
    ret=0; // no comma -> fail
  }

  if (ret && out) {
    *out=val;
  }

  return ret;
}

// Get coordinate from NMEA
// input: string "[d]ddmm.mmmm,[NESW],"
int nmeaGetCoord(char **s, long *outp) {
  long raw,deg;
  if (nmeaGetScaledInt(s,&raw,5)) {
    deg = raw / 10000000 * 10000000; // whole degrees
    raw = raw - deg;                 // minutes
    raw = raw * 100 / 60;            // minutes to fractional degrees
    deg = deg + raw;
    switch (**s) {
    case 'S':
    case 'W':
        deg=-deg;
    case 'N':
    case 'E':
      (*s)++;
      break;
    default:
      break;
    }
    if ((**s) == ',') {
      (*s)++;
      if (outp) *outp = deg;
      return 1;
    }
  }
  if ((**s) == ',') (*s)++; // consume the second comma if no number parsed
  return 0;
}

// process received NMEA sentence
void nmeaProcessSentence(){

  gpsData.sentences++;
  char *p = sentenceBuffer;
  if (!strncmp(p,"GPGGA,",6)) {
    long work;
    p+=6;

    gpsData.fixtime = (nmeaGetScaledInt(&p, &work, 3)) ? work : GPS_INVALID_FIX_TIME;
    gpsData.lat = (nmeaGetCoord(&p,&work)) ? work : GPS_INVALID_ANGLE;
    gpsData.lon = (nmeaGetCoord(&p,&work)) ? work : GPS_INVALID_ANGLE;
    nmeaGetScaledInt(&p, NULL,0); //fix quality
    gpsData.sats = (nmeaGetScaledInt(&p,&work,0)) ? work : 0;
    gpsData.accuracy = (nmeaGetScaledInt(&p,&work,3)) ? work : GPS_INVALID_ACCURACY; //hdop
    gpsData.height = (nmeaGetScaledInt(&p,&work,3)) ? work : GPS_INVALID_ALTITUDE;
  }
  else if (!strncmp(p,"GPGSA,",6)) {
    long work;
    p+=6;

    p++; // validity info, ignored
    if (*(p++) != ',') {
	  return;
	}

    if (nmeaGetScaledInt(&p,&work,0)) {
      gpsData.state = work;
    }
  }
  else if (!strncmp(p,"GPRMC,",6)) {
    long work;
    p+=6;

    gpsData.fixtime = (nmeaGetScaledInt(&p, &work, 3)) ? work : GPS_INVALID_FIX_TIME;

    p++; // fix status - ignored
    if (*(p++) != ',') {
	  return;
	}

    gpsData.lat = (nmeaGetCoord(&p,&work)) ? work : GPS_INVALID_ANGLE;
    gpsData.lon = (nmeaGetCoord(&p,&work)) ? work : GPS_INVALID_ANGLE;
    gpsData.speed = (nmeaGetScaledInt(&p, &work, 3)) ? work * 5144 / 100000 : GPS_INVALID_SPEED; // kt -> cm/s
    gpsData.course = (nmeaGetScaledInt(&p, &work, 3)) ? work : 0;
  }
}

int nmeaProcessData(unsigned char data) {
  int parsed = 0;

  switch (gpsProcessDataState) {
  case WAIT_START:
    if (data == '$') {
      gpsProcessDataState = READ;
      sentenceLength = 0;
      sentenceCalculatedXOR = 0;
    }
    break;
	
  case READ:
    if (data == '*') {
      sentenceBuffer[sentenceLength] = 0; // ensure NUL at end
      gpsProcessDataState = READ_CS1;
    }
    else if (sentenceLength < SENTENCESIZE) {
      sentenceBuffer[sentenceLength++] = data;
      sentenceCalculatedXOR ^= data;
    }
    else {
      // overrun !!
      gpsProcessDataState=WAIT_START;
    }
    break;
	
  case READ_CS1:
    if (data == nib2hex[sentenceCalculatedXOR>>4]) {
      gpsProcessDataState = READ_CS2;
    }
    else {
      gpsProcessDataState=WAIT_START;
    }
    break;
	
  case READ_CS2:
    if (data == nib2hex[sentenceCalculatedXOR & 0xf]) {
      parsed=1;
      nmeaProcessSentence();
    }
    else {
    }
    gpsProcessDataState=WAIT_START;
    break;
  }
  return parsed;
}

#endif



