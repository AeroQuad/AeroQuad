/* simple UBLOX parser */

#ifndef _AQ_nmea_H_
#define _AQ_nmea_H_

#define SENTENCESIZE 80 // Maximum size of sentence (80 in NMEA)
char sentenceBuffer[SENTENCESIZE+1];
unsigned char sentenceLength=0;
unsigned char sentenceCalculatedXOR;
unsigned char sentenceXOR;

enum gpsProcessDataState { WAIT_START, READ, READ_CS1, READ_CS2 } gpsProcessDataState = WAIT_START;

static const char nib2hex[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};



void nmeaInit() {
  
  gpsProcessDataState = WAIT_START;
}

// Grab a number from NMEA sentence with optional decimals
// *p is adjusted for characters extracted
// out,decimals,decimalcount can be NULL - in this case data is discarded
// return 1 if a number was successfully extracted, 0 otherwise

int nmeaGetIntDotInt(char **s, long *out, long *decimals, int *decimalcount) {

  long val=0;
  int  ret=0;
  int  d = 0;

  while (((**s)>='0') && ((**s) <= '9')) {
    val *= 10;
    val = val + (*((*s)++) - '0');
    ret=1;
  }
  if (out) *out = val;
  if ((**s)=='.') {
    (*s)++;
    val=0;
    while (((**s)>='0') && ((**s) <= '9')) {
      val *= 10;
      val = val + (*((*s)++) - '0');
      d++;
      ret=1;
    }
  }

  if (decimals) *decimals = val;

  if (decimalcount) *decimalcount = d;

  return ret;
}

// Get coordinate from NMEA
// input: string "[d]ddmm.mmmm,[NESW]"
int nmeaGetCoord(char **s, long *outp) {
  long min,deg,out;
  int decimals;
  if (nmeaGetIntDotInt(s,&deg,&min,&decimals)) {
    out = (deg / 100) * 10000000;
    while (decimals < 5) {
      min = 10 * min;
      decimals++;
    }
    while (decimals > 5) {
      min = min/10;
      decimals--;
    }
    // convert from minutes to degrees
    out += ((deg % 100) * 100000 + min) * 100 / 60;
    if (**s == ',') {
      (*s)++;
      if ((**s == 'S') || (**s == 'E')) {
	out=-out;
      }
      (*s)++;
      if (outp) *outp=out;
      return 1;
    }
  }
  return 0;
}


void nmeaProcessSentence(){

  char *p = sentenceBuffer;
  if (!strncmp(p,"GPGGA,",6)) {
    long fixtime,lat=0,lon=0;
    p+=6;

    nmeaGetIntDotInt(&p,&fixtime,NULL,NULL);
    if (*(p++) != ',') return;
    nmeaGetCoord(&p,&lat);
    if (*(p++) != ',') return;
    nmeaGetCoord(&p,&lon);

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



