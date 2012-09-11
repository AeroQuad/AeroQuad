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

  if (ret && out) *out=val;
  
  return ret;
}

// Get coordinate from NMEA
// input: string "[d]ddmm.mmmm,[NESW]"
int nmeaGetCoord(char **s, long *outp) {
  long raw,deg;
  if (nmeaGetScaledInt(s,&raw,5)) {
    deg = raw / 10000000 * 10000000; // whole degrees
    raw = raw - deg;                 // minutes
    raw = raw * 100 / 60;            // minutes to fractional degrees
    deg = deg + raw;
    if (**s == ',') {
      (*s)++;
      if ((**s == 'S') || (**s == 'W')) {
        deg=-deg;
      }
      (*s)++;
      if (outp) *outp = deg;
      return 1;
    }
  }
  return 0;
}

void nmeaProcessSentence(){

  gpsData.sentences++;
  char *p = sentenceBuffer;
  if (!strncmp(p,"GPGGA,",6)) {
    long work,workd; int decs;
    p+=6;

    nmeaGetScaledInt(&p, &work, 3);
    gpsData.fixtime = work;
    if (*(p++) != ',') return;

    nmeaGetCoord(&p,&work);
    gpsData.lat = work;
    if (*(p++) != ',') return;

    nmeaGetCoord(&p,&work);
    gpsData.lon = work;
    if (*(p++) != ',') return;

    nmeaGetScaledInt(&p,&work,0); //fix quality
    if (*(p++) != ',') return;

    nmeaGetScaledInt(&p,&work,0); // num sats
    gpsData.sats = work;
    if (*(p++) != ',') return;

    nmeaGetScaledInt(&p,&work,3); //hdop
    gpsData.accuracy = work;
    if (*(p++) != ',') return;

    nmeaGetScaledInt(&p,&work,3); //altitude
    gpsData.accuracy = work;
    if (*(p++) != ',') return;


  }
  else if (!strncmp(p,"GPGSA,",6)) {
    long work,workd;
    p+=6;
    
    p++;
    if (*(p++) != ',') return;

    nmeaGetScaledInt(&p,&work,0);
    gpsData.state = work;
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



