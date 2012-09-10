/* simple UBLOX parser */

/*
typedef unsigned int uint32_t;
typedef int int32_t;
typedef unsigned short uint16_t;
typedef short int16_t;
typedef unsigned char uint8_t;
*/

#define UBLOX_MAX 128

struct ublox_NAV_STATUS { // 01 03 (16)
  uint32_t iTow;
  uint8_t  gpsFix;
  uint8_t  flags;
  uint8_t  fixStat;
  uint8_t  flags2;
  uint32_t ttfx;
  uint32_t msss;
};

struct ublox_NAV_POSLLH { // 01 02 (28)
  uint32_t iTow;
  int32_t lon; // 1e-7 degrees
  int32_t lat; // 1e-7 degrees
  int32_t height; // mm
  int32_t hMSL; // mm
  uint32_t hAcc; //mm
  uint32_t vAcc; //mm
};

struct ublox_NAV_SOL { // 01 6 (52)
  uint32_t iTow;
  int32_t  fTow;
  int16_t  week;
  uint8_t  gspFix;
  uint8_t  flags;
  int32_t  ecefX;
  int32_t  ecefY;
  int32_t  ecefZ;
  int32_t  pAcc;
  int32_t  ecefVX;
  int32_t  ecefVY;
  int32_t  ecefVZ;
  int32_t  sAcc;
  uint16_t pDOP;
  uint8_t  res1;
  uint8_t  numSV;
  uint32_t res2;
};

struct ublox_NAV_VELNED { // 01 12h (36)
  uint32_t iTow;
  int32_t  velN; // cm/s
  int32_t  velE; // cm/s
  int32_t  velD; // cm/s
  uint32_t  speed; // cm/s
  uint32_t  gSpeed; // cm/s
  int32_t  heading; // dev 1e-5
  uint32_t sAcc; // cm/s
  uint32_t cAcc; // deg 1e-5
};

union ublox_message {
  struct ublox_NAV_STATUS nav_status;
  struct ublox_NAV_POSLLH nav_posllh;
  struct ublox_NAV_VELNED nav_velned;
  struct ublox_NAV_SOL nav_sol;
  unsigned char raw[UBLOX_MAX];
} ubloxMessage;

unsigned short ubloxExpectedDataLength;
unsigned short ubloxDataLength;
unsigned short ubloxClass,ubloxId;
unsigned char  ubloxCKA,ubloxCKB;

struct fixdata {
  long lat,lon;
  unsigned char fixtype;
};

enum { WAIT_SYNC1, WAIT_SYNC2, GET_CLASS, GET_ID, GET_LL, GET_LH, GET_DATA, GET_CKA, GET_CKB  } ubloxProcessDataState = WAIT_SYNC1;


void ubloxInit() {
  
  ubloxProcessDataState = WAIT_SYNC1;
}

void ubloxParseData() {// uses publib vars

  if (ubloxClass==1) { //NAV
    if (ubloxId==2) { //POSLLH
      

/*
      printf("TOW: %6d LA %10d LO %10d Z %d ZS %d HA %d VA %d\n",
	     ubloxMessage.nav_posllh.iTow,
	     ubloxMessage.nav_posllh.lon,
	     ubloxMessage.nav_posllh.lat,
	     ubloxMessage.nav_posllh.height,
	     ubloxMessage.nav_posllh.hMSL,
	     ubloxMessage.nav_posllh.hAcc,
	     ubloxMessage.nav_posllh.vAcc);
	     */
    }
    else if (ubloxId==3) { //STATUS
/*
      printf("TOW: %6d gpsf %d fl %02x fixstat %02x fl2 %02x ttfx %d msss %d\n",
	     ubloxMessage.nav_status.iTow,
	     ubloxMessage.nav_status.gpsFix,
	     ubloxMessage.nav_status.flags,
	     ubloxMessage.nav_status.fixStat,
	     ubloxMessage.nav_status.flags2,
	     ubloxMessage.nav_status.ttfx,
	     ubloxMessage.nav_status.msss);
	     */
    }
    else if (ubloxId==6) { // VELNED
      /*
      printf("TOW: %6d numSV %d\n",
	     ubloxMessage.nav_sol.iTow,
	     ubloxMessage.nav_sol.numSV);
*/
    }
    else if (ubloxId==18) { // VELNED
      /*
        printf("TOW: %6d velN %d E %d D %d speed %d gspeed %d, hdg %d sacc %d hacc %d\n",
	     ubloxMessage.nav_velned.iTow,
	     ubloxMessage.nav_velned.velN,
	     ubloxMessage.nav_velned.velE,
	     ubloxMessage.nav_velned.velD,
	     ubloxMessage.nav_velned.speed,
	     ubloxMessage.nav_velned.gSpeed,
	     ubloxMessage.nav_velned.heading,
	     ubloxMessage.nav_velned.sAcc,
	     ubloxMessage.nav_velned.cAcc);
	      */
    }
    else {
      //printf("NAV? %d\n",ubloxId);
    }
  } 
}

int ubloxProcessData(unsigned char data) {
  
  int parsed = 0;
  
  switch (ubloxProcessDataState) {
  case WAIT_SYNC1:
    if (data == 0xb5) {
      ubloxProcessDataState = WAIT_SYNC2;
    }
    break;
  case WAIT_SYNC2:
    if (data == 0x62) {
      ubloxProcessDataState = GET_CLASS;
    }
    else {
      ubloxProcessDataState = WAIT_SYNC1;
    }
    break;
  case GET_CLASS:
    ubloxClass=data;
    ubloxCKA=data;
    ubloxCKB=data;
    ubloxProcessDataState = GET_ID;
    break;
  case GET_ID:
    ubloxId=data;
    ubloxCKA += data;
    ubloxCKB += ubloxCKA;
    ubloxProcessDataState = GET_LL;
    break;
  case GET_LL:
    ubloxExpectedDataLength = data;
    ubloxCKA += data;
    ubloxCKB += ubloxCKA;
    ubloxProcessDataState = GET_LH;
    break;
  case GET_LH:
    ubloxExpectedDataLength += data << 8;
    ubloxDataLength=0;
    ubloxCKA += data;
    ubloxCKB += ubloxCKA;
    ubloxProcessDataState = GET_DATA;
    break;
  case GET_DATA:
    ubloxCKA += data;
    ubloxCKB += ubloxCKA;
    ubloxMessage.raw[ubloxDataLength++] = data;
    if (ubloxDataLength >= ubloxExpectedDataLength) {
      ubloxProcessDataState = GET_CKA;
    }
    break;
  case GET_CKA:
    if (ubloxCKA != data) {
      ubloxProcessDataState = WAIT_SYNC1;
    } else {
      ubloxProcessDataState = GET_CKB;
    }
    break;
  case GET_CKB:
    if (ubloxCKB == data) {
      ubloxParseData();
      parsed = 1;
    }
    ubloxProcessDataState = WAIT_SYNC1;
    break;
  }
  return parsed;
}

