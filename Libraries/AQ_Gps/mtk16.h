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

/* simple MTK binary v1.6 parser */

#ifndef _AQ_mtk16_H_
#define _AQ_mtk16_H_

/* MTK INIT STRINGS

#define MTK_SET_BINARY "$PGCMD,16,0,0,0,0,0*6A\r\n"
#define MTK_SET_NMEA   "$PGCMD,16,1,1,1,1,1*6B\r\n"

#define MTK_OUTPUT_1HZ  "$PMTK220,1000*1F\r\n"
#define MTK_OUTPUT_2HZ  "$PMTK220,500*2B\r\n"
#define MTK_OUTPUT_4HZ  "$PMTK220,250*29\r\n"
#define MTK_OUTPUT_5HZ  "$PMTK220,200*2C\r\n"
#define MTK_OUTPUT_10HZ "$PMTK220,100*2F\r\n"

#define MTK_BAUD_RATE_38400 "$PMTK251,38400*27\r\n"

#define MTK_SBAS_ON  "$PMTK313,1*2E\r\n"
#define MTK_SBAS_OFF "$PMTK313,0*2F\r\n"
 */

static const char *STR_MTKPINNINGOFF = "$PMTK397,0*23\r\n";
static const char *STR_MTK5HZ =        "$PMTK220,200*2C\r\n";
static const char *STR_MTKBINARY =     "$PGCMD,16,0,0,0,0,0*6A\r\n";
static const char *STR_MTK38400 =      "$PMTK251,38400*27\r\n";

#define CONFIG_MTKPINNINGOFF {(byte *)STR_MTKPINNINGOFF, 0}
#define CONFIG_MTK38400      {(byte *)STR_MTK38400, 0}
#define CONFIG_MTK5HZ        {(byte *)STR_MTK5HZ, 0}
#define CONFIG_MTKBINARY     {(byte *)STR_MTKBINARY, 0}

#ifdef UseGPSMTKBINARY
  #define MTK_CONFIGS  CONFIG_MTKPINNINGOFF, CONFIG_MTK38400, CONFIG_MTK5HZ, CONFIG_MTKBINARY
#else
  #define MTK_CONFIGS  CONFIG_MTKPINNINGOFF, CONFIG_MTK38400, CONFIG_MTK5HZ
#endif

// MTK diydrones v1.6 binary packet format
struct __attribute__((packed)) mtk16_fix {
  int32_t         latitude;
  int32_t         longitude;
  int32_t         altitude;
  int32_t         ground_speed;
  int32_t         ground_course;
  uint8_t         satellites;
  uint8_t         fix_type;
  uint32_t        utc_date;
  uint32_t        utc_time;
  uint16_t        hdop;
};

union mtk16_message {
  struct mtk16_fix msg;
  unsigned char raw[32];
} mtk16Message;

unsigned short mtk16DataLength;
unsigned short mtk16ExpectedDataLength;
unsigned char  mtk16CKA,mtk16CKB;

enum mtk16State{ MTK16_WAIT_SYNC1, MTK16_WAIT_SYNC2, MTK16_GET_LEN, MTK16_GET_DATA, MTK16_GET_CKA, MTK16_GET_CKB  } mtk16ProcessDataState;

void mtk16Init() {

  mtk16ProcessDataState = MTK16_WAIT_SYNC1;
}

// Copy data to gspData structure from received binary packet
void mtk16ParseData() {

  if (mtk16DataLength == 32) {
    gpsData.sentences++;
    gpsData.lat = mtk16Message.msg.latitude * 10;
    gpsData.lon = mtk16Message.msg.longitude * 10;
    gpsData.height = mtk16Message.msg.altitude;
    gpsData.accuracy = mtk16Message.msg.hdop;
    gpsData.fixtime = mtk16Message.msg.utc_time;
    switch (mtk16Message.msg.fix_type) {
      case 2:
        gpsData.state = GPS_FIX2D;
        break;
      case 3:
        gpsData.state = GPS_FIX3D;
        break;
      default:
        gpsData.state = GPS_NOFIX;
        break;
    }
    gpsData.sats = mtk16Message.msg.satellites;
    gpsData.course = mtk16Message.msg.ground_course;
    gpsData.speed = mtk16Message.msg.ground_speed;
  }
}

// Parse data from GPS
int mtk16ProcessData(unsigned char data) {

  int parsed = 0;

  switch (mtk16ProcessDataState) {
  case MTK16_WAIT_SYNC1:
    if (data == 0xd0) {
      mtk16ProcessDataState = MTK16_WAIT_SYNC2;
    }
    break;
	
  case MTK16_WAIT_SYNC2:
    if (data == 0xdd) {
      mtk16ProcessDataState = MTK16_GET_LEN;
      mtk16DataLength = 0;
    }
    else if (data != 0xd0) {
      mtk16ProcessDataState = MTK16_WAIT_SYNC1;
    }
    break;
	
  case MTK16_GET_LEN:
    mtk16CKA = data;
    mtk16CKB = mtk16CKA;
    mtk16ExpectedDataLength = data;
    if (mtk16ExpectedDataLength > sizeof(mtk16Message)) {
      // discard overlong message
      mtk16ProcessDataState = MTK16_WAIT_SYNC1;
    }
    else {
      mtk16ProcessDataState = MTK16_GET_DATA;
    }
    break;
	
  case MTK16_GET_DATA:
    mtk16CKA += data;
    mtk16CKB += mtk16CKA;
    // next will discard data if it exceed out biggest parsed msg
    if (mtk16DataLength < sizeof(mtk16Message)) {
      mtk16Message.raw[mtk16DataLength++] = data;
    }
    if (mtk16DataLength >= mtk16ExpectedDataLength) {
      mtk16ProcessDataState = MTK16_GET_CKA;
    }
    break;
	
  case MTK16_GET_CKA:
    if (mtk16CKA != data) {
      mtk16ProcessDataState = MTK16_WAIT_SYNC1;
    }
	else {
      mtk16ProcessDataState = MTK16_GET_CKB;
    }
    break;
	
  case MTK16_GET_CKB:
    if (mtk16CKB == data) {
      parsed = 1;
      mtk16ParseData();
    }
    mtk16ProcessDataState = MTK16_WAIT_SYNC1;
    break;
	
  }
  return parsed;
}

#endif
