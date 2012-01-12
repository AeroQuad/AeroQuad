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

#ifndef _AQ_OSD_MAX7456_NOTIFY_H_
#define _AQ_OSD_MAX7456_NOTIFY_H_

byte osdNotificationTimeToShow = 0;
byte osdNotificationFlags      = 0;
byte osdNotificationCursorL    = 0;
byte osdNotificationCursorR    = 0;
char osdNotificationBuffer[29];     // do not change this

byte notifyOSDmenu(byte flags, byte cursorLeft, byte cursorRight, const char *fmt, ...) {

  va_list ap;
  if ((osdNotificationTimeToShow > 0) && ((flags >> 6) < (osdNotificationFlags >> 6))) {
    return 1; // drop message, we tell it to caller
  }
  if (fmt == NULL) {
    // clear
    memset(osdNotificationBuffer, 0, 28);
    osdNotificationFlags = 0;
  }
  else {
    osdNotificationFlags = flags; // will set priority and flags
    osdNotificationTimeToShow = (flags & OSD_NOCLEAR) ? 255 : 50; // set timeout for message
    va_start(ap, fmt);
    byte len=vsnprintf(osdNotificationBuffer, 29, fmt, ap);
    va_end(ap);
    if (len > 28) {
      len = 28;
    }
    memset(osdNotificationBuffer+len, 0, 28-len);
    if (flags & OSD_CENTER) {
      byte i = (28 - len) / 2;
      if (i) {
        // move text right to center it
        memmove(osdNotificationBuffer + i, osdNotificationBuffer, strlen(osdNotificationBuffer));
        memset(osdNotificationBuffer, 0, i);
        // adjust cursor position also if needed
        if (flags & OSD_CURSOR) {
          cursorLeft  += i;
          cursorRight += i;
        }
      }
    }
    osdNotificationCursorL = cursorLeft < 27 ? cursorLeft : 27;
    osdNotificationCursorR = cursorRight < 27 ? cursorRight : 27;
  }
  if (flags & OSD_NOW) {
    displayNotify();
  }
  else {
    osdNotificationFlags |= OSD_NOW; // this will tell next update to show message
  }
  return 0;
}

byte displayNotify(){

  if (osdNotificationFlags & OSD_NOW) {
    osdNotificationFlags &= ~OSD_NOW;
    // we have new message to show

    if ((osdNotificationFlags&OSD_CURSOR) && (osdNotificationCursorL!=255)) {

      if (osdNotificationCursorL > 0) {
        writeChars(osdNotificationBuffer,osdNotificationCursorL,
          ((osdNotificationFlags&OSD_INVERT)?2:0)|((osdNotificationFlags&OSD_BLINK)?1:0),
          NOTIFY_ROW,NOTIFY_COL);
      }

      writeChars(osdNotificationBuffer+osdNotificationCursorL,osdNotificationCursorR-osdNotificationCursorL+1,
        ((osdNotificationFlags&OSD_INVERT)?2:0)|((osdNotificationFlags&OSD_BLINK)?0:1),
        NOTIFY_ROW,NOTIFY_COL+osdNotificationCursorL);

      if (osdNotificationCursorR < 27) {
        writeChars(osdNotificationBuffer+osdNotificationCursorR+1,27-osdNotificationCursorR,
          ((osdNotificationFlags&OSD_INVERT)?2:0)|((osdNotificationFlags&OSD_BLINK)?1:0),
          NOTIFY_ROW,NOTIFY_COL+osdNotificationCursorR+1);
      }
    }
    else {
      writeChars(osdNotificationBuffer,28,
        ((osdNotificationFlags&OSD_INVERT)?2:0)|((osdNotificationFlags&OSD_BLINK)?1:0),
        NOTIFY_ROW,NOTIFY_COL);
    }
    return 1;
  }

  if ((osdNotificationTimeToShow > 0) && (osdNotificationTimeToShow != 255)) {
    if (osdNotificationTimeToShow-- == 1) {
      writeChars(NULL,28,0,NOTIFY_ROW,NOTIFY_COL);
      return 1;
    }
  }
  return 0; // nothing was done
}

#endif  // #define _AQ_OSD_MAX7456_NOTIFY_H_


