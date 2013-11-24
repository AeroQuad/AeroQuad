/*
  AeroQuad v3.x - 2013
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

#ifndef GCS_MAVLink_h
#define GCS_MAVLink_h

#define MAVLINK_SEND_UART_BYTES(chan, buf, len) comm_send_buffer(chan, buf, len)

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "include/mavlink/v1.0/mavlink_types.h"

mavlink_system_t mavlink_system;

/// Send a byte to the nominated MAVLink channel
///
/// @param chan		Channel to send to
/// @param ch		Byte to send
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    switch(chan) {
	case MAVLINK_COMM_0:
		SERIAL_PORT.write(ch);
		break;
	default:
		break;
	}
}

/*
  send a buffer out a MAVLink channel
 */
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len)
{
    switch(chan) {
	case MAVLINK_COMM_0:
		SERIAL_PORT.write(buf, len);
		break;
	default:
		break;
	}
}

///// Read a byte from the nominated MAVLink channel
/////
///// @param chan	Channel to receive on
///// @returns		Byte read
static inline uint8_t comm_receive_ch(mavlink_channel_t chan)
{
    uint8_t data = 0;

    switch(chan) {
	case MAVLINK_COMM_0:
		data = SERIAL_PORT.read();
		break;
	default:
		break;
	}
    return data;
}
#endif // GCS_MAVLink_h