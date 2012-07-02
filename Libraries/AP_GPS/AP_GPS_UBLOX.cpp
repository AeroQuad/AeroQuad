// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
//  u-blox UBX GPS driver for ArduPilot and ArduPilotMega.
//	Code by Michael Smith, Jordi Munoz and Jose Julio, DIYDrones.com
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//

#define UBLOX_DEBUGGING 0

#if UBLOX_DEBUGGING
#include <FastSerial.h>
# define Debug(fmt, args...)  do {Serial.printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__ , ##args); delay(0); } while(0)
#else
# define Debug(fmt, args...)
#endif

#include "AP_GPS_UBLOX.h"
#include <stdint.h>


// Constructors ////////////////////////////////////////////////////////////////

AP_GPS_UBLOX::AP_GPS_UBLOX(Stream *s) : GPS(s)
{
}

// Public Methods //////////////////////////////////////////////////////////////

void
AP_GPS_UBLOX::init()
{
    // XXX it might make sense to send some CFG_MSG,CFG_NMEA messages to get the
    // right reporting configuration.

    _port->flush();

    _epoch = TIME_OF_WEEK;
    idleTimeout = 1200;	

	// configure the GPS for the messages we want
	_configure_gps();

//	_nav_setting = nav_setting;
}

// Process bytes available from the stream
//
// The stream is assumed to contain only messages we recognise.  If it
// contains other messages, and those messages contain the preamble
// bytes, it is possible for this code to fail to synchronise to the
// stream immediately.  Without buffering the entire message and
// re-processing it from the top, this is unavoidable. The parser
// attempts to avoid this when possible.
//
bool
AP_GPS_UBLOX::read(void)
{
    uint8_t		data;
    int 		numc;
    bool		parsed = false;

    numc = _port->available();
    for (int i = 0; i < numc; i++) {	// Process bytes received

        // read the next byte
        data = _port->read();

        switch(_step) {

            // Message preamble detection
            //
            // If we fail to match any of the expected bytes, we reset
            // the state machine and re-consider the failed byte as
            // the first byte of the preamble.  This improves our
            // chances of recovering from a mismatch and makes it less
            // likely that we will be fooled by the preamble appearing
            // as data in some other message.
            //
        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
			Debug("reset %u", __LINE__);
            // FALLTHROUGH
        case 0:
            if(PREAMBLE1 == data)
                _step++;
            break;

            // Message header processing
            //
            // We sniff the class and message ID to decide whether we
            // are going to gather the message bytes or just discard
            // them.
            //
            // We always collect the length so that we can avoid being
            // fooled by preamble bytes in messages.
            //
        case 2:
            _step++;
			_class = data;
			_ck_b = _ck_a = data;			// reset the checksum accumulators
            break;
        case 3:
            _step++;
            _ck_b += (_ck_a += data);			// checksum byte
            _msg_id = data;
            break;
        case 4:
            _step++;
            _ck_b += (_ck_a += data);			// checksum byte
            _payload_length = data;				// payload length low byte
            break;
        case 5:
            _step++;
            _ck_b += (_ck_a += data);			// checksum byte

            _payload_length += (uint16_t)(data<<8);
			if (_payload_length > 512) {
				Debug("large payload %u", (unsigned)_payload_length);
				// assume very large payloads are line noise
				_payload_length = 0;
				_step = 0;
			}
            _payload_counter = 0;				// prepare to receive payload
            break;

            // Receive message data
            //
        case 6:
            _ck_b += (_ck_a += data);			// checksum byte
			if (_payload_counter < sizeof(_buffer)) {
				_buffer.bytes[_payload_counter] = data;
			}
            if (++_payload_counter == _payload_length)
                _step++;
            break;

            // Checksum and message processing
            //
        case 7:
            _step++;
            if (_ck_a != data) {
				Debug("bad cka %x should be %x", data, _ck_a);
                _step = 0;						// bad checksum
			}
            break;
        case 8:
            _step = 0;
            if (_ck_b != data) {
				Debug("bad ckb %x should be %x", data, _ck_b);
                break;							// bad checksum
			}

			if (_parse_gps()) {
				parsed = true;
            }
        }
    }
    return parsed;
}

// Private Methods /////////////////////////////////////////////////////////////

bool
AP_GPS_UBLOX::_parse_gps(void)
{
	if (_class == CLASS_ACK) {
		Debug("ACK %u", (unsigned)_msg_id);
		return false;
	}

/*	if (_class == CLASS_CFG && _msg_id == MSG_CFG_NAV_SETTINGS) {
		if (_nav_setting != GPS_ENGINE_NONE && 
			_buffer.nav_settings.dynModel != _nav_setting) {
			// we've received the current nav settings, change the engine
			// settings and send them back
			Debug("Changing engine setting from %u to %u\n",
				  (unsigned)_buffer.nav_settings.dynModel, (unsigned)_nav_setting);
			_buffer.nav_settings.dynModel = _nav_setting;
			_send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS, 
						  &_buffer.nav_settings, 
						  sizeof(_buffer.nav_settings));
		}
		return false;
	}
*/	

	if (_class != CLASS_NAV) {
		Debug("Unexpected message 0x%02x 0x%02x", (unsigned)_class, (unsigned)_msg_id);
		if (++_disable_counter == 0) {
			// disable future sends of this message id, but
			// only do this every 256 messages, as some
			// message types can't be disabled and we don't
			// want to get into an ack war
			Debug("Disabling message 0x%02x 0x%02x", (unsigned)_class, (unsigned)_msg_id);
			_configure_message_rate(_class, _msg_id, 0);
		}
		return false;
	}

    switch (_msg_id) {
    case MSG_POSLLH:
		Debug("MSG_POSLLH next_fix=%u", next_fix);
        time		= _buffer.posllh.time;
        longitude	= _buffer.posllh.longitude;
        latitude	= _buffer.posllh.latitude;
        altitude	= _buffer.posllh.altitude_msl / 10;
		fix			= next_fix;
		_new_position = true;
		break;
    case MSG_STATUS:
		Debug("MSG_STATUS fix_status=%u fix_type=%u",
					  _buffer.status.fix_status,
					  _buffer.status.fix_type);
        next_fix	= (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
		if (!next_fix) {
			fix = false;
		}
        break;
    case MSG_SOL:
		Debug("MSG_SOL fix_status=%u fix_type=%u",
					  _buffer.solution.fix_status,
					  _buffer.solution.fix_type);
        next_fix	= (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
		if (!next_fix) {
			fix = false;
		}
        num_sats	= _buffer.solution.satellites;
        hdop		= _buffer.solution.position_DOP;
        break;
    case MSG_VELNED:
		Debug("MSG_VELNED");
        speed_3d	= _buffer.velned.speed_3d;				// cm/s
        ground_speed = _buffer.velned.speed_2d;				// cm/s
        ground_course = _buffer.velned.heading_2d / 1000;	// Heading 2D deg * 100000 rescaled to deg * 100
		_new_speed = true;
        break;
    default:
		Debug("Unexpected NAV message 0x%02x", (unsigned)_msg_id);
		if (++_disable_counter == 0) {
			Debug("Disabling NAV message 0x%02x", (unsigned)_msg_id);
			_configure_message_rate(CLASS_NAV, _msg_id, 0);
		}
        return false;
    }

	// we only return true when we get new position and speed data
	// this ensures we don't use stale data
	if (_new_position && _new_speed) {
		_new_speed = _new_position = false;
		return true;
	}
	return false;
}


// UBlox auto configuration

/*
  update checksum for a set of bytes
 */
void
AP_GPS_UBLOX::_update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b)
{
	while (len--) {
		ck_a += *data;
		ck_b += ck_a;
		data++;
	}
}


/*
  send a ublox message
 */
void
AP_GPS_UBLOX::_send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint8_t size)
{
	struct ubx_header header;
	uint8_t ck_a=0, ck_b=0;
	header.preamble1 = PREAMBLE1;
	header.preamble2 = PREAMBLE2;
	header.msg_class = msg_class;
	header.msg_id    = msg_id;
	header.length    = size;

	_update_checksum((uint8_t *)&header.msg_class, sizeof(header)-2, ck_a, ck_b);
	_update_checksum((uint8_t *)msg, size, ck_a, ck_b);

	_port->write((const uint8_t *)&header, sizeof(header));
	_port->write((const uint8_t *)msg, size);
	_port->write((const uint8_t *)&ck_a, 1);
	_port->write((const uint8_t *)&ck_b, 1);
}


/*
  configure a UBlox GPS for the given message rate for a specific
  message class and msg_id
 */
void
AP_GPS_UBLOX::_configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
	struct ubx_cfg_msg_rate msg;
	msg.msg_class = msg_class;
	msg.msg_id    = msg_id;
	msg.rate	  = rate;
	_send_message(CLASS_CFG, MSG_CFG_SET_RATE, &msg, sizeof(msg));
}

/*
  configure a UBlox GPS for the given message rate
 */
void
AP_GPS_UBLOX::_configure_gps(void)
{
	struct ubx_cfg_nav_rate msg;

	// this type 0x41 pubx sets us up for 38400 with
	// NMEA+UBX input and UBX output
	_send_pubx("$PUBX,41,1,0003,0001,38400,0");

	// ask for navigation solutions every 200ms
	msg.measure_rate_ms = 200;
	msg.nav_rate        = 1;
	msg.timeref         = 0; // UTC time
	_send_message(CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(msg));

	// ask for the messages we parse to be sent on every navigation solution
	_configure_message_rate(CLASS_NAV, MSG_POSLLH, 1);
	_configure_message_rate(CLASS_NAV, MSG_STATUS, 1);
	_configure_message_rate(CLASS_NAV, MSG_SOL, 1);
	_configure_message_rate(CLASS_NAV, MSG_VELNED, 1);

	// ask for the current navigation settings
	_send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS, NULL, 0);	
}

void
AP_GPS_UBLOX::_send_pubx(const char *msg)
{
	uint8_t csum = 0;
	char suffix[4];
	for (uint8_t i=1; msg[i]; i++) {
		csum ^= msg[i];
	}
	_port->write(msg);
	sprintf(suffix, "*%02x", (unsigned)csum);
	_port->write((const uint8_t *)suffix, 3);
}
