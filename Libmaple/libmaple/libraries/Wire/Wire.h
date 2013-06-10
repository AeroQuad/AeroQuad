/* *****************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ****************************************************************************/

/**
 *  @brief Wire library, ported from Arduino. Provides a lean
 *  interface to I2C (two-wire) communication.
 */

#include "wirish.h"

#ifndef _WIRE_H_
#define _WIRE_H_

typedef struct {
  uint8 scl;
  uint8 sda;
} Port;

/* You must update the online docs if you change this value. */
#define WIRE_BUFSIZ 32

/* return codes from endTransmission() */
#define SUCCESS   0        /* transmission was successful */
#define EDATA     1        /* too much data */
#define ENACKADDR 2        /* received nack on transmit of address */
#define ENACKTRNS 3        /* received nack on transmit of data */
#define EOTHER    4        /* other error */

#define SDA 20
#define SCL 21

#define I2C_WRITE 0
#define I2C_READ  1

#if (F_CPU == 168000000)
	#define I2C_DELAY_SCL delay_ns100(6) 
	#define I2C_DELAY_SDA delay_ns100(2)
#else
	#define I2C_DELAY_SCL
	#define I2C_DELAY_SDA
#endif

class TwoWire {
 private:
    uint8 rx_buf[WIRE_BUFSIZ];      /* receive buffer */
    uint8 rx_buf_idx;               /* first unread idx in rx_buf */
    uint8 rx_buf_len;               /* number of bytes read */

    uint8 tx_addr;                  /* address transmitting to */
    uint8 tx_buf[WIRE_BUFSIZ];      /* transmit buffer */
    uint8 tx_buf_idx;               /* next idx available in tx_buf, -1 overflow */
    boolean tx_buf_overflow;
    Port port;
    uint8 writeOneByte(uint8);
    uint8 readOneByte(uint8, uint8*);
 public:
    TwoWire();
    void begin();
    void begin(uint8, uint8);
    void beginTransmission(uint8);
    void beginTransmission(int);
    uint8 endTransmission(void);
    uint8 requestFrom(uint8, int);
    uint8 requestFrom(int, int);
    void send(uint8);
    void send(uint8*, int);
    void send(int);
    void send(int*, int);
    void send(char*);
    uint8 available();
    uint8 receive();

    uint8 read() { return receive(); };
    void write(uint8 data) { send(data); };
    void write(uint8* buf, int len) { send(buf, len); };
    void write(int data) { send(data); };
    void write(int* buf, int len) { send(buf, len); };
    void write(char* buf) { send(buf); };
};

void    i2c_start(Port port);
void    i2c_stop(Port port);
boolean i2c_get_ack(Port port);
void    i2c_send_ack(Port port);
void    i2c_send_nack(Port port);
uint8   i2c_shift_in(Port port);
void    i2c_shift_out(Port port, uint8 val);

extern TwoWire Wire;

#endif // _WIRE_H_
