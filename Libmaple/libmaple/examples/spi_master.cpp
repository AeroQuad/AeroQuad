/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @brief Sample main.cpp file. Sends "Hello world!" out SPI1.
 *
 * SPI1 is set up to be a master transmitter at 4.5MHz, little
 * endianness, and SPI mode 0.
 *
 * Pin 10 is used as slave select.
 */

#include "wirish.h"

#define NSS 10

byte buf[] = "Hello world!";

HardwareSPI spi1(1);

void setup() {
   /* Set up chip select as output */
   pinMode(NSS, OUTPUT);

   /* NSS is usually active LOW, so initialize it HIGH */
   digitalWrite(NSS, HIGH);

   /* Initialize SPI */
   spi1.begin(SPI_4_5MHZ, LSBFIRST, 0);
}

void loop() {
   /* Send message */
   digitalWrite(NSS, LOW);
   spi1.write(buf, sizeof buf);
   digitalWrite(NSS, HIGH);
   delay(1000);
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
    setup();

    while (true) {
        loop();
    }
    return 0;
}

