/*
 * Polling SPI loopback test.
 *
 * Bob is nowhere to be found, so Alice decides to talk to herself.
 *
 * Instructions: Connect SPI2 (Alice) to herself (i.e., MISO to MOSI).
 * Connect to Alice via SerialUSB.  Press any key to start.
 *
 * Alice will talk to herself for a little while.  The sketch will
 * report if Alice can't hear anything she says.  She'll then start
 * talking forever at various frequencies, bit orders, and modes.  Use
 * an oscilloscope to make sure she's not trying to lie about any of
 * those things.
 *
 * This file is released into the public domain.
 *
 * Author: Marti Bolivar <mbolivar@leaflabs.com>
 */

#include "wirish.h"

HardwareSPI alice(2);

#define NFREQS 8
const SPIFrequency spi_freqs[] = {
    SPI_140_625KHZ,
    SPI_281_250KHZ,
    SPI_562_500KHZ,
    SPI_1_125MHZ,
    SPI_2_25MHZ,
    SPI_4_5MHZ,
    SPI_9MHZ,
    SPI_18MHZ,
};

#define TEST_BUF_SIZE 10
uint8 test_buf[TEST_BUF_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

void bad_assert(const char* file, int line, const char* exp) {
    SerialUSB.println();
    SerialUSB.print("ERROR: FAILED ASSERT(");
    SerialUSB.print(exp);
    SerialUSB.print("): ");
    SerialUSB.print(file);
    SerialUSB.print(": ");
    SerialUSB.println(line);
    throb();
}

#undef ASSERT
#define ASSERT(exp)                                    \
    if (exp) {                                         \
    } else {                                           \
        bad_assert(__FILE__, __LINE__, #exp);          \
    }

void haveConversation(uint32 bitOrder);
void soliloquies(uint32 bitOrder);

void setup() {
    pinMode(BOARD_LED_PIN, OUTPUT);
    SerialUSB.read();
}

void loop() {
    SerialUSB.println("** Having a conversation, MSB first");
    haveConversation(MSBFIRST);

    SerialUSB.println("** Having a conversation, LSB first");
    haveConversation(LSBFIRST);

    SerialUSB.println();
    SerialUSB.println("*** All done!  It looks like everything worked.");
    SerialUSB.println();

    SerialUSB.println("** Alice will now wax eloquent in various styles. "
                      "Press any key for the next configuration.");
    soliloquies(MSBFIRST);
    soliloquies(LSBFIRST);

    while (true)
        ;
}

void printFrequencyString(SPIFrequency frequency);
void chat(SPIFrequency frequency, uint32 bitOrder, uint32 mode);

void haveConversation(uint32 bitOrder) {
    for (int f = 0; f < NFREQS; f++) {
        for (int mode = 0; mode < 4; mode++) {
            chat(spi_freqs[f], bitOrder, mode);
            delay(10);
        }
    }
}

void chat(SPIFrequency frequency, uint32 bitOrder, uint32 mode) {
    SerialUSB.print("Having a chat.\tFrequency: ");
    printFrequencyString(frequency);
    SerialUSB.print(",\tbitOrder: ");
    SerialUSB.print(bitOrder == MSBFIRST ? "MSB" : "LSB");
    SerialUSB.print(",\tmode: ");
    SerialUSB.print(mode);
    SerialUSB.print(".");

    SerialUSB.print(" [1] ");
    alice.begin(frequency, bitOrder, mode);

    SerialUSB.print(" [2] ");
    uint32 txed = 0;
    while (txed < TEST_BUF_SIZE) {
        ASSERT(alice.transfer(test_buf[txed]) == test_buf[txed]);
        txed++;
    }

    SerialUSB.print(" [3] ");
    alice.end();

    SerialUSB.println(" ok.");
}

void soliloquy(SPIFrequency freq, uint32 bitOrder, uint32 mode);

void soliloquies(uint32 bitOrder) {
    for (int f = 0; f < NFREQS; f++) {
        for (int mode = 0; mode < 4; mode++) {
            soliloquy(spi_freqs[f], bitOrder, mode);
        }
    }
}

void soliloquy(SPIFrequency frequency, uint32 bitOrder, uint32 mode) {
    const uint8 repeat = 0xAE;
    SerialUSB.print("Alice is giving a soliloquy (repeating 0x");
    SerialUSB.print(repeat, HEX);
    SerialUSB.print("). Frequency: ");
    printFrequencyString(frequency);
    SerialUSB.print(", bitOrder: ");
    SerialUSB.print(bitOrder == MSBFIRST ? "big-endian" : "little-endian");
    SerialUSB.print(", SPI mode: ");
    SerialUSB.println(mode);

    alice.begin(frequency, bitOrder, mode);
    while (!SerialUSB.available()) {
        alice.write(repeat);
        delayMicroseconds(200);
    }
    SerialUSB.read();
}

void printFrequencyString(SPIFrequency frequency) {
    switch (frequency) {
    case SPI_18MHZ:
        SerialUSB.print("18 MHz");
        break;
    case SPI_9MHZ:
        SerialUSB.print("9 MHz");
        break;
    case SPI_4_5MHZ:
        SerialUSB.print("4.5 MHz");
        break;
    case SPI_2_25MHZ:
        SerialUSB.print("2.25 MHZ");
        break;
    case SPI_1_125MHZ:
        SerialUSB.print("1.125 MHz");
        break;
    case SPI_562_500KHZ:
        SerialUSB.print("562.500 KHz");
        break;
    case SPI_281_250KHZ:
        SerialUSB.print("281.250 KHz");
        break;
    case SPI_140_625KHZ:
        SerialUSB.print("140.625 KHz");
        break;
    }
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
