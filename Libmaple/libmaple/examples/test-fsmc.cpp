#include <stddef.h>             // for ptrdiff_t

#include "wirish.h"
#include "fsmc.h"

#ifndef BOARD_maple_native
#error "Sorry, this example only works on Maple Native."
#endif

// Start of FSMC SRAM bank 1
static uint16 *const sram_start = (uint16*)0x60000000;
// End of Maple Native SRAM chip address space (512K 16-bit words)
static uint16 *const sram_end = (uint16*)0x60100000;

void test_single_write(void);
void test_all_addresses(void);

void setup() {
    pinMode(BOARD_LED_PIN, OUTPUT);
    digitalWrite(BOARD_LED_PIN, HIGH);

    Serial1.begin(115200);
    Serial1.println("*** Beginning RAM chip test");

    test_single_write();
    test_all_addresses();

    Serial1.println("Tests pass, finished.");
    Serial1.println("***\n");
}

void loop() {
}

void test_single_write() {
    uint16 *ptr = sram_start;
    uint16 tmp;

    Serial1.print("Writing 0x1234... ");
    *ptr = 0x1234;
    Serial1.println("Done.");

    Serial1.print("Reading... ");
    tmp = *ptr;
    Serial1.print("Done: 0x");
    Serial1.println(tmp, HEX);

    if (tmp != 0x1234) {
        Serial1.println("Mismatch; abort.");
        ASSERT(0);
    }
}

void test_all_addresses() {
    uint32 start, end;
    uint16 count = 0;
    uint16 *ptr;

    Serial1.println("Now writing all memory addresses (unrolled loop)");
    // Turn off the USB interrupt, as it interferes most with timing
    // (don't turn off SysTick, or we won't get micros()).
    SerialUSB.end();
    start = micros();
    for (ptr = sram_start; ptr < sram_end;) {
        *ptr++ = count++;
        *ptr++ = count++;
        *ptr++ = count++;
        *ptr++ = count++;
        *ptr++ = count++;
        *ptr++ = count++;
        *ptr++ = count++;
        *ptr++ = count++;
        *ptr++ = count++;
        *ptr++ = count++;
        *ptr++ = count++;
        *ptr++ = count++;
        *ptr++ = count++;
        *ptr++ = count++;
        *ptr++ = count++;
        *ptr++ = count++;
    }
    end = micros();
    SerialUSB.begin();
    Serial1.print("Done. Elapsed time (us): ");
    Serial1.println(end - start);

    Serial1.println("Validating writes.");
    for (ptr = sram_start, count = 0; ptr < sram_end; ptr++, count++) {
        uint16 value = *ptr;
        if (value != count) {
            Serial1.print("mismatch: 0x");
            Serial1.print((uint32)ptr);
            Serial1.print(" = 0x");
            Serial1.print(value, HEX);
            Serial1.print(", should be 0x");
            Serial1.print(count, HEX);
            Serial1.println(".");
            ASSERT(0);
        }
    }
    Serial1.println("Done; all writes seem valid.");

    ptrdiff_t nwrites = sram_end - sram_start;
    double us_per_write = double(end-start) / double(nwrites);
    Serial1.print("Number of writes = ");
    Serial1.print(nwrites);
    Serial1.print("; avg. time per write = ");
    Serial1.print(us_per_write);
    Serial1.print(" us (");
    Serial1.print(1 / us_per_write);
    Serial1.println(" MHz)");
}

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
