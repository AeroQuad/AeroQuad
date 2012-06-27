/*

  A low-level stress test of SRAM functionality.  Uses slow-ish timing
  by default (DATAST = ADDSET = 0xF).

  Copyright 2011 LeafLabs, LLC.

  This code is released into the public domain.

 */

#include <stdio.h>
#include <stddef.h>

#include "wirish.h"
#include "rcc.h"
#include "fsmc.h"

// -- SRAM config -------------------------------------------------------------

// Timing configuration
#define DATAST   0xF
#define ADDSET   0xF

// Number of SRAM chips to test
#define N             1

// How much of each to test
#define MEM_SIZE 0x3FFF

// Their start addresses in FSMC bank 1
__io uint16 *const starts[N] = {
    // (__io uint16 *const)FSMC_NOR_PSRAM_REGION1,
    // (__io uint16 *const)FSMC_NOR_PSRAM_REGION2,
    (__io uint16 *const)FSMC_NOR_PSRAM_REGION3,
    // (__io uint16 *const)FSMC_NOR_PSRAM_REGION4,
};

// Corresponding FSMC configuration registers
__io uint32 *const bcrs[N] = {
    // &FSMC_NOR_PSRAM1_BASE->BCR,
    // &FSMC_NOR_PSRAM2_BASE->BCR,
    &FSMC_NOR_PSRAM3_BASE->BCR,
    // &FSMC_NOR_PSRAM4_BASE->BCR,
};

// Corresponding FSMC timing registers
__io uint32 *const btrs[N] = {
    // &FSMC_NOR_PSRAM1_BASE->BTR,
    // &FSMC_NOR_PSRAM2_BASE->BTR,
    &FSMC_NOR_PSRAM3_BASE->BTR,
    // &FSMC_NOR_PSRAM4_BASE->BTR,
};

// -- Pseudorandom number generation  -----------------------------------------

const uint32 seed = 0xDEADBEEF;

uint32 num_rand_calls = 0;

uint32 rand(long n) {
    num_rand_calls++;
    return random(n);
}

// -- Printing ----------------------------------------------------------------

// For snprintf()
char snprintf_buf[200];

#define ERR(fmt, ...) do {                                            \
        snprintf(snprintf_buf, sizeof snprintf_buf,                   \
                 "ERROR: " fmt " (seed %d, ncalls %d, line %d)",      \
                 __VA_ARGS__, seed, num_rand_calls, __LINE__);        \
        SerialUSB.println(snprintf_buf);                              \
    } while (0)

// Set to 1 for more output
#define VERBOSE 0

// -- setup()/loop() ----------------------------------------------------------

void setup() {
    fsmc_sram_init_gpios();
    rcc_clk_enable(RCC_FSMC);

    for (int i = 0; i < N; i++) {
        *bcrs[i] = (FSMC_BCR_WREN |
                    FSMC_BCR_MTYP_SRAM |
                    FSMC_BCR_MWID_16BITS |
                    FSMC_BCR_MBKEN);
        *btrs[i] = (DATAST << 8) | ADDSET;
    }

    randomSeed(seed);

    SerialUSB.read();
    SerialUSB.println("Starting test");
}

// stress_test() and simple_roundtrip() are the available test routines
bool stress_test(void);
bool simple_roundtrip(void);

void loop() {
    uint32 last;

    last = millis();
    while (true) {
        if (!stress_test()) {
            SerialUSB.println("Halting due to error.");
            throb();
        } else {
            uint32 now = millis();
            if (now - last > 500) {
                snprintf(snprintf_buf, sizeof snprintf_buf,
                         "everything ok so far, timestamp %d ms", now);
                SerialUSB.println(snprintf_buf);
                last = now;
            }
        }
    }
}

// -- Test routines -----------------------------------------------------------

bool random_trips();
bool sequential_trips();

bool stress_test(void) {
    static int i = 0;
    i = !i;

    switch (i) {
    case 0:
        return random_trips();
    default:
        return sequential_trips();
    }
}

bool simple_roundtrip(void) {
    uint16 wval = 0xAB;

    for (int i = 0; i < N; i++) {
        __io uint16 *addr = starts[i] + 4;
        snprintf(snprintf_buf, sizeof snprintf_buf, "round-trip 0x%x at %p",
                 wval, addr);
        SerialUSB.println(snprintf_buf);

        *addr = wval;
        uint16 rval = *addr;

        if (rval != wval) {
            ERR("wrote 0x%x, read 0x%x, timestamp %d", wval, rval, millis());
            return false;
        } else {
            snprintf(snprintf_buf, sizeof snprintf_buf, "got back 0x%x", rval);
            SerialUSB.println(snprintf_buf);
        }
    }

    return true;
}

bool random_trips(void) {
#if VERBOSE
    SerialUSB.println("[random]");
#endif
    for (int n = 0; n < N; n++) {
        __io uint16 *const start = starts[n];

        for (int i = 0; i < 1000; i++) {
            uint32 offset = rand(MEM_SIZE);
            uint32 wval = rand(0xFFFF);

            *(start + offset) = wval;
            uint32 rval = *(start + offset);

            if (rval != wval) {
                ERR("wrote 0x%x to 0x%x, read 0x%x", wval, offset, rval);
                return false;
            }
        }
    }
    return true;
}

bool sequential_trips(void) {
    static const uint32 seq_length = 300;
#if VERBOSE
    SerialUSB.println("[seq]");
#endif
    for (int n = 0; n < N; n++) {
        __io uint16 *const start = starts[n];

        for (int i = 0; i < 100; i++) {
            uint32 start_offset = rand(MEM_SIZE - seq_length);

            for (uint32 w = 0; w < seq_length; w++) {
                uint32 offset = start_offset + w;

                *(start + offset) = w;
                uint32 r = *(start + offset);

                if (w != r) {
                    ERR("wrote 0x%x to 0x%x, read 0x%x", w, offset, r);
                    return false;
                }
            }
        }
    }
    return true;
}

// ----------------------------------------------------------------------------

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

