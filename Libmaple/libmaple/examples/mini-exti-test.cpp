/*
 * EXTI test (Maple Mini only).
 *
 * Setup: For i from 0 to N_EXTI-1 (see below), connect exti_pins[i]
 * to src_pins[i].  Connect via SerialUSB and press a key to perform a
 * test run.  In the printed results For each EXTI within a test run,
 * the number triggered should match the number handled.
 */

#include <stdio.h>
#include <string.h>

#include "wirish.h"

// test routines
void run_exti_test(void);
void print_test_results(void);

// -- State -------------------------------------------------------------------

// Test using EXTI lines 0 -- (N_EXTI - 1).
#define N_EXTI 6

// src_pins[i] determines the line level for EXTI i.  these *must* be
// in GPIOA.  if you want to change them, make sure they don't
// conflict with exti_pins[].
//
//     src_pins[0] = D5  = PA6
//     src_pins[1] = D4  = PA7
//     src_pins[2] = D27 = PA8
//     src_pins[3] = D26 = PA9
//     src_pins[4] = D25 = PA10
//     src_pins[5] = D22 = PA13
#define SRC0 5
#define SRC0_BIT BIT(6)
#define SRC1 4
#define SRC1_BIT BIT(7)
#define SRC2 27
#define SRC2_BIT BIT(8)
#define SRC3 26
#define SRC3_BIT BIT(9)
#define SRC4 25
#define SRC4_BIT BIT(10)
#define SRC5 22
#define SRC5_BIT BIT(13)
// Setting a bit in GPIOA_SRC_MSK means that the given src pin will
// actually trigger interrupts.  Useful for experimenting.
#define GPIOA_SRC_MSK \
    (SRC0_BIT | SRC1_BIT | SRC2_BIT | SRC3_BIT | SRC4_BIT | SRC5_BIT)
const int src_pins[N_EXTI] = {
    SRC0,
    SRC1,
    SRC2,
    SRC3,
    SRC4,
    SRC5,
};
const int src_gpioa_msks[N_EXTI] = {
    SRC0_BIT,
    SRC1_BIT,
    SRC2_BIT,
    SRC3_BIT,
    SRC4_BIT,
    SRC5_BIT,
};

// exti_pins[i] <-> EXTI line i.  make sure these don't conflict with
// src_pins.
const int exti_pins[N_EXTI] = {
    D3,      // PB0
    D10,     // PA1
    D2,      // PB2
    D19,     // PB3
    D7,      // PA4
    D6,      // PA5
};

// EXTI handlers
void exti_0_handler(void);
void exti_1_handler(void);
void exti_2_handler(void);
void exti_3_handler(void);
void exti_4_handler(void);
void exti_5_handler(void);
voidFuncPtr exti_handlers[N_EXTI] = {
    exti_0_handler,
    exti_1_handler,
    exti_2_handler,
    exti_3_handler,
    exti_4_handler,
    exti_5_handler,
};

// index i = number of times we've triggered EXTI line n
static uint32 n_triggered[N_EXTI];

// index i = number of times we've handled EXTI line n
volatile static uint32 n_handled[N_EXTI];

// -- setup() -----------------------------------------------------------------

void setup(void) {
    // Set up pin modes and get line levels stable
    for (int i = 0; i < N_EXTI; i++) {
        pinMode(src_pins[i], OUTPUT);
        digitalWrite(src_pins[i], LOW);
        pinMode(exti_pins[i], INPUT);
    }

    // Delay to ensure src_pins are all LOW before proceeding
    delay(1);

    // Attach interrupts
    for (int i = 0; i < N_EXTI; i++) {
        attachInterrupt(exti_pins[i], exti_handlers[i], RISING);
    }
}

// -- loop() ------------------------------------------------------------------

void loop(void) {
    // Wait for user to send a byte before starting
    while (!SerialUSB.available())
        ;
    while (SerialUSB.available()) {
        SerialUSB.read();
    }

    // Run the test, print the results
    run_exti_test();
    print_test_results();

    // Clear out the triggered/handled state
    for (int i = 0; i < N_EXTI; i++) {
        n_triggered[i] = 0;
        n_handled[i] = 0;
    }

    SerialUSB.println();
    SerialUSB.println();
    SerialUSB.println();
}

// -- Test routines -----------------------------------------------------------

#define N_RUNS 100
void run_exti_test(void) {
    for (int run = 0; run < N_RUNS; run++) {
        // Trigger EXTIs simultaneously
        GPIOA_BASE->BSRR = GPIOA_SRC_MSK;

        // Reset line levels
        GPIOA_BASE->BSRR = GPIOA_SRC_MSK << 16;

        // Update number of times triggered
        for (int i = 0; i < N_EXTI; i++) {
            if (GPIOA_SRC_MSK & src_gpioa_msks[i]) {
                n_triggered[i]++;
            }
        }
    }
}

// string handling boilerplate
void resetl(void);
void appendl(const char str[]);
void appendl(uint32 n);
void printl(void);

void print_test_results(void) {
    SerialUSB.println("Results:");

    resetl();
    appendl("EXTI");
    appendl("# Triggered");
    appendl("# Handled");
    printl();
    resetl();

    for (uint32 i = 0; i < N_EXTI; i++) {
        appendl(i);
        appendl(n_triggered[i]);
        appendl(n_handled[i]);
        printl();
        resetl();
    }
}

// -- EXTI handlers -----------------------------------------------------------

void exti_0_handler(void) {
    n_handled[0]++;
}

void exti_1_handler(void) {
    n_handled[1]++;
}

void exti_2_handler(void) {
    n_handled[2]++;
}

void exti_3_handler(void) {
    n_handled[3]++;
}

void exti_4_handler(void) {
    n_handled[4]++;
}

void exti_5_handler(void) {
    n_handled[5]++;
}

// -- String handling ---------------------------------------------------------

#define C_SIZ 20
#define LIN_SIZ 80
char l[LIN_SIZ + 1];
char tmp[C_SIZ + 1];

void resetl(void) {
    l[0] = '\0';
}

void appendl(const char str[]) {
    snprintf(tmp, C_SIZ, "%-*s", C_SIZ, str);
    strncat(l, tmp, LIN_SIZ - strlen(tmp));
}

void appendl(uint32 n) {
    snprintf(tmp, C_SIZ, "%-*u", C_SIZ, n);
    strncat(l, tmp, LIN_SIZ - strlen(tmp));
}

void printl(void) {
    SerialUSB.println(l);
}

// -- init()/main() -----------------------------------------------------------

__attribute__((constructor)) void premain() { init(); }

int main(void) {
    setup();

    while (true)
        loop();

    return 0;
}
