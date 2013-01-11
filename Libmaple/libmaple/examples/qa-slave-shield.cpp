// Slave mode for Quality Assurance test

#include "wirish.h"

#define INTER_TOGGLE_DELAY_NORMAL 5
#define INTER_TOGGLE_DELAY_SLOW   80

void interToggleDelay(void);

void setup() {
    pinMode(BOARD_LED_PIN, OUTPUT);
    pinMode(BOARD_BUTTON_PIN, INPUT);

    // All unused pins start out low.
    for (int i = 0; i < BOARD_NR_GPIO_PINS; i++) {
        if (boardUsesPin(i))
            continue;
        pinMode(i, OUTPUT);
        digitalWrite(i, LOW);
    }
    SerialUSB.println("OK, starting...");
}

void loop() {
    toggleLED();
    delay(100);
    toggleLED();

    for (int i = 0; i < BOARD_NR_GPIO_PINS; i++) {
        if (boardUsesPin(i))
            continue;

        // Bring just this pin high.
        digitalWrite(i, HIGH);
        // Give the master time to detect if any other pins also went high.
        interToggleDelay();
        // Bring this pin back low again; all pins should now be low.
        digitalWrite(i, LOW);
        // Give the master time to detect if any pins are still high.
        interToggleDelay();
    }
}

void interToggleDelay(void) {
    if (digitalRead(BOARD_BUTTON_PIN)) { // don't pay the debouncing time
        delay(INTER_TOGGLE_DELAY_SLOW);
    } else {
        delay(INTER_TOGGLE_DELAY_NORMAL);
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

