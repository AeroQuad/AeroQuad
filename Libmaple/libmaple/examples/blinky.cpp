// Blinks the built-in LED

#include "wirish.h"

void setup() {
    pinMode(BOARD_LED_PIN, OUTPUT);
}

int toggle = 1;

void loop() {
    // You could just use toggleLED() instead, but this illustrates
    // the use of digitalWrite():
    digitalWrite(BOARD_LED_PIN, toggle);
    toggle ^= 1;
    delay(100);
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
