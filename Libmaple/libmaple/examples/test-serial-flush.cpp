/*
 * Tests the "flush" Serial function.
 */

#include "wirish.h"

void setup() {
    Serial1.begin(9600);
    Serial1.println("Hello world!");
}

void loop() {
    Serial1.println("Waiting for multiple input...");
    while (Serial1.available() < 5)
        ;
    Serial1.println(Serial1.read());
    Serial1.println(Serial1.read());
    Serial1.flush();

    if (Serial1.available()) {
        Serial1.println("FAIL! Still had junk in the buffer...");
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
