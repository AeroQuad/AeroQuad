// Tests the SysTick enable/disable functions

#include "wirish.h"
#include "systick.h"

void setup() {
    pinMode(BOARD_LED_PIN, OUTPUT);
    pinMode(BOARD_BUTTON_PIN, INPUT);
}

bool disable = true;
long time = 0;

void loop() {
    volatile int i = 0;
    toggleLED();

    // An artificial delay
    for(i = 0; i < 150000; i++)
        ;

    if (isButtonPressed()) {
        if (disable) {
            systick_disable();
            SerialUSB.println("Disabling SysTick");
        } else {
            SerialUSB.println("Re-enabling SysTick");
            systick_enable();
        }
        disable = !disable;
    }

    SerialUSB.println(millis());
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated object that need libmaple may fail.
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
