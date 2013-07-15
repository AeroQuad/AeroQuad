// Test sketch for figuring out DTR/RTS behavior on different platforms.

#include "wirish.h"
#include "usb.h"

void setup() {
    /* Set up the LED to blink  */
    pinMode(BOARD_LED_PIN, OUTPUT);

    /* Send a message out USART2  */
    Serial2.begin(9600);
    Serial2.println("Debugging DTR/RTS...");

}

void loop() {
    toggleLED();
    delay(100);

    Serial2.print("DTR: ");
    Serial2.print(usbGetDTR(), DEC);
    Serial2.print("\tRTS: ");
    Serial2.println(usbGetRTS(), DEC);
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
    setup();

    while (1) {
        loop();
    }
    return 0;
}

