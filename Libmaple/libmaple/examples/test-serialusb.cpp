// Tests SerialUSB functionality.

#include "wirish.h"
#include "usb.h"

#define QUICKPRINT  0
#define BIGSTUFF    1
#define NUMBERS     2
#define SIMPLE      3
#define ONOFF       4

uint32 state = 0;

void setup() {
    /* Set up the LED to blink  */
    pinMode(BOARD_LED_PIN, OUTPUT);

    /* Set up Serial2 for use as a debug channel */
    Serial2.begin(9600);
    Serial2.println("This is the debug channel. Press any key.");
    while (!Serial2.available())
        ;
    Serial2.read();
}

uint8 c1 = '-';

void loop() {
    toggleLED();
    delay(1000);

    if (Serial2.available()) {
        Serial2.read();
        state++;
    }

    switch (state) {
        case QUICKPRINT:
            for (int i = 0; i < 30; i++) {
                usbSendBytes(&c1, 1);
                SerialUSB.print('.');
                SerialUSB.print('|');
            }
            Serial2.println(SerialUSB.pending(), DEC);
            SerialUSB.println();
            break;
        case BIGSTUFF:
            SerialUSB.println("0123456789012345678901234567890123456789"
                              "0123456789012345678901234567890123456789"
                              "012345678901234567890");
            SerialUSB.println((int64)123456789, DEC);
            SerialUSB.println(3.1415926535);
            Serial2.println(SerialUSB.pending(), DEC);
            break;
        case NUMBERS:
            SerialUSB.println("Numbers! -----------------------------");
            Serial2.println("Numbers! -----------------------------");
            SerialUSB.println('1');
            Serial2.println('1');
            SerialUSB.println(1, DEC);
            Serial2.println(1, DEC);
            SerialUSB.println(-1, DEC);
            Serial2.println(-1, DEC);
            SerialUSB.println(3.14159265);
            Serial2.println(3.14159265);
            SerialUSB.println(123456789, DEC);
            Serial2.println(123456789, DEC);
            SerialUSB.println(-123456789, DEC);
            Serial2.println(-123456789, DEC);
            SerialUSB.println(65535, HEX);
            Serial2.println(65535, HEX);
            break;
        case SIMPLE:
            Serial2.println("Trying write('a')");
            SerialUSB.write('a');
            Serial2.println("Trying write(\"b\")");
            SerialUSB.write("b");
            Serial2.println("Trying print('c')");
            SerialUSB.print('c');
            Serial2.println("Trying print(\"d\")");
            SerialUSB.print("d");
            Serial2.println("Trying print(\"efg\")");
            SerialUSB.print("efg");
            Serial2.println("Trying println(\"hij\\n\\r\")");
            SerialUSB.print("hij\n\r");
            SerialUSB.write(' ');
            SerialUSB.println();
            Serial2.println("Trying println(123456789, DEC)");
            SerialUSB.println(123456789, DEC);
            Serial2.println("Trying println(3.141592)");
            SerialUSB.println(3.141592);
            Serial2.println("Trying println(\"DONE\")");
            SerialUSB.println("DONE");
            break;
        case ONOFF:
            Serial2.println("Shutting down...");
            SerialUSB.println("Shutting down...");
            SerialUSB.end();
            Serial2.println("Waiting 4 seconds...");
            delay(4000);
            Serial2.println("Starting up...");
            SerialUSB.begin();
            SerialUSB.println("Hello World!");
            Serial2.println("Waiting 4 seconds...");
            delay(4000);
            state++;
            break;
        default:
            state = 0;
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
