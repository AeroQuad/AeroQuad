/*
 * Simple DAC test.
 *
 * Author: Marti Bolivar <mbolivar@leaflabs.com>
 *
 * This file is released into the public domain.
 */

#include "wirish.h"
#include "dac.h"

uint16 count = 0;

void setup() {
    pinMode(BOARD_LED_PIN, OUTPUT);
    digitalWrite(BOARD_LED_PIN, HIGH);

    Serial1.begin(9600);
    Serial1.println("**** Beginning DAC test");

    Serial1.print("Init... ");
    dac_init(DAC, DAC_CH1 | DAC_CH2);
    Serial1.println("Done.");
}

void loop() {
    toggleLED();
    delay(100);

    count += 100;
    if (count > 4095) {
        count = 0;
    }

    dac_write_channel(DAC, 1, 4095 - count);
    dac_write_channel(DAC, 2, count);
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

