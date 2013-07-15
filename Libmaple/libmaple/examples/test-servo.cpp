/*
 * Basic Servo library test program.
 *
 * Setup:
 *
 * - Connect a potentiometer to POT_PIN (default pin 15)
 * - Connect an oscilloscope to SERVO_PIN1 (default pin 5) and
 *   SERVO_PIN2 (default pin 6).
 * - Connect a serial monitor to SerialUSB
 *
 * The potentiometer controls the target angle for each of two Servo
 * objects, one with angles in [-90, 90], and another in [0, 180].
 * Servo pulse width range is [1000, 2000].
 *
 * Serial2 will tell you what inputs it's giving to each servo object,
 * and some information it gets back.  Pressing the button
 * detaches/reattaches the Servo objects.
 *
 * Tests you should perform:
 *
 * - Check calculated pulse widths for each servo's target angle
 * - Check that calculated pulse widths match actual pulse widths
 * - Check that the period of the pulse train is roughly 20 ms
 * - Check that the pulses stop when detached, and resume when reattached
 * - Check that Servo::write() and Servo::read() round-trip properly
 *
 * This file is released into the public domain.
 */

#include <stdio.h>

#include "wirish.h"

#include "libraries/Servo/Servo.h"

#define POT_PIN 15

#define MIN_PW 1000
#define MAX_PW 2000

#define SERVO_PIN1 5
#define MIN_ANGLE1 0
#define MAX_ANGLE1 180

#define SERVO_PIN2 6
#define MIN_ANGLE2 (-90)
#define MAX_ANGLE2 90

Servo servo1;
Servo servo2;

#define BUF_SIZE 100
char buf[BUF_SIZE];

#define print_buf(fmt, ...) do {                  \
    snprintf(buf, BUF_SIZE, fmt, __VA_ARGS__);    \
    Serial2.println(buf); } while (0)

int averageAnalogReads(int);
void attach();
void detach();

void setup() {
    pinMode(POT_PIN, INPUT_ANALOG);
    pinMode(BOARD_BUTTON_PIN, INPUT);
    pinMode(BOARD_LED_PIN, OUTPUT);

    Serial2.begin(9600);

    servo1.attach(SERVO_PIN1, MIN_PW, MAX_PW, MIN_ANGLE1, MAX_ANGLE1);
    servo2.attach(SERVO_PIN2, MIN_PW, MAX_PW, MIN_ANGLE2, MAX_ANGLE2);

    ASSERT(servo1.attachedPin() == SERVO_PIN1);
    ASSERT(servo2.attachedPin() == SERVO_PIN2);
}

void loop() {
    delay(250);
    toggleLED();

    if (isButtonPressed()) {
        if (servo1.attached()) detach();
        else                   attach();
    }

    if (!servo1.attached()) return;

    int32 average = averageAnalogReads(250);
    int16 angle1 = (int16)map(average, 0, 4095, MIN_ANGLE1, MAX_ANGLE1);
    int16 angle2 = (int16)map(average, 0, 4095, MIN_ANGLE2, MAX_ANGLE2);

    print_buf("pot reading = %d, angle 1 = %d, angle 2 = %d.",
              average, angle1, angle2);

    servo1.write(angle1);
    servo2.write(angle2);

    int16 read1 = servo1.read();
    int16 read2 = servo2.read();

    print_buf("write/read angle 1: %d/%d, angle 2: %d/%d",
              angle1, read1, angle2, read2);

    ASSERT(abs(angle1 - read1) <= 1);
    ASSERT(abs(angle2 - read2) <= 1);

    print_buf("pulse width 1: %d, pulse width 2: %d",
              servo1.readMicroseconds(), servo2.readMicroseconds());

     Serial2.println("\n--------------------------\n");
}

int32 averageAnalogReads(int n) {
    uint64 total = 0;

    for (int i = 0; i < n; i++) {
        total += analogRead(POT_PIN);
    }

    return (int32)(total / n);
}

void attach() {
    Serial2.println("attaching");
    servo1.attach(SERVO_PIN1);
    servo2.attach(SERVO_PIN2);
    ASSERT(servo1.attachedPin() == SERVO_PIN1);
    ASSERT(servo2.attachedPin() == SERVO_PIN2);
}

void detach() {
    Serial2.println("detaching");
    servo1.detach();
    servo2.detach();
    ASSERT(!servo1.attached());
    ASSERT(!servo2.attached());
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
