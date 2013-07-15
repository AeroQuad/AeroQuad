/*
 * print-test.cpp
 *
 * Tests the various Print methods. (For USBSerial; assuming that
 * writing a single character works, this should generalize to
 * HardwareSerial).
 *
 * This file is released into the public domain.
 */

#include "wirish.h"
#undef min
#undef max

// For snprintf()
#include <stdio.h>
// The <limits.h> that comes with newlib is missing LLONG_MAX, etc.
#include <limits>

using namespace std;

#define BUF_SIZE 100
char buf[BUF_SIZE];

void test_numbers(void);
void test_base_arithmetic(void);
void test_floating_point(void);

void print_separator(void);

void setup() {
    while (!SerialUSB.available())
        continue;
    SerialUSB.read();
}

void loop() {
    SerialUSB.println("Testing Print methods.");
    print_separator();

    test_numbers();
    print_separator();

    test_base_arithmetic();
    print_separator();

    test_floating_point();
    print_separator();

    SerialUSB.println("Test finished.");
    while (true) {
        continue;
    }
}

void test_numbers(void) {
    SerialUSB.println("Numeric types:");

    SerialUSB.print("unsigned char: ");
    // prevent Print from treating it as an (extended) ASCII character:
    SerialUSB.println((uint32)numeric_limits<unsigned char>::max());

    SerialUSB.print("int: ");
    SerialUSB.print(numeric_limits<int>::min());
    SerialUSB.print(" -- ");
    SerialUSB.println(numeric_limits<int>::max());

    SerialUSB.print("unsigned int: ");
    SerialUSB.print(numeric_limits<unsigned int>::max());
    SerialUSB.println();

    SerialUSB.print("long: ");
    SerialUSB.print(numeric_limits<long>::min());
    SerialUSB.print(" -- ");
    SerialUSB.println(numeric_limits<long>::max());

    SerialUSB.print("long long: ");
    SerialUSB.print(numeric_limits<long long>::min());
    SerialUSB.print(" -- ");
    SerialUSB.println(numeric_limits<long long>::max());

    SerialUSB.print("unsigned long long: ");
    SerialUSB.println(numeric_limits<unsigned long long>::max());
}

void base_test(int base) {
    SerialUSB.print("\tuint8: ");
    SerialUSB.println(numeric_limits<uint8>::max(), base);
    SerialUSB.print("\tint: ");
    SerialUSB.print(numeric_limits<int>::max(), base);
    SerialUSB.print(", unsigned int: ");
    SerialUSB.println(numeric_limits<unsigned int>::max(), base);
    SerialUSB.print("\tlong: ");
    SerialUSB.print(numeric_limits<long>::max(), base);
    SerialUSB.print(", unsigned long: ");
    SerialUSB.println(numeric_limits<unsigned long>::max(), base);
    SerialUSB.print("\tlong long: ");
    SerialUSB.print(numeric_limits<long long>::max(), base);
    SerialUSB.print(", unsigned long long: ");
    SerialUSB.println(numeric_limits<unsigned long long>::max(), base);
}

void test_base_arithmetic(void) {
    SerialUSB.println("Base arithmetic:");

    SerialUSB.println("Binary:");
    base_test(BIN);

    SerialUSB.println("Octal:");
    base_test(OCT);

    SerialUSB.println("Decimal:");
    base_test(DEC);

    SerialUSB.println("Hexadecimal:");
    base_test(HEX);
}

void test_floating_point(void) {
    double dmax = numeric_limits<double>::max();

    SerialUSB.println("Floating point:");

    SerialUSB.print("println(-5.67): ");
    SerialUSB.print(-5.67);
    SerialUSB.print(". println(5.67, 5): ");
    SerialUSB.println(5.67, 5);
    SerialUSB.print("println((double)(LLONG_MAX - 10)):                 ");
    SerialUSB.print((double)(numeric_limits<long long>::max() - 10));
    SerialUSB.print("; from snprintf(): ");
    snprintf(buf, BUF_SIZE, "%.2f",
             (double)(numeric_limits<long long>::max() - 10));
    SerialUSB.println(buf);
    SerialUSB.print("println((double)LLONG_MAX / 2):                 ");
    SerialUSB.print((double)(numeric_limits<long long>::max()) / 2);
    SerialUSB.print("; from snprintf(): ");
    snprintf(buf, BUF_SIZE, "%.2f",
             (double)(numeric_limits<long long>::max()) / 2);
    SerialUSB.println(buf);
    SerialUSB.print("DBL_MAX:                 ");
    SerialUSB.print(dmax);
    SerialUSB.print("; from snprintf(): ");
    snprintf(buf, BUF_SIZE, "%g", dmax);
    SerialUSB.println(buf);
    SerialUSB.print("-DBL_MAX / 2:                 ");
    SerialUSB.print(-dmax / 2.0);
    SerialUSB.print("; from snprintf(): ");
    snprintf(buf, BUF_SIZE, "%g", -dmax / 2.0);
    SerialUSB.println(buf);
    SerialUSB.print("Double epsilon, round error: ");
    SerialUSB.print(numeric_limits<double>::epsilon());
    SerialUSB.print(", ");
    SerialUSB.println(numeric_limits<double>::round_error());

    SerialUSB.println();

    float fmax = numeric_limits<float>::max();

    SerialUSB.print("println(-5.67f): ");
    SerialUSB.println(-5.67f);
    SerialUSB.print("Float max: ");
    SerialUSB.println(fmax);
}

void print_separator(void) {
    SerialUSB.println();
    SerialUSB.println(" ** ");
    SerialUSB.println();
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
