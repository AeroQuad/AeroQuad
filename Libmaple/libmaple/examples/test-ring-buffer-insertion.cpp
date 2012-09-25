/*
 * Simple ring_buffer test.
 *
 * Does a basic test of functionality on rb_full_count(), rb_reset(),
 * rb_push_insert(), and rb_safe_insert().
 *
 * To test:
 *
 *     - Connect a serial monitor to SerialUSB
 *     - Press any key
 *
 * This file is released into the public domain.
 */

#include "wirish.h"

#include "ring_buffer.h"

#define BUF_SIZE 64
ring_buffer ring_buf;
ring_buffer *rb;
uint8 rb_buffer[BUF_SIZE];

void test_rb_push_insert(int num_bytes_to_insert);
void test_rb_safe_insert(int num_bytes_to_insert);
void test_rb_insertion_function(int num_bytes_to_insert,
                                int (*insertion_fn)(ring_buffer*, uint8),
                                const char insertion_fn_name[]);
void print_rb_contents(void);

void setup() {
    rb = &ring_buf;
    rb_init(rb, BUF_SIZE, rb_buffer);

    while (!SerialUSB.available())
        ;

    SerialUSB.println("Beginning test.");
    SerialUSB.println();
}

void loop() {
    test_rb_push_insert(63);
    SerialUSB.println("------------------------------");
    test_rb_push_insert(64);
    SerialUSB.println("------------------------------");
    test_rb_safe_insert(63);
    SerialUSB.println("------------------------------");
    test_rb_safe_insert(64);
    SerialUSB.println("------------------------------");

    SerialUSB.println();
    SerialUSB.println("Test finished.");
    while (true)
        ;
}

void test_rb_push_insert(int num_bytes_to_insert) {
    test_rb_insertion_function(num_bytes_to_insert,
                               rb_push_insert,
                               "rb_push_insert()");
}

void test_rb_safe_insert(int num_bytes_to_insert) {
    test_rb_insertion_function(num_bytes_to_insert,
                               rb_safe_insert,
                               "rb_safe_insert()");
}

void test_rb_insertion_function(int num_bytes_to_insert,
                                int (*insertion_fn)(ring_buffer *, uint8),
                                const char insertion_fn_name[]) {
    SerialUSB.println("resetting ring buffer.");
    rb_reset(rb);
    print_rb_contents();

    SerialUSB.print(insertion_fn_name);
    SerialUSB.print("-ing ");
    SerialUSB.print(num_bytes_to_insert);
    SerialUSB.println(" bytes.");
    for (uint8 i = 1; i <= num_bytes_to_insert; i++)
        insertion_fn(rb, i);

    uint16 count = rb_full_count(rb);
    SerialUSB.print("rb_full_count(rb) = ");
    SerialUSB.println(count);

    print_rb_contents();
}

void print_rb_contents() {
    uint16 count = rb_full_count(rb);
    SerialUSB.print("ring buffer contents: ");
    for (uint16 i = 0; i < count; i++) {
        SerialUSB.print((int)rb_remove(rb));
        if (i < count - 1) SerialUSB.print(", ");
    }
    SerialUSB.println();
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
