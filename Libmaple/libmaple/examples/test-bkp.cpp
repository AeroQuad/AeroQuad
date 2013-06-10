#include <stdio.h>              // for snprintf()

#include "wirish.h"
#include "bkp.h"
#include "iwdg.h"

void print_bkp_contents();
void write_to_bkp(uint16 val);

#define comm Serial2

void setup() {
    pinMode(BOARD_BUTTON_PIN, INPUT);

    comm.begin(9600);
    comm.println("*** Beginning BKP test");

    comm.println("Init...");
    bkp_init();
    comm.println("Done.");

    print_bkp_contents();
    write_to_bkp(10);
    print_bkp_contents();

    comm.println("Enabling backup writes.");
    bkp_enable_writes();
    write_to_bkp(20);
    print_bkp_contents();

    comm.println("Disabling backup writes.");
    bkp_disable_writes();
    write_to_bkp(30);
    print_bkp_contents();

    comm.println("Done testing backup registers; press button to enable "
                    "independent watchdog (in order to cause a reset).");
    waitForButtonPress(0);
    iwdg_init(IWDG_PRE_4, 1);
    comm.println();
}

void loop() {
}

void print_bkp_contents() {
    comm.println("Backup data register contents:");
    char buf[100];
    for (int i = 1; i <= BKP_NR_DATA_REGS; i++) {
        snprintf(buf, sizeof buf, "DR%d: %d ", i, bkp_read(i));
        comm.print(buf);
        if (i % 5 == 0) comm.println();
    }
    comm.println();
}

void write_to_bkp(uint16 val) {
    comm.print("Attempting to write ");
    comm.print(val);
    comm.println(" to backup registers...");
    for (int i = 1; i <= BKP_NR_DATA_REGS; i++) {
        bkp_write(i, val);
    }
    comm.println("Done.");
}

__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
    init();
    setup();

    while (1) {
        loop();
    }
    return 0;
}

