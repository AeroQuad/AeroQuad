/*
  VGA Oscilloscope demo.

  Connect a microphone or something like it to ANALOG_PIN (0V -- 3.3V
  only; 0.2V -- 3.1V will probably look nicer); an attached VGA
  monitor will display the signal roughly in real-time.

  The thick blue line corresponds roughly to 0V.

  This is a fairy crude hack, but it's fun to watch/toy around with.

  SerialUSB and SysTick are disabled to get rid of the most frequently
  occurring interrupts (which mess with timing). This means that you
  have to use perpetual bootloader mode or the reset button to flash
  new programs.

  How to wire this to a VGA port:
  D6 via ~200ohms to VGA Red     (1)
  D7 via ~200ohms to VGA Green   (2)
  D8 via ~200ohms to VGA Blue    (3)
  D11 to VGA VSync               (14)  (swapped?)
  D12 to VGA HSync               (13)  (swapped?)
  GND to VGA Ground              (5)
  GND to VGA Sync Ground         (10)

  See also:
  - http://pinouts.ru/Video/VGA15_pinout.shtml
  - http://www.epanorama.net/documents/pc/vga_timing.html

  This code is released into the public domain.

  Authors:

  Bryan Newbold <bnewbold@leaflabs.com>
  Marti Bolivar <mbolivar@leaflabs.com>
 */

#include "wirish.h"
#include "systick.h"

// FIXME: generalize for Native and Mini

#define ANALOG_PIN 18

// Pinouts -- you also must change the GPIO macros below if you change
// these
#define VGA_R 6     // STM32: A8
#define VGA_G 7     // STM32: A9
#define VGA_B 8     // STM32: A10
#define VGA_V 11    // STM32: A6
#define VGA_H 12    // STM32: A7

// These low level (and STM32 specific) macros make GPIO writes much
// faster
#define ABSRR ((volatile uint32*)0x40010810)
#define ABRR  ((volatile uint32*)0x40010814)

#define RBIT 8                  // (see pinouts)
#define GBIT 9
#define BBIT 10

#define VGA_R_HIGH *ABSRR = BIT(RBIT)
#define VGA_R_LOW  *ABRR  = BIT(RBIT)
#define VGA_G_HIGH *ABSRR = BIT(GBIT)
#define VGA_G_LOW  *ABRR  = BIT(GBIT)
#define VGA_B_HIGH *ABSRR = BIT(BBIT)
#define VGA_B_LOW  *ABRR  = BIT(BBIT)

#define COLOR_WHITE (BIT(RBIT) | BIT(GBIT) | BIT(BBIT))
#define COLOR_BLACK 0
#define COLOR_RED   BIT(RBIT)
#define COLOR_GREEN BIT(GBIT)
#define COLOR_BLUE  BIT(BBIT)

#define BORDER_COLOR COLOR_BLUE

// set has priority, so clear every bit and set some given bits:
#define VGA_COLOR(c) (*ABSRR = c |                                      \
                      BIT(RBIT + 16) | BIT(GBIT + 16) | BIT(BBIT + 16))

#define VGA_V_HIGH *ABSRR = BIT(6)
#define VGA_V_LOW  *ABRR  = BIT(6)
#define VGA_H_HIGH *ABSRR = BIT(7)
#define VGA_H_LOW  *ABRR  = BIT(7)

void isr_porch(void);
void isr_start(void);
void isr_stop(void);
void isr_update(void);

void setup() {
    pinMode(BOARD_LED_PIN, OUTPUT);
    pinMode(ANALOG_PIN, INPUT_ANALOG);
    digitalWrite(BOARD_LED_PIN, 1);
    pinMode(VGA_R, OUTPUT);
    pinMode(VGA_G, OUTPUT);
    pinMode(VGA_B, OUTPUT);
    pinMode(VGA_V, OUTPUT);
    pinMode(VGA_H, OUTPUT);

    // Send a message out USART2
    Serial2.begin(9600);
    Serial2.println("Time to kill the radio star...");

    // This gets rid of the majority of the interrupt artifacts;
    // there's still a glitch for low values of y, but let's not worry
    // about that. (Probably due to the hackish way vsync is done).
    SerialUSB.end();
    systick_disable();

    digitalWrite(VGA_R, 0);
    digitalWrite(VGA_G, 0);
    digitalWrite(VGA_B, 0);
    digitalWrite(VGA_H, 1);
    digitalWrite(VGA_V, 1);

    timer_pause(TIMER4);
    timer_set_prescaler(TIMER4, 0);
    timer_set_mode(TIMER4, 1, TIMER_OUTPUT_COMPARE);
    timer_set_mode(TIMER4, 2, TIMER_OUTPUT_COMPARE);
    timer_set_mode(TIMER4, 3, TIMER_OUTPUT_COMPARE);
    timer_set_mode(TIMER4, 4, TIMER_OUTPUT_COMPARE);
    timer_set_reload(TIMER4, 2287);
    timer_set_compare(TIMER4, 1, 200);
    timer_set_compare(TIMER4, 2, 250);
    timer_set_compare(TIMER4, 3, 2170);  // 2219 max...
    timer_set_compare(TIMER4, 4, 1);
    timer_attach_interrupt(TIMER4, 1, isr_porch);
    timer_attach_interrupt(TIMER4, 2, isr_start);
    timer_attach_interrupt(TIMER4, 3, isr_stop);
    timer_attach_interrupt(TIMER4, 4, isr_update);

    timer_set_count(TIMER4, 0);
    timer_resume(TIMER4);
}

uint16 y = 0;
uint16 val = 0;
bool v_active = true;
const uint16 x_max = 60;        // empirically (and sloppily) determined

void isr_porch(void) {
    VGA_H_HIGH;
    y++;
    val = map(analogRead(ANALOG_PIN), 0, 4095, 0, x_max);
    if (y >= 523) {
        y = 1;
        v_active = true;
        return;
    }
    if (y >= 492) {
        VGA_V_HIGH;
        return;
    }
    if (y >= 490) {
        VGA_V_LOW;
        return;
    }
    if (y >= 479) {
        v_active = false;
        return;
    }

}

void isr_start(void) {
    if (!v_active) {
        return;
    }
    VGA_COLOR(BORDER_COLOR);
    for (int x = 0; x < val; x++) {
        VGA_COLOR(COLOR_BLACK);
    }
    VGA_COLOR(COLOR_WHITE);
    VGA_COLOR(COLOR_BLACK);
}

void isr_stop(void) {
    if (!v_active) {
        return;
    }
    VGA_COLOR(COLOR_BLACK);
}

void isr_update(void) {
    VGA_H_LOW;
}

void loop() {
    toggleLED();
    delay(100);
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
