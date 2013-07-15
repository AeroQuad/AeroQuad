/*
 VGA Output

 Outputs a red and white leaf to VGA.  It should run most VGA monitors
 at 640x480, though it does not follow the timing spec very
 carefully. Real twisted or shielded wires, proper grounding, and not
 doing this on a breadboard are recommended (but it seems to work ok
 without).

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

 Created 20 July 2010
 By Bryan Newbold for LeafLabs
 This code is released with no strings attached.
 */

// FIXME: generalize for Native and Mini

#include "wirish.h"

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

#define ON_COLOR   BIT(RBIT)
#define OFF_COLOR  (BIT(RBIT) | BIT(GBIT) | BIT(BBIT))

// set has priority, so clear every bit and set some given bits:
#define VGA_COLOR(c) (*ABSRR = c |                                      \
                      BIT(RBIT+16) | BIT(GBIT+16) | BIT(BBIT+16))

#define VGA_V_HIGH *ABSRR = BIT(6)
#define VGA_V_LOW  *ABRR  = BIT(6)
#define VGA_H_HIGH *ABSRR = BIT(7)
#define VGA_H_LOW  *ABRR  = BIT(7)

void isr_porch(void);
void isr_start(void);
void isr_stop(void);
void isr_update(void);

uint16 x = 0;       // X coordinate
uint16 y = 0;       // Y coordinate
uint16 logo_y = 0;  // Y coordinate, mapped into valid logo index (for speed)
bool v_active = true; // Are we in the image?

const uint8 x_max = 16;
const uint8 y_max = 18;
uint32 logo[y_max][x_max] = {
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,},
    {0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,},
    {0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,},
    {0,0,0,0,1,0,0,1,0,0,1,0,0,0,0,0,},
    {0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,},
    {0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,},
    {0,0,1,0,0,1,0,1,0,1,0,0,1,0,0,0,},
    {0,1,0,0,0,0,1,1,1,0,0,0,0,1,0,0,},
    {0,1,0,1,0,0,0,1,0,0,0,1,0,1,0,0,},
    {1,0,0,0,1,0,0,1,0,0,1,0,0,0,1,0,},
    {1,0,0,0,0,1,0,1,0,1,0,0,0,0,1,0,},
    {1,0,0,0,0,0,1,1,1,0,0,0,0,0,1,0,},
    {0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,},
    {0,0,1,1,0,0,0,1,0,0,0,1,1,0,0,0,},
    {0,0,0,0,1,1,1,0,1,1,1,0,0,0,0,0,},
    {0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,},
    {0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,}, };

HardwareTimer timer(4);

void setup() {
    // Setup our pins
    pinMode(BOARD_LED_PIN, OUTPUT);
    pinMode(VGA_R, OUTPUT);
    pinMode(VGA_G, OUTPUT);
    pinMode(VGA_B, OUTPUT);
    pinMode(VGA_V, OUTPUT);
    pinMode(VGA_H, OUTPUT);
    digitalWrite(VGA_R, LOW);
    digitalWrite(VGA_G, LOW);
    digitalWrite(VGA_B, LOW);
    digitalWrite(VGA_H, HIGH);
    digitalWrite(VGA_V, HIGH);

    // Fill the logo array with color patterns corresponding to its
    // truth value.  Note that we could get more tricky here, since
    // there are 3 bits of color.
    for (int y = 0; y < y_max; y++) {
        for (int x = 0; x < x_max; x++) {
            logo[y][x] = logo[y][x] ? ON_COLOR : OFF_COLOR;
        }
    }

    // This gets rid of the majority of the interrupt artifacts;
    // there's still a glitch for low values of y, but let's not worry
    // about that. (Probably due to the hackish way vsync is done).
    SerialUSB.end();
    systick_disable();

    // Configure
    timer.pause(); // while we configure
    timer.setPrescaleFactor(1);     // Full speed
    timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
    timer.setMode(TIMER_CH2, TIMER_OUTPUT_COMPARE);
    timer.setMode(TIMER_CH3, TIMER_OUTPUT_COMPARE);
    timer.setMode(TIMER_CH4, TIMER_OUTPUT_COMPARE);
    timer.setOverflow(2287);   // Total line time

    timer.setCompare(TIMER_CH1, 200);
    timer.attachInterrupt(TIMER_CH1, isr_porch);
    timer.setCompare(TIMER_CH2, 300);
    timer.attachInterrupt(TIMER_CH2, isr_start);
    timer.setCompare(TIMER_CH3, 2170);
    timer.attachInterrupt(TIMER_CH3, isr_stop);
    timer.setCompare(TIMER_CH4, 1);      // Could be zero, I guess
    timer.attachInterrupt(TIMER_CH4, isr_update);

    timer.setCount(0);         // Ready...
    timer.resume();            // Go!
}

void loop() {
    toggleLED();
    delay(100);

    // Everything happens in the interrupts!
}

// This ISR will end horizontal sync for most of the image and
// setup the vertical sync for higher line counts
void isr_porch(void) {
    VGA_H_HIGH;
    y++;
    logo_y = map(y, 0, 478, 0, y_max);
    // Back to the top
    if (y >= 523) {
        y = 1;
        logo_y = 0;
        v_active = true;
        return;
    }
    // Other vsync stuff below the image
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

// This is the main horizontal sweep
void isr_start(void) {
    // Skip if we're not in the image at all
    if (!v_active) {
        return;
    }

    // Start Red
    VGA_R_LOW;
    VGA_R_HIGH;

    // For each "pixel", go ON_COLOR or OFF_COLOR
    for (x = 0; x < 16; x++) {
        // setting the color several times is just an easy way to
        // delay, so the image is wider.  if you only do the following
        // once, you'll be able to make the logo array bigger:
        VGA_COLOR(logo[logo_y][x]);
        VGA_COLOR(logo[logo_y][x]);
        VGA_COLOR(logo[logo_y][x]);
        VGA_COLOR(logo[logo_y][x]);
        VGA_COLOR(logo[logo_y][x]);
        VGA_COLOR(logo[logo_y][x]);
    }
}

// End of the horizontal line
void isr_stop(void) {
    if (!v_active) {
        return;
    }
    VGA_R_LOW;
    VGA_G_LOW;
    VGA_B_LOW;
}

// Setup horizonal sync
void isr_update(void) {
    VGA_H_LOW;
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
