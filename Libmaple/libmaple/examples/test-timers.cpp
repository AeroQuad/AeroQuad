// Program to test the timer.h implementation's essential functionality.

#include "wirish.h"
#include "timer.h"

void handler1(void);
void handler2(void);
void handler3(void);
void handler4(void);

void handler3b(void);
void handler4b(void);

int t;

int count1 = 0;
int count2 = 0;
int count3 = 0;
int count4 = 0;
uint16 rate1 = 1000;
uint16 rate2 = 2000;
uint16 rate3 = 4000;
uint16 rate4 = 8000;
uint16 val1 = 10000;
uint16 val2 = 10000;
uint16 val3 = 10000;
uint16 val4 = 10000;

// FIXME [0.1.0] high density timer test (especially basic timers + DAC)
timer_dev *timers[] = {TIMER1, TIMER2, TIMER3, TIMER4};
voidFuncPtr handlers[] = {handler1, handler2, handler3, handler4};

void initTimer(timer_dev *dev);
void setTimerPeriod(timer_dev *dev, uint32 period_us);
void testSetTimerPeriod(uint32 period);
void testTimerChannels(timer_dev *dev);
int timerNumber(timer_dev *dev);

void setup() {
    // Set up the LED to blink
    pinMode(BOARD_LED_PIN, OUTPUT);

    // Setup the button as input
    pinMode(BOARD_BUTTON_PIN, INPUT);

    // Send a message out Serial2
    Serial2.begin(115200);
    Serial2.println("*** Initializing timers...");
    timer_foreach(initTimer);
    Serial2.println("*** Done. Beginning timer test.");
}

void loop() {
    Serial2.println("-----------------------------------------------------");

    Serial2.println("Testing timer_get_count()/timer_set_count()");
    Serial2.print("TIMER1 count = ");
    Serial2.println(timer_get_count(TIMER1));
    Serial2.println("timer_set_count(TIMER1, 1234)");
    timer_set_count(TIMER1, 1234);
    Serial2.print("timer_get_count(TIMER1) = ");
    Serial2.println(timer_get_count(TIMER1));

    Serial2.println("-----------------------------------------------------");
    Serial2.println("Testing pause/resume; button roughly controls TIMER4");
    // when BUT is held down, TIMER4 is in the "pause" state and the
    // timer doesn't increment, so the final counts should reflect the
    // ratio of time that BUT was held down.
    count3 = 0;
    count4 = 0;
    timer_set_mode(TIMER3, TIMER_CH1, TIMER_OUTPUT_COMPARE);
    timer_set_mode(TIMER4, TIMER_CH1, TIMER_OUTPUT_COMPARE);
    timer_pause(TIMER3);
    timer_pause(TIMER4);
    timer_set_count(TIMER3, 0);
    timer_set_count(TIMER4, 0);
    timer_set_reload(TIMER3, 30000);
    timer_set_reload(TIMER4, 30000);
    timer_set_compare(TIMER3, 1, 1000);
    timer_set_compare(TIMER4, 1, 1000);
    timer_attach_interrupt(TIMER3, TIMER_CC1_INTERRUPT, handler3b);
    timer_attach_interrupt(TIMER4, TIMER_CC1_INTERRUPT, handler4b);
    timer_resume(TIMER3);
    timer_resume(TIMER4);

    Serial2.println("Testing for ~4 seconds...");
    for(int i = 0; i < 4000; i++) {
        if (isButtonPressed()) {
            timer_pause(TIMER4);
        } else {
            timer_resume(TIMER4);
        }
        delay(1);
    }

    timer_set_mode(TIMER3, TIMER_CH1, TIMER_DISABLED);
    timer_set_mode(TIMER4, TIMER_CH1, TIMER_DISABLED);

    Serial2.print("TIMER3 count: ");
    Serial2.println(timer_get_count(TIMER3));
    Serial2.print("TIMER4 count: ");
    Serial2.println(timer_get_count(TIMER4));

    Serial2.println("-----------------------------------------------------");
    Serial2.println("Testing setTimerPeriod()");
    testSetTimerPeriod(10);
    testSetTimerPeriod(30000);
    testSetTimerPeriod(300000);
    testSetTimerPeriod(30000);

    Serial2.println("Sanity check (with hand-coded reload and prescaler for "
                    "72 MHz timers):");
    timer_set_mode(TIMER4, TIMER_CH1, TIMER_OUTPUT_COMPARE);
    timer_set_prescaler(TIMER4, 33);
    timer_set_reload(TIMER4, 65454);
    timer_pause(TIMER4);
    timer_set_count(TIMER4, 0);
    timer_set_compare(TIMER4, TIMER_CH1, 1);
    timer_attach_interrupt(TIMER4, TIMER_CC1_INTERRUPT, handler4b);
    Serial2.println("Period 30000ms, wait 2 seconds...");
    count4 = 0;
    timer_resume(TIMER4);
    delay(2000);
    timer_pause(TIMER4);
    timer_set_mode(TIMER4, TIMER_CH1, TIMER_DISABLED);
    Serial2.print("TIMER4 count: ");
    Serial2.println(count4);
    Serial2.println("  (Should be around 2sec/30000ms ~ 67)");

    // Test all the individual timer channels
    timer_foreach(testTimerChannels);
}

void initTimer(timer_dev *dev) {
    switch (dev->type) {
    case TIMER_ADVANCED:
    case TIMER_GENERAL:
        Serial2.print("Initializing timer ");
        Serial2.println(timerNumber(dev));
        for (int c = 1; c <= 4; c++) {
            timer_set_mode(dev, c, TIMER_OUTPUT_COMPARE);
        }
        Serial2.println("Done.");
        break;
    case TIMER_BASIC:
        break;
    }
}

void testSetTimerPeriod(uint32 period) {
    timer_set_mode(TIMER4, TIMER_CH1, TIMER_OUTPUT_COMPARE);
    timer_set_compare(TIMER4, TIMER_CH1, 1);
    setTimerPeriod(TIMER4, period);
    timer_pause(TIMER4);
    timer_set_count(TIMER4, 0);
    timer_attach_interrupt(TIMER4, TIMER_CC1_INTERRUPT, handler4b);
    Serial2.println("Period ");
    Serial2.print(period);
    Serial2.print(" ms. Waiting 2 seconds...");
    count4 = 0;
    timer_resume(TIMER4);
    delay(2000);
    timer_pause(TIMER4);
    timer_set_mode(TIMER4, TIMER_CH1, TIMER_DISABLED);
    Serial2.print("TIMER4 count: ");
    Serial2.println(timer_get_count(TIMER4));
    Serial2.print("  (Should be around 2 sec / ");
    Serial2.print(period);
    Serial2.print(" ms = ");
    Serial2.print(double(2) / period * 1000);
    Serial2.println(", modulo delays due to interrupts)");
}

int timerNumber(timer_dev *dev) {
    switch (dev->clk_id) {
    case RCC_TIMER1:
        return 1;
    case RCC_TIMER2:
        return 2;
    case RCC_TIMER3:
        return 3;
    case RCC_TIMER4:
        return 4;
#ifdef STM32_HIGH_DENSITY
    case RCC_TIMER5:
        return 5;
    case RCC_TIMER6:
        return 6;
    case RCC_TIMER7:
        return 7;
    case RCC_TIMER8:
        return 8;
#endif
    default:
        ASSERT(0);
        return 0;
    }
}

/* This function touches every channel of a given timer. The output
 * ratios should reflect the ratios of the rate variables.  It
 * demonstrates that, over time, the actual timing rates get blown
 * away by other system interrupts. */
void testTimerChannels(timer_dev *dev) {
    t = timerNumber(dev);
    toggleLED();
    delay(100);
    Serial2.println("-----------------------------------------------------");
    switch (dev->type) {
    case TIMER_BASIC:
        Serial2.print("NOT testing channels for basic timer ");
        Serial2.println(t);
        break;
    case TIMER_ADVANCED:
    case TIMER_GENERAL:
        Serial2.print("Testing channels for timer ");
        Serial2.println(t);
        timer_pause(dev);
        count1 = count2 = count3 = count4 = 0;
        timer_set_reload(dev, 0xFFFF);
        timer_set_prescaler(dev, 1);
        for (int c = 1; c <= 4; c++) {
            timer_set_compare(dev, c, 65535);
            timer_set_mode(dev, c, TIMER_OUTPUT_COMPARE);
            timer_attach_interrupt(dev, c, handlers[c - 1]);
        }
        timer_resume(dev);
        delay(3000);
        for (int c = 1; c <= 4; c++) {
            timer_set_mode(dev, c, TIMER_DISABLED);
        }
        Serial2.print("Channel 1 count: "); Serial2.println(count1);
        Serial2.print("Channel 2 count: "); Serial2.println(count2);
        Serial2.print("Channel 3 count: "); Serial2.println(count3);
        Serial2.print("Channel 4 count: "); Serial2.println(count4);
        break;
    }
}

// FIXME [0.1.0] move some incarnation of this into timer.h
void setTimerPeriod(timer_dev *dev, uint32 period_us) {
    if (!period_us) {
        // FIXME handle this case
        ASSERT(0);
        return;
    }

    uint32 cycles = period_us * CYCLES_PER_MICROSECOND;
    uint16 pre = (uint16)((cycles >> 16) + 1);
    timer_set_prescaler(dev, pre);
    timer_set_reload(dev, cycles / pre - 1);
}

void handler1(void) {
    val1 += rate1;
    timer_set_compare(timers[t], TIMER_CH1, val1);
    count1++;
}

void handler2(void) {
    val2 += rate2;
    timer_set_compare(timers[t], TIMER_CH2, val2);
    count2++;
}

void handler3(void) {
    val3 += rate3;
    timer_set_compare(timers[t], TIMER_CH3, val3);
    count3++;
}

void handler4(void) {
    val4 += rate4;
    timer_set_compare(timers[t], TIMER_CH4, val4);
    count4++;
}

void handler3b(void) {
    count3++;
}

void handler4b(void) {
    count4++;
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
