/**
 * @file test-usart-dma.cpp
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 *
 * Simple test of DMA used with a USART receiver.
 *
 * Configures a USART receiver for use with DMA.  Received bytes are
 * placed into a buffer, with an interrupt firing when the buffer is
 * full.  At that point, the USART transmitter will print the contents
 * of the byte buffer.  The buffer is continually filled and refilled
 * in this manner.
 *
 * This example isn't very robust; don't use it in production.  In
 * particular, since the buffer keeps filling (DMA_CIRC_MODE is set),
 * if you keep typing after filling the buffer, you'll overwrite
 * earlier bytes; this may happen before those earlier bytes are done
 * printing.
 *
 * This code is released into the public domain.
 */

#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include "wirish.h"

#define BAUD 9600

#define USART USART2
#define USART_HWSER Serial2
#define USART_DMA_DEV DMA1
#define USART_RX_DMA_CHANNEL DMA_CH6
#define USART_TX BOARD_USART2_TX_PIN
#define USART_RX BOARD_USART2_RX_PIN

#define BUF_SIZE 8
uint8 rx_buf[BUF_SIZE];

dma_irq_cause irq_cause;

volatile uint32 irq_fired = 0;

void init_usart(void);
void init_dma_xfer(void);
void rx_dma_irq(void);

void setup(void) {
    pinMode(BOARD_LED_PIN, OUTPUT);

    init_dma_xfer();
    init_usart();
}

void loop(void) {
    toggleLED();
    delay(100);

    dma_channel_reg_map *ch_regs = dma_channel_regs(USART_DMA_DEV,
                                                    USART_RX_DMA_CHANNEL);
    if (irq_fired) {
        USART_HWSER.println("** IRQ **");
        irq_fired = 0;
    }
    USART_HWSER.print("[");
    USART_HWSER.print(millis());
    USART_HWSER.print("]\tISR bits: 0x");
    uint8 isr_bits = dma_get_isr_bits(USART_DMA_DEV, USART_RX_DMA_CHANNEL);
    USART_HWSER.print(isr_bits, HEX);
    USART_HWSER.print("\tCCR: 0x");
    USART_HWSER.print(ch_regs->CCR, HEX);
    USART_HWSER.print("\tCNDTR: 0x");
    USART_HWSER.print(ch_regs->CNDTR, HEX);
    USART_HWSER.print("\tBuffer contents: ");
     for (int i = 0; i < BUF_SIZE; i++) {
        USART_HWSER.print('\'');
        USART_HWSER.print(rx_buf[i]);
        USART_HWSER.print('\'');
        if (i < BUF_SIZE - 1) USART_HWSER.print(", ");
    }
    USART_HWSER.println();
    if (isr_bits == 0x7) {
        USART_HWSER.println("** Clearing ISR bits.");
        dma_clear_isr_bits(USART_DMA_DEV, USART_RX_DMA_CHANNEL);
    }
}

/* Configure USART receiver for use with DMA */
void init_usart(void) {
    USART_HWSER.begin(BAUD);
    USART->regs->CR3 = USART_CR3_DMAR;
}

/* Configure DMA transmission */
void init_dma_xfer(void) {
    dma_init(USART_DMA_DEV);
    dma_setup_transfer(USART_DMA_DEV, USART_RX_DMA_CHANNEL,
                       &USART->regs->DR, DMA_SIZE_8BITS,
                       rx_buf,           DMA_SIZE_8BITS,
                       (DMA_MINC_MODE | DMA_CIRC_MODE | DMA_TRNS_CMPLT));
    dma_set_num_transfers(USART_DMA_DEV, USART_RX_DMA_CHANNEL, BUF_SIZE);
    dma_attach_interrupt(USART_DMA_DEV, USART_RX_DMA_CHANNEL, rx_dma_irq);
    dma_enable(USART_DMA_DEV, USART_RX_DMA_CHANNEL);
}

void rx_dma_irq(void) {
    irq_fired = true;
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
