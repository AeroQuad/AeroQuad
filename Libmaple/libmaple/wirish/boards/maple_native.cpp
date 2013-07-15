/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file   maple_native.cpp
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief  Maple Native board file.
 */

#ifdef BOARD_maple_native

#include "maple_native.h"

#include "fsmc.h"
#include "gpio.h"
#include "rcc.h"
#include "timer.h"

#include "wirish_types.h"

static void initSRAMChip(void);

void boardInit(void) {
    initSRAMChip();
}

extern const stm32_pin_info PIN_MAP[BOARD_NR_GPIO_PINS] = {

    /* Top header */

    {GPIOB,   NULL, NULL, 10, 0, ADCx}, /* D0/PB10 */
    {GPIOB,   NULL, NULL, 11, 0, ADCx}, /* D1/PB11 */
    {GPIOB,   NULL, NULL, 12, 0, ADCx}, /* D2/PB12 */
    {GPIOB,   NULL, NULL, 13, 0, ADCx}, /* D3/PB13 */
    {GPIOB,   NULL, NULL, 14, 0, ADCx}, /* D4/PB14 */
    {GPIOB,   NULL, NULL, 15, 0, ADCx}, /* D5/PB15 */
    {GPIOG,   NULL, NULL, 15, 0, ADCx}, /* D6/PG15 (BUT) */
    {GPIOC,   NULL, ADC1,  0, 0,   10}, /* D7/PC0 */
    {GPIOC,   NULL, ADC1,  1, 0,   11}, /* D8/PC1 */
    {GPIOC,   NULL, ADC1,  2, 0,   12}, /* D9/PC2 */
    {GPIOC,   NULL, ADC1,  3, 0,   13}, /* D10/PC3 */
    {GPIOC,   NULL, ADC1,  4, 0,   14}, /* D11/PC4 */
    {GPIOC,   NULL, ADC1,  5, 0,   15}, /* D12/PC5 */
    {GPIOC, TIMER8, NULL,  6, 1, ADCx}, /* D13/PC6 */
    {GPIOC, TIMER8, NULL,  7, 2, ADCx}, /* D14/PC7 */
    {GPIOC, TIMER8, NULL,  8, 3, ADCx}, /* D15/PC8 */
    {GPIOC, TIMER8, NULL,  9, 4, ADCx}, /* D16/PC9 */
    {GPIOC,   NULL, NULL, 10, 0, ADCx}, /* D17/PC10 */
    {GPIOC,   NULL, NULL, 11, 0, ADCx}, /* D18/PC11 */
    {GPIOC,   NULL, NULL, 12, 0, ADCx}, /* D19/PC12 */
    {GPIOC,   NULL, NULL, 13, 0, ADCx}, /* D20/PC13 */
    {GPIOC,   NULL, NULL, 14, 0, ADCx}, /* D21/PC14 */
    {GPIOC,   NULL, NULL, 15, 0, ADCx}, /* D22/PC15 (LED) */
    {GPIOA, TIMER1, NULL,  8, 1, ADCx}, /* D23/PA8 */
    {GPIOA, TIMER1, NULL,  9, 2, ADCx}, /* D24/PA9 */
    {GPIOA, TIMER1, NULL, 10, 3, ADCx}, /* D25/PA10 */
    {GPIOB, TIMER4, NULL,  9, 4, ADCx}, /* D26/PB9 */

    /* Bottom header */
    /* Note: D{48, 49, 50, 51} are also TIMER2_CH{1, 2, 3, 4}, respectively. */
    /* TODO remap timer 2 in boardInit(); make the appropriate changes here */

    {GPIOD,   NULL, NULL,  2, 0, ADCx}, /* D27/PD2 */
    {GPIOD,   NULL, NULL,  3, 0, ADCx}, /* D28/PD3 */
    {GPIOD,   NULL, NULL,  6, 0, ADCx}, /* D29/PD6 */
    {GPIOG,   NULL, NULL, 11, 0, ADCx}, /* D30/PG11 */
    {GPIOG,   NULL, NULL, 12, 0, ADCx}, /* D31/PG12 */
    {GPIOG,   NULL, NULL, 13, 0, ADCx}, /* D32/PG13 */
    {GPIOG,   NULL, NULL, 14, 0, ADCx}, /* D33/PG14 */
    {GPIOG,   NULL, NULL,  8, 0, ADCx}, /* D34/PG8 */
    {GPIOG,   NULL, NULL,  7, 0, ADCx}, /* D35/PG7 */
    {GPIOG,   NULL, NULL,  6, 0, ADCx}, /* D36/PG6 */
    {GPIOB,   NULL, NULL,  5, 0, ADCx}, /* D37/PB5 */
    {GPIOB, TIMER4, NULL,  6, 1, ADCx}, /* D38/PB6 */
    {GPIOB, TIMER4, NULL,  7, 2, ADCx}, /* D39/PB7 */
    {GPIOF,   NULL, NULL, 11, 0, ADCx}, /* D40/PF11 */
    {GPIOF,   NULL, ADC3,  6, 0,    4}, /* D41/PF6 */
    {GPIOF,   NULL, ADC3,  7, 0,    5}, /* D42/PF7 */
    {GPIOF,   NULL, ADC3,  8, 0,    6}, /* D43/PF8 */
    {GPIOF,   NULL, ADC3,  9, 0,    7}, /* D44/PF9 */
    {GPIOF,   NULL, ADC3, 10, 0,    8}, /* D45/PF10 */
    {GPIOB, TIMER3, ADC1,  1, 4,    9}, /* D46/PB1 */
    {GPIOB, TIMER3, ADC1,  0, 3,    8}, /* D47/PB0 */
    {GPIOA, TIMER5, ADC1,  0, 1,    0}, /* D48/PA0 */
    {GPIOA, TIMER5, ADC1,  1, 2,    1}, /* D49/PA1 */
    {GPIOA, TIMER5, ADC1,  2, 3,    2}, /* D50/PA2 */
    {GPIOA, TIMER5, ADC1,  3, 4,    3}, /* D51/PA3 */
    {GPIOA,   NULL, ADC1,  4, 0,    4}, /* D52/PA4 */
    {GPIOA,   NULL, ADC1,  5, 0,    5}, /* D53/PA5 */
    {GPIOA, TIMER3, ADC1,  6, 1,    6}, /* D54/PA6 */
    {GPIOA, TIMER3, ADC1,  7, 2,    7}, /* D55/PA7 */

    /* FSMC (triple) header */

    {GPIOF,   NULL, NULL,  0, 0, ADCx}, /* D56/PF0 */
    {GPIOD,   NULL, NULL, 11, 0, ADCx}, /* D57/PD11 */
    {GPIOD,   NULL, NULL, 14, 0, ADCx}, /* D58/PD14 */
    {GPIOF,   NULL, NULL,  1, 0, ADCx}, /* D59/PF1 */
    {GPIOD,   NULL, NULL, 12, 0, ADCx}, /* D60/PD12 */
    {GPIOD,   NULL, NULL, 15, 0, ADCx}, /* D61/PD15 */
    {GPIOF,   NULL, NULL,  2, 0, ADCx}, /* D62/PF2 */
    {GPIOD,   NULL, NULL, 13, 0, ADCx}, /* D63/PD13 */
    {GPIOD,   NULL, NULL,  0, 0, ADCx}, /* D64/PD0 */
    {GPIOF,   NULL, NULL,  3, 0, ADCx}, /* D65/PF3 */
    {GPIOE,   NULL, NULL,  3, 0, ADCx}, /* D66/PE3 */
    {GPIOD,   NULL, NULL,  1, 0, ADCx}, /* D67/PD1 */
    {GPIOF,   NULL, NULL,  4, 0, ADCx}, /* D68/PF4 */
    {GPIOE,   NULL, NULL,  4, 0, ADCx}, /* D69/PE4 */
    {GPIOE,   NULL, NULL,  7, 0, ADCx}, /* D70/PE7 */
    {GPIOF,   NULL, NULL,  5, 0, ADCx}, /* D71/PF5 */
    {GPIOE,   NULL, NULL,  5, 0, ADCx}, /* D72/PE5 */
    {GPIOE,   NULL, NULL,  8, 0, ADCx}, /* D73/PE8 */
    {GPIOF,   NULL, NULL, 12, 0, ADCx}, /* D74/PF12 */
    {GPIOE,   NULL, NULL,  6, 0, ADCx}, /* D75/PE6 */
    {GPIOE,   NULL, NULL,  9, 0, ADCx}, /* D76/PE9 */
    {GPIOF,   NULL, NULL, 13, 0, ADCx}, /* D77/PF13 */
    {GPIOE,   NULL, NULL, 10, 0, ADCx}, /* D78/PE10 */
    {GPIOF,   NULL, NULL, 14, 0, ADCx}, /* D79/PF14 */
    {GPIOG,   NULL, NULL,  9, 0, ADCx}, /* D80/PG9 */
    {GPIOE,   NULL, NULL, 11, 0, ADCx}, /* D81/PE11 */
    {GPIOF,   NULL, NULL, 15, 0, ADCx}, /* D82/PF15 */
    {GPIOG,   NULL, NULL, 10, 0, ADCx}, /* D83/PG10 */
    {GPIOE,   NULL, NULL, 12, 0, ADCx}, /* D84/PE12 */
    {GPIOG,   NULL, NULL,  0, 0, ADCx}, /* D85/PG0 */
    {GPIOD,   NULL, NULL,  5, 0, ADCx}, /* D86/PD5 */
    {GPIOE,   NULL, NULL, 13, 0, ADCx}, /* D87/PE13 */
    {GPIOG,   NULL, NULL,  1, 0, ADCx}, /* D88/PG1 */
    {GPIOD,   NULL, NULL,  4, 0, ADCx}, /* D89/PD4 */
    {GPIOE,   NULL, NULL, 14, 0, ADCx}, /* D90/PE14 */
    {GPIOG,   NULL, NULL,  2, 0, ADCx}, /* D91/PG2 */
    {GPIOE,   NULL, NULL,  1, 0, ADCx}, /* D92/PE1 */
    {GPIOE,   NULL, NULL, 15, 0, ADCx}, /* D93/PE15 */
    {GPIOG,   NULL, NULL,  3, 0, ADCx}, /* D94/PG3 */
    {GPIOE,   NULL, NULL,  0, 0, ADCx}, /* D95/PE0 */
    {GPIOD,   NULL, NULL,  8, 0, ADCx}, /* D96/PD8 */
    {GPIOG,   NULL, NULL,  4, 0, ADCx}, /* D97/PG4 */
    {GPIOD,   NULL, NULL,  9, 0, ADCx}, /* D98/PD9 */
    {GPIOG,   NULL, NULL,  5, 0, ADCx}, /* D99/PG5 */
    {GPIOD,   NULL, NULL, 10, 0, ADCx}, /* D100/PD10 */

    /* JTAG header */

    {GPIOA,   NULL, NULL, 13, 0, ADCx}, /* D101/PA13 */
    {GPIOA,   NULL, NULL, 14, 0, ADCx}, /* D102/PA14 */
    {GPIOA,   NULL, NULL, 15, 0, ADCx}, /* D103/PA15 */
    {GPIOB,   NULL, NULL,  3, 0, ADCx}, /* D104/PB3  */
    {GPIOB,   NULL, NULL,  4, 0, ADCx}  /* D105/PB4  */
};

extern const uint8 boardPWMPins[BOARD_NR_PWM_PINS] __FLASH__ = {
    13, 14, 15, 16, 23, 24, 25, 26, 38, 39, 46, 47, 48, 49, 50, 51, 54, 55
};

extern const uint8 boardADCPins[BOARD_NR_ADC_PINS] __FLASH__ = {
    7, 8, 9, 10, 11, 12, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53,
    54, 55
};

extern const uint8 boardUsedPins[BOARD_NR_USED_PINS] __FLASH__ = {
    BOARD_LED_PIN, BOARD_BUTTON_PIN, BOARD_JTMS_SWDIO_PIN,
    BOARD_JTCK_SWCLK_PIN, BOARD_JTDI_PIN, BOARD_JTDO_PIN, BOARD_NJTRST_PIN,
    56, 58, 59, 61, 62, 64, 65, 67, 68, 70, 71, 73, 74, 76, 77, 78, 79, 81,
    82, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100
};

static void initSRAMChip(void) {
    fsmc_nor_psram_reg_map *regs = FSMC_NOR_PSRAM1_BASE;

    fsmc_sram_init_gpios();
    rcc_clk_enable(RCC_FSMC);

    regs->BCR = (FSMC_BCR_WREN | FSMC_BCR_MWID_16BITS | FSMC_BCR_MTYP_SRAM |
                 FSMC_BCR_MBKEN);
    fsmc_nor_psram_set_addset(regs, 0);
    fsmc_nor_psram_set_datast(regs, 3);
}

#endif
