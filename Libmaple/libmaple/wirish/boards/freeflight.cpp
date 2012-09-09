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
 * @file   aeroquad32mini.cpp
 * @author ala42
 * @brief  aeroquad32mini board file.
 */

#ifdef BOARD_freeflight

#include "freeflight.h"

//#include "fsmc.h"
#include "gpio.h"
#include "rcc.h"
#include "timer.h"

#include "wirish_types.h"

//static void initSRAMChip(void);
void boardInit(void) {
	__io uint32 *mapr = &AFIO_BASE->MAPR;
	*mapr = ((*mapr) & ~(7 << 24)  )
		| AFIO_MAPR_SWJ_CFG_NO_JTAG_SW;
}


#if 0
typedef struct stm32_pin_info {
    gpio_dev *gpio_device;      /**< Maple pin's GPIO device */
    timer_dev *timer_device;    /**< Pin's timer device, if any. */
    const adc_dev *adc_device;  /**< ADC device, if any. */
    uint8 gpio_bit;             /**< Pin's GPIO port bit. */
    uint8 timer_channel;        /**< Timer channel, or 0 if none. */
    uint8 adc_channel;          /**< Pin ADC channel, or ADCx if none. */
} stm32_pin_info;

#endif

extern const stm32_pin_info PIN_MAP[BOARD_NR_GPIO_PINS] = {
    {GPIOA, TIMER2, ADC1,  0, 1,    0}, /* D00/PA0  */
    {GPIOA, TIMER2, ADC1,  1, 2,    1}, /* D01/PA1  */
    {GPIOA, TIMER2, ADC1,  2, 3,    2}, /* D02/PA2  */
    {GPIOA, TIMER2, ADC1,  3, 4,    3}, /* D03/PA3  */
    {GPIOA,   NULL, ADC1,  4, 0,    4}, /* D04/PA4  */
    {GPIOA,   NULL, ADC1,  5, 0,    5}, /* D05/PA5  */
    {GPIOA, TIMER3, ADC1,  6, 1,    6}, /* D06/PA6  */ // ala check TIMER3
    {GPIOA, TIMER3, ADC1,  7, 2,    7}, /* D07/PA7  */
    {GPIOA, TIMER1, NULL,  8, 1, ADCx}, /* D08/PA8  */ // remap out
    {GPIOA,   NULL, NULL,  9, 0, ADCx}, /* D09/PA9  */ // remap out
    {GPIOA,   NULL, NULL, 10, 0, ADCx}, /* D10/PA10 */ // remap out
    {GPIOA, TIMER1, NULL, 11, 4, ADCx}, /* D11/PA11 */ // remap out
    {GPIOA,   NULL, NULL, 12, 0, ADCx}, /* D12/PA12 */
    {GPIOA,   NULL, NULL, 13, 0, ADCx}, /* D13/PA13 */
    {GPIOA,   NULL, NULL, 14, 0, ADCx}, /* D14/PA14 */
    {GPIOA,   NULL, NULL, 15, 0, ADCx}, /* D15/PA15 */ // remap in

    {GPIOB, TIMER3, ADC1,  0, 3,    8}, /* D16/PB0  */
    {GPIOB, TIMER3, ADC1,  1, 4,    9}, /* D17/PB1  */
    {GPIOB,   NULL, NULL,  2, 0, ADCx}, /* D18/PB2  */
    {GPIOB,   NULL, NULL,  3, 2, ADCx}, /* D19/PB3  */ // remap in
    {GPIOB,   NULL, NULL,  4, 1, ADCx}, /* D20/PB4  */ // remap in
    {GPIOB,   NULL, NULL,  5, 2, ADCx}, /* D21/PB5  */ // remap in
    {GPIOB, TIMER4, NULL,  6, 1, ADCx}, /* D22/PB6  */ // remap out
    {GPIOB, TIMER4, NULL,  7, 2, ADCx}, /* D23/PB7  */ // remap out
    {GPIOB, TIMER4, NULL,  8, 3, ADCx}, /* D24/PB8  */ // remap out
    {GPIOB, TIMER4, NULL,  9, 4, ADCx}, /* D25/PB9  */ // remap out
    {GPIOB,   NULL, NULL, 10, 0, ADCx}, /* D26/PB10 */
    {GPIOB,   NULL, NULL, 11, 0, ADCx}, /* D27/PB11 */
    {GPIOB,   NULL, NULL, 12, 0, ADCx}, /* D28/PB12 */
    {GPIOB,   NULL, NULL, 13, 0, ADCx}, /* D29/PB13 */
    {GPIOB,   NULL, NULL, 14, 0, ADCx}, /* D30/PB14 */
    {GPIOB,   NULL, NULL, 15, 0, ADCx}, /* D31/PB15 */
};

extern const uint8 boardPWMPins[BOARD_NR_PWM_PINS] __FLASH__ = {
    0, 1, 2, 3, 6, 7, 8, 11, 16,17, 22, 23, 24, 25
};

extern const uint8 boardADCPins[BOARD_NR_ADC_PINS] __FLASH__ = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9
};

extern const uint8 boardUsedPins[BOARD_NR_USED_PINS] __FLASH__ = {
    BOARD_LED_PIN, BOARD_BUTTON_PIN, BOARD_JTMS_SWDIO_PIN,
    BOARD_JTCK_SWCLK_PIN, BOARD_JTDI_PIN, BOARD_JTDO_PIN, BOARD_NJTRST_PIN
};

#endif
