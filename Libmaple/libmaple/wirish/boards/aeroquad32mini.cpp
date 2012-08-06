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

#ifdef BOARD_aeroquad32mini

#include "aeroquad32mini.h"

//#include "fsmc.h"
#include "gpio.h"
#include "rcc.h"
#include "timer.h"

#include "wirish_types.h"

//static void initSRAMChip(void);
void boardInit(void) {
	__io uint32 *mapr = &AFIO_BASE->MAPR;
	*mapr = ((*mapr) & ~(7 << 24)  )
		| AFIO_MAPR_TIM2_REMAP_PA15_PB3_PA2_PA3
		| AFIO_MAPR_SWJ_CFG_NO_JTAG_NO_SW;
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
    {GPIOA,   NULL, NULL,  9, 0, ADCx}, /* D00/PA9  */
    {GPIOA,   NULL, NULL, 10, 0, ADCx}, /* D01/PA10 */

	{GPIOB, TIMER4, NULL,  7, 2, ADCx}, /* D02/PB7  */
    {GPIOA, TIMER3, ADC1,  7, 2,    7}, /* D03/PA7  */
    {GPIOB, TIMER4, NULL,  6, 1, ADCx}, /* D04/PB6  */
    {GPIOB, TIMER4, NULL,  8, 3, ADCx}, /* D05/PB8  */
    {GPIOB, TIMER4, NULL,  9, 4, ADCx}, /* D06/PB9  */
    {GPIOA, TIMER2, NULL, 15, 1, ADCx}, /* D07/PA15 */ // remap in
    {GPIOB, TIMER2, NULL,  3, 2, ADCx}, /* D08/PB3  */ // remap in

    {GPIOA, TIMER3, ADC1,  6, 1,    6}, /* D09/PA6  */ // ala check TIMER3
    {GPIOB, TIMER3, ADC1,  0, 3,    8}, /* D10/PB0  */
    {GPIOB, TIMER3, ADC1,  1, 4,    9}, /* D11/PB1  */
    {GPIOA, TIMER2, ADC1,  2, 3,    2}, /* D12/PA2  */
    {GPIOA, TIMER2, ADC1,  3, 4,    3}, /* D13/PA3  */

	{GPIOA,   NULL, ADC1,  0, 0,    0}, /* D14/PA0  */
    {GPIOA,   NULL, ADC1,  5, 0,    5}, /* D15/PA5  */

    {GPIOB,   NULL, NULL, 11, 0, ADCx}, /* D16/PB11 */
    {GPIOB,   NULL, NULL, 10, 0, ADCx}, /* D17/PB10 */

	{GPIOA,   NULL, ADC1,  4, 0, ADCx}, /* D18/PA4  */
};

extern const uint8 boardPWMPins[BOARD_NR_PWM_PINS] __FLASH__ = {
    2, 3, 4, 5,  6, 7, 8, 9,  10, 11, 12, 13
};

extern const uint8 boardADCPins[BOARD_NR_ADC_PINS] __FLASH__ = {
    3, 9, 10, 11, 12, 13, 14, 15
};

extern const uint8 boardUsedPins[BOARD_NR_USED_PINS] __FLASH__ = {
    BOARD_LED_PIN, BOARD_BUTTON_PIN, BOARD_JTMS_SWDIO_PIN,
    BOARD_JTCK_SWCLK_PIN, BOARD_JTDI_PIN, BOARD_JTDO_PIN, BOARD_NJTRST_PIN
};

#endif
