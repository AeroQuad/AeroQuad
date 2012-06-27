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
 * @file   maple_native.h
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief  Private include file for Maple Native in boards.h
 *
 * See maple.h for more information on these definitions.
 */

#ifndef _BOARD_MAPLE_NATIVE_H_
#define _BOARD_MAPLE_NATIVE_H_

#define CYCLES_PER_MICROSECOND  72
#define SYSTICK_RELOAD_VAL      71999

#define BOARD_LED_PIN           22
#define BOARD_BUTTON_PIN        6

#define BOARD_NR_USARTS         5
#define BOARD_USART1_TX_PIN     24
#define BOARD_USART1_RX_PIN     25
#define BOARD_USART2_TX_PIN     50
#define BOARD_USART2_RX_PIN     51
#define BOARD_USART3_TX_PIN     0
#define BOARD_USART3_RX_PIN     1
#define BOARD_UART4_TX_PIN      17
#define BOARD_UART4_RX_PIN      18
#define BOARD_UART5_TX_PIN      19
#define BOARD_UART5_RX_PIN      27

#define BOARD_NR_SPI            3
#define BOARD_SPI1_NSS_PIN      52
#define BOARD_SPI1_MOSI_PIN     55
#define BOARD_SPI1_MISO_PIN     54
#define BOARD_SPI1_SCK_PIN      53
#define BOARD_SPI2_NSS_PIN      2
#define BOARD_SPI2_MOSI_PIN     5
#define BOARD_SPI2_MISO_PIN     4
#define BOARD_SPI2_SCK_PIN      3
#define BOARD_SPI3_NSS_PIN      103
#define BOARD_SPI3_MOSI_PIN     37
#define BOARD_SPI3_MISO_PIN     105
#define BOARD_SPI3_SCK_PIN      104

#define BOARD_NR_GPIO_PINS      106
#define BOARD_NR_PWM_PINS       18
#define BOARD_NR_ADC_PINS       21
#define BOARD_NR_USED_PINS      43
#define BOARD_JTMS_SWDIO_PIN    101
#define BOARD_JTCK_SWCLK_PIN    102
#define BOARD_JTDI_PIN          103
#define BOARD_JTDO_PIN          104
#define BOARD_NJTRST_PIN        105

#endif
