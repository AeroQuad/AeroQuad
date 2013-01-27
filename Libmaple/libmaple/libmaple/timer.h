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
 * @file   timer.h
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief  New-style timer interface.
 *
 * Replaces old timers.h implementation.
 */

#ifndef _TIMERS_H_
#define _TIMERS_H_

#include "libmaple.h"
#include "rcc.h"
#include "nvic.h"
#include "bitband.h"

#ifdef __cplusplus
extern "C"{
#endif

/*
 * Register maps and devices
 */

/** Advanced control timer register map type */
typedef struct timer_adv_reg_map {
    __io uint32 CR1;            /**< Control register 1 */
    __io uint32 CR2;            /**< Control register 2 */
    __io uint32 SMCR;           /**< Slave mode control register */
    __io uint32 DIER;           /**< DMA/Interrupt enable register */
    __io uint32 SR;             /**< Status register */
    __io uint32 EGR;            /**< Event generation register  */
    __io uint32 CCMR1;          /**< Capture/compare mode register 1 */
    __io uint32 CCMR2;          /**< Capture/compare mode register 2 */
    __io uint32 CCER;           /**< Capture/compare enable register */
    __io uint32 CNT;            /**< Counter */
    __io uint32 PSC;            /**< Prescaler */
    __io uint32 ARR;            /**< Auto-reload register */
    __io uint32 RCR;            /**< Repetition counter register */
    __io uint32 CCR1;           /**< Capture/compare register 1 */
    __io uint32 CCR2;           /**< Capture/compare register 2 */
    __io uint32 CCR3;           /**< Capture/compare register 3 */
    __io uint32 CCR4;           /**< Capture/compare register 4 */
    __io uint32 BDTR;           /**< Break and dead-time register */
    __io uint32 DCR;            /**< DMA control register */
    __io uint32 DMAR;           /**< DMA address for full transfer */
} timer_adv_reg_map;

/** General purpose timer register map type */
typedef struct timer_gen_reg_map {
    __io uint32 CR1;            /**< Control register 1 */
    __io uint32 CR2;            /**< Control register 2 */
    __io uint32 SMCR;           /**< Slave mode control register */
    __io uint32 DIER;           /**< DMA/Interrupt enable register */
    __io uint32 SR;             /**< Status register */
    __io uint32 EGR;            /**< Event generation register  */
    __io uint32 CCMR1;          /**< Capture/compare mode register 1 */
    __io uint32 CCMR2;          /**< Capture/compare mode register 2 */
    __io uint32 CCER;           /**< Capture/compare enable register */
    __io uint32 CNT;            /**< Counter */
    __io uint32 PSC;            /**< Prescaler */
    __io uint32 ARR;            /**< Auto-reload register */
    const uint32 RESERVED1;     /**< Reserved */
    __io uint32 CCR1;           /**< Capture/compare register 1 */
    __io uint32 CCR2;           /**< Capture/compare register 2 */
    __io uint32 CCR3;           /**< Capture/compare register 3 */
    __io uint32 CCR4;           /**< Capture/compare register 4 */
    const uint32 RESERVED2;     /**< Reserved */
    __io uint32 DCR;            /**< DMA control register */
    __io uint32 DMAR;           /**< DMA address for full transfer */
} timer_gen_reg_map;

/** Basic timer register map type */
typedef struct timer_bas_reg_map {
    __io uint32 CR1;            /**< Control register 1 */
    __io uint32 CR2;            /**< Control register 2 */
    const uint32 RESERVED1;     /**< Reserved */
    __io uint32 DIER;           /**< DMA/Interrupt enable register */
    __io uint32 SR;             /**< Status register */
    __io uint32 EGR;            /**< Event generation register  */
    const uint32 RESERVED2;     /**< Reserved */
    const uint32 RESERVED3;     /**< Reserved */
    const uint32 RESERVED4;     /**< Reserved */
    __io uint32 CNT;            /**< Counter */
    __io uint32 PSC;            /**< Prescaler */
    __io uint32 ARR;            /**< Auto-reload register */
} timer_bas_reg_map;


#ifdef STM32F2
  /** Timer 1 register map base pointer */
  #define TIMER1_BASE        ((struct timer_adv_reg_map*)0x40010000)
#else
  /** Timer 1 register map base pointer */
  #define TIMER1_BASE        ((struct timer_adv_reg_map*)0x40012C00)
#endif
/** Timer 2 register map base pointer */
#define TIMER2_BASE        ((struct timer_gen_reg_map*)0x40000000)
/** Timer 3 register map base pointer */
#define TIMER3_BASE        ((struct timer_gen_reg_map*)0x40000400)
/** Timer 4 register map base pointer */
#define TIMER4_BASE        ((struct timer_gen_reg_map*)0x40000800)
#ifdef STM32_HIGH_DENSITY
/** Timer 5 register map base pointer */
#define TIMER5_BASE        ((struct timer_gen_reg_map*)0x40000C00)
/** Timer 6 register map base pointer */
#define TIMER6_BASE        ((struct timer_bas_reg_map*)0x40001000)
/** Timer 7 register map base pointer */
#define TIMER7_BASE        ((struct timer_bas_reg_map*)0x40001400)

#ifdef STM32F2
  /** Timer 8 register map base pointer */
  #define TIMER8_BASE        ((struct timer_adv_reg_map*)0x40010400)
#else
  /** Timer 8 register map base pointer */
  #define TIMER8_BASE        ((struct timer_adv_reg_map*)0x40013400)
#endif
#endif

/*
 * Timer devices
 */

/**
 * @brief Timer register map type.
 *
 * Just holds a pointer to the correct type of register map, based on
 * the timer's type.
 */
typedef union timer_reg_map {
    timer_adv_reg_map *adv;     /**< Advanced register map */
    timer_gen_reg_map *gen;     /**< General purpose register map */
    timer_bas_reg_map *bas;     /**< Basic register map */
} timer_reg_map;

/**
 * @brief Timer type
 *
 * Type marker for timer_dev.
 *
 * @see timer_dev
 */
typedef enum timer_type {
    TIMER_ADVANCED,             /**< Advanced type */
    TIMER_GENERAL,              /**< General purpose type */
    TIMER_BASIC                 /**< Basic type */
} timer_type;

/** Timer device type */
typedef struct timer_dev {
    timer_reg_map regs;         /**< Register map */
    rcc_clk_id clk_id;          /**< RCC clock information */
    timer_type type;            /**< Timer's type */
    voidFuncPtr handlers[];     /**< User IRQ handlers */
} timer_dev;

extern timer_dev *TIMER1;
extern timer_dev *TIMER2;
extern timer_dev *TIMER3;
extern timer_dev *TIMER4;
#ifdef STM32_HIGH_DENSITY
extern timer_dev *TIMER5;
extern timer_dev *TIMER6;
extern timer_dev *TIMER7;
extern timer_dev *TIMER8;
#endif

/*
 * Register bit definitions
 */

/* Control register 1 (CR1) */

#define TIMER_CR1_ARPE_BIT              7
#define TIMER_CR1_DIR_BIT               4
#define TIMER_CR1_OPM_BIT               3
#define TIMER_CR1_URS_BIT               2
#define TIMER_CR1_UDIS_BIT              1
#define TIMER_CR1_CEN_BIT               0

#define TIMER_CR1_CKD                   (0x3 << 8)
#define TIMER_CR1_CKD_1TCKINT           (0x0 << 8)
#define TIMER_CR1_CKD_2TCKINT           (0x1 << 8)
#define TIMER_CR1_CKD_4TICKINT          (0x2 << 8)
#define TIMER_CR1_ARPE                  BIT(TIMER_CR1_ARPE_BIT)
#define TIMER_CR1_CKD_CMS               (0x3 << 5)
#define TIMER_CR1_CKD_CMS_EDGE          (0x0 << 5)
#define TIMER_CR1_CKD_CMS_CENTER1       (0x1 << 5)
#define TIMER_CR1_CKD_CMS_CENTER2       (0x2 << 5)
#define TIMER_CR1_CKD_CMS_CENTER3       (0x3 << 5)
#define TIMER_CR1_DIR                   BIT(TIMER_CR1_DIR_BIT)
#define TIMER_CR1_OPM                   BIT(TIMER_CR1_OPM_BIT)
#define TIMER_CR1_URS                   BIT(TIMER_CR1_URS_BIT)
#define TIMER_CR1_UDIS                  BIT(TIMER_CR1_UDIS_BIT)
#define TIMER_CR1_CEN                   BIT(TIMER_CR1_CEN_BIT)

/* Control register 2 (CR2) */

#define TIMER_CR2_OIS4_BIT              14
#define TIMER_CR2_OIS3N_BIT             13
#define TIMER_CR2_OIS3_BIT              12
#define TIMER_CR2_OIS2N_BIT             11
#define TIMER_CR2_OIS2_BIT              10
#define TIMER_CR2_OIS1N_BIT             9
#define TIMER_CR2_OIS1_BIT              8
#define TIMER_CR2_TI1S_BIT              7 /* tills? yikes */
#define TIMER_CR2_CCDS_BIT              3
#define TIMER_CR2_CCUS_BIT              2
#define TIMER_CR2_CCPC_BIT              0

#define TIMER_CR2_OIS4                  BIT(TIMER_CR2_OIS4_BIT)
#define TIMER_CR2_OIS3N                 BIT(TIMER_CR2_OIS3N_BIT)
#define TIMER_CR2_OIS3                  BIT(TIMER_CR2_OIS3_BIT)
#define TIMER_CR2_OIS2N                 BIT(TIMER_CR2_OIS2N_BIT)
#define TIMER_CR2_OIS2                  BIT(TIMER_CR2_OIS2_BIT)
#define TIMER_CR2_OIS1N                 BIT(TIMER_CR2_OIS1N_BIT)
#define TIMER_CR2_OIS1                  BIT(TIMER_CR2_OIS1_BIT)
#define TIMER_CR2_TI1S                  BIT(TIMER_CR2_TI1S_BIT)
#define TIMER_CR2_MMS                   (0x7 << 4)
#define TIMER_CR2_MMS_RESET             (0x0 << 4)
#define TIMER_CR2_MMS_ENABLE            (0x1 << 4)
#define TIMER_CR2_MMS_UPDATE            (0x2 << 4)
#define TIMER_CR2_MMS_COMPARE_PULSE     (0x3 << 4)
#define TIMER_CR2_MMS_COMPARE_OC1REF    (0x4 << 4)
#define TIMER_CR2_MMS_COMPARE_OC2REF    (0x5 << 4)
#define TIMER_CR2_MMS_COMPARE_OC3REF    (0x6 << 4)
#define TIMER_CR2_MMS_COMPARE_OC4REF    (0x7 << 4)
#define TIMER_CR2_CCDS                  BIT(TIMER_CR2_CCDS_BIT)
#define TIMER_CR2_CCUS                  BIT(TIMER_CR2_CCUS_BIT)
#define TIMER_CR2_CCPC                  BIT(TIMER_CR2_CCPC_BIT)

/* Slave mode control register (SMCR) */

#define TIMER_SMCR_ETP_BIT              15
#define TIMER_SMCR_ECE_BIT              14
#define TIMER_SMCR_MSM_BIT              7

#define TIMER_SMCR_ETP                  BIT(TIMER_SMCR_ETP_BIT)
#define TIMER_SMCR_ECE                  BIT(TIMER_SMCR_ECE_BIT)
#define TIMER_SMCR_ETPS                 (0x3 << 12)
#define TIMER_SMCR_ETPS_OFF             (0x0 << 12)
#define TIMER_SMCR_ETPS_DIV2            (0x1 << 12)
#define TIMER_SMCR_ETPS_DIV4            (0x2 << 12)
#define TIMER_SMCR_ETPS_DIV8            (0x3 << 12)
#define TIMER_SMCR_ETF                  (0xF << 12)
#define TIMER_SMCR_MSM                  BIT(TIMER_SMCR_MSM_BIT)
#define TIMER_SMCR_TS                   (0x3 << 4)
#define TIMER_SMCR_TS_ITR0              (0x0 << 4)
#define TIMER_SMCR_TS_ITR1              (0x1 << 4)
#define TIMER_SMCR_TS_ITR2              (0x2 << 4)
#define TIMER_SMCR_TS_ITR3              (0x3 << 4)
#define TIMER_SMCR_TS_TI1F_ED           (0x4 << 4)
#define TIMER_SMCR_TS_TI1FP1            (0x5 << 4)
#define TIMER_SMCR_TS_TI2FP2            (0x6 << 4)
#define TIMER_SMCR_TS_ETRF              (0x7 << 4)
#define TIMER_SMCR_SMS                  0x3
#define TIMER_SMCR_SMS_DISABLED         0x0
#define TIMER_SMCR_SMS_ENCODER1         0x1
#define TIMER_SMCR_SMS_ENCODER2         0x2
#define TIMER_SMCR_SMS_ENCODER3         0x3
#define TIMER_SMCR_SMS_RESET            0x4
#define TIMER_SMCR_SMS_GATED            0x5
#define TIMER_SMCR_SMS_TRIGGER          0x6
#define TIMER_SMCR_SMS_EXTERNAL         0x7

/* DMA/Interrupt enable register (DIER) */

#define TIMER_DIER_TDE_BIT              14
#define TIMER_DIER_CC4DE_BIT            12
#define TIMER_DIER_CC3DE_BIT            11
#define TIMER_DIER_CC2DE_BIT            10
#define TIMER_DIER_CC1DE_BIT            9
#define TIMER_DIER_UDE_BIT              8
#define TIMER_DIER_TIE_BIT              6
#define TIMER_DIER_CC4IE_BIT            4
#define TIMER_DIER_CC3IE_BIT            3
#define TIMER_DIER_CC2IE_BIT            2
#define TIMER_DIER_CC1IE_BIT            1
#define TIMER_DIER_UIE_BIT              0

#define TIMER_DIER_TDE                  BIT(TIMER_DIER_TDE_BIT)
#define TIMER_DIER_CC4DE                BIT(TIMER_DIER_CC4DE_BIT)
#define TIMER_DIER_CC3DE                BIT(TIMER_DIER_CC3DE_BIT)
#define TIMER_DIER_CC2DE                BIT(TIMER_DIER_CC2DE_BIT)
#define TIMER_DIER_CC1DE                BIT(TIMER_DIER_CC1DE_BIT)
#define TIMER_DIER_UDE                  BIT(TIMER_DIER_UDE_BIT)
#define TIMER_DIER_TIE                  BIT(TIMER_DIER_TIE_BIT)
#define TIMER_DIER_CC4IE                BIT(TIMER_DIER_CC4IE_BIT)
#define TIMER_DIER_CC3IE                BIT(TIMER_DIER_CC3IE_BIT)
#define TIMER_DIER_CC2IE                BIT(TIMER_DIER_CC2IE_BIT)
#define TIMER_DIER_CC1IE                BIT(TIMER_DIER_CC1IE_BIT)
#define TIMER_DIER_UIE                  BIT(TIMER_DIER_UIE_BIT)

/* Status register (SR) */

#define TIMER_SR_CC4OF_BIT              12
#define TIMER_SR_CC3OF_BIT              11
#define TIMER_SR_CC2OF_BIT              10
#define TIMER_SR_CC1OF_BIT              9
#define TIMER_SR_BIF_BIT                7
#define TIMER_SR_TIF_BIT                6
#define TIMER_SR_COMIF_BIT              5
#define TIMER_SR_CC4IF_BIT              4
#define TIMER_SR_CC3IF_BIT              3
#define TIMER_SR_CC2IF_BIT              2
#define TIMER_SR_CC1IF_BIT              1
#define TIMER_SR_UIF_BIT                0

#define TIMER_SR_CC4OF                  BIT(TIMER_SR_CC4OF_BIT)
#define TIMER_SR_CC3OF                  BIT(TIMER_SR_CC3OF_BIT)
#define TIMER_SR_CC2OF                  BIT(TIMER_SR_CC2OF_BIT)
#define TIMER_SR_CC1OF                  BIT(TIMER_SR_CC1OF_BIT)
#define TIMER_SR_BIF                    BIT(TIMER_SR_BIF_BIT)
#define TIMER_SR_TIF                    BIT(TIMER_SR_TIF_BIT)
#define TIMER_SR_COMIF                  BIT(TIMER_SR_COMIF_BIT)
#define TIMER_SR_CC4IF                  BIT(TIMER_SR_CC4IF_BIT)
#define TIMER_SR_CC3IF                  BIT(TIMER_SR_CC3IF_BIT)
#define TIMER_SR_CC2IF                  BIT(TIMER_SR_CC2IF_BIT)
#define TIMER_SR_CC1IF                  BIT(TIMER_SR_CC1IF_BIT)
#define TIMER_SR_UIF                    BIT(TIMER_SR_UIF_BIT)

/* Event generation register (EGR) */

#define TIMER_EGR_TG_BIT                6
#define TIMER_EGR_CC4G_BIT              4
#define TIMER_EGR_CC3G_BIT              3
#define TIMER_EGR_CC2G_BIT              2
#define TIMER_EGR_CC1G_BIT              1
#define TIMER_EGR_UG_BIT                0

#define TIMER_EGR_TG                    BIT(TIMER_EGR_TG_BIT)
#define TIMER_EGR_CC4G                  BIT(TIMER_EGR_CC4G_BIT)
#define TIMER_EGR_CC3G                  BIT(TIMER_EGR_CC3G_BIT)
#define TIMER_EGR_CC2G                  BIT(TIMER_EGR_CC2G_BIT)
#define TIMER_EGR_CC1G                  BIT(TIMER_EGR_CC1G_BIT)
#define TIMER_EGR_UG                    BIT(TIMER_EGR_UG_BIT)

/* Capture/compare mode registers, common values */

#define TIMER_CCMR_CCS_OUTPUT           0x0
#define TIMER_CCMR_CCS_INPUT_TI1        0x1
#define TIMER_CCMR_CCS_INPUT_TI2        0x2
#define TIMER_CCMR_CCS_INPUT_TRC        0x3

/* Capture/compare mode register 1 (CCMR1) */

#define TIMER_CCMR1_OC2CE_BIT           15
#define TIMER_CCMR1_OC2PE_BIT           11
#define TIMER_CCMR1_OC2FE_BIT           10
#define TIMER_CCMR1_OC1CE_BIT           7
#define TIMER_CCMR1_OC1PE_BIT           3
#define TIMER_CCMR1_OC1FE_BIT           2

#define TIMER_CCMR1_OC2CE               BIT(TIMER_CCMR1_OC2CE_BIT)
#define TIMER_CCMR1_OC2M                (0x3 << 12)
#define TIMER_CCMR1_IC2F                (0xF << 12)
#define TIMER_CCMR1_OC2PE               BIT(TIMER_CCMR1_OC2PE_BIT)
#define TIMER_CCMR1_OC2FE               BIT(TIMER_CCMR1_OC2FE_BIT)
#define TIMER_CCMR1_IC2PSC              (0x3 << 10)
#define TIMER_CCMR1_CC2S                (0x3 << 8)
#define TIMER_CCMR1_CC2S_OUTPUT         (TIMER_CCMR_CCS_OUTPUT << 8)
#define TIMER_CCMR1_CC2S_INPUT_TI1      (TIMER_CCMR_CCS_INPUT_TI1 << 8)
#define TIMER_CCMR1_CC2S_INPUT_TI2      (TIMER_CCMR_CCS_INPUT_TI2 << 8)
#define TIMER_CCMR1_CC2S_INPUT_TRC      (TIMER_CCMR_CCS_INPUT_TRC << 8)
#define TIMER_CCMR1_OC1CE               BIT(TIMER_CCMR1_OC1CE_BIT)
#define TIMER_CCMR1_OC1M                (0x3 << 4)
#define TIMER_CCMR1_IC1F                (0xF << 4)
#define TIMER_CCMR1_OC1PE               BIT(TIMER_CCMR1_OC1PE_BIT)
#define TIMER_CCMR1_OC1FE               BIT(TIMER_CCMR1_OC1FE_BIT)
#define TIMER_CCMR1_IC1PSC              (0x3 << 2)
#define TIMER_CCMR1_CC1S                0x3
#define TIMER_CCMR1_CC1S_OUTPUT         TIMER_CCMR_CCS_OUTPUT
#define TIMER_CCMR1_CC1S_INPUT_TI1      TIMER_CCMR_CCS_INPUT_TI1
#define TIMER_CCMR1_CC1S_INPUT_TI2      TIMER_CCMR_CCS_INPUT_TI2
#define TIMER_CCMR1_CC1S_INPUT_TRC      TIMER_CCMR_CCS_INPUT_TRC

/* Capture/compare mode register 2 (CCMR2) */

#define TIMER_CCMR2_OC4CE_BIT           15
#define TIMER_CCMR2_OC4PE_BIT           11
#define TIMER_CCMR2_OC4FE_BIT           10
#define TIMER_CCMR2_OC3CE_BIT           7
#define TIMER_CCMR2_OC3PE_BIT           3
#define TIMER_CCMR2_OC3FE_BIT           2

#define TIMER_CCMR2_OC4CE               BIT(TIMER_CCMR2_OC4CE_BIT)
#define TIMER_CCMR2_OC4M                (0x3 << 12)
#define TIMER_CCMR2_IC2F                (0xF << 12)
#define TIMER_CCMR2_OC4PE               BIT(TIMER_CCMR2_OC4PE_BIT)
#define TIMER_CCMR2_OC4FE               BIT(TIMER_CCMR2_OC4FE_BIT)
#define TIMER_CCMR2_IC2PSC              (0x3 << 10)
#define TIMER_CCMR2_CC4S                (0x3 << 8)
#define TIMER_CCMR1_CC4S_OUTPUT         (TIMER_CCMR_CCS_OUTPUT << 8)
#define TIMER_CCMR1_CC4S_INPUT_TI1      (TIMER_CCMR_CCS_INPUT_TI1 << 8)
#define TIMER_CCMR1_CC4S_INPUT_TI2      (TIMER_CCMR_CCS_INPUT_TI2 << 8)
#define TIMER_CCMR1_CC4S_INPUT_TRC      (TIMER_CCMR_CCS_INPUT_TRC << 8)
#define TIMER_CCMR2_OC3CE               BIT(TIMER_CCMR2_OC3CE_BIT)
#define TIMER_CCMR2_OC3M                (0x3 << 4)
#define TIMER_CCMR2_IC1F                (0xF << 4)
#define TIMER_CCMR2_OC3PE               BIT(TIMER_CCMR2_OC3PE_BIT)
#define TIMER_CCMR2_OC3FE               BIT(TIMER_CCMR2_OC3FE_BIT)
#define TIMER_CCMR2_IC1PSC              (0x3 << 2)
#define TIMER_CCMR2_CC3S                0x3
#define TIMER_CCMR1_CC3S_OUTPUT         TIMER_CCMR_CCS_OUTPUT
#define TIMER_CCMR1_CC3S_INPUT_TI1      TIMER_CCMR_CCS_INPUT_TI1
#define TIMER_CCMR1_CC3S_INPUT_TI2      TIMER_CCMR_CCS_INPUT_TI2
#define TIMER_CCMR1_CC3S_INPUT_TRC      TIMER_CCMR_CCS_INPUT_TRC

/* Capture/compare enable register (CCER) */

#define TIMER_CCER_CC4P_BIT             13
#define TIMER_CCER_CC4E_BIT             12
#define TIMER_CCER_CC3P_BIT             9
#define TIMER_CCER_CC3E_BIT             8
#define TIMER_CCER_CC2P_BIT             5
#define TIMER_CCER_CC2E_BIT             4
#define TIMER_CCER_CC1P_BIT             1
#define TIMER_CCER_CC1E_BIT             0

#define TIMER_CCER_CC4P                 BIT(TIMER_CCER_CC4P_BIT)
#define TIMER_CCER_CC4E                 BIT(TIMER_CCER_CC4E_BIT)
#define TIMER_CCER_CC3P                 BIT(TIMER_CCER_CC3P_BIT)
#define TIMER_CCER_CC3E                 BIT(TIMER_CCER_CC3E_BIT)
#define TIMER_CCER_CC2P                 BIT(TIMER_CCER_CC2P_BIT)
#define TIMER_CCER_CC2E                 BIT(TIMER_CCER_CC2E_BIT)
#define TIMER_CCER_CC1P                 BIT(TIMER_CCER_CC1P_BIT)
#define TIMER_CCER_CC1E                 BIT(TIMER_CCER_CC1E_BIT)

/* Break and dead-time register (BDTR) */

#define TIMER_BDTR_MOE_BIT              15
#define TIMER_BDTR_AOE_BIT              14
#define TIMER_BDTR_BKP_BIT              13
#define TIMER_BDTR_BKE_BIT              12
#define TIMER_BDTR_OSSR_BIT             11
#define TIMER_BDTR_OSSI_BIT             10

#define TIMER_BDTR_MOE                  BIT(TIMER_BDTR_MOE_BIT)
#define TIMER_BDTR_AOE                  BIT(TIMER_BDTR_AOE_BIT)
#define TIMER_BDTR_BKP                  BIT(TIMER_BDTR_BKP_BIT)
#define TIMER_BDTR_BKE                  BIT(TIMER_BDTR_BKE_BIT)
#define TIMER_BDTR_OSSR                 BIT(TIMER_BDTR_OSSR_BIT)
#define TIMER_BDTR_OSSI                 BIT(TIMER_BDTR_OSSI_BIT)
#define TIMER_BDTR_LOCK                 (0x3 << 8)
#define TIMER_BDTR_LOCK_OFF             (0x0 << 8)
#define TIMER_BDTR_LOCK_LEVEL1          (0x1 << 8)
#define TIMER_BDTR_LOCK_LEVEL2          (0x2 << 8)
#define TIMER_BDTR_LOCK_LEVEL3          (0x3 << 8)
#define TIMER_BDTR_DTG                  0xFF

/* DMA control register (DCR) */

#define TIMER_DCR_DBL                   (0x1F << 8)
#define TIMER_DCR_DBL_1BYTE             (0x0 << 8)
#define TIMER_DCR_DBL_2BYTE             (0x1 << 8)
#define TIMER_DCR_DBL_3BYTE             (0x2 << 8)
#define TIMER_DCR_DBL_4BYTE             (0x3 << 8)
#define TIMER_DCR_DBL_5BYTE             (0x4 << 8)
#define TIMER_DCR_DBL_6BYTE             (0x5 << 8)
#define TIMER_DCR_DBL_7BYTE             (0x6 << 8)
#define TIMER_DCR_DBL_8BYTE             (0x7 << 8)
#define TIMER_DCR_DBL_9BYTE             (0x8 << 8)
#define TIMER_DCR_DBL_10BYTE            (0x9 << 8)
#define TIMER_DCR_DBL_11BYTE            (0xA << 8)
#define TIMER_DCR_DBL_12BYTE            (0xB << 8)
#define TIMER_DCR_DBL_13BYTE            (0xC << 8)
#define TIMER_DCR_DBL_14BYTE            (0xD << 8)
#define TIMER_DCR_DBL_15BYTE            (0xE << 8)
#define TIMER_DCR_DBL_16BYTE            (0xF << 8)
#define TIMER_DCR_DBL_17BYTE            (0x10 << 8)
#define TIMER_DCR_DBL_18BYTE            (0x11 << 8)
#define TIMER_DCR_DBA                   0x1F
#define TIMER_DCR_DBA_CR1               0x0
#define TIMER_DCR_DBA_CR2               0x1
#define TIMER_DCR_DBA_SMCR              0x2
#define TIMER_DCR_DBA_DIER              0x3
#define TIMER_DCR_DBA_SR                0x4
#define TIMER_DCR_DBA_EGR               0x5
#define TIMER_DCR_DBA_CCMR1             0x6
#define TIMER_DCR_DBA_CCMR2             0x7
#define TIMER_DCR_DBA_CCER              0x8
#define TIMER_DCR_DBA_CNT               0x9
#define TIMER_DCR_DBA_PSC               0xA
#define TIMER_DCR_DBA_ARR               0xB
#define TIMER_DCR_DBA_RCR               0xC
#define TIMER_DCR_DBA_CCR1              0xD
#define TIMER_DCR_DBA_CCR2              0xE
#define TIMER_DCR_DBA_CCR3              0xF
#define TIMER_DCR_DBA_CCR4              0x10
#define TIMER_DCR_DBA_BDTR              0x11
#define TIMER_DCR_DBA_DCR               0x12
#define TIMER_DCR_DBA_DMAR              0x13

/*
 * Convenience routines
 */

/**
 * Used to configure the behavior of a timer channel.  Note that not
 * all timers can be configured in every mode.
 */
/* TODO TIMER_PWM_CENTER_ALIGNED, TIMER_INPUT_CAPTURE, TIMER_ONE_PULSE */
typedef enum timer_mode {
    TIMER_DISABLED, /**< In this mode, the timer stops counting,
                         channel interrupts are detached, and no state
                         changes are output. */
    TIMER_PWM, /**< PWM output mode. This is the default mode for pins
                    after initialization. */
    /* TIMER_PWM_CENTER_ALIGNED, /\**< Center-aligned PWM output mode. *\/ */
    TIMER_OUTPUT_COMPARE, /**< In this mode, the timer counts from 0
                               to its reload value repeatedly; every
                               time the counter value reaches one of
                               the channel compare values, the
                               corresponding interrupt is fired. */
    /* TIMER_INPUT_CAPTURE, /\**< In this mode, the timer can measure the */
    /*                           pulse lengths of input signals. *\/ */
    /* TIMER_ONE_PULSE /\**< In this mode, the timer can generate a single */
    /*                      pulse on a GPIO pin for a specified amount of */
    /*                      time. *\/ */
} timer_mode;

/** Timer channel numbers */
typedef enum timer_channel {
    TIMER_CH1 = 1, /**< Channel 1 */
    TIMER_CH2 = 2, /**< Channel 2 */
    TIMER_CH3 = 3, /**< Channel 3 */
    TIMER_CH4 = 4  /**< Channel 4 */
} timer_channel;

/*
 * Note: Don't require timer_channel arguments! We want to be able to say
 *
 * for (int channel = 1; channel <= 4; channel++) {
 *    ...
 * }
 *
 * without the compiler yelling at us.
 */

void timer_init(timer_dev *dev);
void timer_disable(timer_dev *dev);
void timer_set_mode(timer_dev *dev, uint8 channel, timer_mode mode);
void timer_foreach(void (*fn)(timer_dev*));

/**
 * @brief Timer interrupt number.
 *
 * Not all timers support all of these values; see the descriptions
 * for each value.
 */
typedef enum timer_interrupt_id {
    TIMER_UPDATE_INTERRUPT, /**< Update interrupt, available on all timers. */
    TIMER_CC1_INTERRUPT, /**< Capture/compare 1 interrupt, available
                              on general and advanced timers only. */
    TIMER_CC2_INTERRUPT, /**< Capture/compare 2 interrupt, general and
                              advanced timers only. */
    TIMER_CC3_INTERRUPT, /**< Capture/compare 3 interrupt, general and
                              advanced timers only. */
    TIMER_CC4_INTERRUPT, /**< Capture/compare 4 interrupt, general and
                              advanced timers only. */
    TIMER_COM_INTERRUPT, /**< COM interrupt, advanced timers only */
    TIMER_TRG_INTERRUPT, /**< Trigger interrupt, general and advanced
                              timers only */
    TIMER_BREAK_INTERRUPT /**< Break interrupt, advanced timers only. */
} timer_interrupt_id;

void timer_attach_interrupt(timer_dev *dev,
                            uint8 interrupt,
                            voidFuncPtr handler);
void timer_detach_interrupt(timer_dev *dev, uint8 interrupt);

/**
 * Initialize all timer devices on the chip.
 */
static inline void timer_init_all(void) {
    timer_foreach(timer_init);
}

/**
 * Disables all timers on the device.
 */
static inline void timer_disable_all(void) {
    timer_foreach(timer_disable);
}

/**
 * @brief Stop a timer's counter from changing.
 *
 * Does not affect the timer's mode or other settings.
 *
 * @param dev Device whose counter to pause.
 */
static inline void timer_pause(timer_dev *dev) {
    *bb_perip(&(dev->regs).bas->CR1, TIMER_CR1_CEN_BIT) = 0;
}

/**
 * @brief Start a timer's counter.
 *
 * Does not affect the timer's mode or other settings.
 *
 * @param dev Device whose counter to resume
 */
static inline void timer_resume(timer_dev *dev) {
    *bb_perip(&(dev->regs).bas->CR1, TIMER_CR1_CEN_BIT) = 1;
}

/**
 * @brief Returns the timer's counter value.
 *
 * This value is likely to be inaccurate if the counter is running
 * with a low prescaler.
 *
 * @param dev Timer whose counter to return
 */
static inline uint16 timer_get_count(timer_dev *dev) {
    return (uint16)(dev->regs).bas->CNT;
}

/**
 * @brief Sets the counter value for the given timer.
 * @param dev Timer whose counter to set
 * @param value New counter value
 */
static inline void timer_set_count(timer_dev *dev, uint16 value) {
    (dev->regs).bas->CNT = value;
}

/**
 * @brief Returns the given timer's prescaler.
 *
 * Note that if the timer's prescaler is set (e.g. via
 * timer_set_prescaler() or accessing a TIMx_PSC register), the value
 * returned by this function will reflect the new setting, but the
 * timer's counter will only reflect the new prescaler at the next
 * update event.
 *
 * @param dev Timer whose prescaler to return
 * @see timer_generate_update()
 */
static inline uint16 timer_get_prescaler(timer_dev *dev) {
    return (uint16)(dev->regs).bas->PSC;
}

/**
 * @brief Set a timer's prescale value.
 *
 * Divides the input clock by (PSC+1).  The new value will not take
 * effect until the next update event.
 *
 * @param dev Timer whose prescaler to set
 * @param psc New prescaler value
 * @see timer_generate_update()
 */
static inline void timer_set_prescaler(timer_dev *dev, uint16 psc) {
    (dev->regs).bas->PSC = psc;
}

/**
 * @brief Returns a timer's reload value.
 * @param dev Timer whose reload value to return
 */
static inline uint16 timer_get_reload(timer_dev *dev) {
    return (uint16)(dev->regs).bas->ARR;
}

/**
 * @brief Set a timer's reload value.
 * @param dev Timer whose reload value to set
 * @param arr New reload value to use.  Takes effect at next update event.
 * @see timer_generate_update()
 */
static inline void timer_set_reload(timer_dev *dev, uint16 arr) {
    (dev->regs).bas->ARR = arr;
}

/**
 * @brief Get the compare value for the given timer channel.
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @param channel Channel whose compare value to get.
 */
static inline uint16 timer_get_compare(timer_dev *dev, uint8 channel) {
    __io uint32 *ccr = &(dev->regs).gen->CCR1 + (channel - 1);
    return *ccr;
}

/**
 * @brief Set the compare value for the given timer channel.
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @param channel Channel whose compare value to set.
 * @param value   New compare value.
 */
static inline void timer_set_compare(timer_dev *dev,
                                     uint8 channel,
                                     uint16 value) {
    __io uint32 *ccr = &(dev->regs).gen->CCR1 + (channel - 1);
    *ccr = value;
}

/**
 * @brief Generate an update event for the given timer.
 *
 * Normally, this will cause the prescaler and auto-reload values in
 * the PSC and ARR registers to take immediate effect.  However, this
 * function will do nothing if the UDIS bit is set in the timer's CR1
 * register (UDIS is cleared by default).
 *
 * @param dev Timer device to generate an update for.
 */
static inline void timer_generate_update(timer_dev *dev) {
    *bb_perip(&(dev->regs).bas->EGR, TIMER_EGR_UG_BIT) = 1;
}

/**
 * @brief Enable a timer's trigger DMA request
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL
 */
static inline void timer_dma_enable_trg_req(timer_dev *dev) {
    *bb_perip(&(dev->regs).gen->DIER, TIMER_DIER_TDE_BIT) = 1;
}

/**
 * @brief Disable a timer's trigger DMA request
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL
 */
static inline void timer_dma_disable_trg_req(timer_dev *dev) {
    *bb_perip(&(dev->regs).gen->DIER, TIMER_DIER_TDE_BIT) = 0;
}

/**
 * @brief Enable a timer channel's DMA request.
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL
 * @param channel Channel whose DMA request to enable.
 */
static inline void timer_dma_enable_req(timer_dev *dev, uint8 channel) {
    *bb_perip(&(dev->regs).gen->DIER, channel + 8) = 1;
}

/**
 * @brief Disable a timer channel's DMA request.
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @param channel Channel whose DMA request to disable.
 */
static inline void timer_dma_disable_req(timer_dev *dev, uint8 channel) {
    *bb_perip(&(dev->regs).gen->DIER, channel + 8) = 0;
}

/**
 * @brief Enable a timer interrupt.
 * @param dev Timer device.
 * @param interrupt Interrupt number to enable; this may be any
 *                  timer_interrupt_id value appropriate for the timer.
 * @see timer_interrupt_id
 * @see timer_channel
 */
static inline void timer_enable_irq(timer_dev *dev, uint8 interrupt) {
    *bb_perip(&(dev->regs).adv->DIER, interrupt) = 1;
}

/**
 * @brief Disable a timer interrupt.
 * @param dev Timer device.
 * @param interrupt Interrupt number to disable; this may be any
 *                  timer_interrupt_id value appropriate for the timer.
 * @see timer_interrupt_id
 * @see timer_channel
 */
static inline void timer_disable_irq(timer_dev *dev, uint8 interrupt) {
    *bb_perip(&(dev->regs).adv->DIER, interrupt) = 0;
}

/**
 * @brief Enable a timer channel's capture/compare signal.
 *
 * If the channel is configured as output, the corresponding output
 * compare signal will be output on the corresponding output pin.  If
 * the channel is configured as input, enables capture of the counter
 * value into the input capture/compare register.
 *
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @param channel Channel to enable, from 1 to 4.
 */
static inline void timer_cc_enable(timer_dev *dev, uint8 channel) {
    *bb_perip(&(dev->regs).gen->CCER, 4 * (channel - 1)) = 1;
}

/**
 * @brief Disable a timer channel's output compare or input capture signal.
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @param channel Channel to disable, from 1 to 4.
 * @see timer_cc_enable()
 */
static inline void timer_cc_disable(timer_dev *dev, uint8 channel) {
    *bb_perip(&(dev->regs).gen->CCER, 4 * (channel - 1)) = 0;
}

/**
 * @brief Get a channel's capture/compare output polarity
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @param channel Channel whose capture/compare output polarity to get.
 * @return Polarity, either 0 or 1.
 * @see timer_cc_set_polarity()
 */
static inline uint8 timer_cc_get_pol(timer_dev *dev, uint8 channel) {
    return *bb_perip(&(dev->regs).gen->CCER, 4 * (channel - 1) + 1);
}

/**
 * @brief Set a timer channel's capture/compare output polarity.
 *
 * If the timer channel is configured as output: polarity == 0 means
 * the output channel will be active high; polarity == 1 means active
 * low.
 *
 * If the timer channel is configured as input: polarity == 0 means
 * capture is done on the rising edge of ICn; when used as an external
 * trigger, ICn is non-inverted.  polarity == 1 means capture is done
 * on the falling edge of ICn; when used as an external trigger, ICn
 * is inverted.
 *
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @param channel Channel whose capture/compare output polarity to set.
 * @param pol New polarity, 0 or 1.
 */
static inline void timer_cc_set_pol(timer_dev *dev, uint8 channel, uint8 pol) {
    *bb_perip(&(dev->regs).gen->CCER, 4 * (channel - 1) + 1) = pol;
}

/**
 * @brief Get a timer's DMA burst length.
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @return Number of bytes to be transferred per DMA request, from 1 to 18.
 */
static inline uint8 timer_dma_get_burst_len(timer_dev *dev) {
    uint32 dbl = ((dev->regs).gen->DCR & TIMER_DCR_DBL) >> 8;
    return dbl + 1;             /* 0 means 1 byte, etc. */
}

/**
 * @brief Set a timer's DMA burst length.
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @param length DMA burst length; i.e., number of bytes to transfer
 *               per DMA request, from 1 to 18.
 */
static inline void timer_dma_set_burst_len(timer_dev *dev, uint8 length) {
    uint32 tmp = (dev->regs).gen->DCR;
    tmp &= ~TIMER_DCR_DBL;
    tmp |= (length - 1) << 8;
    (dev->regs).gen->DCR = tmp;
}

/**
 * @brief Timer DMA base address.
 *
 * Defines the base address for DMA transfers.
 */
typedef enum timer_dma_base_addr {
    TIMER_DMA_BASE_CR1 = TIMER_DCR_DBA_CR1, /**< Base is control register 1 */
    TIMER_DMA_BASE_CR2 = TIMER_DCR_DBA_CR2, /**< Base is control register 2 */
    TIMER_DMA_BASE_SMCR = TIMER_DCR_DBA_SMCR, /**< Base is slave mode
                                                   control register */
    TIMER_DMA_BASE_DIER = TIMER_DCR_DBA_DIER, /**< Base is DMA interrupt enable
                                                   register */
    TIMER_DMA_BASE_SR = TIMER_DCR_DBA_SR, /**< Base is status register */
    TIMER_DMA_BASE_EGR = TIMER_DCR_DBA_EGR, /**< Base is event generation
                                                 register */
    TIMER_DMA_BASE_CCMR1 = TIMER_DCR_DBA_CCMR1, /**< Base is capture/compare
                                                     mode register 1 */
    TIMER_DMA_BASE_CCMR2 = TIMER_DCR_DBA_CCMR2, /**< Base is capture/compare
                                                     mode register 2 */
    TIMER_DMA_BASE_CCER = TIMER_DCR_DBA_CCER,   /**< Base is capture/compare
                                                     enable register */
    TIMER_DMA_BASE_CNT = TIMER_DCR_DBA_CNT,     /**< Base is counter */
    TIMER_DMA_BASE_PSC = TIMER_DCR_DBA_PSC,     /**< Base is prescaler */
    TIMER_DMA_BASE_ARR = TIMER_DCR_DBA_ARR,     /**< Base is auto-reload
                                                     register */
    TIMER_DMA_BASE_RCR = TIMER_DCR_DBA_RCR,     /**< Base is repetition
                                                     counter register */
    TIMER_DMA_BASE_CCR1 = TIMER_DCR_DBA_CCR1,   /**< Base is capture/compare
                                                     register 1 */
    TIMER_DMA_BASE_CCR2 = TIMER_DCR_DBA_CCR2,   /**< Base is capture/compare
                                                     register 2 */
    TIMER_DMA_BASE_CCR3 = TIMER_DCR_DBA_CCR3,   /**< Base is capture/compare
                                                     register 3 */
    TIMER_DMA_BASE_CCR4 = TIMER_DCR_DBA_CCR4,   /**< Base is capture/compare
                                                     register 4 */
    TIMER_DMA_BASE_BDTR = TIMER_DCR_DBA_BDTR,   /**< Base is break and
                                                     dead-time register */
    TIMER_DMA_BASE_DCR = TIMER_DCR_DBA_DCR,     /**< Base is DMA control
                                                     register */
    TIMER_DMA_BASE_DMAR = TIMER_DCR_DBA_DMAR    /**< Base is DMA address for
                                                     full transfer */
} timer_dma_base_addr;

/**
 * @brief Get the timer's DMA base address.
 *
 * Some restrictions apply; see ST RM0008.
 *
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @return DMA base address
 */
static inline timer_dma_base_addr timer_dma_get_base_addr(timer_dev *dev) {
    uint32 dcr = (dev->regs).gen->DCR;
    return (timer_dma_base_addr)(dcr & TIMER_DCR_DBA);
}

/**
 * @brief Set the timer's DMA base address.
 *
 * Some restrictions apply; see ST RM0008.
 *
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @param dma_base DMA base address.
 */
static inline void timer_dma_set_base_addr(timer_dev *dev,
                                           timer_dma_base_addr dma_base) {
    uint32 tmp = (dev->regs).gen->DCR;
    tmp &= ~TIMER_DCR_DBA;
    tmp |= dma_base;
    (dev->regs).gen->DCR = tmp;
}

/**
 * Timer output compare modes.
 */
typedef enum timer_oc_mode {
    TIMER_OC_MODE_FROZEN = 0 << 4, /**< Frozen: comparison between output
                                      compare register and counter has no
                                      effect on the outputs. */
    TIMER_OC_MODE_ACTIVE_ON_MATCH = 1 << 4, /**< OCxREF signal is forced
                                               high when the count matches
                                               the channel capture/compare
                                               register. */
    TIMER_OC_MODE_INACTIVE_ON_MATCH = 2 << 4, /**< OCxREF signal is forced
                                                 low when the counter matches
                                                 the channel capture/compare
                                                 register. */
    TIMER_OC_MODE_TOGGLE = 3 << 4, /**< OCxREF toggles when counter
                                      matches the cannel capture/compare
                                      register. */
    TIMER_OC_MODE_FORCE_INACTIVE = 4 << 4, /**< OCxREF is forced low. */
    TIMER_OC_MODE_FORCE_ACTIVE = 5 << 4, /**< OCxREF is forced high. */
    TIMER_OC_MODE_PWM_1 = 6 << 4, /**< PWM mode 1.  In upcounting, channel is
                                     active as long as count is less than
                                     channel capture/compare register, else
                                     inactive.  In downcounting, channel is
                                     inactive as long as count exceeds
                                     capture/compare register, else
                                     active. */
    TIMER_OC_MODE_PWM_2 = 7 << 4  /**< PWM mode 2. In upcounting, channel is
                                     inactive as long as count is less than
                                     capture/compare register, else active.
                                     In downcounting, channel is active as
                                     long as count exceeds capture/compare
                                     register, else inactive. */
} timer_oc_mode;

/**
 * Timer output compare mode flags.
 * @see timer_oc_set_mode()
 */
typedef enum timer_oc_mode_flags {
    TIMER_OC_CE = BIT(7),       /**< Output compare clear enable. */
    TIMER_OC_PE = BIT(3),       /**< Output compare preload enable. */
    TIMER_OC_FE = BIT(2)        /**< Output compare fast enable. */
} timer_oc_mode_flags;

/**
 * @brief Configure a channel's output compare mode.
 *
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @param channel Channel to configure in output compare mode.
 * @param mode Timer mode to set.
 * @param flags OR of timer_oc_mode_flags.
 * @see timer_oc_mode
 * @see timer_oc_mode_flags
 */
static inline void timer_oc_set_mode(timer_dev *dev,
                                     uint8 channel,
                                     timer_oc_mode mode,
                                     uint8 flags) {
    uint8 bit0 = channel & 1;
    //uint8 bit1 = (channel >> 1) & 1;  // original
    uint8 bit1 = ((channel-1) >> 1) & 1;  // fixed
    /* channel == 1,2 -> CCMR1; channel == 3,4 -> CCMR2 */
    __io uint32 *ccmr = &(dev->regs).gen->CCMR1 + bit1;
    /* channel == 1,3 -> shift = 0, channel == 2,4 -> shift = 8 */
    uint8 shift = 8 * (1 - bit0);

    uint32 tmp = *ccmr;
    tmp &= ~(0xFF << shift);
    tmp |= (mode | flags | TIMER_CCMR_CCS_OUTPUT) << shift;
    *ccmr = tmp;
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif
