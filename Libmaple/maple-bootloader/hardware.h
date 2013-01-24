/* *****************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ****************************************************************************/

#ifndef __HARDWARE_H
#define __HARDWARE_H

#include "stm32f10x_type.h"
#include "cortexm3_macro.h"
#include "common.h"

/* macro'd register and peripheral definitions */
#ifdef STM32F2
#define PERIPH_BASE           ((u32)0x40000000) /*!< Peripheral base address in the alias region */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000)

/*!< AHB1 peripherals */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASE            (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASE            (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASE            (AHB1PERIPH_BASE + 0x2000)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00)


#define RCC   ((u32)RCC_BASE)
#define FLASH ((u32)FLASH_R_BASE)
#define GPIOA ((u32)GPIOA_BASE)
#define GPIOB ((u32)GPIOB_BASE)
#define GPIOC ((u32)GPIOC_BASE)
#define GPIOD ((u32)GPIOD_BASE)
#define GPIOE ((u32)GPIOE_BASE)
#define GPIOF ((u32)GPIOF_BASE)
#define GPIOG ((u32)GPIOG_BASE)
#else
#define RCC   ((u32)0x40021000)
#define FLASH ((u32)0x40022000)
#define GPIOA ((u32)0x40010800)
#define GPIOB ((u32)0x40010C00)
#define GPIOC ((u32)0x40011000)
#define GPIOD ((u32)0x40011400)
#define GPIOE ((u32)0x40011800)
#define GPIOF ((u32)0x40011C00)
#define GPIOG ((u32)0x40012000)

#define RCC_CR      (RCC)
#define RCC_CFGR    (RCC + 0x04)
#define RCC_CIR     (RCC + 0x08)
#define RCC_AHBENR  (RCC + 0x14)
#define RCC_APB2ENR (RCC + 0x18)
#define RCC_APB1ENR (RCC + 0x16)
#endif

#define FLASH_ACR     (FLASH + 0x00)
#define FLASH_KEYR    (FLASH + 0x04)
#define FLASH_OPTKEYR (FLASH + 0x08)
#define FLASH_SR      (FLASH + 0x0C)
#define FLASH_CR      (FLASH + 0x10)
#define FLASH_AR      (FLASH + 0x14)
#define FLASH_OBR     (FLASH + 0x1C)
#define FLASH_WRPR    (FLASH + 0x20)

#define FLASH_KEY1     0x45670123
#define FLASH_KEY2     0xCDEF89AB
#define FLASH_RDPRT    0x00A5
#define FLASH_SR_BSY   0x01
#define FLASH_CR_PER   0x02
#define FLASH_CR_PG    0x01
#define FLASH_CR_START 0x40

#define GPIO_CRL(port)  port
#define GPIO_CRH(port)  (port+0x04)
#define GPIO_IDR(port)  (port+0x08)
#define GPIO_ODR(port)  (port+0x0c)

#ifdef STM32F2
#define GPIO_BSRR(port) (port+0x18)
#else
#define GPIO_BSRR(port) (port+0x10)
#endif

#define SCS_BASE   ((u32)0xE000E000)
#define NVIC_BASE  (SCS_BASE + 0x0100)
#define SCB_BASE   (SCS_BASE + 0x0D00)


#define SCS      0xE000E000
#define NVIC     (SCS+0x100)
#define SCB      (SCS+0xD00)
#define STK      (SCS+0x10)

#define SCB_VTOR (SCB+0x08)
#define STK_CTRL (STK+0x00)

/*
#define TIM1_APB2_ENB ((u32)0x00000800)
#define TIM1          ((u32)0x40012C00)
#define TIM1_PSC      (TIM1+0x28)
#define TIM1_ARR      (TIM1+0x2C)
#define TIM1_RCR      (TIM1+0x30)
#define TIM1_CR1      (TIM1+0x00)
#define TIM1_CR2      (TIM1+0x04)
#define TIM1_DIER     (TIM1+0x0C)
#define TIM1_UP_IRQ_Channel ((u8)0x19)

#define TIM2_IRQ    ((u8)0x1C)
*/

#define USB_HP_IRQ  ((u8)0x13)
#define USB_LP_IRQ  ((u8)0x14)


/* AIRCR  */
#define AIRCR_RESET         0x05FA0000
#define AIRCR_RESET_REQ     (AIRCR_RESET | (u32)0x04);

/* temporary copyage of example from kiel */
#define __VAL(__TIMCLK, __PERIOD) ((__TIMCLK/1000000UL)*__PERIOD)
#define __PSC(__TIMCLK, __PERIOD)  (((__VAL(__TIMCLK, __PERIOD)+49999UL)/50000UL) - 1)
#define __ARR(__TIMCLK, __PERIOD) ((__VAL(__TIMCLK, __PERIOD)/(__PSC(__TIMCLK, __PERIOD)+1)) - 1)

#define SET_REG(addr,val) do { *(vu32*)(addr)=(val); } while(0)
#define GET_REG(addr)     (*(vu32*)(addr))

/* todo: there must be some major misunderstanding in how we access regs. The direct access approach (GET_REG)
   causes the usb init to fail upon trying to activate RCC_APB1 |= 0x00800000. However, using the struct approach
   from ST, it works fine...temporarily switching to that approach */
#ifdef STM32F2
/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N */
#define PLL_M      8
#define PLL_N      240

/* SYSCLK = PLL_VCO / PLL_P */
#define PLL_P      2

/* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
#define PLL_Q      5

#define uint16_t u16
#define uint32_t u32
#define HSE_STARTUP_TIMEOUT  ((uint16_t)0x0500)   /*!< Time out for HSE start up */

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define  RCC_CR_HSION                        ((uint32_t)0x00000001)
#define  RCC_CR_HSIRDY                       ((uint32_t)0x00000002)

#define  RCC_CR_HSITRIM                      ((uint32_t)0x000000F8)
#define  RCC_CR_HSITRIM_0                    ((uint32_t)0x00000008)/*!<Bit 0 */
#define  RCC_CR_HSITRIM_1                    ((uint32_t)0x00000010)/*!<Bit 1 */
#define  RCC_CR_HSITRIM_2                    ((uint32_t)0x00000020)/*!<Bit 2 */
#define  RCC_CR_HSITRIM_3                    ((uint32_t)0x00000040)/*!<Bit 3 */
#define  RCC_CR_HSITRIM_4                    ((uint32_t)0x00000080)/*!<Bit 4 */

#define  RCC_CR_HSICAL                       ((uint32_t)0x0000FF00)
#define  RCC_CR_HSICAL_0                     ((uint32_t)0x00000100)/*!<Bit 0 */
#define  RCC_CR_HSICAL_1                     ((uint32_t)0x00000200)/*!<Bit 1 */
#define  RCC_CR_HSICAL_2                     ((uint32_t)0x00000400)/*!<Bit 2 */
#define  RCC_CR_HSICAL_3                     ((uint32_t)0x00000800)/*!<Bit 3 */
#define  RCC_CR_HSICAL_4                     ((uint32_t)0x00001000)/*!<Bit 4 */
#define  RCC_CR_HSICAL_5                     ((uint32_t)0x00002000)/*!<Bit 5 */
#define  RCC_CR_HSICAL_6                     ((uint32_t)0x00004000)/*!<Bit 6 */
#define  RCC_CR_HSICAL_7                     ((uint32_t)0x00008000)/*!<Bit 7 */

#define  RCC_CR_HSEON                        ((uint32_t)0x00010000)
#define  RCC_CR_HSERDY                       ((uint32_t)0x00020000)
#define  RCC_CR_HSEBYP                       ((uint32_t)0x00040000)
#define  RCC_CR_CSSON                        ((uint32_t)0x00080000)
#define  RCC_CR_PLLON                        ((uint32_t)0x01000000)
#define  RCC_CR_PLLRDY                       ((uint32_t)0x02000000)
#define  RCC_CR_PLLI2SON                     ((uint32_t)0x04000000)
#define  RCC_CR_PLLI2SRDY                    ((uint32_t)0x08000000)


/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define  RCC_CFGR_SW                         ((uint32_t)0x00000003)        /*!< SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  RCC_CFGR_SW_1                       ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  RCC_CFGR_SW_HSI                     ((uint32_t)0x00000000)        /*!< HSI selected as system clock */
#define  RCC_CFGR_SW_HSE                     ((uint32_t)0x00000001)        /*!< HSE selected as system clock */
#define  RCC_CFGR_SW_PLL                     ((uint32_t)0x00000002)        /*!< PLL selected as system clock */

/*!< SWS configuration */
#define  RCC_CFGR_SWS                        ((uint32_t)0x0000000C)        /*!< SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  RCC_CFGR_SWS_1                      ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  RCC_CFGR_SWS_HSI                    ((uint32_t)0x00000000)        /*!< HSI oscillator used as system clock */
#define  RCC_CFGR_SWS_HSE                    ((uint32_t)0x00000004)        /*!< HSE oscillator used as system clock */
#define  RCC_CFGR_SWS_PLL                    ((uint32_t)0x00000008)        /*!< PLL used as system clock */

/*!< HPRE configuration */
#define  RCC_CFGR_HPRE                       ((uint32_t)0x000000F0)        /*!< HPRE[3:0] bits (AHB prescaler) */
#define  RCC_CFGR_HPRE_0                     ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  RCC_CFGR_HPRE_1                     ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  RCC_CFGR_HPRE_2                     ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  RCC_CFGR_HPRE_3                     ((uint32_t)0x00000080)        /*!< Bit 3 */

#define  RCC_CFGR_HPRE_DIV1                  ((uint32_t)0x00000000)        /*!< SYSCLK not divided */
#define  RCC_CFGR_HPRE_DIV2                  ((uint32_t)0x00000080)        /*!< SYSCLK divided by 2 */
#define  RCC_CFGR_HPRE_DIV4                  ((uint32_t)0x00000090)        /*!< SYSCLK divided by 4 */
#define  RCC_CFGR_HPRE_DIV8                  ((uint32_t)0x000000A0)        /*!< SYSCLK divided by 8 */
#define  RCC_CFGR_HPRE_DIV16                 ((uint32_t)0x000000B0)        /*!< SYSCLK divided by 16 */
#define  RCC_CFGR_HPRE_DIV64                 ((uint32_t)0x000000C0)        /*!< SYSCLK divided by 64 */
#define  RCC_CFGR_HPRE_DIV128                ((uint32_t)0x000000D0)        /*!< SYSCLK divided by 128 */
#define  RCC_CFGR_HPRE_DIV256                ((uint32_t)0x000000E0)        /*!< SYSCLK divided by 256 */
#define  RCC_CFGR_HPRE_DIV512                ((uint32_t)0x000000F0)        /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define  RCC_CFGR_PPRE1                      ((uint32_t)0x00001C00)        /*!< PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_CFGR_PPRE1_0                    ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE1_1                    ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE1_2                    ((uint32_t)0x00001000)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE1_DIV1                 ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 ((uint32_t)0x00001000)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE1_DIV4                 ((uint32_t)0x00001400)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE1_DIV8                 ((uint32_t)0x00001800)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE1_DIV16                ((uint32_t)0x00001C00)        /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define  RCC_CFGR_PPRE2                      ((uint32_t)0x0000E000)        /*!< PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2_0                    ((uint32_t)0x00002000)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE2_1                    ((uint32_t)0x00004000)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE2_2                    ((uint32_t)0x00008000)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE2_DIV1                 ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE2_DIV2                 ((uint32_t)0x00008000)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE2_DIV4                 ((uint32_t)0x0000A000)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE2_DIV8                 ((uint32_t)0x0000C000)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE2_DIV16                ((uint32_t)0x0000E00)         /*!< HCLK divided by 16 */

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define  RCC_PLLCFGR_PLLM                    ((uint32_t)0x0000003F)
#define  RCC_PLLCFGR_PLLM_0                  ((uint32_t)0x00000001)
#define  RCC_PLLCFGR_PLLM_1                  ((uint32_t)0x00000002)
#define  RCC_PLLCFGR_PLLM_2                  ((uint32_t)0x00000004)
#define  RCC_PLLCFGR_PLLM_3                  ((uint32_t)0x00000008)
#define  RCC_PLLCFGR_PLLM_4                  ((uint32_t)0x00000010)
#define  RCC_PLLCFGR_PLLM_5                  ((uint32_t)0x00000020)

#define  RCC_PLLCFGR_PLLN                     ((uint32_t)0x00007FC0)
#define  RCC_PLLCFGR_PLLN_0                   ((uint32_t)0x00000040)
#define  RCC_PLLCFGR_PLLN_1                   ((uint32_t)0x00000080)
#define  RCC_PLLCFGR_PLLN_2                   ((uint32_t)0x00000100)
#define  RCC_PLLCFGR_PLLN_3                   ((uint32_t)0x00000200)
#define  RCC_PLLCFGR_PLLN_4                   ((uint32_t)0x00000400)
#define  RCC_PLLCFGR_PLLN_5                   ((uint32_t)0x00000800)
#define  RCC_PLLCFGR_PLLN_6                   ((uint32_t)0x00001000)
#define  RCC_PLLCFGR_PLLN_7                   ((uint32_t)0x00002000)
#define  RCC_PLLCFGR_PLLN_8                   ((uint32_t)0x00004000)

#define  RCC_PLLCFGR_PLLP                    ((uint32_t)0x00030000)
#define  RCC_PLLCFGR_PLLP_0                  ((uint32_t)0x00010000)
#define  RCC_PLLCFGR_PLLP_1                  ((uint32_t)0x00020000)

#define  RCC_PLLCFGR_PLLSRC                  ((uint32_t)0x00400000)
#define  RCC_PLLCFGR_PLLSRC_HSE              ((uint32_t)0x00400000)
#define  RCC_PLLCFGR_PLLSRC_HSI              ((uint32_t)0x00000000)

#define  RCC_PLLCFGR_PLLQ                    ((uint32_t)0x0F000000)
#define  RCC_PLLCFGR_PLLQ_0                  ((uint32_t)0x01000000)
#define  RCC_PLLCFGR_PLLQ_1                  ((uint32_t)0x02000000)
#define  RCC_PLLCFGR_PLLQ_2                  ((uint32_t)0x04000000)
#define  RCC_PLLCFGR_PLLQ_3                  ((uint32_t)0x08000000)



/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define  RCC_AHB1ENR_GPIOAEN                 ((uint32_t)0x00000001)
#define  RCC_AHB1ENR_GPIOBEN                 ((uint32_t)0x00000002)
#define  RCC_AHB1ENR_GPIOCEN                 ((uint32_t)0x00000004)
#define  RCC_AHB1ENR_GPIODEN                 ((uint32_t)0x00000008)
#define  RCC_AHB1ENR_GPIOEEN                 ((uint32_t)0x00000010)
#define  RCC_AHB1ENR_GPIOFEN                 ((uint32_t)0x00000020)
#define  RCC_AHB1ENR_GPIOGEN                 ((uint32_t)0x00000040)
#define  RCC_AHB1ENR_GPIOHEN                 ((uint32_t)0x00000080)
#define  RCC_AHB1ENR_GPIOIEN                 ((uint32_t)0x00000100)
#define  RCC_AHB1ENR_CRCEN                   ((uint32_t)0x00001000)
#define  RCC_AHB1ENR_BKPSRAMEN               ((uint32_t)0x00040000)
#define  RCC_AHB1ENR_DMA1EN                  ((uint32_t)0x00200000)
#define  RCC_AHB1ENR_DMA2EN                  ((uint32_t)0x00400000)
#define  RCC_AHB1ENR_ETHMACEN                ((uint32_t)0x02000000)
#define  RCC_AHB1ENR_ETHMACTXEN              ((uint32_t)0x04000000)
#define  RCC_AHB1ENR_ETHMACRXEN              ((uint32_t)0x08000000)
#define  RCC_AHB1ENR_ETHMACPTPEN             ((uint32_t)0x10000000)
#define  RCC_AHB1ENR_OTGHSEN                 ((uint32_t)0x20000000)
#define  RCC_AHB1ENR_OTGHSULPIEN             ((uint32_t)0x40000000)

/********************  Bit definition for RCC_AHB2ENR register  ***************/
#define  RCC_AHB2ENR_DCMIEN                  ((uint32_t)0x00000001)
#define  RCC_AHB2ENR_CRYPEN                  ((uint32_t)0x00000010)
#define  RCC_AHB2ENR_HASHEN                  ((uint32_t)0x00000020)
#define  RCC_AHB2ENR_RNGEN                   ((uint32_t)0x00000040)
#define  RCC_AHB2ENR_OTGFSEN                 ((uint32_t)0x00000080)



/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bits definition for FLASH_ACR register  *****************/
#define FLASH_ACR_LATENCY                    ((uint32_t)0x00000007)
#define FLASH_ACR_LATENCY_0WS                ((uint32_t)0x00000000)
#define FLASH_ACR_LATENCY_1WS                ((uint32_t)0x00000001)
#define FLASH_ACR_LATENCY_2WS                ((uint32_t)0x00000002)
#define FLASH_ACR_LATENCY_3WS                ((uint32_t)0x00000003)
#define FLASH_ACR_LATENCY_4WS                ((uint32_t)0x00000004)
#define FLASH_ACR_LATENCY_5WS                ((uint32_t)0x00000005)
#define FLASH_ACR_LATENCY_6WS                ((uint32_t)0x00000006)
#define FLASH_ACR_LATENCY_7WS                ((uint32_t)0x00000007)

#define FLASH_ACR_PRFTEN                     ((uint32_t)0x00000100)
#define FLASH_ACR_ICEN                       ((uint32_t)0x00000200)
#define FLASH_ACR_DCEN                       ((uint32_t)0x00000400)
#define FLASH_ACR_ICRST                      ((uint32_t)0x00000800)
#define FLASH_ACR_DCRST                      ((uint32_t)0x00001000)
#define FLASH_ACR_BYTE0_ADDRESS              ((uint32_t)0x40023C00)
#define FLASH_ACR_BYTE2_ADDRESS              ((uint32_t)0x40023C03)

/**
  * @brief FLASH Registers
  */

typedef struct
{
  vu32 ACR;      /*!< FLASH access control register, Address offset: 0x00 */
  vu32 KEYR;     /*!< FLASH key register,            Address offset: 0x04 */
  vu32 OPTKEYR;  /*!< FLASH option key register,     Address offset: 0x08 */
  vu32 SR;       /*!< FLASH status register,         Address offset: 0x0C */
  vu32 CR;       /*!< FLASH control register,        Address offset: 0x10 */
  vu32 OPTCR;    /*!< FLASH option control register, Address offset: 0x14 */
} FLASH_TypeDef;


/**
  * @brief General Purpose I/O
  */

typedef struct
{
  vu32 MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  vu32 OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  vu32 OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  vu32 PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  vu32 IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  vu32 ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  vu16 BSRRL;    /*!< GPIO port bit set/reset low register,  Address offset: 0x18      */
  vu16 BSRRH;    /*!< GPIO port bit set/reset high register, Address offset: 0x1A      */
  vu32 LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  vu32 AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x24-0x28 */
} GPIO_TypeDef;


typedef struct
{
  vu32 CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  vu32 PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  vu32 CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  vu32 CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  vu32 AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  vu32 AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  vu32 AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  u32  RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  vu32 APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  vu32 APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  u32  RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  vu32 AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  vu32 AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  vu32 AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  u32  RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  vu32 APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  vu32 APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  u32  RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  vu32 AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  vu32 AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  vu32 AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  u32  RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  vu32 APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  vu32 APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  u32  RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  vu32 BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  vu32 CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  u32  RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  vu32 SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  vu32 PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
} RCC_RegStruct;
#else
typedef struct
{
  vu32 CRL;
  vu32 CRH;
  vu32 IDR;
  vu32 ODR;
  vu32 BSRR;
  vu32 BRR;
  vu32 LCKR;
} GPIO_TypeDef;

typedef struct
{
  vu32 CR;
  vu32 CFGR;
  vu32 CIR;
  vu32 APB2RSTR;
  vu32 APB1RSTR;
  vu32 AHBENR;
  vu32 APB2ENR;
  vu32 APB1ENR;
  vu32 BDCR;
  vu32 CSR;
} RCC_RegStruct;
#endif
#define pRCC ((RCC_RegStruct *) RCC)



typedef struct {
  vu32 ISER[2];
  u32  RESERVED0[30];
  vu32 ICER[2];
  u32  RSERVED1[30];
  vu32 ISPR[2];
  u32  RESERVED2[30];
  vu32 ICPR[2];
  u32  RESERVED3[30];
  vu32 IABR[2];
  u32  RESERVED4[62];
  vu32 IPR[15];
} NVIC_TypeDef;

typedef struct {
  u8 NVIC_IRQChannel;
  u8 NVIC_IRQChannelPreemptionPriority;
  u8 NVIC_IRQChannelSubPriority;
  bool NVIC_IRQChannelCmd; /* TRUE for enable */
} NVIC_InitTypeDef;

typedef struct {
  vuc32 CPUID;
  vu32 ICSR;
  vu32 VTOR;
  vu32 AIRCR;
  vu32 SCR;
  vu32 CCR;
  vu32 SHPR[3];
  vu32 SHCSR;
  vu32 CFSR;
  vu32 HFSR;
  vu32 DFSR;
  vu32 MMFAR;
  vu32 BFAR;
  vu32 AFSR;
} SCB_TypeDef;

void setPin    (u32 bank, u8 pin);
void resetPin  (u32 bank, u8 pin);
bool readPin   (GPIO_TypeDef *bank, u8 pin);
void strobePin (u32 bank, u8 pin, u8 count, u32 rate);

void systemHardReset(void);
void systemReset   (void);
void setupCLK      (void);
void setupLED      (void);
void setupFLASH    (void);
void setupBUTTON   (void);
bool checkUserCode (u32 usrAddr);
void jumpToUser    (u32 usrAddr);

bool flashWriteWord  (u32 addr, u32 word);
bool flashErasePage  (u32 addr);
bool flashErasePages (u32 addr, u16 n);
void flashLock       (void);
void flashUnlock     (void);
void nvicInit        (NVIC_InitTypeDef*);
void nvicDisableInterrupts(void);

#endif
