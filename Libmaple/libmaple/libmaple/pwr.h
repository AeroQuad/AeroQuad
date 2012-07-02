/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs, LLC.
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
 * @file pwr.h
 * @brief Power control (PWR) defines.
 */

#include "libmaple.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Power interface register map. */
typedef struct pwr_reg_map {
    __io uint32 CR;      /**< Control register */
    __io uint32 CSR;     /**< Control and status register */
} pwr_reg_map;

/** Power peripheral register map base pointer. */
#define PWR_BASE                        ((struct pwr_reg_map*)0x40007000)

/*
 * Register bit definitions
 */

/* Control register */

/** Disable backup domain write protection bit */
#define PWR_CR_DBP  8
/** Power voltage detector enable bit */
#define PWR_CR_PVDE 4
/** Clear standby flag bit */
#define PWR_CR_CSBF 3
/** Clear wakeup flag bit */
#define PWR_CR_CWUF 2
/** Power down deepsleep bit */
#define PWR_CR_PDDS 1
/** Low-power deepsleep bit */
#define PWR_CR_LPDS 0

/* Control and status register */

/** Enable wakeup pin bit */
#define PWR_CSR_EWUP 8
/** PVD output bit */
#define PWR_CSR_PVDO 2
/** Standby flag bit */
#define PWR_CSR_SBF  1
/** Wakeup flag bit */
#define PWR_CSR_WUF  0

/*
 * Convenience functions
 */

void pwr_init(void);

#ifdef __cplusplus
}
#endif
