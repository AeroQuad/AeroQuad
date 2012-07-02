/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
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
 * @file spi.h
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief Serial Peripheral Interface (SPI) and Integrated
 *        Interchip Sound (I2S) peripheral support.
 *
 * I2S support is currently limited to register maps and bit definitions.
 */

#ifndef _SPI_H_
#define _SPI_H_

#include "libmaple_types.h"
#include "rcc.h"
#include "nvic.h"
#include "gpio.h"
#include "util.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Register maps
 */

/** SPI register map type. */
typedef struct spi_reg_map {
    __io uint32 CR1;            /**< Control register 1 */
    __io uint32 CR2;            /**< Control register 2 */
    __io uint32 SR;             /**< Status register */
    __io uint32 DR;             /**< Data register */
    __io uint32 CRCPR;          /**< CRC polynomial register */
    __io uint32 RXCRCR;         /**< RX CRC register */
    __io uint32 TXCRCR;         /**< TX CRC register */
    __io uint32 I2SCFGR;        /**< I2S configuration register */
    __io uint32 I2SPR;          /**< I2S prescaler register */
} spi_reg_map;

/** SPI1 register map base pointer */
#define SPI1_BASE                       ((struct spi_reg_map*)0x40013000)
/** SPI2 register map base pointer */
#define SPI2_BASE                       ((struct spi_reg_map*)0x40003800)
/** SPI3 register map base pointer */
#define SPI3_BASE                       ((struct spi_reg_map*)0x40003C00)

/*
 * Register bit definitions
 */

/* Control register 1 */

#define SPI_CR1_BIDIMODE_BIT            15
#define SPI_CR1_BIDIOE_BIT              14
#define SPI_CR1_CRCEN_BIT               13
#define SPI_CR1_CRCNEXT_BIT             12
#define SPI_CR1_DFF_BIT                 11
#define SPI_CR1_RXONLY_BIT              10
#define SPI_CR1_SSM_BIT                 9
#define SPI_CR1_SSI_BIT                 8
#define SPI_CR1_LSBFIRST_BIT            7
#define SPI_CR1_SPE_BIT                 6
#define SPI_CR1_MSTR_BIT                2
#define SPI_CR1_CPOL_BIT                1
#define SPI_CR1_CPHA_BIT                0

#define SPI_CR1_BIDIMODE                BIT(SPI_CR1_BIDIMODE_BIT)
#define SPI_CR1_BIDIMODE_2_LINE         (0x0 << SPI_CR1_BIDIMODE_BIT)
#define SPI_CR1_BIDIMODE_1_LINE         (0x1 << SPI_CR1_BIDIMODE_BIT)
#define SPI_CR1_BIDIOE                  BIT(SPI_CR1_BIDIOE_BIT)
#define SPI_CR1_CRCEN                   BIT(SPI_CR1_CRCEN_BIT)
#define SPI_CR1_CRCNEXT                 BIT(SPI_CR1_CRCNEXT_BIT)
#define SPI_CR1_DFF                     BIT(SPI_CR1_DFF_BIT)
#define SPI_CR1_DFF_8_BIT               (0x0 << SPI_CR1_DFF_BIT)
#define SPI_CR1_DFF_16_BIT              (0x1 << SPI_CR1_DFF_BIT)
#define SPI_CR1_RXONLY                  BIT(SPI_CR1_RXONLY_BIT)
#define SPI_CR1_SSM                     BIT(SPI_CR1_SSM_BIT)
#define SPI_CR1_SSI                     BIT(SPI_CR1_SSI_BIT)
#define SPI_CR1_LSBFIRST                BIT(SPI_CR1_LSBFIRST_BIT)
#define SPI_CR1_SPE                     BIT(SPI_CR1_SPE_BIT)
#define SPI_CR1_BR                      (0x7 << 3)
#define SPI_CR1_BR_PCLK_DIV_2           (0x0 << 3)
#define SPI_CR1_BR_PCLK_DIV_4           (0x1 << 3)
#define SPI_CR1_BR_PCLK_DIV_8           (0x2 << 3)
#define SPI_CR1_BR_PCLK_DIV_16          (0x3 << 3)
#define SPI_CR1_BR_PCLK_DIV_32          (0x4 << 3)
#define SPI_CR1_BR_PCLK_DIV_64          (0x5 << 3)
#define SPI_CR1_BR_PCLK_DIV_128         (0x6 << 3)
#define SPI_CR1_BR_PCLK_DIV_256         (0x7 << 3)
#define SPI_CR1_MSTR                    BIT(SPI_CR1_MSTR_BIT)
#define SPI_CR1_CPOL                    BIT(SPI_CR1_CPOL_BIT)
#define SPI_CR1_CPOL_LOW                (0x0 << SPI_CR1_CPOL_BIT)
#define SPI_CR1_CPOL_HIGH               (0x1 << SPI_CR1_CPOL_BIT)
#define SPI_CR1_CPHA                    BIT(SPI_CR1_CPHA_BIT)

/* Control register 2 */

/* RM0008-ism: SPI CR2 has "TXDMAEN" and "RXDMAEN" bits, while the
 * USARTs have CR3 "DMAR" and "DMAT" bits. */

#define SPI_CR2_TXEIE_BIT               7
#define SPI_CR2_RXNEIE_BIT              6
#define SPI_CR2_ERRIE_BIT               5
#define SPI_CR2_SSOE_BIT                2
#define SPI_CR2_TXDMAEN_BIT             1
#define SPI_CR2_RXDMAEN_BIT             0

#define SPI_CR2_TXEIE                   BIT(SPI_CR2_TXEIE_BIT)
#define SPI_CR2_RXNEIE                  BIT(SPI_CR2_RXNEIE_BIT)
#define SPI_CR2_ERRIE                   BIT(SPI_CR2_ERRIE_BIT)
#define SPI_CR2_SSOE                    BIT(SPI_CR2_SSOE_BIT)
#define SPI_CR2_TXDMAEN                 BIT(SPI_CR2_TXDMAEN_BIT)
#define SPI_CR2_RXDMAEN                 BIT(SPI_CR2_RXDMAEN_BIT)

/* Status register */

#define SPI_SR_BSY_BIT                  7
#define SPI_SR_OVR_BIT                  6
#define SPI_SR_MODF_BIT                 5
#define SPI_SR_CRCERR_BIT               4
#define SPI_SR_UDR_BIT                  3
#define SPI_SR_CHSIDE_BIT               2
#define SPI_SR_TXE_BIT                  1
#define SPI_SR_RXNE_BIT                 0

#define SPI_SR_BSY                      BIT(SPI_SR_BSY_BIT)
#define SPI_SR_OVR                      BIT(SPI_SR_OVR_BIT)
#define SPI_SR_MODF                     BIT(SPI_SR_MODF_BIT)
#define SPI_SR_CRCERR                   BIT(SPI_SR_CRCERR_BIT)
#define SPI_SR_UDR                      BIT(SPI_SR_UDR_BIT)
#define SPI_SR_CHSIDE                   BIT(SPI_SR_CHSIDE_BIT)
#define SPI_SR_CHSIDE_LEFT              (0x0 << SPI_SR_CHSIDE_BIT)
#define SPI_SR_CHSIDE_RIGHT             (0x1 << SPI_SR_CHSIDE_BIT)
#define SPI_SR_TXE                      BIT(SPI_SR_TXE_BIT)
#define SPI_SR_RXNE                     BIT(SPI_SR_RXNE_BIT)

/* I2S configuration register */

/* RM0008-ism: CR1 has "CPOL", I2SCFGR has "CKPOL". */

#define SPI_I2SCFGR_I2SMOD_BIT          11
#define SPI_I2SCFGR_I2SE_BIT            10
#define SPI_I2SCFGR_PCMSYNC_BIT         7
#define SPI_I2SCFGR_CKPOL_BIT           3
#define SPI_I2SCFGR_CHLEN_BIT           0

#define SPI_I2SCFGR_I2SMOD              BIT(SPI_I2SCFGR_I2SMOD_BIT)
#define SPI_I2SCFGR_I2SMOD_SPI          (0x0 << SPI_I2SCFGR_I2SMOD_BIT)
#define SPI_I2SCFGR_I2SMOD_I2S          (0x1 << SPI_I2SCFGR_I2SMOD_BIT)
#define SPI_I2SCFGR_I2SE                BIT(SPI_I2SCFGR_I2SE_BIT)
#define SPI_I2SCFGR_I2SCFG              (0x3 << 8)
#define SPI_I2SCFGR_I2SCFG_SLAVE_TX     (0x0 << 8)
#define SPI_I2SCFGR_I2SCFG_SLAVE_RX     (0x1 << 8)
#define SPI_I2SCFGR_I2SCFG_MASTER_TX    (0x2 << 8)
#define SPI_I2SCFGR_I2SCFG_MASTER_RX    (0x3 << 8)
#define SPI_I2SCFGR_PCMSYNC             BIT(SPI_I2SCFGR_PCMSYNC_BIT)
#define SPI_I2SCFGR_PCMSYNC_SHORT       (0x0 << SPI_I2SCFGR_PCMSYNC_BIT)
#define SPI_I2SCFGR_PCMSYNC_LONG        (0x1 << SPI_I2SCFGR_PCMSYNC_BIT)
#define SPI_I2SCFGR_I2SSTD              (0x3 << 4)
#define SPI_I2SCFGR_I2SSTD_PHILLIPS     (0x0 << 4)
#define SPI_I2SCFGR_I2SSTD_MSB          (0x1 << 4)
#define SPI_I2SCFGR_I2SSTD_LSB          (0x2 << 4)
#define SPI_I2SCFGR_I2SSTD_PCM          (0x3 << 4)
#define SPI_I2SCFGR_CKPOL               BIT(SPI_I2SCFGR_CKPOL_BIT)
#define SPI_I2SCFGR_CKPOL_LOW           (0x0 << SPI_I2SCFGR_CKPOL_BIT)
#define SPI_I2SCFGR_CKPOL_HIGH          (0x1 << SPI_I2SCFGR_CKPOL_BIT)
#define SPI_I2SCFGR_DATLEN              (0x3 << 1)
#define SPI_I2SCFGR_DATLEN_16_BIT       (0x0 << 1)
#define SPI_I2SCFGR_DATLEN_24_BIT       (0x1 << 1)
#define SPI_I2SCFGR_DATLEN_32_BIT       (0x2 << 1)
#define SPI_I2SCFGR_CHLEN               BIT(SPI_I2SCFGR_CHLEN_BIT)
#define SPI_I2SCFGR_CHLEN_16_BIT        (0x0 << SPI_I2SCFGR_CHLEN_BIT)
#define SPI_I2SCFGR_CHLEN_32_BIT        (0x1 << SPI_I2SCFGR_CHLEN_BIT)

/*
 * Devices
 */

/** SPI device type */
typedef struct spi_dev {
    spi_reg_map *regs;          /**< Register map */
    rcc_clk_id clk_id;          /**< RCC clock information */
    nvic_irq_num irq_num;       /**< NVIC interrupt number */
} spi_dev;

extern spi_dev *SPI1;
extern spi_dev *SPI2;
#ifdef STM32_HIGH_DENSITY
extern spi_dev *SPI3;
#endif
#ifdef STM32F2
extern spi_dev *SPI4;
#endif

/*
 * SPI Convenience functions
 */

void spi_init(spi_dev *dev);

void spi_gpio_cfg(uint8 as_master,
                  gpio_dev *nss_dev,
                  uint8 nss_bit,
                  gpio_dev *comm_dev,
                  uint8 sck_bit,
                  uint8 miso_bit,
                  uint8 mosi_bit);

/**
 * @brief SPI mode configuration.
 *
 * Determines a combination of clock polarity (CPOL), which determines
 * idle state of the clock line, and clock phase (CPHA), which
 * determines which clock edge triggers data capture.
 */
typedef enum spi_mode {
    SPI_MODE_0,  /**< Clock line idles low (0), data capture on first
                    clock transition. */
    SPI_MODE_1,  /**< Clock line idles low (0), data capture on second
                    clock transition */
    SPI_MODE_2,  /**< Clock line idles high (1), data capture on first
                    clock transition. */
    SPI_MODE_3   /**< Clock line idles high (1), data capture on
                    second clock transition. */
} spi_mode;

/**
 * @brief SPI baud rate configuration, as a divisor of f_PCLK, the
 *        PCLK clock frequency.
 */
typedef enum spi_baud_rate {
    SPI_BAUD_PCLK_DIV_2   = SPI_CR1_BR_PCLK_DIV_2,   /**< f_PCLK/2 */
    SPI_BAUD_PCLK_DIV_4   = SPI_CR1_BR_PCLK_DIV_4,   /**< f_PCLK/4 */
    SPI_BAUD_PCLK_DIV_8   = SPI_CR1_BR_PCLK_DIV_8,   /**< f_PCLK/8 */
    SPI_BAUD_PCLK_DIV_16  = SPI_CR1_BR_PCLK_DIV_16,  /**< f_PCLK/16 */
    SPI_BAUD_PCLK_DIV_32  = SPI_CR1_BR_PCLK_DIV_32,  /**< f_PCLK/32 */
    SPI_BAUD_PCLK_DIV_64  = SPI_CR1_BR_PCLK_DIV_64,  /**< f_PCLK/64 */
    SPI_BAUD_PCLK_DIV_128 = SPI_CR1_BR_PCLK_DIV_128, /**< f_PCLK/128 */
    SPI_BAUD_PCLK_DIV_256 = SPI_CR1_BR_PCLK_DIV_256, /**< f_PCLK/256 */
} spi_baud_rate;

/**
 * @brief SPI initialization flags.
 * @see spi_master_enable()
 * @see spi_slave_enable()
 */
typedef enum spi_cfg_flag {
    SPI_BIDIMODE   = SPI_CR1_BIDIMODE,   /**< Bidirectional mode enable */
    SPI_BIDIOE     = SPI_CR1_BIDIOE,     /**< Output enable in bidirectional
                                            mode */
    SPI_CRCEN      = SPI_CR1_CRCEN,      /**< Cyclic redundancy check (CRC)
                                            enable */
    SPI_DFF_8_BIT  = SPI_CR1_DFF_8_BIT,  /**< 8-bit data frame format (this is
                                            the default) */
    SPI_DFF_16_BIT = SPI_CR1_DFF_16_BIT, /**< 16-bit data frame format */
    SPI_RX_ONLY    = SPI_CR1_RXONLY,     /**< Receive only */
    SPI_SW_SLAVE   = SPI_CR1_SSM,        /**< Software slave management */
    SPI_SOFT_SS    = SPI_CR1_SSI,        /**< Software (internal) slave
                                            select.  This flag only has an
                                            effect when used in combination
                                            with SPI_SW_SLAVE. */
    SPI_FRAME_LSB  = SPI_CR1_LSBFIRST,   /**< LSB-first (little-endian) frame
                                            format */
    SPI_FRAME_MSB  = 0,                  /**< MSB-first (big-endian) frame
                                            format (this is the default) */
} spi_cfg_flag;

void spi_master_enable(spi_dev *dev,
                       spi_baud_rate baud,
                       spi_mode mode,
                       uint32 flags);

void spi_slave_enable(spi_dev *dev,
                      spi_mode mode,
                      uint32 flags);

uint32 spi_tx(spi_dev *dev, const void *buf, uint32 len);

void spi_foreach(void (*fn)(spi_dev (*dev)));

void spi_peripheral_enable(spi_dev *dev);
void spi_peripheral_disable(spi_dev *dev);

void spi_tx_dma_enable(spi_dev *dev);
void spi_tx_dma_disable(spi_dev *dev);

void spi_rx_dma_enable(spi_dev *dev);
void spi_rx_dma_disable(spi_dev *dev);

/**
 * @brief Determine if a SPI peripheral is enabled.
 * @param dev SPI device
 * @return True, if and only if dev's peripheral is enabled.
 */
static inline uint8 spi_is_enabled(spi_dev *dev) {
    return dev->regs->CR1 & SPI_CR1_SPE_BIT;
}

/**
 * @brief Disable all SPI peripherals
 */
static inline void spi_peripheral_disable_all(void) {
    spi_foreach(spi_peripheral_disable);
}

/** Available SPI interrupts */
typedef enum spi_interrupt {
    SPI_TXE_INTERRUPT  = SPI_CR2_TXEIE,  /**< TX buffer empty interrupt */
    SPI_RXNE_INTERRUPT = SPI_CR2_RXNEIE, /**< RX buffer not empty interrupt */
    SPI_ERR_INTERRUPT  = SPI_CR2_ERRIE   /**<
                                          * Error interrupt (CRC, overrun,
                                          * and mode fault errors for SPI;
                                          * underrun, overrun errors for I2S)
                                          */
} spi_interrupt;

/**
 * @brief Mask for all spi_interrupt values
 * @see spi_interrupt
 */
#define SPI_INTERRUPTS_ALL              (SPI_TXE_INTERRUPT  |           \
                                         SPI_RXNE_INTERRUPT |           \
                                         SPI_ERR_INTERRUPT)

/**
 * @brief Enable SPI interrupt requests
 * @param dev SPI device
 * @param interrupt_flags Bitwise OR of spi_interrupt values to enable
 * @see spi_interrupt
 */
static inline void spi_irq_enable(spi_dev *dev, uint32 interrupt_flags) {
    dev->regs->CR2 |= interrupt_flags;
    nvic_irq_enable(dev->irq_num);
}

/**
 * @brief Disable SPI interrupt requests
 * @param dev SPI device
 * @param interrupt_flags Bitwise OR of spi_interrupt values to disable
 * @see spi_interrupt
 */
static inline void spi_irq_disable(spi_dev *dev, uint32 interrupt_flags) {
    dev->regs->CR2 &= ~interrupt_flags;
}

/**
 * @brief Get the data frame format flags with which a SPI port is
 *        configured.
 * @param dev SPI device whose data frame format to get.
 * @return SPI_DFF_8_BIT, if dev has an 8-bit data frame format.
 *         Otherwise, SPI_DFF_16_BIT.
 */
static inline spi_cfg_flag spi_dff(spi_dev *dev) {
    return ((dev->regs->CR1 & SPI_CR1_DFF) == SPI_CR1_DFF_8_BIT ?
            SPI_DFF_8_BIT :
            SPI_DFF_16_BIT);
}

/**
 * @brief Determine whether the device's peripheral receive (RX)
 *        register is empty.
 * @param dev SPI device
 * @return true, iff dev's RX register is empty.
 */
static inline uint8 spi_is_rx_nonempty(spi_dev *dev) {
    return dev->regs->SR & SPI_SR_RXNE;
}

/**
 * @brief Retrieve the contents of the device's peripheral receive
 *        (RX) register.
 *
 * You may only call this function when the RX register is nonempty.
 * Calling this function clears the contents of the RX register.
 *
 * @param dev SPI device
 * @return Contents of dev's peripheral RX register
 * @see spi_is_rx_reg_nonempty()
 */
static inline uint16 spi_rx_reg(spi_dev *dev) {
    return (uint16)dev->regs->DR;
}

/**
 * @brief Determine whether the device's peripheral transmit (TX)
 *        register is empty.
 * @param dev SPI device
 * @return true, iff dev's TX register is empty.
 */
static inline uint8 spi_is_tx_empty(spi_dev *dev) {
    return dev->regs->SR & SPI_SR_TXE;
}

/**
 * @brief Load a value into the device's peripheral transmit (TX) register.
 *
 * You may only call this function when the TX register is empty.
 * Calling this function loads val into the peripheral's TX register.
 * If the device is properly configured, this will initiate a
 * transmission, the completion of which will cause the TX register to
 * be empty again.
 *
 * @param dev SPI device
 * @param val Value to load into the TX register.  If the SPI data
 *            frame format is 8 bit, the value must be right-aligned.
 * @see spi_is_tx_reg_empty()
 * @see spi_init()
 * @see spi_master_enable()
 * @see spi_slave_enable()
 */
static inline void spi_tx_reg(spi_dev *dev, uint16 val) {
    dev->regs->DR = val;
}

/**
 * @brief Determine whether the device's peripheral busy (SPI_SR_BSY)
 *        flag is set.
 * @param dev SPI device
 * @return true, iff dev's BSY flag is set.
 */
static inline uint8 spi_is_busy(spi_dev *dev) {
    return dev->regs->SR & SPI_SR_BSY;
}

/*
 * I2S convenience functions (TODO)
 */

#ifdef __cplusplus
}
#endif

#endif
