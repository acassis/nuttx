/************************************************************************************
 * arm/arm/src/efm32/efm32_spi_p.h
 *
 *   Copyright (C) 2009-2013 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2009-2013 Bouteville Pierre-Noel. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Author: Bouteville Pierre-Noel <pnb990@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_EFM32_EFM32_SPI_P_H
#define __ARCH_ARM_EFM32_EFM32_SPI_P_H

#include "efm32_spi.h"
/************************************************************************************
 * Private Types
 ************************************************************************************/

struct efm32_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  efm32_spi_cfg_t  cfg;        /* SPI hardware config */
  uint32_t         spiclock;   /* Clocking for the SPI module */
#ifdef CONFIG_EFM32_SPI_INTERRUPTS
  uint8_t          spiirq;     /* SPI IRQ number */
#endif
#ifdef CONFIG_EFM32_SPI_DMA
  volatile uint8_t rxresult;   /* Result of the RX DMA */
  volatile uint8_t txresult;   /* Result of the RX DMA */
  uint8_t          rxch;       /* The RX DMA channel number */
  uint8_t          txch;       /* The TX DMA channel number */
  DMA_HANDLE       rxdma;      /* DMA channel handle for RX transfers */
  DMA_HANDLE       txdma;      /* DMA channel handle for TX transfers */
  sem_t            rxsem;      /* Wait for RX DMA to complete */
  sem_t            txsem;      /* Wait for TX DMA to complete */
  uint32_t         txccr;      /* DMA control register for TX transfers */
  uint32_t         rxccr;      /* DMA control register for RX transfers */
#endif
#ifndef CONFIG_SPI_OWNBUS
  sem_t            exclsem;    /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  int8_t           nbits;      /* Width of word in bits (8 or 16) */
  uint8_t          mode;       /* Mode 0,1,2,3 */
#endif
};

/* DMA support */

#ifdef CONFIG_EFM32_SPI_DMA
static void        spi_dmarxwait(   FAR struct efm32_spidev_s *priv);
static void        spi_dmatxwait(   FAR struct efm32_spidev_s *priv);
static inline void spi_dmarxwakeup( FAR struct efm32_spidev_s *priv);
static inline void spi_dmatxwakeup( FAR struct efm32_spidev_s *priv);
static void        spi_dmarxcallback(DMA_HANDLE handle, uint8_t isr, void *arg);
static void        spi_dmatxcallback(DMA_HANDLE handle, uint8_t isr, void *arg);
static void        spi_dmarxsetup(FAR struct efm32_spidev_s *priv,
                                  FAR void *rxbuffer, 
                                  FAR void *rxdummy, 
                                  size_t nwords
                                 );
static void        spi_dmatxsetup(FAR struct efm32_spidev_s *priv,
                                  FAR const void *txbuffer, 
                                  FAR const void *txdummy, 
                                  size_t nwords
                                 );
static inline void spi_dmarxstart(FAR struct efm32_spidev_s *priv);
static inline void spi_dmatxstart(FAR struct efm32_spidev_s *priv);
#endif

#endif

