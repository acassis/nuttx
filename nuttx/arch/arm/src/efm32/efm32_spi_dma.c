/************************************************************************************
 * arm/arm/src/efm32/efm32_spi_dma.c
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "efm32.h"
#include "efm32_gpio.h"
#include "efm32_dma.h"
#include "efm32_spi.h"

#if defined(CONFIG_EFM32_SPI) 

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* SPI interrupts */

#ifdef CONFIG_EFM32_SPI_DMA
#  error "DMA driven SPI not yet supported"



/* Debug ****************************************************************************/
/* Check if (non-standard) SPI debug is enabled */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_SPI
#endif

#ifdef CONFIG_DEBUG_SPI
#  define spidbgdma lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define spivdbgdma lldbg
#  else
#    define spivdbgdma(x...)
#  endif
#else
#  define spidbgdma(x...)
#  define spivdbgdma(x...)
#endif

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Helpers */

static inline uint16_t spi_readword(FAR struct efm32_spidev_s *priv);
static inline void spi_writeword(FAR struct efm32_spidev_s *priv, uint16_t byte);
static inline bool spi_16bitmode(FAR struct efm32_spidev_s *priv);

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

/* SPI methods */

#ifndef CONFIG_SPI_OWNBUS
static int         spi_lock(        FAR struct spi_dev_s *dev, bool lock);
#endif
static uint32_t    spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void        spi_setmode(     FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void        spi_setbits(     FAR struct spi_dev_s *dev, int nbits);
static uint16_t    spi_send(        FAR struct spi_dev_s *dev, uint16_t wd);
static void        spi_exchange(    FAR struct spi_dev_s *dev, 
                                    FAR const void *txbuffer,
                                    FAR void *rxbuffer, 
                                    size_t nwords
                               );
#ifndef CONFIG_SPI_EXCHANGE
static void        spi_sndblock(    FAR struct spi_dev_s *dev, 
                                    FAR const void *txbuffer,
                                    size_t nwords
                               );
static void        spi_recvblock(   FAR struct spi_dev_s *dev, 
                                    FAR void *rxbuffer,
                                    size_t nwords
                                );
#endif

/* Initialization */

static void        spi_portinitialize(FAR struct efm32_spidev_s *priv);

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_EFM32_SPI_NBR
static const struct spi_ops_s g_spiops_tb[CONFIG_EFM32_SPI_NBR] =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = spi_elect, /* TODO */
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = spi_status, /* TODO */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = spi_cmddata, /* TODO */
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,
};

static struct efm32_spidev_s g_spidev_tb[CONFIG_EFM32_SPI_NBR] =
{
  .spidev   = NULL, /* TODO: { &g_spiops[x] } */
  .spibase  = NULL, /* TODO: EFM32_SPIx_BASE */
  .spiclock = EFM32_PCLK2_FREQUENCY,
#ifdef CONFIG_EFM32_SPI_INTERRUPTS
  .spiirq   = -1,   /* TODO: EFM32_IRQ_SPI */
#endif
#ifdef CONFIG_EFM32_SPI_DMA
  .rxch     = NULL, /* TODO: DMACHAN_SPI1_RX */
  .txch     = NULL, /* TODO: DMACHAN_SPI1_TX */
#endif
};
#endif


/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/


/************************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ************************************************************************************/

static void spi_dmarxwait(FAR struct efm32_spidev_s *priv)
{
    /* TODO */
  /* Take the semaphore (perhaps waiting).  If the result is zero, then the DMA
   * must not really have completed???
   */

  while (sem_wait(&priv->rxsem) != 0 || priv->rxresult == 0)
    {
      /* The only case that an error should occur here is if the wait was awakened
       * by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/************************************************************************************
 * Name: spi_dmatxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ************************************************************************************/

static void spi_dmatxwait(FAR struct efm32_spidev_s *priv)
{
    /* TODO */
  /* Take the semaphore (perhaps waiting).  If the result is zero, then the DMA
   * must not really have completed???
   */

  while (sem_wait(&priv->txsem) != 0 || priv->txresult == 0)
    {
      /* The only case that an error should occur here is if the wait was awakened
       * by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/************************************************************************************
 * Name: spi_dmarxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ************************************************************************************/

static inline void spi_dmarxwakeup(FAR struct efm32_spidev_s *priv)
{
    /* TODO */
  (void)sem_post(&priv->rxsem);
}

/************************************************************************************
 * Name: spi_dmatxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ************************************************************************************/

static inline void spi_dmatxwakeup(FAR struct efm32_spidev_s *priv)
{
    /* TODO */
  (void)sem_post(&priv->txsem);
}

/************************************************************************************
 * Name: spi_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ************************************************************************************/

static void spi_dmarxcallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
    /* TODO */
  FAR struct efm32_spidev_s *priv = (FAR struct efm32_spidev_s *)arg;

  /* Wake-up the SPI driver */

  priv->rxresult = isr | 0x080;  /* OR'ed with 0x80 to assure non-zero */
  spi_dmarxwakeup(priv);
}

/************************************************************************************
 * Name: spi_dmatxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ************************************************************************************/

static void spi_dmatxcallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
    /* TODO */
  FAR struct efm32_spidev_s *priv = (FAR struct efm32_spidev_s *)arg;

  /* Wake-up the SPI driver */

  priv->txresult = isr | 0x080;  /* OR'ed with 0x80 to assure non-zero */
  spi_dmatxwakeup(priv);
}

/************************************************************************************
 * Name: spi_dmarxsetup
 *
 * Description:
 *   Setup to perform RX DMA
 *
 ************************************************************************************/

static void spi_dmarxsetup(FAR struct efm32_spidev_s *priv, FAR void *rxbuffer,
                           FAR void *rxdummy, size_t nwords)
{
    /* TODO */
  /* 8- or 16-bit mode? */

  if (spi_16bitmode(priv))
    {
      /* 16-bit mode -- is there a buffer to receive data in? */

      if (rxbuffer)
        {
          priv->rxccr = SPI_RXDMA16_CONFIG;
        }
      else
        {
          rxbuffer    = rxdummy;
          priv->rxccr = SPI_RXDMA16NULL_CONFIG;
        }
    }
  else
    {
      /* 8-bit mode -- is there a buffer to receive data in? */

      if (rxbuffer)
        {
          priv->rxccr = SPI_RXDMA8_CONFIG;
        }
      else
        {
          rxbuffer    = rxdummy;
          priv->rxccr = SPI_RXDMA8NULL_CONFIG;
        }
     }

  /* Configure the RX DMA */

  stm32_dmasetup(priv->rxdma, priv->spibase + STM32_SPI_DR_OFFSET,
                 (uint32_t)rxbuffer, nwords, priv->rxccr);
}

/************************************************************************************
 * Name: spi_dmatxsetup
 *
 * Description:
 *   Setup to perform TX DMA
 *
 ************************************************************************************/

static void spi_dmatxsetup(FAR struct efm32_spidev_s *priv, FAR const void *txbuffer,
                           FAR const void *txdummy, size_t nwords)
{
    /* TODO */
  /* 8- or 16-bit mode? */

  if (spi_16bitmode(priv))
    {
      /* 16-bit mode -- is there a buffer to transfer data from? */

      if (txbuffer)
        {
          priv->txccr = SPI_TXDMA16_CONFIG;
        }
      else
        {
          txbuffer    = txdummy;
          priv->txccr = SPI_TXDMA16NULL_CONFIG;
        }
    }
  else
    {
      /* 8-bit mode -- is there a buffer to transfer data from? */

      if (txbuffer)
        {
          priv->txccr = SPI_TXDMA8_CONFIG;
        }
      else
        {
          txbuffer    = txdummy;
          priv->txccr = SPI_TXDMA8NULL_CONFIG;
        }
    }

  /* Setup the TX DMA */

  stm32_dmasetup(priv->txdma, priv->spibase + STM32_SPI_DR_OFFSET,
                 (uint32_t)txbuffer, nwords, priv->txccr);
}

/************************************************************************************
 * Name: spi_dmarxstart
 *
 * Description:
 *   Start RX DMA
 *
 ************************************************************************************/

static inline void spi_dmarxstart(FAR struct efm32_spidev_s *priv)
{
    /* TODO */
  priv->rxresult = 0;
  stm32_dmastart(priv->rxdma, spi_dmarxcallback, priv, false);
}

/************************************************************************************
 * Name: spi_dmatxstart
 *
 * Description:
 *   Start TX DMA
 *
 ************************************************************************************/

static inline void spi_dmatxstart(FAR struct efm32_spidev_s *priv)
{
    /* TODO */
  priv->txresult = 0;
  stm32_dmastart(priv->txdma, spi_dmatxcallback, priv, false);
}


/************************************************************************************
 * Name: spi_exchange (no DMA).  aka spi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchaned in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_exchange_nodma(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                               FAR void *rxbuffer, size_t nwords)
{
  FAR struct efm32_spidev_s *priv = (FAR struct efm32_spidev_s *)dev;
  DEBUGASSERT(priv && priv->spibase);

  spivdbgdma("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* 8- or 16-bit mode? */

  if (spi_16bitmode(priv))
    {
      /* 16-bit mode */

      const uint16_t *src  = (const uint16_t*)txbuffer;;
            uint16_t *dest = (uint16_t*)rxbuffer;
            uint16_t  word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = *src++;
            }
          else
          {
              word = 0xffff;
          }

          /* Exchange one word */

          word = spi_send(dev, word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = word;
            }
        }
    }
  else
    {
      /* 8-bit mode */

      const uint8_t *src  = (const uint8_t*)txbuffer;;
            uint8_t *dest = (uint8_t*)rxbuffer;
            uint8_t  word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = *src++;
            }
          else
          {
              word = 0xff;
          }

          /* Exchange one word */

          word = (uint8_t)spi_send(dev, (uint16_t)word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = word;
            }
        }
    }
}

/*************************************************************************
 * Name: spi_exchange (with DMA capability)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  FAR struct efm32_spidev_s *priv = (FAR struct efm32_spidev_s *)dev;

#ifdef CONFIG_EFM32_DMACAPABLE
  if ((txbuffer && !stm32_dmacapable((uint32_t)txbuffer, nwords, priv->txccr)) ||
      (rxbuffer && !stm32_dmacapable((uint32_t)rxbuffer, nwords, priv->rxccr)))
    {
      /* Unsupported memory region, fall back to non-DMA method. */

      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
    }
  else
#endif
    {
      static uint16_t rxdummy = 0xffff;
      static const uint16_t txdummy = 0xffff;

      spivdbgdma("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);
      DEBUGASSERT(priv && priv->spibase);

      /* Setup DMAs */

      spi_dmarxsetup(priv, rxbuffer, &rxdummy, nwords);
      spi_dmatxsetup(priv, txbuffer, &txdummy, nwords);

      /* Start the DMAs */

      spi_dmarxstart(priv);
      spi_dmatxstart(priv);

      /* Then wait for each to complete */

      spi_dmarxwait(priv);
      spi_dmatxwait(priv);
    }
}



/************************************************************************************
 * Name: spi_portinitialize
 *
 * Description:
 *   Initialize the selected SPI port in its default state (Master, 8-bit, mode 0, etc.)
 *
 * Input Parameter:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void spi_portinitialize_dma(FAR struct efm32_spidev_s *priv)
{

  sem_init(&priv->rxsem, 0, 0);
  sem_init(&priv->txsem, 0, 0);

  /* Get DMA channels.  NOTE: stm32_dmachannel() will always assign the DMA channel.
   * if the channel is not available, then stm32_dmachannel() will block and wait
   * until the channel becomes available.  WARNING: If you have another device sharing
   * a DMA channel with SPI and the code never releases that channel, then the call
   * to stm32_dmachannel()  will hang forever in this function!  Don't let your
   * design do that!
   */

  priv->rxdma = stm32_dmachannel(priv->rxch);
  priv->txdma = stm32_dmachannel(priv->txch);
  DEBUGASSERT(priv->rxdma && priv->txdma);

  spi_putreg(priv, STM32_SPI_CR2_OFFSET, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif

#endif /* CONFIG_EFM32_SPI */

