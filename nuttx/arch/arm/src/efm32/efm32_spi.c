/************************************************************************************
 * arm/arm/src/efm32/efm32_spi.c
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
 * The external functions, efm32_spi1/2/3select and efm32_spi1/2/3status must be
 * provided by board-specific logic.  They are implementations of the select
 * and status methods of the SPI interface defined by struct spi_ops_s (see
 * include/nuttx/spi/spi.h). All other methods (including up_spiinitialize())
 * are provided by common EFM32 logic.  To use this common SPI logic on your
 * board:
 *
 *   1. Provide logic in efm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide efm32_spi1/2/3select() and efm32_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************c********************************/

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
#include "efm32_cmu.h"
#include "efm32_usart.h"
//#include "efm32_dma.h"
#include "efm32_spi_p.h"

#if defined(CONFIG_EFM32_SPI_NBR) && ( CONFIG_EFM32_SPI_NBR > 0 ) 

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* SPI interrupts */

#ifdef CONFIG_EFM32_SPI_DMA
#  error "DMA driven SPI not yet supported"
#endif

/* Can't have both interrupt driven SPI and SPI DMA */

#if defined(CONFIG_EFM32_SPI_INTERRUPTS) && defined(CONFIG_EFM32_SPI_DMA)
#  error "Cannot enable both interrupt mode and DMA mode for SPI"
#endif


/* Debug ****************************************************************************/
/* Check if (non-standard) SPI debug is enabled */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_SPI
#endif

#ifdef CONFIG_DEBUG_SPI
#  define spidbg lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  define spidbg(x...)
#  define spivdbg(x...)
#endif


/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

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
static const struct spi_ops_s g_spiops_tb[CONFIG_EFM32_SPI_NBR];
static struct efm32_spidev_s  g_spidev_tb[CONFIG_EFM32_SPI_NBR];
#endif


/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/



/************************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct efm32_spidev_s *priv = (FAR struct efm32_spidev_s *)dev;

  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      while (sem_wait(&priv->exclsem) != 0)
        {
          /* The only case that an error should occur here is if the wait was awakened
           * by a signal.
           */

          ASSERT(errno == EINTR);
        }
    }
  else
    {
      (void)sem_post(&priv->exclsem);
    }
  return OK;
}
#endif

/************************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ************************************************************************************/

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
    FAR struct efm32_spidev_s *priv = (FAR struct efm32_spidev_s *)dev;

    /* Has the frequency changed? */

#ifndef CONFIG_SPI_OWNBUS
    if (frequency != priv->frequency)
    {
#endif

        USART_BaudrateSyncSet(priv->cfg->usart, 0, frequency);

        /* Save the frequency selection so that subsequent reconfigurations will be
         * faster.
         */

        spivdbg("Frequency %d\n", frequency);

#ifndef CONFIG_SPI_OWNBUS
        priv->frequency = frequency;
    }
#endif
    return frequency;
}

/************************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   Returns void
 *
 ************************************************************************************/

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
    FAR struct efm32_spidev_s *priv = (FAR struct efm32_spidev_s *)dev;

    spivdbg("mode=%d\n", mode);

    /* Has the mode changed? */

#ifndef CONFIG_SPI_OWNBUS
    if (mode != priv->mode)
    {
#endif
        unsigned int u;

        switch (mode)
        {
            case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
                u = USART_CTRL_CLKPOL_IDLELOW | USART_CTRL_CLKPHA_SAMPLELEADING;
                break;

            case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
                u = USART_CTRL_CLKPOL_IDLELOW | USART_CTRL_CLKPHA_SAMPLETRAILING;
                break;

            case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
                u = USART_CTRL_CLKPOL_IDLEHIGH | USART_CTRL_CLKPHA_SAMPLELEADING;
                break;

            case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
                u = USART_CTRL_CLKPOL_IDLEHIGH | USART_CTRL_CLKPHA_SAMPLETRAILING;
                break;

            default:
                return;
        }

        priv->usart->CTRL &= ~(USART_CTRL_CLKPOL_MASK | USART_CTRL_CLKPHA_MASK);
        priv->usart->CTRL = u;


        /* Save the mode so that subsequent re-configurations will be faster */

#ifndef CONFIG_SPI_OWNBUS
        priv->mode = mode;
    }
#endif
}

/************************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
    FAR struct efm32_spidev_s *priv = (FAR struct efm32_spidev_s *)dev;

    spivdbg("nbits=%d\n", nbits);

    /* Has the number of bits changed? */

#ifndef CONFIG_SPI_OWNBUS
    if (nbits != priv->nbits)
    {
#endif
        int i;
        uint32_t frame = priv->usart->FRAME;

        if ( nbits < 0 )
        {
            priv->usart->CTRL &= ~USART_CTRL_MSBF;
            i = -nbits;
        }
        else
        {
            priv->usart->CTRL |= USART_CTRL_MSBF;
            i = nbits;
        }

        switch (i)
        {
            case  4  : i= USART_FRAME_DATABITS_FOUR;    break;
            case  5  : i= USART_FRAME_DATABITS_FIVE;    break;
            case  6  : i= USART_FRAME_DATABITS_SIX;     break;
            case  7  : i= USART_FRAME_DATABITS_SEVEN;   break;
            case  8  : i= USART_FRAME_DATABITS_EIGHT;   break;
            case  9  : i= USART_FRAME_DATABITS_NINE;    break;
            case  10 : i= USART_FRAME_DATABITS_TEN;     break;
            case  11 : i= USART_FRAME_DATABITS_ELEVEN;  break;
            case  12 : i= USART_FRAME_DATABITS_TWELVE;  break;
            case  13 : i= USART_FRAME_DATABITS_THIRTEEN;break;
            case  14 : i= USART_FRAME_DATABITS_FOURTEEN;break;
            case  15 : i= USART_FRAME_DATABITS_FIFTEEN; break;
            case  16 : i= USART_FRAME_DATABITS_SIXTEEN  break;
            default:
                       return;
        }

        priv->usart->FRAME &= USART_FRAME_DATABITS_MASK;
        priv->usart->FRAME |= i;
        /* Save the selection so the subsequence re-configurations will be faster */

#ifndef CONFIG_SPI_OWNBUS
        priv->nbits = nbits;
    }
#endif
}

/************************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ************************************************************************************/

static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
    FAR struct efm32_spidev_s *priv = (FAR struct efm32_spidev_s *)dev;
    uint16_t ret;

    DEBUGASSERT(priv && priv->usart);

    {
        while (!(priv->usart->STATUS & USART_STATUS_TXBL))
        {};
        priv->usart->TXDATA = (uint32_t) data;
        while (!(priv->usart->STATUS & USART_STATUS_TXC))
        {};
        ret = (uint16_t) (priv->usart->RXDATA);
    }

    spivdbg("Sent: %04x Return: %04x \n", wd, ret);

    return ret;
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

#if !defined(CONFIG_EFM32_SPI_DMA) || defined(CONFIG_STM32_DMACAPABLE)
#if !defined(CONFIG_EFM32_SPI_DMA)
static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
#else
static void spi_exchange_nodma(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                               FAR void *rxbuffer, size_t nwords)
#endif
{
  FAR struct efm32_spidev_s *priv = (FAR struct efm32_spidev_s *)dev;
  DEBUGASSERT(priv && priv->usart);

  spivdbg("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

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
#endif /* !CONFIG_EFM32_SPI_DMA || CONFIG_STM32_DMACAPABLE */

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

#ifdef CONFIG_EFM32_SPI_DMA
static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  FAR struct efm32_spidev_s *priv = (FAR struct efm32_spidev_s *)dev;

#ifdef CONFIG_STM32_DMACAPABLE
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

      spivdbg("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);
      DEBUGASSERT(priv && priv->usart);

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
#endif /* CONFIG_EFM32_SPI_DMA */

/*************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer, size_t nwords)
{
  spivdbg("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return spi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/************************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to recieve data
 *   nwords   - the length of data that can be received in the buffer in number
 *              of words.  The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer, size_t nwords)
{
  spivdbg("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

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

static void spi_portinitialize(FAR struct efm32_spidev_s *priv)
{
  uint16_t setbits;
  uint16_t clrbits;

  /* Configure CR1. Default configuration:
   *   Mode 0:                        CPHA=0 and CPOL=0
   *   Master:                        MSTR=1
   *   8-bit:                         DFF=0
   *   MSB tranmitted first:          LSBFIRST=0
   *   Replace NSS with SSI & SSI=1:  SSI=1 SSM=1 (prevents MODF error)
   *   Two lines full duplex:         BIDIMODE=0 BIDIOIE=(Don't care) and RXONLY=0
   */

  clrbits = SPI_CR1_CPHA|SPI_CR1_CPOL|SPI_CR1_BR_MASK|SPI_CR1_LSBFIRST|
            SPI_CR1_RXONLY|SPI_CR1_DFF|SPI_CR1_BIDIOE|SPI_CR1_BIDIMODE;
  setbits = SPI_CR1_MSTR|SPI_CR1_SSI|SPI_CR1_SSM;
  spi_modifycr1(priv, setbits, clrbits);

#ifndef CONFIG_SPI_OWNBUS
  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;
#endif

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* CRCPOLY configuration */

  spi_putreg(priv, STM32_SPI_CRCPR_OFFSET, 7);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

#ifndef CONFIG_SPI_OWNBUS
  sem_init(&priv->exclsem, 0, 1);
#endif

  /* Initialize the SPI semaphores that is used to wait for DMA completion */

#ifdef CONFIG_EFM32_SPI_DMA
  spi_portinitialize_dma(priv);
#endif

  /* Enable spi */

  spi_modifycr1(priv, SPI_CR1_SPE, 0);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ************************************************************************************/

FAR struct spi_dev_s *up_spiinitialize(int port)
{
    /* TODO */
    FAR struct efm32_spidev_s *priv = NULL;

    irqstate_t flags = irqsave();

    if (port < CONFIG_EFM32_SPI_NBR)
    {
        priv = &(g_spidev[port]);

        priv->spidev   = &g_spiops_tb[port]; 
        priv->usart    = NULL; /* TODO: base address of USART */
#ifdef CONFIG_EFM32_SPI_INTERRUPTS
        priv->spiirq   = -1;   /* TODO: EFM32_IRQ_SPI */
#endif
#ifdef CONFIG_EFM32_SPI_DMA
        priv->rxch     = NULL; /* TODO: DMACHAN_SPI1_RX */
        priv->txch     = NULL; /* TODO: DMACHAN_SPI1_TX */
#endif

#if 0
#ifndef CONFIG_SPI_OWNBUS
        priv->lock              = spi_lock;
#endif
        priv->select            = spi_elect; /* TODO */
        priv->setfrequency      = spi_setfrequency;
        priv->setmode           = spi_setmode;
        priv->setbits           = spi_setbits;
        priv->status            = spi_status; /* TODO */
#ifdef CONFIG_SPI_CMDDATA
        priv->cmddata           = spi_cmddata; /* TODO */
#endif
        priv->send              = spi_send;
#ifdef CONFIG_SPI_EXCHANGE
        priv->exchange          = spi_exchange;
#else
        priv->sndblock          = spi_sndblock;
        priv->recvblock         = spi_recvblock;
#endif
        priv->registercallback  = 0;
#endif

        GPIO_PinModeSet(cfg->clk_port,cfg->clk_pin,gpioModePushPull,0);
        GPIO_PinModeSet(cfg->miso_port,cfg->miso_pin,gpioModePushPull,0);
        GPIO_PinModeSet(cfg->mosi_port,cfg->mosi_pin,gpioModePushPull,0);
        GPIO_PinModeSet(cfg->cs_port,cfg->cs_pin,gpioModePushPull,0);

        spi_portinitialize(priv);

    }
    else
    {
        spidbg("ERROR: Unsupported SPI port: %d\n", port);
        return NULL;
    }

    irqrestore(flags);
    return (FAR struct spi_dev_s *)priv;
}

#endif /* CONFIG_EFM32_SPI_NBR */
