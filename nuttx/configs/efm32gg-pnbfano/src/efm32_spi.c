/************************************************************************************
 * configs/efm32gg-pnbfano/src/efm32_spi.c
 *
 *   Copyright (C) 2009-2013 Bouteville Pierre-Noel. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>

#include <nuttx/wireless/wireless.h>
#include <nuttx/wireless/cc3000.h>

#include <arch/board/board.h>


#include "up_arch.h"
#include "chip.h"
#include "efm32gg-pnbfano.h"
#include "efm32_gpio.h"
#include "efm32_spi.h"

#if defined(CONFIG_PNBFANO_USE_MMCSD) 

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

#undef SPI_DEBUG   /* Define to enable debug */
#undef SPI_VERBOSE /* Define to enable verbose debug */

#ifdef SPI_DEBUG
#  define spidbg(...)  syslog(LOG_DEBUG,__VA_ARGS__)
#  ifdef SPI_VERBOSE
#    define spivdbg(...) syslog(LOG_DEBUG,__VA_ARGS__)
#  else
#    define spivdbg(...)
#  endif
#else
#  undef SPI_VERBOSE
#  define spidbg(...)
#  define spivdbg(...)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/


/****************************************************************************
 * Name:  stm32_spi1/2/3select and stm32_spi1/2/3status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status must be
 *   provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods (including up_spiinitialize())
 *   are provided by common STM32 logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_USART0_ISSPI
void efm32_spi0_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  if (devid == SPIDEV_MMCSD)
    {
      efm32_gpiowrite(GPIO_SDCARD_SPI_CS, !selected);
    }
#if defined(CONFIG_PNBFANO_CC3000)
  else if (devid == SPIDEV_WIRELESS)
    {
      efm32_gpiowrite(GPIO_WIFI_CS, !selected);
    }
#endif
  else
    {
        ASSERT(false);
    }
}

uint8_t efm32_spi0_status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
    /* TODO check presense */
    return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_EFM32_USART1_ISSPI
void efm32_spi1_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
    /* nerver used in this board */
#if 0
  if (devid == SPIDEV_DISPLAY)
    {
      efm32_gpiowrite(GPIO_OLED_CS, !selected);
    }
    {
      efm32_gpiowrite(GPIO_CS_MEMS, !selected);
    }
#endif
}

uint8_t efm32_spi1_status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return 0;
}
#endif

#ifdef CONFIG_EFM32_USART2_ISSPI
void efm32_spi2_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
    /* nerver used in this board */
}

uint8_t efm32_spi2_status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return 0;
}
#endif

#ifdef  CONFIG_PNBFANO_USE_MMCSD
int sdcard_initialize(FAR struct spi_dev_s *spi)
{
    int ret = OK;
    /* Bind the SPI port to the slot */

    syslog(LOG_DEBUG,"Binding SPI port %d to MMC/SD slot %d\n",
           PNBFANO_SDCARD_EXT_SPINO, PNBFANO_SDCARD_SLOTNO
          );

    ret = mmcsd_spislotinitialize(PNBFANO_SDCARD_MINOR, 
                                  PNBFANO_SDCARD_SLOTNO, 
                                  spi);

    if (ret < 0)
    {
        syslog(LOG_ERR,"Failed to bind SPI port %d to MMC/SD slot %d: %d\n",
               PNBFANO_SDCARD_EXT_SPINO, PNBFANO_SDCARD_MINOR, ret
              );
        return ret;
    }

    syslog(LOG_INFO,"Successfuly bound SPI port %d to MMC/SD slot %d\n",
           PNBFANO_SDCARD_EXT_SPINO, PNBFANO_SDCARD_MINOR
          );
    return OK;
}
#endif

int efm32_initialize_spi_devices(void)
{
#ifdef  CONFIG_PNBFANO_USE_MMCSD
    FAR struct spi_dev_s* spi = NULL;

    /* configure chips selects */

    efm32_configgpio(GPIO_SDCARD_SPI_CS );
    efm32_configgpio(GPIO_WIFI_CS    );

    /* Get the SPI port */

    syslog(LOG_DEBUG,"Initializing SPI port %d\n",
           PNBFANO_SDCARD_EXT_SPINO);

    spi = up_spiinitialize(PNBFANO_SDCARD_EXT_SPINO);
    if (!spi)
    {
        syslog(LOG_ERR,"Failed to initialize SPI port %d\n",
               PNBFANO_SDCARD_EXT_SPINO);
        return -ENODEV;
    }

    syslog(LOG_INFO,"Successfully initialized SPI port %d\n",
           PNBFANO_SDCARD_EXT_SPINO);


    if ( sdcard_initialize(spi) < 0 )
    {
        syslog(LOG_ERR,"SDCARD initialization Failed\n");
        return -ENODEV;
    }

    syslog(LOG_INFO,"Successfully initialized SDCARD port %d\n",
           PNBFANO_SDCARD_EXT_SPINO);


#else
    syslog(LOG_WARNING,"SDHC not used\n");
#endif

#ifdef  CONFIG_WIRELESS
    syslog(LOG_DEBUG,"Initializing WIFI\n");
    if ( wireless_archinitialize(0) < 0 )
    {
        syslog(LOG_ERR,"WIFI driver error!\n");
        return -ENODEV;
    }
    syslog(LOG_DEBUG,"WIFI initialized!\n");
#endif

    return OK;
}

#endif 
