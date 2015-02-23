/****************************************************************************
 * drivers/sensors/mpu9250_spi.c
 *
 *   Copyright (C) 2015 Pierre-noel Bouteville . All rights reserved.
 *   Authors: Pierre-noel Bouteville <pnb990@gmail.com>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sensors/mpu9250.h>

#include "mpu9250.h"

#if defined(CONFIG_SENSORS_MPU9250) && defined(CONFIG_MPU9250_SPI)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu9250_configspi
 *
 * Description:
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static inline void mpu9250_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the MPU9250 */

  SPI_SETMODE(spi, SPIDEV_MODE3);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, MPU9250_SPI_MAXFREQUENCY);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu9250_getreg8
 *
 * Description:
 *   Read from an 8-bit MPU9250 register
 *
 ****************************************************************************/

uint8_t mpu9250_getreg8(FAR struct mpu9250_dev_s *priv, uint8_t regaddr)
{
  uint8_t regval;

  /* If SPI bus is shared then lock and configure it */

#ifndef CONFIG_SPI_OWNBUS
  (void)SPI_LOCK(priv->spi, true);
  mpu9250_configspi(priv->spi);
#endif

  /* Select the MPU9250 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER, true);
  
  /* Send register to read and get the next byte */

  (void)SPI_SEND(priv->spi, regaddr);
  SPI_RECVBLOCK(priv->spi, &regval, 1);

  /* Deselect the MPU9250 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER, false);

  /* Unlock bus */

#ifndef CONFIG_SPI_OWNBUS
  (void)SPI_LOCK(priv->spi, false);
#endif

#ifdef CONFIG_MPU9250_REGDEBUG
  dbg("%02x->%02x\n", regaddr, regval);
#endif
  return regval;
}

/****************************************************************************
 * Name: mpu9250_putreg8
 *
 * Description:
 *   Write a value to an 8-bit MPU9250 register
 *
 ****************************************************************************/

void mpu9250_putreg8(FAR struct mpu9250_dev_s *priv, uint8_t regaddr,
                     uint8_t regval)
{
#ifdef CONFIG_MPU9250_REGDEBUG
  dbg("%02x<-%02x\n", regaddr, regval);
#endif

  /* If SPI bus is shared then lock and configure it */

#ifndef CONFIG_SPI_OWNBUS
  (void)SPI_LOCK(priv->spi, true);
  mpu9250_configspi(priv->spi);
#endif

  /* Select the MPU9250 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER, true);
  
  /* Send register address and set the value */

  (void)SPI_SEND(priv->spi, regaddr);
  (void)SPI_SEND(priv->spi, regval);

  /* Deselect the MPU9250 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER, false);

  /* Unlock bus */
#ifndef CONFIG_SPI_OWNBUS
  (void)SPI_LOCK(priv->spi, false);
#endif
}

/****************************************************************************
 * Name: mpu9250_getreg16
 *
 * Description:
 *   Read 16-bits of data from an MPU9250 register
 *
 ****************************************************************************/

uint16_t mpu9250_getreg16(FAR struct mpu9250_dev_s *priv, uint8_t regaddr)
{
  uint16_t regval;

  /* If SPI bus is shared then lock and configure it */

#ifndef CONFIG_SPI_OWNBUS
  (void)SPI_LOCK(priv->spi, true);
  mpu9250_configspi(priv->spi);
#endif

  /* Select the MPU9250 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER, true);
  
  /* Send register to read and get the next 2 bytes */

  (void)SPI_SEND(priv->spi, regaddr);
  SPI_RECVBLOCK(priv->spi, &regval, 2);

  /* Deselect the MPU9250 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER, false);

  /* Unlock bus */

#ifndef CONFIG_SPI_OWNBUS
  (void)SPI_LOCK(priv->spi, false);
#endif

#ifdef CONFIG_MPU9250_REGDEBUG
  dbg("%02x->%04x\n", regaddr, regval);
#endif

  return regval;
}

#endif /* CONFIG_SENSORS_MPU9250 && CONFIG_MPU9250_SPI*/
