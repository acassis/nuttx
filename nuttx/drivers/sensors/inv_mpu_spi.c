/****************************************************************************
 * drivers/sensors/inv_mpu_spi.c
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
#include <time.h>
#include <nuttx/spi/spi.h>

#include "inv_mpu.h"

#if defined(CONFIG_INVENSENSE_SPI)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mpu_spi_low_s
{
    struct mpu_low_s *low;
    int akm_addr;
    int spi_dev_s* spi;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mpu_spi_write(FAR struct mpu_low_s* low, int reg_off, 
                                const uint8_t *buf, int size);
static int mpu_spi_read( FAR struct mpu_low_s* low, int reg_off, 
                                const uint8_t *buf, int size);
static int akm_spi_write(FAR struct mpu_low_s* low, int reg_off, 
                                const uint8_t *buf, int size);
static int akm_spi_read( FAR struct mpu_low_s* low, int reg_off, 
                                const uint8_t *buf, int size);

/****************************************************************************
 * Private data
 ****************************************************************************/

static struct mpu_low_s mpu_i2c_low = 
{
    .mpu_write = mpu_spi_write,
    .mpu_read  = mpu_spi_read,
    .akm_write = akm_spi_write,
    .akm_read  = akm_spi_read
};

struct mpu_spi_low_s g_mpu_spi;


/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static inline void mpu_spi_configspi(FAR struct spi_dev_s *spi)
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

static inline int mpu_spi_trans(FAR struct mpu_spi_low_s *priv, bool read,
                                uint8_t reg_off, uint8_t buf, int size)
{

  /* If SPI bus is shared then lock and configure it */

#ifndef CONFIG_SPI_OWNBUS
  (void)SPI_LOCK(priv->spi, true);
  mpu9250_configspi(priv->spi);
#endif

  /* Select the MPU9250 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER, true);
  
  /* Send register to read and get the next byte */

  (void)SPI_SEND(priv->spi, regaddr);
  if ( read )
  {
      SPI_RECVBLOCK(priv->spi, buf, size);
  }
  else
  {
      SPI_SEND(     priv->spi, buf, size);
  }

  /* Deselect the MPU9250 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER, false);

  /* Unlock bus */

#ifndef CONFIG_SPI_OWNBUS
  (void)SPI_LOCK(priv->spi, false);
#endif

  return 0;
}


/* MPU Write access */

static int mpu_i2c_write(FAR struct mpu_low_s* low, int reg_off, 
                                const uint8_t *buf, int size)
{

  return mpu_i2c_trans(priv,priv->mpu_addr,false,reg_off,buf,size);
}

/* MPU read access */

static int mpu_i2c_read(FAR struct mpu_low_s* low, int reg_off, 
                                const uint8_t *buf, int size)
{
  struct mpu_i2c_low_s priv = (struct mpu_i2c_low_s*)low;

  return mpu_i2c_trans(priv,priv->mpu_addr,true,reg_off,buf,size);
}

/* AKM Write access */

static int akm_i2c_write(FAR struct mpu_low_s* low, int reg_off, 
                                const uint8_t *buf, int size)
{
  struct mpu_i2c_low_s priv = (struct mpu_i2c_low_s*)low;

  /* TODO use internal I2C MASTER of MPU */
#warning 'not implemented"

  return -1;
}

/* AKM read access */

static int akm_i2c_read(FAR struct mpu_low_s* low, int reg_off, 
                                const uint8_t *buf, int size)
{
  struct mpu_i2c_low_s priv = (struct mpu_i2c_low_s*)low;

  /* TODO use internal I2C MASTER of MPU */
#warning 'not implemented"

  return -1;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/
struct mpu_low_s* mpu_low_spi_init(int devno, int akm_addr, 
                                   FAR struct spi_dev_s* spi)
{
    struct mpu_spi_low_s* mpu_spi;

    /* only one device supported */

    if ( devno != 0 )
        return NULL;

    mpu_spi = &g_mpu_spi;

    mpu_spi->low = mpu_i2c_low;
    mpu_spi->mpu_addr = mpu_addr;
    mpu_spi->akm_addr = akm_addr;
    mpu_spi->i2c;

    return mpu_i2c;

}

#endif /* CONFIG_INVENSENSE_SPI */
