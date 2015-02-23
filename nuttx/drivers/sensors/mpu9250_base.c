/****************************************************************************
 * drivers/sensors/mpu9250_base.c
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
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sensors/mpu9250.h>

#include "mpu9250.h"

#if defined(CONFIG_SENSORS_MPU9250)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Character driver methods */

static int     mpu9250_open(FAR struct file *filep);
static int     mpu9250_close(FAR struct file *filep);
static ssize_t mpu9250_read(FAR struct file *filep, FAR char *buffer,
                            size_t len);
static int     mpu9250_ioctl(FAR struct file *filep, int cmd, 
                             unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
  int          mpu9250_poll(FAR struct file *filep, struct pollfd *fds, 
                            bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_mpu9250fops =
{
  mpu9250_open,    /* open */
  mpu9250_close,   /* close */
  mpu9250_read,    /* read */
  0,               /* write */
  0,               /* seek */
  0,               /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  int     (*poll)(FAR struct file *filep, struct pollfd *fds, bool setup);
#endif
};

/****************************************************************************
 * Name: mpu9250_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int mpu9250_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: mpu9250_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int mpu9250_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: mpu9250_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t mpu9250_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode         *inode;
  FAR struct mpu9250_dev_s *priv;
  struct mpu9250_sample_s   sample;
  int                       ret;

  snvdbg("len=%d\n", len);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct mpu9250_dev_s *)inode->i_private;

  /* Verify that the caller has provided a buffer large enough to receive
   * the accelerometer data.
   */

  if (len < sizeof(struct mpu9250_sample_s))
    {
      /* We could provide logic to break up a touch report into segments and
       * handle smaller reads... but why?
       */

      return -ENOSYS;
    }

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->exclsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Read accelerometer X Y Z axes */

  sample.data_x =  mpu9250_getreg8(priv, MPU9250_DATAX1);
  sample.data_x = (sample.data_x << 8) | mpu9250_getreg8(priv, MPU9250_DATAX0);
  sample.data_y =  mpu9250_getreg8(priv, MPU9250_DATAY1);
  sample.data_y = (sample.data_y << 8) | mpu9250_getreg8(priv, MPU9250_DATAY0);
  sample.data_z =  mpu9250_getreg8(priv, MPU9250_DATAZ1);
  sample.data_z = (sample.data_z << 8) | mpu9250_getreg8(priv, MPU9250_DATAZ0);

  /* Return read sample */

  buffer = (FAR char *) &sample;

  sem_post(&priv->exclsem);
  return sizeof(struct mpu9250_sample_s);
}

/****************************************************************************
 * Name: mpu9250_register
 *
 * Description:
 *  This function will register the touchscreen driver as /dev/accelN where N
 *  is the minor device number
 *
 * Input Parameters:
 *   handle    - The handle previously returned by mpu9250_register
 *   minor     - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int mpu9250_register(MPU9250_HANDLE handle, int minor)
{
  FAR struct mpu9250_dev_s *priv = (FAR struct mpu9250_dev_s *)handle;
  char devname[DEV_NAMELEN];
  int ret;

  snvdbg("handle=%p minor=%d\n", handle, minor);
  DEBUGASSERT(priv);

  /* Get exclusive access to the device structure */

  ret = sem_wait(&priv->exclsem);
  if (ret < 0)
    {
      int errval = errno;
      sndbg("ERROR: sem_wait failed: %d\n", errval);
      return -errval;
    }

  /* Register the character driver */

  snprintf(devname, DEV_NAMELEN, DEV_FORMAT, minor);
  ret = register_driver(devname, &g_mpu9250fops, 0666, priv);
  if (ret < 0)
    {
      sndbg("ERROR: Failed to register driver %s: %d\n", devname, ret);
      sem_post(&priv->exclsem);
      return ret;
    }

  /* Indicate that the accelerometer was successfully initialized */

  priv->status |= MPU9250_STAT_INITIALIZED;  /* Accelerometer is initialized */
  sem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: mpu9250_worker
 *
 * Description:
 *   This is the "bottom half" of the MPU9250 interrupt handler
 *
 ****************************************************************************/

static void mpu9250_worker(FAR void *arg)
{
  FAR struct mpu9250_dev_s *priv = (FAR struct mpu9250_dev_s *)arg;
  uint8_t regval;

  DEBUGASSERT(priv && priv->config);

  /* Get the global interrupt status */

  regval =  mpu9250_getreg8(priv, MPU9250_INT_SOURCE);

  /* Check for a data ready interrupt */

  if ((regval & INT_DATA_READY) != 0)
    {
      /* Read accelerometer data to sample */

      priv->sample.data_x =  mpu9250_getreg8(priv, MPU9250_DATAX1);
      priv->sample.data_x = (priv->sample.data_x << 8) | mpu9250_getreg8(priv, MPU9250_DATAX0);
      priv->sample.data_y =  mpu9250_getreg8(priv, MPU9250_DATAY1);
      priv->sample.data_y = (priv->sample.data_y << 8) | mpu9250_getreg8(priv, MPU9250_DATAY0);
      priv->sample.data_z =  mpu9250_getreg8(priv, MPU9250_DATAZ1);
      priv->sample.data_z = (priv->sample.data_z << 8) | mpu9250_getreg8(priv, MPU9250_DATAZ0);
    }

  /* Re-enable the MPU9250 GPIO interrupt */

  priv->config->enable(priv->config, true);
}

/****************************************************************************
 * Name: mpu9250_interrupt
 *
 * Description:
 *  The MPU9250 interrupt handler
 *
 ****************************************************************************/

static void mpu9250_interrupt(FAR struct mpu9250_config_s *config, FAR void *arg)
{
  FAR struct mpu9250_dev_s *priv = (FAR struct mpu9250_dev_s *)arg;
  int ret;

  DEBUGASSERT(priv && priv->config == config);

  /* Disable further interrupts */

  config->enable(config, false);

  /* Check if interrupt work is already queue.  If it is already busy, then
   * we already have interrupt processing in the pipeline and we need to do
   * nothing more.
   */

  if (work_available(&priv->work))
    {
      /* Yes.. Transfer processing to the worker thread.  Since MPU9250
       * interrupts are disabled while the work is pending, no special
       * action should be required to protect the work queue.
       */

      ret = work_queue(HPWORK, &priv->work, mpu9250_worker, priv, 0);
      if (ret != 0)
        {
          snlldbg("Failed to queue work: %d\n", ret);
        }
    }

  /* Clear any pending interrupts and return success */

  config->clear(config);
}

/****************************************************************************
 * Name: mpu9250_checkid
 *
 * Description:
 *   Read and verify the MPU9250 chip ID
 *
 ****************************************************************************/

static int mpu9250_checkid(FAR struct mpu9250_dev_s *priv)
{
  uint8_t devid = 0;

  /* Read device ID  */

  devid = mpu9250_getreg8(priv, MPU9250_DEVID);
  snvdbg("devid: %04x\n", devid);

  if (devid != (uint16_t) DEVID)
    {
      /* ID is not Correct */

      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: mpu9250_reset
 *
 * Description:
 *  Reset the MPU9250
 *
 ****************************************************************************/

static void mpu9250_reset(FAR struct mpu9250_dev_s *priv)
{
  /* MPU9250 doesn't have software reset */

  /* Wait a bit to make the GOD of TIME happy */

  usleep(20*1000);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu9250_instantiate
 *
 * Description:
 *   Instantiate and configure the MPU9250 device driver to use the provided
 *   I2C or SPIdevice instance.
 *
 * Input Parameters:
 *   dev     - An I2C or SPI driver instance
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   A non-zero handle is returned on success.  This handle may then be used
 *   to configure the MPU9250 driver as necessary.  A NULL handle value is
 *   returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MPU9250_SPI
MPU9250_HANDLE mpu9250_instantiate(FAR struct spi_dev_s *dev,
                                   FAR struct mpu9250_config_s *config)
#else
MPU9250_HANDLE mpu9250_instantiate(FAR struct i2c_dev_s *dev,
                                   FAR struct mpu9250_config_s *config)
#endif
{
  FAR struct mpu9250_dev_s *priv;
  uint8_t regval;
  int ret;

  /* Allocate the MPU9250 driver instance */

  priv = (FAR struct mpu9250_dev_s *)kmm_zalloc(sizeof(struct mpu9250_dev_s));
  if (!priv)
    {
      sndbg("Failed to allocate the device structure!\n");
      return NULL;
    }

  /* Initialize the device state structure */

  sem_init(&priv->exclsem, 0, 1);
  priv->config = config;

#ifdef CONFIG_MPU9250_SPI
  priv->spi = dev;

  /* If this SPI bus is not shared, then we can config it now.
   * If it is shared, then other device could change our config,
   * then just configure before sending data.
   */

#ifdef CONFIG_SPI_OWNBUS
  /* Configure SPI for the MPU9250 */

  SPI_SETMODE(priv->spi, SPIDEV_MODE3);
  SPI_SETBITS(priv->spi, 8);
  SPI_SETFREQUENCY(priv->spi, MPU9250_SPI_MAXFREQUENCY);
#endif

#else
  priv->i2c = dev;

  /* Set the I2C address and frequency.  REVISIT:  This logic would be
   * insufficient if we share the I2C bus with any other devices that also
   * modify the address and frequency.
   */

  I2C_SETADDRESS(dev, config->address, 7);
  I2C_SETFREQUENCY(dev, config->frequency);
#endif

  /* Read and verify the MPU9250 device ID */

  ret = mpu9250_checkid(priv);
  if (ret < 0)
    {
      sndbg("Wrong Device ID!\n");
      kmm_free(priv);
      return NULL;
    }

  /* Generate MPU9250 Software reset */

  mpu9250_reset(priv);

  /* Configure the interrupt output pin to generate interrupts on high or low level. */

  regval  = mpu9250_getreg8(priv, MPU9250_DATA_FORMAT);
#ifdef CONFIG_MPU9250_ACTIVELOW
  regval |= DATA_FMT_INT_INVERT; /* Pin polarity: Active low / falling edge */
#else
  regval &= ~DATA_FMT_INT_INVERT;  /* Pin polarity: Active high / rising edge */
#endif
  mpu9250_putreg8(priv, MPU9250_DATA_FORMAT, regval);

  /* Attach the MPU9250 interrupt handler. */

  config->attach(config, (mpu9250_handler_t)mpu9250_interrupt, (FAR void *)priv);

  /* Leave standby mode */

  mpu9250_putreg8(priv, MPU9250_POWER_CTL, POWER_CTL_MEASURE);

  config->clear(config);
  config->enable(config, true);

  /* Enable interrupts */

  mpu9250_putreg8(priv, MPU9250_INT_ENABLE, INT_DATA_READY);

  /* Return our private data structure as an opaque handle */

  return (MPU9250_HANDLE)priv;
}

#endif /* CONFIG_SENSORS_MPU9250 */
