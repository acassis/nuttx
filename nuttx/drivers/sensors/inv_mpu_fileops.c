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
#include <nuttx/arch.h>

#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>

#include "nuttx/sensors/inv_mpu.h"

#if defined(CONFIG_SENSORS_INVENSENSE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mpu_dev_s {
    struct mpu_inst_s* inst;
    sem_t exclsem;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Character driver methods */

static int     mpu_open(FAR struct file *filep);
static int     mpu_close(FAR struct file *filep);
static ssize_t mpu_read(FAR struct file *filep, FAR char *buffer, size_t len);
static int     mpu_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int     mpu_poll(FAR struct file *filep, struct pollfd *fds, bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_mpu_fops =
{
  mpu_open,     /* open */
  mpu_close,    /* close */
  mpu_read,     /* read */
  0,            /* write */
  0,            /* seek */
  mpu_ioctl,    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  mpu_poll      /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int mpu_open(FAR struct file *filep)
{
    int ret = -EINVAL;
    FAR struct inode      *inode;
    FAR struct mpu_dev_s  *priv;

    DEBUGASSERT(filep);
    inode = filep->f_inode;
    DEBUGASSERT(inode && inode->i_private);
    priv  = (FAR struct mpu_dev_s *)inode->i_private;

    /* TODO: .... */
    UNUSED(priv);

    return ret;
}

/****************************************************************************
 * Name: mpu9250_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int mpu_close(FAR struct file *filep)
{
    int ret = -EINVAL;
    FAR struct inode      *inode;
    FAR struct mpu_dev_s  *priv;

    DEBUGASSERT(filep);
    inode = filep->f_inode;
    DEBUGASSERT(inode && inode->i_private);
    priv  = (FAR struct mpu_dev_s *)inode->i_private;

    /* TODO: .... */
    UNUSED(priv);

    return ret;
}

/****************************************************************************
 * Name: mpu9250_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t mpu_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode      *inode;
  FAR struct mpu_dev_s  *priv;
  int                   ret;
  int                   more;

  snvdbg("len=%d\n", len);

  DEBUGASSERT(filep);
  inode = filep->f_inode;
  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct mpu_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->exclsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Read accelerometer X Y Z axes */

  ret = mpu_read_fifo_stream(priv->inst, len, (uint8_t*)buffer, &more);

  sem_post(&priv->exclsem);

  if ( ret < 0 )
      return ret;

  return more;
}

/****************************************************************************
 * Name: mpu_ioctl
 *
 * Description:
 *  The invensense MPU ioctl handler
 *
 ****************************************************************************/

static int mpu_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
    int ret = -EINVAL;
    FAR struct inode      *inode;
    FAR struct mpu_dev_s  *priv;

    DEBUGASSERT(filep);
    inode = filep->f_inode;
    DEBUGASSERT(inode && inode->i_private);
    priv  = (FAR struct mpu_dev_s *)inode->i_private;

    switch (cmd)
    {

        default:
        /* TODO: .... */
        UNUSED(priv);
        UNUSED(arg);

    }
    return ret;
}

/****************************************************************************
 * Name: mpu_poll
 *
 * Description:
 *  The invensense MPU ioctl handler
 *
 ****************************************************************************/

static int mpu_poll(FAR struct file *filep, struct pollfd *fds, bool setup)
{
    int ret = -EINVAL;
    FAR struct inode      *inode;
    FAR struct mpu_dev_s  *priv;

    DEBUGASSERT(filep);
    inode = filep->f_inode;
    DEBUGASSERT(inode && inode->i_private);
    priv  = (FAR struct mpu_dev_s *)inode->i_private;

    /* TODO: .... */
    UNUSED(priv);
    UNUSED(fds);
    UNUSED(setup);

    return ret;
}

#if 0
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
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu_register
 *
 * Description:
 *   regiter invensense mpu ic.
 *
 * Input Parameters:
 *   path      - path to register this device
 *   minor     - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int mpu_register(struct mpu_inst_s* inst,const char *path ,int minor)
{
  FAR struct mpu_dev_s *priv;
  int ret;

  /* Allocate the MPU9250 driver instance */

  priv = (FAR struct mpu_dev_s *)kmm_zalloc(sizeof(struct mpu_dev_s));
  if (!priv)
    {
      sndbg("Failed to allocate the device structure!\n");
      return -ENOMEM;
    }

  /* Initialize the device state structure */

  sem_init(&priv->exclsem, 0, 1);
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

  ret = register_driver(path, &g_mpu_fops, 0666, priv);
  if (ret < 0)
    {
      sndbg("ERROR: Failed to register driver %s: %d\n", devname, ret);
      sem_post(&priv->exclsem);
      return ret;
    }

  return ret;
}

#endif /* CONFIG_SENSORS_INVENSENSE */
