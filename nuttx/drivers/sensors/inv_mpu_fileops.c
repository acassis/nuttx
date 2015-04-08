/****************************************************************************
 * drivers/sensors/inv_mpu_fileops.c
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

#include <string.h>
//#include <unistd.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include "nuttx/sensors/inv_mpu.h"

#ifdef CONFIG_INVENSENSE_DMP
#   include "inv_mpu_dmp.h"
#endif

#if defined(CONFIG_SENSORS_INVENSENSE)
/*******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/ 

#ifndef CONFIG_INVENSENSE_POLLING_MS
#   define CONFIG_INVENSENSE_POLLING_MS 100
#endif

#define MPU_LOG(...)
//#define MPU_LOG(...) lldbg(__VA_ARGS__)
//#define MPU_LOG(...) syslog(LOG_NOTICE,__VA_ARGS__)

#ifndef BOARD_ENABLES_SENSORS
#   define BOARD_ENABLES_SENSORS MPU_XYZ_GYRO|MPU_XYZ_ACCEL
#endif

#ifndef BOARD_DEFAULT_MPU_HZ
#   define BOARD_DEFAULT_MPU_HZ (10) /* in Hz */
#endif

#ifndef BOARD_ENABLES_DMP_FEATURES
#   define BOARD_ENABLES_DMP_FEATURES ( DMP_FEATURE_6X_LP_QUAT      | \
                                        DMP_FEATURE_SEND_RAW_ACCEL  | \
                                        DMP_FEATURE_SEND_RAW_GYRO   )
#endif

#ifndef BOARD_DEFAULT_DMP_HZ
#   define BOARD_DEFAULT_DMP_HZ (10) /* in Hz */
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef FAR struct file file_t;

struct mpu_dev_s {
    struct mpu_inst_s* inst;
#ifdef CONFIG_INVENSENSE_DMP
    struct dmp_s *dmp;
#endif
    uint8_t mpu_int_status;
    uint8_t dmp_int_status;
    sem_t   exclsem;
#ifndef CONFIG_DISABLE_POLL
    sem_t   *poll_sem;
#ifndef BOARD_INV_MPU_IRQ
    struct work_s work;
#endif
#endif
};

/****************************************************************************
 * Private Functions prototypes
 ****************************************************************************/

/* Character driver methods */

static int     mpu_open(FAR struct file *filep);
static int     mpu_close(FAR struct file *filep);
static ssize_t mpu_read(FAR struct file *filep, FAR char *buffer, size_t len);
static int     mpu_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int     mpu_poll(FAR struct file *filep, struct pollfd *fds, bool setup);
#endif

#ifndef CONFIG_DISABLE_POLL
static void mpu_worker(FAR void *arg);
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

/******************************************************************************
 * Name: mpu_takesem
 ******************************************************************************/

static int mpu_takesem(FAR sem_t *sem, bool errout)
{
  /* Loop, ignoring interrupts, until we have successfully acquired the semaphore */

  while (sem_wait(sem) != OK)
    {
      /* The only case that an error should occur here is if the wait was awakened
       * by a signal.
       */

      ASSERT(get_errno() == EINTR);

      /* When the signal is received, should we errout? Or should we just continue
       * waiting until we have the semaphore?
       */

      if (errout)
        {
          return -EINTR;
        }
    }

  return OK;
}

/******************************************************************************
 * Name: mpu_givensem
 ******************************************************************************/

#define mpu_givesem(sem) (void)sem_post(sem)


/****************************************************************************
 * Name: efm32_lcd_kbd_open
 ****************************************************************************/
#if !defined(CONFIG_DISABLE_POLL) && !defined(BOARD_INV_MPU_IRQ)
static void mpu_set_next_poll(struct mpu_dev_s* dev)
{
    if ( work_queue(HPWORK, 
                    &(dev->work), 
                    mpu_worker,
                    dev, 
                    MSEC2TICK(CONFIG_INVENSENSE_POLLING_MS)
                    ) != OK )
    {
        MPU_LOG("Cannot register worker !\n");
        return;
    }
}
#endif

/****************************************************************************
 * irq handler
 ****************************************************************************/
#ifndef CONFIG_DISABLE_POLL
static void mpu_worker(FAR void *arg)
{
    struct mpu_dev_s *dev = (struct mpu_dev_s*)arg;

    if ( mpu_takesem(&dev->exclsem, true) < 0 )
    {
        MPU_LOG("Cannot take semaphore !\n");
        return;
    }

    if ( mpu_get_int_status(dev->inst, &dev->mpu_int_status, 
                             &dev->dmp_int_status) < 0 )
    {
        MPU_LOG("Cannot get interrupts status !\n");
    }
    else
    {
        MPU_LOG("mpu_int_status 0x%02X dmp_int_status 0x%02X \n");

        if ( dev->mpu_int_status & MPU_INT_STATUS_DATA_READY )
        {

            MPU_LOG("Data ready !\n");

            /* add event to waiting semaphore */

            if ( dev->poll_sem )
            {
                sem_post( dev->poll_sem );
            }
        }
    }

    mpu_givesem(&dev->exclsem);

#if !defined(BOARD_INV_MPU_IRQ)
    mpu_set_next_poll(dev);
#endif

    return;
}
#endif

/****************************************************************************
 * Name: mpu_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int mpu_open(FAR struct file *filep)
{
    int ret = 0;
    FAR struct inode *inode     = filep->f_inode;
    FAR struct mpu_dev_s *dev   = inode->i_private;

    if ( mpu_takesem(&dev->exclsem, true) < 0 )
    {
        MPU_LOG("Cannot take semaphore !\n");
        return -1;
    }

#ifdef CONFIG_INVENSENSE_DMP
    if ( dev->dmp )
    {
        if ( ret == 0 )
            ret = dmp_enable_feature(dev->dmp,BOARD_ENABLES_DMP_FEATURES);

        if ( ret == 0 )
            ret = dmp_set_fifo_rate(dev->dmp,BOARD_DEFAULT_DMP_HZ);

        if ( ret == 0 )
            ret = mpu_set_dmp_on(dev->inst);
    }
#endif

    mpu_givesem(&dev->exclsem);

    return ret;
}

/****************************************************************************
 * Name: mpu_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int mpu_close(FAR struct file *filep)
{
    int ret;
    FAR struct inode *inode     = filep->f_inode;
    FAR struct mpu_dev_s *dev   = inode->i_private;

    if ( mpu_takesem(&dev->exclsem, true) < 0 )
    {
        MPU_LOG("Cannot take semaphore !\n");
        return -1;
    }

#ifdef CONFIG_INVENSENSE_DMP
    if ( dev->dmp )
    {
        if ( dmp_enable_feature(dev->dmp,0) < 0 )
            MPU_LOG("Cannot disable dmp feature !\n");

        if ( mpu_set_dmp_off(dev->inst) < 0 )
            MPU_LOG("Cannot turn off dmp !\n");
    }
#endif

    ret = mpu_set_sensors_enable(dev->inst,0);

    if ( ret >= 0 )
        ret = mpu_reset_fifo(dev->inst);

    mpu_givesem(&dev->exclsem);

    return ret;
}

/****************************************************************************
 * Name: mpu_poll
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int mpu_poll(file_t * filep, FAR struct pollfd *fds, bool setup)
{

    FAR struct inode *inode     = filep->f_inode;
    FAR struct mpu_dev_s *dev    = inode->i_private;

    int res = 0;

    /* Are we setting up the poll?  Or tearing it down? */

    res = mpu_takesem(&dev->exclsem, true);

    if (res < 0)
    {
        /* A signal received while waiting for access to the poll data
         * will abort the operation.
         */

        return res;
    }

    if (setup)
    {

        fds->revents = 0;
        /* This is a request to set up the poll.  Find an available
         * slot for the poll structure reference
         */

        if ( dev->poll_sem != NULL)
        {
            res = -EINVAL;
            goto errout;
        }

        if( dev->mpu_int_status & MPU_INT_STATUS_DATA_READY )
        {
            fds->revents |= (fds->events & POLLIN);
        }

        if ( fds->revents == 0 )
        {
            dev->poll_sem = fds->sem ;
        }
        else
        {
            sem_post(fds->sem);
            res = 1;
        }
    }
    else if ( dev->poll_sem == fds->sem )
    {
        dev->poll_sem = NULL;
    }
errout:
    mpu_givesem(&dev->exclsem);
    return res;
}
#endif
/****************************************************************************
 * Name: mpu_read
 *
 * Description:
 *   read fifo
 *
 ****************************************************************************/

static ssize_t mpu_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
    FAR struct inode *inode     = filep->f_inode;
    FAR struct mpu_dev_s *dev    = inode->i_private;

    int ret;

    snvdbg("len=%d\n", len);

    /* Get exclusive access to the driver data structure */

    if ( mpu_takesem(&dev->exclsem, true) < 0 )
    {
        MPU_LOG("Cannot take semaphore !\n");
        return -EINTR;
    }


    /* Read accelerometer X Y Z axes */

    ret = mpu_read_fifo_level(dev->inst);

    while ( (len > 0) && (ret > 0) )
    {
#ifdef CONFIG_INVENSENSE_DMP
        if ( dev->dmp )
        {
            if ( len < sizeof(struct mpu_fifo_dmp_s) )
                ret = -1; 
            if ( ret == 0 )
                ret = dmp_read_fifo(dev->dmp, (struct mpu_fifo_dmp_s*)buffer);
            if ( ret > 0 )
                len -= sizeof(struct mpu_fifo_dmp_s);
        }
        else
#endif
        {
            if ( len < sizeof(struct mpu_fifo_mpu_s) )
                ret = -1; 
            if ( ret == 0 )
                ret = mpu_read_fifo(dev->inst, (struct mpu_fifo_mpu_s*)buffer);
            if ( ret > 0 )
                len -= sizeof(struct mpu_fifo_mpu_s);
        }
    }

    mpu_givesem(&dev->exclsem);

    if ( ret < 0 )
        return ret;

    return len;
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

    FAR struct inode *inode     = filep->f_inode;
    FAR struct mpu_dev_s *dev   = inode->i_private;

    switch (cmd)
    {

        case MPU_ENABLE:
            ret = mpu_set_sensors_enable(dev->inst,arg);
            if ( ret == 0 )
                ret = mpu_set_fifo_config(dev->inst,arg);
            break;

        case MPU_FREQUENCY:
            if ( arg < UINT16_MAX )
            {
#ifdef CONFIG_INVENSENSE_DMP
                if ( dev->dmp )
                {
                    ret = dmp_set_fifo_rate(dev->dmp,arg);
                }
                else
#endif
                {
                    ret = mpu_set_sample_rate(dev->inst,arg);
                }
            }
            break;

    }
    return ret;
}


#if BOARD_INV_MPU_IRQ
/****************************************************************************
 * Name: mpu_interrupt
 *
 * Description:
 *  The MPU interrupt handler
 *
 ****************************************************************************/

static void mpu_interrupt(FAR struct mpu_config_s *config, FAR void *arg)
{
  FAR struct mpu_dev_s *priv = (FAR struct mpu_dev_s *)arg;
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
      /* Yes.. Transfer processing to the worker thread.  Since MPU
       * interrupts are disabled while the work is pending, no special
       * action should be required to protect the work queue.
       */

      ret = work_queue(HPWORK, &priv->work, mpu_worker, priv, 0);
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
 * Name: mpu_fileops_init
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

int mpu_fileops_init(struct mpu_inst_s* inst,const char *path ,int minor )
{
    FAR struct mpu_dev_s *dev;
    int ret = 0;

    /* Allocate the MPU driver instance */

    dev = (FAR struct mpu_dev_s *)kmm_zalloc(sizeof(struct mpu_dev_s));
    if (!dev)
    {
        sndbg("Failed to allocate the device structure!\n");
        return -ENOMEM;
    }

    DEBUGASSERT(dev);

    /* Initialize the device state structure */

    memset(dev,0,sizeof(*dev));
    //dev->poll_sem = NULL; /* already done by memset */
    sem_init(&dev->exclsem, 0, 1);
    dev->inst = inst;


    /* Get/set hardware configuration. Start gyro. 
     * Wake up all sensors. 
     */

    if ( ret == 0 )
        ret = mpu_set_sensors_enable(dev->inst, BOARD_ENABLES_SENSORS ); 

    /* Push both gyro and accel data into the FIFO at default rate. */

    if ( ret == 0 )
        ret = mpu_set_fifo_config(dev->inst, BOARD_ENABLES_SENSORS );

    if ( ret == 0 )
        ret = mpu_set_sample_rate(dev->inst, BOARD_DEFAULT_MPU_HZ );

#ifdef CONFIG_INVENSENSE_DMP
    if ( dev->dmp == NULL )
    {
        dev->dmp = dmp_init(dev->inst);
    }
#endif

    /* Register the character driver */

    ret = register_driver(path, &g_mpu_fops, 0666, dev);

    if (ret < 0)
    {
        sndbg("ERROR: Failed to register driver %s: %d\n", devname, ret);
        return ret;
    }

#ifndef CONFIG_DISABLE_POLL
    mpu_set_next_poll(dev);
#endif

    return ret;
}

#endif /* CONFIG_SENSORS_INVENSENSE */
