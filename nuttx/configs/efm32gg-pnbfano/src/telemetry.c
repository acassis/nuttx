/****************************************************************************
 * configs/efm32gg-pnbfano/src/telemetry.c
 *
 *   Copyright (C) 2014 Pierre-noel Bouteville. All rights reserved.
 *   Author: Pierre-noel Bouteville <pnb990@gmail.com>
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

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/irq.h>
#include <arch/board/board.h>
#include <arch/board/telemetry.h>

#include <string.h>
#include <poll.h>
#include <errno.h>
#include <nuttx/kmalloc.h>

#include "up_arch.h"
#include "efm32gg-pnbfano.h"

//#define EFM32_TELEMETRY_LOG(...)
//#define EFM32_TELEMETRY_LOG(lvl,...)    lldbg(__VA_ARGS__)
#define EFM32_TELEMETRY_LOG(...)        syslog(__VA_ARGS__)


/****************************************************************************
 * Fileops Prototypes and Structures
 ****************************************************************************/

typedef FAR struct file file_t;

static int efm32_telemetry_open(      file_t * filep);
static int efm32_telemetry_close(     file_t * filep);
static int efm32_telemetry_ioctl(     file_t * filep, int cmd, unsigned long arg );
#ifndef CONFIG_DISABLE_POLL
static int efm32_telemetry_poll(file_t * filep, 
                                FAR struct pollfd *fds, 
                                bool setup
                               );
#endif

static void efm32_telemetry_set_next_poll(struct mpu_dev_s* dev);

static const struct file_operations telemetry_ops =
{
    .open   = efm32_telemetry_open,
    .close  = efm32_telemetry_close,
    .write  = NULL,                
    .seek   = NULL,
    .ioctl  = efm32_telemetry_ioctl, 
#ifndef CONFIG_DISABLE_POLL
    .poll   = efm32_telemetry_poll, 
#endif
};


/****************************************************************************
 * Name: efm32_telemetry_t
 * Description:
 *  variable of keypad
 ****************************************************************************/
typedef struct 
{
    /* open counter */

    int     open_count;

    /* Poll event semaphore */

    sem_t   *poll_sem;

    /* mutex protection */

    sem_t   mutex;
	
    /* fifo semaphore */

    sem_t   rd_sem;

    /* fifo read index */

    int     rd_idx;

    /* fifo write index */

    int     wr_idx;

    /* fifo data */

    telemetry_t buf[CONFIG_EFM32_TELEMETRY_BUFSIZE];

}efm32_telemetry_t;

/****************************************************************************
 * Name: efm32_telemetry
 * Description:
 *  variable of keypad instance.
 ****************************************************************************/
efm32_telemetry_t* efm32_telemetry;

/************************************************************************************
 * Name: efm32_telemetry_takesem
 ************************************************************************************/

static int efm32_telemetry_takesem(FAR sem_t *sem, bool errout)
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

/*******************************************************************************
 * Name: efm32_telemetry_givesem
 ******************************************************************************/

#define efm32_telemetry_givesem(sem) (void)sem_post(sem)


/****************************************************************************
 * Name: efm32_chrono_level
 ****************************************************************************/

inline int efm32_telemetry_level(FAR efm32_gpio_chrono_t *dev)
{
    int level = dev->wr_idx - dev->rd_idx;

    if ( level < 0 )
        level += CONFIG_EFM32_TELEMETRY_BUFSIZE;

    return level;
}


/****************************************************************************
 * worker handler
 ****************************************************************************/
int efm32_telemetry_worker(int irq, FAR void* context)
{
    (void)context;
    (void)irq;
    struct timespec tp;
    chrono_t *ptr;
    efm32_telemetry_t *dev = efm32_telemetry;

    if ( clock_gettime(CLOCK_REALTIME,&tp) < 0 )
    {
        return -1;
    }

    ASSERT(dev != NULL);

    if (efm32_gpio_chrono_level(dev) >= CONFIG_EFM32_TELEMETRY_BUFSIZE)
    {
        EFM32_TELEMETRY_LOG(LOG_WARNING,"Buffer overflow\n");
        return -1; 
    }


    ptr = &dev->buf[dev->wr_idx];

    ptr->tp = tp;
    ptr->breaking = efm32_gpioread(GPIO_BREAKING);
    ptr->suspension_rear    = 
    ptr->suspension_front   = 


    EFM32_TELEMETRY_LOG(LOG_NOTICE,
                        "Timespec %8d.%3d\n",
                        tp.tv_sec,
                        tp.tv_nsec/1000000
                       );

    dev->wr_idx++;
    if ( dev->wr_idx >= CONFIG_EFM32_GPIO_CHRONO_BUFSIZE )
        dev->wr_idx = 0;

    sem_post(&dev->rd_sem);

    /* add event to waiting semaphore */
    if ( dev->poll_sem )
    {
        sem_post( dev->poll_sem );
    }

    efm32_telemetry_set_next_poll(dev);

    return 0;
}

/****************************************************************************
 * Name: efm32_telemetry_set_next_poll
 ****************************************************************************/
static void efm32_telemetry_set_next_poll(struct mpu_dev_s* dev)
{
    if ( work_queue(HPWORK, 
                    &(dev->work), 
                    efm32_telemetry_worker,
                    dev, 
                    dev->tick
                    ) != OK )
    {
        EFM32_TELEMETRY_LOG("Cannot register worker !\n");
        return;
    }
}

/****************************************************************************
 * Name: efm32_gpio_pps_init
 *
 * Description:
 *  Initialize All GPIO for key pad.
 * Input parameters:
 *  _key_map    - first key mapping of mapping GPIO<=>Key list.
 *                    to Finish list set Pin with negative value.
 * Returned Value:
 *   None (User allocated instance initialized).
 ****************************************************************************/
int efm32_telemetry_init( int i2c_address, struct i2c_dev_s * i2c );
{
    irqstate_t flags;
    efm32_telemetry_t *dev;

    /* Disable interrupts until we are done.  This guarantees that the
     * following operations are atomic.
     */

    ASSERT(efm32_telemetry == NULL);

    dev = (efm32_telemetry_t*)kmm_malloc(sizeof(efm32_telemetry_t));
    if ( dev == NULL )
    {
        EFM32_SUPENSIONS_LOG(LOG_ERR,"Cannot allocate it!\n");
        return -ENODEV;
    }

    flags = irqsave();

    memset(dev,0,sizeof(*dev));

    //dev->open_count = 0; already done */
    //dev->poll_sem = NULL; already done */
    dev->tick = MSEC2TICK(1000); /* 1 second by default */
    sem_init(&dev->mutex,  0, 1);

    ASSERT(efm32_telemetry == NULL);

    efm32_telemetry = dev;

    irqrestore(flags);

    efm32_telemetry_set_next_poll(dev);

    return register_driver("/dev/telemetry0", &telemetry_ops, 0444, dev);

}


/****************************************************************************
 * Name: efm32_telemetry_open
 ****************************************************************************/

static int efm32_telemetry_open(file_t * filep)
{
    int res;
    FAR struct inode *inode = filep->f_inode;
    FAR efm32_telemetry_t *dev    = inode->i_private;

    ASSERT( dev != NULL );

    res = efm32_telemetry_takesem(&dev->mutex, true);
    if (res < 0)
        return res;

    dev->open_count++;

    if ( dev->open_count == 1 )
    {
        /* TODO: efm32_telemetry_open */
    }

    efm32_telemetry_givesem( &dev->mutex );

    return OK;
}

/****************************************************************************
 * Name: efm32_telemetry_close
 ****************************************************************************/

static int efm32_telemetry_close(file_t * filep)
{
    int res;
    FAR struct inode *inode = filep->f_inode;
    FAR efm32_telemetry_t *dev    = inode->i_private;

    res = efm32_telemetry_takesem(&dev->mutex, true);
    if (res < 0)
        return res;

    dev->open_count--;

    DEBUGASSERT(dev->open_count >= 0);

    if ( dev->open_count == 0 )
    {
        /* TODO: efm32_telemetry_close */
    }

    efm32_telemetry_givesem( &dev->mutex );

    return OK;
}


/****************************************************************************
 * Name: efm32_telemetry_ioctl
 ****************************************************************************/

static int efm32_telemetry_ioctl(file_t *filep, int cmd, unsigned long arg)
{
    int res;
    FAR struct inode *inode         = filep->f_inode;
    FAR efm32_telemetry_t *dev    = inode->i_private;

    irqstate_t flags;

    ASSERT( dev != NULL );

    res = efm32_gpio_pps_takesem(&dev->mutex, true);
    if (res < 0)
        return res;

    switch(cmd)
    {
        case TELEMETRY_FREQUENCY: 
            /* saturate to 1 second */
            if ( arg > 1000 )
                arg = 1000;
            dev->tick = MSEC2TICK(arg)
            break;
        default:
            return -EINVAL;
    }

    efm32_gpio_pps_givesem( &dev->mutex );

    return res;
}




