/****************************************************************************
 * configs/efm32gg-pnbfano/src/suspensions.c
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
#include <arch/board/suspensions.h>

#include <string.h>
#include <poll.h>
#include <errno.h>
#include <nuttx/kmalloc.h>

#include "up_arch.h"
#include "efm32gg-pnbfano.h"

#define EFM32_SUPENSIONS_LOG(...)
//#define EFM32_SUPENSIONS_LOG(lvl,...)    lldbg(__VA_ARGS__)
//#define EFM32_SUPENSIONS_LOG(...)        syslog(__VA_ARGS__)


/****************************************************************************
 * Fileops Prototypes and Structures
 ****************************************************************************/

typedef FAR struct file file_t;

static int efm32_suspensions_open(      file_t * filep);
static int efm32_suspensions_close(     file_t * filep);
static int efm32_suspensions_ioctl(     file_t * filep, int cmd, unsigned long arg );

static const struct file_operations suspensions_ops =
{
    .open   = efm32_suspensions_open,
    .close  = efm32_suspensions_close,
    .write  = NULL,                
    .seek   = NULL,
    .ioctl  = efm32_suspensions_ioctl, 
};


/****************************************************************************
 * Name: efm32_suspensions_t
 * Description:
 *  variable of keypad
 ****************************************************************************/
typedef struct 
{
    /* open counter */

    int     open_count;

    /* Chronometer event from start */

    sem_t   mutex;
	

}efm32_suspensions_t;

/****************************************************************************
 * Name: efm32_suspensions
 * Description:
 *  variable of keypad instance.
 ****************************************************************************/
efm32_suspensions_t* efm32_suspensions;

/************************************************************************************
 * Name: efm32_suspensions_takesem
 ************************************************************************************/

static int efm32_suspensions_takesem(FAR sem_t *sem, bool errout)
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
 * Name: efm32_suspensions_givesem
 ******************************************************************************/

#define efm32_suspensions_givesem(sem) (void)sem_post(sem)



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
int efm32_suspensions_init( int i2c_address, struct i2c_dev_s * i2c );
{
    irqstate_t flags;
    efm32_suspensions_t *dev;

    /* Disable interrupts until we are done.  This guarantees that the
     * following operations are atomic.
     */

    ASSERT(efm32_suspensions == NULL);

    dev = (efm32_suspensions_t*)kmm_malloc(sizeof(efm32_suspensions_t));
    if ( dev == NULL )
    {
        EFM32_SUPENSIONS_LOG(LOG_ERR,"Cannot allocate it!\n");
        return -ENODEV;
    }

    flags = irqsave();

    memset(dev,0,sizeof(*dev));

    //dev->open_count = 0; already done */
    //dev->poll_sem = NULL; already done */
    sem_init(&dev->mutex,  0, 1);

    efm32_configgpio(GPIO_VBAT);
    efm32_configgpio(GPIO_VTEMP);

    (void)irq_attach(EFM32_IRQ_ACMP, efm32_acomp_irq);

    ASSERT(efm32_suspensions == NULL);

    efm32_suspensions = dev;

    irqrestore(flags);


    return register_driver("/dev/suspensions0", &suspensions_ops, 0444, dev);

}


/****************************************************************************
 * Name: efm32_suspensions_open
 ****************************************************************************/

static int efm32_suspensions_open(file_t * filep)
{
    int res;
    FAR struct inode *inode = filep->f_inode;
    FAR efm32_suspensions_t *dev    = inode->i_private;

    ASSERT( dev != NULL );

    res = efm32_suspensions_takesem(&dev->mutex, true);
    if (res < 0)
        return res;

    dev->open_count++;

    if ( dev->open_count == 1 )
    {
        /* TODO: efm32_suspensions_open */
    }

    efm32_suspensions_givesem( &dev->mutex );

    return OK;
}

/****************************************************************************
 * Name: efm32_suspensions_close
 ****************************************************************************/

static int efm32_suspensions_close(file_t * filep)
{
    int res;
    FAR struct inode *inode = filep->f_inode;
    FAR efm32_suspensions_t *dev    = inode->i_private;

    res = efm32_suspensions_takesem(&dev->mutex, true);
    if (res < 0)
        return res;

    dev->open_count--;

    DEBUGASSERT(dev->open_count >= 0);

    if ( dev->open_count == 0 )
    {
        /* TODO: efm32_suspensions_close */
    }

    efm32_suspensions_givesem( &dev->mutex );

    return OK;
}


/****************************************************************************
 * Name: efm32_suspensions_ioctl
 ****************************************************************************/

static int efm32_suspensions_ioctl(file_t *filep, int cmd, unsigned long arg)
{
    int res;
    FAR struct inode *inode         = filep->f_inode;
    FAR efm32_suspensions_t *dev    = inode->i_private;

    irqstate_t flags;

    ASSERT( dev != NULL );

    res = efm32_gpio_pps_takesem(&dev->mutex, true);
    if (res < 0)
        return res;

    switch(cmd)
    {
        case SUPENSIONS_VBAT: /* VBAT */
            break;
        case SUPENSIONS_TEMP: /* Temperature */
            break;
        default:
            return -EINVAL;
    }

    efm32_gpio_pps_givesem( &dev->mutex );

    return res;
}




