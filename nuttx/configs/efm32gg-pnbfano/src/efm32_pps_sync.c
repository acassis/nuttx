/****************************************************************************
 * configs/efm32gg-pnbfano/src/efm32_pps_sync.c
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
#include <arch/board/pps.h>

#include <string.h>
#include <poll.h>
#include <errno.h>
#include <nuttx/kmalloc.h>

#include "up_arch.h"
#include "efm32_gpio.h"
#include "efm32gg-pnbfano.h"

#define EFM32_GPIO_PPS_LOG(...)
//#define EFM32_GPIO_PPS_LOG(lvl,...) lldbg(__VA_ARGS__)
//#define EFM32_GPIO_PPS_LOG(...) syslog(__VA_ARGS__)

#ifndef CONFIG_EFM32_GPIO_PPS_BUFSIZE
#  define CONFIG_EFM32_GPIO_PPS_BUFSIZE 64
#endif

/****************************************************************************
 * Fileops Prototypes and Structures
 ****************************************************************************/

typedef FAR struct file file_t;

static int efm32_gpio_pps_open(file_t * filep);
static int efm32_gpio_pps_close(file_t * filep);
static ssize_t efm32_gpio_pps_read(file_t * filep, 
                                      FAR char *buf, 
                                      size_t buflen
                                     );
static int efm32_gpio_pps_ioctl(FAR struct file *filep,
                                   int cmd, 
                                   unsigned long arg
                                  );
#ifndef CONFIG_DISABLE_POLL
static int efm32_gpio_pps_poll(file_t * filep, 
                                  FAR struct pollfd *fds, 
                                  bool setup
                                 );
#endif

static const struct file_operations gpio_pps_ops =
{
    .open   = efm32_gpio_pps_open,
    .close  = efm32_gpio_pps_close,
    .read   = efm32_gpio_pps_read,                 
    .write  = NULL,                
    .seek   = NULL,
    .ioctl  = efm32_gpio_pps_ioctl, 
#ifndef CONFIG_DISABLE_POLL
    .poll   = efm32_gpio_pps_poll, 
#endif
};


/****************************************************************************
 * Name: efm32_gpio_pps_t
 * Description:
 *  variable of keypad
 ****************************************************************************/
typedef struct 
{
    /* open counter */

    int     open_count;

    /* Poll event semaphore */

    sem_t   *poll_sem;

    /* Chronometer event from start */

    sem_t   mutex;
	
    /* number of pps found without shift */

    int     pps_ok_nbr;

    /* last pps received */

    struct timespec last_pps;

    /* fifo semaphore */

    sem_t   rd_sem;

    /* fifo read index */

    int     rd_idx;

    /* fifo write index */

    int     wr_idx;

    /* fifo data */

    pps_t buf[CONFIG_EFM32_GPIO_PPS_BUFSIZE];

}efm32_gpio_pps_t;

/****************************************************************************
 * Name: efm32_gpio_pps
 * Description:
 *  variable of keypad instance.
 ****************************************************************************/
efm32_gpio_pps_t* efm32_gpio_pps;

/************************************************************************************
 * Name: efm32_gpio_pps_takesem
 ************************************************************************************/

static int efm32_gpio_pps_takesem(FAR sem_t *sem, bool errout)
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

/************************************************************************************
 * Name: efm32_gpio_pps_givesem
 ************************************************************************************/

#define efm32_gpio_pps_givesem(sem) (void)sem_post(sem)


/****************************************************************************
 * Name: efm32_gpio_pps_level
 ****************************************************************************/

int efm32_gpio_pps_level(FAR efm32_gpio_pps_t *dev)
{
    int level = dev->wr_idx - dev->rd_idx;

    if ( level < 0 )
        level += CONFIG_EFM32_GPIO_PPS_BUFSIZE;

    return level;
}

/******************************************************************************
 * Name: timeval_subtract 
 *
 * result = X - Y,
 * storing the result in RESULT.
 * Return 1 if the difference is negative, otherwise 0. 
 *****************************************************************************/


static inline bool timespec_subtract (struct timespec *result, 
                                      struct timespec *x, 
                                      struct timespec *y
                                     )
{
    if ( x->tv_sec > y->tv_sec )
    {
        if (x->tv_nsec >= y->tv_nsec) 
        {
            result->tv_sec = x->tv_sec - y->tv_sec;
            result->tv_nsec = x->tv_nsec - y->tv_nsec;
        } 
        else 
        {
            result->tv_sec = x->tv_sec - y->tv_sec - 1;
            result->tv_nsec = 1000000000+x->tv_nsec - y->tv_nsec;
        }
        return false;
    }
    else if ( x->tv_sec < y->tv_sec )
    {

        if (y->tv_nsec >= x->tv_nsec) 
        {
            result->tv_sec = y->tv_sec - x->tv_sec;
            result->tv_nsec = y->tv_nsec - x->tv_nsec;
        } 
        else 
        {
            result->tv_sec = y->tv_sec - x->tv_sec - 1;
            result->tv_nsec = 1000000000+y->tv_nsec - x->tv_nsec;
        }
        return true;
    }
    else
    {
        result->tv_sec = 0;
        if (x->tv_nsec >= y->tv_nsec) 
        {
            result->tv_nsec = x->tv_nsec - y->tv_nsec;
            return false;
        } 
        else 
        {
            result->tv_nsec = y->tv_nsec - x->tv_nsec;
            return true;
        }
    }

}
 
/****************************************************************************
 * irq handler
 ****************************************************************************/
int efm32_gpio_pps_irq(int irq, FAR void* context)
{
    (void)context;
    (void)irq;
    bool    negatif;
    struct timespec tp;
    struct timespec diff;
    pps_t *ptr;
    efm32_gpio_pps_t *dev = efm32_gpio_pps;

    if ( clock_gettime(CLOCK_REALTIME,&tp) < 0 )
    {
        return -1;
    }

    ASSERT(dev != NULL);

    /* next pps so one second more */

    dev->last_pps.tv_sec++;

    negatif = timespec_subtract(&diff,&dev->last_pps,&tp);

    EFM32_GPIO_PPS_LOG(LOG_DEBUG,"tp : %d.%09d last %d.%09d diff %c%d.%09d\n",
                       tp.tv_sec,tp.tv_nsec,
                       dev->last_pps.tv_sec,dev->last_pps.tv_nsec,
                       (negatif)?'-':' ',
                       diff.tv_sec,diff.tv_nsec
                      );

    if ( diff.tv_sec != 0 )
    {
        EFM32_GPIO_PPS_LOG(LOG_WARNING,"PPS Missed ! \n");
        if ( dev->pps_ok_nbr < 0 )
            return -1; /* pps lost event already sent */

        /* missed pps */
        dev->last_pps.tv_sec  = tp.tv_sec;
        dev->last_pps.tv_nsec = tp.tv_nsec;
        dev->pps_ok_nbr = -1;
    }
    else if ( diff.tv_nsec == 0 ) 
    {
        /* pps synchronised */
        if ( dev->pps_ok_nbr >= 0 )
        {
            dev->pps_ok_nbr++;
            return 0; /* event already sent */
        }
    }

    if (efm32_gpio_pps_level(dev) >= CONFIG_EFM32_GPIO_PPS_BUFSIZE)
    {
        EFM32_GPIO_PPS_LOG(LOG_WARNING,"Buffer overflow\n");
        return -1; 
    }

    ptr = &dev->buf[dev->wr_idx];

    if ( negatif )
        ptr->shift_nsec = -diff.tv_nsec;
    else
        ptr->shift_nsec = diff.tv_nsec;

    ptr->pps_ok_nbr = dev->pps_ok_nbr;
    ptr->tp = tp;

    dev->pps_ok_nbr = 0;
    dev->last_pps = tp;

    dev->wr_idx++;
    if ( dev->wr_idx >= CONFIG_EFM32_GPIO_PPS_BUFSIZE )
        dev->wr_idx = 0;

    sem_post(&dev->rd_sem);

    /* add event to waiting semaphore */
    if ( dev->poll_sem )
    {
        sem_post( dev->poll_sem );
    }

    return 0;
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
int efm32_gpio_pps_init( void )
{
    irqstate_t flags;
    efm32_gpio_pps_t *dev;

    /* Disable interrupts until we are done.  This guarantees that the
     * following operations are atomic.
     */

    ASSERT(efm32_gpio_pps == NULL);

    dev = (efm32_gpio_pps_t*)kmm_malloc(sizeof(efm32_gpio_pps_t));
    if ( dev == NULL )
    {
        EFM32_GPIO_PPS_LOG(LOG_ERR,"Cannot allocate it!\n");
        return -ENODEV;
    }

    flags = irqsave();

    memset(dev,0,sizeof(*dev));

    //dev->open_count = 0; already done */
    //dev->poll_sem = NULL; already done */
    sem_init(&dev->rd_sem, 0, 0);
    sem_init(&dev->mutex,  0, 1);

    efm32_configgpio(GPIO_PPS);
    efm32_gpioirq(GPIO_PPS);
    (void)irq_attach(IRQ_PPS, efm32_gpio_pps_irq);

    ASSERT(efm32_gpio_pps == NULL);

    efm32_gpio_pps = dev;

    irqrestore(flags);


    return register_driver("/dev/pps0", 
                           &gpio_pps_ops, 
                           0444, 
                           dev
                          );

}


/****************************************************************************
 * Name: efm32_gpio_kbd_open
 ****************************************************************************/

static int efm32_gpio_pps_open(file_t * filep)
{
    int res;
    FAR struct inode *inode     = filep->f_inode;
    FAR efm32_gpio_pps_t *dev    = inode->i_private;

    ASSERT( dev != NULL );

    res = efm32_gpio_pps_takesem(&dev->mutex, true);
    if (res < 0)
        return res;

    dev->open_count++;

    if ( dev->open_count == 1 )
        efm32_gpioirqenable(IRQ_PPS);

    efm32_gpio_pps_givesem( &dev->mutex );

    return OK;
}

/****************************************************************************
 * Name: efm32_gpio_kbd_close
 ****************************************************************************/

static int efm32_gpio_pps_close(file_t * filep)
{
    int res;
    FAR struct inode *inode     = filep->f_inode;
    FAR efm32_gpio_pps_t *dev    = inode->i_private;

    res = efm32_gpio_pps_takesem(&dev->mutex, true);
    if (res < 0)
        return res;

    dev->open_count--;

    DEBUGASSERT(dev->open_count >= 0);

    if ( dev->open_count == 0 )
        efm32_gpioirqdisable(IRQ_PPS);

    efm32_gpio_pps_givesem( &dev->mutex );

    return OK;
}


/****************************************************************************
 * Name: efm32_gpio_kbd_read
 ****************************************************************************/

static int efm32_gpio_pps_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
    int res;
    FAR struct inode *inode     = filep->f_inode;
    FAR efm32_gpio_pps_t *dev    = inode->i_private;

    irqstate_t flags;

    ASSERT( dev != NULL );

    res = efm32_gpio_pps_takesem(&dev->mutex, true);
    if (res < 0)
        return res;

    switch(cmd)
    {
        case 0: /* reset pps */
                flags = irqsave();
                dev->rd_idx = 0;
                dev->wr_idx = 0;
                dev->pps_ok_nbr = 0;
                dev->last_pps.tv_nsec = -1;
                dev->last_pps.tv_sec  = -1;
                irqrestore(flags);
            break;
        default:
            return -EINVAL;
    }

    efm32_gpio_pps_givesem( &dev->mutex );

    return res;
}

/****************************************************************************
 * Name: efm32_gpio_pps_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int efm32_gpio_pps_poll(file_t * filep, FAR struct pollfd *fds, bool setup)
{
    int res;
    FAR struct inode *inode     = filep->f_inode;
    FAR efm32_gpio_pps_t *dev    = inode->i_private;

    res = efm32_gpio_pps_takesem(&dev->mutex, true);
    if (res < 0)
        return res;

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

        if ( efm32_gpio_pps_level(dev) > 0 )
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
        if ( efm32_gpio_pps_level(dev) > 0 )
        {
            fds->revents |= (fds->events & POLLIN);
        }
        dev->poll_sem = NULL;
    }
errout:
    efm32_gpio_pps_givesem(&dev->mutex);
    return res;
}
#endif


/****************************************************************************
 * Name: efm32_gpio_pps_read
 ****************************************************************************/

static ssize_t efm32_gpio_pps_read(file_t * filep, FAR char *buf, size_t buflen)
{
    int res;
    FAR struct inode *inode      = filep->f_inode;
    FAR efm32_gpio_pps_t *dev    = inode->i_private;

    pps_t pps;
    ssize_t size = 0;

    if ( dev == NULL )
    {
        EFM32_GPIO_PPS_LOG(LOG_ERR,"Not initialized!\n");
        return -EINVAL;
    }

    res = efm32_gpio_pps_takesem(&dev->mutex, true);
    if (res < 0)
        return res;

    while (size < buflen)
    {
        int len = buflen;

        /* first lock it then only try lock */

        if ( size == 0 )
            sem_wait( &dev->rd_sem);
        else if ( sem_trywait( &dev->rd_sem ) < 0 )
            break;

        irqstate_t saved_state;

        saved_state = irqsave();

        pps = dev->buf[dev->rd_idx++];
        if (dev->rd_idx >= CONFIG_EFM32_GPIO_PPS_BUFSIZE )
            dev->rd_idx=0;

        irqrestore(saved_state);

        if ( len > sizeof(pps) )
            len = sizeof(pps);

        EFM32_GPIO_PPS_LOG(LOG_NOTICE,
                              "Read %d bytes of shift %5d.%06d msec, pps sync %3d, timespec %8d.%3d\n",
                              len,
                              pps.shift_nsec/1000000,
                              pps.shift_nsec%1000000,
                              pps.pps_ok_nbr,
                              pps.tp.tv_sec,
                              pps.tp.tv_nsec/1000000
                             );

        memcpy(&buf[size],&pps,len);

        size += len; 
    }

    efm32_gpio_pps_givesem( &dev->mutex );

    return size;
}

