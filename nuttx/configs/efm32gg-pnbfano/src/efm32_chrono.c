/****************************************************************************
 * configs/efm32gg-pnbfano/src/efm32_chrono.c
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
#include <arch/board/chrono.h>

#include <string.h>
//#include <stdbool.h>
#include <errno.h>
#include <nuttx/kmalloc.h>

#include "up_arch.h"
#include "efm32_gpio.h"
#include "efm32gg-pnbfano.h"

//#define EFM32_GPIO_CHRONO_LOG(...)
//#define EFM32_GPIO_CHRONO_LOG(lvl,...) lldbg(__VA_ARGS__)
#define EFM32_GPIO_CHRONO_LOG(...) syslog(__VA_ARGS__)

#ifndef CONFIG_EFM32_GPIO_CHRONO_BUFSIZE
#  define CONFIG_EFM32_GPIO_CHRONO_BUFSIZE 64
#endif

/****************************************************************************
 * Fileops Prototypes and Structures
 ****************************************************************************/

typedef FAR struct file file_t;

static int efm32_gpio_chrono_open(file_t * filep);
static int efm32_gpio_chrono_close(file_t * filep);
static int efm32_gpio_chrono_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
//static int efm32_gpio_chrono_poll(file_t * filep, FAR struct pollfd *fds, bool setup);
#endif

static const struct file_operations gpio_chrono_ops =
{
    efm32_gpio_chrono_open,  /* open */
    efm32_gpio_chrono_close, /* close */
    NULL,                 /* write */
    NULL,                 /* seek */
    NULL,                 /* ioctl */
    efm32_gpio_chrono_ioctl, /* read */
#ifndef CONFIG_DISABLE_POLL
    NULL,                 /* efm32_gpio_chrono_poll */ /* poll */
#endif
};

/****************************************************************************
 * Name: efm32_gpio_chrono_t
 * Description:
 *  variable of keypad
 ****************************************************************************/
typedef struct 
{
    /* Chronometer event from start */

    sem_t   mutex;

    /* Chronometer event from start */

    int     event_nbr;

    /* start date */

    struct timespec start;

    /* fifo semaphore */

    sem_t   rd_sem;

    /* fifo read index */

    int     rd_idx;

    /* fifo write index */

    int     wr_idx;

    /* fifo data */

    chrono_t buf[CONFIG_EFM32_GPIO_CHRONO_BUFSIZE];

}efm32_gpio_chrono_t;

/****************************************************************************
 * Name: efm32_gpio_chrono
 * Description:
 *  variable of keypad instance.
 ****************************************************************************/
efm32_gpio_chrono_t* efm32_gpio_chrono;


/****************************************************************************
 * irq handler
 ****************************************************************************/
int efm32_gpio_chrono_irq(int irq, FAR void* context)
{
    (void)context;
    (void)irq;
    int level;
    struct timespec tp;
    chrono_t *ptr;

    if ( clock_gettime(CLOCK_REALTIME,&tp) < 0 )
    {
        return -1;
    }

    ASSERT(efm32_gpio_chrono != NULL);

    level = efm32_gpio_chrono->wr_idx - efm32_gpio_chrono->rd_idx;

    if ( level < 0 )
        level += CONFIG_EFM32_GPIO_CHRONO_BUFSIZE;

    if (level >= CONFIG_EFM32_GPIO_CHRONO_BUFSIZE)
    {
        EFM32_GPIO_CHRONO_LOG(LOG_WARNING,"Buffer overflow\n");
        return -1; 
    }

    ptr = &efm32_gpio_chrono->buf[efm32_gpio_chrono->wr_idx];

    ptr->trigged = true;
    ptr->event_nbr = efm32_gpio_chrono->event_nbr++;
    ptr->tp = tp;

    efm32_gpio_chrono->wr_idx++;
    if ( efm32_gpio_chrono->wr_idx >= CONFIG_EFM32_GPIO_CHRONO_BUFSIZE )
        efm32_gpio_chrono->wr_idx = 0;

    sem_post(&efm32_gpio_chrono->rd_sem);

    return 0;
}


/****************************************************************************
 * Name: efm32_gpio_chrono_init
 *
 * Description:
 *  Initialize All GPIO for key pad.
 * Input parameters:
 *  _key_map    - first key mapping of mapping GPIO<=>Key list.
 *                    to Finish list set Pin with negative value.
 * Returned Value:
 *   None (User allocated instance initialized).
 ****************************************************************************/
int efm32_gpio_chrono_init( void )
{
    irqstate_t flags;

    /* Disable interrupts until we are done.  This guarantees that the
     * following operations are atomic.
     */


    ASSERT(efm32_gpio_chrono == NULL);

    efm32_gpio_chrono = (efm32_gpio_chrono_t*)kmm_malloc(sizeof(efm32_gpio_chrono_t));
    if ( efm32_gpio_chrono == NULL )
    {
        EFM32_GPIO_CHRONO_LOG(LOG_ERR,"Cannot allocate it!\n");
        return -ENODEV;
    }


    memset(efm32_gpio_chrono,0,sizeof(*efm32_gpio_chrono));

    sem_init(&efm32_gpio_chrono->rd_sem, 0, 0);
    sem_init(&efm32_gpio_chrono->mutex,  0, 1);

    flags = irqsave();
    efm32_configgpio(GPIO_CHRONO);
    efm32_gpioirq(GPIO_CHRONO);
    (void)irq_attach(GPIO_CHRONO_IRQ, efm32_gpio_chrono_irq);
    efm32_gpioirqenable(GPIO_CHRONO_IRQ);
    irqrestore(flags);

    return register_driver("/dev/chrono0", &gpio_chrono_ops, 0444, NULL);

}


/****************************************************************************
 * Name: efm32_gpio_chrono_open
 ****************************************************************************/

static int efm32_gpio_chrono_open(file_t * filep)
{
    if ( efm32_gpio_chrono == NULL )
    {
        EFM32_GPIO_CHRONO_LOG(LOG_ERR,"Not initialized!\n");
        return -EINVAL;
    }

    return OK;
}

/****************************************************************************
 * Name: efm32_gpio_chrono_close
 ****************************************************************************/

static int efm32_gpio_chrono_close(file_t * filep)
{

    /* nothing to do */

    return OK;
}


/****************************************************************************
 * Name: efm32_gpio_chrono_read
 ****************************************************************************/

static int efm32_gpio_chrono_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
    int res = 0;
    irqstate_t flags;


    if ( efm32_gpio_chrono == NULL )
    {
        EFM32_GPIO_CHRONO_LOG(LOG_ERR,"Not initialized!\n");
        return -EINVAL;
    }

    sem_wait( &efm32_gpio_chrono->mutex );

    switch(cmd)
    {
        case 0: /* restart with number event to start */
            flags = irqsave();
            efm32_gpio_chrono->rd_idx = 0;
            efm32_gpio_chrono->wr_idx = 0;
            efm32_gpio_chrono->event_nbr = (int)arg;
            irqrestore(flags);
            break;
        case 1: /* restart with number event to start */
            flags = irqsave();
            /* TODO */
            *((chrono_t*)arg) = efm32_gpio_chrono->buf[efm32_gpio_chrono->rd_idx];
            irqrestore(flags);
            break;
        default:
            res = -EINVAL;
    }

    sem_post( &efm32_gpio_chrono->mutex );

    return res;
}



