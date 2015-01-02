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

#include <string.h>
//#include <stdbool.h>
#include <errno.h>
#include <nuttx/kmalloc.h>

#include "up_arch.h"
#include "efm32_gpio.h"
#include "efm32gg-pnbfano.h"

//#define EFM32_GPIO_PPS_LOG(...)
//#define EFM32_GPIO_PPS_LOG(lvl,...) lldbg(__VA_ARGS__)
#define EFM32_GPIO_PPS_LOG(...) syslog(__VA_ARGS__)


/****************************************************************************
 * Fileops Prototypes and Structures
 ****************************************************************************/

typedef FAR struct file file_t;

static int efm32_gpio_pps_open(file_t * filep);
static int efm32_gpio_pps_close(file_t * filep);
static int efm32_gpio_pps_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
//static int efm32_gpio_pps_poll(file_t * filep, FAR struct pollfd *fds, bool setup);
#endif

static const struct file_operations gpio_pps_ops =
{
    efm32_gpio_pps_open,  /* open */
    efm32_gpio_pps_close, /* close */
    NULL,                 /* write */
    NULL,                 /* seek */
    NULL,                 /* ioctl */
    efm32_gpio_pps_ioctl, /* read */
#ifndef CONFIG_DISABLE_POLL
    NULL,                 /* efm32_gpio_pps_poll */ /* poll */
#endif
};


/****************************************************************************
 * Name: efm32_gpio_pps_t
 * Description:
 *  variable of keypad
 ****************************************************************************/
typedef struct 
{
    sem_t   mutex;
    bool    pps;
    time_t  next_time;
    int     pps_nsec;
}efm32_gpio_pps_t;

/****************************************************************************
 * Name: efm32_gpio_pps
 * Description:
 *  variable of keypad instance.
 ****************************************************************************/
efm32_gpio_pps_t* efm32_gpio_pps;

/****************************************************************************
 * irq handler
 ****************************************************************************/
int efm32_gpio_pps_irq(int irq, FAR void* context)
{
    (void)context;
    (void)irq;
    struct timespec tp;

    if ( clock_gettime(CLOCK_REALTIME,&tp) < 0 )
    {
        return -1;
    }

    efm32_gpio_pps->pps_nsec = tp.tv_nsec;
    efm32_gpio_pps->pps = true;

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

    /* Disable interrupts until we are done.  This guarantees that the
     * following operations are atomic.
     */


    ASSERT(efm32_gpio_pps == NULL);

    efm32_gpio_pps = (efm32_gpio_pps_t*)kmm_malloc(sizeof(efm32_gpio_pps_t));
    if ( efm32_gpio_pps == NULL )
    {
        EFM32_GPIO_PPS_LOG(LOG_ERR,"Cannot allocate it!\n");
        return -ENODEV;
    }


    memset(efm32_gpio_pps,0,sizeof(*efm32_gpio_pps));

    sem_init(&efm32_gpio_pps->mutex, 0, 1);

    flags = irqsave();
    efm32_configgpio(GPIO_PPS);
    efm32_gpioirq(GPIO_PPS);
    (void)irq_attach(GPIO_PPS_IRQ, efm32_gpio_pps_irq);
    efm32_gpioirqenable(GPIO_PPS_IRQ);
    irqrestore(flags);

    return register_driver("/dev/pps0", &gpio_pps_ops, 0444, NULL);

}


/****************************************************************************
 * Name: efm32_gpio_kbd_open
 ****************************************************************************/

static int efm32_gpio_pps_open(file_t * filep)
{
    if ( efm32_gpio_pps == NULL )
    {
        EFM32_GPIO_PPS_LOG(LOG_ERR,"Not initialized!\n");
        return -EINVAL;
    }

    return OK;
}

/****************************************************************************
 * Name: efm32_gpio_kbd_close
 ****************************************************************************/

static int efm32_gpio_pps_close(file_t * filep)
{

    /* nothing to do */

    return OK;
}


/****************************************************************************
 * Name: efm32_gpio_kbd_read
 ****************************************************************************/

static int efm32_gpio_pps_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
    int res = 0;
    int i;
    static int shift_nano_second = 0;
    irqstate_t flags;

    if ( efm32_gpio_pps == NULL )
    {
        EFM32_GPIO_PPS_LOG(LOG_ERR,"Not initialized!\n");
        return -EINVAL;
    }

    switch(cmd)
    {
        case 0:
            if ( efm32_gpio_pps->pps )
            {
                flags = irqsave();
                i = efm32_gpio_pps->pps_nsec;
                efm32_gpio_pps->pps = false;
                irqrestore(flags);
                *((int*)arg) = i-shift_nano_second;
                shift_nano_second = i;
                res = 1;
            }
            break;
        default:
            return -EINVAL;
    }

    return res;
}



