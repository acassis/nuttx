/****************************************************************************
 * configs/efm32gg-pnbfano/src/efm32_slow_polling.c
 *
 *   Copyright (C) 2015 Pierre-noel Bouteville. All rights reserved.
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
#include <poll.h>
#include <errno.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include "up_arch.h"
#include "efm32_gpio.h"
#include "efm32gg-pnbfano.h"

//#define EFM32_SLOW_POLL_LOG(...)
//#define EFM32_SLOW_POLL_LOG(...) lldbg(__VA_ARGS__)
#define EFM32_SLOW_POLL_LOG(...) syslog(LOG_NOTICE,__VA_ARGS__)

#ifndef CONFIG_SLOW_POLL_MS
#   define CONFIG_SLOW_POLL_MS 2000
#endif

static struct work_s work;
static void efm32_slow_poll_worker(FAR void *arg);

/****************************************************************************
 * Name: efm32_slow_poll_next_poll
 ****************************************************************************/
static int efm32_slow_poll_next_poll(void)
{
    if ( work_queue(HPWORK, 
                    &work, 
                    efm32_slow_poll_worker,
                    NULL, 
                    MSEC2TICK(CONFIG_SLOW_POLL_MS)
                    ) != OK )
    {
        EFM32_SLOW_POLL_LOG("Cannot register slow poll work !\n");
        return -1;
    }
    return 0;
}

/****************************************************************************
 * irq handler
 ****************************************************************************/
static void efm32_slow_poll_worker(FAR void *arg)
{
    UNUSED(arg);

    EFM32_SLOW_POLL_LOG("slow poll work...\n");

#if CONFIG_USBDEV
    efm32_usbdev_slow_poll();
#endif

    efm32_slow_poll_next_poll();

    return;
}



/****************************************************************************
 * Name: efm32_slow_poll_init
 *
 * Description:
 *  Initialize slow poller to detect event like card insertion, etc...
 * 
 ****************************************************************************/
int efm32_slow_poll_init( void )
{

    if ( efm32_slow_poll_next_poll() < 0 )
        return -1;
    return 0;
}




