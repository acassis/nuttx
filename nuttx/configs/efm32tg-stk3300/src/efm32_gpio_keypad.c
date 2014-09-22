/****************************************************************************
 * arch/arm/src/efm32/keypad/keypad.c
 * Driver for Stk3300 keypad hardware
 *
 *   Copyright (C) 2011 Stefan Richter. All rights reserved.
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <stdint.h>
#include <errno.h>
#include <unistd.h>

#include <nuttx/streams.h>
#include <nuttx/input/kbd_codec.h>

#include <nuttx/input/keypad.h>

#include "efm32.h"
#include <efm32_gpio.h>


/****************************************************************************
 * Keypad interrupt handler
 *   mask interrupts
 *   prepare column drivers for scan
 *   posts keypad semaphore
 ****************************************************************************/

int efm32_gpio_kbd_irq(int irq, FAR void* context)
{
    (void)irq;
    (void)context;
    return 0;
}

/****************************************************************************
 * Initialize GPIO for key pad.
 ****************************************************************************/

int keypad_kbdinit(void)
{

    /* pnbtodo : initialise irq pin or polling */

    irq_attach(EFM32_IRQ_GPIO_ODD,  efm32_gpio_kbd_irq);
    irq_attach(EFM32_IRQ_GPIO_EVEN, efm32_gpio_kbd_irq);
    up_enable_irq(EFM32_IRQ_GPIO_ODD);
    up_enable_irq(EFM32_IRQ_GPIO_EVEN);

    GPIO_PinModeSet( gpioPortB,
                     0,
                     gpioModeInputPullFilter,
                     1
                   );

    GPIO_PinModeSet( gpioPortB,
                     1,
                     gpioModeInputPullFilter,
                     1
                   );

    GPIO_IntConfig( gpioPortB,
                    0,
                    true,
                    true,
                    true
                  );

    GPIO_IntConfig( gpioPortB,
                    0,
                    true,
                    true,
                    true
                  );

    return 0;
}



