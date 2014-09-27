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

#include <nuttx/input/kbd_codec.h>

#include <debug.h>
#include <efm32.h>
#include <efm32_lowputc.h>
#include <efm32_gpio.h>
#include <efm32_gpio_irq.h>


/****************************************************************************
 * Keypad interrupt handler
 *   mask interrupts
 *   prepare column drivers for scan
 *   posts keypad semaphore
 ****************************************************************************/

int efm32_gpio_kbd_irq(int pin, FAR void* context)
{
    (void)context;
    switch (pin)
    {
        case 11:
            if ( GPIO_PinInGet(gpioPortB,11) == 0 )
            {
                lldbg("PB1 pressed\n"); 
            }
            else
            {
                lldbg("PB1 released\n"); 
            }
            break;
        case 8:
            if ( GPIO_PinInGet(gpioPortD,8) == 0 )
            {
                lldbg("PB0 pressed\n"); 
            }
            else
            {
                lldbg("PB0 released\n"); 
            }
            break;
        default: 
            lldbg("Unknown pin %d \n",pin);
            break;
    }

    return 0;
}

/****************************************************************************
 * Initialize GPIO for key pad.
 PortD  8  => PB0 LEFT
 PortB 11  => PB1 RIGHT
 ****************************************************************************/

int keypad_kbdinit(void)
{

    irq_gpio_attach(11,  efm32_gpio_kbd_irq);
    irq_gpio_attach(8,   efm32_gpio_kbd_irq);

    GPIO_PinModeSet( gpioPortB,
                     11,
                     gpioModeInputPullFilter,
                     1
                   );

    GPIO_PinModeSet( gpioPortD,
                     8,
                     gpioModeInputPullFilter,
                     1
                   );

    GPIO_IntConfig( gpioPortB,
                    11,
                    true,
                    true,
                    true
                  );

    GPIO_IntConfig( gpioPortD,
                    8,
                    true,
                    true,
                    true
                  );

    return 0;
}



