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


static keybad_gpio_list_t *gpio_list_p = NULL;

/****************************************************************************
 * Keypad interrupt handler
 *   mask interrupts
 *   prepare column drivers for scan
 *   posts keypad semaphore
 ****************************************************************************/

inline int efm32_gpio_kbd_irq(int irq, uint32_t * regs)
{
    (void)irq;
    (void)regs;
    return 0;
}

/****************************************************************************
 * Initialize GPIO for key pad.
 ****************************************************************************/

int keypad_gpio_init(keybad_gpio_list_t* list)
{

    DEBUGASSERT(gpio_list_p == NULL);

    gpio_list_p = list;
    /* pnbtodo : initialise irq pin or polling */

    return 0;
}



