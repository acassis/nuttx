/****************************************************************************
 * configs/efm32gg-pnbfano/src/efm32_gpio_keypad.c
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

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "efm32_gpio.h"
#include <efm32_gpio_keypad.h>
#include "efm32gg-pnbfano.h"

/* Buttons:
 *
 * The pnbfano board has four buttons, BUT1-4. Each is grounded and so should
 * have a weak pull-up so that it will be sensed as "1" when open and "0"
 * when closed.
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * PE12                  BUT1
 * PE13                  BUT2
 * PE14                  BUT3
 * PE15                  BUT4
 * --------------------- ---------------------
 */

#ifdef CONFIG_EFM32_GPIO_IRQ
#  define GPIO_BUTTON_1 (GPIO_INPUT_PULLUP|GPIO_INT_BOTH|GPIO_PORTE|GPIO_PIN12)
#  define GPIO_BUTTON_2 (GPIO_INPUT_PULLUP|GPIO_INT_BOTH|GPIO_PORTE|GPIO_PIN13)
#  define GPIO_BUTTON_3 (GPIO_INPUT_PULLUP|GPIO_INT_BOTH|GPIO_PORTE|GPIO_PIN14)
#  define GPIO_BUTTON_4 (GPIO_INPUT_PULLUP|GPIO_INT_BOTH|GPIO_PORTE|GPIO_PIN15)

#  define GPIO_IRQ_BUTTON_1 EFM32_IRQ_EXTI12
#  define GPIO_IRQ_BUTTON_2 EFM32_IRQ_EXTI13
#  define GPIO_IRQ_BUTTON_3 EFM32_IRQ_EXTI14
#  define GPIO_IRQ_BUTTON_4 EFM32_IRQ_EXTI15
#endif


/****************************************************************************
 * Keypad mapping for stk3300 board
 ****************************************************************************/

efm32_gpio_keypad_t pnbfano_key_map[] = 
{
    {   /* BP1 */
        .gpio = GPIO_BUTTON_1,
        .irq  = GPIO_IRQ_BUTTON_1,
        .special_key = KEYCODE_LEFT
        //.special_key = KEYCODE_NORMAL,
        //.key    = 'a'
    },
    {   /* BP2 */
        .gpio = GPIO_BUTTON_2,
        .irq  = GPIO_IRQ_BUTTON_2,
        .special_key = KEYCODE_DOWN
        //.special_key = KEYCODE_NORMAL,
        //.key    = 'a'
    },
    {   /* BP3 */
        .gpio = GPIO_BUTTON_3,
        .irq  = GPIO_IRQ_BUTTON_3,
        .special_key = KEYCODE_UP
        //.special_key = KEYCODE_NORMAL,
        //.key    = 'a'
    },
    {   /* BP4 */
        .gpio = GPIO_BUTTON_4,
        .irq  = GPIO_IRQ_BUTTON_4,
        .special_key = KEYCODE_RIGHT
        //.special_key = KEYCODE_NORMAL,
        //.key    = 'b'
    },
    {
        .gpio = EFM32_GPIO_KEYPAD_END
    }
};


/****************************************************************************
 * Register all board drivers for key pad.
 ****************************************************************************/

int keypad_kbdinit(void)
{

    efm32_gpio_keypad_init(pnbfano_key_map,"/dev/keypad");

    return 0;
}



