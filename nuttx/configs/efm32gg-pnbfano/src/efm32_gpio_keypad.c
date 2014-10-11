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

#include <nuttx/arch.h>

#include <efm32_gpio_keypad.h>


/****************************************************************************
 * Keypad mapping for stk3300 board
 ****************************************************************************/

efm32_gpio_keypad_t pnbfano_key_map[] = 
{
    {   /* BP1 */
        .port = gpioPortE,
        .pin  = 12,
        .special_key = KEYCODE_LEFT
        //.special_key = KEYCODE_NORMAL,
        //.key    = 'a'
    },
    {   /* BP2 */
        .port = gpioPortE,
        .pin  = 13,
        .special_key = KEYCODE_DOWN
        //.special_key = KEYCODE_NORMAL,
        //.key    = 'a'
    },
    {   /* BP3 */
        .port = gpioPortE,
        .pin  = 14,
        .special_key = KEYCODE_UP
        //.special_key = KEYCODE_NORMAL,
        //.key    = 'a'
    },
    {   /* BP4 */
        .port = gpioPortE,
        .pin  = 15,
        .special_key = KEYCODE_RIGHT
        //.special_key = KEYCODE_NORMAL,
        //.key    = 'b'
    },
    {
        .pin = -1
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



