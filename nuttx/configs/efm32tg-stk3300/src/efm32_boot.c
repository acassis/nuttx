/*****************************************************************************
 * configs/efm32-dk3650/src/efm32_boot.c
 *
 *   Copyright (C) 2014 Richard Cochran. All rights reserved.
 *   Author: Richard Cochran <richardcochran@gmail.com>
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_INPUT_EFM32_GPIO_KEYPAD

#include <nuttx/input/kbd_codec.h>
#include <nuttx/input/keypad.h>

keybad_gpio_list_t gpio_kbd_list[] = 
{
    {
        .pin        =   0               ,
        .port       =   1               , /* 1 => portB */
        .keycode    =   KEYCODE_LEFT    ,
    },
    {
        .pin        =   1               ,
        .port       =   1               , /* 1 => portB */ 
        .keycode    =   KEYCODE_RIGHT   ,
    },
    {
        .pin = -1,
    }
};

#endif

void efm32_boardinitialize(void)
{
    //efm32_gpio_kbd_init(gpio_kbd_list);
}


