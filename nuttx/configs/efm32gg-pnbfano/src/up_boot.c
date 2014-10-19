/*****************************************************************************
 * configs/efm32gg-pnbfano/src/efm32_boot.c
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

#include <nuttx/config.h>

#include <nuttx/input/keypad.h>

/* pnbtodo:is it the right way ? */
#include <arch/board/board.h>


#include "efm32.h"
#include "efm32_spi.h"
#include "stdio.h"


/****************************************************************************
 * Spi mapping Functions
 ****************************************************************************/
const efm32_spi_cfg_t efm32_spi_cfg_tb[CONFIG_EFM32_SPI_NBR] = 
{
#ifdef CONFIG_PNBFANO_USE_MMCSD
    {
        .usart      = USART0,
        .location   = 2,
#ifdef CONFIG_EFM32_SPI_INTERRUPTS
        .spiirq     = -1,     /* SPI IRQ number : -1 => No interrupts */
#endif
        .clk_port    = gpioPortC,
        .clk_pin     =  9,
        .mosi_port   = gpioPortC,
        .mosi_pin    = 11,
        .miso_port   = gpioPortC,
        .miso_pin    = 10,
        .cs_port     = gpioPortC,
        .cs_pin      =  8
    }
#endif
};




/****************************************************************************
 * Public Functions
 ****************************************************************************/


#ifdef CONFIG_BOARD_INITIALIZE
void board_initialize(void)
{

#if defined(CONFIG_EFM32_GPIO_KEYPAD) 
    keypad_kbdinit();
#endif

  /* Mount the SDIO-based MMC/SD block driver */

  nsh_archinitialize();

}
#endif

