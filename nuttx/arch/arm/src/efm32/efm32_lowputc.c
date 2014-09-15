/****************************************************************************
 * arch/arm/src/efm32/efm32_lowputc.c
 *
 *   Copyright (C) 2014 Richard Cochran. All rights reserved.
 *   Author: Richard Cochran <richardcochran@gmail.com>
 *   Copyright (C) 2014 Pierre-noel Bouteville . All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "up_internal.h"

/* pnbtodo:is it the right way ? */
#include "arch/board/board.h"

#include "nvic.h"
#include "efm32.h"
#include "efm32_gpio.h"
#include "efm32_lowputc.h"

#include "up_arch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

void efm32_lowsetup(void)
{
    /* Enable GPIO clock. */

    CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;

    /* Enable Serial wire output pin */

    GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;

    /* Set location */

    GPIO_DbgLocationSet(CONFIG_EFM32_SWO_LOCATION);

    /* Enable output on pin */

    GPIO_PinModeSet(CONFIG_EFM32_SWO_PORT,
                    CONFIG_EFM32_SWO_PIN, 
                    gpioModePushPull, 
                    0
                   );

    /* Enable debug clock AUXHFRCO */

    CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

    /* Wait until clock is ready */

    while (!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));

    /* Enable trace in core debug */
    
    putreg32(getreg32(NVIC_DEMCR)|NVIC_DEMCR_TRCENA, NVIC_DEMCR);
    putreg32(0xC5ACCE55,ITM_LAR );
    putreg32(0x0,       ITM_TER );
    putreg32(0x0,       ITM_TCR );
    putreg32(2  ,       TPI_SPPR); /* pin protocol: 2=> Manchester (USART) */


    /* default 880kbps */

    //putreg32(0xf       , TPI_ACPR ); /* TRACECLKIN/(ACPR+1) SWO speed */
    putreg32(14        , TPI_ACPR ); /* TRACECLKIN/(ACPR+1) SWO speed */
    putreg32(0x0       , ITM_TPR  );
    putreg32(0x400003FE, DWT_CTRL );
    putreg32(0x0001000D, ITM_TCR  );
    putreg32(0x00000100, TPI_FFCR );
    putreg32(0xFFFFFFFF, ITM_TER  ); /* enable 32 Ports */
}

void up_lowputc(char c)
{
    /* use Port #0 at default */

    int ch = 0;

    /* ITM enabled */

    if ((getreg32(ITM_TCR) & ITM_TCR_ITMENA_Msk) == 0 )   
        return;

    /* ITM Port "ch" enabled */

    if ( getreg32(ITM_TER) & (1UL << ch) )            
    {
        while (getreg32(ITM_PORT(ch)) == 0);
        putreg8((uint8_t) c,ITM_PORT(ch));
    }
}

void up_putc(char c)
{
    /* Convert LF into CRLF. */
    
    if (c == '\n')
        up_lowputc('\r');

    up_lowputc(c);
}


