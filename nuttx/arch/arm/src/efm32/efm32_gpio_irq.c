/************************************************************************************
 * arch/arm/src/efm32/efm32_gpio_irq.c
 *
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
 ************************************************************************************/


#include <nuttx/config.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include <debug.h>
#include "efm32.h"
#include "arch/irq.h"
#include "efm32_gpio.h"
#include "efm32_gpio_irq.h"


/* dynamique gpio table */

xcpt_t irq_gpio_handler_table[EFM32_GPIO_IRQ_NBR];


/****************************************************************************
 * Name: irq_gpio_init
 *
 * Description:
 *   Initialise dispatcher gpio IRQ.
 *
 ****************************************************************************/
void irq_gpio_init(void)
{
    /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
     * certain that there are no issues with the state of global variables.
     */

    int i = EFM32_GPIO_IRQ_NBR;
    while (i--)
    {
        irq_gpio_handler_table[i] = NULL;
    }
}


/****************************************************************************
 * Name: irq_gpio_dispatcher
 *
 * Description:
 *   GPIO ODD/EVEN IRQ dispatcher.
 *
 ****************************************************************************/

#define irqGPIO_MASK2IDX(mask) (__CLZ(__RBIT(mask)))
static int irq_gpio_dispatcher(int irq, FAR void *context)
{
    uint32_t iflags;

    switch ( irq )
    {
       case EFM32_IRQ_GPIO_ODD:     iflags = 0x0000AAAA;    break;
       case EFM32_IRQ_GPIO_EVEN:    iflags = 0x00005555;    break;
       default: lldbg("Unknown GPIO IRQ %d\n",irq);     return -1; 
    }

    iflags &= GPIO_IntGetEnabled();

    /* Clean only even interrupts. */
    GPIO_IntClear(iflags);

    /* Nuttx Doesn't include CMSIS so irqGPIO_MASK2IDX doesn't work. */

#if ( defined( __CLZ ) && defined ( __RBIT ) )
    /* check for all flags set in IF register */
    while(iflags)
    {
        int irqIdx;
        irqIdx = irqGPIO_MASK2IDX(iflags);

        /* clear flag*/
        iflags &= ~(1 << irqIdx);

        if ( irq_gpio_handler_table[irqIdx] != NULL )
            irq_gpio_handler_table[irqIdx](irqIdx);
    }
#else
    /* check for all flags set in IF register */
    int irqIdx = EFM32_GPIO_IRQ_NBR;
    while(irqIdx--)
    {
        /* clear flag*/
        if ( iflags & (1 << irqIdx) )
        {
            DEBUGASSERT(irq_gpio_handler_table[irqIdx] != NULL );
            irq_gpio_handler_table[irqIdx](irqIdx,context);
        }
    }
#endif
    return 0;
}



/****************************************************************************
 * Name: irq_gpio_request
 *
 * Description:
 *   request an gpio interrupts.
 *
 ****************************************************************************/
void irq_gpio_request(int pin, xcpt_t handler)
{
    DEBUGASSERT(pin < EFM32_GPIO_IRQ_NBR);

    irqstate_t irq_state = irqsave();

    irq_gpio_handler_table[pin] = handler;

    int i = EFM32_GPIO_IRQ_NBR;

    bool odd = false;
    bool even = false;
    while (i--)
    {
        if ( irq_gpio_handler_table[i] != NULL )
        {
            if ( i & 1 )
                odd = true;
            else
                even = true;
        }
    }

    irq_attach(EFM32_IRQ_GPIO_ODD, (odd )?irq_gpio_dispatcher:NULL);
    irq_attach(EFM32_IRQ_GPIO_EVEN,(even)?irq_gpio_dispatcher:NULL);

    irqrestore(irq_state);

}


