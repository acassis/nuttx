/************************************************************************************
 * configs/efm32gg-pnbfano/src/efm32_vcmp.c
 *
 *   Copyright (C) 2009-2013 Bouteville Pierre-Noel. All rights reserved.
 *   Author: Bouteville Pierre-Noel <pnb990@gmail.com>
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mmcsd.h>
#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <arch/irq.h>

#include "up_arch.h"
#include "chip.h"
#include "efm32gg-pnbfano.h"
#include "efm32_gpio.h"
#include "chip/efm32_cmu.h"
#include "chip/efm32_vcmp.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

//#undef EM_SYSLOG  /* Define to enable debug */
#define EM_DEBUG   /* Define to enable debug */
//#undef EM_VERBOSE /* Define to enable verbose debug */

#ifdef EM_SYSLOG
#  define vcmpdbg(...)    syslog(LOG_DEBUG,"VCMP :"__VA_ARGS__)
#  ifdef EM_VERBOSE
#    define vcmpvdbg(...) syslog(LOG_DEBUG,"VCMP :"__VA_ARGS__)
#  else
#    define vcmpvdbg(...)
#  endif
#elif defined EM_DEBUG
#  define vcmpdbg(lvl,...)    lldbg("VCMP:"__VA_ARGS__)
#  ifdef EM_VERBOSE
#       define vcmpvdbg(...)  syslog("VCMP:"__VA_ARGS__)
#  else
#    define vcmpvdbg(...)
#  endif
#else
#  undef EM_VERBOSE
#  define vcmpdbg(...)
#  define vcmpvdbg(...)
#endif


/************************************************************************************
 * Private Functions
 ************************************************************************************/


/****************************************************************************
 * irq handler
 ****************************************************************************/
int efm32_vcmp_irq(int irq, FAR void* context)
{
    (void)context;
    (void)irq;

    if ( getreg32(EFM32_VCMP_STATUS) & VCMP_STATUS_VCMPOUT )
        vcmpdbg("vcmp irg is 1\n");
    else
        vcmpdbg("vcmp irg is 0\n");

    return 0;
}


/************************************************************************************
 * Public Functions
 ************************************************************************************/



int efm32_vcmp_initialize(void)
{
    uint32_t regval;

    DEBUGASSERT(BOARD_VCMP_BIASPROG >=0);
    DEBUGASSERT(BOARD_VCMP_WARMUP   >=0);
    DEBUGASSERT(BOARD_VCMP_LEVEL    >=0);

    modifyreg32(EFM32_CMU_HFPERCLKEN0,0,CMU_HFPERCLKEN0_VCMP);

    regval = getreg32(EFM32_VCMP_CTRL);

    /* Configure Half Bias setting to save energy */
    regval |= VCMP_CTRL_HALFBIAS;

    /* Configure bias prog */
    regval &= ~(_VCMP_CTRL_BIASPROG_MASK);
    regval |= (BOARD_VCMP_BIASPROG << _VCMP_CTRL_BIASPROG_SHIFT);

    /* Configure sense for falling edge */
    regval |= VCMP_CTRL_IFALL;

    /* Configure sense for rising edge */
    regval |= VCMP_CTRL_IRISE;

    /* Configure warm-up time */
    regval &= ~(_VCMP_CTRL_WARMTIME_MASK);
    regval |= (BOARD_VCMP_WARMUP << _VCMP_CTRL_WARMTIME_SHIFT);

    /* Configure hysteresis 20mV */
    regval |= VCMP_CTRL_HYSTEN;

    /* Configure inactive output value: VBAT is ok when inactive */
    regval |= ( 1 << _VCMP_CTRL_INACTVAL_SHIFT);

    /* apply */
    putreg32(regval,EFM32_VCMP_CTRL);

    /* Configure trigger level */
    modifyreg32(EFM32_VCMP_INPUTSEL,_VCMP_INPUTSEL_TRIGLEVEL_MASK,
                ( ((uint32_t)((BOARD_VCMP_LEVEL-1.667)/0.034)) \
                  << _VCMP_INPUTSEL_TRIGLEVEL_SHIFT)
               );

    /* TODO: */
//#if defined EM_SYSLOG || defined EM_DEBUG
#if 0
    {
        uint32_t regval2 = getreg32(EFM32_VCMP_INPUTSEL);
        float volt;
        regval2 >>= _VCMP_INPUTSEL_TRIGLEVEL_SHIFT;
        volt = (regval2)*0.0034+1.667;

        vcmpdbg("set trig level to %f (%d)\n",volt,regval2);
    }
#endif

    regval |= VCMP_CTRL_EN;    			/* Enable VCMP */

    putreg32(regval,EFM32_VCMP_CTRL);   /* apply */

    /* If Low Power Reference is enabled, wait until VCMP is ready */
    /* before enabling it, see reference manual for deatils        */
    /* Configuring Low Power Ref without enable has no effect      */

    /* Poll for VCMP ready */
    while( (getreg32(EFM32_VCMP_STATUS) & VCMP_STATUS_VCMPACT) == 0 );

    /* Enable Low Power Reference setting */
    modifyreg32(EFM32_VCMP_INPUTSEL,0,VCMP_INPUTSEL_LPREF);

    /* Clear edge interrupt */
    modifyreg32(EFM32_VCMP_IFC,0,VCMP_IF_EDGE);

    (void)irq_attach(EFM32_IRQ_VCMP , efm32_vcmp_irq);

    up_enable_irq(EFM32_IRQ_VCMP);

    return 0;
}



