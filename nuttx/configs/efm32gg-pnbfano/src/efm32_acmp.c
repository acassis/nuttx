/************************************************************************************
 * configs/efm32gg-pnbfano/src/efm32_acmp.c
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

#include "up_arch.h"
#include "chip.h"
#include "efm32gg-pnbfano.h"
#include "efm32_gpio.h"
#include "chip/efm32_acmp.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

//#undef EM_SYSLOG  /* Define to enable debug */
#define EM_DEBUG   /* Define to enable debug */
//#undef EM_VERBOSE /* Define to enable verbose debug */

#ifdef EM_SYSLOG
#  define acmpdbg(...)    syslog(LOG_DEBUG,"ACMP :"__VA_ARGS__)
#  ifdef EM_VERBOSE
#    define acmpvdbg(...) syslog(LOG_DEBUG,"ACMP :"__VA_ARGS__)
#  else
#    define acmpvdbg(...)
#  endif
#elif defined EM_DEBUG
#  define acmpdbg(lvl,...)    lldbg("CHRONO:"__VA_ARGS__)
#  ifdef EM_VERBOSE
#       define acmpvdbg(...)  syslog("CHRONO:"__VA_ARGS__)
#  else
#    define acmpvdbg(...)
#  endif
#else
#  undef EM_VERBOSE
#  define acmpdbg(...)
#  define acmpvdbg(...)
#endif

#ifdef BOARD_ACMP_ENABLE

/************************************************************************************
 * Private Functions
 ************************************************************************************/
static void inline acmp_putreg(uint32_t off, uint32_t value)
{
    putreg32(EFM32_ACMP0_BASE+off,value);
}

static uint32_t inline acmp_getreg(uint32_t off)
{
    return getreg32(EFM32_ACMP0_BASE+off);
}

/****************************************************************************
 * irq handler
 ****************************************************************************/
int efm32_acmp_irq(int irq, FAR void* context)
{
    (void)context;
    (void)irq;

    acmpdbg("acmp irg %d\n");

    return 0;
}


/************************************************************************************
 * Public Functions
 ************************************************************************************/



int efm32_initialize_acmp(void)
{
    int regval;


    acmp_putreg(EFM32_ACMP_CTRL_OFFSET      , _ACMP_CTRL_RESETVALUE);
    acmp_putreg(EFM32_ACMP_INPUTSEL_OFFSET  , _ACMP_INPUTSEL_RESETVALUE);
    acmp_putreg(EFM32_ACMP_IEN_OFFSET       , _ACMP_IEN_RESETVALUE);
    acmp_putreg(EFM32_ACMP_IFC_OFFSET       , _ACMP_IF_MASK);

    regval = acmp_getreg(EFM32_ACMP_CTRL_OFFSET);

    /* Set control register. No need to set interrupt modes 
     * Quickest reaction time
     * Bias 8µA
     * Hysteresis 57mV
     */

    regval = (0 << _ACMP_CTRL_FULLBIAS_DEFAULT  ) | \
             (1 << _ACMP_CTRL_HALFBIAS_SHIFT    ) | \
             (8 << _ACMP_CTRL_BIASPROG_S        ) | \
             (0 << _ACMP_CTRL_WARMTIME_SHIFT    ) | \
             /* Hysteresis 57mV */
             ACMP_CTRL_HYSTSEL_HYST7            | \
             /* Interrupt on falling edge */
             ACMP_CTRL_IFALL_ENABLED            | \
             /* Interrupt on rising edge */
             ACMP_CTRL_IRISE_ENABLED            | \
             /* if disabled VBAT is OK */
             ACMP_CTRL_INACTVAL_HIGH            ;

    acmp_putreg(EFM32_ACMP_CTRL_OFFSET,regval); /* Apply */

    regval = (1                         << _ACMP_INPUTSEL_LPREF_SHIFT ) | \
             (BOARD_ACMP_INPUT          << _ACMP_INPUTSEL_NEGSEL_SHIFT) | \
             (ACMP_INPUTSEL_NEGSEL_2V5  << _ACMP_INPUTSEL_NEGSEL_SHIFT) | \
             ;

acmp->INPUTSEL

    (void)irq_attach(EFM32_IRQ_ACMP , efm32_acmp_irq);

    /* Enable ACMP */

    regval = acmp_getreg(EFM32_ACMP_CTRL_OFFSET);
    regval |= _ACMP_CTRL_EN_SHIFT;
    acmp_putreg(EFM32_ACMP_CTRL_OFFSET,regval); /* Apply */


    return 0;
}

#else

int efm32_initialize_acmp(void)
{
    acmpdbg("acmp Disabled!\n");
    return 0;
}

#endif
