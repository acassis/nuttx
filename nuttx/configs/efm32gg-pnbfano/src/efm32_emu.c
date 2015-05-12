/************************************************************************************
 * configs/efm32gg-pnbfano/src/efm32_emu.c
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
#include "efm32_bitband.h"
#include "chip/efm32_rmu.h"
#include "chip/efm32_emu.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

//#undef EM_SYSLOG  /* Define to enable debug */
#define EM_DEBUG   /* Define to enable debug */
//#undef EM_VERBOSE /* Define to enable verbose debug */

#ifdef EM_SYSLOG
#  define emudbg(...)    syslog(LOG_DEBUG,"EMU :"__VA_ARGS__)
#  ifdef EM_VERBOSE
#    define emuvdbg(...) syslog(LOG_DEBUG,"EMU :"__VA_ARGS__)
#  else
#    define emuvdbg(...)
#  endif
#elif defined EM_DEBUG
#  define emudbg(lvl,...)    lldbg("CHRONO:"__VA_ARGS__)
#  ifdef EM_VERBOSE
#       define emuvdbg(...)  syslog("CHRONO:"__VA_ARGS__)
#  else
#    define emuvdbg(...)
#  endif
#else
#  undef EM_VERBOSE
#  define emudbg(...)
#  define emuvdbg(...)
#endif

/* Fix for errata EMU_E107 - non-WIC interrupt masks. */                              
#if defined(_EFM32_GECKO_FAMILY) 
  #define ERRATA_FIX_EMU_E107_EN
  #define NON_WIC_INT_MASK_0    (~(0x0dfc0323U))
  #define NON_WIC_INT_MASK_1    (~(0x0U))     
#elif defined(_EFM32_TINY_FAMILY) 
  #define ERRATA_FIX_EMU_E107_EN
  #define NON_WIC_INT_MASK_0    (~(0x001be323U))
  #define NON_WIC_INT_MASK_1    (~(0x0U))  
#elif defined(_EFM32_GIANT_FAMILY)     
  #define ERRATA_FIX_EMU_E107_EN
  #define NON_WIC_INT_MASK_0    (~(0xff020e63U))
  #define NON_WIC_INT_MASK_1    (~(0x00000046U))
#elif defined(_EFM32_WONDER_FAMILY)
  #define ERRATA_FIX_EMU_E107_EN
  #define NON_WIC_INT_MASK_0    (~(0xff020e63U))
  #define NON_WIC_INT_MASK_1    (~(0x00000046U))     
#else
/* Zero Gecko and future families are not affected by errata EMU_E107 */
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/****************************************************************************
 * irq handler
 ****************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/



int efm32_emu_initialize(void)
{
    uint32_t regval;

#if 0
    if ( bitband_get_peripheral(EFM32_EMU_STATUS,_EMU_STATUS_BURDY_SHIFT) )
    {
        emudbg("Backup domain already ready!");
        return 0;
    }
#endif

    /* unlock EM4Lock */

    bitband_set_peripheral(EFM32_EMU_EM4CONF, _EMU_EM4CONF_LOCKCONF_SHIFT, 0);

    /* Set power connection configuration */

    regval = getreg32(EFM32_EMU_PWRCONF);

    regval &= ~( _EMU_PWRCONF_PWRRES_MASK       |
                 _EMU_PWRCONF_VOUTSTRONG_MASK   |
                 _EMU_PWRCONF_VOUTMED_MASK      |
                 _EMU_PWRCONF_VOUTWEAK_MASK     );

    regval |= (_EMU_PWRCONF_PWRRES_RES3 << _EMU_PWRCONF_PWRRES_SHIFT    )|\
              (0                        << _EMU_PWRCONF_VOUTSTRONG_SHIFT)|\
              (0                        << _EMU_PWRCONF_VOUTMED_SHIFT   )|\
              (0                        << _EMU_PWRCONF_VOUTWEAK_SHIFT  );

    putreg32(regval,EFM32_EMU_PWRCONF);

    /* Set backup domain inactive mode configuration */

    modifyreg32(EFM32_EMU_BUINACT,_EMU_BUINACT_PWRCON_MASK,
                _EMU_BUINACT_PWRCON_NODIODE << _EMU_BUINACT_PWRCON_SHIFT);

    /* Set backup domain active mode configuration */

    modifyreg32(EFM32_EMU_BUACT,_EMU_BUINACT_PWRCON_MASK,
                _EMU_BUINACT_PWRCON_MAINBU << _EMU_BUINACT_PWRCON_SHIFT);

    /* Set power control configuration */

    regval = getreg32(EFM32_EMU_BUCTRL);

    regval &= ~( _EMU_BUCTRL_PROBE_MASK     |
                 _EMU_BUCTRL_BODCAL_MASK    |
                 _EMU_BUCTRL_STATEN_MASK    |
                 _EMU_BUCTRL_EN_MASK        );

    /* Note use of ->enable to both enable BUPD, use BU_VIN pin input and
       release reset */
    regval |=  (_EMU_BUCTRL_PROBE_DISABLE   << _EMU_BUCTRL_PROBE_SHIFT )|\
               (0                           << _EMU_BUCTRL_BODCAL_SHIFT)|\
               (0                           << _EMU_BUCTRL_STATEN_SHIFT)|\
               (1                           << _EMU_BUCTRL_EN_SHIFT);

    putreg32(regval,EFM32_EMU_BUCTRL); /* apply */

    /* enable BU_VIN input power pin */

    bitband_set_peripheral(EFM32_EMU_ROUTE,_EMU_ROUTE_BUVINPEN_SHIFT,1);

    regval = getreg32(EFM32_EMU_EM4CONF);

    /* Clear fields that will be reconfigured */
    regval &= ~(
                _EMU_EM4CONF_OSC_MASK         |
                _EMU_EM4CONF_BURTCWU_MASK     |
                _EMU_EM4CONF_VREGEN_MASK      );

    /* Configure new settings */
    regval |= (
               (_EMU_EM4CONF_OSC_LFXO   << _EMU_EM4CONF_OSC_SHIFT       )|
               (0                       << _EMU_EM4CONF_BURTCWU_SHIFT   )|
               (1                       << _EMU_EM4CONF_VREGEN_SHIFT    )
              );

    /* Apply configuration. Note that lock can be set after this stage. */

    putreg32(regval,EFM32_EMU_EM4CONF);


    /* Lock EM4 */

    bitband_set_peripheral(EFM32_EMU_EM4CONF, _EMU_EM4CONF_LOCKCONF_SHIFT, 1);

    return 0;
}

