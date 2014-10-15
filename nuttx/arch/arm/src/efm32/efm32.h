/************************************************************************************
 * arch/arm/src/efm32/efm32.h
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

#ifndef __ARCH_ARM_SRC_EFM32_EFM32_H
#define __ARCH_ARM_SRC_EFM32_EFM32_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "up_internal.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Additional Configuration *********************************************************/
/* Custom debug settings used in the EFM32 port.  These are managed by EFM32-specific
 * logic and not the common logic in include/debug.h.  NOTE:  Some of these also
 * depend on CONFIG_DEBUG_VERBOSE
 */


#ifdef __cplusplus
  #define   __I     volatile             /* Defines 'read only' permissions                 */
#else
  #define   __I     volatile const       /* Defines 'read only' permissions                 */
#endif
#define     __O     volatile             /* Defines 'write only' permissions                */
#define     __IO    volatile             /* Defines 'read / write' permissions              */

#define EFM_ASSERT DEBUGASSERT


/* Peripherals **********************************************************************/

#include "chip.h"

#if defined(CONFIG_EFM32_EFM32TG)

#   include "efm32tg.h"

#elif defined(CONFIG_EFM32_EFM32GG)

#   include "efm32gg.h"

#endif

#endif /* __ARCH_ARM_SRC_EFM32_EFM32_H */

