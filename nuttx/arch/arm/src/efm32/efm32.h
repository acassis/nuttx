/************************************************************************************
 * arch/arm/src/efm32/efm32.h
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Authors: Uros Platise <uros.platise@isotel.eu>
 *            Gregory Nutt <gnutt@nuttx.org>
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


/* Peripherals **********************************************************************/

#include "chip.h"
//#include "efm32_adc.h"
//#include "efm32_bkp.h"
//#include "efm32_can.h"
//#include "efm32_dbgmcu.h"
//#include "efm32_dma.h"
//#include "efm32_exti.h"
//#include "efm32_flash.h"
//#include "efm32_fsmc.h"
//#include "efm32_gpio.h"
//#include "efm32_i2c.h"
//#include "efm32_ltdc.h"
//#include "efm32_pwr.h"
//#include "efm32_rcc.h"
//#include "efm32_rtc.h"
//#include "efm32_sdio.h"
//#include "efm32_spi.h"
//#include "efm32_tim.h"
//#include "efm32_uart.h"
//#include "efm32_usbdev.h"
//#include "efm32_wdg.h"
//#include "efm32_lowputc.h"
//#include "efm32_eth.h"

#endif /* __ARCH_ARM_SRC_EFM32_EFM32_H */

