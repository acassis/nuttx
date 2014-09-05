/************************************************************************************
 * arch/arm/include/efm32/chip.h
 *
 *   Copyright (C) 2009, 2011-2014 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2014 Pierre-noel Bouteville . All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Pierre-noel Bouteville <pnb990@gmail.com>
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

#ifndef __ARCH_ARM_INCLUDE_EFM32_CHIP_H
#define __ARCH_ARM_INCLUDE_EFM32_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip and provide alternate function pin-mapping
 *
 * NOTE: Each GPIO pin may serve either for general purpose I/O or for a special
 * alternate function (such as USART, CAN, USB, SDIO, etc.).  That particular
 * pin-mapping will depend on the package and EFM32 family.  If you are incorporating
 * a new EFM32 chip into NuttX, you will need to add the pin-mapping to a header file
 * and to include that header file below. The chip-specific pin-mapping is defined in
 * the chip datasheet.
 */

/* EFM32 EnergyMicro ************************************************************/



#if defined(CONFIG_ARCH_CHIP_EFM32TG840F32) 
#  define CONFIG_EFM32_EFM32TG               
#  define CONFIG_EFM32_EFM32TG8XX            
#  define CONFIG_EFM32_EFM32TG840            
#  define CONFIG_EFM32_EFM32XXXXXF32         

/* Part number capabilities */
#define EFM32_ACMP_NBR          2 
#define EFM32_USART_NBR         2 
#define EFM32_TIMER_NBR         2 
#define EFM32_LEUART_NBR        1 
#define EFM32_LETIMER_NBR       1 
#define EFM32_PCNT_NBR          1 
#define EFM32_ADC_NBR           1 
#define EFM32_DAC_NBR           1 
#define EFM32_I2C_NBR           1 
#define EFM32_AES_NBR           1
#define EFM32_DMA_NBR           1
#define EFM32_LE_NBR            1
#define EFM32_MSC_NBR           1
#define EFM32_EMU_NBR           1
#define EFM32_RMU_NBR           1
#define EFM32_CMU_NBR           1
#define EFM32_LESENSE_NBR       1
#define EFM32_RTC_NBR           1
#define EFM32_GPIO_NBR          1
#define EFM32_VCMP_NBR          1
#define EFM32_PRS_NBR           1
#define EFM32_OPAMP_NBR         1
#define EFM32_LCD_NBR           1
#define EFM32_HFXTAL_NBR        1
#define EFM32_LFXTAL_NBR        1
#define EFM32_WDOG_NBR          1
#define EFM32_DBG_NBR           1
#define EFM32_BOOTLOADER_NBR    1
#define EFM32_ANALOG_NBR        1

#else
#  error "Unsupported EFM32 chip"
#endif

/* NVIC priority levels *************************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xe0 /* Bits [7:5] set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x20 /* Three bits of interrupt priority used */

/* If CONFIG_ARMV7M_USEBASEPRI is selected, then interrupts will be disabled
 * by setting the BASEPRI register to NVIC_SYSH_DISABLE_PRIORITY so that most
 * interrupts will not have execution priority.  SVCall must have execution
 * priority in all cases.
 *
 * In the normal cases, interrupts are not nest-able and all interrupts run
 * at an execution priority between NVIC_SYSH_PRIORITY_MIN and
 * NVIC_SYSH_PRIORITY_MAX (with NVIC_SYSH_PRIORITY_MAX reserved for SVCall).
 *
 * If, in addition, CONFIG_ARCH_HIPRI_INTERRUPT is defined, then special
 * high priority interrupts are supported.  These are not "nested" in the
 * normal sense of the word.  These high priority interrupts can interrupt
 * normal processing but execute outside of OS (although they can "get back
 * into the game" via a PendSV interrupt).
 *
 * In the normal course of things, interrupts must occasionally be disabled
 * using the irqsave() inline function to prevent contention in use of
 * resources that may be shared between interrupt level and non-interrupt
 * level logic.  Now the question arises, if CONFIG_ARCH_HIPRI_INTERRUPT,
 * do we disable all interrupts (except SVCall), or do we only disable the
 * "normal" interrupts.  Since the high priority interrupts cannot interact
 * with the OS, you may want to permit the high priority interrupts even if
 * interrupts are disabled.  The setting CONFIG_ARCH_INT_DISABLEALL can be
 * used to select either behavior:
 *
 *   ----------------------------+--------------+----------------------------
 *   CONFIG_ARCH_HIPRI_INTERRUPT |      NO      |             YES
 *   ----------------------------+--------------+--------------+-------------
 *   CONFIG_ARCH_INT_DISABLEALL  |     N/A      |     YES      |      NO
 *   ----------------------------+--------------+--------------+-------------
 *                               |              |              |    SVCall
 *                               |    SVCall    |    SVCall    |    HIGH
 *   Disable here and below --------> MAXNORMAL ---> HIGH --------> MAXNORMAL
 *                               |              |    MAXNORMAL |
 *   ----------------------------+--------------+--------------+-------------
 */

#if defined(CONFIG_ARCH_HIPRI_INTERRUPT) && defined(CONFIG_ARCH_INT_DISABLEALL)
#  define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + 2*NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_HIGH_PRIORITY       (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_HIGH_PRIORITY
#  define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX
#else
#  define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_HIGH_PRIORITY       NVIC_SYSH_PRIORITY_MAX
#  define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_MAXNORMAL_PRIORITY
#  define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX
#endif

#endif /* __ARCH_ARM_INCLUDE_STM32_CHIP_H */
