/************************************************************************************
 * arch/arm/include/efm32/chip.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#  define CONFIG_EFM32_EFM32TG               /* EFM32TGxx family */
#  define CONFIG_EFM32_EFM32TG8XX            /* EFM32TG8XX family */
#  define CONFIG_EFM32_EFM32TG840            /* EFM32F205x and EFM32F207x */

#  define EFM32_LCD                    1   
#  define EFM32_USART                  2   
#  define EFM32_LEUSART                1   
#  define EFM32_I2C                    1   
#  define EFM32_TIMER                  2   
#  define EFM32_LETIMER                1   
#  define EFM32_RTC                    1   
#  define EFM32_PCNT                   1   
#  define EFM32_WATHDOG                1   
#  define EFM32_ADC                    1   
#  define EFM32_DAC                    1   
#  define EFM32_ACMP                   2  
#  define EFM32_AES                    1  
#  define EFM32_EBI                    0  
#  define EFM32_LESENSE                1  
#  define EFM32_OPAMP                  1  


#else
#  error "Unsupported EFM32 chip"
#endif

/* NVIC priority levels *************************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xe0 /* Bits [7:5] set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x20 /* Three bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_EFM32_CHIP_H */
