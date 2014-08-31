/****************************************************************************************************
 * arch/arm/include/efm32s/efm32tg8xx_irq.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

/* This file should never be included directed but, rather, only indirectly through nuttx/irq.h */

#ifndef __ARCH_ARM_INCLUDE_EFM32TG8XX_IRQ_H
#define __ARCH_ARM_INCLUDE_EFM32TG8XX_IRQ_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 *
 * Processor Exceptions (vectors 0-15).  These common definitions can be found
 * in nuttx/arch/arm/include/efm32/irq.h
 *
 * External interrupts (vectors >= 16)
 */

#define EFM32_IRQ_DMA         (EFM32_IRQ_INTERRUPTS+0)  /* 0:  Window Watchdog interrupt */
#define EFM32_IRQ_GPIO_EVEN   (EFM32_IRQ_INTERRUPTS+1)  /* 1:  PVD through EXTI Line detection interrupt */
#define EFM32_IRQ_TIMER0      (EFM32_IRQ_INTERRUPTS+2)  /* 2:  Tamper interrupt, or */
#define EFM32_IRQ_USART0_RX   (EFM32_IRQ_INTERRUPTS+3)  /* 3:  RTC global interrupt */
#define EFM32_IRQ_USART0_TX   (EFM32_IRQ_INTERRUPTS+4)  /* 4:  Flash global interrupt */
#define EFM32_IRQ_ACMP        (EFM32_IRQ_INTERRUPTS+5)  /* 5:  RCC global interrupt */
#define EFM32_IRQ_ADC0        (EFM32_IRQ_INTERRUPTS+6)  /* 6:  EXTI Line 0 interrupt */
#define EFM32_IRQ_EXTI1       (EFM32_IRQ_INTERRUPTS+7)  /* 7:  EXTI Line 1 interrupt */
#define EFM32_IRQ_EXTI2       (EFM32_IRQ_INTERRUPTS+8)  /* 8:  EXTI Line 2 interrupt, or */
#define EFM32_IRQ_TSC         (EFM32_IRQ_INTERRUPTS+8)  /* 8:  TSC interrupt */
#define EFM32_IRQ_EXTI3       (EFM32_IRQ_INTERRUPTS+9)  /* 9:  EXTI Line 3 interrupt */
#define EFM32_IRQ_EXTI4       (EFM32_IRQ_INTERRUPTS+10) /* 10: EXTI Line 4 interrupt */

#define NR_VECTORS            (EFM32_IRQ_INTERRUPTS+82)
#define NR_IRQS               (EFM32_IRQ_INTERRUPTS+82)

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
****************************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_EFM32TG8XX_IRQ_H */

