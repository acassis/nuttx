/************************************************************************************
 * arch/arm/src/efm32/EFM32GG/efm32gg_dmareq.h
 *
 *  Copyright 2014 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.@n
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.@n
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Laboratories, Inc.
 * has no obligation to support this Software. Silicon Laboratories, Inc. is
 * providing the Software "AS IS", with no express or implied warranties of any
 * kind, including, but not limited to, any implied warranties of
 * merchantability or fitness for any particular purpose or warranties against
 * infringement of any proprietary rights of a third party.
 *
 * Silicon Laboratories, Inc. will not be liable for any consequential,
 * incidental, or special damages, or any other relief, or for any claim by
 * any third party, arising from your use of this Software.
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

#ifndef __ARCH_ARM_SRC_EFM32_EFM32GG_EFM32GG_DMAREQ_H_
#define __ARCH_ARM_SRC_EFM32_EFM32GG_EFM32GG_DMAREQ_H_

#define DMAREQ_ADC0_SINGLE            ((8 << 16) + 0)  /* DMA channel select for ADC0_SINGLE */
#define DMAREQ_ADC0_SCAN              ((8 << 16) + 1)  /* DMA channel select for ADC0_SCAN */
#define DMAREQ_DAC0_CH0               ((10 << 16) + 0) /* DMA channel select for DAC0_CH0 */
#define DMAREQ_DAC0_CH1               ((10 << 16) + 1) /* DMA channel select for DAC0_CH1 */
#define DMAREQ_USART0_RXDATAV         ((12 << 16) + 0) /* DMA channel select for USART0_RXDATAV */
#define DMAREQ_USART0_TXBL            ((12 << 16) + 1) /* DMA channel select for USART0_TXBL */
#define DMAREQ_USART0_TXEMPTY         ((12 << 16) + 2) /* DMA channel select for USART0_TXEMPTY */
#define DMAREQ_USART1_RXDATAV         ((13 << 16) + 0) /* DMA channel select for USART1_RXDATAV */
#define DMAREQ_USART1_TXBL            ((13 << 16) + 1) /* DMA channel select for USART1_TXBL */
#define DMAREQ_USART1_TXEMPTY         ((13 << 16) + 2) /* DMA channel select for USART1_TXEMPTY */
#define DMAREQ_USART1_RXDATAVRIGHT    ((13 << 16) + 3) /* DMA channel select for USART1_RXDATAVRIGHT */
#define DMAREQ_USART1_TXBLRIGHT       ((13 << 16) + 4) /* DMA channel select for USART1_TXBLRIGHT */
#define DMAREQ_USART2_RXDATAV         ((14 << 16) + 0) /* DMA channel select for USART2_RXDATAV */
#define DMAREQ_USART2_TXBL            ((14 << 16) + 1) /* DMA channel select for USART2_TXBL */
#define DMAREQ_USART2_TXEMPTY         ((14 << 16) + 2) /* DMA channel select for USART2_TXEMPTY */
#define DMAREQ_USART2_RXDATAVRIGHT    ((14 << 16) + 3) /* DMA channel select for USART2_RXDATAVRIGHT */
#define DMAREQ_USART2_TXBLRIGHT       ((14 << 16) + 4) /* DMA channel select for USART2_TXBLRIGHT */
#define DMAREQ_LEUART0_RXDATAV        ((16 << 16) + 0) /* DMA channel select for LEUART0_RXDATAV */
#define DMAREQ_LEUART0_TXBL           ((16 << 16) + 1) /* DMA channel select for LEUART0_TXBL */
#define DMAREQ_LEUART0_TXEMPTY        ((16 << 16) + 2) /* DMA channel select for LEUART0_TXEMPTY */
#define DMAREQ_LEUART1_RXDATAV        ((17 << 16) + 0) /* DMA channel select for LEUART1_RXDATAV */
#define DMAREQ_LEUART1_TXBL           ((17 << 16) + 1) /* DMA channel select for LEUART1_TXBL */
#define DMAREQ_LEUART1_TXEMPTY        ((17 << 16) + 2) /* DMA channel select for LEUART1_TXEMPTY */
#define DMAREQ_I2C0_RXDATAV           ((20 << 16) + 0) /* DMA channel select for I2C0_RXDATAV */
#define DMAREQ_I2C0_TXBL              ((20 << 16) + 1) /* DMA channel select for I2C0_TXBL */
#define DMAREQ_I2C1_RXDATAV           ((21 << 16) + 0) /* DMA channel select for I2C1_RXDATAV */
#define DMAREQ_I2C1_TXBL              ((21 << 16) + 1) /* DMA channel select for I2C1_TXBL */
#define DMAREQ_TIMER0_UFOF            ((24 << 16) + 0) /* DMA channel select for TIMER0_UFOF */
#define DMAREQ_TIMER0_CC0             ((24 << 16) + 1) /* DMA channel select for TIMER0_CC0 */
#define DMAREQ_TIMER0_CC1             ((24 << 16) + 2) /* DMA channel select for TIMER0_CC1 */
#define DMAREQ_TIMER0_CC2             ((24 << 16) + 3) /* DMA channel select for TIMER0_CC2 */
#define DMAREQ_TIMER1_UFOF            ((25 << 16) + 0) /* DMA channel select for TIMER1_UFOF */
#define DMAREQ_TIMER1_CC0             ((25 << 16) + 1) /* DMA channel select for TIMER1_CC0 */
#define DMAREQ_TIMER1_CC1             ((25 << 16) + 2) /* DMA channel select for TIMER1_CC1 */
#define DMAREQ_TIMER1_CC2             ((25 << 16) + 3) /* DMA channel select for TIMER1_CC2 */
#define DMAREQ_TIMER2_UFOF            ((26 << 16) + 0) /* DMA channel select for TIMER2_UFOF */
#define DMAREQ_TIMER2_CC0             ((26 << 16) + 1) /* DMA channel select for TIMER2_CC0 */
#define DMAREQ_TIMER2_CC1             ((26 << 16) + 2) /* DMA channel select for TIMER2_CC1 */
#define DMAREQ_TIMER2_CC2             ((26 << 16) + 3) /* DMA channel select for TIMER2_CC2 */
#define DMAREQ_TIMER3_UFOF            ((27 << 16) + 0) /* DMA channel select for TIMER3_UFOF */
#define DMAREQ_TIMER3_CC0             ((27 << 16) + 1) /* DMA channel select for TIMER3_CC0 */
#define DMAREQ_TIMER3_CC1             ((27 << 16) + 2) /* DMA channel select for TIMER3_CC1 */
#define DMAREQ_TIMER3_CC2             ((27 << 16) + 3) /* DMA channel select for TIMER3_CC2 */
#define DMAREQ_UART0_RXDATAV          ((44 << 16) + 0) /* DMA channel select for UART0_RXDATAV */
#define DMAREQ_UART0_TXBL             ((44 << 16) + 1) /* DMA channel select for UART0_TXBL */
#define DMAREQ_UART0_TXEMPTY          ((44 << 16) + 2) /* DMA channel select for UART0_TXEMPTY */
#define DMAREQ_UART1_RXDATAV          ((45 << 16) + 0) /* DMA channel select for UART1_RXDATAV */
#define DMAREQ_UART1_TXBL             ((45 << 16) + 1) /* DMA channel select for UART1_TXBL */
#define DMAREQ_UART1_TXEMPTY          ((45 << 16) + 2) /* DMA channel select for UART1_TXEMPTY */
#define DMAREQ_MSC_WDATA              ((48 << 16) + 0) /* DMA channel select for MSC_WDATA */
#define DMAREQ_AES_DATAWR             ((49 << 16) + 0) /* DMA channel select for AES_DATAWR */
#define DMAREQ_AES_XORDATAWR          ((49 << 16) + 1) /* DMA channel select for AES_XORDATAWR */
#define DMAREQ_AES_DATARD             ((49 << 16) + 2) /* DMA channel select for AES_DATARD */
#define DMAREQ_AES_KEYWR              ((49 << 16) + 3) /* DMA channel select for AES_KEYWR */
#define DMAREQ_LESENSE_BUFDATAV       ((50 << 16) + 0) /* DMA channel select for LESENSE_BUFDATAV */
#define DMAREQ_EBI_PXL0EMPTY          ((51 << 16) + 0) /* DMA channel select for EBI_PXL0EMPTY */
#define DMAREQ_EBI_PXL1EMPTY          ((51 << 16) + 1) /* DMA channel select for EBI_PXL1EMPTY */
#define DMAREQ_EBI_PXLFULL            ((51 << 16) + 2) /* DMA channel select for EBI_PXLFULL */
#define DMAREQ_EBI_DDEMPTY            ((51 << 16) + 3) /* DMA channel select for EBI_DDEMPTY */

#endif

