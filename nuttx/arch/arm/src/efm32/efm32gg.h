/************************************************************************************
 * arch/arm/src/efm32/efm32gg.h
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

#ifndef __ARCH_ARM_SRC_EFM32_EFM32TG_H
#define __ARCH_ARM_SRC_EFM32_EFM32TG_H

#include "DWT_cm3.h"
#include "TPI_cm3.h"
#include "ITM_cm3.h"

/* Part number capabilities */
#define EFM32_ACMP_NBR          2 
#define EFM32_USART_NBR         3 
#define EFM32_UART_NBR          2 
#define EFM32_TIMER_NBR         3 
#define EFM32_LEUART_NBR        2 
#define EFM32_LETIMER_NBR       1 
#define EFM32_PCNT_NBR          3 
#define EFM32_ADC_NBR           1 
#define EFM32_DAC_NBR           1 
#define EFM32_I2C_NBR           2 
#define EFM32_AES_NBR           1
#define EFM32_DMA_NBR           1
#define EFM32_MSC_NBR           1
#define EFM32_EMU_NBR           1
#define EFM32_RMU_NBR           1
#define EFM32_CMU_NBR           1
#define EFM32_LESENSE_NBR       1
#define EFM32_RTC_NBR           1
#define EFM32_BURTC_NBR         1
#define EFM32_GPIO_NBR          1
#define EFM32_GPIO_IRQ_NBR      16
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


#define AFCHANLOC_MAX   7

#include "chip/efm32gg_memorymap.h"

#include "EFM32GG/efm32gg_aes.h"
#include "EFM32GG/efm32gg_dma_ch.h"
#include "EFM32GG/efm32gg_dma.h"
#include "EFM32GG/efm32gg_msc.h"
#include "EFM32GG/efm32gg_emu.h"
#include "EFM32GG/efm32gg_rmu.h"
#include "EFM32GG/efm32gg_cmu.h"
#include "EFM32GG/efm32gg_lesense_st.h"
#include "EFM32GG/efm32gg_lesense_buf.h"
#include "EFM32GG/efm32gg_lesense_ch.h"
#include "EFM32GG/efm32gg_lesense.h"
#include "EFM32GG/efm32gg_rtc.h"
#include "EFM32GG/efm32gg_acmp.h"
#include "EFM32GG/efm32gg_usart.h"
#include "EFM32GG/efm32gg_timer_cc.h"
#include "EFM32GG/efm32gg_timer.h"
#include "EFM32GG/efm32gg_gpio_p.h"
#include "EFM32GG/efm32gg_gpio.h"
#include "EFM32GG/efm32gg_vcmp.h"
#include "EFM32GG/efm32gg_prs_ch.h"
#include "EFM32GG/efm32gg_prs.h"
#include "EFM32GG/efm32gg_leuart.h"
#include "EFM32GG/efm32gg_letimer.h"
#include "EFM32GG/efm32gg_pcnt.h"
#include "EFM32GG/efm32gg_adc.h"
#include "EFM32GG/efm32gg_dac.h"
#include "EFM32GG/efm32gg_i2c.h"
#include "EFM32GG/efm32gg_lcd.h"
#include "EFM32GG/efm32gg_wdog.h"
#include "EFM32GG/efm32gg_dma_descriptor.h"
#include "EFM32GG/efm32gg_devinfo.h"
#include "EFM32GG/efm32gg_romtable.h"
#include "EFM32GG/efm32gg_calibrate.h"


#endif
