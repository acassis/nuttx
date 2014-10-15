/************************************************************************************
 * arch/arm/src/efm32/efm32tg.h
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

#   include "core_cmInstr.h"

/* Part number capabilities */
#define ACMP_PRESENT          2 
#define USART_PRESENT         2 
#define TIMER_PRESENT         2 
#define LEUART_PRESENT        1 
#define LETIMER_PRESENT       1 
#define PCNT_PRESENT          1 
#define ADC_PRESENT           1 
#define DAC_PRESENT           1 
#define I2C_PRESENT           1 
#define AES_PRESENT           1
#define DMA_PRESENT           1
#define LE_PRESENT            1
#define MSC_PRESENT           1
#define EMU_PRESENT           1
#define RMU_PRESENT           1
#define CMU_PRESENT           1
#define LESENSE_PRESENT       1
#define RTC_PRESENT           1
#define GPIO_PRESENT          1
#define GPIO_IRQ_PRESENT      16
#define VCMP_PRESENT          1
#define PRS_PRESENT           1
#define OPAMP_PRESENT         1
#define LCD_PRESENT           1
#define HFXTAL_PRESENT        1
#define LFXTAL_PRESENT        1
#define WDOG_PRESENT          1
#define DBG_PRESENT           1
#define BOOTLOADER_PRESENT    1
#define ANALOG_PRESENT        1


#define AFCHANLOC_MAX   7

#include "chip/efm32tg_memorymap.h"

//#include "EFM32TG/efm32tg_aes.h"
//#include "EFM32TG/efm32tg_dma_ch.h"
//#include "EFM32TG/efm32tg_dma.h"
//#include "EFM32TG/efm32tg_msc.h"
//#include "EFM32TG/efm32tg_emu.h"
//#include "EFM32TG/efm32tg_rmu.h"
#include "EFM32TG/efm32tg_cmu.h"
//#include "EFM32TG/efm32tg_lesense_st.h"
//#include "EFM32TG/efm32tg_lesense_buf.h"
//#include "EFM32TG/efm32tg_lesense_ch.h"
//#include "EFM32TG/efm32tg_lesense.h"
//#include "EFM32TG/efm32tg_rtc.h"
//#include "EFM32TG/efm32tg_acmp.h"
//#include "EFM32TG/efm32tg_usart.h"
//#include "EFM32TG/efm32tg_timer_cc.h"
//#include "EFM32TG/efm32tg_timer.h"
#include "EFM32TG/efm32tg_gpio_p.h"
#include "EFM32TG/efm32tg_gpio.h"
//#include "EFM32TG/efm32tg_vcmp.h"
//#include "EFM32TG/efm32tg_prs_ch.h"
//#include "EFM32TG/efm32tg_prs.h"
//#include "EFM32TG/efm32tg_leuart.h"
//#include "EFM32TG/efm32tg_letimer.h"
//#include "EFM32TG/efm32tg_pcnt.h"
//#include "EFM32TG/efm32tg_adc.h"
//#include "EFM32TG/efm32tg_dac.h"
//#include "EFM32TG/efm32tg_i2c.h"
//#include "EFM32TG/efm32tg_lcd.h"
//#include "EFM32TG/efm32tg_wdog.h"
//#include "EFM32TG/efm32tg_dma_descriptor.h"
//#include "EFM32TG/efm32tg_devinfo.h"
//#include "EFM32TG/efm32tg_romtable.h"
//#include "EFM32TG/efm32tg_calibrate.h"


#endif
