/************************************************************************************
 * arch/arm/src/efm32/chip/efm32tg8xx_memorymap.h
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

#ifndef __ARCH_ARM_SRC_EFM32_CHIP_EFM32F10XXX_MEMORYMAP_H_
#define __ARCH_ARM_SRC_EFM32_CHIP_EFM32F10XXX_MEMORYMAP_H_

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Memory Base addresses and limits */
#define EFM32_FLASH_MEM_BASE       ((uint32_t) 0x0UL)        /* FLASH base address  */
#define EFM32_FLASH_MEM_SIZE       ((uint32_t) 0x10000000UL) /* FLASH available address space  */
#define EFM32_FLASH_MEM_END        ((uint32_t) 0xFFFFFFFUL)  /* FLASH end address  */
#define EFM32_FLASH_MEM_BITS       ((uint32_t) 0x28UL)       /* FLASH used bits  */
#define EFM32_AES_MEM_BASE         ((uint32_t) 0x400E0000UL) /* AES base address  */
#define EFM32_AES_MEM_SIZE         ((uint32_t) 0x400UL)      /* AES available address space  */
#define EFM32_AES_MEM_END          ((uint32_t) 0x400E03FFUL) /* AES end address  */
#define EFM32_AES_MEM_BITS         ((uint32_t) 0x10UL)       /* AES used bits  */
#define EFM32_PER_MEM_BASE         ((uint32_t) 0x40000000UL) /* PER base address  */
#define EFM32_PER_MEM_SIZE         ((uint32_t) 0xE0000UL)    /* PER available address space  */
#define EFM32_PER_MEM_END          ((uint32_t) 0x400DFFFFUL) /* PER end address  */
#define EFM32_PER_MEM_BITS         ((uint32_t) 0x20UL)       /* PER used bits  */
#define EFM32_RAM_MEM_BASE         ((uint32_t) 0x20000000UL) /* RAM base address  */
#define EFM32_RAM_MEM_SIZE         ((uint32_t) 0x40000UL)    /* RAM available address space  */
#define EFM32_RAM_MEM_END          ((uint32_t) 0x2003FFFFUL) /* RAM end address  */
#define EFM32_RAM_MEM_BITS         ((uint32_t) 0x18UL)       /* RAM used bits  */
#define EFM32_RAM_CODE_MEM_BASE    ((uint32_t) 0x10000000UL) /* RAM_CODE base address  */
#define EFM32_RAM_CODE_MEM_SIZE    ((uint32_t) 0x4000UL)     /* RAM_CODE available address space  */
#define EFM32_RAM_CODE_MEM_END     ((uint32_t) 0x10003FFFUL) /* RAM_CODE end address  */
#define EFM32_RAM_CODE_MEM_BITS    ((uint32_t) 0x14UL)       /* RAM_CODE used bits  */

/* Bit banding area */
#define EFM32_BITBAND_PER_BASE     ((uint32_t) 0x42000000UL) /* Peripheral Address Space bit-band area */
#define EFM32_BITBAND_RAM_BASE     ((uint32_t) 0x22000000UL) /* SRAM Address Space bit-band area */

/* Flash and SRAM limits for EFM32TG840F32 */
#define EFM32_FLASH_BASE           (0x00000000UL) /* Flash Base Address */
#define EFM32_FLASH_SIZE           (0x00008000UL) /* Available Flash Memory */
#define EFM32_FLASH_PAGE_SIZE      512            /* Flash Memory page size */
#define EFM32_SRAM_BASE            (0x20000000UL) /* SRAM Base Address */
#define EFM32_SRAM_SIZE            (0x00001000UL) /* Available SRAM Memory */
#define EFM32_CM3_REV              0x201          /* Cortex-M3 Core revision r2p1 */
#define EFM32_PRS_CHAN_COUNT       8              /* Number of PRS channels */
#define EFM32_DMA_CHAN_COUNT       8              /* Number of DMA channels */

/* Peripheral base address for EFM32TG840F32 */
#define EFM32_AES_BASE          (0x400E0000UL) /* AES base address  */
#define EFM32_DMA_BASE          (0x400C2000UL) /* DMA base address  */
#define EFM32_MSC_BASE          (0x400C0000UL) /* MSC base address  */
#define EFM32_EMU_BASE          (0x400C6000UL) /* EMU base address  */
#define EFM32_RMU_BASE          (0x400CA000UL) /* RMU base address  */
#define EFM32_CMU_BASE          (0x400C8000UL) /* CMU base address  */
#define EFM32_LESENSE_BASE      (0x4008C000UL) /* LESENSE base address  */
#define EFM32_RTC_BASE          (0x40080000UL) /* RTC base address  */
#define EFM32_ACMP0_BASE        (0x40001000UL) /* ACMP0 base address  */
#define EFM32_ACMP1_BASE        (0x40001400UL) /* ACMP1 base address  */
#define EFM32_USART0_BASE       (0x4000C000UL) /* USART0 base address  */
#define EFM32_USART1_BASE       (0x4000C400UL) /* USART1 base address  */
#define EFM32_TIMER0_BASE       (0x40010000UL) /* TIMER0 base address  */
#define EFM32_TIMER1_BASE       (0x40010400UL) /* TIMER1 base address  */
#define EFM32_GPIO_BASE         (0x40006000UL) /* GPIO base address  */
#define EFM32_VCMP_BASE         (0x40000000UL) /* VCMP base address  */
#define EFM32_PRS_BASE          (0x400CC000UL) /* PRS base address  */
#define EFM32_LEUART0_BASE      (0x40084000UL) /* LEUART0 base address  */
#define EFM32_LETIMER0_BASE     (0x40082000UL) /* LETIMER0 base address  */
#define EFM32_PCNT0_BASE        (0x40086000UL) /* PCNT0 base address  */
#define EFM32_ADC0_BASE         (0x40002000UL) /* ADC0 base address  */
#define EFM32_DAC0_BASE         (0x40004000UL) /* DAC0 base address  */
#define EFM32_I2C0_BASE         (0x4000A000UL) /* I2C0 base address  */
#define EFM32_LCD_BASE          (0x4008A000UL) /* LCD base address  */
#define EFM32_WDOG_BASE         (0x40088000UL) /* WDOG base address  */
#define EFM32_CALIBRATE_BASE    (0x0FE08000UL) /* CALIBRATE base address */
#define EFM32_DEVINFO_BASE      (0x0FE081B0UL) /* DEVINFO base address */
#define EFM32_ROMTABLE_BASE     (0xE00FFFD0UL) /* ROMTABLE base address */
#define EFM32_LOCKBITS_BASE     (0x0FE04000UL) /* Lock-bits page base address */
#define EFM32_USERDATA_BASE     (0x0FE00000UL) /* User data page base address */

/* Peripheral Memory */
#define EFM32_AES          ((AES_TypeDef *)         EFM32_AES_BASE)         /* AES base pointer */
#define EFM32_DMA          ((DMA_TypeDef *)         EFM32_DMA_BASE)         /* DMA base pointer */
#define EFM32_MSC          ((MSC_TypeDef *)         EFM32_MSC_BASE)         /* MSC base pointer */
#define EFM32_EMU          ((EMU_TypeDef *)         EFM32_EMU_BASE)         /* EMU base pointer */
#define EFM32_RMU          ((RMU_TypeDef *)         EFM32_RMU_BASE)         /* RMU base pointer */
#define EFM32_CMU          ((CMU_TypeDef *)         EFM32_CMU_BASE)         /* CMU base pointer */
#define EFM32_LESENSE      ((LESENSE_TypeDef *)     EFM32_LESENSE_BASE)     /* LESENSE base pointer */
#define EFM32_RTC          ((RTC_TypeDef *)         EFM32_RTC_BASE)         /* RTC base pointer */
#define EFM32_ACMP0        ((ACMP_TypeDef *)        EFM32_ACMP0_BASE)       /* ACMP0 base pointer */
#define EFM32_ACMP1        ((ACMP_TypeDef *)        EFM32_ACMP1_BASE)       /* ACMP1 base pointer */
#define EFM32_USART0       ((USART_TypeDef *)       EFM32_USART0_BASE)      /* USART0 base pointer */
#define EFM32_USART1       ((USART_TypeDef *)       EFM32_USART1_BASE)      /* USART1 base pointer */
#define EFM32_TIMER0       ((TIMER_TypeDef *)       EFM32_TIMER0_BASE)      /* TIMER0 base pointer */
#define EFM32_TIMER1       ((TIMER_TypeDef *)       EFM32_TIMER1_BASE)      /* TIMER1 base pointer */
#define EFM32_GPIO         ((GPIO_TypeDef *)        EFM32_GPIO_BASE)        /* GPIO base pointer */
#define EFM32_VCMP         ((VCMP_TypeDef *)        EFM32_VCMP_BASE)        /* VCMP base pointer */
#define EFM32_PRS          ((PRS_TypeDef *)         EFM32_PRS_BASE)         /* PRS base pointer */
#define EFM32_LEUART0      ((LEUART_TypeDef *)      EFM32_LEUART0_BASE)     /* LEUART0 base pointer */
#define EFM32_LETIMER0     ((LETIMER_TypeDef *)     EFM32_LETIMER0_BASE)    /* LETIMER0 base pointer */
#define EFM32_PCNT0        ((PCNT_TypeDef *)        EFM32_PCNT0_BASE)       /* PCNT0 base pointer */
#define EFM32_ADC0         ((ADC_TypeDef *)         EFM32_ADC0_BASE)        /* ADC0 base pointer */
#define EFM32_DAC0         ((DAC_TypeDef *)         EFM32_DAC0_BASE)        /* DAC0 base pointer */
#define EFM32_I2C0         ((I2C_TypeDef *)         EFM32_I2C0_BASE)        /* I2C0 base pointer */
#define EFM32_LCD          ((LCD_TypeDef *)         EFM32_LCD_BASE)         /* LCD base pointer */
#define EFM32_WDOG         ((WDOG_TypeDef *)        EFM32_WDOG_BASE)        /* WDOG base pointer */
#define EFM32_CALIBRATE    ((CALIBRATE_TypeDef *)   EFM32_CALIBRATE_BASE)   /* CALIBRATE base pointer */
#define EFM32_DEVINFO      ((DEVINFO_TypeDef *)     EFM32_DEVINFO_BASE)     /* DEVINFO base pointer */
#define EFM32_ROMTABLE     ((ROMTABLE_TypeDef *)    EFM32_ROMTABLE_BASE)    /* ROMTABLE base pointer */

#endif 

