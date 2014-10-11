/************************************************************************************
 * arch/arm/src/efm32/chip/efm32ggxxx_memorymap.h
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

#ifndef __ARCH_ARM_SRC_EFM32_CHIP_EFM32GGXXX_MEMORYMAP_H_
#define __ARCH_ARM_SRC_EFM32_CHIP_EFM32GGXXX_MEMORYMAP_H_

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Memory Base addresses and limits */
#define FLASH_MEM_BASE       ((uint32_t) 0x0UL)        /* FLASH base address  */
#define FLASH_MEM_SIZE       ((uint32_t) 0x10000000UL) /* FLASH available address space  */
#define FLASH_MEM_END        ((uint32_t) 0xFFFFFFFUL)  /* FLASH end address  */
#define FLASH_MEM_BITS       ((uint32_t) 0x28UL)       /* FLASH used bits  */
#define AES_MEM_BASE         ((uint32_t) 0x400E0000UL) /* AES base address  */
#define AES_MEM_SIZE         ((uint32_t) 0x400UL)      /* AES available address space  */
#define AES_MEM_END          ((uint32_t) 0x400E03FFUL) /* AES end address  */
#define AES_MEM_BITS         ((uint32_t) 0x10UL)       /* AES used bits  */
#define PER_MEM_BASE         ((uint32_t) 0x40000000UL) /* PER base address  */
#define PER_MEM_SIZE         ((uint32_t) 0xE0000UL)    /* PER available address space  */
#define PER_MEM_END          ((uint32_t) 0x400DFFFFUL) /* PER end address  */
#define PER_MEM_BITS         ((uint32_t) 0x20UL)       /* PER used bits  */
#define RAM_MEM_BASE         ((uint32_t) 0x20000000UL) /* RAM base address  */
#define RAM_MEM_SIZE         ((uint32_t) 0x40000UL)    /* RAM available address space  */
#define RAM_MEM_END          ((uint32_t) 0x2003FFFFUL) /* RAM end address  */
#define RAM_MEM_BITS         ((uint32_t) 0x18UL)       /* RAM used bits  */
#define RAM_CODE_MEM_BASE    ((uint32_t) 0x10000000UL) /* RAM_CODE base address  */
#define RAM_CODE_MEM_SIZE    ((uint32_t) 0x4000UL)     /* RAM_CODE available address space  */
#define RAM_CODE_MEM_END     ((uint32_t) 0x10003FFFUL) /* RAM_CODE end address  */
#define RAM_CODE_MEM_BITS    ((uint32_t) 0x14UL)       /* RAM_CODE used bits  */

/* Bit banding area */
#define BITBAND_PER_BASE     ((uint32_t) 0x42000000UL) /* Peripheral Address Space bit-band area */
#define BITBAND_RAM_BASE     ((uint32_t) 0x22000000UL) /* SRAM Address Space bit-band area */

/* Flash and SRAM limits for EFM32XXXXXF32 */
#ifdef (CONFIG_EFM32_EFM32XXXXXF1024)
#define FLASH_SIZE           (0x00100000UL) /* Available Flash Memory */
#define SRAM_SIZE            (0x00001000UL) /* Available SRAM Memory */
#endif
#define FLASH_BASE           (0x00000000UL) /* Flash Base Address */
#define FLASH_SIZE           (0x00008000UL) /* Available Flash Memory */
#define FLASH_PAGE_SIZE      512            /* Flash Memory page size */
#define SRAM_BASE            (0x20000000UL) /* SRAM Base Address */
#define SRAM_SIZE            (0x00020000UL) /* Available SRAM Memory */
#define CM3_REV              0x201          /* Cortex-M3 Core revision r2p1 */
#define PRS_CHAN_COUNT       8              /* Number of PRS channels */
#define DMA_CHAN_COUNT       8              /* Number of DMA channels */

/* Peripheral base address for EFM32GGxxx */
#define AES_BASE          (0x400E0000UL) /* AES base address  */
#define PRS_BASE          (0x400CC000UL) /* PRS base address  */
#define RMU_BASE          (0x400CA000UL) /* RMU base address  */
#define CMU_BASE          (0x400C8000UL) /* CMU base address  */
#define EMU_BASE          (0x400C6000UL) /* EMU base address  */
#define USB_BASE          (0x400C4000UL) /* USB base address  */
#define DMA_BASE          (0x400C2000UL) /* DMA base address  */
#define MSC_BASE          (0x400C0000UL) /* MSC base address  */
#define LESENSE_BASE      (0x4008C000UL) /* LESENSE base address  */
#define LCD_BASE          (0x4008A000UL) /* LCD base address  */
#define WDOG_BASE         (0x40088000UL) /* WDOG base address  */
#define PCNT2_BASE        (0x40086800UL) /* PCNT2 base address  */
#define PCNT1_BASE        (0x40086400UL) /* PCNT1 base address  */
#define PCNT0_BASE        (0x40086000UL) /* PCNT0 base address  */
#define LEUART1_BASE      (0x40084400UL) /* LEUART1 base address  */
#define LEUART0_BASE      (0x40084000UL) /* LEUART0 base address  */
#define LETIMER0_BASE     (0x40082000UL) /* LETIMER0 base address  */
#define BCKRTC_BASE       (0x40081000UL) /* BCKRTC base address  */
#define RTC_BASE          (0x40080000UL) /* RTC base address  */
#define TIMER3_BASE       (0x40010C00UL) /* TIMER3 base address  */
#define TIMER2_BASE       (0x40010800UL) /* TIMER2 base address  */
#define TIMER1_BASE       (0x40010400UL) /* TIMER1 base address  */
#define TIMER0_BASE       (0x40010000UL) /* TIMER0 base address  */
#define UART1_BASE        (0x4000E400UL) /* UART1 base address  */
#define UART0_BASE        (0x4000E000UL) /* UART0 base address  */
#define USART2_BASE       (0x4000C800UL) /* USART2 base address  */
#define USART1_BASE       (0x4000C400UL) /* USART1 base address  */
#define USART0_BASE       (0x4000C000UL) /* USART0 base address  */
#define I2C1_BASE         (0x4000A400UL) /* I2C0 base address  */
#define I2C0_BASE         (0x4000A000UL) /* I2C0 base address  */
#define EBI_BASE          (0x40008000UL) /* EBI base address  */
#define GPIO_BASE         (0x40006000UL) /* GPIO base address  */
#define DAC0_BASE         (0x40004000UL) /* DAC0 base address  */
#define ADC0_BASE         (0x40002000UL) /* ADC0 base address  */
#define ACMP1_BASE        (0x40001400UL) /* ACMP1 base address  */
#define ACMP0_BASE        (0x40001000UL) /* ACMP0 base address  */
#define VCMP_BASE         (0x40000000UL) /* VCMP base address  */


/* ROM specific region */
#define ROMTABLE_BASE     (0xE00FFFD0UL) /* ROMTABLE base address */

/* Flash specific region */
#define CALIBRATE_BASE    (0x0FE08000UL) /* CALIBRATE base address */
#define DEVINFO_BASE      (0x0FE081B0UL) /* DEVINFO base address */
#define LOCKBITS_BASE     (0x0FE04000UL) /* Lock-bits page base address */
#define USERDATA_BASE     (0x0FE00000UL) /* User data page base address */

/* Peripheral Memory */
#define AES          ((AES_TypeDef *)         AES_BASE)         /* AES base pointer */
#define PRS          ((PRS_TypeDef *)         PRS_BASE)         /* PRS base pointer */
#define RMU          ((RMU_TypeDef *)         RMU_BASE)         /* RMU base pointer */
#define CMU          ((CMU_TypeDef *)         CMU_BASE)         /* CMU base pointer */
#define EMU          ((EMU_TypeDef *)         EMU_BASE)         /* EMU base pointer */
#define USB          ((USB_TypeDef *)         USB_BASE)         /* USB base pointer */
#define DMA          ((DMA_TypeDef *)         DMA_BASE)         /* DMA base pointer */
#define MSC          ((MSC_TypeDef *)         MSC_BASE)         /* MSC base pointer */
#define LESENSE      ((LESENSE_TypeDef *)     LESENSE_BASE)     /* LESENSE base pointer */
#define LCD          ((LCD_TypeDef *)         LCD_BASE)         /* LCD base pointer */
#define WDOG         ((WDOG_TypeDef *)        WDOG_BASE)        /* WDOG base pointer */
#define PCNT2        ((PCNT_TypeDef *)        PCNT2_BASE)       /* PCNT2 base pointer */
#define PCNT1        ((PCNT_TypeDef *)        PCNT1_BASE)       /* PCNT1 base pointer */
#define PCNT0        ((PCNT_TypeDef *)        PCNT0_BASE)       /* PCNT0 base pointer */
#define LEUART1      ((LEUART_TypeDef *)      LEUART1_BASE)     /* LEUART1 base pointer */
#define LEUART0      ((LEUART_TypeDef *)      LEUART0_BASE)     /* LEUART0 base pointer */
#define LETIMER0     ((LETIMER_TypeDef *)     LETIMER0_BASE)    /* LETIMER0 base pointer */
#define BCKRTC       ((BCKRTC_TypeDef *)      BCKRTC_BASE)      /* BCKRTC base pointer */
#define RTC          ((RTC_TypeDef *)         RTC_BASE)         /* RTC base pointer */
#define TIMER3       ((TIMER_TypeDef *)       TIMER3_BASE)      /* TIMER3 base pointer */
#define TIMER2       ((TIMER_TypeDef *)       TIMER2_BASE)      /* TIMER2 base pointer */
#define TIMER1       ((TIMER_TypeDef *)       TIMER1_BASE)      /* TIMER1 base pointer */
#define TIMER0       ((TIMER_TypeDef *)       TIMER0_BASE)      /* TIMER0 base pointer */
#define UART1        ((UART_TypeDef *)        UART1_BASE)       /* UART1 base pointer */
#define UART0        ((UART_TypeDef *)        UART0_BASE)       /* UART0 base pointer */
#define USART2       ((USART_TypeDef *)       USART2_BASE)      /* USART2 base pointer */
#define USART1       ((USART_TypeDef *)       USART1_BASE)      /* USART1 base pointer */
#define USART0       ((USART_TypeDef *)       USART0_BASE)      /* USART0 base pointer */
#define I2C1         ((I2C_TypeDef *)         I2C1_BASE)        /* I2C1 base pointer */
#define I2C0         ((I2C_TypeDef *)         I2C0_BASE)        /* I2C0 base pointer */
#define EBI          ((EBI_TypeDef *)         EBI_BASE)         /* EBI base pointer */
#define GPIO         ((GPIO_TypeDef *)        GPIO_BASE)        /* GPIO base pointer */
#define DAC0         ((DAC_TypeDef *)         DAC0_BASE)        /* DAC0 base pointer */
#define ADC0         ((ADC_TypeDef *)         ADC0_BASE)        /* ADC0 base pointer */
#define ACMP1        ((ACMP_TypeDef *)        ACMP1_BASE)       /* ACMP1 base pointer */
#define ACMP0        ((ACMP_TypeDef *)        ACMP0_BASE)       /* ACMP0 base pointer */
#define VCMP         ((VCMP_TypeDef *)        VCMP_BASE)        /* VCMP base pointer */


/* ROM specific region */
#define ROMTABLE     ((ROMTABLE_TypeDef *)    ROMTABLE_BASE)    /* ROMTABLE base pointer */

/* Flash specific region */
#define CALIBRATE    ((CALIBRATE_TypeDef *)   CALIBRATE_BASE)   /* CALIBRATE base pointer */
#define DEVINFO      ((DEVINFO_TypeDef *)     DEVINFO_BASE)     /* DEVINFO base pointer */


#endif 

