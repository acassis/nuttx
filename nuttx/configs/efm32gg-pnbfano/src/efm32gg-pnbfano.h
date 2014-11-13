/****************************************************************************
 * configs/olimex-efm32g880f128-stk/src/efm32gg-pnbfano.h
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
 ****************************************************************************/

#ifndef __CONFIGS_EFM32GG_PNBFANO_SRC_EFM32GG_PNBFANO_H
#define __CONFIGS_EFM32GG_PNBFANO_SRC_EFM32GG_PNBFANO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LCD:
 *
 * The PnbFano board has NHD‐C12864KGZ wired by 8080 bus.
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * PA0                   /LCD_CS
 * PA1                   /LCD_RST
 * PA2                   LCD_A0
 * PA3                   /LCD_WR
 * PA4                   /LCD_RD
 * --------------------- ---------------------
 */

#define GPIO_LCD_CS       (GPIO_OUTPUT_PUSHPULL|GPIO_PORTA|GPIO_PIN0|GPIO_OUTPUT_SET)
#define GPIO_LCD_RST      (GPIO_OUTPUT_PUSHPULL|GPIO_PORTA|GPIO_PIN1|GPIO_OUTPUT_SET)
#define GPIO_LCD_A0       (GPIO_OUTPUT_PUSHPULL|GPIO_PORTA|GPIO_PIN2|GPIO_OUTPUT_SET)
#define GPIO_LCD_WR       (GPIO_OUTPUT_PUSHPULL|GPIO_PORTA|GPIO_PIN3|GPIO_OUTPUT_SET)
#define GPIO_LCD_RD       (GPIO_OUTPUT_PUSHPULL|GPIO_PORTA|GPIO_PIN4|GPIO_OUTPUT_SET)

#define GPIO_LCD_D0       (GPIO_OUTPUT_PUSHPULL|GPIO_PORTE|GPIO_PIN8 |GPIO_OUTPUT_SET)
#define GPIO_LCD_D1       (GPIO_OUTPUT_PUSHPULL|GPIO_PORTE|GPIO_PIN9 |GPIO_OUTPUT_SET)
#define GPIO_LCD_D2       (GPIO_OUTPUT_PUSHPULL|GPIO_PORTE|GPIO_PIN10|GPIO_OUTPUT_SET)
#define GPIO_LCD_D3       (GPIO_OUTPUT_PUSHPULL|GPIO_PORTE|GPIO_PIN11|GPIO_OUTPUT_SET)
#define GPIO_LCD_D4       (GPIO_OUTPUT_PUSHPULL|GPIO_PORTE|GPIO_PIN12|GPIO_OUTPUT_SET)
#define GPIO_LCD_D5       (GPIO_OUTPUT_PUSHPULL|GPIO_PORTE|GPIO_PIN13|GPIO_OUTPUT_SET)
#define GPIO_LCD_D6       (GPIO_OUTPUT_PUSHPULL|GPIO_PORTE|GPIO_PIN14|GPIO_OUTPUT_SET)
#define GPIO_LCD_D7       (GPIO_OUTPUT_PUSHPULL|GPIO_PORTE|GPIO_PIN15|GPIO_OUTPUT_SET)

#define GPIO_LCD_PORT       GPIO_PORTE
#define GPIO_LCD_PORT_SHIFT 8


/* Buttons:
 *
 * The pnbfano board has four buttons, BUT1-4. Each is grounded and so should
 * have a weak pull-up so that it will be sensed as "1" when open and "0"
 * when closed.
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * PE12                  BUT1
 * PE13                  BUT2
 * PE14                  BUT3
 * PE15                  BUT4
 * --------------------- ---------------------
 */

#ifdef CONFIG_EFM32_GPIO_IRQ
#  define GPIO_BUTTON_1 (GPIO_INPUT_PULLUP|GPIO_INT_BOTH|GPIO_PORTE|GPIO_PIN12)
#  define GPIO_BUTTON_2 (GPIO_INPUT_PULLUP|GPIO_INT_BOTH|GPIO_PORTE|GPIO_PIN13)
#  define GPIO_BUTTON_3 (GPIO_INPUT_PULLUP|GPIO_INT_BOTH|GPIO_PORTE|GPIO_PIN14)
#  define GPIO_BUTTON_4 (GPIO_INPUT_PULLUP|GPIO_INT_BOTH|GPIO_PORTE|GPIO_PIN15)

#  define GPIO_IRQ_BUTTON_1 EFM32_IRQ_EXTI12
#  define GPIO_IRQ_BUTTON_2 EFM32_IRQ_EXTI13
#  define GPIO_IRQ_BUTTON_3 EFM32_IRQ_EXTI14
#  define GPIO_IRQ_BUTTON_4 EFM32_IRQ_EXTI15
#endif

/* SPI:
 *
 * The pnbfano board one SPI with two CS SDCARD SPI_SDCARD and WIFI (EXT_SPI). 
 * both use USART0 as SPI device
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * PC7                   For external spi (WIFI)
 * PC8                   for SDCARD
 * --------------------- ---------------------
 */

#define GPIO_EXT_SPI_CS     (GPIO_OUTPUT_PUSHPULL|GPIO_PORTC|GPIO_PIN7|GPIO_OUTPUT_SET)
#define GPIO_SDCARD_SPI_CS  (GPIO_OUTPUT_PUSHPULL|GPIO_PORTC|GPIO_PIN8|GPIO_OUTPUT_SET)

#define PNBFANO_SDCARD_EXT_SPINO  0

/* SDHC Slot: */
#define PNBFANO_SDCARD_SLOTNO 0

/* SDHC Slot: */
#define PNBFANO_SDCARD_MINOR 0

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int efm32_initialize_spi_devices(void);

int up_lcdinitialize(void);

#endif /* __CONFIGS_EFM32_DK3650_INCLUDE_BOARD_H */
