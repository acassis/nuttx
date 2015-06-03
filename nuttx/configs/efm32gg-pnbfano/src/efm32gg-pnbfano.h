/****************************************************************************
 * configs/efm32gg-pnbfano/src/efm32gg-pnbfano.h
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
 *
 * PA8                   LCD_D0
 * PA9                   LCD_D1
 * PA10                  LCD_D2
 * PA11                  LCD_D3
 * PA12                  LCD_D4
 * PA13                  LCD_D5
 * PA14                  LCD_D6
 * PA15                  LCD_D7
 * 
 * PA15                  LCD_PWM (Back light)
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

#define GPIO_LCD_PORT           GPIO_PORTE
#define GPIO_LCD_PORT_SHIFT     8
#define GPIO_LCD_PORT_BUS_WIDTH 8
#define GPIO_LCD_PORT_MASK      (0xFF << GPIO_LCD_PORT_SHIFT)

//#define GPIO_LCD_PWM_PIN      (GPIO_OUTPUT_PUSHPULL|GPIO_PORTC|GPIO_PIN0|GPIO_OUTPUT_CLEAR)
#define GPIO_LCD_PWM_DEV      "/dev/pwm0"


/* ADC GPIO:
 *
 * The pnbfano board has a Pulse Per Second GPIO input. 
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * VBAT(PD6)              VBAT 
 * VTEMP(PD5)             Temperature
 * --------------------- ---------------------
 */

#  define GPIO_VBAT     (GPIO_INPUT|GPIO_PORTD|GPIO_PIN6)
#  define GPIO_VTEMP    (GPIO_INPUT|GPIO_PORTD|GPIO_PIN5)

/* PPS GPIO:
 *
 * The pnbfano board has a Pulse Per Second GPIO input. 
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * PPS(PA8)              PPS
 * PPS GPIO_IRQ_PA8
 * --------------------- ---------------------
 */

#  define GPIO_PPS      (GPIO_INPUT_PULLDOWN|\
                         GPIO_INT_RISING|\
                         GPIO_PORTA|\
                         GPIO_PIN8\
                        )
#  define GPIO_PPS_IRQ  (EFM32_IRQ_EXTI8)

/* Chrono GPIO:
 *
 * The pnbfano board has a Lap chronometer GPIO input. 
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * CHRONO (PD3)          CHRONO
 * CHRONO GPIO_IRQ_PD3
 * --------------------- ---------------------
 */

#  define GPIO_CHRONO     (GPIO_INPUT_PULLUP|\
                           GPIO_INT_FALLING|\
                           GPIO_PORTD|\
                           GPIO_PIN3\
                          )
#  define GPIO_CHRONO_IRQ (EFM32_IRQ_EXTI3)

/* Lcd Buttons:
 *
 * The pnbfano board has four buttons, BUT1-4. Each is grounded and so should
 * have a weak pull-up so that it will be sensed as "1" when open and "0"
 * when closed.
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * LCD_D4(PE8)           KEYCODE_NONE
 * LCD_D5(PE9)           KEYCODE_NONE
 * LCD_D6(PE10)          KEYCODE_NONE
 * LCD_D7(PE11)          KEYCODE_NONE
 * LCD_D4(PE12)          KEYCODE_LEFT
 * LCD_D5(PE13)          KEYCODE_DOWN
 * LCD_D6(PE14)          KEYCODE_UP
 * LCD_D7(PE15)          KEYCODE_RIGHT
 * --------------------- ---------------------
 */

#  define GPIO_LCD_KEY_D0 (EFM32_LCD_KEY_NONE)
#  define GPIO_LCD_KEY_D1 (EFM32_LCD_KEY_NONE)
#  define GPIO_LCD_KEY_D2 (EFM32_LCD_KEY_NONE)
#  define GPIO_LCD_KEY_D3 (EFM32_LCD_KEY_NONE)
#  define GPIO_LCD_KEY_D4 (EFM32_LCD_KEY_SPECIAL(KEYCODE_RIGHT  ))
#  define GPIO_LCD_KEY_D5 (EFM32_LCD_KEY_SPECIAL(KEYCODE_UP     ))
#  define GPIO_LCD_KEY_D6 (EFM32_LCD_KEY_SPECIAL(KEYCODE_LEFT   ))
#  define GPIO_LCD_KEY_D7 (EFM32_LCD_KEY_SPECIAL(KEYCODE_DOWN   ))


/* SPI:
 *
 * The pnbfano board one SPI with two CS SDCARD SPI_SDCARD and WIFI (EXT_SPI). 
 * both use USART0 as SPI device
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * PC4                   For external spi (WIFI)
 * PC5                   for SDCARD
 * --------------------- ---------------------
 */

#define GPIO_SDCARD_SPI_CS  (GPIO_OUTPUT_PUSHPULL|GPIO_PORTC|GPIO_PIN8|GPIO_OUTPUT_SET)

#define GPIO_WIFI_CS     (GPIO_OUTPUT_PUSHPULL|GPIO_PORTC|GPIO_PIN7|GPIO_OUTPUT_SET)
#define GPIO_WIFI_IRQ       (GPIO_INPUT_PULLUP|GPIO_PORTC|GPIO_PIN6)
#define GPIO_WIFI_EN        (GPIO_OUTPUT_PUSHPULL|GPIO_PORTD|GPIO_PIN7|GPIO_OUTPUT_CLEAR)


#define PNBFANO_SDCARD_EXT_SPINO  0

/* SDHC Slot: */
#define PNBFANO_SDCARD_SLOTNO 0

/* SDHC Slot: */
#define PNBFANO_SDCARD_MINOR 0



/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int efm32_slow_poll_init(void);

int efm32_initialize_spi_devices(void);

int efm32_emu_initialize(void);
int efm32_acmp_initialize(void);
int efm32_vcmp_initialize(void);

int efm32_usbdev_is_connected(void);
int efm32_usbdev_is_enable(void);
int efm32_usbdev_disable_usbmsc(void);
int efm32_usbdev_enable_usbmsc(void);


int up_lcdinitialize(void);

int st7565_lock(void);

void st7565_unlock(void);

#ifdef CONFIG_PNBFANO_GPIO_PPS
int efm32_gpio_pps_init( void );
#endif

#ifdef CONFIG_PNBFANO_GPIO_CHRONO
int efm32_gpio_chrono_init( void );
#endif

#if CONFIG_USBMSC
int  efm32_usbdev_init(void);
void efm32_usbdev_slow_poll(void);
#endif

#endif /* __CONFIGS_EFM32_DK3650_INCLUDE_BOARD_H */
