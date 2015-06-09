/****************************************************************************
 * configs/efm32-pnbfano/include/board.h
 *
 *   Copyright (C) 2014 Pierre-Noel Bouteville. All rights reserved.
 *   Author: Pierre-Noel Bouteville <pnb990@gmail.com>
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

#ifndef __CONFIGS_EFM32_PNBFANO_INCLUDE_BOARD_H
#define __CONFIGS_EFM32_PNBFANO_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Clocking *****************************************************************/
/* Clock Sources
 *   - 1-28 MHz High Frequency RC Oscillator (HFRCO)
 *   - 4-32 MHz High Frequency Crystal Oscillator (HFXO)
 *   - 32.768 kHz Low Frequency RC Oscillator (LFRCO)
 *   - 32.768 kHz Low Frequency Crystal Oscillator (LFXO)
 *   - 1KHz Ultra Low Frequency RC Oscillator (ULFRCO)
 *
 * The device boots with 14 MHz HFRCO as the HFCLK source.
 */

#define BOARD_HAVE_HFXO        1        /* Have High frequency crystal oscillator */
#define BOARD_HAVE_LFXO        1        /* Have Low frequency crystal oscillator */

#define BOARD_HFRCO_FREQUENCY  14000000 /* 14MHz on reset */
#define BOARD_HFXO_FREQUENCY   48000000 /* 32MHz crystal on board */
#define BOARD_LFRCO_FREQUENCY  32768    /* Low frequency oscillator */
#define BOARD_LFXO_FREQUENCY   32768    /* 32KHz crystal on board */
#define BOARD_ULFRCO_FREQUNCY  1000     /* Ultra low frequency oscillator */

#if BOARD_HAVE_HFXO
#   define BOARD_SYSTEM_FREQUENCY  BOARD_HFXO_FREQUENCY
#else
#   define BOARD_SYSTEM_FREQUENCY  BOARD_HFRCO_FREQUENCY
#endif

/* HFCLK - High Frequency Clock
 *
 * HFCLK is the selected High Frequency Clock. This clock is used by the CMU
 * and drives the two prescalers that generate HFCORECLK and HFPERCLK. The
 * HFCLK can be driven by a high-frequency oscillator (HFRCO or HFXO) or one
 * of the low-frequency oscillators (LFRCO or LFXO). By default the HFRCO is
 * selected.
 */

#define BOARD_HFCLKSEL            _CMU_CMD_HFCLKSEL_HFXO
#define BOARD_HFCLKDIV            0     /* Does not apply to EFM32G */
#define BOARD_HFCLK_FREQUENCY     BOARD_SYSTEM_FREQUENCY

/* HFCORECLK - High Frequency Core Clock
 *
 * HFCORECLK is a prescaled version of HFCLK. This clock drives the Core
 * Modules, which consists of the CPU and modules that are tightly coupled
 * to the CPU, e.g. MSC, DMA etc.  The frequency of HFCORECLK is set using
 * the CMU_HFCORECLKDIV register.
 */

#define BOARD_HFCORECLKDIV        _CMU_HFCORECLKDIV_HFCORECLKDIV_DEFAULT
#define BOARD_HFCORECLK_FREQUENCY BOARD_SYSTEM_FREQUENCY

/* HFPERCLK - High Frequency Peripheral Clock
 *
 * Like HFCORECLK, HFPERCLK can also be a prescaled version of HFCLK. This
 * clock drives the High-Frequency Peripherals. The frequency of HFPERCLK is
 * set using the CMU_HFPERCLKDIV register.
 */

#define BOARD_HFPERCLKDIV        _CMU_HFPERCLKDIV_HFPERCLKDIV_DEFAULT
#define BOARD_HFPERCLK_FREQUENCY BOARD_SYSTEM_FREQUENCY

/* LFACLK - Low Frequency A Clock
 *
 * LFACLK is the selected clock for the Low Energy A Peripherals. There are
 * four selectable sources for LFACLK: LFRCO, LFXO, HFCORECLK/2 and ULFRCO.
 * From reset, the LFACLK source is set to LFRCO. However, note that the
 * LFRCO is disabled from reset. The selection is configured using the LFA
 * field in CMU_LFCLKSEL. The HFCORECLK/2 setting allows the Low Energy A
 * Peripherals to be used as high-frequency peripherals.
 *
 * Use _CMU_LFCLKSEL_LFA_DISABLED to disable
 * ULFRCO is a special case.
 */

#if BOARD_HAVE_LFXO
#   define BOARD_LFACLKSEL           _CMU_LFCLKSEL_LFA_LFXO
#   undef  BOARD_LFACLK_ULFRCO
#   define BOARD_LFACLK_FREQUENCY    BOARD_LFXO_FREQUENCY
#else
#   define BOARD_LFACLKSEL           _CMU_LFCLKSEL_LFA_LFRCO
#   undef  BOARD_LFACLK_ULFRCO
#   define BOARD_LFACLK_FREQUENCY    BOARD_LFRCO_FREQUENCY
#endif

/* LFBCLK - Low Frequency B Clock
 *
 * LFBCLK is the selected clock for the Low Energy B Peripherals. There are
 * four selectable sources for LFBCLK: LFRCO, LFXO, HFCORECLK/2 and ULFRCO.
 * From reset, the LFBCLK source is set to LFRCO. However, note that the
 * LFRCO is disabled from reset. The selection is configured using the LFB
 * field in CMU_LFCLKSEL. The HFCORECLK/2 setting allows the Low Energy B
 * Peripherals to be used as high-frequency peripherals.
 *
 * Use _CMU_LFCLKSEL_LFB_DISABLED to disable
 * ULFRCO is a special case.
 */

#if BOARD_HAVE_LFXO
#   define BOARD_LFBCLKSEL           _CMU_LFCLKSEL_LFB_LFXO
#   undef  BOARD_LFBCLK_ULFRCO
#   define BOARD_LFBCLK_FREQUENCY    BOARD_LFXO_FREQUENCY
#else
#   define BOARD_LFBCLKSEL           _CMU_LFCLKSEL_LFB_LFRCO
#   undef  BOARD_LFBCLK_ULFRCO
#   define BOARD_LFBCLK_FREQUENCY    BOARD_LFRCO_FREQUENCY
#endif


/* BURTC Clock source 
 * select clock source from following value:
 *  - BURTC_CTRL_CLKSEL_LFRCO 
 *  - BURTC_CTRL_CLKSEL_LFXO  
 *  - BURTC_CTRL_CLKSEL_ULFRCO 
 */
#define BOARD_BURTC_CLKSRC  BURTC_CTRL_CLKSEL_LFXO

/* BURTC Prescaler 
 * select Prescaler from following value:
 *  - BURTC_CTRL_PRESC_DIV1
 *  - BURTC_CTRL_PRESC_DIV2
 *  - BURTC_CTRL_PRESC_DIV4
 *  - BURTC_CTRL_PRESC_DIV8
 *  - BURTC_CTRL_PRESC_DIV16
 *  - BURTC_CTRL_PRESC_DIV32
 *  - BURTC_CTRL_PRESC_DIV64
 *  - BURTC_CTRL_PRESC_DIV128
 */
#define BOARD_BURTC_PRESC   BURTC_CTRL_PRESC_DIV1

/* BURTC Mode 
 * select enable mode from following value:
 *  - BURTC_CTRL_MODE_EM2EN
 *  - BURTC_CTRL_MODE_EM3EN
 *  - BURTC_CTRL_MODE_EM4EN
 */
#define BOARD_BURTC_MODE    BURTC_CTRL_MODE_EM4EN


/* PCNTnCLK - Pulse Counter n Clock
 *
 * Each available pulse counter is driven by its own clock, PCNTnCLK where
 * n is the pulse counter instance number. Each pulse counter can be
 * configured to use an external pin (PCNTn_S0) or LFACLK as PCNTnCLK.
 */

/* WDOGCLK - Watchdog Timer Clock
 *
 * The Watchdog Timer (WDOG) can be configured to use one of three different
 * clock sources: LFRCO, LFXO or ULFRCO. ULFRCO (Ultra Low Frequency RC
 * Oscillator) is a separate 1 kHz RC oscillator that also runs in EM3.
 */

/* AUXCLK - Auxiliary Clock
 *
 * AUXCLK is a 1-28 MHz clock driven by a separate RC oscillator, AUXHFRCO.
 * This clock is used for flash programming and Serial Wire Output (SWO).
 * During flash programming this clock will be active. If the AUXHFRCO has
 * not been enabled explicitly by software, the MSC will automatically
 * start and stop it. The AUXHFRCO is enabled by writing a 1 to AUXHFRCOEN
 * in CMU_OSCENCMD. This explicit enabling is required when SWO is used.
 */


/* SWO Location - Where SWO goes out.
 *
 * On some board there is possible to use many location for swo output.
 */
#define BOARD_SWOPORT_LOCATION  0

/* SWO Location - Where SWO goes out.
 *
 * On some board there is possible to different 2 pin in function of 
 * swo location output.
 */
#define BOARD_GPIO_SWOPORT   ( GPIO_OUTPUT_PUSHPULL | \
                               GPIO_PORTF           | \
                               GPIO_PIN2                )


/* LEDs *********************************************************************/
/* The EFM32 Gecko Starter Kit supports 4 yellow LEDs.  One side is grounded
 * so these LEDs are illuminated by outputting a high value.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with efm32_setled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2

#define BOARD_LED_SHIFT     BOARD_LED1
#define BOARD_LED_SHIFT2    BOARD_LED2

/* LED bits for use with efm32_setleds() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)



/* Pin routing **************************************************************/
/* USART0 is SPI:
 *
 *   U0_CLK  #2 PC9 
 *   U0_MISO #2 PC10  
 *   U0_MOSI #2 PC11 
 */

#define BOARD_USART0_CLK_GPIO       (GPIO_PORTC|GPIO_PIN9 )
#define BOARD_USART0_RX_GPIO        (GPIO_PORTC|GPIO_PIN10)
#define BOARD_USART0_TX_GPIO        (GPIO_PORTC|GPIO_PIN11)
#define BOARD_USART0_ROUTE_LOCATION _USART_ROUTE_LOCATION_LOC2

/* Pin routing **************************************************************/
/* UART1 for KLINE:
 *
 *   U1_RX #1 PD1  
 *   U1_TX #1 PD0  
 */

#define BOARD_USART1_RX_GPIO        (GPIO_PORTD|GPIO_PIN1)
#define BOARD_USART1_TX_GPIO        (GPIO_PORTD|GPIO_PIN0)
#define BOARD_USART1_ROUTE_LOCATION _USART_ROUTE_LOCATION_LOC1

/* Pin routing **************************************************************/
/* UART2 for GPS:
 *
 *   U2_RX #1 PC3  
 *   U2_TX #1 PC2  
 */

#define BOARD_USART2_RX_GPIO        (GPIO_PORTC|GPIO_PIN3)
#define BOARD_USART2_TX_GPIO        (GPIO_PORTC|GPIO_PIN2)
#define BOARD_USART2_ROUTE_LOCATION _USART_ROUTE_LOCATION_LOC0

/* Pin routing **************************************************************/
/* PWM on TIMER0 for backlight:
 *
 *   TIMER0 Channel 1 #4 PC0  
 */
#define BOARD_PWM_TIMER0_PINCFG     (GPIO_PORTC|GPIO_PIN0|GPIO_OUTPUT_PUSHPULL|GPIO_OUTPUT_SET)
#define BOARD_PWM_TIMER0_PINLOC     _TIMER_ROUTE_LOCATION_LOC4
#define BOARD_PWM_TIMER0_CLKIN      BOARD_SYSTEM_FREQUENCY


/* IC1:
 *
 * The pnbfano board one I2C. 
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * PC7                   For external spi (WIFI)
 * PC8                   for SDCARD
 * --------------------- ---------------------
 */

#define BOARD_I2C1_SDA   (GPIO_PORTC|GPIO_PIN4|GPIO_OUTPUT_WIREDAND|GPIO_OUTPUT_SET)
#define BOARD_I2C1_SCL   (GPIO_PORTC|GPIO_PIN5|GPIO_OUTPUT_WIREDAND|GPIO_OUTPUT_SET)
#define BOARD_I2C1_ROUTE_LOCATION _I2C_ROUTE_LOCATION_LOC0

/* VCMP:
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * VDD                   level 3.1 V 
 * --------------------- ---------------------
 */
#define BOARD_VCMP_HALFBIAS  1      /* Enable half bias          */
#define BOARD_VCMP_BIASPROG  8      /* Half bias 8 => 1µA        */
#define BOARD_VCMP_WARMUP    0      /* 4 cycle (quickest)        */
#define BOARD_VCMP_LEVEL    (3.1)   /* Set to 3V1               */

/* ACMP:
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * VBAT (PD6)             10V
 * --------------------- ---------------------
 */
//#define BOARD_ACMP_ENABLE
#define BOARD_ACMP_WARMUP    0 /* 4 cycle (quickest)*/
#define BOARD_ACMP_LEVEL    -1 /* TODO BOARD_ACMP_LEVEL     */
#define BOARD_ACMP_INPUT     0 /* CH0 */
#define BOARD_ACMP_LOCATION  0 /* Location 0 */


#define BOARD_SDHC_BLOCK_DEV_PATH   "/dev/mmcsd0"
#define BOARD_SDHC_MOUNT_PATH       "/mnt"


//#define BOARD_SPI0_CLK       (GPIO_PORTC|GPIO_PIN9 |GPIO_OUTPUT_PUSHPULL_DRIVE|GPIO_DRIVE_HIGH|GPIO_OUTPUT_SET)
#define BOARD_SPI0_CLK       (GPIO_PORTC|GPIO_PIN9 |GPIO_OUTPUT_PUSHPULL|GPIO_OUTPUT_SET)
#define BOARD_SPI0_MISO      (GPIO_PORTC|GPIO_PIN10|GPIO_INPUT_PULLUP)
#define BOARD_SPI0_MOSI      (GPIO_PORTC|GPIO_PIN11|GPIO_OUTPUT_PUSHPULL|GPIO_OUTPUT_SET)


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  efm32_ledinit, efm32_setled, and efm32_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then the following interfaces
 *   are available to control the LEDs from user applications.
 *
 ****************************************************************************/

int board_format_sdcard(void);
int board_mount_sdcard(void);
int board_umount_sdcard(void);
int board_is_usb_connected(void);
int board_is_usb_enabled(void);
int board_enable_usbmsc(void);
int board_disable_usbmsc(void);

#ifndef CONFIG_ARCH_LEDS
void efm32_ledinit(void);
void efm32_setled(int led, bool ledon);
void efm32_setleds(uint8_t ledset);
#endif

#endif /* __CONFIGS_EFM32_PNBFANO_INCLUDE_BOARD_H */
