/************************************************************************************
 * configs/efm32gg-pnbfano/src/efm32_spi.c
 *
 *   Copyright (C) 2009-2013 Bouteville Pierre-Noel. All rights reserved.
 *   Author: Bouteville Pierre-Noel <pnb990@gmail.com>
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

//#include <nuttx/spi/spi.h>
//#include <nuttx/mmcsd.h>
#include <arch/board/board.h>

#include <nuttx/arch.h>
#include "up_arch.h"
#include "chip.h"
#include "efm32gg-pnbfano.h"
#include "chip/efm32_gpio.h"
#include "efm32_gpio.h"
#include "nuttx/lcd/st7565.h"

/************************************************************************************
 * Constant Private Data
 ************************************************************************************/
#if defined(GPIO_LCD_PORT_PIN_NBR) 
static const uint32_t g_lcd_port_pins[GPIO_LCD_PORT_PIN_NBR] = 
{
    GPIO_LCD_D0,
    GPIO_LCD_D1,
    GPIO_LCD_D2,
    GPIO_LCD_D3,
    GPIO_LCD_D4,
    GPIO_LCD_D5,
    GPIO_LCD_D6,
    GPIO_LCD_D7
};
#elif (defined(GPIO_LCD_PORT) )
#   ifndef GPIO_LCD_PORT_SHIFT
#       error "Should declare also GPIO_LCD_PORT_SHIFT"
#   endif
#   define __LCD_OPTIMISATION
#else
#   error "neither GPIO_LCD_PORT neither GPIO_LCD_PORT_PIN_NBR"
#endif

/******************************************************************************
 * Private Data
 ******************************************************************************/

FAR struct lcd_dev_s *g_lcd = NULL;

/****************************************************************************
 * Fileops Prototypes and Structures
 ****************************************************************************/

static void st7565_reset     (FAR struct st7565_lcd_s *lcd, bool on);
static void st7565_select    (FAR struct st7565_lcd_s *lcd);
static void st7565_deselect  (FAR struct st7565_lcd_s *lcd);
static void st7565_cmddata   (FAR struct st7565_lcd_s *lcd, const uint8_t cmd);
static int  st7565_senddata  (FAR struct st7565_lcd_s *lcd, const uint8_t *data, int size);
static int  st7565_backlight (FAR struct st7565_lcd_s *lcd, int level);

struct st7565_lcd_s efm32_st7565_lcd =
{
#ifdef GPIO_LCD_RST
    .reset      = &st7565_reset,
#else
    .reset      = NULL,
#endif
    .select     = &st7565_select,
    .deselect   = &st7565_deselect, 
    .cmddata    = &st7565_cmddata,  
    .senddata   = &st7565_senddata, 
    .backlight  = &st7565_backlight
};


/**************************************************************************************
 * Name:  st7565_reset
 *
 * Description:
 *   Enable/Disable reset pinn of LCD the device.
 *
 *   lcd - A reference to the lcd interface specific structure
 *
 **************************************************************************************/

static void st7565_reset     (FAR struct st7565_lcd_s *lcd, bool on)
{
    UNUSED(lcd);
    efm32_gpiowrite(GPIO_LCD_RST,!on);
}

/**************************************************************************************
 * Name:  st7565_select
 *
 * Description:
 *   Select the device (as neccessary) before performing any operations.
 *
 *   lcd - A reference to the lcd interface specific structure
 *
 **************************************************************************************/

static void st7565_select    (FAR struct st7565_lcd_s *lcd)
{
    UNUSED(lcd);
    efm32_gpiowrite(GPIO_LCD_CS,false);
}

/**************************************************************************************
 * Name:  st7565_deselect
 *
 * Description:
 *   Deselect the device (as necessary).
 *
 *   lcd - A reference to the lcd interface specific structure
 *
 **************************************************************************************/

static void st7565_deselect  (FAR struct st7565_lcd_s *lcd)
{
    UNUSED(lcd);
    efm32_gpiowrite(GPIO_LCD_CS,true);
}

/**************************************************************************************
 * Name:  st7565_cmddata
 *
 * Description:
 *   Select command (A0 = 0) or data (A0 = 1) mode.
 *
 *   lcd - A reference to the lcd interface specific structure
 *   cmd    - If true command mode will be seleted.
 *
 **************************************************************************************/

static void st7565_cmddata   (FAR struct st7565_lcd_s *lcd, const uint8_t cmd)
{
    UNUSED(lcd);
    efm32_gpiowrite(GPIO_LCD_A0,!cmd);
}

/**************************************************************************************
 * Name:  st7565_send_data_buf
 *
 * Description:
 *   Send a data buffer to the LCD driver (A0 = 1).
 *
 *   lcd - A reference to the lcd interface specific structure
 *   buf    - buffer sent as data to LCD driver.
 *   size   - size of buffer in bytes.
 *
 **************************************************************************************/

static int  st7565_senddata  (FAR struct st7565_lcd_s *lcd, const uint8_t *data, int size)
{
#ifdef __LCD_OPTIMISATION
    int base   = EFM32_GPIO_Pn_BASE(GPIO_LCD_PORT>>GPIO_PORT_SHIFT);
#endif

    UNUSED(lcd);

    while(size--)
    {
#ifdef __LCD_OPTIMISATION
        /* optimisation */
        int regval = *data << GPIO_LCD_PORT_SHIFT;

        putreg32(regval, base + EFM32_GPIO_Pn_DOUTSET_OFFSET);
        regval ^= _GPIO_P_DOUT_MASK;
        putreg32(regval, base + EFM32_GPIO_Pn_DOUTCLR_OFFSET);
#else
        for (i = 0; i < GPIO_LCD_PORT_PIN_NBR; i++)
        {
            efm32_gpiowrite(g_lcd_port_pins[i],*data & (1<<i));
        }
#endif

        efm32_gpiowrite(GPIO_LCD_WR,false);
        data++;
        efm32_gpiowrite(GPIO_LCD_WR,true);

    }


    return OK;

}

/**************************************************************************************
 * Name:  st7565_backlight
 *
 * Description:
 *   Change backlight level of display.
 *
 *   lcd - A reference to the lcd interface specific structure
 *   level  - set backlight pwm from 0 CONFIG_LCD_MAXPOWER-1.
 *
 **************************************************************************************/

static int  st7565_backlight (FAR struct st7565_lcd_s *lcd, int level)
{
    /* TODO */
    UNUSED(lcd);
    UNUSED(level);

    return OK;

}




 /************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name:  up_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off).
 *
 ************************************************************************************/

int up_lcdinitialize(void)
{

  if ( g_lcd != NULL )
  {
      ldbg("Already initialized\n");
      return OK;
  }

  ldbg("Initializing\n");

  /* Configure GPIO pins.  The initial state of priv->output is false, so
   * we need to configure pins for output initially.
   */

#ifdef GPIO_LCD_RST
  efm32_configgpio(GPIO_LCD_RST);
#endif

  efm32_configgpio(GPIO_LCD_CS );
  efm32_configgpio(GPIO_LCD_A0 );
  efm32_configgpio(GPIO_LCD_WR );
  efm32_configgpio(GPIO_LCD_RD );

  efm32_configgpio(GPIO_LCD_D0 );
  efm32_configgpio(GPIO_LCD_D1 );
  efm32_configgpio(GPIO_LCD_D2 );
  efm32_configgpio(GPIO_LCD_D3 );
  efm32_configgpio(GPIO_LCD_D4 );
  efm32_configgpio(GPIO_LCD_D5 );
  efm32_configgpio(GPIO_LCD_D6 );
  efm32_configgpio(GPIO_LCD_D7 );

  /* Configure and enable LCD */

  up_mdelay(50);

  g_lcd = st7565_initialize(&efm32_st7565_lcd, 0);
  if ( g_lcd == NULL )
  {
      ldbg("Initializing\n");
      return -ENODEV;
  }

  return OK;
}


/************************************************************************************
 * Name:  up_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off).
 *
 ************************************************************************************/

FAR struct lcd_dev_s *up_lcdgetdev(int lcddev)
{
    if ( lcddev != 0 )
        return NULL;
    return g_lcd;
}



