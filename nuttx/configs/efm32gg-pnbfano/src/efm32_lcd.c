/************************************************************************************
 * configs/efm32gg-pnbfano/src/efm32_spi.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <semaphore.h>
#include <sys/ioctl.h>

//#include <nuttx/spi/spi.h>
//#include <nuttx/mmcsd.h>
#include <arch/board/board.h>

#include <nuttx/arch.h>
#include "up_arch.h"
#include "chip.h"
#include "efm32gg-pnbfano.h"
#include "chip/efm32_gpio.h"
#include "efm32_gpio.h"
#include "nuttx/pwm.h"
#include "nuttx/lcd/lcd.h"
#include "nuttx/lcd/st7565.h"

#ifdef GPIO_LCD_PWM_DEV
#include <nuttx/pwm.h>
#endif

/* TODO: put all bus 8080 access in separate file (driver) */

#ifndef CONFIG_SCHED_HPWORK
#       error "This module need CONFIG_SCHED_WORKQUEUE"
#endif


#ifndef GPIO_LCD_PORT_BUS_WIDTH
#       error "Should declare also GPIO_LCD_PORT_BUS_WIDTH (bus width)"
#endif

/************************************************************************************
 * Constant Private Data
 ************************************************************************************/
static const uint32_t g_lcd_port_pins[GPIO_LCD_PORT_BUS_WIDTH] = 
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

#if (defined(GPIO_LCD_PORT) )
#   ifndef GPIO_LCD_PORT_SHIFT
#       error "Should declare also GPIO_LCD_PORT_SHIFT if GPIO_LCD_PORT declared"
#   endif
#   ifndef GPIO_LCD_PORT_MASK
#       error "Should declare also GPIO_LCD_PORT_MASK if GPIO_LCD_PORT declared"
#   endif
#   define __LCD_OPTIMISATION
#else
#   error "neither GPIO_LCD_PORT neither GPIO_LCD_PORT_BUS_WIDTH"
#endif

/******************************************************************************
 * Private Data
 ******************************************************************************/

FAR struct lcd_dev_s *g_lcd = NULL;
#ifdef GPIO_LCD_PWM_DEV
static int lcd_pwm_dev = -1;
#endif


/****************************************************************************
 * Fileops Prototypes and Structures
 ****************************************************************************/

static void st7565_reset     (FAR struct st7565_lcd_s *lcd, bool on);
static void st7565_select    (FAR struct st7565_lcd_s *lcd);
static void st7565_deselect  (FAR struct st7565_lcd_s *lcd);
static void st7565_cmddata   (FAR struct st7565_lcd_s *lcd, const uint8_t cmd);
static int  st7565_senddata  (FAR struct st7565_lcd_s *lcd, const uint8_t *data, int size);
static int  st7565_backlight (FAR struct st7565_lcd_s *lcd, int level);

#if CONFIG_PNBFANO_LCD_KEYPAD
static sem_t st7565_lock_sem;
#endif

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
 * Name:  st7565_set_bus_output
 *
 * Description:
 *   set bus in output mode
 *
 **************************************************************************************/

static void st7565_set_bus_output(void)
{
    int i;

    /* TODO: May be optimized */

    for (i = 0; i < GPIO_LCD_PORT_BUS_WIDTH; i++)
    {
        int cfg = g_lcd_port_pins[i];
        cfg &= ~(GPIO_MODE_MASK | GPIO_MODE_DOUT_MASK);
        cfg |= GPIO_OUTPUT_PUSHPULL;
        efm32_configgpio(cfg);
    }
}

/**************************************************************************************
 * Name:  st7565_set_bus_output
 *
 * Description:
 *   set bus in output mode
 *
 **************************************************************************************/

#if CONFIG_PNBFANO_LCD_KEYPAD
static void st7565_set_bus_input_pullup(void)
{
    int i;

    /* TODO: May be optimized */

    for (i = 0; i < GPIO_LCD_PORT_BUS_WIDTH; i++)
    {
        int cfg = g_lcd_port_pins[i];
        cfg &= ~(GPIO_MODE_MASK | GPIO_MODE_DOUT_MASK);
        cfg |= GPIO_INPUT_PULLUP;
        efm32_configgpio(cfg);
    }
}
#endif

/**************************************************************************************
 * Name:  st7565_write_bus
 *
 * Description:
 *   restore configuration of shared gpio
 *
 **************************************************************************************/

static void st7565_write_bus( int data )
{

#ifdef __LCD_OPTIMISATION
    /* optimisation */
    int base   = EFM32_GPIO_Pn_BASE(GPIO_LCD_PORT>>GPIO_PORT_SHIFT);

    int regval = ( data << GPIO_LCD_PORT_SHIFT ) & GPIO_LCD_PORT_MASK ;

    putreg32(regval, base + EFM32_GPIO_Pn_DOUTSET_OFFSET);
    regval ^= _GPIO_P_DOUT_MASK;
    putreg32(regval, base + EFM32_GPIO_Pn_DOUTCLR_OFFSET);
#else
    int i;
    for (i = 0; i < GPIO_LCD_PORT_BUS_WIDTH; i++)
    {
        efm32_gpiowrite(g_lcd_port_pins[i],*data & (1<<i));
    }
#endif
}

/**************************************************************************************
 * Name:  st7565_read_bus
 *
 * Description:
 *   Return bus value
 *
 **************************************************************************************/

static int st7565_read_bus( void )
{
    int res = 0;
#ifdef __LCD_OPTIMISATION
    {
        /* optimisation */
        int base   = EFM32_GPIO_Pn_BASE(GPIO_LCD_PORT>>GPIO_PORT_SHIFT);

        res = ( getreg32( base + EFM32_GPIO_Pn_DIN_OFFSET) 
                & GPIO_LCD_PORT_MASK 
              ) >> GPIO_LCD_PORT_SHIFT;
    }
#else
    {
        int i;
        if ( efm32_gpioread(g_lcd_port_pins[i]) == 0 )
        {
            res |= 1 << i;
        }
    }
#endif
    return res;
}

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
 * Name:  st7565_lock
 *
 * Description:
 *   lock lcd access to leave D0-7 RD,WR,A0 free to allow access to key pad or other.
 *
 *   return OK if it well locked, -1 value otherwise.
 *
 **************************************************************************************/

#if CONFIG_PNBFANO_LCD_KEYPAD
int lcd_read_bus_keypad(void)
{
    int res = -1;

    if ( g_lcd != NULL )
    {

        ASSERT( sem_wait(&st7565_lock_sem) == OK );

        /* set all IO in input with pull-up */

        st7565_set_bus_input_pullup();

        /* don't need to wait code delay is enougth */

        res = st7565_read_bus() ^ (GPIO_LCD_PORT_MASK>>GPIO_LCD_PORT_SHIFT);

        /* restore all gpio */

        st7565_set_bus_output();

        ASSERT( sem_post(&st7565_lock_sem) == OK );

    }

    return res;
}
#endif

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
#if CONFIG_PNBFANO_LCD_KEYPAD
    ASSERT( sem_wait(&st7565_lock_sem) == OK );
#endif
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
#if CONFIG_PNBFANO_LCD_KEYPAD
    ASSERT( sem_post(&st7565_lock_sem) == OK );
#endif
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

    UNUSED(lcd);

    while(size--)
    {
        st7565_write_bus(*data);

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
    int ret = OK;
    UNUSED(lcd);

#ifdef GPIO_LCD_PWM_PIN
    efm32_gpiowrite(GPIO_LCD_PWM_PIN,level);
#endif

#ifdef GPIO_LCD_PWM_DEV
    if ( lcd_pwm_dev < 0 )
    {
        lcd_pwm_dev = open(GPIO_LCD_PWM_DEV, O_RDONLY);
        if (lcd_pwm_dev < 0)
        {
            ldbg("LCD pwm open failed errno: %d\n");
        }
    }
    if ( lcd_pwm_dev < 0 )
    {
        ret = -1;
    }
    else
    {
        struct pwm_info_s info;
        /* Configure the characteristics of the pulse train */

        /* Frequency of the pulse train */
        info.frequency = 100; /* 100 Hz */
        /* Duty of the pulse train, "1"-to-"0" duration.
         * Maximum: 65535/65536 (0x0000ffff)
         * Minimum:     1/65536 (0x00000001) */
        info.duty      = (((uint32_t)level)*65535) / 100;

#ifdef CONFIG_PWM_PULSECOUNT
        /* The number of pulse to generate.  0 means to
         * generate an indefinite number of pulses  */
        info.count     = 0;
#endif

        ret = ioctl(lcd_pwm_dev, 
                    PWMIOC_SETCHARACTERISTICS, 
                    (unsigned long)&info
                   );
        if (ret < 0)
        {
            ldbg("pwm_main: ioctl(PWMIOC_SETCHARACTERISTICS)"
                 " failed errno: %d\n",
                 errno
                );
        }

        /* Then start the pulse train.  Since the driver was opened in blocking
         * mode, this call will block if the count value is greater than zero.
         */

        if ( ret >= 0 )
        {
            ret = ioctl(lcd_pwm_dev, PWMIOC_START, 0);
            if (ret < 0)
            {
                ldbg("pwm_main: ioctl(PWMIOC_START) failed: %d\n", errno);
            }
        }
    }

#endif

    return ret;

}




 /************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off).
 *
 ************************************************************************************/

int board_lcd_initialize(void)
{

  if ( g_lcd != NULL )
  {
      ldbg("Already initialized\n");
      return OK;
  }

  ldbg("Initializing\n");

#if CONFIG_PNBFANO_LCD_KEYPAD
  sem_init(&st7565_lock_sem,1,1);
#endif

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

  st7565_set_bus_output();

#ifdef GPIO_LCD_PWM_PIN
  efm32_configgpio(GPIO_LCD_PWM_PIN);
#endif
#ifdef GPIO_LCD_PWM_DEV

    lcd_pwm_dev = open(GPIO_LCD_PWM_DEV, O_RDONLY);
    if (lcd_pwm_dev < 0)
    {
        ldbg("LCD pwm open failed errno: %d\n");
    }
#endif

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
 * Name:  board_lcd_getdev
 *
 * Description:
 *  Give lcd pointer instance
 *
 ************************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
    if ( lcddev != 0 )
        return NULL;
    return g_lcd;
}



