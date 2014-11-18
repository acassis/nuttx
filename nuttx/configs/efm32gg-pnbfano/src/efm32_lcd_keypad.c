/****************************************************************************
 * arch/arm/include/efm32/keypad/keypad.c
 * Driver for Stk3300 keypad hardware
 *
 *   Copyright (C) 2011 Stefan Richter. All rights reserved.
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

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include <string.h>
#include <errno.h>
#include <nuttx/kmalloc.h>

#include "up_arch.h"
#include "efm32_gpio.h"
#include "efm32_lcd.h"
#include "efm32gg-pnbfano.h"

#include <efm32_lcd_keypad.h>

//#define EFM32_LCD_KBD_LOG(...)
//#define EFM32_LCD_KBD_LOG(...) lldbg(__VA_ARGS__)
#define EFM32_LCD_KBD_LOG(...) syslog(LOG_NOTICE,__VA_ARGS__)

#ifndef CONFIG_EFM32_LCD_KBD_BUFSIZE
#  define CONFIG_EFM32_LCD_KBD_BUFSIZE 64
#endif

/****************************************************************************
 * Fileops Prototypes and Structures
 ****************************************************************************/

typedef FAR struct file file_t;

static int efm32_lcd_kbd_open(file_t * filep);
static int efm32_lcd_kbd_close(file_t * filep);
static ssize_t efm32_lcd_kbd_read(file_t * filep, FAR char *buffer, size_t buflen);
#ifndef CONFIG_DISABLE_POLL
/* TODO */
//static int efm32_lcd_kbd_poll(file_t * filep, FAR struct pollfd *fds, bool setup);
#endif

static const struct file_operations keypad_ops =
{
    efm32_lcd_kbd_open,  /* open */
    efm32_lcd_kbd_close, /* close */
    efm32_lcd_kbd_read,  /* read */
    NULL,                 /* write */
    NULL,                 /* seek */
    NULL,                 /* ioctl */
#ifndef CONFIG_DISABLE_POLL
    NULL,                 /* TODO: efm32_lcd_kbd_poll */ /* poll */
#endif
};


/****************************************************************************
 * Name: efm32_lcd_kbd_t
 * Description:
 *  variable of keypad
 ****************************************************************************/
typedef struct 
{
    sem_t   mutex;
    sem_t   rd_sem;
    int     rd_idx;
    int     wr_idx;
    uint8_t kbdbuffer[CONFIG_EFM32_LCD_KBD_BUFSIZE];
}efm32_lcd_kbd_t;

/****************************************************************************
 * Name: efm32_lcd_kbd
 * Description:
 *  variable of keypad instance.
 ****************************************************************************/
efm32_lcd_kbd_t* efm32_lcd_kbd;

/****************************************************************************
 * Name: key_mapping
 *  keep mapping of keyboard.
 ****************************************************************************/
static const int key_mapping[GPIO_LCD_PORT_BUS_WIDTH] = 
{
    GPIO_LCD_KEY_D0,
    GPIO_LCD_KEY_D1,
    GPIO_LCD_KEY_D2,
    GPIO_LCD_KEY_D3,
    GPIO_LCD_KEY_D4,
    GPIO_LCD_KEY_D5,
    GPIO_LCD_KEY_D6,
    GPIO_LCD_KEY_D7
};

/****************************************************************************
 * Name: efm32_lcd_kbd_open
 ****************************************************************************/
static void efm32_lcd_kbd_putc(FAR struct lib_outstream_s *this, int ch)
{
    if ( efm32_lcd_kbd == NULL )
    {
        EFM32_LCD_KBD_LOG("driver not initialized!\n");
        return; 
    }

    int level = efm32_lcd_kbd->wr_idx - efm32_lcd_kbd->rd_idx;

    if ( level < 0 )
        level += CONFIG_EFM32_LCD_KBD_BUFSIZE;

    if (level >= CONFIG_EFM32_LCD_KBD_BUFSIZE)
    {
        EFM32_LCD_KBD_LOG("Buffer overflow\n");
        return; 
    }

    efm32_lcd_kbd->kbdbuffer[efm32_lcd_kbd->wr_idx++] = ch;
    if ( efm32_lcd_kbd->wr_idx >= CONFIG_EFM32_LCD_KBD_BUFSIZE )
        efm32_lcd_kbd->wr_idx = 0;

    sem_post(&efm32_lcd_kbd->rd_sem);

    this->nput++;
}

/****************************************************************************
 * irq handler
 ****************************************************************************/
void efm32_lcd_idle(void)
{
    int res ;
    int i;
    static int last_res = 0;

    res = lcd_read_bus_keypad();

    if ( res == -1 )
    {
        EFM32_LCD_KBD_LOG("Cannot read keypad !\n");
        return;
    }

    /* something happen ? */

    if ( last_res == res )
    {
        return;
    }

    for (i = 0; i < GPIO_LCD_PORT_BUS_WIDTH; i++)
    {
        int mask = 1 << i;
        if ( ( key_mapping[i] != EFM32_LCD_KEY_NONE )
             && ( res & mask ) != ( last_res & mask ) 
           )
        {
            struct lib_outstream_s stream = {
                .put  = efm32_lcd_kbd_putc,
                .nput = 0
            };

            if ( res & mask )
            {
                if ( EFM32_LCD_KEY_IS_SPECIAL(key_mapping[i]) )
                {
                    kbd_specpress(EFM32_LCD_KEY_SPECIAL(key_mapping[i]), 
                                &stream
                               );
                }
                else
                {
                    kbd_press(key_mapping[i], &stream);
                }
            }
            else
            {
                if ( EFM32_LCD_KEY_IS_SPECIAL(key_mapping[i]) )
                {
                    kbd_specrel(EFM32_LCD_KEY_SPECIAL(key_mapping[i]), 
                                &stream
                               );
                }
                else
                {
                    kbd_release(key_mapping[i], &stream);
                }
            }
        }
    }

    return;
}



/****************************************************************************
 * Name: efm32_lcd_keypad_init
 *
 * Description:
 *  Initialize All GPIO for key pad.
 * Input parameters:
 *  _key_map    - first key mapping of mapping GPIO<=>Key list.
 *                    to Finish list set Pin with negative value.
 * Returned Value:
 *   None (User allocated instance initialized).
 ****************************************************************************/
void keypad_kbdinit( void )
{

    /* can be called only once */

    if ( key_mapping != NULL)
    {
        EFM32_LCD_KBD_LOG("Already initialized !\n");
        return;
    }

    efm32_lcd_kbd = (efm32_lcd_kbd_t*)kmm_malloc(sizeof(efm32_lcd_kbd_t));
    if ( efm32_lcd_kbd == NULL )
    {
        EFM32_LCD_KBD_LOG("Cannot allocate it!\n");
        return;
    }

    memset(efm32_lcd_kbd,0,sizeof(*efm32_lcd_kbd));

    sem_init(&efm32_lcd_kbd->rd_sem,    1, 0);
    sem_init(&efm32_lcd_kbd->mutex,     1, 1);

    register_driver("/dev/keypad", &keypad_ops, 0444, NULL);

}



/****************************************************************************
 * Name: efm32_lcd_kbd_open
 ****************************************************************************/

static int efm32_lcd_kbd_open(file_t * filep)
{
    if ( efm32_lcd_kbd == NULL )
    {
        EFM32_LCD_KBD_LOG("Not initialized!\n");
        return -EINVAL;
    }

    return OK;
}

/****************************************************************************
 * Name: efm32_lcd_kbd_close
 ****************************************************************************/

static int efm32_lcd_kbd_close(file_t * filep)
{

    /* nothing to do */

    return OK;
}

/****************************************************************************
 * Name: efm32_lcd_kbd_read
 ****************************************************************************/

static ssize_t efm32_lcd_kbd_read(file_t * filep, FAR char *buf, size_t buflen)
{
    ssize_t size = 0;

    if ( efm32_lcd_kbd == NULL )
    {
        EFM32_LCD_KBD_LOG("Not initialized!\n");
        return -EINVAL;
    }

    sem_wait( &efm32_lcd_kbd->mutex );

    while (size < buflen)
    {

        /* first lock it then only try lock */

        if ( size == 0 )
            sem_wait( &efm32_lcd_kbd->rd_sem);
        else if ( sem_trywait( &efm32_lcd_kbd->rd_sem) < 0 )
            break;

        irqstate_t saved_state;


        saved_state = irqsave();

        buf[size] = efm32_lcd_kbd->kbdbuffer[efm32_lcd_kbd->rd_idx++];
        if (efm32_lcd_kbd->rd_idx >= CONFIG_EFM32_LCD_KBD_BUFSIZE )
            efm32_lcd_kbd->rd_idx=0;

        irqrestore(saved_state);

        EFM32_LCD_KBD_LOG("Read %c,0x%2X\n",buf[size],buf[size]);

        size++; 
    }

    sem_post( &efm32_lcd_kbd->mutex );

    return size;
}

