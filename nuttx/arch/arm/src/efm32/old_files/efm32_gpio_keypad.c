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

#include <nuttx/arch.h>

#include <efm32_gpio_keypad.h>

#include <nuttx/kmalloc.h>
#include <string.h>
#include <errno.h>

#define EFM32_GPIO_KBD_LOG(...)
//#define EFM32_GPIO_KBD_LOG(...) lldbg(__VA_ARGS__)

#ifndef CONFIG_EFM32_GPIO_KBD_BUFSIZE
#  define CONFIG_EFM32_GPIO_KBD_BUFSIZE 64
#endif

/****************************************************************************
 * Fileops Prototypes and Structures
 ****************************************************************************/

typedef FAR struct file file_t;

static int efm32_gpio_kbd_open(file_t * filep);
static int efm32_gpio_kbd_close(file_t * filep);
static ssize_t efm32_gpio_kbd_read(file_t * filep, FAR char *buffer, size_t buflen);
#ifndef CONFIG_DISABLE_POLL
/* TODO */
//static int efm32_gpio_kbd_poll(file_t * filep, FAR struct pollfd *fds, bool setup);
#endif

static const struct file_operations keypad_ops =
{
    efm32_gpio_kbd_open,  /* open */
    efm32_gpio_kbd_close, /* close */
    efm32_gpio_kbd_read,  /* read */
    NULL,                 /* write */
    NULL,                 /* seek */
    NULL,                 /* ioctl */
#ifndef CONFIG_DISABLE_POLL
    NULL,                 /* TODO: efm32_gpio_kbd_poll */ /* poll */
#endif
};


/****************************************************************************
 * Name: efm32_gpio_kbd_t
 * Description:
 *  variable of keypad
 ****************************************************************************/
typedef struct 
{
    sem_t   mutex;
    sem_t   rd_sem;
    int     rd_idx;
    int     wr_idx;
    uint8_t kbdbuffer[CONFIG_EFM32_GPIO_KBD_BUFSIZE];
}efm32_gpio_kbd_t;

/****************************************************************************
 * Name: efm32_gpio_kbd
 * Description:
 *  variable of keypad instance.
 ****************************************************************************/
efm32_gpio_kbd_t* efm32_gpio_kbd;

/****************************************************************************
 * Name: key_mapping
 *  keep mapping of keyboard.
 ****************************************************************************/
static efm32_gpio_keypad_t* key_mapping = NULL;

/****************************************************************************
 * Name: efm32_gpio_kbd_open
 ****************************************************************************/
void efm32_gpio_kbd_putc(FAR struct lib_outstream_s *this, int ch)
{
    if ( efm32_gpio_kbd == NULL )
    {
        lldbg("driver not initialized!\n");
        return; 
    }

    int level = efm32_gpio_kbd->wr_idx - efm32_gpio_kbd->rd_idx;

    if ( level < 0 )
        level += CONFIG_EFM32_GPIO_KBD_BUFSIZE;

    if (level >= CONFIG_EFM32_GPIO_KBD_BUFSIZE)
    {
        lldbg("Buffer overflow\n");
        return; 
    }

    efm32_gpio_kbd->kbdbuffer[efm32_gpio_kbd->wr_idx++] = ch;
    if ( efm32_gpio_kbd->wr_idx >= CONFIG_EFM32_GPIO_KBD_BUFSIZE )
        efm32_gpio_kbd->wr_idx = 0;

    sem_post(&efm32_gpio_kbd->rd_sem);

    this->nput++;
}

/****************************************************************************
 * irq handler
 ****************************************************************************/
int efm32_gpio_kbd_irq(int pin, FAR void* context)
{
    (void)context;
    efm32_gpio_keypad_t* _key_map;

    DEBUGASSERT(key_mapping != NULL);

    _key_map = key_mapping;

    while ( _key_map->pin >= 0 )
    {
        if ( _key_map->pin == pin )
        {
            struct lib_outstream_s stream = {
                .put  = efm32_gpio_kbd_putc,
                .nput = 0
            };

            if ( GPIO_PinInGet(_key_map->port,_key_map->pin) == 0 )
            {
                EFM32_GPIO_KBD_LOG("PB on %c,%2d pressed\n",
                                   'A'+_key_map->port,
                                   _key_map->pin
                                  ); 

                if ( _key_map->special_key != KEYCODE_NORMAL )
                {
                    kbd_specpress(_key_map->special_key, &stream);
                }
                else
                {
                    kbd_press(_key_map->key, &stream);
                }
            }
            else
            {
                EFM32_GPIO_KBD_LOG("PB on %c,%2d Released\n",
                                   'A'+_key_map->port,
                                   _key_map->pin
                                  ); 
                if ( _key_map->special_key != KEYCODE_NORMAL )
                {
                    kbd_specrel(_key_map->special_key, &stream);
                }
                else
                {
                    kbd_release(_key_map->key, &stream);
                }
            }

        }
        _key_map++;
    }

    return 0;
}


/****************************************************************************
 * Name: efm32_gpio_keypad_init
 *
 * Description:
 *  Initialize All GPIO for key pad.
 * Input parameters:
 *  _key_map    - first key mapping of mapping GPIO<=>Key list.
 *                    to Finish list set Pin with negative value.
 * Returned Value:
 *   None (User allocated instance initialized).
 ****************************************************************************/
void efm32_gpio_keypad_init( efm32_gpio_keypad_t *_key_map,
                             const char *devname
                           )
{

    /* can be called only once */

    if ( key_mapping != NULL)
    {
        lldbg("Already initialized !\n");
        return;
    }

    efm32_gpio_kbd = (efm32_gpio_kbd_t*)kmm_malloc(sizeof(efm32_gpio_kbd_t));
    if ( efm32_gpio_kbd == NULL )
    {
        lldbg("Cannot allocate it!\n");
        return;
    }

    memset(efm32_gpio_kbd,0,sizeof(*efm32_gpio_kbd));

    sem_init(&efm32_gpio_kbd->rd_sem,    0, 0);
    sem_init(&efm32_gpio_kbd->mutex, 0, 1);

    key_mapping = _key_map;

    while ( _key_map->pin >= 0 )
    {
        irq_gpio_attach(_key_map->pin, efm32_gpio_kbd_irq);

        GPIO_PinModeSet(_key_map->port,
                        _key_map->pin,
                        gpioModeInputPullFilter,
                        1
                       );

        GPIO_IntConfig( _key_map->port,
                        _key_map->pin,
                        true,
                        true,
                        true
                      );

        _key_map++;
    }

    register_driver(devname, &keypad_ops, 0444, NULL);

}



/****************************************************************************
 * Name: efm32_gpio_kbd_open
 ****************************************************************************/

static int efm32_gpio_kbd_open(file_t * filep)
{
    if ( efm32_gpio_kbd == NULL )
    {
        lldbg("Not initialized!\n");
        return -EINVAL;
    }

    return OK;
}

/****************************************************************************
 * Name: efm32_gpio_kbd_close
 ****************************************************************************/

static int efm32_gpio_kbd_close(file_t * filep)
{

    /* nothing to do */

    return OK;
}

/****************************************************************************
 * Name: efm32_gpio_kbd_read
 ****************************************************************************/

static ssize_t efm32_gpio_kbd_read(file_t * filep, FAR char *buf, size_t buflen)
{
    ssize_t size = 0;

    if ( efm32_gpio_kbd == NULL )
    {
        lldbg("Not initialized!\n");
        return -EINVAL;
    }

    sem_wait( &efm32_gpio_kbd->mutex );

    while (size < buflen)
    {

        /* first lock it then only try lock */

        if ( size == 0 )
            sem_wait( &efm32_gpio_kbd->rd_sem);
        else if ( sem_trywait( &efm32_gpio_kbd->rd_sem) < 0 )
            break;

        irqstate_t saved_state;


        saved_state = irqsave();

        buf[size] = efm32_gpio_kbd->kbdbuffer[efm32_gpio_kbd->rd_idx++];
        if (efm32_gpio_kbd->rd_idx >= CONFIG_EFM32_GPIO_KBD_BUFSIZE )
            efm32_gpio_kbd->rd_idx=0;

        irqrestore(saved_state);

        EFM32_GPIO_KBD_LOG("Read %c,0x%2X\n",buf[size],buf[size]);

        size++; 
    }

    sem_post( &efm32_gpio_kbd->mutex );

    return size;
}

