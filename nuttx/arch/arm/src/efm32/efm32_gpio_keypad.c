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

//#define EFM32_GPIO_KBD_LOG(...)
#define EFM32_GPIO_KBD_LOG(...) lldbg(__VA_ARGS__)

/****************************************************************************
 * Fileops Prototypes and Structures
 ****************************************************************************/

typedef FAR struct file file_t;

static int efm32_gpio_kbd_open(file_t * filep);
static int efm32_gpio_kbd_close(file_t * filep);
static ssize_t efm32_gpio_kbd_read(file_t * filep, FAR char *buffer, size_t buflen);
#ifndef CONFIG_DISABLE_POLL
static int efm32_gpio_kbd_read(file_t * filep, FAR struct pollfd *fds, bool setup);
#endif

static const struct file_operations keypad_ops =
{
    efm32_gpio_kbd_open,  /* open */
    efm32_gpio_kbd_close, /* close */
    efm32_gpio_kbd_read,  /* read */
    0,                    /* write */
    0,                    /* seek */
    0,                    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
    efm32_gpio_kbd_poll   /* poll */
#endif
};



/****************************************************************************
 * Name: key_mapping
 *  keep mapping of keyboard.
 ****************************************************************************/
static efm32_gpio_keypad_t* key_mapping = NULL;

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
            if ( GPIO_PinInGet(_key_map->port,_key_map->pin) == 0 )
            {
                EFM32_GPIO_KBD_LOG("PB on %c,%2d pressed\n",
                                   'A'+_key_map->port,
                                   _key_map->pin
                                  ); 
                //kbd_specpress(key);
            }
            else
            {
                EFM32_GPIO_KBD_LOG("PB on %c,%2d Released\n",
                                   'A'+_key_map->port,
                                   _key_map->pin
                                  ); 
                //kbd_specrel(key)
            }
        }
        _key_map++;
    }

    return 0;
}

/****************************************************************************
 * Name: efm32_gpio_kbd_open
 ****************************************************************************/

static int efm32_gpio_kbd_open(file_t * filep)
{

    /** todo */

    return OK;
}

/****************************************************************************
 * Name: efm32_gpio_kbd_close
 ****************************************************************************/

static int efm32_gpio_kbd_close(file_t * filep)
{

    /** todo */

    return OK;
}

/****************************************************************************
 * Name: efm32_gpio_kbd_read
 ****************************************************************************/

static ssize_t efm32_gpio_kbd_read(file_t * filep, FAR char *buf, size_t buflen)
{
    ssize_t size = -1;

    return size;
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
        lldbg("Already initialized !");
        return;
    }

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



