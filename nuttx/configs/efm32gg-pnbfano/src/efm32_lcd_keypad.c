/****************************************************************************
 * configs/efm32gg-pnbfano/src/efm32_slow_polling.c
 *
 *   Copyright (C) 2014 Pierre-noel Bouteville. All rights reserved.
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
#include <poll.h>
#include <errno.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include "up_arch.h"
#include "efm32_gpio.h"
#include "efm32_lcd.h"
#include "efm32gg-pnbfano.h"

#include <efm32_lcd_keypad.h>

#define EFM32_LCD_KBD_LOG(...)
//#define EFM32_LCD_KBD_LOG(...) lldbg(__VA_ARGS__)
//#define EFM32_LCD_KBD_LOG(...) syslog(LOG_NOTICE,__VA_ARGS__)

#ifndef CONFIG_EFM32_LCD_KBD_BUFSIZE
#  define CONFIG_EFM32_LCD_KBD_BUFSIZE 64
#endif

#ifndef CONFIG_LCD_KBD_POLL_MS
#   define CONFIG_LCD_KBD_POLL_MS 100
#endif

/****************************************************************************
 * Fileops Prototypes and Structures
 ****************************************************************************/

typedef FAR struct file file_t;

typedef struct 
{
    sem_t   mutex;
    sem_t   rd_sem;
    int     rd_idx;
    int     wr_idx;
    uint8_t kbdbuffer[CONFIG_EFM32_LCD_KBD_BUFSIZE];

    sem_t   *poll_sem;
    struct work_s work;
}efm32_lcd_kbd_t;

struct efm32_lcd_kbd_stream 
{
    struct lib_outstream_s  stream;
    efm32_lcd_kbd_t         *dev;
};


static int efm32_lcd_kbd_open(file_t * filep);
static int efm32_lcd_kbd_close(file_t * filep);
static ssize_t efm32_lcd_kbd_read(file_t * filep, FAR char *buffer, size_t buflen);
#ifndef CONFIG_DISABLE_POLL
static int efm32_lcd_kbd_poll(file_t * filep, FAR struct pollfd *fds, bool setup);
#endif

static const struct file_operations keypad_ops =
{
    efm32_lcd_kbd_open,  /* open    */
    efm32_lcd_kbd_close, /* close   */
    efm32_lcd_kbd_read,  /* read    */
    NULL,                /* write   */
    NULL,                /* seek    */
    NULL,                /* ioctl   */
#ifndef CONFIG_DISABLE_POLL
    efm32_lcd_kbd_poll,  /* poll    */
#endif
};

/************************************************************************************
 * Name: efm32_lcd_kbd_takesem
 ************************************************************************************/

static int efm32_lcd_kbd_takesem(FAR sem_t *sem, bool errout)
{
  /* Loop, ignoring interrupts, until we have successfully acquired the semaphore */

  while (sem_wait(sem) != OK)
    {
      /* The only case that an error should occur here is if the wait was awakened
       * by a signal.
       */

      ASSERT(get_errno() == EINTR);

      /* When the signal is received, should we errout? Or should we just continue
       * waiting until we have the semaphore?
       */

      if (errout)
        {
          return -EINTR;
        }
    }

  return OK;
}

/************************************************************************************
 * Name: uart_givesem
 ************************************************************************************/

#define efm32_lcd_kbd_givesem(sem) (void)sem_post(sem)



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

static void efm32_lcd_kbd_worker(FAR void *arg);

/****************************************************************************
 * Name: efm32_lcd_kbd_level
 ****************************************************************************/

int efm32_lcd_kbd_level(FAR efm32_lcd_kbd_t *dev)
{
    int level = dev->wr_idx - dev->rd_idx;

    if ( level < 0 )
        level += CONFIG_EFM32_LCD_KBD_BUFSIZE;

    return level;
}

/****************************************************************************
 * Name: efm32_lcd_kbd_open
 ****************************************************************************/

static void efm32_lcd_kbd_putc(FAR struct lib_outstream_s *this, int ch)
{
    struct efm32_lcd_kbd_stream *p = (struct efm32_lcd_kbd_stream*)this;
    efm32_lcd_kbd_t *dev = p->dev;

    if ( dev == NULL )
    {
        EFM32_LCD_KBD_LOG("driver not initialized!\n");
        return; 
    }

    sem_wait(&dev->mutex);
    if ( efm32_lcd_kbd_level(dev) < CONFIG_EFM32_LCD_KBD_BUFSIZE )
    {
        dev->kbdbuffer[dev->wr_idx++] = ch;
        if ( dev->wr_idx >= CONFIG_EFM32_LCD_KBD_BUFSIZE )
            dev->wr_idx = 0;

        sem_post(&dev->rd_sem);
        this->nput++;

        /* add event to waiting semaphore */
        if ( dev->poll_sem )
        {
            sem_post( dev->poll_sem );
        }
    }
    else
    {
        EFM32_LCD_KBD_LOG("Buffer overflow\n");
    }
    sem_post(&dev->mutex);
}

/****************************************************************************
 * Name: efm32_lcd_kbd_open
 ****************************************************************************/
static void efm32_lcd_kbd_set_next_poll(efm32_lcd_kbd_t* dev)
{
    if ( work_queue(HPWORK, 
                    &(dev->work), 
                    efm32_lcd_kbd_worker,
                    dev, 
                    MSEC2TICK(CONFIG_LCD_KBD_POLL_MS)
                    ) != OK )
    {
        EFM32_LCD_KBD_LOG("Cannot register work !\n");
        return;
    }
}

/****************************************************************************
 * irq handler
 ****************************************************************************/
static void efm32_lcd_kbd_worker(FAR void *arg)
{
    int res ;
    int i;
    static int last_res = 0;
    efm32_lcd_kbd_t *dev = (efm32_lcd_kbd_t*)arg;

    res = lcd_read_bus_keypad();

    if ( res == -1 )
    {
        EFM32_LCD_KBD_LOG("Cannot read keypad !\n");
    }
    else if ( last_res != res )
    {
        for (i = 0; i < GPIO_LCD_PORT_BUS_WIDTH; i++)
        {
            int mask = 1 << i;
            if ( ( key_mapping[i] != EFM32_LCD_KEY_NONE )
                 && ( res & mask ) != ( last_res & mask ) 
               )
            {
                struct efm32_lcd_kbd_stream stream = {
                    .stream = { .put  = efm32_lcd_kbd_putc, .nput = 0 },
                    .dev    = dev
                };

                if ( res & mask )
                {
                    if ( EFM32_LCD_KEY_IS_SPECIAL(key_mapping[i]) )
                    {
                        kbd_specpress(EFM32_LCD_KEY_SPECIAL(key_mapping[i]), 
                                      (struct lib_outstream_s*)&stream
                                     );
                    }
                    else
                    {
                        kbd_press(key_mapping[i], 
                                  (struct lib_outstream_s*)&stream
                                 );
                    }
                }
                else
                {
                    if ( EFM32_LCD_KEY_IS_SPECIAL(key_mapping[i]) )
                    {
                        kbd_specrel(EFM32_LCD_KEY_SPECIAL(key_mapping[i]), 
                                    (struct lib_outstream_s*)&stream
                                   );
                    }
                    else
                    {
                        kbd_release(key_mapping[i], 
                                    (struct lib_outstream_s*)&stream
                                   );
                    }
                }
            }
        }
    }
    last_res = res;

    efm32_lcd_kbd_set_next_poll(dev);

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
int keypad_kbdinit( void )
{
    efm32_lcd_kbd_t * dev = NULL;

    /* can be called only once */

    if ( dev != NULL)
    {
        EFM32_LCD_KBD_LOG("Already initialized !\n");
        return OK;
    }


    dev = (efm32_lcd_kbd_t*)kmm_malloc(sizeof(efm32_lcd_kbd_t));
    if ( dev == NULL )
    {
        EFM32_LCD_KBD_LOG("Cannot allocate it!\n");
        return -ENODEV;
    }

    memset(dev,0,sizeof(*dev));
    //dev->poll_sem = NULL; already done */
    sem_init(&dev->rd_sem,    1, 0);
    sem_init(&dev->mutex,     1, 1);

    efm32_lcd_kbd_set_next_poll(dev);

    register_driver("/dev/keypad", &keypad_ops, 0444, dev);

    return OK;
}



/****************************************************************************
 * Name: efm32_lcd_kbd_open
 ****************************************************************************/

static int efm32_lcd_kbd_open(file_t * filep)
{
    FAR struct inode *inode     = filep->f_inode;
    FAR efm32_lcd_kbd_t *dev    = inode->i_private;

    if ( dev == NULL )
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
 * Name: efm32_lcd_kbd_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int efm32_lcd_kbd_poll(file_t * filep, FAR struct pollfd *fds, bool setup)
{

    FAR struct inode *inode     = filep->f_inode;
    FAR efm32_lcd_kbd_t *dev    = inode->i_private;

    int res = 0;

    /* Are we setting up the poll?  Or tearing it down? */

    res = efm32_lcd_kbd_takesem(&dev->mutex, true);

    if (res < 0)
    {
        /* A signal received while waiting for access to the poll data
         * will abort the operation.
         */

        return res;
    }

    if (setup)
    {
        fds->revents = 0;
        /* This is a request to set up the poll.  Find an available
         * slot for the poll structure reference
         */

        if ( dev->poll_sem != NULL)
        {
            res = -EINVAL;
            goto errout;
        }

        if ( efm32_lcd_kbd_level(dev) > 0 )
        {
            fds->revents |= (fds->events & POLLIN);
        }

        if ( fds->revents == 0 )
        {
            dev->poll_sem = fds->sem ;
        }
        else
        {
            sem_post(fds->sem);
            res = 1;
        }
    }
    else if ( dev->poll_sem == fds->sem )
    {
        dev->poll_sem = NULL;
    }
errout:
    efm32_lcd_kbd_givesem(&dev->mutex);
    return res;
}
#endif

/****************************************************************************
 * Name: efm32_lcd_kbd_read
 ****************************************************************************/

static ssize_t efm32_lcd_kbd_read(file_t * filep, FAR char *buf, size_t buflen)
{
    ssize_t size = 0;

    FAR struct inode *inode     = filep->f_inode;
    FAR efm32_lcd_kbd_t *dev    = inode->i_private;

    if ( dev == NULL )
    {
        EFM32_LCD_KBD_LOG("Not initialized!\n");
        return -EINVAL;
    }

    sem_wait( &dev->mutex );

    while (size < buflen)
    {

        /* first lock it then only try lock */

        if ( size == 0 )
            sem_wait( &dev->rd_sem);
        else if ( sem_trywait( &dev->rd_sem) < 0 )
            break;

        irqstate_t saved_state;


        saved_state = irqsave();

        buf[size] = dev->kbdbuffer[dev->rd_idx++];
        if (dev->rd_idx >= CONFIG_EFM32_LCD_KBD_BUFSIZE )
            dev->rd_idx=0;

        irqrestore(saved_state);

        EFM32_LCD_KBD_LOG("Read %c,0x%2X\n",buf[size],buf[size]);

        size++; 
    }

    sem_post( &dev->mutex );

    return size;
}

