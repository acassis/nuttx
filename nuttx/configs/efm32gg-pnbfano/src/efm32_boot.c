/*****************************************************************************
 * configs/efm32gg-pnbfano/src/efm32_boot.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>

#include "efm32_start.h"
#include "efm32gg-pnbfano.h"

#include <nuttx/input/keypad.h>
#include <syslog.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <string.h>
#include <errno.h>


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_boardinitialize
 *
 * Description:
 *   All EFM32 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization before any devices
 *   have been initialized.
 *
 ****************************************************************************/

void efm32_boardinitialize(void)
{

    /* test log Message */

	syslog(LOG_DEBUG,"EFM32 Board initialization.\n");

	syslog(LOG_DEBUG    ,   "LOG_DEBUG    \n");
	syslog(LOG_INFO     ,   "LOG_INFO     \n");
	syslog(LOG_NOTICE   ,   "LOG_NOTICE   \n");
	syslog(LOG_WARNING  ,   "LOG_WARNING  \n");
	syslog(LOG_ERR      ,   "LOG_ERR      \n");
	syslog(LOG_CRIT     ,   "LOG_CRIT     \n");
	syslog(LOG_ALERT    ,   "LOG_ALERT    \n");
	syslog(LOG_EMERG    ,   "LOG_EMERG    \n");

}

/****************************************************************************
 * Name: board_get_dev_vplane
 *
 * Description:
 *  It return NX_DRIVER of device devno,vplaneno.
 *
 ****************************************************************************/

NX_DRIVERTYPE board_get_nx_dev(int devno, int vplaneno)
{
    NX_DRIVERTYPE dev = NULL;

    dev = up_lcdgetdev(int devno);

    return dev;
}

/****************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
void board_initialize(void)
{

    syslog(LOG_NOTICE,"initialize LCD !\n");
    if ( up_lcdinitialize() != OK )
    {
        syslog(LOG_ERR,"Cannot initialize LCD !\n");
    }

#if defined(CONFIG_PNBFANO_GPIO_KEYPAD) || defined(CONFIG_PNBFANO_LCD_KEYPAD) 
    keypad_kbdinit();
#endif

  /* Mount the SDIO-based MMC/SD block driver */

    if ( efm32_initialize_spi_devices() < 0 )
    {
        syslog(LOG_ERR,"Cannot initialize SDcard\n");
        return;
    }

    if ( mount("/dev/mmcsd0","/mnt","vfat",0,NULL) < 0 )
    {
        syslog(LOG_ERR,"Cannot Mount SDcard\n");
        return;
    }

    syslog(LOG_NOTICE,"Board Ready !\n");

}
#endif
