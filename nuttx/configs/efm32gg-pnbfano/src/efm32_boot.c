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
#include <nuttx/arch.h>

#include <arch/board/board.h>

#include <sys/mount.h>

#include "efm32_start.h"
#include "efm32_pwm.h"
#include "efm32gg-pnbfano.h"

#include <nuttx/sensors/inv_mpu.h>
#include <nuttx/input/keypad.h>
#include <nuttx/pwm.h>
#include <nuttx/board.h>
#include <nuttx/nx/nx.h>
#include <syslog.h>

#include <nuttx/fs/mkfatfs.h>

#include <time.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <string.h>
#include <errno.h>

#include "efm32_rmu.h"

#define LEN(x)              (sizeof(x)/sizeof(*(x)))

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

    efm32_rmu_initialize();

    efm32_emu_initialize();

    efm32_vcmp_initialize();
              
#ifdef BOARD_ACMP_ENABLE
    efm32_acmp_initialize();
#endif

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

#ifdef CONFIG_SCHED_INSTRUMENTATION

void   sched_note_start(FAR struct tcb_s *tcb)
{
    up_putc('B');
}
void   sched_note_stop(FAR struct tcb_s *tcb)
{
    up_putc('E');
}
void   sched_note_switch(FAR struct tcb_s *pFromTcb,
                         FAR struct tcb_s *pToTcb)
{
    up_putc('S');
}

#endif /* CONFIG_SCHED_INSTRUMENTATION */

/****************************************************************************
 * Name: board_init_pwm
 *
 * Description:
 *  It return NX_DRIVER of device devno,vplaneno.
 *
 ****************************************************************************/
int board_init_pwm(void)
{
    int ret = 0;
    struct pwm_lowerhalf_s *pwm;    

    pwm = efm32_pwminitialize(0);

    if (!pwm)
    {
        syslog(LOG_ERR,"Failed to get the EFM32 PWM lower half\n");
        return -ENODEV;
    }

    /* Register the PWM driver at "/dev/pwm0" */

    ret = pwm_register("/dev/pwm0", pwm);
    if (ret < 0)
    {
        syslog(LOG_ERR,"pwm_register failed: %d\n", ret);
        return ret;
    }

    /* Now we are initialized */
    return 0;
}

int efm32_initialize_mpu(int devno)
{
    struct mpu_low_s * low;
    struct i2c_dev_s * i2c;
    struct mpu_inst_s* mpu_inst;

    /* i2c port 1 */

    i2c = up_i2cinitialize(1);

    if ( i2c == NULL )
    {
        syslog(LOG_ERR,"Cannot initialize I2C !\n");
        return -1;
    }

    low = mpu_low_i2c_init(devno, 0xD0>>1, 0xC0>>1, i2c );
    if ( low == NULL )
    {
        syslog(LOG_ERR,"Cannot initialize mpu_low !\n");
        return -1;
    }

    mpu_inst = mpu_instantiate(low);
    if ( mpu_inst == NULL ) 
    {
        syslog(LOG_ERR,"Cannot initialize mpu instance !\n");
        return -1;
    }

    if ( mpu_fileops_init(mpu_inst, "/dev/invmpu0", 0) < 0 )
    {
        syslog(LOG_ERR,"Cannot register mpu device !\n");
        return -1;
    }

    return OK;
}

int board_mount_sdcard(void)
{
    syslog(LOG_NOTICE,"Mount SDCARD.\n");
    if ( mount(BOARD_SDHC_BLOCK_DEV_PATH,BOARD_SDHC_MOUNT_PATH,"vfat",0,NULL) 
         < 0 )
    {
        syslog(LOG_ERR,"Cannot Mount SDcard\n");
        return -1;
    }
    return 0;
}

int board_umount_sdcard(void)
{
    syslog(LOG_NOTICE,"Umount SDCARD.\n");
    if ( umount(BOARD_SDHC_MOUNT_PATH) < 0 )
    {
        syslog(LOG_ERR,"umount failed!\n");
        return -1;
    }
    return 0;
}

int board_is_usb_connected(void)
{
    int ret;
    ret = efm32_usbdev_is_connected();
    if ( ret < 0 )
    {
        syslog(LOG_ERR,"Cannot check is usb is connected!\n");
        return -1;
    }
    return ret;
}

int board_is_usb_enabled(void)
{
    int ret;
    ret = efm32_usbdev_is_enable();
    if ( ret < 0 )
    {
        syslog(LOG_ERR,"Cannot check is usb is enabled!\n");
        return -1;
    }
    return ret;
}

int board_enable_usbmsc(void)
{
    syslog(LOG_NOTICE,"Enable USBMSC.\n");
    if (  efm32_usbdev_enable_usbmsc() < 0 )
    {
        syslog(LOG_ERR,"Enable USBMSC failed!\n");
        return -1;
    }
    return 0;
}

int board_disable_usbmsc(void)
{
    syslog(LOG_NOTICE,"Disable USBMSC.\n");
    if (  efm32_usbdev_disable_usbmsc() < 0 )
    {
        syslog(LOG_ERR,"Disable USBMSC failed!\n");
        return -1;
    }
    return 0;
}


int board_format_sdcard(void)
{
    struct fat_format_s fmt = FAT_FORMAT_INITIALIZER;
    syslog(LOG_NOTICE,"Format.\n");
    fmt.ff_fattype = 32;
    strncpy((char*)fmt.ff_volumelabel,"PnbFano",LEN(fmt.ff_volumelabel)); /* Volume label */
    if ( mkfatfs(BOARD_SDHC_BLOCK_DEV_PATH,&fmt) < 0 )
    {
        syslog(LOG_ERR,"format failed!\n");
        return -1;
    }
    return 0;
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

    syslog(LOG_NOTICE,"initialize PWM \n");
	if ( board_init_pwm() < 0 )
    {
        syslog(LOG_ERR,"Cannot initialize PWMs\n");
    }

    syslog(LOG_NOTICE,"initialize CHRONO \n");
	if ( efm32_gpio_chrono_init() < 0 )
    {
        syslog(LOG_ERR,"Cannot initialize Chrono\n");
    }

    syslog(LOG_NOTICE,"initialize PPS \n");
	if ( efm32_gpio_pps_init() < 0 )
    {
        syslog(LOG_ERR,"Cannot initialize PPS\n");
    }

    syslog(LOG_NOTICE,"initialize LCD \n");
    if ( board_lcd_initialize() != OK )
    {
        syslog(LOG_ERR,"Cannot initialize LCD \n");
    }

#if defined(CONFIG_PNBFANO_GPIO_KEYPAD) || \
    defined(CONFIG_PNBFANO_LCD_KEYPAD ) 
    keypad_kbdinit();
#endif

    /* initialise MPU9250 */

    syslog(LOG_NOTICE,"initialize MPU \n");
    if ( efm32_initialize_mpu(0) < 0 )
    {
        syslog(LOG_ERR,"Cannot initialize MPU\n");
    }

    /* Mount the SDIO-based MMC/SD block driver */

    syslog(LOG_NOTICE,"initialize all SPI.\n");
    if ( efm32_initialize_spi_devices() < 0 )
    {
        syslog(LOG_ERR,"Cannot initialize SDcard\n");
    }

    syslog(LOG_NOTICE,"Start slow polling \n");
    if ( efm32_slow_poll_init() < 0 )
    {
        syslog(LOG_ERR,"Cannot Start slow polling\n");
    }

    syslog(LOG_NOTICE,"initialize usbdev \n");
    if ( efm32_usbdev_init() < 0 )
    {
        syslog(LOG_ERR,"Cannot initialize usbdev\n");
    }

    board_mount_sdcard();

    syslog(LOG_NOTICE,"Board Ready \n");

}
#endif
