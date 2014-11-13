/****************************************************************************
 * include/nuttx/lcd/st7565.h
 *
 * Definitions for the ST7565 128x64 Dot Matrix LCD
 * Driver with C
 *
 *   Copyright (C) 2014 Pierre-noel Bouteville. All rights reserved.
 *   Author: Pierre-noel Boutevlle <pnb990@gmail.com>
 *
 * Based on drivers/lcd/st7567.h
 *
 *   Copyright (C) 2013 Zilogic Systems. All rights reserved.
 *   Author: Manikandan <code@zilogic.com>
 *
 * Based on include/nuttx/lcd/ug-9664hswag01.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NUTTX_ST7565_H
#define __INCLUDE_NUTTX_ST7565_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ST7565 Configuration Settings:
 *
 * CONFIG_ST7565_NINTERFACES - Specifies the number of physical
 *   ST7565 devices that will be supported.  NOTE:  At present, this
 *   must be undefined or defined to be 1.
 *
 * Required LCD driver settings:
 * CONFIG_LCD_ST7565 - Enable ST7565 support
 * CONFIG_LCD_MAXCONTRAST should be 255, but any value >0 and <=255 will be accepted.
 *
 */

/* Some important "colors" */

#define ST7565_Y1_BLACK  0
#define ST7565_Y1_WHITE  1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  st7565_initialize
 *
 * Description:
 *   Initialize the ST7565 video hardware.  The initial state of the
 *   LCD is fully initialized, display memory cleared, and the LCD ready
 *   to use, but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 throw CONFIG_ST7565_NINTERFACES-1.
 *     This allows support for multiple LCD devices.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified LCD.  NULL is returned on any failure.
 *
 ****************************************************************************/

struct lcd_dev_s;       /* see nuttx/lcd.h */
struct st7565_lcd_s;    /* see drivers/lcd/st7565.h */
FAR struct lcd_dev_s *st7565_initialize(FAR struct st7565_lcd_s *lcd, 
                                        unsigned int devno
                                       );


#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_ST7567_H */
