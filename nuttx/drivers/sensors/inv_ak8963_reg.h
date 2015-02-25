/****************************************************************************
 * drivers/sensors/inv_ak8963.h
 *
 *   Copyright (C) 2015 Pierre-noel Bouteville . All rights reserved.
 *   Authors: Pierre-noel Bouteville <pnb990@gmail.com>
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

#ifndef _DRIVERS_SENSOR_INV_AK8963_REG_H_
#define _DRIVERS_SENSOR_INV_AK8963_REG_H_

#define INV_AK89_RAW_COMPASS    0X49
#define INV_AK89_S0_ADDR        0X25
#define INV_AK89_S0_REG         0X26
#define INV_AK89_S0_CTRL        0X27
#define INV_AK89_S1_ADDR        0X28
#define INV_AK89_S1_REG         0X29
#define INV_AK89_S1_CTRL        0X2A
#define INV_AK89_S4_CTRL        0X34
#define INV_AK89_S0_DO          0X63
#define INV_AK89_S1_DO          0X64
#define INV_AK89_I2C_DELAY_CTRL 0X67

#define INV_AK89_HIGH_SENS   (0x10)

/* the compass full-scale range. */
#define INV_AK89_FSR         (4915)


#define AKM_REG_WHOAMI      (0x00)

#define AKM_REG_ST1         (0x02)
#define AKM_REG_HXL         (0x03)
#define AKM_REG_ST2         (0x09)

#define AKM_REG_CNTL        (0x0A)
#define AKM_REG_ASTC        (0x0C)
#define AKM_REG_ASAX        (0x10)
#define AKM_REG_ASAY        (0x11)
#define AKM_REG_ASAZ        (0x12)

#define AKM_DATA_READY      (0x01)
#define AKM_DATA_OVERRUN    (0x02)
#define AKM_OVERFLOW        (0x80)
#define AKM_DATA_ERROR      (0x40)

#define AKM_BIT_SELF_TEST   (0x40)

#define AKM_POWER_DOWN          (0x00 | INV_AK89_HIGH_SENS)
#define AKM_SINGLE_MEASUREMENT  (0x01 | INV_AK89_HIGH_SENS)
#define AKM_FUSE_ROM_ACCESS     (0x0F | INV_AK89_HIGH_SENS)
#define AKM_MODE_SELF_TEST      (0x08 | INV_AK89_HIGH_SENS)

#define AKM_WHOAMI      (0x48)

#define MAX_COMPASS_SAMPLE_RATE (100)

#endif  /* _DRIVERS_SENSOR_INV_AK8963_REG_H_ */
