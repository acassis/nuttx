/****************************************************************************
 * drivers/sensors/inv_mpu_dmp_firmware.h
 *
 * Modification to fit Nuttx requierement.
 *   Copyright (C) 2015 Pierre-noel Bouteville . All rights reserved.
 *   Authors: Pierre-noel Bouteville <pnb990@gmail.com>
 *
 * License:
 *  Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
 *  See included License.txt for License information.
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

#ifndef __MISC_DRIVERS_INV_MPU_DMP_INPUT_INV_MPU_FIRMWARE_H
#define __MISC_DRIVERS_INV_MPU_DMP_INPUT_INV_MPU_FIRMWARE_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <nuttx/i2c.h>
#include <nuttx/spi/spi.h>

#include <nuttx/irq.h>

/******************************************************************************
 * Pre-Processor Definitions
 ******************************************************************************/
/* Configuration **************************************************************/

#ifdef CONFIG_INVENSENSE_DMP

/* These defines may change for each DMP image, so be sure to modify
 * these values when switching to a new image.
 */
#define DMP_CFG_LP_QUAT             (2712)
#define DMP_END_ORIENT_TEMP         (1866)
#define DMP_CFG_27                  (2742)
#define DMP_CFG_20                  (2224)
#define DMP_CFG_23                  (2745)
#define DMP_CFG_FIFO_ON_EVENT       (2690)
#define DMP_END_PREDICTION_UPDATE   (1761)
#define DMP_CGNOTICE_INTR           (2620)
#define DMP_X_GRT_Y_TMP             (1358)
#define DMP_CFG_DR_INT              (1029)
#define DMP_CFG_AUTH                (1035)
#define DMP_UPDATE_PROP_ROT         (1835)
#define DMP_END_COMPARE_Y_X_TMP2    (1455)
#define DMP_SKIP_X_GRT_Y_TMP        (1359)
#define DMP_SKIP_END_COMPARE        (1435)
#define DMP_FCFG_3                  (1088)
#define DMP_FCFG_2                  (1066)
#define DMP_FCFG_1                  (1062)
#define DMP_END_COMPARE_Y_X_TMP3    (1434)
#define DMP_FCFG_7                  (1073)
#define DMP_FCFG_6                  (1106)
#define DMP_FLAT_STATE_END          (1713)
#define DMP_SWING_END_4             (1616)
#define DMP_SWING_END_2             (1565)
#define DMP_SWING_END_3             (1587)
#define DMP_SWING_END_1             (1550)
#define DMP_CFG_8                   (2718)
#define DMP_CFG_15                  (2727)
#define DMP_CFG_16                  (2746)
#define DMP_CFG_EXT_GYRO_BIAS       (1189)
#define DMP_END_COMPARE_Y_X_TMP     (1407)
#define DMP_DO_NOT_UPDATE_PROP_ROT  (1839)
#define DMP_CFG_7                   (1205)
#define DMP_FLAT_STATE_END_TEMP     (1683)
#define DMP_END_COMPARE_Y_X         (1484)
#define DMP_SKIP_SWING_END_1        (1551)
#define DMP_SKIP_SWING_END_3        (1588)
#define DMP_SKIP_SWING_END_2        (1566)
#define DMP_TILTG75_START           (1672)
#define DMP_CFG_6                   (2753)
#define DMP_TILTL75_END             (1669)
#define DMP_END_ORIENT              (1884)
#define DMP_CFG_FLICK_IN            (2573)
#define DMP_TILTL75_START           (1643)
#define DMP_CFG_MOTION_BIAS         (1208)
#define DMP_X_GRT_Y                 (1408)
#define DMP_TEMPLABEL               (2324)
#define DMP_CFG_ANDROID_ORIENT_INT  (1853)
#define DMP_CFG_GYRO_RAW_DATA       (2722)
#define DMP_X_GRT_Y_TMP2            (1379)

#define DMP_D_0_22                  (22+512)
#define DMP_D_0_24                  (24+512)

#define DMP_D_0_36                  (36)
#define DMP_D_0_52                  (52)
#define DMP_D_0_96                  (96)
#define DMP_D_0_104                 (104)
#define DMP_D_0_108                 (108)
#define DMP_D_0_163                 (163)
#define DMP_D_0_188                 (188)
#define DMP_D_0_192                 (192)
#define DMP_D_0_224                 (224)
#define DMP_D_0_228                 (228)
#define DMP_D_0_232                 (232)
#define DMP_D_0_236                 (236)

#define DMP_D_1_2                   (256 + 2)
#define DMP_D_1_4                   (256 + 4)
#define DMP_D_1_8                   (256 + 8)
#define DMP_D_1_10                  (256 + 10)
#define DMP_D_1_24                  (256 + 24)
#define DMP_D_1_28                  (256 + 28)
#define DMP_D_1_36                  (256 + 36)
#define DMP_D_1_40                  (256 + 40)
#define DMP_D_1_44                  (256 + 44)
#define DMP_D_1_72                  (256 + 72)
#define DMP_D_1_74                  (256 + 74)
#define DMP_D_1_79                  (256 + 79)
#define DMP_D_1_88                  (256 + 88)
#define DMP_D_1_90                  (256 + 90)
#define DMP_D_1_92                  (256 + 92)
#define DMP_D_1_96                  (256 + 96)
#define DMP_D_1_98                  (256 + 98)
#define DMP_D_1_106                 (256 + 106)
#define DMP_D_1_108                 (256 + 108)
#define DMP_D_1_112                 (256 + 112)
#define DMP_D_1_128                 (256 + 144)
#define DMP_D_1_152                 (256 + 12)
#define DMP_D_1_160                 (256 + 160)
#define DMP_D_1_176                 (256 + 176)
#define DMP_D_1_178                 (256 + 178)
#define DMP_D_1_218                 (256 + 218)
#define DMP_D_1_232                 (256 + 232)
#define DMP_D_1_236                 (256 + 236)
#define DMP_D_1_240                 (256 + 240)
#define DMP_D_1_244                 (256 + 244)
#define DMP_D_1_250                 (256 + 250)
#define DMP_D_1_252                 (256 + 252)
#define DMP_D_2_12                  (512 + 12)
#define DMP_D_2_96                  (512 + 96)
#define DMP_D_2_108                 (512 + 108)
#define DMP_D_2_208                 (512 + 208)
#define DMP_D_2_224                 (512 + 224)
#define DMP_D_2_236                 (512 + 236)
#define DMP_D_2_244                 (512 + 244)
#define DMP_D_2_248                 (512 + 248)
#define DMP_D_2_252                 (512 + 252)

#define DMP_CPASS_BIAS_X            (35 * 16 + 4)
#define DMP_CPASS_BIAS_Y            (35 * 16 + 8)
#define DMP_CPASS_BIAS_Z            (35 * 16 + 12)
#define DMP_CPASS_MTX_00            (36 * 16)
#define DMP_CPASS_MTX_01            (36 * 16 + 4)
#define DMP_CPASS_MTX_02            (36 * 16 + 8)
#define DMP_CPASS_MTX_10            (36 * 16 + 12)
#define DMP_CPASS_MTX_11            (37 * 16)
#define DMP_CPASS_MTX_12            (37 * 16 + 4)
#define DMP_CPASS_MTX_20            (37 * 16 + 8)
#define DMP_CPASS_MTX_21            (37 * 16 + 12)
#define DMP_CPASS_MTX_22            (43 * 16 + 12)
#define DMP_D_EXT_GYRO_BIAS_X       (61 * 16)
#define DMP_D_EXT_GYRO_BIAS_Y       (61 * 16) + 4
#define DMP_D_EXT_GYRO_BIAS_Z       (61 * 16) + 8
#define DMP_D_ACT0                  (40 * 16)
#define DMP_D_ACSX                  (40 * 16 + 4)
#define DMP_D_ACSY                  (40 * 16 + 8)
#define DMP_D_ACSZ                  (40 * 16 + 12)

#define DMP_FLICK_MSG               (45 * 16 + 4)
#define DMP_FLICK_COUNTER           (45 * 16 + 8)
#define DMP_FLICK_LOWER             (45 * 16 + 12)
#define DMP_FLICK_UPPER             (46 * 16 + 12)

#define DMP_D_AUTH_OUT              (992)
#define DMP_D_AUTH_IN               (996)
#define DMP_D_AUTH_A                (1000)
#define DMP_D_AUTH_B                (1004)

#define DMP_D_PEDSTD_BP_B           (768 + 0x1C)
#define DMP_D_PEDSTD_HP_A           (768 + 0x78)
#define DMP_D_PEDSTD_HP_B           (768 + 0x7C)
#define DMP_D_PEDSTD_BP_A4          (768 + 0x40)
#define DMP_D_PEDSTD_BP_A3          (768 + 0x44)
#define DMP_D_PEDSTD_BP_A2          (768 + 0x48)
#define DMP_D_PEDSTD_BP_A1          (768 + 0x4C)
#define DMP_D_PEDSTD_INT_THRSH      (768 + 0x68)
#define DMP_D_PEDSTD_CLIP           (768 + 0x6C)
#define DMP_D_PEDSTD_SB             (768 + 0x28)
#define DMP_D_PEDSTD_SB_TIME        (768 + 0x2C)
#define DMP_D_PEDSTD_PEAKTHRSH      (768 + 0x98)
#define DMP_D_PEDSTD_TIML           (768 + 0x2A)
#define DMP_D_PEDSTD_TIMH           (768 + 0x2E)
#define DMP_D_PEDSTD_PEAK           (768 + 0X94)
#define DMP_D_PEDSTD_STEPCTR        (768 + 0x60)
#define DMP_D_PEDSTD_TIMECTR        (964)
#define DMP_D_PEDSTD_DECI           (768 + 0xA0)

#define DMP_D_HOST_NO_MOT           (976)
#define DMP_D_ACCEL_BIAS            (660)

#define DMP_D_ORIENT_GAP            (76)

#define DMP_D_TILT0_H               (48)
#define DMP_D_TILT0_L               (50)
#define DMP_D_TILT1_H               (52)
#define DMP_D_TILT1_L               (54)
#define DMP_D_TILT2_H               (56)
#define DMP_D_TILT2_L               (58)
#define DMP_D_TILT3_H               (60)
#define DMP_D_TILT3_L               (62)

#define DMP_CODE_SIZE               (3062)
#define DMP_SAMPLE_RATE             (200)



/******************************************************************************
 * Public Types
 ******************************************************************************/


/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

struct dmp_firmware_s
{
    int start_addr;
    int sample_rate;
    int size;
    uint8_t data[]; 
};

const struct dmp_firmware_s* dmp_firmware;


#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif  /* #ifdef CONFIG_INVENSENSE_DMP */
#endif  /* #ifndef __INCLUDE_NUTTX_INPUT_INV_MPU_FIRMWARE_H */


