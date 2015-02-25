/****************************************************************************
 * drivers/sensors/inv_mpu6050.h
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

#ifndef _DRIVERS_SENSOR_INV_MPU6050_H_
#define _DRIVERS_SENSOR_INV_MPU6050_H_

#define INV_MPU_WHO_AM_I        0X75
#define INV_MPU_RATE_DIV        0X19
#define INV_MPU_LPF             0X1A
#define INV_MPU_PROD_ID         0X0C
#define INV_MPU_USER_CTRL       0X6A
#define INV_MPU_FIFO_EN         0X23
#define INV_MPU_GYRO_CFG        0X1B
#define INV_MPU_ACCEL_CFG       0X1C
#define INV_MPU_MOTION_THR      0X1F
#define INV_MPU_MOTION_DUR      0X20
#define INV_MPU_FIFO_COUNT_H    0X72
#define INV_MPU_FIFO_R_W        0X74
#define INV_MPU_RAW_GYRO        0X43
#define INV_MPU_RAW_ACCEL       0X3B
#define INV_MPU_TEMP            0X41
#define INV_MPU_INT_ENABLE      0X38
#define INV_MPU_DMP_INT_STATUS  0X39
#define INV_MPU_INT_STATUS      0X3A
#define INV_MPU_PWR_MGMT_1      0X6B
#define INV_MPU_PWR_MGMT_2      0X6C
#define INV_MPU_INT_PIN_CFG     0X37
#define INV_MPU_MEM_R_W         0X6F
#define INV_MPU_ACCEL_OFFS      0X06
#define INV_MPU_I2C_MST         0X24
#define INV_MPU_BANK_SEL        0X6D
#define INV_MPU_MEM_START_ADDR  0X6E
#define INV_MPU_PRGM_START_H    0X70

/* Hardware Value *****************************************************************/

#define INV_MPU_MAX_FIFO    0x1024
#define INV_MPU_NUM_REG     118
#define INV_MPU_TEMP_SENS   340
#define INV_MPU_TEMP_OFFSET -521
#define INV_MPU_BANK_SIZE   256

/* TEST Value *****************************************************************/

#define INV_MPU_GYRO_SENS       (32768/250)
#define INV_MPU_ACCEL_SENS      (32768/16)
#define INV_MPU_REG_RATE_DIV    (0)    /* 1kHz. */
#define INV_MPU_REG_LPF         (1)    /* 188Hz. */
#define INV_MPU_REG_GYRO_FSR    (0)    /* 250dps. */
#define INV_MPU_REG_ACCEL_FSR   (0x18) /* 16g. */
#define INV_MPU_PACKET_THRESH   (5)    /* 5% */
#define INV_MPU_WAIT_MS         (50)

/* Criteria A */
#define INV_MPU_MAX_ACCEL_VAR   (0.14f)
#define INV_MPU_MAX_GYRO_VAR    (0.14f)

/* Criteria B */
#define INV_MPU_MIN_ACCEL_G     (0.3f)
#define INV_MPU_MAX_ACCEL_G     (0.95f)
#define INV_MPU_MAX_GYRO_DPS    (105.f)

/* Criteria C */
#define INV_MPU_MIN_DPS         (10.f)

#define HWST_MAX_PACKET_LENGTH  (512)

#endif  /* _DRIVERS_SENSOR_INV_MPU6050_H_ */

