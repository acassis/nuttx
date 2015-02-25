/****************************************************************************
 * drivers/sensors/inv_mpu6500.h
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

#ifndef _DRIVERS_SENSOR_INV_MPU6500_H_
#define _DRIVERS_SENSOR_INV_MPU6500_H_

#define INV_MPU_WHO_AM_I        0x75
#define INV_MPU_RATE_DIV        0x19
#define INV_MPU_LPF             0x1A
#define INV_MPU_PROD_ID         0x0C
#define INV_MPU_USER_CTRL       0x6A
#define INV_MPU_FIFO_EN         0x23
#define INV_MPU_GYRO_CFG        0x1B
#define INV_MPU_ACCEL_CFG       0x1C
#define INV_MPU_ACCEL_CFG2      0x1D
#define INV_MPU_LP_ACCEL_ODR    0x1E
#define INV_MPU_MOTION_THR      0x1F
#define INV_MPU_MOTION_DUR      0x20
#define INV_MPU_FIFO_COUNT_H    0x72
#define INV_MPU_FIFO_R_W        0x74
#define INV_MPU_RAW_GYRO        0x43
#define INV_MPU_RAW_ACCEL       0x3B
#define INV_MPU_TEMP            0x41
#define INV_MPU_INT_ENABLE      0x38
#define INV_MPU_DMP_INT_STATUS  0x39
#define INV_MPU_INT_STATUS      0x3A
#define INV_MPU_ACCEL_INTEL     0x69
#define INV_MPU_PWR_MGMT_1      0x6B
#define INV_MPU_PWR_MGMT_2      0x6C
#define INV_MPU_INT_PIN_CFG     0x37
#define INV_MPU_MEM_R_W         0x6F
#define INV_MPU_ACCEL_OFFS      0x77
#define INV_MPU_I2C_MST         0x24
#define INV_MPU_BANK_SEL        0x6D
#define INV_MPU_MEM_START_ADDR  0x6E
#define INV_MPU_PRGM_START_H    0x70

/* Hardware Value *****************************************************************/

#define INV_MPU_ADDR            0x68
#define INV_MPU_MAX_FIFO        1024
#define INV_MPU_NUM_REG         128
#define INV_MPU_TEMP_SENS       321
#define INV_MPU_TEMP_OFFSET     0
#define INV_MPU_BANK_SIZE       256

/* TEST Value *****************************************************************/

#define INV_MPU_GYRO_SENS       (32768/250)
#define INV_MPU_ACCEL_SENS      (32768/2)   /* FSR = +-2G = 16384 LSB/G */
#define INV_MPU_REG_RATE_DIV    (0)     /* 1kHz. */
#define INV_MPU_REG_LPF         (2)     /* 92Hz low pass filter*/
#define INV_MPU_REG_GYRO_FSR    (0)     /* 250dps. */
#define INV_MPU_REG_ACCEL_FSR   (0x0)   /* Accel FSR setting = 2g. */
#define INV_MPU_WAIT_MS         (200)   /* 200ms stabilization time */
#define INV_MPU_PACKET_THRESH   (200)   /* 200 samples */
#define INV_MPU_SAMPLE_WAIT_MS  (10)    /* 10ms sample time wait */

/* Criteria A */
#define INV_MPU_MAX_ACCEL_VAR   (.5f)   /* Accel must be within 50% variation */
#define INV_MPU_MAX_GYRO_VAR    (.5f)   /* Must exceed +50% variation */

/* Criteria B */
#define INV_MPU_MIN_ACCEL_G     (.225f) /* Accel must exceed Min 225 mg */
#define INV_MPU_MAX_ACCEL_G     (.675f) /* Accel cannot exceed Max 675 mg */
#define INV_MPU_MAX_GYRO_DPS    (60.f)  /* Must exceed 60 dps threshold */

/* Criteria C */
#define INV_MPU_min_GYRO_DPS        (20.f)  /* 20 dps for Gyro */
#define INV_MPU_max_ACCEL_G_OFFSET  (.5f)   /* 500 mg for Accel */

#define HWST_MAX_PACKET_LENGTH (512)

#endif  /* _DRIVERS_SENSOR_INV_MPU6500_H_ */

