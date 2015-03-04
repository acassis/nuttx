/****************************************************************************
 * drivers/sensors/inv_mpu.h
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

#ifndef _DRIVERS_SENSOR_INV_MPU_H_
#define _DRIVERS_SENSOR_INV_MPU_H_

#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

#define MPU_INT_STATUS_DATA_READY       (0x0001)
#define MPU_INT_STATUS_DMP              (0x0002)
#define MPU_INT_STATUS_PLL_READY        (0x0004)
#define MPU_INT_STATUS_I2C_MST          (0x0008)
#define MPU_INT_STATUS_FIFO_OVERFLOW    (0x0010)
#define MPU_INT_STATUS_ZMOT             (0x0020)
#define MPU_INT_STATUS_MOT              (0x0040)
#define MPU_INT_STATUS_FREE_FALL        (0x0080)
#define MPU_INT_STATUS_DMP_0            (0x0100)
#define MPU_INT_STATUS_DMP_1            (0x0200)
#define MPU_INT_STATUS_DMP_2            (0x0400)
#define MPU_INT_STATUS_DMP_3            (0x0800)
#define MPU_INT_STATUS_DMP_4            (0x1000)
#define MPU_INT_STATUS_DMP_5            (0x2000)

struct mpu_lowlevel_s {
    void *priv;
    int (*write)(void * priv,int reg, uint8_t*buf, int size);
    int (*read )(void * priv,int reg, uint8_t*buf, int size);
};

struct mpu_inst_s;

/* Set up APIs */
struct mpu_inst_s* mpu_instantiate(struct mpu_lowlevel_s* low, int devno)
int mpu_reset_default(struct mpu_inst_s* inst);
int mpu_set_bypass(struct mpu_inst_s* inst, bool bypass_on);

/* Configuration APIs */
int mpu_lp_accel_mode(struct mpu_inst_s* inst, uint8_t rate);
int mpu_lp_motion_interrupt(struct mpu_inst_s* inst,uint16_t thresh, 
                            uint8_t time, uint8_t lpa_freq);
int mpu_set_int_level(struct mpu_inst_s* inst, uint8_t active_low);
int mpu_set_int_latched(struct mpu_inst_s* inst,bool enable);

int mpu_set_dmp_state(struct mpu_inst_s* inst,bool enable);
int mpu_get_dmp_state(struct mpu_inst_s* inst,bool *enabled);

int mpu_get_lpf(struct mpu_inst_s* inst,uint16_t *lpf);
int mpu_set_lpf(struct mpu_inst_s* inst,uint16_t lpf);

int mpu_get_gyro_fsr(struct mpu_inst_s* inst,uint16_t *fsr);
int mpu_set_gyro_fsr(struct mpu_inst_s* inst,uint16_t fsr);

int mpu_get_accel_fsr(struct mpu_inst_s* inst,uint8_t *fsr);
int mpu_set_accel_fsr(struct mpu_inst_s* inst,uint8_t fsr);

int mpu_get_compass_fsr(struct mpu_inst_s* inst,uint16_t *fsr);

int mpu_get_gyro_sens(struct mpu_inst_s* inst,float *sens);
int mpu_get_accel_sens(struct mpu_inst_s* inst,uint16_t *sens);

int mpu_get_sample_rate(struct mpu_inst_s* inst,uint16_t *rate);
int mpu_set_sample_rate(struct mpu_inst_s* inst,uint16_t rate);
int mpu_get_compass_sample_rate(struct mpu_inst_s* inst,uint16_t *rate);
int mpu_set_compass_sample_rate(struct mpu_inst_s* inst,uint16_t rate);

int mpu_get_fifo_config(struct mpu_inst_s* inst,uint8_t *sensors);
int mpu_configure_fifo(struct mpu_inst_s* inst,uint8_t sensors);

int mpu_get_power_state(struct mpu_inst_s* inst,uint8_t *power_on);
int mpu_set_sensors(struct mpu_inst_s* inst,uint8_t sensors);

int mpu_read_6500_accel_bias(struct mpu_inst_s* inst,long *accel_bias);
int mpu_set_gyro_bias_reg(struct mpu_inst_s* inst,long * gyro_bias);
int mpu_set_accel_bias_6500_reg(struct mpu_inst_s* inst,const long *accel_bias);
int mpu_read_6050_accel_bias(struct mpu_inst_s* inst,long *accel_bias);
int mpu_set_accel_bias_6050_reg(struct mpu_inst_s* inst,const long *accel_bias);

/* Data getter/setter APIs */
int mpu_get_gyro_reg(struct mpu_inst_s* inst,int16_t *data, 
                     uint32_t *timestamp);
int mpu_get_accel_reg(struct mpu_inst_s* inst,int16_t *data, 
                      uint32_t *timestamp);
int mpu_get_compass_reg(struct mpu_inst_s* inst,int16_t *data, 
                        uint32_t *timestamp);
int mpu_get_temperature(struct mpu_inst_s* inst,long *data, 
                        uint32_t *timestamp);

int mpu_get_int_status(struct mpu_inst_s* inst,int16_t *status);
int mpu_read_fifo(struct mpu_inst_s* inst,int16_t *gyro, int16_t *accel, 
                  uint32_t *timestamp, uint8_t *sensors, uint8_t *more);
int mpu_read_fifo_stream(struct mpu_inst_s* inst,uint16_t length, uint8_t *data,
                         uint8_t *more);
int mpu_reset_fifo(struct mpu_inst_s* inst);

int mpu_write_mem(struct mpu_inst_s* inst,uint16_t mem_addr, uint16_t length,
    uint8_t *data);
int mpu_read_mem(struct mpu_inst_s* inst,uint16_t mem_addr, uint16_t length,
    uint8_t *data);
int mpu_load_firmware(struct mpu_inst_s* inst,uint16_t length, 
                      const uint8_t *firmware, uint16_t start_addr, 
                      uint16_t sample_rate);

int mpu_reg_dump(struct mpu_inst_s* inst);
int mpu_read_reg(struct mpu_inst_s* inst,uint8_t reg, uint8_t *data);
int mpu_run_self_test(struct mpu_inst_s* inst,long *gyro, long *accel);
int mpu_run_6500_self_test(struct mpu_inst_s* inst,long *gyro, long *accel, 
                           uint8_t debug);

#endif  /* #ifndef _INV_MPU_H_ */

