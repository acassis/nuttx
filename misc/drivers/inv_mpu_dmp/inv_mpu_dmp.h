/****************************************************************************
 * drivers/sensors/inv_mpu_dmp.h
 *
 * Modification to fit Nuttx requierement.
 *   Copyright (C) 2015 Pierre-noel Bouteville . All rights reserved.
 *   Authors: Pierre-noel Bouteville <pnb990@gmail.com>
 *
 * License:
 *   Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
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
  
#ifndef __MISC_DRIVERS_INV_MPU_DMP_INPUT_INV_MPU_DMP_H
#define __MISC_DRIVERS_INV_MPU_DMP_INPUT_INV_MPU_DMP_H
  
#define TAP_X               (0x01)
#define TAP_Y               (0x02)
#define TAP_Z               (0x04)
#define TAP_XYZ             (0x07)
 
#define TAP_X_UP            (0x01)
#define TAP_X_DOWN          (0x02)
#define TAP_Y_UP            (0x03)
#define TAP_Y_DOWN          (0x04)
#define TAP_Z_UP            (0x05)
#define TAP_Z_DOWN          (0x06)
 
#define ANDROID_ORIENT_PORTRAIT             (0x00)
#define ANDROID_ORIENT_LANDSCAPE            (0x01)
#define ANDROID_ORIENT_REVERSE_PORTRAIT     (0x02)
#define ANDROID_ORIENT_REVERSE_LANDSCAPE    (0x03)
 
#define DMP_INT_GESTURE     (0x01)
#define DMP_INT_CONTINUOUS  (0x02)
 
#define DMP_FEATURE_TAP             (0x001)
#define DMP_FEATURE_ANDROID_ORIENT  (0x002)
#define DMP_FEATURE_LP_QUAT         (0x004)
#define DMP_FEATURE_PEDOMETER       (0x008)
#define DMP_FEATURE_6X_LP_QUAT      (0x010)
#define DMP_FEATURE_GYRO_CAL        (0x020)
#define DMP_FEATURE_SEND_RAW_ACCEL  (0x040)
#define DMP_FEATURE_SEND_RAW_GYRO   (0x080)
#define DMP_FEATURE_SEND_CAL_GYRO   (0x100)
 
#define INV_WXYZ_QUAT       (0x100)
  
struct dmp_s;

/* Set up functions. */ 

int dmp_load_motion_driver_firmware(struct dmp_s* dmp);
int dmp_set_fifo_rate(          struct dmp_s* dmp,uint16_t rate);
int dmp_get_fifo_rate(          struct dmp_s* dmp,uint16_t *rate);
int dmp_enable_feature(         struct dmp_s* dmp,uint16_t mask);
int dmp_get_enabled_features(   struct dmp_s* dmp,uint16_t *mask);
int dmp_set_interrupt_mode(     struct dmp_s* dmp,uint8_t mode);
int dmp_set_orientation(        struct dmp_s* dmp,uint16_t orient);
int dmp_set_gyro_bias(          struct dmp_s* dmp,int32_t *bias);
int dmp_set_accel_bias(         struct dmp_s* dmp,int32_t *bias);
 

/* Tap functions. */ 

int dmp_register_tap_cb(void (*func) (struct dmp_s* dmp, uint8_t, uint8_t));
int dmp_set_tap_thresh(struct dmp_s* dmp,uint8_t axis, uint16_t thresh);
int dmp_set_tap_axes(struct dmp_s* dmp,uint8_t axis);
int dmp_set_tap_count(struct dmp_s* dmp,uint8_t min_taps);
int dmp_set_tap_time(struct dmp_s* dmp,uint16_t time);
int dmp_set_tap_time_multi(struct dmp_s* dmp,uint16_t time);
int dmp_set_shake_reject_thresh(struct dmp_s* dmp,int32_t sf, uint16_t thresh);
int dmp_set_shake_reject_time(struct dmp_s* dmp,uint16_t time);
int dmp_set_shake_reject_timeout(struct dmp_s* dmp,uint16_t time);
 

/* Android orientation functions. */ 

int dmp_register_android_orient_cb(void (*func) (struct dmp_s* dmp,uint8_t));

/* LP quaternion functions. */ 

int dmp_enable_lp_quat(struct dmp_s* dmp,uint8_t enable);
int dmp_enable_6x_lp_quat(struct dmp_s* dmp,uint8_t enable);
 

/* Pedometer functions. */ 

int dmp_get_pedometer_step_count(   struct dmp_s* dmp,uint32_t *count);
int dmp_set_pedometer_step_count(   struct dmp_s* dmp,uint32_t count);
int dmp_get_pedometer_walk_time(    struct dmp_s* dmp,uint32_t *time);
int dmp_set_pedometer_walk_time(    struct dmp_s* dmp,uint32_t time);
 

/* DMP gyro calibration functions. */ 

int dmp_enable_gyro_cal(struct dmp_s* dmp,uint8_t enable);
 

/* Read function. This function should be called whenever the MPU interrupt is
 * detected.
 */ 
 
int dmp_read_fifo(struct dmp_s* dmp, int16_t *gyro, int16_t *accel, int32_t *quat, 
                  uint32_t *timestamp, int16_t *sensors, 
                  uint8_t *more);
 
#endif  /* #ifndef __MISC_DRIVERS_INV_MPU_DMP_INPUT_INV_MPU_DMP_H */
  
