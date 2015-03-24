/****************************************************************************
 * drivers/sensors/inv_mpu_dmp.c
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
  
  

/****************************************************************************
 * Included Files
 ****************************************************************************/ 
  
#include <nuttx/config.h>
  
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
  
#include "inv_mpu.h"
#include "inv_mpu_dmp.h"
#include "inv_mpu_dmp_key.h"
#include "inv_mpu_dmp_map.h"
  

/*******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/ 
  
#define INT_SRC_TAP             (0x01)
#define INT_SRC_ANDROID_ORIENT  (0x08)
  
#define DMP_FEATURE_SEND_ANY_GYRO   (DMP_FEATURE_SEND_RAW_GYRO | \
  DMP_FEATURE_SEND_CAL_GYRO)  
#define MAX_PACKET_LENGTH   (32)
  
#define GYRO_SF             (46850825LL * 200 / MPU_DMP_SAMPLE_RATE)
  
#define FIFO_CORRUPTION_CHECK
#ifdef FIFO_CORRUPTION_CHECK
#  define QUAT_ERROR_THRESH       (1L<<24)
#  define QUAT_MAG_SQ_NORMALIZED  (1L<<28)
#  define QUAT_MAG_SQ_MIN         (QUAT_MAG_SQ_NORMALIZED - QUAT_ERROR_THRESH)
#  define QUAT_MAG_SQ_MAX         (QUAT_MAG_SQ_NORMALIZED + QUAT_ERROR_THRESH)
#endif
   

/*******************************************************************************
 * Private Types
 ******************************************************************************/ 
  struct dmp_s
  {
    void (*tap_cb) (uint8_t count, uint8_t direction);
    void (*android_orient_cb) (uint8_t orientation);
    uint16_t orient;
    uint16_t feature_mask;
    uint16_t fifo_rate;
    uint8_t packet_length;
  };

 static struct dmp_s dmp =
  {
  .tap_cb = NULL, .android_orient_cb = NULL, .orient = 0, .feature_mask = 0, .fifo_rate = 0, .packet_length = 0 };
 

/****************************************************************************
 * Public Data
 ****************************************************************************/ 
  

/****************************************************************************
 * Private Functions
 ****************************************************************************/ 
  

/*******************************************************************************
 * Private Data
 ******************************************************************************/ 
  

/****************************************************************************
 * Public Functions
 ****************************************************************************/ 
  

/*
 *  Name:dmp_set_orientation
 *  Description:
 *      Push gyro and accel orientation to the DMP.
 *   The orientation is represented here as the output of
 *   inv_orientation_matrix_to_scalar.
 *
 *  Parameters:
 *   orient  Gyro and accel orientation in body frame.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_set_orientation(uint16_t orient) 
{
  uint8_t gyro_regs[3];
  uint8_t accel_regs[3];
  const uint8_t gyro_axes[3] =
  {
  DINA4C, DINACD, DINA6C
  };
  const uint8_t accel_axes[3] =
  {
  DINA0C, DINAC9, DINA2C
  };
  const uint8_t gyro_sign[3] =
  {
  DINA36, DINA56, DINA76
  };
  const uint8_t accel_sign[3] =
  {
  DINA26, DINA46, DINA66
  };
   gyro_regs[0] = gyro_axes[orient & 3];
  gyro_regs[1] = gyro_axes[(orient >> 3) & 3];
  gyro_regs[2] = gyro_axes[(orient >> 6) & 3];
  accel_regs[0] = accel_axes[orient & 3];
  accel_regs[1] = accel_axes[(orient >> 3) & 3];
  accel_regs[2] = accel_axes[(orient >> 6) & 3];
   
    /* Chip-to-body, axes only. */ 
    if (mpu_write_mem(FCFG_1, 3, gyro_regs))
    return -1;
  if (mpu_write_mem(FCFG_2, 3, accel_regs))
    return -1;
   memcpy(gyro_regs, gyro_sign, 3);
  memcpy(accel_regs, accel_sign, 3);
  if (orient & 4)
    {
      gyro_regs[0] |= 1;
      accel_regs[0] |= 1;
    }
  if (orient & 0x20)
    {
      gyro_regs[1] |= 1;
      accel_regs[1] |= 1;
    }
  if (orient & 0x100)
    {
      gyro_regs[2] |= 1;
      accel_regs[2] |= 1;
    }
   
    /* Chip-to-body, sign only. */ 
    if (mpu_write_mem(FCFG_3, 3, gyro_regs))
    return -1;
  if (mpu_write_mem(FCFG_7, 3, accel_regs))
    return -1;
  dmp.orient = orient;
  return 0;
}

 

/*
 *  Name: dmp_set_gyro_bias
 *  Description:
 *   Push gyro biases to the DMP.
 *   Because the gyro integration is handled in the DMP, any gyro biases
 *   calculated by the MPL should be pushed down to DMP memory to remove
 *   3-axis quaternion drift.
 *
 *  Note: 
 *   If the DMP-based gyro calibration is enabled, the DMP will
 *   overwrite the biases written to this location once a new one is computed.
 *  Parameters
 *   bias    Gyro biases in q16.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_set_gyro_bias(int32_t * bias) 
{
  int32_t gyro_bias_body[3];
  uint8_t regs[4];
   gyro_bias_body[0] = bias[dmp.orient & 3];
  if (dmp.orient & 4)
    gyro_bias_body[0] *= -1;
  gyro_bias_body[1] = bias[(dmp.orient >> 3) & 3];
  if (dmp.orient & 0x20)
    gyro_bias_body[1] *= -1;
  gyro_bias_body[2] = bias[(dmp.orient >> 6) & 3];
  if (dmp.orient & 0x100)
    gyro_bias_body[2] *= -1;
   
#ifdef EMPL_NO_64BIT
    gyro_bias_body[0] =
    (int32_t) (((float)gyro_bias_body[0] * GYRO_SF) / 1073741824.f);
  	gyro_bias_body[1] =
    (int32_t) (((float)gyro_bias_body[1] * GYRO_SF) / 1073741824.f);
  	gyro_bias_body[2] =
    (int32_t) (((float)gyro_bias_body[2] * GYRO_SF) / 1073741824.f);
  
#else
    gyro_bias_body[0] =
    (int32_t) (((int32_t int32_t) gyro_bias_body[0] * GYRO_SF) >> 30);
  	gyro_bias_body[1] =
    (int32_t) (((int32_t int32_t) gyro_bias_body[1] * GYRO_SF) >> 30);
  	gyro_bias_body[2] =
    (int32_t) (((int32_t int32_t) gyro_bias_body[2] * GYRO_SF) >> 30);
  
#endif
    regs[0] = (uint8_t) ((gyro_bias_body[0] >> 24) & 0xFF);
  regs[1] = (uint8_t) ((gyro_bias_body[0] >> 16) & 0xFF);
  regs[2] = (uint8_t) ((gyro_bias_body[0] >> 8) & 0xFF);
  regs[3] = (uint8_t) (gyro_bias_body[0] & 0xFF);
  if (mpu_write_mem(D_EXT_GYRO_BIAS_X, 4, regs))
    return -1;
   regs[0] = (uint8_t) ((gyro_bias_body[1] >> 24) & 0xFF);
  regs[1] = (uint8_t) ((gyro_bias_body[1] >> 16) & 0xFF);
  regs[2] = (uint8_t) ((gyro_bias_body[1] >> 8) & 0xFF);
  regs[3] = (uint8_t) (gyro_bias_body[1] & 0xFF);
  if (mpu_write_mem(D_EXT_GYRO_BIAS_Y, 4, regs))
    return -1;
   regs[0] = (uint8_t) ((gyro_bias_body[2] >> 24) & 0xFF);
  regs[1] = (uint8_t) ((gyro_bias_body[2] >> 16) & 0xFF);
  regs[2] = (uint8_t) ((gyro_bias_body[2] >> 8) & 0xFF);
  regs[3] = (uint8_t) (gyro_bias_body[2] & 0xFF);
  return mpu_write_mem(D_EXT_GYRO_BIAS_Z, 4, regs);
}

 

/*
 *  Name: dmp_set_accel_bias
 *  Description:
 *   Push accel biases to the DMP.
 *  These biases will be removed from the DMP 6-axis quaternion.
 *
 *  Parameters
 *   bias    Accel biases in q16.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_set_accel_bias(int32_t * bias) 
{
  int32_t accel_bias_body[3];
  uint8_t regs[12];
  int32_t int32_t accel_sf;
  uint16_t accel_sens;
   mpu_get_accel_sens(&accel_sens);
  accel_sf = (int32_t int32_t) accel_sens << 15;
  __no_operation();
   accel_bias_body[0] = bias[dmp.orient & 3];
  if (dmp.orient & 4)
    accel_bias_body[0] *= -1;
  accel_bias_body[1] = bias[(dmp.orient >> 3) & 3];
  if (dmp.orient & 0x20)
    accel_bias_body[1] *= -1;
  accel_bias_body[2] = bias[(dmp.orient >> 6) & 3];
  if (dmp.orient & 0x100)
    accel_bias_body[2] *= -1;
   
#ifdef EMPL_NO_64BIT
    accel_bias_body[0] =
    (int32_t) (((float)accel_bias_body[0] * accel_sf) / 1073741824.f);
  	accel_bias_body[1] =
    (int32_t) (((float)accel_bias_body[1] * accel_sf) / 1073741824.f);
  	accel_bias_body[2] =
    (int32_t) (((float)accel_bias_body[2] * accel_sf) / 1073741824.f);
  
#else
    accel_bias_body[0] =
    (int32_t) (((int32_t int32_t) accel_bias_body[0] * accel_sf) >> 30);
  	accel_bias_body[1] =
    (int32_t) (((int32_t int32_t) accel_bias_body[1] * accel_sf) >> 30);
  	accel_bias_body[2] =
    (int32_t) (((int32_t int32_t) accel_bias_body[2] * accel_sf) >> 30);
  
#endif
    regs[0] = (uint8_t) ((accel_bias_body[0] >> 24) & 0xFF);
  regs[1] = (uint8_t) ((accel_bias_body[0] >> 16) & 0xFF);
  regs[2] = (uint8_t) ((accel_bias_body[0] >> 8) & 0xFF);
  regs[3] = (uint8_t) (accel_bias_body[0] & 0xFF);
  regs[4] = (uint8_t) ((accel_bias_body[1] >> 24) & 0xFF);
  regs[5] = (uint8_t) ((accel_bias_body[1] >> 16) & 0xFF);
  regs[6] = (uint8_t) ((accel_bias_body[1] >> 8) & 0xFF);
  regs[7] = (uint8_t) (accel_bias_body[1] & 0xFF);
  regs[8] = (uint8_t) ((accel_bias_body[2] >> 24) & 0xFF);
  regs[9] = (uint8_t) ((accel_bias_body[2] >> 16) & 0xFF);
  regs[10] = (uint8_t) ((accel_bias_body[2] >> 8) & 0xFF);
  regs[11] = (uint8_t) (accel_bias_body[2] & 0xFF);
  return mpu_write_mem(D_ACCEL_BIAS, 12, regs);
}

 

/*
 *  Name: dmp_set_fifo_rate
 *  Description:
 *   Set DMP output rate.
 *  Note:
 *   Only used when DMP is on.
 *  Parameters
 *   rate    Desired fifo rate (Hz).
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_set_fifo_rate(uint16_t rate) 
{
  const uint8_t regs_end[12] =
  {
  DINAFE, DINAF2, DINAAB, 0xc4, DINAAA, DINAF1, DINADF, DINADF, 0xBB, 0xAF,
      DINADF, DINADF
  };
  uint16_t div;
  uint8_t tmp[8];
   if (rate > MPU_DMP_SAMPLE_RATE)
    return -1;
   div = MPU_DMP_SAMPLE_RATE / rate - 1;
  tmp[0] = (uint8_t) ((div >> 8) & 0xFF);
  tmp[1] = (uint8_t) (div & 0xFF);
   if (mpu_write_mem(D_0_22, 2, tmp))
    return -1;
  if (mpu_write_mem(CFG_6, 12, (uint8_t *) regs_end))
    return -1;
   dmp.fifo_rate = rate;
  return 0;
}

 

/*
 *  Name: dmp_get_fifo_rate
 *  Description:
 *   Get DMP output rate.
 *  Note:
 *   Only used when DMP is on.
 *  Parameters
 *   rate    Current fifo rate (Hz).
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_get_fifo_rate(uint16_t * rate) 
{
  rate[0] = dmp.fifo_rate;
  return 0;
}

 

/*
 *  Name: dmp_set_tap_thresh
 *  Description:
 *   Set tap threshold for a specific axis.
 *  Note:
 *   Only used when DMP is on.
 *  Parameters
 *   axis    1, 2, and 4 for XYZ accel, respectively.
 *   thresh  Tap threshold, in mg/ms.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_set_tap_thresh(uint8_t axis, uint16_t thresh) 
{
  uint8_t tmp[4], accel_fsr;
  float scaled_thresh;
  uint16_t dmp_thresh, dmp_thresh_2;
  if (!(axis & TAP_XYZ) || thresh > 1600)
    return -1;
   scaled_thresh = (float)thresh / MPU_DMP_SAMPLE_RATE;
   mpu_get_accel_fsr(&accel_fsr);
  switch (accel_fsr)
    {
    case 2:
      dmp_thresh = (uint16_t) (scaled_thresh * 16384);
      
        /* dmp_thresh * 0.75 */ 
        dmp_thresh_2 = (uint16_t) (scaled_thresh * 12288);
      break;
    case 4:
      dmp_thresh = (uint16_t) (scaled_thresh * 8192);
      
        /* dmp_thresh * 0.75 */ 
        dmp_thresh_2 = (uint16_t) (scaled_thresh * 6144);
      break;
    case 8:
      dmp_thresh = (uint16_t) (scaled_thresh * 4096);
      
        /* dmp_thresh * 0.75 */ 
        dmp_thresh_2 = (uint16_t) (scaled_thresh * 3072);
      break;
    case 16:
      dmp_thresh = (uint16_t) (scaled_thresh * 2048);
      
        /* dmp_thresh * 0.75 */ 
        dmp_thresh_2 = (uint16_t) (scaled_thresh * 1536);
      break;
    default:
      return -1;
    }
  tmp[0] = (uint8_t) (dmp_thresh >> 8);
  tmp[1] = (uint8_t) (dmp_thresh & 0xFF);
  tmp[2] = (uint8_t) (dmp_thresh_2 >> 8);
  tmp[3] = (uint8_t) (dmp_thresh_2 & 0xFF);
   if (axis & TAP_X)
    {
      if (mpu_write_mem(DMP_TAP_THX, 2, tmp))
        return -1;
      if (mpu_write_mem(D_1_36, 2, tmp + 2))
        return -1;
    }
  if (axis & TAP_Y)
    {
      if (mpu_write_mem(DMP_TAP_THY, 2, tmp))
        return -1;
      if (mpu_write_mem(D_1_40, 2, tmp + 2))
        return -1;
    }
  if (axis & TAP_Z)
    {
      if (mpu_write_mem(DMP_TAP_THZ, 2, tmp))
        return -1;
      if (mpu_write_mem(D_1_44, 2, tmp + 2))
        return -1;
    }
  return 0;
}

 

/*
 *  Name: dmp_set_tap_axes
 *  Description:
 *   Set which axes will register a tap.
 *  Note:
 *   Only used when DMP is on.
 *  Parameters
 *   axis    1, 2, and 4 for XYZ, respectively.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_set_tap_axes(uint8_t axis) 
{
  uint8_t tmp = 0;
   if (axis & TAP_X)
    tmp |= 0x30;
  if (axis & TAP_Y)
    tmp |= 0x0C;
  if (axis & TAP_Z)
    tmp |= 0x03;
  return mpu_write_mem(D_1_72, 1, &tmp);
}

 

/*
 *  Name: dmp_set_tap_count
 *  Description:
 *   Set minimum number of taps needed for an interrupt.
 *  Note:
 *   Only used when DMP is on.
 *  Parameters
 *   min_taps    Minimum consecutive taps (1-4).
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_set_tap_count(uint8_t min_taps) 
{
  uint8_t tmp;
   if (min_taps < 1)
    min_taps = 1;
  
  else if (min_taps > 4)
    min_taps = 4;
   tmp = min_taps - 1;
  return mpu_write_mem(D_1_79, 1, &tmp);
}

 

/*
 *  Name: dmp_set_tap_time
 *  Description:
 *   Set length between valid taps.
 *  Note:
 *   Only used when DMP is on.
 *  Parameters
 *   time    Milliseconds between taps.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_set_tap_time(uint16_t time) 
{
  uint16_t dmp_time;
  uint8_t tmp[2];
   dmp_time = time / (1000 / MPU_DMP_SAMPLE_RATE);
  tmp[0] = (uint8_t) (dmp_time >> 8);
  tmp[1] = (uint8_t) (dmp_time & 0xFF);
  return mpu_write_mem(DMP_TAPW_MIN, 2, tmp);
}

 

/*
 *  Name: dmp_set_tap_time_multi
 *  Description:
 *   Set max time between taps to register as a multi-tap.
 *  Note:
 *   Only used when DMP is on.
 *  Parameters
 *   time    Max milliseconds between taps.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_set_tap_time_multi(uint16_t time) 
{
  uint16_t dmp_time;
  uint8_t tmp[2];
   dmp_time = time / (1000 / MPU_DMP_SAMPLE_RATE);
  tmp[0] = (uint8_t) (dmp_time >> 8);
  tmp[1] = (uint8_t) (dmp_time & 0xFF);
  return mpu_write_mem(D_1_218, 2, tmp);
}

 

/*
 *  Name: dmp_set_shake_reject_thresh
 *  Description:
 *   Set shake rejection threshold.
 *   If the DMP detects a gyro sample larger than thresh, taps are rejected.
 *  Note:
 *   Only used when DMP is on.
 *  Parameters
 *   sf      Gyro scale factor.
 *   thresh  Gyro threshold in dps.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_set_shake_reject_thresh(int32_t sf, uint16_t thresh) 
{
  uint8_t tmp[4];
  int32_t thresh_scaled = sf / 1000 * thresh;
  tmp[0] = (uint8_t) (((int32_t) thresh_scaled >> 24) & 0xFF);
  tmp[1] = (uint8_t) (((int32_t) thresh_scaled >> 16) & 0xFF);
  tmp[2] = (uint8_t) (((int32_t) thresh_scaled >> 8) & 0xFF);
  tmp[3] = (uint8_t) ((int32_t) thresh_scaled & 0xFF);
  return mpu_write_mem(D_1_92, 4, tmp);
}

 

/*
 *  Name: dmp_set_shake_reject_time
 *  Description:
 *   Set shake rejection time.
 *   Sets the length of time that the gyro must be outside of the threshold set
 *   by @e gyro_set_shake_reject_thresh before taps are rejected. A mandatory
 *   60 ms is added to this parameter.
 *  Note:
 *   Only used when DMP is on.
 *  Parameters
 *   time    Time in milliseconds.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_set_shake_reject_time(uint16_t time) 
{
  uint8_t tmp[2];
   time /= (1000 / MPU_DMP_SAMPLE_RATE);
  tmp[0] = time >> 8;
  tmp[1] = time & 0xFF;
  return mpu_write_mem(D_1_90, 2, tmp);
}

 

/*
 *  Name: dmp_set_shake_reject_timeout
 *  Description:
 *   Set shake rejection timeout.
 *   Sets the length of time after a shake rejection that the gyro must stay
 *   inside of the threshold before taps can be detected again. A mandatory
 *   60 ms is added to this parameter.
 *  Note:
 *   Only used when DMP is on.
 *  Parameters
 *   time    Time in milliseconds.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_set_shake_reject_timeout(uint16_t time) 
{
  uint8_t tmp[2];
   time /= (1000 / MPU_DMP_SAMPLE_RATE);
  tmp[0] = time >> 8;
  tmp[1] = time & 0xFF;
  return mpu_write_mem(D_1_88, 2, tmp);
}

 

/*
 *  Name: dmp_set_shake_reject_timeout
 *  Description:
 *   Get current step count.
 *  Note:
 *   Only used when DMP is on.
 *  Parameters
 *   count   Number of steps detected.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_get_pedometer_step_count(uint32_t * count) 
{
  uint8_t tmp[4];
  if (!count)
    return -1;
   if (mpu_read_mem(D_PEDSTD_STEPCTR, 4, tmp))
    return -1;
   count[0] = ((uint32_t) tmp[0] << 24) | ((uint32_t) tmp[1] << 16) | 
    ((uint32_t) tmp[2] << 8) | tmp[3];
  return 0;
}

 

/*
 *  Name: dmp_set_pedometer_step_count
 *  Description:
 *   Overwrite current step count.
 *  Warning: This function writes to DMP memory and could potentially encounter
 *  a race condition if called while the pedometer is enabled.
 *  Note:
 *   Only used when DMP is on.
 *  Parameters
 *   count   New step count.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_set_pedometer_step_count(uint32_t count) 
{
  uint8_t tmp[4];
   tmp[0] = (uint8_t) ((count >> 24) & 0xFF);
  tmp[1] = (uint8_t) ((count >> 16) & 0xFF);
  tmp[2] = (uint8_t) ((count >> 8) & 0xFF);
  tmp[3] = (uint8_t) (count & 0xFF);
  return mpu_write_mem(D_PEDSTD_STEPCTR, 4, tmp);
}

 

/*
 *  Name: dmp_get_pedometer_walk_time
 *  Description:
 *   Get duration of walking time.
 *  Note:
 *   Only used when DMP is on.
 *  Parameters
 *   time    Walk time in milliseconds.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_get_pedometer_walk_time(uint32_t * time) 
{
  uint8_t tmp[4];
  if (!time)
    return -1;
   if (mpu_read_mem(D_PEDSTD_TIMECTR, 4, tmp))
    return -1;
   time[0] = (((uint32_t) tmp[0] << 24) | ((uint32_t) tmp[1] << 16) | 
                ((uint32_t) tmp[2] << 8) | tmp[3]) * 20;
  return 0;
}

 

/*
 *  Name: dmp_get_pedometer_walk_time
 *  Description:
 *   Overwrite current walk time.
 *  Warning: This function writes to DMP memory and could potentially encounter
 *  a race condition if called while the pedometer is enabled.
 *  Note:
 *   Only used when DMP is on.
 *  Parameters
 *   time    New walk time in milliseconds.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_set_pedometer_walk_time(uint32_t time) 
{
  uint8_t tmp[4];
   time /= 20;
   tmp[0] = (uint8_t) ((time >> 24) & 0xFF);
  tmp[1] = (uint8_t) ((time >> 16) & 0xFF);
  tmp[2] = (uint8_t) ((time >> 8) & 0xFF);
  tmp[3] = (uint8_t) (time & 0xFF);
  return mpu_write_mem(D_PEDSTD_TIMECTR, 4, tmp);
}

 

/*
 *  Name: dmp_get_pedometer_walk_time
 *  Description:
 *   Enable DMP features.
 *   The following \#define's are used in the input mask:
 *   - DMP_FEATURE_TAP
 *   - DMP_FEATURE_ANDROID_ORIENT
 *   - DMP_FEATURE_LP_QUAT
 *   - DMP_FEATURE_6X_LP_QUAT
 *   - DMP_FEATURE_GYRO_CAL
 *   - DMP_FEATURE_SEND_RAW_ACCEL
 *   - DMP_FEATURE_SEND_RAW_GYRO
 *  Note: DMP_FEATURE_LP_QUAT and DMP_FEATURE_6X_LP_QUAT are mutually
 *  exclusive.
 *  Note: DMP_FEATURE_SEND_RAW_GYRO and DMP_FEATURE_SEND_CAL_GYRO are also
 *  mutually exclusive.
 *  Warning: This function writes to DMP memory and could potentially encounter
 *  a race condition if called while the pedometer is enabled.
 *  Parameters
 *   mask    Mask of features to enable.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_enable_feature(uint16_t mask) 
{
  uint8_t tmp[10];
   
    /* TODO: All of these settings can probably be integrated into the default
     * DMP image. */ 
    
    /* Set integration scale factor. */ 
    tmp[0] = (uint8_t) ((GYRO_SF >> 24) & 0xFF);
  tmp[1] = (uint8_t) ((GYRO_SF >> 16) & 0xFF);
  tmp[2] = (uint8_t) ((GYRO_SF >> 8) & 0xFF);
  tmp[3] = (uint8_t) (GYRO_SF & 0xFF);
  mpu_write_mem(D_0_104, 4, tmp);
   
    /* Send sensor data to the FIFO. */ 
    tmp[0] = 0xA3;
  if (mask & DMP_FEATURE_SEND_RAW_ACCEL)
    
    {
      tmp[1] = 0xC0;
      tmp[2] = 0xC8;
      tmp[3] = 0xC2;
    }
  
  else
    
    {
      tmp[1] = 0xA3;
      tmp[2] = 0xA3;
      tmp[3] = 0xA3;
    }
  if (mask & DMP_FEATURE_SEND_ANY_GYRO)
    
    {
      tmp[4] = 0xC4;
      tmp[5] = 0xCC;
      tmp[6] = 0xC6;
    }
  
  else
    
    {
      tmp[4] = 0xA3;
      tmp[5] = 0xA3;
      tmp[6] = 0xA3;
    }
  tmp[7] = 0xA3;
  tmp[8] = 0xA3;
  tmp[9] = 0xA3;
  mpu_write_mem(CFG_15, 10, tmp);
   
    /* Send gesture data to the FIFO. */ 
    if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
    tmp[0] = DINA20;
  
  else
    tmp[0] = 0xD8;
  mpu_write_mem(CFG_27, 1, tmp);
   if (mask & DMP_FEATURE_GYRO_CAL)
    dmp_enable_gyro_cal(1);
  
  else
    dmp_enable_gyro_cal(0);
   if (mask & DMP_FEATURE_SEND_ANY_GYRO)
    
    {
      if (mask & DMP_FEATURE_SEND_CAL_GYRO)
        
        {
          tmp[0] = 0xB2;
          tmp[1] = 0x8B;
          tmp[2] = 0xB6;
          tmp[3] = 0x9B;
        }
      
      else
        
        {
          tmp[0] = DINAC0;
          tmp[1] = DINA80;
          tmp[2] = DINAC2;
          tmp[3] = DINA90;
        }
      mpu_write_mem(CFG_GYRO_RAW_DATA, 4, tmp);
    }
   if (mask & DMP_FEATURE_TAP)
    
    {
       
        /* Enable tap. */ 
        tmp[0] = 0xF8;
      mpu_write_mem(CFG_20, 1, tmp);
      dmp_set_tap_thresh(TAP_XYZ, 250);
      dmp_set_tap_axes(TAP_XYZ);
      dmp_set_tap_count(1);
      dmp_set_tap_time(100);
      dmp_set_tap_time_multi(500);
       dmp_set_shake_reject_thresh(GYRO_SF, 200);
      dmp_set_shake_reject_time(40);
      dmp_set_shake_reject_timeout(10);
    }
  
  else
    
    {
      tmp[0] = 0xD8;
      mpu_write_mem(CFG_20, 1, tmp);
    }
   if (mask & DMP_FEATURE_ANDROID_ORIENT)
    tmp[0] = 0xD9;
  
  else
    tmp[0] = 0xD8;
   mpu_write_mem(CFG_ANDROID_ORIENT_INT, 1, tmp);
   if (mask & DMP_FEATURE_LP_QUAT)
    dmp_enable_lp_quat(1);
  
  else
    dmp_enable_lp_quat(0);
   if (mask & DMP_FEATURE_6X_LP_QUAT)
    dmp_enable_6x_lp_quat(1);
  
  else
    dmp_enable_6x_lp_quat(0);
   
    /* Pedometer is always enabled. */ 
    dmp.feature_mask = mask | DMP_FEATURE_PEDOMETER;
  mpu_reset_fifo();
   dmp.packet_length = 0;
  if (mask & DMP_FEATURE_SEND_RAW_ACCEL)
    dmp.packet_length += 6;
  if (mask & DMP_FEATURE_SEND_ANY_GYRO)
    dmp.packet_length += 6;
  if (mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT))
    dmp.packet_length += 16;
  if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
    dmp.packet_length += 4;
   return 0;
}

 

/*
 *  Name: dmp_get_pedometer_walk_time
 *  Description:
 *   Get list of currently enabled DMP features.
 *  Parameters
 *   Mask of enabled features.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_get_enabled_features(uint16_t * mask) 
{
  mask[0] = dmp.feature_mask;
  return 0;
}

 

/*
 *  Name: dmp_enable_gyro_cal
 *  Description:
 *   Calibrate the gyro data in the DMP.
 *   After eight seconds of no motion, the DMP will compute gyro biases and
 *   subtract them from the quaternion output. If @e dmp_enable_feature is
 *   called with @e DMP_FEATURE_SEND_CAL_GYRO, the biases will also be
 *   subtracted from the gyro output.
 *  Parameters
 *   enable  1 to enable gyro calibration.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_enable_gyro_cal(uint8_t enable) 
{
  if (enable)
    
    {
      uint8_t regs[9] =
      {
      0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d
	  };
      return mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
    }
  else
    
    {
      uint8_t regs[9] =
      {
      0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7
	  };
      return mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
    }
}

 

/*
 *  Name: dmp_enable_lp_quat
 *  Description:
 *   Generate 3-axis quaternions from the DMP.
 *   In this driver, the 3-axis and 6-axis DMP quaternion features are mutually
 *   exclusive.
 *  Parameters
 *   enable  1 to enable 3-axis quaternion.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_enable_lp_quat(uint8_t enable) 
{
  uint8_t regs[4];
  if (enable)
    {
      regs[0] = DINBC0;
      regs[1] = DINBC2;
      regs[2] = DINBC4;
      regs[3] = DINBC6;
    }
  
  else
    memset(regs, 0x8B, 4);
   mpu_write_mem(CFG_LP_QUAT, 4, regs);
   return mpu_reset_fifo();
}

 

/*
 *  Name: dmp_enable_lp_quat
 *  Description:
 *   Generate 6-axis quaternions from the DMP.
 *   In this driver, the 3-axis and 6-axis DMP quaternion features are mutually
 *   exclusive.
 *  Parameters
 *   enable  1 to enable 6-axis quaternion.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_enable_6x_lp_quat(uint8_t enable) 
{
  uint8_t regs[4];
  if (enable)
    {
      regs[0] = DINA20;
      regs[1] = DINA28;
      regs[2] = DINA30;
      regs[3] = DINA38;
    }
  else
    memset(regs, 0xA3, 4);
   mpu_write_mem(CFG_8, 4, regs);
   return mpu_reset_fifo();
}

 

/*
 *  Name: dmp_enable_lp_quat
 *  Description:
 *   Decode the four-byte gesture data and execute any callbacks.
 *  Parameters
 *   gesture Gesture data from DMP packet.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
static int decode_gesture(uint8_t * gesture) 
{
  uint8_t tap, android_orient;
   android_orient = gesture[3] & 0xC0;
  tap = 0x3F & gesture[3];
   if (gesture[1] & INT_SRC_TAP)
    {
      uint8_t direction, count;
      direction = tap >> 3;
      count = (tap % 8) + 1;
      if (dmp.tap_cb)
        dmp.tap_cb(direction, count);
    }
   if (gesture[1] & INT_SRC_ANDROID_ORIENT)
    {
      if (dmp.android_orient_cb)
        dmp.android_orient_cb(android_orient >> 6);
    }
   return 0;
}

 

/*
 *  Name: dmp_set_interrupt_mode
 *  Description:
 *   Specify when a DMP interrupt should occur.
 *   A DMP interrupt can be configured to trigger on either of the two
 *   conditions below:
 *   a. One FIFO period has elapsed (set by @e mpu_set_sample_rate).
 *   b. A tap event has been detected.
 *  Parameters
 *   mode DMP_INT_GESTURE or DMP_INT_CONTINUOUS.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_set_interrupt_mode(uint8_t mode) 
{
  const uint8_t regs_continuous[11] = 
  {
  0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9
  };
  const uint8_t regs_gesture[11] = 
  {
  0xda, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0xda, 0xb4, 0xda
  };
   switch (mode)
    
    {
    case DMP_INT_CONTINUOUS:
      return mpu_write_mem(CFG_FIFO_ON_EVENT, 11, (uint8_t *) regs_continuous);
    case DMP_INT_GESTURE:
      return mpu_write_mem(CFG_FIFO_ON_EVENT, 11, (uint8_t *) regs_gesture);
    default:
      return -1;
    }
}

 

/**
 *  @brief      Get one packet from the FIFO.
 *  If @e sensors does not contain a particular sensor, disregard the data
 *  returned to that pointer.
 *  \n @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_WXYZ_QUAT
 *  \n If the FIFO has no new data, @e sensors will be zero.
 *  \n If the FIFO is disabled, @e sensors will be zero and this function will
 *  return a non-zero error code.
 *  @param[out] gyro        Gyro data in hardware units.
 *  @param[out] accel       Accel data in hardware units.
 *  @param[out] quat        3-axis quaternion data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds.
 *  @param[out] sensors     Mask of sensors read from FIFO.
 *  @param[out] more        Number of remaining packets.
 *  @return     0 if successful.
 */ 
int dmp_read_fifo(int16_t * gyro, int16_t * accel, int32_t * quat,
                  uint32_t * timestamp, int16_t * sensors, uint8_t * more) 
{
  uint8_t fifo_data[MAX_PACKET_LENGTH];
  uint8_t ii = 0;
   
    /* TODO: sensors[0] only changes when dmp_enable_feature is called. We can
     * cache this value and save some cycles. */ 
    sensors[0] = 0;
   
    /* Get a packet. */ 
    if (mpu_read_fifo_stream(dmp.packet_length, fifo_data, more))
    return -1;
   
    /* Parse DMP packet. */ 
    if (dmp.feature_mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT))
    {
      
#ifdef FIFO_CORRUPTION_CHECK
        int32_t quat_q14[4], quat_mag_sq;
      
#endif
        quat[0] =
        ((int32_t) fifo_data[0] << 24) | ((int32_t) fifo_data[1] << 16) |       		((int32_t) fifo_data[2] << 8) | fifo_data[3];
      		quat[1] =
        ((int32_t) fifo_data[4] << 24) | ((int32_t) fifo_data[5] << 16) |
        ((int32_t) fifo_data[6] << 8) | fifo_data[7];
      		quat[2] =
        ((int32_t) fifo_data[8] << 24) | ((int32_t) fifo_data[9] << 16) |
        ((int32_t) fifo_data[10] << 8) | fifo_data[11];
      		quat[3] =
        ((int32_t) fifo_data[12] << 24) | ((int32_t) fifo_data[13] << 16) |
        ((int32_t) fifo_data[14] << 8) | fifo_data[15];
      ii += 16;
      
#ifdef FIFO_CORRUPTION_CHECK
        /* We can detect a corrupted FIFO by monitoring the quaternion data
         * and ensuring that the magnitude is always normalized to one. This
         * shouldn't happen in normal operation, but if an I2C error occurs,
         * the FIFO reads might become misaligned.  Let's start by scaling
         * down the quaternion data to avoid int32_t int32_t math. */ 
        quat_q14[0] = quat[0] >> 16;
      quat_q14[1] = quat[1] >> 16;
      quat_q14[2] = quat[2] >> 16;
      quat_q14[3] = quat[3] >> 16;
      quat_mag_sq =
        quat_q14[0] * quat_q14[0] + quat_q14[1] * quat_q14[1] +
        quat_q14[2] * quat_q14[2] + quat_q14[3] * quat_q14[3];
      if ((quat_mag_sq < QUAT_MAG_SQ_MIN) || (quat_mag_sq > QUAT_MAG_SQ_MAX))
        {
          
            /* Quaternion is outside of the acceptable threshold. */ 
            mpu_reset_fifo();
          sensors[0] = 0;
          return -1;
        }
      sensors[0] |= INV_WXYZ_QUAT;
      
#endif
    }
   if (dmp.feature_mask & DMP_FEATURE_SEND_RAW_ACCEL)
    {
      accel[0] = ((int16_t) fifo_data[ii + 0] << 8) | fifo_data[ii + 1];
      accel[1] = ((int16_t) fifo_data[ii + 2] << 8) | fifo_data[ii + 3];
      accel[2] = ((int16_t) fifo_data[ii + 4] << 8) | fifo_data[ii + 5];
      ii += 6;
      sensors[0] |= INV_XYZ_ACCEL;
    }
   if (dmp.feature_mask & DMP_FEATURE_SEND_ANY_GYRO)
    {
      gyro[0] = ((int16_t) fifo_data[ii + 0] << 8) | fifo_data[ii + 1];
      gyro[1] = ((int16_t) fifo_data[ii + 2] << 8) | fifo_data[ii + 3];
      gyro[2] = ((int16_t) fifo_data[ii + 4] << 8) | fifo_data[ii + 5];
      ii += 6;
      sensors[0] |= INV_XYZ_GYRO;
    }
   
    /* Gesture data is at the end of the DMP packet. Parse it and call the
     * gesture callbacks (if registered). */ 
    if (dmp.feature_mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
    decode_gesture(fifo_data + ii);
   get_ms(timestamp);
  return 0;
}

 

/**
 *  @brief      Register a function to be executed on a tap event.
 *  The tap direction is represented by one of the following:
 *  \n TAP_X_UP
 *  \n TAP_X_DOWN
 *  \n TAP_Y_UP
 *  \n TAP_Y_DOWN
 *  \n TAP_Z_UP
 *  \n TAP_Z_DOWN
 *  @param[in]  func    Callback function.
 *  @return     0 if successful.
 */ 
int dmp_register_tap_cb(void (*func) (uint8_t, uint8_t)) 
{
  dmp.tap_cb = func;
  return 0;
}

 

/**
 *  @brief      Register a function to be executed on a android orientation event.
 *  @param[in]  func    Callback function.
 *  @return     0 if successful.
 */ 
int dmp_register_android_orient_cb(void (*func) (uint8_t)) 
{
  dmp.android_orient_cb = func;
  return 0;
}

 

/**
 *  @}
 */ 
  
