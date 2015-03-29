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

#include <nuttx/kmalloc.h>

#include "nuttx/sensors/inv_mpu.h"

#include "inv_mpu_dmp.h"
#include "inv_mpu_dmp_key.h"
#include "inv_mpu_dmp_map.h"
#include "inv_mpu_dmp_firmware.h"


/*******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/ 

#define INT_SRC_TAP             (0x01)
#define INT_SRC_ANDROID_ORIENT  (0x08)

#define DMP_FEATURE_SEND_ANY_GYRO   (DMP_FEATURE_SEND_RAW_GYRO | \
                                     DMP_FEATURE_SEND_CAL_GYRO)  
#define MAX_PACKET_LENGTH   (32)

#define GYRO_SF             (46850825LL * 200 / DMP_SAMPLE_RATE)

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
    void (*tap_cb) (struct dmp_s * dmp, uint8_t count, uint8_t direction);
    void (*android_orient_cb) (struct dmp_s * dmp, uint8_t orientation);
    uint16_t orient;
    uint16_t feature_mask;
    uint16_t fifo_rate;
    uint8_t packet_length;
    struct mpu_inst_s* inst;
};


/****************************************************************************
 * Public Data
 ****************************************************************************/ 


/****************************************************************************
 * Private Functions
 ****************************************************************************/ 

inline int dmp_write_32( struct dmp_s* dmp, int reg, uint32_t val32)
{
    uint8_t buf[4];
    buf[0] = (uint8_t) ((val32 >> 24) & 0xFF);
    buf[1] = (uint8_t) ((val32 >> 16) & 0xFF);
    buf[2] = (uint8_t) ((val32 >>  8) & 0xFF);
    buf[3] = (uint8_t) ((val32 >>  0) & 0xFF);

    if (mpu_write_mem(dmp->inst, reg, 4, buf) < 0 )
        return -1;

    return 0;
}

inline int dmp_read_32( struct dmp_s* dmp, int reg, uint32_t *val32)
{
    uint8_t buf[4];

    if (mpu_read_mem(dmp->inst, reg, 4, buf) < 0 )
        return -1;

    *val32 = ((uint32_t) buf[0] << 24) | \
             ((uint32_t) buf[1] << 16) | \
             ((uint32_t) buf[2] <<  8) | \
             ((uint32_t) buf[3] <<  0) ;

    return 0;
}


inline int dmp_write_16( struct dmp_s* dmp, int reg, uint16_t val16)
{ 
    uint8_t buf[2];
    buf[0] = (uint8_t) ((val16 >>  8) & 0xFF);
    buf[1] = (uint8_t) ((val16 >>  0) & 0xFF);
    if (mpu_write_mem(dmp->inst, reg, 2, buf) < 0 )
        return -1;
    return 0;
}

inline int dmp_write_3bytes( struct dmp_s* dmp, int reg, uint8_t* buf)
{
    if (mpu_write_mem(dmp->inst, reg, 3, buf) < 0 )
        return -1;
    return 0;
}

inline int dmp_write_8( struct dmp_s* dmp, int reg, uint8_t val)
{
    if (mpu_write_mem(dmp->inst, reg, 1, &val) < 0 )
        return -1;
    return 0;
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
static int decode_gesture(struct dmp_s* dmp, uint8_t * gesture) 
{
    uint8_t tap;
    uint8_t android_orient;

    android_orient = gesture[3] & 0xC0;
    tap = 0x3F & gesture[3];

    if (gesture[1] & INT_SRC_TAP)
    {
        uint8_t direction, count;
        direction = tap >> 3;
        count = (tap % 8) + 1;
        if (dmp->tap_cb)
            dmp->tap_cb(dmp, direction, count);
    }

    if (gesture[1] & INT_SRC_ANDROID_ORIENT)
    {
        if (dmp->android_orient_cb)
            dmp->android_orient_cb(dmp,android_orient >> 6);
    }

    return 0;
}


/*******************************************************************************
 * Private Data
 ******************************************************************************/ 


/****************************************************************************
 * Public Functions
 ****************************************************************************/ 

struct dmp_s* dmp_init(struct mpu_inst_s* inst)
{
  struct dmp_s* dmp;
  const struct dmp_firmware_s* firm = dmp_firmware;

  dmp = (FAR struct dmp_s *)kmm_zalloc(sizeof(struct dmp_s));
  if (!dmp)
    {
      sndbg("Failed to allocate the device structure!\n");
      return NULL;
    }

  memset(dmp,0,sizeof(*dmp));
  dmp->inst = inst;

  if ( mpu_load_firmware( inst, firm->size, firm->data, firm->start_addr, 
                          firm->sample_rate) < 0 )
    {
      sndbg("Failed to allocate the device structure!\n");
      free(dmp);
      return NULL;
    }


  return dmp;
    
}

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
int dmp_set_orientation(struct dmp_s* dmp, uint16_t orient) 
{
    uint8_t gyro_regs[3];
    uint8_t accel_regs[3];

    const uint8_t gyro_axes[3] = { 0x4C, 0xCD, 0x6C };
    const uint8_t accel_axes[3] = { 0x0C, 0xC9, 0x2C };

    const uint8_t gyro_sign[3] = { 0x36, 0x56, 0x76 };
    const uint8_t accel_sign[3] = { 0x26, 0x46, 0x66 };

    /* Chip-to-body, axes only. */ 

    gyro_regs[0] = gyro_axes[orient & 3];
    gyro_regs[1] = gyro_axes[(orient >> 3) & 3];
    gyro_regs[2] = gyro_axes[(orient >> 6) & 3];
    if (dmp_write_3bytes(dmp, KEY_FCFG_1, gyro_regs) < 0 )
        return -1;

    accel_regs[0] = accel_axes[orient & 3];
    accel_regs[1] = accel_axes[(orient >> 3) & 3];
    accel_regs[2] = accel_axes[(orient >> 6) & 3];
    if (dmp_write_3bytes(dmp, KEY_FCFG_2, accel_regs) < 0 )
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

    if (dmp_write_3bytes(dmp, KEY_FCFG_3, gyro_regs) < 0 )
        return -1;

    if (dmp_write_3bytes(dmp, KEY_FCFG_7, accel_regs) < 0 )
        return -1;

    dmp->orient = orient;

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
int dmp_set_gyro_bias(struct dmp_s* dmp, int32_t * bias) 
{
    int32_t gyro_bias_body[3];

    gyro_bias_body[0] = bias[dmp->orient & 3];
    if (dmp->orient & 4)
        gyro_bias_body[0] *= -1;

    gyro_bias_body[1] = bias[(dmp->orient >> 3) & 3];
    if (dmp->orient & 0x20)
        gyro_bias_body[1] *= -1;

    gyro_bias_body[2] = bias[(dmp->orient >> 6) & 3];
    if (dmp->orient & 0x100)
        gyro_bias_body[2] *= -1;

#ifdef EMPL_NO_64BIT
    gyro_bias_body[0] = (((float)gyro_bias_body[0] * GYRO_SF) / 1073741824.f);
    gyro_bias_body[1] = (((float)gyro_bias_body[1] * GYRO_SF) / 1073741824.f);
    gyro_bias_body[2] = (((float)gyro_bias_body[2] * GYRO_SF) / 1073741824.f);
#else
    gyro_bias_body[0] = (((int64_t)gyro_bias_body[0] * GYRO_SF) >> 30);
    gyro_bias_body[1] = (((int64_t)gyro_bias_body[1] * GYRO_SF) >> 30);
    gyro_bias_body[2] = (((int64_t)gyro_bias_body[2] * GYRO_SF) >> 30);
#endif

    if (dmp_write_32(dmp, DMP_D_EXT_GYRO_BIAS_X, gyro_bias_body[0]) < 0 )
        return -1;

    if (dmp_write_32(dmp, DMP_D_EXT_GYRO_BIAS_Y, gyro_bias_body[1]) < 0 )
        return -1;

    if (dmp_write_32(dmp, DMP_D_EXT_GYRO_BIAS_Z, gyro_bias_body[2]) < 0 )
        return -1;

    return 0;
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
int dmp_set_accel_bias(struct dmp_s* dmp, int32_t * bias) 
{
    int32_t accel_bias_body[3];
    uint8_t regs[12];
    int32_t accel_sf;
    uint16_t accel_sens;

    if ( mpu_get_accel_sensibilty(dmp->inst,&accel_sens) < 0 )
        return -1;

    accel_sf = (int64_t) accel_sens << 15;
    //__no_operation();

    accel_bias_body[0] = bias[dmp->orient & 3];
    if (dmp->orient & 4)
        accel_bias_body[0] *= -1;

    accel_bias_body[1] = bias[(dmp->orient >> 3) & 3];
    if (dmp->orient & 0x20)
        accel_bias_body[1] *= -1;

    accel_bias_body[2] = bias[(dmp->orient >> 6) & 3];
    if (dmp->orient & 0x100)
        accel_bias_body[2] *= -1;

#ifdef EMPL_NO_64BIT
    accel_bias_body[0] = (((float)accel_bias_body[0]*accel_sf) / 1073741824.f);
    accel_bias_body[1] = (((float)accel_bias_body[1]*accel_sf) / 1073741824.f);
    accel_bias_body[2] = (((float)accel_bias_body[2]*accel_sf) / 1073741824.f);
#else
    accel_bias_body[0] = (((int64_t)accel_bias_body[0] * accel_sf) >> 30);
    accel_bias_body[1] = (((int64_t)accel_bias_body[1] * accel_sf) >> 30);
    accel_bias_body[2] = (((int64_t)accel_bias_body[2] * accel_sf) >> 30);
#endif

    regs[ 0] = (uint8_t) ((accel_bias_body[0] >> 24) & 0xFF);
    regs[ 1] = (uint8_t) ((accel_bias_body[0] >> 16) & 0xFF);
    regs[ 2] = (uint8_t) ((accel_bias_body[0] >> 8) & 0xFF);
    regs[ 3] = (uint8_t) (accel_bias_body[0] & 0xFF);
    regs[ 4] = (uint8_t) ((accel_bias_body[1] >> 24) & 0xFF);
    regs[ 5] = (uint8_t) ((accel_bias_body[1] >> 16) & 0xFF);
    regs[ 6] = (uint8_t) ((accel_bias_body[1] >> 8) & 0xFF);
    regs[ 7] = (uint8_t) (accel_bias_body[1] & 0xFF);
    regs[ 8] = (uint8_t) ((accel_bias_body[2] >> 24) & 0xFF);
    regs[ 9] = (uint8_t) ((accel_bias_body[2] >> 16) & 0xFF);
    regs[10] = (uint8_t) ((accel_bias_body[2] >> 8) & 0xFF);
    regs[11] = (uint8_t) (accel_bias_body[2] & 0xFF);

    if ( mpu_write_mem(dmp->inst,DMP_D_ACCEL_BIAS, 12, regs) < 0 )
        return -1;
    return 0;
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
int dmp_set_fifo_rate(struct dmp_s* dmp, uint16_t rate) 
{
    const uint8_t regs_end[12] =
    {
        0xFE, 0xF2, 0xAB, 0xc4, 0xAA, 0xF1, 0xDF, 0xDF, 0xBB, 
        0xAF, 0xDF, 0xDF
    };

    if (rate > DMP_SAMPLE_RATE)
        return -1;

    if (dmp_write_16(dmp, DMP_D_0_22, DMP_SAMPLE_RATE / rate - 1) < 0 )
        return -1;

    if (mpu_write_mem(dmp->inst, DMP_CFG_6, 12, (uint8_t *) regs_end) < 0 )
        return -1;

    dmp->fifo_rate = rate;

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
int dmp_get_fifo_rate(struct dmp_s* dmp, uint16_t * rate) 
{
    rate[0] = dmp->fifo_rate;
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
int dmp_set_tap_thresh(struct dmp_s* dmp, uint8_t axis, uint16_t thresh) 
{
    uint8_t accel_fsr;
    float scaled_thresh;
    uint16_t dmp_thresh;
    uint16_t dmp_thresh_2;

    if (!(axis & TAP_XYZ) || thresh > 1600)
        return -1;

    scaled_thresh = (float)thresh / DMP_SAMPLE_RATE;

    if ( mpu_get_accel_fsr(dmp->inst, &accel_fsr) < 0 )
        return -1;

    switch (accel_fsr)
    {
        case 2:
            dmp_thresh   = (uint16_t) (scaled_thresh * 16384);
            dmp_thresh_2 = (uint16_t) (scaled_thresh * 12288);
            break;
        case 4:
            dmp_thresh   = (uint16_t) (scaled_thresh * 8192);
            dmp_thresh_2 = (uint16_t) (scaled_thresh * 6144);
            break;
        case 8:
            dmp_thresh   = (uint16_t) (scaled_thresh * 4096);
            dmp_thresh_2 = (uint16_t) (scaled_thresh * 3072);
            break;
        case 16:
            dmp_thresh   = (uint16_t) (scaled_thresh * 2048);
            dmp_thresh_2 = (uint16_t) (scaled_thresh * 1536);
            break;
        default:
            return -1;
    }

    if (axis & TAP_X)
    {
        if (dmp_write_16(dmp, DMP_TAP_THX, dmp_thresh) < 0 )
            return -1;
        if (dmp_write_16(dmp, DMP_D_1_36, dmp_thresh_2) < 0 )
            return -1;
    }

    if (axis & TAP_Y)
    {
        if (dmp_write_16(dmp, DMP_TAP_THY, dmp_thresh) < 0 )
            return -1;
        if (dmp_write_16(dmp, DMP_D_1_40, dmp_thresh_2) < 0 )
            return -1;
    }

    if (axis & TAP_Z)
    {
        if (dmp_write_16(dmp, DMP_TAP_THZ, dmp_thresh) < 0 )
            return -1;
        if (dmp_write_16(dmp, DMP_D_1_44, dmp_thresh_2) < 0 )
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
int dmp_set_tap_axes(struct dmp_s* dmp, uint8_t axis) 
{
    uint8_t regval = 0;
    
    if (axis & TAP_X)
        regval |= 0x30;
    if (axis & TAP_Y)
        regval |= 0x0C;
    if (axis & TAP_Z)
        regval |= 0x03;
    return dmp_write_8(dmp, DMP_D_1_72, regval);
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
int dmp_set_tap_count(struct dmp_s* dmp, uint8_t min_taps) 
{
    
    if (min_taps < 1)
        min_taps = 1;
    else if (min_taps > 4)
        min_taps = 4;

    return dmp_write_8(dmp, DMP_D_1_79, min_taps - 1);
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
int dmp_set_tap_time(struct dmp_s* dmp, uint16_t time) 
{
    uint16_t dmp_time;

    dmp_time = time / (1000 / DMP_SAMPLE_RATE);

    return dmp_write_16(dmp, DMP_TAPW_MIN, dmp_time);
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
int dmp_set_tap_time_multi(struct dmp_s* dmp, uint16_t time) 
{
    uint16_t dmp_time;

    dmp_time = time / (1000 / DMP_SAMPLE_RATE);

    return dmp_write_16(dmp, DMP_D_1_218, dmp_time);
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
int dmp_set_shake_reject_thresh(struct dmp_s* dmp, int32_t sf, uint16_t thresh) 
{
    int32_t thresh_scaled = sf / 1000 * thresh;

    return dmp_write_32(dmp, DMP_D_1_92,thresh_scaled);
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
int dmp_set_shake_reject_time(struct dmp_s* dmp, uint16_t time) 
{
    return dmp_write_16(dmp, DMP_D_1_90, time/(1000/DMP_SAMPLE_RATE) );
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
int dmp_set_shake_reject_timeout(struct dmp_s* dmp, uint16_t time) 
{
    return dmp_write_16(dmp, DMP_D_1_88, time/(1000/DMP_SAMPLE_RATE) );
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
int dmp_get_pedometer_step_count(struct dmp_s* dmp, uint32_t * count) 
{

    if (!count)
        return -1;

    return dmp_read_32(dmp, DMP_D_PEDSTD_STEPCTR, count);
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
int dmp_set_pedometer_step_count(struct dmp_s* dmp, uint32_t count) 
{
    return dmp_write_32(dmp, DMP_D_PEDSTD_STEPCTR, count);
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
int dmp_get_pedometer_walk_time(struct dmp_s* dmp, uint32_t * time) 
{

    if (!time)
        return -1;

    if ( dmp_read_32(dmp, DMP_D_PEDSTD_TIMECTR, time) < 0 );
        return -1;

    *time *= 20;

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
int dmp_set_pedometer_walk_time(struct dmp_s* dmp, uint32_t time) 
{
    return dmp_write_32(dmp, DMP_D_PEDSTD_STEPCTR, time/20);
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
int dmp_enable_feature(struct dmp_s* dmp, uint16_t mask) 
{
    uint8_t regval;
    uint8_t tmp[10];

    /* TODO: All of these settings can probably be integrated into the default
     * DMP image. */ 

    /* Set integration scale factor. */ 
    if ( dmp_write_32(dmp, DMP_D_0_104, GYRO_SF) < 0 )
        return -1;

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

    if (mpu_write_mem(dmp->inst, KEY_CFG_15, 10, tmp) < 0)
        return -1;

    /* Send gesture data to the FIFO. */ 

    if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
        regval = 0x20;
    else
        regval = 0xD8;

    if ( dmp_write_8(dmp, KEY_CFG_27, 0x20) < 0)
        return -1;


    dmp_enable_gyro_cal(dmp,(mask & DMP_FEATURE_GYRO_CAL)?1:0);

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
            tmp[0] = 0xC0;
            tmp[1] = 0x80;
            tmp[2] = 0xC2;
            tmp[3] = 0x90;
        }

        if ( mpu_write_mem(dmp->inst, DMP_CFG_GYRO_RAW_DATA, 4, tmp) < 0)
            return -1;
    }

    if (mask & DMP_FEATURE_TAP)
    {

        /* Enable tap. */ 

        if (dmp_write_8(dmp, KEY_CFG_20, 0xF8) < 0 )
            return -1;
        if (dmp_set_tap_thresh(dmp, TAP_XYZ, 250) < 0 )
            return -1;
        if (dmp_set_tap_axes(dmp, TAP_XYZ) < 0 )
            return -1;
        if (dmp_set_tap_count(dmp, 1) < 0 )
            return -1;
        if (dmp_set_tap_time(dmp, 100) < 0 )
            return -1;
        if (dmp_set_tap_time_multi(dmp, 500) < 0 )
            return -1;
        if (dmp_set_shake_reject_thresh(dmp, GYRO_SF, 200) < 0 )
            return -1;
        if (dmp_set_shake_reject_time(dmp, 40) < 0 )
            return -1;
        if (dmp_set_shake_reject_timeout(dmp, 10) < 0 )
            return -1;
    }
    else
    {
        if (dmp_write_8(dmp, KEY_CFG_20, 0xD8) < 0 )
            return -1;
    }

    if (mask & DMP_FEATURE_ANDROID_ORIENT)
        regval = 0xD9;
    else
        regval = 0xD8;

    if (dmp_write_8(dmp, KEY_CFG_ANDROID_ORIENT_INT, regval) < 0 )
        return -1;

    if (mask & DMP_FEATURE_LP_QUAT)
        regval = true;
    else
        regval = false;

    if (dmp_enable_lp_quat(dmp, regval) < 0 )
        return -1;

    if (mask & DMP_FEATURE_6X_LP_QUAT)
        regval = true;
    else
        regval = false;

    if (dmp_enable_6x_lp_quat(dmp, regval) < 0 )
        return -1;

    /* Pedometer is always enabled. */ 

    dmp->feature_mask = mask | DMP_FEATURE_PEDOMETER;
    if ( mpu_reset_fifo(dmp->inst) < 0 )
        return -1;

    dmp->packet_length = 0;

    if (mask & DMP_FEATURE_SEND_RAW_ACCEL)
        dmp->packet_length += 6;

    if (mask & DMP_FEATURE_SEND_ANY_GYRO)
        dmp->packet_length += 6;

    if (mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT))
        dmp->packet_length += 16;

    if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
        dmp->packet_length += 4;

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
int dmp_get_enabled_features(struct dmp_s* dmp, uint16_t * mask) 
{
    *mask = dmp->feature_mask;
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
int dmp_enable_gyro_cal(struct dmp_s* dmp, uint8_t enable) 
{
    const uint8_t *sel;
    const uint8_t regs_enable[9] =
    {
        0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d
    };
    const uint8_t regs_disable[9] =
    {
        0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7
    };

    if (enable)
    {
        sel = regs_enable;
    }
    else
    {
        sel = regs_disable;
    }
    return mpu_write_mem(dmp->inst, KEY_CFG_MOTION_BIAS, 9, (uint8_t *)sel);
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
int dmp_enable_lp_quat(struct dmp_s* dmp, bool enable) 
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
    {
        memset(regs, 0x8B, 4);
    }

    if ( mpu_write_mem(dmp->inst, KEY_CFG_LP_QUAT, 4, regs) < 0 )
        return -1;

    return mpu_reset_fifo(dmp->inst);
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
int dmp_enable_6x_lp_quat(struct dmp_s* dmp, bool enable) 
{
    uint8_t regs[4];
    if (enable)
    {
        regs[0] = 0x20;
        regs[1] = 0x28;
        regs[2] = 0x30;
        regs[3] = 0x38;
    }
    else
    {
        memset(regs, 0xA3, 4);
    }

    if ( mpu_write_mem(dmp->inst, KEY_CFG_LP_QUAT, 4, regs) < 0 )
        return -1;

    return mpu_reset_fifo(dmp->inst);
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
int dmp_set_interrupt_mode(struct dmp_s* dmp, uint8_t mode) 
{
    const uint8_t *sel;
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
            sel = regs_continuous;
        case DMP_INT_GESTURE:
            sel = regs_gesture;
        default:
            return -1;
    }

    return mpu_write_mem(dmp->inst, KEY_CFG_FIFO_ON_EVENT, 11, (uint8_t *) sel);
}



/*
 *  Name: dmp_read_fifo
 *  Description:
 *      Get one packet from the FIFO.
 *   If @e sensors does not contain a particular sensor, disregard the data
 *   returned to that pointer.
 *   @e sensors can contain a combination of the following flags:
 *   INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *   INV_XYZ_GYRO
 *   INV_XYZ_ACCEL
 *   INV_WXYZ_QUAT
 *   If the FIFO has no new data, @e sensors will be zero.
 *   If the FIFO is disabled, @e sensors will be zero and this function will
 *   return a non-zero error code.
 *  Parameters
 *   gyro        Gyro data in hardware units.
 *   accel       Accel data in hardware units.
 *   quat        3-axis quaternion data in hardware units.
 *   timestamp   Timestamp in milliseconds.
 *   sensors     Mask of sensors read from FIFO.
 *   more        Number of remaining packets.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_read_fifo(struct dmp_s* dmp, int16_t * gyro, int16_t * accel, 
                  int32_t * quat, struct timespec *tp, int16_t * sensors, 
                  int * more) 
{
    uint8_t fifo_data[MAX_PACKET_LENGTH];
    uint8_t ii = 0;

    /* TODO: *sensors only changes when dmp_enable_feature is called. We can
     * cache this value and save some cycles. 
     */ 

    *sensors = 0;

    /* Get a packet. */ 

    if (mpu_read_fifo_stream(dmp->inst,dmp->packet_length,fifo_data,more) < 0)
        return -1;

    /* Parse DMP packet. */ 

    if (dmp->feature_mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT))
    {

#ifdef FIFO_CORRUPTION_CHECK
        int32_t quat_q14[4], quat_mag_sq;
#endif

        quat[0] = ((int32_t) fifo_data[ 0] << 24) | \
                  ((int32_t) fifo_data[ 1] << 16) | \
                  ((int32_t) fifo_data[ 2] <<  8) | \
                  ((int32_t) fifo_data[ 3] <<  0) ;
        quat[1] = ((int32_t) fifo_data[ 4] << 24) | \
                  ((int32_t) fifo_data[ 5] << 16) | \
                  ((int32_t) fifo_data[ 6] <<  8) | \
                  ((int32_t) fifo_data[ 7] <<  0) ;
        quat[2] = ((int32_t) fifo_data[ 8] << 24) | \
                  ((int32_t) fifo_data[ 9] << 16) | \
                  ((int32_t) fifo_data[10] <<  8) | \
                  ((int32_t) fifo_data[11] <<  0) ;
        quat[3] = ((int32_t) fifo_data[12] << 24) | \
                  ((int32_t) fifo_data[13] << 16) | \
                  ((int32_t) fifo_data[14] <<  8) | \
                  ((int32_t) fifo_data[15] <<  0) ;

        ii += 16;

#ifdef FIFO_CORRUPTION_CHECK
        /* We can detect a corrupted FIFO by monitoring the quaternion data
         * and ensuring that the magnitude is always normalized to one. This
         * shouldn't happen in normal operation, but if an I2C error occurs,
         * the FIFO reads might become misaligned.  Let's start by scaling
         * down the quaternion data to avoid int32_t int32_t math. 
         */

        quat_q14[0] = quat[0] >> 16;
        quat_q14[1] = quat[1] >> 16;
        quat_q14[2] = quat[2] >> 16;
        quat_q14[3] = quat[3] >> 16;

        quat_mag_sq = quat_q14[0] * quat_q14[0] + \
                      quat_q14[1] * quat_q14[1] + \
                      quat_q14[2] * quat_q14[2] + \
                      quat_q14[3] * quat_q14[3] ;

        if ( (quat_mag_sq < QUAT_MAG_SQ_MIN) || \
             (quat_mag_sq > QUAT_MAG_SQ_MAX)    )
        {

            /* Quaternion is outside of the acceptable threshold. */ 

            *sensors = 0;

            mpu_reset_fifo(dmp->inst);

            return -1;
        }

        *sensors |= DMP_WXYZ_QUAT;

#endif
    }

    if (dmp->feature_mask & DMP_FEATURE_SEND_RAW_ACCEL)
    {
        accel[0] = ((int16_t) fifo_data[ii + 0] << 8) | fifo_data[ii + 1];
        accel[1] = ((int16_t) fifo_data[ii + 2] << 8) | fifo_data[ii + 3];
        accel[2] = ((int16_t) fifo_data[ii + 4] << 8) | fifo_data[ii + 5];
        ii += 6;
        *sensors |= MPU_XYZ_ACCEL;
    }

    if (dmp->feature_mask & DMP_FEATURE_SEND_ANY_GYRO)
    {
        gyro[0] = ((int16_t) fifo_data[ii + 0] << 8) | fifo_data[ii + 1];
        gyro[1] = ((int16_t) fifo_data[ii + 2] << 8) | fifo_data[ii + 3];
        gyro[2] = ((int16_t) fifo_data[ii + 4] << 8) | fifo_data[ii + 5];
        ii += 6;
        *sensors |= MPU_XYZ_GYRO;
    }

    /* Gesture data is at the end of the DMP packet. Parse it and call the
     * gesture callbacks (if registered). 
     */
    if (dmp->feature_mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
        decode_gesture(dmp,fifo_data + ii);

    if ( tp != NULL ) 
    { 
        if ( clock_gettime(CLOCK_REALTIME,tp) < 0 )
        {
            tp->tv_sec  = 0;
            tp->tv_nsec = 0;
        }
    }

    return 0;
}



/*
 *  Name: dmp_read_fifo
 *  Description:
 *   Register a function to be executed on a tap event.
 *   The tap direction is represented by one of the following:
 *   TAP_X_UP
 *   TAP_X_DOWN
 *   TAP_Y_UP
 *   TAP_Y_DOWN
 *   TAP_Z_UP
 *   TAP_Z_DOWN
 *  Parameters
 *   func    Callback function.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_register_tap_cb(struct dmp_s* dmp, 
                        void (*func) (struct dmp_s*, uint8_t, uint8_t)) 
{
    dmp->tap_cb = func;
    return 0;
}



/*
 *  Name: dmp_read_fifo
 *  Description:
 *   Register a function to be executed on a android orientation event.
 *  Parameters
 *   func    Callback function.
 *  Return:
 *   0 if successful, negative value in case of error.
 */ 
int dmp_register_android_orient_cb(struct dmp_s* dmp, 
                                   void (*func) (struct dmp_s*, uint8_t)) 
{
    dmp->android_orient_cb = func;
    return 0;
}


