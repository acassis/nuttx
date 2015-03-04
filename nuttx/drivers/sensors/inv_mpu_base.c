/****************************************************************************
 * drivers/sensors/inv_mpu_base.c
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
#include "inv_mpu.h"


/*******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

#if ( ( defined CONFIG_SENSOR_MPU6050 ) || \
      ( defined CONFIG_SENSOR_MPU9150 ) || \
      ( defined CONFIG_SENSOR_MPU6500 ) || \
      ( defined CONFIG_SENSOR_MPU9250 ) )

/* Manage equivalence *********************************************************/

#if defined CONFIG_SENSOR_MPU9150
/* is equivalent to.. */
#   define CONFIG_SENSOR_MPU6050
#   define CONFIG_SENSOR_AK8975_SECONDARY
#endif                         

#if defined CONFIG_SENSOR_MPU9250        
/* is equivalent to.. */
#   define CONFIG_SENSOR_MPU6500
#   define CONFIG_SENSOR_AK8963_SECONDARY
#endif                         

/* Configuration **************************************************************/

#if (defined CONFIG_SENSOR_AK8975_SECONDARY )
#   define CONFIG_SENSOR_AK89XX_SECONDARY
#   include "inv_ak8975_reg.h"
#endif

#if (defined CONFIG_SENSOR_AK8963_SECONDARY )
#   define CONFIG_SENSOR_AK89XX_SECONDARY
#   include "inv_ak8963_reg.h"
#endif

#if defined CONFIG_SENSOR_MPU6500
#   include "inv_mpu6500_reg.h"
#endif

#if defined CONFIG_SENSOR_MPU6050
#   include "inv_mpu6050_reg.h"
#endif

/* Debug **********************************************************************/
/* CONFIG_DEBUG_I2C + CONFIG_DEBUG enables general I2C debug output. */

#ifdef CONFIG_INVENSENSE_DEBUG
#  define invdbg dbg
#  define invvdbg vdbg
#else
#  define invdbg(x...)
#  define invvdbg(x...)
#endif

/* BIT MASK *******************************************************************/

#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)


#define MAX_PACKET_LENGTH (12)
#define HWST_MAX_PACKET_LENGTH (512)


/*******************************************************************************
 * Private Types
 ******************************************************************************/

/* Filter configurations. */

enum mpu_lpf_e {
    MPU_LPF_256HZ_NOLPF2 = 0,
    MPU_LPF_188HZ,
    MPU_LPF_98HZ,
    MPU_LPF_42HZ,
    MPU_LPF_20HZ,
    MPU_LPF_10HZ,
    MPU_LPF_5HZ,
    MPU_LPF_2100HZ_NOLPF,
    MPU_LPF_NBR
};

/* Full scale ranges. */

enum mpu_gyro_fsr_e {
    MPU_GYRO_FSR_250DPS = 0,
    MPU_GYRO_FSR_500DPS,
    MPU_GYRO_FSR_1000DPS,
    MPU_GYRO_FSR_2000DPS,
    MPU_GYRO_FSR_NBR
};

/* Clock sources. */

enum mpu_clk_sel_e {
    MPU_CLK_INTERNAL = 0,
    MPU_CLK_PLL,
    MPU_CLK_NBR
};

/* Full scale ranges. */

enum mpu_acc_fsr_e {
    MPU_ACC_FSR_2G = 0,
    MPU_ACC_FSR_4G,
    MPU_ACC_FSR_8G,
    MPU_ACC_FSR_16G,
    MPU_ACC_FSR_NBR
};

/* Low-power accel wakeup rates. */

enum mpu_lpa_rate_e {
    MPU_LPA_0_3125HZ,
    MPU_LPA_0_625HZ,
    MPU_LPA_1_25HZ,
    MPU_LPA_2_5HZ,
    MPU_LPA_5HZ,
    MPU_LPA_10HZ,
    MPU_LPA_20HZ,
    MPU_LPA_40HZ,
    MPU_LPA_80HZ,
    MPU_LPA_160HZ,
    MPU_LPA_320HZ,
    MPU_LPA_640HZ
    MPU_LPA_NBR
};


/* pnb I think that is not needed */
/* When entering motion interrupt mode, the driver keeps track of the
 * previous state so that it can be restored at a later time.
 * TODO: This is tacky. Fix it.
 */
struct motion_int_cache_s {
    uint16_t gyro_fsr;
    uint8_t accel_fsr;
    uint16_t lpf;
    uint16_t sample_rate;
    uint8_t fifo_sensors;
    bool sensors_on;
    bool dmp_on;
};

/* Debug **********************************************************************/
/* CONFIG_DEBUG_I2C + CONFIG_DEBUG enables general I2C debug output. */

/* Cached chip configuration data.
 * TODO: A lot of these can be handled with a bitmask.
 */
struct chip_cfg_s {
    /* Matches gyro_cfg >> 3 & 0x03 */
    uint8_t gyro_fsr;
    /* Matches accel_cfg >> 3 & 0x03 */
    uint8_t accel_fsr;
    /* Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2. */
    uint8_t sensors;
    /* Matches config register. */
    uint8_t lpf;
    uint8_t clk_src;
    /* Sample rate, NOT rate divider. */
    uint16_t sample_rate;
    /* Matches fifo_en register. */
    uint8_t fifo_enable;
    /* Matches int enable register. */
    uint8_t int_enable;
    /* 1 if devices on auxiliary I2C bus appear on the primary. */
    uint8_t bypass_mode;
    /* 1 if half-sensitivity.
     * NOTE: This doesn't belong here, but everything else in hw_s is const,
     * and this allows us to save some precious RAM.
     */
    uint8_t accel_half;
    /* 1 if device in low-power accel-only mode. */
    uint8_t lp_accel_mode;
    /* 1 if interrupts are only triggered on motion events. */
    uint8_t int_motion_only;
    struct motion_int_cache_s cache;
    /* 1 for active low interrupts. */
    uint8_t active_low_int;
    /* 1 for latched interrupts. */
    uint8_t latched_int;
    /* 1 if DMP is enabled. */
    uint8_t dmp_on;
    /* Ensures that DMP will only be loaded once. */
    uint8_t dmp_loaded;
    /* Sampling rate used when DMP is enabled. */
    uint16_t dmp_sample_rate;
#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
    /* Compass sample rate. */
    uint16_t compass_sample_rate;
    uint8_t compass_addr;
#endif
};

/* Gyro driver state variables. */
struct mpu_inst_s {
    struct mpu_lowlevel_s *     low;
    struct chip_cfg_s           chip_cfg;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int set_int_enable(struct mpu_inst_s* inst, bool enable);

#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
static int setup_compass(void);
#endif

/*******************************************************************************
 * Private Data
 ******************************************************************************/

struct mpu_inst_s g_dev_inst;

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

static inline int mpu_write(struct mpu_inst_s* inst,int reg_off,uint8_t *buf,
                            int size)
{
    return (inst)->low->write((inst)->low->priv,reg_off,buf,size);
}

static inline int mpu_read(struct mpu_inst_s* inst,int reg_off,uint8_t *buf,
                            int size)
{
    return (inst)->low->read((inst)->low->priv,reg_off,buf,size);
}

static inline void mpu_get_ms(struct timespec *tp)
{
    if ( tp != NULL ) && ( clock_gettime(CLOCK_REALTIME,tp) != OK )
    {
        tp->tv_sec  = 0;
        tp->tv_nsec = 0;
    }
}

/*******************************************************************************
 * Name: inv_mpu_set_int_enable
 *
 * Description:
 *  Enable/disable data ready interrupt.
 *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data ready
 *  interrupt is used.
 * Params
 *  inst    instance of inv_mpu driver.
 *  enable  true ot enable interruption of invensense IC.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

static int inv_mpu_set_int_enable(struct mpu_inst_s* inst, bool enable)
{
    uint8_t tmp;

    if (inst->chip_cfg.dmp_on) {
        if (enable)
            tmp = BIT_DMP_INT_EN;
        else
            tmp = 0x00;
        if (mpu_write(inst,INV_MPU_INT_ENABLE,&tmp,1) < 0 )
            return -1;
        inst->chip_cfg.int_enable = tmp;
    } else {
        if (!inst->chip_cfg.sensors)
            return -1;
        if (enable && inst->chip_cfg.int_enable)
            return 0;
        if (enable)
            tmp = BIT_DATA_RDY_EN;
        else
            tmp = 0x00;
        if (mpu_write(inst,INV_MPU_INT_ENABLE,&tmp,1) < 0 )
            return -1;
        inst->chip_cfg.int_enable = tmp;
    }
    return 0;
}

/*******************************************************************************
 * Name: inv_mpu_reg_dump
 *
 * Description:
 *  Register dump for testing.
 * Params
 *  inst    instance of inv_mpu driver.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int inv_mpu_reg_dump(struct mpu_inst_s* inst)
{
    uint8_t ii;
    uint8_t data;

    for (ii = 0; ii < INV_MPU_NUM_REG; ii++) {
        if (ii == INV_MPU_FIFO_R_W || ii == INV_MPU_MEM_R_W)
            continue;
        if (mpu_read(inst, ii, &data, 1))
            return -1;
        invdbg("%#5x: %#5x\r\n", ii, data);
    }
    return 0;
}

/*******************************************************************************
 * Name: mpu_read_reg
 *
 * Description:
 *  Read from a single register.
 *  NOTE: The memory and FIFO read/write registers cannot be accessed.
 * Params
 *  inst    instance of inv_mpu driver.
 *  reg     Register address.
 *  data    Register data.
 *
 * Return
 *  Register value.
 ******************************************************************************/

int mpu_read_reg(struct mpu_inst_s* inst, uint8_t reg, 
                 uint8_t *data)
{
    if (reg == INV_MPU_FIFO_R_W || reg == INV_MPU_MEM_R_W)
        return -1;
    if (reg >= INV_MPU_NUM_REG)
        return -1;
    return mpu_read(inst,reg,data,1);
}

/*******************************************************************************
 * Name: mpu_instantiate
 *
 * Description:
 *
 * Params
 *  low          instance of lowlevel access to invensense IC.
 *  devno        instance of invensense driver.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

struct mpu_inst_s* mpu_instantiate(struct mpu_lowlevel_s* low, int devno)
{
    struct mpu_inst_s* inst;

    /* We support only one device */

    if ( devno != 0 )
        return NULL;

    inst = &g_dev_inst;

    inst->low = low;

    if ( mpu_reset_default(inst) < 0 )
        return NULL;

    return inst;
}

/*******************************************************************************
 * Name: mpu_reset_default
 *
 * Description:
 *  Initialize hardware with initial configuration:
 *  Gyro FSR: +/- 2000DPS
 *  Accel FSR +/- 2G
 *  DLPF: 42Hz
 *  FIFO rate: 50Hz
 *  Clock source: Gyro PLL
 *  FIFO: Disabled.
 *  Data ready interrupt: Disabled, active low, unlatched.
 *
 * Params
 *  inst    instance of inv_mpu driver.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_reset_default(struct mpu_inst_s* inst)
{
    uint8_t data[6];

    /* Reset device. */

    data[0] = BIT_RESET;
    if (mpu_write(inst,INV_MPU_PWR_MGMT_1,data,1) < 0 )
        return -1;
    up_mdelay(100);

    /* Wake up chip. */

    data[0] = 0x00;
    if (mpu_write(inst,INV_MPU_PWR_MGMT_1,data,1) < 0 )
        return -1;

    inst->chip_cfg.accel_half = 0;

#ifdef CONFIG_SENSOR_MPU6500

    /* MPU6500 shares 4kB of memory between the DMP and the FIFO. Since the
     * first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
     */

    data[0] = BIT_FIFO_SIZE_1024 | 0x8;
    if (mpu_write(inst,INV_MPU_ACCEL_CFG2,data,1) < 0 )
        return -1;
#endif

    /* Set to invalid values to ensure no I2C writes are skipped. */

    inst->chip_cfg.sensors = 0xFF;
    inst->chip_cfg.gyro_fsr = 0xFF;
    inst->chip_cfg.accel_fsr = 0xFF;
    inst->chip_cfg.lpf = 0xFF;
    inst->chip_cfg.sample_rate = 0xFFFF;
    inst->chip_cfg.fifo_enable = 0xFF;
    inst->chip_cfg.bypass_mode = 0xFF;
#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
    inst->chip_cfg.compass_sample_rate = 0xFFFF;
#endif

    /* mpu_set_sensors always preserves this setting. */

    inst->chip_cfg.clk_src = INV_CLK_PLL;

    /* Handled in next call to mpu_set_bypass. */

    inst->chip_cfg.active_low_int = 1;
    inst->chip_cfg.latched_int = 0;
    inst->chip_cfg.int_motion_only = 0;
    inst->chip_cfg.lp_accel_mode = 0;
    memset(&inst->chip_cfg.cache, 0, sizeof(inst->chip_cfg.cache));
    inst->chip_cfg.dmp_on = 0;
    inst->chip_cfg.dmp_loaded = 0;
    inst->chip_cfg.dmp_sample_rate = 0;

    if (mpu_set_gyro_fsr(inst,2000))
        return -1;
    if (mpu_set_accel_fsr(inst,2))
        return -1;
    if (mpu_set_lpf(inst,42))
        return -1;
    if (mpu_set_sample_rate(inst,50))
        return -1;
    if (mpu_configure_fifo(inst,0))
        return -1;

#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
    setup_compass();
    if (mpu_set_compass_sample_rate(inst,10))
        return -1;
#else

    /* Already disabled by setup_compass. */

    if (mpu_set_bypass(inst,0))
        return -1;
#endif

    mpu_set_sensors(inst,0);
    return 0;
}

/*******************************************************************************
 * Name: mpu_lp_accel_mode
 *
 * Description:
 *  Enter low-power accel-only mode.
 *  In low-power accel mode, the chip goes to sleep and only wakes up to sample
 *  the accelerometer at one of the following frequencies:
 *  MPU6050: 1.25Hz, 5Hz, 20Hz, 40Hz
 *  MPU6500: 1.25Hz, 2.5Hz, 5Hz, 10Hz, 20Hz, 40Hz, 80Hz, 160Hz, 320Hz, 640Hz
 *  If the requested rate is not one listed above, the device will be set to
 *  the next highest rate. Requesting a rate above the maximum supported
 *  frequency will result in an error.
 *  To select a fractional wake-up frequency, round down the value passed to
 *  rate.
 * 
 * Params
 *  inst        instance of inv_mpu driver.
 *  rate        Minimum sampling rate in Hz, or zero to disable LP accel mode.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_lp_accel_mode(struct mpu_inst_s* inst, uint8_t rate)
{
    uint8_t tmp[2];

    if (!rate) {
        mpu_set_int_latched(inst,false);
        tmp[0] = 0;
        tmp[1] = BIT_STBY_XYZG;
        if (mpu_write(inst,INV_MPU_PWR_MGMT_1,tmp,2) < 0 )
            return -1;
        inst->chip_cfg.lp_accel_mode = 0;
        return 0;
    }

    /* For LP accel, we automatically configure the hardware to produce latched
     * interrupts. In LP accel mode, the hardware cycles into sleep mode before
     * it gets a chance to deassert the interrupt pin; therefore, we shift this
     * responsibility over to the MCU.
     *
     * Any register read will clear the interrupt.
     */

    mpu_set_int_latched(inst,true);

#if defined CONFIG_SENSOR_MPU6050
    if (rate == 1) 
    {
        tmp[1] = INV_LPA_1_25HZ;
        mpu_set_lpf(5);
    } else if (rate <= 5) 
    {
        tmp[1] = INV_LPA_5HZ;
        mpu_set_lpf(5);
    } else if (rate <= 20) 
    {
        tmp[1] = INV_LPA_20HZ;
        mpu_set_lpf(10);
    } else if (rate <= 40) 
    {
        tmp[1] = INV_LPA_40HZ;
        mpu_set_lpf(20);
    }
    else 
    {
        return -1;
    }

    tmp[0] = BIT_LPA_CYCLE;
    tmp[1] = (tmp[1] << 6) | BIT_STBY_XYZG;

    if (mpu_write(inst,INV_MPU_PWR_MGMT_1,tmp,2) < 0 )
        return -1;

#elif defined MPU6500

    /* Set wake frequency. */

    if (rate == 1)
        tmp[0] = INV_LPA_1_25HZ;
    else if (rate == 2)
        tmp[0] = INV_LPA_2_5HZ;
    else if (rate <= 5)
        tmp[0] = INV_LPA_5HZ;
    else if (rate <= 10)
        tmp[0] = INV_LPA_10HZ;
    else if (rate <= 20)
        tmp[0] = INV_LPA_20HZ;
    else if (rate <= 40)
        tmp[0] = INV_LPA_40HZ;
    else if (rate <= 80)
        tmp[0] = INV_LPA_80HZ;
    else if (rate <= 160)
        tmp[0] = INV_LPA_160HZ;
    else if (rate <= 320)
        tmp[0] = INV_LPA_320HZ;
    else if (rate <= 640)
        tmp[0] = INV_LPA_640HZ;
    else 
        return -1;

    if (mpu_write(inst,INV_MPU_LP_ACCEL_ODR,tmp,1) < 0 )
        return -1;

    tmp[0] = BIT_LPA_CYCLE;
    if (mpu_write(inst,INV_MPU_PWR_MGMT_1,tmp,1) < 0 )
        return -1;
#endif

    inst->chip_cfg.sensors = INV_XYZ_ACCEL;
    inst->chip_cfg.clk_src = 0;
    inst->chip_cfg.lp_accel_mode = 1;
    mpu_configure_fifo(inst,0);

    return 0;
}

/*******************************************************************************
 * Name: mpu_get_accel_raw
 *
 * Description:
 *  Read raw accel data directly from the registers.
 * 
 * Params
 *  inst        instance of inv_mpu driver.
 *  data        Raw data in hardware units.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_get_accel_raw(struct mpu_inst_s* inst, int16_t *data)
{
    uint8_t tmp[6];

    if (!(inst->chip_cfg.sensors & INV_XYZ_ACCEL))
        return -1;

    if (mpu_read(inst,raw_accel, 6, tmp) < 0 )
        return -1;

    data[0] = ((int16_t)tmp[0] << 8) | tmp[1];
    data[1] = ((int16_t)tmp[2] << 8) | tmp[3];
    data[2] = ((int16_t)tmp[4] << 8) | tmp[5];

    return 0;
}

/*******************************************************************************
 * Name: mpu_get_gyro_raw
 *
 * Description:
 *  Read raw gyro data directly from the registers.
 * 
 * Params
 *  inst        instance of inv_mpu driver.
 *  data        Raw data in hardware units.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_get_gyro_raw(struct mpu_inst_s* inst, int16_t *data )
{
    uint8_t tmp[6];

    if (!(inst->chip_cfg.sensors & INV_XYZ_GYRO))
        return -1;

    if (mpu_read(inst,raw_gyro, 6, tmp) < 0 )
        return -1;

    data[0] = ((int16_t)tmp[0] << 8) | tmp[1];
    data[1] = ((int16_t)tmp[2] << 8) | tmp[3];
    data[2] = ((int16_t)tmp[4] << 8) | tmp[5];

    return 0;
}

/*******************************************************************************
 * Name: mpu_get_compass_raw
 *
 * Description:
 *  Read raw compass data directly from the registers.
 * 
 * Params
 *  inst        instance of inv_mpu driver.
 *  data        Raw data in hardware units.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_get_compass_raw(int16_t *data)
{
#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
    uint8_t tmp[9];

    if (!(inst->chip_cfg.sensors & INV_XYZ_COMPASS))
        return -1;

    if (mpu_read(inst,raw_compass, 8, tmp) < 0 )
        return -1;

    /* AK8975 doesn't have the overrun error bit. */
    
#if defined AK8975_SECONDARY
    if (!(tmp[0] & AKM_DATA_READY))
        return -1;
    if ((tmp[7] & AKM_OVERFLOW) || (tmp[7] & AKM_DATA_ERROR))
        return -1;

    /* AK8963 doesn't have the data read error bit. */

#elif defined AK8963_SECONDARY
    if (!(tmp[0] & AKM_DATA_READY) || (tmp[0] & AKM_DATA_OVERRUN))
        return -1;
    if (tmp[7] & AKM_OVERFLOW)
        return -1;
#endif

    data[0] = (tmp[2] << 8) | tmp[1];
    data[1] = (tmp[4] << 8) | tmp[3];
    data[2] = (tmp[6] << 8) | tmp[5];

    return 0;
#else
    return -1;
#endif
}

/*******************************************************************************
 * Name: mpu_get_temperature
 *
 * Description:
 *  Read temperature data directly from the registers.
 * 
 * Params
 *  inst        instance of inv_mpu driver.
 *  data        Data in q16 format.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_get_temperature(struct mpu_inst_s* inst, long *data )
{
    uint8_t tmp[2];
    int16_t raw;

    if (!(inst->chip_cfg.sensors))
        return -1;

    if (mpu_read(inst,temp, 2, tmp) < 0 )
        return -1;

    raw = (tmp[0] << 8) | tmp[1];

    data[0] = (long)((35 + ((raw - (float)inst->hw->temp_offset) / 
                            inst->hw->temp_sens)) * 65536L);
    return 0;
}

/*******************************************************************************
 * Name: mpu_get_accel_off
 *
 * Description:
 *  Read offset accelerometer.
 *  The register is initialized with OTP factory trim values.
 * 
 * Params
 *  inst        instance of inv_mpu driver.
 *  accel_off   returned structure with the accelerometer (tab[3])
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_get_accel_off(struct mpu_inst_s* inst, int16_t *accel_off) 
{
    uint8_t data[2];

    /* due to space between register we cannot do it in one read */

    if (mpu_read(inst, INV_MPU_XA_OFFSET_H, 2, data))
        return -1;

    accel_off[0] = ((long)data[0]<<8) | data[1];

    if (mpu_read(inst, INV_MPU_YA_OFFSET_H, 2, data))

        return -1
    accel_off[1] = ((long)data[0]<<8) | data[1];

    if (mpu_read(inst, INV_MPU_ZA_OFFSET_H, 2, data))
        return -1;

    accel_off[2] = ((long)data[0]<<8) | data[1];

    return 0;
}

/*******************************************************************************
 * Name: mpu_set_accel_off
 *
 * Description:
 *  Set offset of accelerometer.
 *  The register is initialized with OTP factory trim values.
 * 
 * Params
 *  inst        instance of inv_mpu driver.
 *  gyro_off    setted structure with the accelerometer offset.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_set_accel_off(struct mpu_inst_s* inst,const int16_t *accel_off)
{
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    long accel_reg_off[3] = {0, 0, 0};
    long mask = 0x0001;
    uint8_t mask_bit[3] = {0, 0, 0};
    uint8_t i = 0;

    if(mpu_read_accel_bias(accel_reg_off))
        return -1;

    //bit 0 of the 2 byte bias is for temp comp
    //calculations need to compensate for this and not change it
    for(i=0; i<3; i++) {
        if(accel_reg_off[i]&mask)
            mask_bit[i] = 0x01;
    }

    accel_reg_off[0] -= accel_off[0];
    accel_reg_off[1] -= accel_off[1];
    accel_reg_off[2] -= accel_off[2];

    data[0] = (accel_reg_off[0] >> 8) & 0xff;
    data[1] = (accel_reg_off[0]) & 0xff;
    data[1] = data[1]|mask_bit[0];
    data[2] = (accel_reg_off[1] >> 8) & 0xff;
    data[3] = (accel_reg_off[1]) & 0xff;
    data[3] = data[3]|mask_bit[1];
    data[4] = (accel_reg_off[2] >> 8) & 0xff;
    data[5] = (accel_reg_off[2]) & 0xff;
    data[5] = data[5]|mask_bit[2];

    if (mpu_write(inst, INV_MPU_XA_OFFSET_H, &data[0], 2) < 0)
        return -1;
    if (mpu_write(inst, INV_MPU_XA_OFFSET_H, &data[2], 2) < 0)
        return -1;
    if (mpu_write(inst, INV_MPU_XA_OFFSET_H, &data[4], 2) < 0)
        return -1;

    return 0;
}


/*******************************************************************************
 * Name: mpu_get_gyro_off
 *
 * Description:
 *  Get offset of gyroscope.
 *  The register is initialized with OTP factory trim values.
 * 
 * Params
 *  inst        instance of inv_mpu driver.
 *  gyro_off    returned structure with the gyroscope.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_get_gyro_off(int16_t *gyro_off) 
{
    uint8_t data[6];

    if (mpu_read(inst, INV_MPU_XG_OFFSET_H, 6, &data[0]))
        return -1;

    gyro_off[0] = ((int16_t)data[0]<<8) | data[1];
    gyro_off[1] = ((int16_t)data[2]<<8) | data[3];
    gyro_off[2] = ((int16_t)data[4]<<8) | data[5];

    return 0;
}

/*******************************************************************************
 * Name: mpu_set_gyro_off
 *
 * Description:
 *  Set offset of gyroscope.
 *  The register is initialized with OTP factory trim values.
 * 
 * Params
 *  inst        instance of inv_mpu driver.
 *  gyro_off    setted structure with the gyroscope offset.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_set_gyro_off(struct mpu_inst_s* inst, const uint16_t *gyro_off)
{
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    int i=0;

    for(i=0;i<3;i++) {
        gyro_off[i]= (-gyro_off[i]);
    }

    data[0] = (gyro_off[0] >> 8) & 0xff;
    data[1] = (gyro_off[0]) & 0xff;
    data[2] = (gyro_off[1] >> 8) & 0xff;
    data[3] = (gyro_off[1]) & 0xff;
    data[4] = (gyro_off[2] >> 8) & 0xff;
    data[5] = (gyro_off[2]) & 0xff;

    if (mpu_write(inst, INV_MPU_XG_OFFSET_H, 6, &data[0]))
        return -1;

    return 0;
}


/*******************************************************************************
 * Name: mpu_reset_fifo
 *
 * Description:
 *  Reset FIFO read/write pointers.
 * 
 * Params
 *  inst        instance of inv_mpu driver.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_reset_fifo(struct mpu_inst_s* inst)
{
    uint8_t data;

    if (!(inst->chip_cfg.sensors))
        return -1;

    data = 0;
    if (mpu_write(inst,INV_MPU_INT_ENABLE,&data,1) < 0 )
        return -1;
    if (mpu_write(inst,INV_MPU_FIFO_EN,&data,1) < 0 )
        return -1;
    if (mpu_write(inst,INV_MPU_USER_CTRL,&data,1) < 0 )
        return -1;

    if (inst->chip_cfg.dmp_on) 
    {
        data = BIT_FIFO_RST | BIT_DMP_RST;
        if (mpu_write(inst,INV_MPU_USER_CTRL,&data,1) < 0 )
            return -1;
        up_mdelay(50);
        data = BIT_DMP_EN | BIT_FIFO_EN;
        if (inst->chip_cfg.sensors & INV_XYZ_COMPASS)
            data |= BIT_AUX_IF_EN;
        if (mpu_write(inst,INV_MPU_USER_CTRL,&data,1) < 0 )
            return -1;
        if (inst->chip_cfg.int_enable)
            data = BIT_DMP_INT_EN;
        else
            data = 0;
        if (mpu_write(inst,INV_MPU_INT_ENABLE,&data,1) < 0 )
            return -1;
        data = 0;
        if (mpu_write(inst,INV_MPU_FIFO_EN,&data,1) < 0 )
            return -1;
    } 
    else 
    {
        data = BIT_FIFO_RST;
        if (mpu_write(inst,INV_MPU_USER_CTRL,&data,1) < 0 )
            return -1;

        if (  (inst->chip_cfg.bypass_mode                   ) || 
              ( !(inst->chip_cfg.sensors & INV_XYZ_COMPASS  ) ) )
        {
            data = BIT_FIFO_EN;
        }
        else
        {
            data = BIT_FIFO_EN | BIT_AUX_IF_EN;
        }

        if (mpu_write(inst,INV_MPU_USER_CTRL,&data,1) < 0 )
            return -1;

        up_mdelay(50);

        if (inst->chip_cfg.int_enable)
        {
            data = BIT_DATA_RDY_EN;
        }
        else
        {
            data = 0;
        }

        if (mpu_write(inst,INV_MPU_INT_ENABLE,&data,1) < 0 )
            return -1;

        if (mpu_write(inst,INV_MPU_FIFO_EN,&inst->chip_cfg.fifo_enable,1) < 0 )
            return -1;

    }
    return 0;
}

/*******************************************************************************
 * Name: mpu_get_accel_fsr
 *
 * Description:
 *  Get the accel full-scale range.
 * 
 * Params
 *  inst        instance of inv_mpu driver.
 *  fsr         Current full-scale range in G.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_get_accel_fsr(uint8_t *fsr)
{
    switch (inst->chip_cfg.accel_fsr) {
        case MPU_ACC_FSR_2G:
            *fsr = 2;
            break;
        case MPU_ACC_FSR_4G:
            *fsr = 4;
            break;
        case MPU_ACC_FSR_8G:
            *fsr = 8;
            break;
        case MPU_ACC_FSR_16G:
            *fsr = 16;
            break;
        default:
            return -1;
    }

    if (inst->chip_cfg.accel_half)
        *fsr <<= 1;

    return 0;
}

/*******************************************************************************
 * Name: mpu_set_accel_fsr
 *
 * Description:
 *  Set the accel full-scale range.
 * 
 * Params
 *  inst        instance of inv_mpu driver.
 *  fsr         Desired full-scale range in G.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_set_accel_fsr(uint8_t fsr)
{
    uint8_t data;

    if (!(inst->chip_cfg.sensors))
        return -1;

    switch (fsr) {
        case 2:
            data = MPU_ACC_FSR_2G << 3;
            break;
        case 4:
            data = MPU_ACC_FSR_4G << 3;
            break;
        case 8:
            data = MPU_ACC_FSR_8G << 3;
            break;
        case 16:
            data = MPU_ACC_FSR_16G << 3;
            break;
        default:
            return -1;
    }

    if (inst->chip_cfg.accel_fsr == (data >> 3))
        return 0;

    if (mpu_write(inst,INV_MPU_ACCEL_CFG,&data,1) < 0 )
        return -1;

    inst->chip_cfg.accel_fsr = data >> 3;

    return 0;
}

/*******************************************************************************
 * Name: mpu_get_gyro_fsr
 *
 * Description:
 *  Get the gyroscope full-scale range.
 * 
 * Params
 *  inst        instance of inv_mpu driver.
 *  fsr         Current full-scale range in DPS.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_get_gyro_fsr(uint16_t *fsr)
{
    switch (inst->chip_cfg.gyro_fsr) {
        case MPU_GYRO_FSR_250DPS:
            *fsr = 250;
            break;
        case MPU_GYRO_FSR_500DPS:
            *fsr = 500;
            break;
        case MPU_GYRO_FSR_1000DPS:
            *fsr = 1000;
            break;
        case MPU_GYRO_FSR_2000DPS:
            *fsr = 2000;
            break;
        default:
            *fsr = 0;
            break;
    }
    return 0;
}

/*******************************************************************************
 * Name: mpu_set_gyro_fsr
 *
 * Description:
 *  Set the gyroscope full-scale range.
 * 
 * Params
 *  inst        instance of inv_mpu driver.
 *  fsr         Desired full-scale range in DPS.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_set_gyro_fsr(uint16_t fsr)
{
    uint8_t data;

    if (!(inst->chip_cfg.sensors))
        return -1;

    switch (fsr) {
        case 250:
            data = MPU_GYRO_FSR_250DPS << 3;
            break;
        case 500:
            data = MPU_GYRO_FSR_500DPS << 3;
            break;
        case 1000:
            data = MPU_GYRO_FSR_1000DPS << 3;
            break;
        case 2000:
            data = MPU_GYRO_FSR_2000DPS << 3;
            break;
        default:
            return -1;
    }

    if (inst->chip_cfg.gyro_fsr == (data >> 3))
        return 0;
    
    if (mpu_write(inst,INV_MPU_GYRO_CFG,&data,1) < 0 )
        return -1;

    inst->chip_cfg.gyro_fsr = data >> 3;

    return 0;
}

/*******************************************************************************
 * Name: mpu_get_lpf
 *
 * Description:
 *  Get the low power filter.
 * 
 * Params
 *  inst        instance of inv_mpu driver.
 *  lpf         Current low power filter range in Hz.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_get_lpf(uint16_t *lpf)
{
    switch (inst->chip_cfg.lpf) {
        case MPU_LPF_188HZ:
            *lpf = 188;
            break;
        case MPU_LPF_98HZ:
            *lpf = 98;
            break;
        case MPU_LPF_42HZ:
            *lpf = 42;
            break;
        case MPU_LPF_20HZ:
            *lpf = 20;
            break;
        case MPU_LPF_10HZ:
            *lpf = 10;
            break;
        case MPU_LPF_5HZ:
            *lpf = 5;
            break;
        case MPU_LF_256HZ_NOLPF2:
        case MPU_LF_2100HZ_NOLPF:
        default:
            *lpf = 0;
            break;
    }
    return 0;
}

/*******************************************************************************
 * Name: mpu_set_lpf
 *
 * Description:
 *  Set the low power filter.
 * 
 * Params
 *  inst        instance of inv_mpu driver.
 *  lpf         Desired low power filter range in Hz.
 *
 * Return
 *  0 on success, negative value in case of error.
 ******************************************************************************/

int mpu_set_lpf(uint16_t lpf)
{
    uint8_t data;

    if (!(inst->chip_cfg.sensors))
        return -1;

    if (lpf >= 188)
    {
        data = MPU_LPF_188HZ;
    }
    else if (lpf >= 98)
    {
        data = MPU_LPF_98HZ;
    }
    else if (lpf >= 42)
    {
        data = MPU_LPF_42HZ;
    }
    else if (lpf >= 20)
    {
        data = MPU_LPF_20HZ;
    }
    else if (lpf >= 10)
    {
        data = MPU_LPF_10HZ;
    }
    else
    {
        data = MPU_LPF_5HZ;
    }

    if (inst->chip_cfg.lpf == data)
        return 0;

    if (mpu_write(inst,INV_MPU_LPF,&data,1) < 0 )
        return -1;

    inst->chip_cfg.lpf = data;
    return 0;
}

/**
 *  @brief      Get sampling rate.
 *  @param[out] rate    Current sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_get_sample_rate(uint16_t *rate)
{
    if (inst->chip_cfg.dmp_on)
        return -1;
    else
        rate[0] = inst->chip_cfg.sample_rate;
    return 0;
}

/**
 *  @brief      Set sampling rate.
 *  Sampling rate must be between 4Hz and 1kHz.
 *  @param[in]  rate    Desired sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_set_sample_rate(uint16_t rate)
{
    uint8_t data;

    if (!(inst->chip_cfg.sensors))
        return -1;

    if (inst->chip_cfg.dmp_on)
        return -1;
    else 
    {
        if (inst->chip_cfg.lp_accel_mode) 
        {

            /* Just stay in low-power accel mode. */

            if ( mpu_lp_accel_mode(rate) == 0 )
            {
                return 0;
            }

            if ( rate ) 
            {

                /* LP accel mode failed, retry to go back to full-power mode. */

                if ( mpu_lp_accel_mode(0) < 0 )
                    return -1;
            }
        }
        if (rate < 4)
        {
            rate = 4;
        }
        else if (rate > 1000)
        {
            rate = 1000;
        }

        data = 1000 / rate - 1;

        if (mpu_write(inst,INV_MPU_RATE_DIV,&data,1) < 0 )
            return -1;

        inst->chip_cfg.sample_rate = 1000 / (1 + data);

#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
        if ( inst->chip_cfg.sample_rate > MAX_COMPASS_SAMPLE_RATE )
            inst->chip_cfg.sample_rate = MAX_COMPASS_SAMPLE_RATE;

        mpu_set_compass_sample_rate(inst->chip_cfg.compass_sample_rate);
#endif

        /* Automatically set LPF to 1/2 sampling rate. */

        mpu_set_lpf(inst->chip_cfg.sample_rate >> 1);

        return 0;
    }
}

/**
 *  @brief      Get compass sampling rate.
 *  @param[out] rate    Current compass sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_get_compass_sample_rate(uint16_t *rate)
{
#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
    rate[0] = inst->chip_cfg.compass_sample_rate;
    return 0;
#else
    rate[0] = 0;
    return -1;
#endif
}

/**
 *  @brief      Set compass sampling rate.
 *  The compass on the auxiliary I2C bus is read by the MPU hardware at a
 *  maximum of 100Hz. The actual rate can be set to a fraction of the gyro
 *  sampling rate.
 *
 *  \n WARNING: The new rate may be different than what was requested. Call
 *  mpu_get_compass_sample_rate to check the actual setting.
 *  @param[in]  rate    Desired compass sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_set_compass_sample_rate(uint16_t rate)
{
#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
    uint8_t div;
    if (!rate || rate > inst->chip_cfg.sample_rate || rate > MAX_COMPASS_SAMPLE_RATE)
        return -1;

    div = inst->chip_cfg.sample_rate / rate - 1;
    if (mpu_write(inst,INV_MPU_S4_CTRL,&div,1) < 0 )
        return -1;
    inst->chip_cfg.compass_sample_rate = inst->chip_cfg.sample_rate / (div + 1);
    return 0;
#else
    return -1;
#endif
}

/**
 *  @brief      Get gyro sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to dps.
 *  @return     0 if successful.
 */
int mpu_get_gyro_sens(float *sens)
{
    switch (inst->chip_cfg.gyro_fsr) {
    case INV_FSR_250DPS:
        sens[0] = 131.f;
        break;
    case INV_FSR_500DPS:
        sens[0] = 65.5f;
        break;
    case INV_FSR_1000DPS:
        sens[0] = 32.8f;
        break;
    case INV_FSR_2000DPS:
        sens[0] = 16.4f;
        break;
    default:
        return -1;
    }
    return 0;
}

/**
 *  @brief      Get accel sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to g's.
 *  @return     0 if successful.
 */
int mpu_get_accel_sensibilty(uint16_t *sens)
{
    switch (inst->chip_cfg.accel_fsr) {
        case MPU_ACC_FSR_2G:
            sens[0] = 16384;
            break;
        case MPU_ACC_FSR_4G:
            sens[0] = 8092;
            break;
        case MPU_ACC_FSR_8G:
            sens[0] = 4096;
            break;
        case MPU_ACC_FSR_16G:
            sens[0] = 2048;
            break;
        default:
            return -1;
    }
    if (inst->chip_cfg.accel_half)
        sens[0] >>= 1;
    return 0;
}

/**
 *  @brief      Get current FIFO configuration.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[out] sensors Mask of sensors in FIFO.
 *  @return     0 if successful.
 */
int mpu_get_fifo_config(struct mpu_inst_s* inst,uint8_t *sensors)
{
    *sensors = inst->chip_cfg.fifo_enable;
    return 0;
}

/**
 *  @brief      Select which sensors are pushed to FIFO.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[in]  sensors Mask of sensors to push to FIFO.
 *  @return     0 if successful.
 */
int mpu_set_fifo_config(struct mpu_inst_s* inst, uint8_t sensors)
{
    uint8_t prev;
    int result = 0;

    /* Compass data isn't going into the FIFO. Stop trying. */
    sensors &= ~INV_XYZ_COMPASS;

    if (inst->chip_cfg.dmp_on)
        return 0;
    else {
        if (!(inst->chip_cfg.sensors))
            return -1;
        prev = inst->chip_cfg.fifo_enable;
        inst->chip_cfg.fifo_enable = sensors & inst->chip_cfg.sensors;
        if (inst->chip_cfg.fifo_enable != sensors)
            /* You're not getting what you asked for. Some sensors are
             * asleep.
             */
            result = -1;
        else
            result = 0;
        if (sensors || inst->chip_cfg.lp_accel_mode)
            set_int_enable(1);
        else
            set_int_enable(0);
        if (sensors) {
            if (mpu_reset_fifo()) {
                inst->chip_cfg.fifo_enable = prev;
                return -1;
            }
        }
    }

    return result;
}

int mpu_get_sensors_enable(struct mpu_inst_s* inst, uint8_t* sensors)
{
    *sensors = inst->chip_cfg.sensors = 0;
    return 0;
}

/**
 *  @brief      Turn specific sensors on/off.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_XYZ_COMPASS
 *  @param[in]  sensors    Mask of sensors to wake.
 *  @return     0 if successful.
 */
int mpu_set_sensors_enable(uint8_t sensors)
{
    uint8_t data;
#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
    uint8_t user_ctrl;
#endif

    if (sensors & INV_XYZ_GYRO)
        data = INV_CLK_PLL;
    else if (sensors)
        data = 0;
    else
        data = BIT_SLEEP;
    if (mpu_write(inst,INV_MPU_PWR_MGMT_1,&data,1)) {
        inst->chip_cfg.sensors = 0;
        return -1;
    }
    inst->chip_cfg.clk_src = data & ~BIT_SLEEP;

    data = 0;
    if (!(sensors & INV_X_GYRO))
        data |= BIT_STBY_XG;
    if (!(sensors & INV_Y_GYRO))
        data |= BIT_STBY_YG;
    if (!(sensors & INV_Z_GYRO))
        data |= BIT_STBY_ZG;
    if (!(sensors & INV_XYZ_ACCEL))
        data |= BIT_STBY_XYZA;
    if (mpu_write(inst,INV_MPU_PWR_MGMT_2,&data,1)) {
        inst->chip_cfg.sensors = 0;
        return -1;
    }

    if (sensors && (sensors != INV_XYZ_ACCEL))
        /* Latched interrupts only used in LP accel mode. */
        mpu_set_int_latched(false);

#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
    if (mpu_read(inst,user_ctrl, 1, &user_ctrl) < 0 )
        return -1;
    /* Handle AKM power management. */
    if (sensors & INV_XYZ_COMPASS) {
        data = AKM_SINGLE_MEASUREMENT;
        user_ctrl |= BIT_AUX_IF_EN;
    } else {
        data = AKM_POWER_DOWN;
        user_ctrl &= ~BIT_AUX_IF_EN;
    }
    if (inst->chip_cfg.dmp_on)
        user_ctrl |= BIT_DMP_EN;
    else
        user_ctrl &= ~BIT_DMP_EN;
    if (mpu_write(inst,INV_MPU_S1_DO,&data,1) < 0 )
        return -1;
    /* Enable/disable I2C master mode. */
    if (mpu_write(inst,INV_MPU_USER_CTRL,&user_ctrl,1) < 0 )
        return -1;
#endif

    inst->chip_cfg.sensors = sensors;
    inst->chip_cfg.lp_accel_mode = 0;
    up_mdelay(50);
    return 0;
}

/**
 *  @brief      Read the MPU and DMP interrupt status registers.
 *  @param[out] status  Mask of interrupt bits.
 *  @return     0 if successful.
 */
int mpu_get_int_status(int16_t *status)
{
    uint8_t tmp[2];
    if (!inst->chip_cfg.sensors)
        return -1;
    if (mpu_read(inst,INV_MPU_DMP_INT_STATUS, 2, tmp) < 0 )
        return -1;
    status[0] = (tmp[0] << 8) | tmp[1];
    return 0;
}

/**
 *  @brief      Get one packet from the FIFO.
 *  If @e sensors does not contain a particular sensor, disregard the data
 *  returned to that pointer.
 *  \n @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n If the FIFO has no new data, @e sensors will be zero.
 *  \n If the FIFO is disabled, @e sensors will be zero and this function will
 *  return a non-zero error code.
 *  @param[out] gyro        Gyro data in hardware units.
 *  @param[out] accel       Accel data in hardware units.
 *  @param[out] tp          value of CLOCK_REALTIME just after fifo read (set NULL if not used).
 *  @param[out] sensors     Mask of sensors read from FIFO.
 *  @param[out] more        Number of remaining packets.
 *  @return     0 if successful.
 */
int mpu_read_fifo(struct mpu_inst_s* inst, int16_t *accel,int16_t *gyro, 
                  uint8_t *sensors, uint8_t *more, 
                  struct timespec *tp)
{
    /* Assumes maximum packet size is gyro (6) + accel (6). */
    uint8_t data[MAX_PACKET_LENGTH];
    uint8_t packet_size = 0;
    uint16_t fifo_count, index = 0;

    if (inst->chip_cfg.dmp_on)
        return -1;

    sensors[0] = 0;
    if (!inst->chip_cfg.sensors)
        return -1;
    if (!inst->chip_cfg.fifo_enable)
        return -1;

    if (inst->chip_cfg.fifo_enable & INV_X_GYRO)
        packet_size += 2;
    if (inst->chip_cfg.fifo_enable & INV_Y_GYRO)
        packet_size += 2;
    if (inst->chip_cfg.fifo_enable & INV_Z_GYRO)
        packet_size += 2;
    if (inst->chip_cfg.fifo_enable & INV_XYZ_ACCEL)
        packet_size += 6;

    if (mpu_read(inst,fifo_count_h, 2, data) < 0 )
        return -1;
    fifo_count = (data[0] << 8) | data[1];
    if (fifo_count < packet_size)
        return 0;
    //    invdbg("FIFO count: %hd\n", fifo_count);
    if (fifo_count > (inst->hw->max_fifo >> 1)) {
        /* FIFO is 50% full, better check overflow bit. */
        if (mpu_read(inst,int_status, 1, data) < 0 )
            return -1;
        if (data[0] & BIT_FIFO_OVERFLOW) {
            mpu_reset_fifo();
            return -2;
        }
    }
    mpu_get_ms(tp);

    if (mpu_read(inst,fifo_r_w, packet_size, data) < 0 )
        return -1;
    more[0] = fifo_count / packet_size - 1;
    sensors[0] = 0;

    if ((index != packet_size) && inst->chip_cfg.fifo_enable & INV_XYZ_ACCEL) {
        accel[0] = (data[index+0] << 8) | data[index+1];
        accel[1] = (data[index+2] << 8) | data[index+3];
        accel[2] = (data[index+4] << 8) | data[index+5];
        sensors[0] |= INV_XYZ_ACCEL;
        index += 6;
    }
    if ((index != packet_size) && inst->chip_cfg.fifo_enable & INV_X_GYRO) {
        gyro[0] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= INV_X_GYRO;
        index += 2;
    }
    if ((index != packet_size) && inst->chip_cfg.fifo_enable & INV_Y_GYRO) {
        gyro[1] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= INV_Y_GYRO;
        index += 2;
    }
    if ((index != packet_size) && inst->chip_cfg.fifo_enable & INV_Z_GYRO) {
        gyro[2] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= INV_Z_GYRO;
        index += 2;
    }

    return 0;
}

/**
 *  @brief      Get one unparsed packet from the FIFO.
 *  This function should be used if the packet is to be parsed elsewhere.
 *  @param[in]  length  Length of one FIFO packet.
 *  @param[in]  data    FIFO packet.
 *  @param[in]  more    Number of remaining packets.
 */
int mpu_read_fifo_stream(struct mpu_inst_s* inst,uint16_t length, 
                         uint8_t *data, uint8_t *more)
{
    uint8_t tmp[2];
    uint16_t fifo_count;
    if (!inst->chip_cfg.dmp_on)
        return -1;
    if (!inst->chip_cfg.sensors)
        return -1;

    if (mpu_read(inst,fifo_count_h, 2, tmp) < 0 )
        return -1;
    fifo_count = (tmp[0] << 8) | tmp[1];
    if (fifo_count < length) {
        more[0] = 0;
        return -1;
    }
    if (fifo_count > (inst->hw->max_fifo >> 1)) {
        /* FIFO is 50% full, better check overflow bit. */
        if (mpu_read(inst,int_status, 1, tmp) < 0 )
            return -1;
        if (tmp[0] & BIT_FIFO_OVERFLOW) {
            mpu_reset_fifo();
            return -2;
        }
    }

    if (mpu_read(inst,fifo_r_w, length, data) < 0 )
        return -1;
    more[0] = fifo_count / length - 1;
    return 0;
}

/**
 *  @brief      Set device to bypass mode.
 *  @param[in]  bypass_on   1 to enable bypass mode.
 *  @return     0 if successful.
 */
int mpu_set_bypass(struct mpu_inst_s* inst, bool bypass_on)
{
    uint8_t tmp;

    if (inst->chip_cfg.bypass_mode == bypass_on)
        return 0;

    if (bypass_on) {
        if (mpu_read(inst,user_ctrl, 1, &tmp) < 0 )
            return -1;
        tmp &= ~BIT_AUX_IF_EN;
        if (mpu_write(inst,INV_MPU_USER_CTRL,&tmp,1) < 0 )
            return -1;
        up_mdelay(3);
        tmp = BIT_BYPASS_EN;
        if (inst->chip_cfg.active_low_int)
            tmp |= BIT_ACTL;
        if (inst->chip_cfg.latched_int)
            tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
        if (mpu_write(inst,INV_MPU_INT_PIN_CFG,&tmp,1) < 0 )
            return -1;
    } else {
        /* Enable I2C master mode if compass is being used. */
        if (mpu_read(inst,user_ctrl, 1, &tmp) < 0 )
            return -1;
        if (inst->chip_cfg.sensors & INV_XYZ_COMPASS)
            tmp |= BIT_AUX_IF_EN;
        else
            tmp &= ~BIT_AUX_IF_EN;
        if (mpu_write(inst,INV_MPU_USER_CTRL,&tmp,1) < 0 )
            return -1;
        up_mdelay(3);
        if (inst->chip_cfg.active_low_int)
            tmp = BIT_ACTL;
        else
            tmp = 0;
        if (inst->chip_cfg.latched_int)
            tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
        if (mpu_write(inst,INV_MPU_INT_PIN_CFG,&tmp,1) < 0 )
            return -1;
    }
    inst->chip_cfg.bypass_mode = bypass_on;
    return 0;
}

/**
 *  @brief      Set interrupt level.
 *  @param[in]  active_low  1 for active low, 0 for active high.
 *  @return     0 if successful.
 */
int mpu_set_int_level(uint8_t active_low)
{
    inst->chip_cfg.active_low_int = active_low;
    return 0;
}

/**
 *  @brief      Enable latched interrupts.
 *  Any MPU register will clear the interrupt.
 *  @param[in]  enable  1 to enable, 0 to disable.
 *  @return     0 if successful.
 */
int mpu_set_int_latched(bool enable)
{
    uint8_t tmp;
    if (inst->chip_cfg.latched_int == enable)
        return 0;

    if (enable)
        tmp = BIT_LATCH_EN | BIT_ANY_RD_CLR;
    else
        tmp = 0;
    if (inst->chip_cfg.bypass_mode)
        tmp |= BIT_BYPASS_EN;
    if (inst->chip_cfg.active_low_int)
        tmp |= BIT_ACTL;
    if (mpu_write(inst,INV_MPU_INT_PIN_CFG,&tmp,1) < 0 )
        return -1;
    inst->chip_cfg.latched_int = enable;
    return 0;
}

#ifdef CONFIG_SENSOR_MPU6050
static int get_accel_prod_shift(float *st_shift)
{
    uint8_t tmp[4], shift_code[3], ii;

    if (i2c_read(inst->hw->addr, 0x0D, 4, tmp))
        return 0x07;

    shift_code[0] = ((tmp[0] & 0xE0) >> 3) | ((tmp[3] & 0x30) >> 4);
    shift_code[1] = ((tmp[1] & 0xE0) >> 3) | ((tmp[3] & 0x0C) >> 2);
    shift_code[2] = ((tmp[2] & 0xE0) >> 3) | (tmp[3] & 0x03);
    for (ii = 0; ii < 3; ii++) {
        if (!shift_code[ii]) {
            st_shift[ii] = 0.f;
            continue;
        }
        /* Equivalent to..
         * st_shift[ii] = 0.34f * powf(0.92f/0.34f, (shift_code[ii]-1) / 30.f)
         */
        st_shift[ii] = 0.34f;
        while (--shift_code[ii])
            st_shift[ii] *= 1.034f;
    }
    return 0;
}

static int accel_self_test(long *bias_regular, long *bias_st)
{
    int jj, result = 0;
    float st_shift[3], st_shift_cust, st_shift_var;

    get_accel_prod_shift(st_shift);
    for(jj = 0; jj < 3; jj++) {
        st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
        if (st_shift[jj]) {
            st_shift_var = st_shift_cust / st_shift[jj] - 1.f;
            if (fabs(st_shift_var) > teinst->max_accel_var)
                result |= 1 << jj;
        } else if ((st_shift_cust < teinst->min_g) ||
                   (st_shift_cust > teinst->max_g))
            result |= 1 << jj;
    }

    return result;
}

static int gyro_self_test(long *bias_regular, long *bias_st)
{
    int jj, result = 0;
    uint8_t tmp[3];
    float st_shift, st_shift_cust, st_shift_var;

    if (i2c_read(inst->hw->addr, 0x0D, 3, tmp))
        return 0x07;

    tmp[0] &= 0x1F;
    tmp[1] &= 0x1F;
    tmp[2] &= 0x1F;

    for (jj = 0; jj < 3; jj++) {
        st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
        if (tmp[jj]) {
            st_shift = 3275.f / teinst->gyro_sens;
            while (--tmp[jj])
                st_shift *= 1.046f;
            st_shift_var = st_shift_cust / st_shift - 1.f;
            if (fabs(st_shift_var) > teinst->max_gyro_var)
                result |= 1 << jj;
        } else if ((st_shift_cust < teinst->min_dps) ||
                   (st_shift_cust > teinst->max_dps))
            result |= 1 << jj;
    }
    return result;
}

#endif 
#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
static int compass_self_test(void)
{
    uint8_t tmp[6];
    uint8_t tries = 10;
    int result = 0x07;
    int16_t data;

    mpu_set_bypass(1);

    tmp[0] = AKM_POWER_DOWN;
    if (i2c_write(inst->chip_cfg.compass_addr, AKM_REG_CNTL, 1, tmp))
        return 0x07;
    tmp[0] = AKM_BIT_SELF_TEST;
    if (i2c_write(inst->chip_cfg.compass_addr, AKM_REG_ASTC, 1, tmp))
        goto AKM_restore;
    tmp[0] = AKM_MODE_SELF_TEST;
    if (i2c_write(inst->chip_cfg.compass_addr, AKM_REG_CNTL, 1, tmp))
        goto AKM_restore;

    do {
        up_mdelay(10);
        if (i2c_read(inst->chip_cfg.compass_addr, AKM_REG_ST1, 1, tmp))
            goto AKM_restore;
        if (tmp[0] & AKM_DATA_READY)
            break;
    } while (tries--);
    if (!(tmp[0] & AKM_DATA_READY))
        goto AKM_restore;

    if (i2c_read(inst->chip_cfg.compass_addr, AKM_REG_HXL, 6, tmp))
        goto AKM_restore;

    result = 0;
#if defined MPU9150
    data = (int16_t)(tmp[1] << 8) | tmp[0];
    if ((data > 100) || (data < -100))
        result |= 0x01;
    data = (int16_t)(tmp[3] << 8) | tmp[2];
    if ((data > 100) || (data < -100))
        result |= 0x02;
    data = (int16_t)(tmp[5] << 8) | tmp[4];
    if ((data > -300) || (data < -1000))
        result |= 0x04;
#elif defined MPU9250
    data = (int16_t)(tmp[1] << 8) | tmp[0];
    if ((data > 200) || (data < -200))  
        result |= 0x01;
    data = (int16_t)(tmp[3] << 8) | tmp[2];
    if ((data > 200) || (data < -200))  
        result |= 0x02;
    data = (int16_t)(tmp[5] << 8) | tmp[4];
    if ((data > -800) || (data < -3200))  
        result |= 0x04;
#endif
AKM_restore:
    tmp[0] = 0 | INV_AK89_HIGH_SENS;
    i2c_write(inst->chip_cfg.compass_addr, AKM_REG_ASTC, 1, tmp);
    tmp[0] = INV_AK89_HIGH_SENS;
    i2c_write(inst->chip_cfg.compass_addr, AKM_REG_CNTL, 1, tmp);
    mpu_set_bypass(0);
    return result;
}
#endif

static int get_st_biases(long *gyro, long *accel, uint8_t hw_test)
{
    uint8_t data[MAX_PACKET_LENGTH];
    uint8_t packet_count, ii;
    uint16_t fifo_count;

    data[0] = 0x01;
    data[1] = 0;
    if (mpu_write(inst,INV_MPU_PWR_MGMT_1,data,2) < 0 )
        return -1;
    up_mdelay(200);
    data[0] = 0;
    if (mpu_write(inst,INV_MPU_INT_ENABLE,data,1) < 0 )
        return -1;
    if (mpu_write(inst,INV_MPU_FIFO_EN,data,1) < 0 )
        return -1;
    if (mpu_write(inst,INV_MPU_PWR_MGMT_1,data,1) < 0 )
        return -1;
    if (mpu_write(inst,INV_MPU_I2C_MST,data,1) < 0 )
        return -1;
    if (mpu_write(inst,INV_MPU_USER_CTRL,data,1) < 0 )
        return -1;
    data[0] = BIT_FIFO_RST | BIT_DMP_RST;
    if (mpu_write(inst,INV_MPU_USER_CTRL,data,1) < 0 )
        return -1;
    up_mdelay(15);
    data[0] = inst->test->reg_lpf;
    if (mpu_write(inst,INV_MPU_LPF,data,1) < 0 )
        return -1;
    data[0] = inst->test->reg_rate_div;
    if (mpu_write(inst,INV_MPU_RATE_DIV,data,1) < 0 )
        return -1;
    if (hw_test)
        data[0] = inst->test->reg_gyro_fsr | 0xE0;
    else
        data[0] = inst->test->reg_gyro_fsr;
    if (mpu_write(inst,INV_MPU_GYRO_CFG,data,1) < 0 )
        return -1;

    if (hw_test)
        data[0] = inst->test->reg_accel_fsr | 0xE0;
    else
        data[0] = teinst->reg_accel_fsr;
    if (mpu_write(inst,INV_MPU_ACCEL_CFG,data,1) < 0 )
        return -1;
    if (hw_test)
        up_mdelay(200);

    /* Fill FIFO for teinst->wait_ms milliseconds. */
    data[0] = BIT_FIFO_EN;
    if (mpu_write(inst,INV_MPU_USER_CTRL,data,1) < 0 )
        return -1;

    data[0] = INV_XYZ_GYRO | INV_XYZ_ACCEL;
    if (mpu_write(inst,INV_MPU_FIFO_EN,data,1) < 0 )
        return -1;
    up_mdelay(teinst->wait_ms);
    data[0] = 0;
    if (mpu_write(inst,INV_MPU_FIFO_EN,data,1) < 0 )
        return -1;

    if (mpu_read(inst,fifo_count_h,data,2) < 0 )
        return -1;

    fifo_count = (data[0] << 8) | data[1];
    packet_count = fifo_count / MAX_PACKET_LENGTH;
    gyro[0] = gyro[1] = gyro[2] = 0;
    accel[0] = accel[1] = accel[2] = 0;

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_cur[3], gyro_cur[3];
        if (mpu_read(inst,fifo_r_w, MAX_PACKET_LENGTH, data) < 0 )
            return -1;
        accel_cur[0] = ((int16_t)data[0] << 8) | data[1];
        accel_cur[1] = ((int16_t)data[2] << 8) | data[3];
        accel_cur[2] = ((int16_t)data[4] << 8) | data[5];
        accel[0] += (long)accel_cur[0];
        accel[1] += (long)accel_cur[1];
        accel[2] += (long)accel_cur[2];
        gyro_cur[0] = (((int16_t)data[6] << 8) | data[7]);
        gyro_cur[1] = (((int16_t)data[8] << 8) | data[9]);
        gyro_cur[2] = (((int16_t)data[10] << 8) | data[11]);
        gyro[0] += (long)gyro_cur[0];
        gyro[1] += (long)gyro_cur[1];
        gyro[2] += (long)gyro_cur[2];
    }
#ifdef EMPL_NO_64BIT
    gyro[0] = (long)(((float)gyro[0]*65536.f) / teinst->gyro_sens / packet_count);
    gyro[1] = (long)(((float)gyro[1]*65536.f) / teinst->gyro_sens / packet_count);
    gyro[2] = (long)(((float)gyro[2]*65536.f) / teinst->gyro_sens / packet_count);
    if (has_accel) {
        accel[0] = (long)(((float)accel[0]*65536.f) / teinst->accel_sens /
                          packet_count);
        accel[1] = (long)(((float)accel[1]*65536.f) / teinst->accel_sens /
                          packet_count);
        accel[2] = (long)(((float)accel[2]*65536.f) / teinst->accel_sens /
                          packet_count);
        /* Don't remove gravity! */
        accel[2] -= 65536L;
    }
#else
    gyro[0] = (long)(((long long)gyro[0]<<16) / teinst->gyro_sens / packet_count);
    gyro[1] = (long)(((long long)gyro[1]<<16) / teinst->gyro_sens / packet_count);
    gyro[2] = (long)(((long long)gyro[2]<<16) / teinst->gyro_sens / packet_count);
    accel[0] = (long)(((long long)accel[0]<<16) / teinst->accel_sens /
                      packet_count);
    accel[1] = (long)(((long long)accel[1]<<16) / teinst->accel_sens /
                      packet_count);
    accel[2] = (long)(((long long)accel[2]<<16) / teinst->accel_sens /
                      packet_count);
    /* Don't remove gravity! */
    if (accel[2] > 0L)
        accel[2] -= 65536L;
    else
        accel[2] += 65536L;
#endif

    return 0;
}

#ifdef MPU6500
#define REG_6500_XG_ST_DATA     0x0
#define REG_6500_XA_ST_DATA     0xD
static const uint16_t mpu_6500_st_tb[256] = {
    2620,2646,2672,2699,2726,2753,2781,2808, //7
    2837,2865,2894,2923,2952,2981,3011,3041, //15
    3072,3102,3133,3165,3196,3228,3261,3293, //23
    3326,3359,3393,3427,3461,3496,3531,3566, //31
    3602,3638,3674,3711,3748,3786,3823,3862, //39
    3900,3939,3979,4019,4059,4099,4140,4182, //47
    4224,4266,4308,4352,4395,4439,4483,4528, //55
    4574,4619,4665,4712,4759,4807,4855,4903, //63
    4953,5002,5052,5103,5154,5205,5257,5310, //71
    5363,5417,5471,5525,5581,5636,5693,5750, //79
    5807,5865,5924,5983,6043,6104,6165,6226, //87
    6289,6351,6415,6479,6544,6609,6675,6742, //95
    6810,6878,6946,7016,7086,7157,7229,7301, //103
    7374,7448,7522,7597,7673,7750,7828,7906, //111
    7985,8065,8145,8227,8309,8392,8476,8561, //119
    8647,8733,8820,8909,8998,9088,9178,9270,
    9363,9457,9551,9647,9743,9841,9939,10038,
    10139,10240,10343,10446,10550,10656,10763,10870,
    10979,11089,11200,11312,11425,11539,11654,11771,
    11889,12008,12128,12249,12371,12495,12620,12746,
    12874,13002,13132,13264,13396,13530,13666,13802,
    13940,14080,14221,14363,14506,14652,14798,14946,
    15096,15247,15399,15553,15709,15866,16024,16184,
    16346,16510,16675,16842,17010,17180,17352,17526,
    17701,17878,18057,18237,18420,18604,18790,18978,
    19167,19359,19553,19748,19946,20145,20347,20550,
    20756,20963,21173,21385,21598,21814,22033,22253,
    22475,22700,22927,23156,23388,23622,23858,24097,
    24338,24581,24827,25075,25326,25579,25835,26093,
    26354,26618,26884,27153,27424,27699,27976,28255,
    28538,28823,29112,29403,29697,29994,30294,30597,
    30903,31212,31524,31839,32157,32479,32804,33132
};
static int accel_6500_self_test(long *bias_regular, long *bias_st)
{
    int i, result = 0, otp_value_zero = 0;
    float accel_st_al_min, accel_st_al_max;
    float st_shift_cust[3], st_shift_ratio[3], ct_shift_prod[3], accel_offset_max;
    uint8_t regs[3];
    if (i2c_read(inst->hw->addr, REG_6500_XA_ST_DATA, 3, regs)) {
        invdbg("Reading OTP Register Error.\n");
        return 0x07;
    }
    invdbg("Accel OTP:%d, %d, %d\n", regs[0], regs[1], regs[2]);
    for (i = 0; i < 3; i++) {
        if (regs[i] != 0) {
            ct_shift_prod[i] = mpu_6500_st_tb[regs[i] - 1];
            ct_shift_prod[i] *= 65536.f;
            ct_shift_prod[i] /= teinst->accel_sens;
        }
        else {
            ct_shift_prod[i] = 0;
            otp_value_zero = 1;
        }
    }
    if(otp_value_zero == 0) {
        invdbg("ACCEL:CRITERIA A\n");
        for (i = 0; i < 3; i++) {
            st_shift_cust[i] = bias_st[i] - bias_regular[i];
            invdbg("Bias_Shift=%7.4f, Bias_Reg=%7.4f, Bias_HWST=%7.4f\r\n",
                   st_shift_cust[i]/1.f, bias_regular[i]/1.f,
                   bias_st[i]/1.f);
            invdbg("OTP value: %7.4f\r\n", ct_shift_prod[i]/1.f);
        }

        st_shift_ratio[i] = st_shift_cust[i] / ct_shift_prod[i] - 1.f;

        invdbg("ratio=%7.4f, threshold=%7.4f\r\n", st_shift_ratio[i]/1.f,
               teinst->max_accel_var/1.f);

        if (fabs(st_shift_ratio[i]) > teinst->max_accel_var) {
            invdbg("ACCEL Fail Axis = %d\n", i);
            result |= 1 << i;	//Error condition
        }
    }
}
else {
    /* Self Test Pass/Fail Criteria B */
    accel_st_al_min = teinst->min_g * 65536.f;
    accel_st_al_max = teinst->max_g * 65536.f;

    invdbg("ACCEL:CRITERIA B\r\n");
    invdbg("Min MG: %7.4f\r\n", accel_st_al_min/1.f);
    invdbg("Max MG: %7.4f\r\n", accel_st_al_max/1.f);

    for (i = 0; i < 3; i++) {
        st_shift_cust[i] = bias_st[i] - bias_regular[i];

        invdbg("Bias_shift=%7.4f, st=%7.4f, reg=%7.4f\n", st_shift_cust[i]/1.f, bias_st[i]/1.f, bias_regular[i]/1.f);
        if(st_shift_cust[i] < accel_st_al_min || st_shift_cust[i] > accel_st_al_max) {
            invdbg("Accel FAIL axis:%d <= 225mg or >= 675mg\n", i);
            result |= 1 << i;	//Error condition
        }
    }
}

if(result == 0) {
    /* Self Test Pass/Fail Criteria C */
    accel_offset_max = teinst->max_g_offset * 65536.f;
    invdbg("Accel:CRITERIA C: bias less than %7.4f\n", accel_offset_max/1.f);
    for (i = 0; i < 3; i++) {
        if(fabs(bias_regular[i]) > accel_offset_max) {
            invdbg("FAILED: Accel axis:%d = %d > 500mg\n", i, bias_regular[i]);
            result |= 1 << i;	//Error condition
        }
    }
}

return result;
}

static int gyro_6500_self_test(long *bias_regular, long *bias_st)
{
    int i, result = 0, otp_value_zero = 0;
    float gyro_st_al_max;
    float st_shift_cust[3], st_shift_ratio[3], ct_shift_prod[3], gyro_offset_max;
    uint8_t regs[3];

    if (i2c_read(inst->hw->addr, REG_6500_XG_ST_DATA, 3, regs)) {
        invdbg("Reading OTP Register Error.\n");
        return 0x07;
    }

    invdbg("Gyro OTP:%d, %d, %d\r\n", regs[0], regs[1], regs[2]);

    for (i = 0; i < 3; i++) {
        if (regs[i] != 0) {
            ct_shift_prod[i] = mpu_6500_st_tb[regs[i] - 1];
            ct_shift_prod[i] *= 65536.f;
            ct_shift_prod[i] /= teinst->gyro_sens;
        }
        else {
            ct_shift_prod[i] = 0;
            otp_value_zero = 1;
        }
    }

    if(otp_value_zero == 0) {
        invdbg("GYRO:CRITERIA A\n");
        /* Self Test Pass/Fail Criteria A */
        for (i = 0; i < 3; i++) {
            st_shift_cust[i] = bias_st[i] - bias_regular[i];

            invdbg("Bias_Shift=%7.4f, Bias_Reg=%7.4f, Bias_HWST=%7.4f\r\n",
                   st_shift_cust[i]/1.f, bias_regular[i]/1.f,
                   bias_st[i]/1.f);
            invdbg("OTP value: %7.4f\r\n", ct_shift_prod[i]/1.f);
        }

        st_shift_ratio[i] = st_shift_cust[i] / ct_shift_prod[i];

        invdbg("ratio=%7.4f, threshold=%7.4f\r\n", st_shift_ratio[i]/1.f,
               teinst->max_gyro_var/1.f);

        if (fabs(st_shift_ratio[i]) < teinst->max_gyro_var) {
            invdbg("Gyro Fail Axis = %d\n", i);
            result |= 1 << i;	//Error condition
        }
    }
}
else {
    /* Self Test Pass/Fail Criteria B */
    gyro_st_al_max = teinst->max_dps * 65536.f;

    invdbg("GYRO:CRITERIA B\r\n");
    invdbg("Max DPS: %7.4f\r\n", gyro_st_al_max/1.f);
}

for (i = 0; i < 3; i++) {
    st_shift_cust[i] = bias_st[i] - bias_regular[i];

    invdbg("Bias_shift=%7.4f, st=%7.4f, reg=%7.4f\n", st_shift_cust[i]/1.f, bias_st[i]/1.f, bias_regular[i]/1.f);
    if(st_shift_cust[i] < gyro_st_al_max) {
        invdbg("GYRO FAIL axis:%d greater than 60dps\n", i);
        result |= 1 << i;	//Error condition
    }
}
}

if(result == 0) {
    /* Self Test Pass/Fail Criteria C */
    gyro_offset_max = teinst->min_dps * 65536.f;
    invdbg("Gyro:CRITERIA C: bias less than %7.4f\n", gyro_offset_max/1.f);
    for (i = 0; i < 3; i++) {
        if(fabs(bias_regular[i]) > gyro_offset_max) {
            invdbg("FAILED: Gyro axis:%d = %d > 20dps\n", i, bias_regular[i]);
            result |= 1 << i;	//Error condition
        }
    }
}
return result;
}

static int get_st_6500_biases(long *gyro, long *accel, uint8_t hw_test)
{
    uint8_t data[HWST_MAX_PACKET_LENGTH];
    uint8_t packet_count, ii;
    uint16_t fifo_count;
    int s = 0, read_size = 0, ind;

    data[0] = 0x01;
    data[1] = 0;
    if (mpu_write(inst,INV_MPU_PWR_MGMT_1,data,2) < 0 )
        return -1;
    up_mdelay(200);
    data[0] = 0;
    if (mpu_write(inst,INV_MPU_INT_ENABLE,data,1) < 0 )
        return -1;
    if (mpu_write(inst,INV_MPU_FIFO_EN,data,1) < 0 )
        return -1;
    if (mpu_write(inst,INV_MPU_PWR_MGMT_1,data,1) < 0 )
        return -1;
    if (mpu_write(inst,INV_MPU_I2C_MST,data,1) < 0 )
        return -1;
    if (mpu_write(inst,INV_MPU_USER_CTRL,data,1) < 0 )
        return -1;
    data[0] = BIT_FIFO_RST | BIT_DMP_RST;
    if (mpu_write(inst,INV_MPU_USER_CTRL,data,1) < 0 )
        return -1;
    up_mdelay(15);
    data[0] = inst->test->reg_lpf;
    if (mpu_write(inst,INV_MPU_LPF,data,1) < 0 )
        return -1;
    data[0] = inst->test->reg_rate_div;
    if (mpu_write(inst,INV_MPU_RATE_DIV,data,1) < 0 )
        return -1;
    if (hw_test)
        data[0] = inst->test->reg_gyro_fsr | 0xE0;
    else
        data[0] = inst->test->reg_gyro_fsr;
    if (mpu_write(inst,INV_MPU_GYRO_CFG,data,1) < 0 )
        return -1;

    if (hw_test)
        data[0] = inst->test->reg_accel_fsr | 0xE0;
    else
        data[0] = teinst->reg_accel_fsr;
    if (mpu_write(inst,INV_MPU_ACCEL_CFG,data,1) < 0 )
        return -1;

    up_mdelay(teinst->wait_ms);  //wait 200ms for sensors to stabilize

    /* Enable FIFO */
    data[0] = BIT_FIFO_EN;
    if (mpu_write(inst,INV_MPU_USER_CTRL,data,1) < 0 )
        return -1;
    data[0] = INV_XYZ_GYRO | INV_XYZ_ACCEL;
    if (mpu_write(inst,INV_MPU_FIFO_EN,data,1) < 0 )
        return -1;

    //initialize the bias return values
    gyro[0] = gyro[1] = gyro[2] = 0;
    accel[0] = accel[1] = accel[2] = 0;

    invdbg("Starting Bias Loop Reads\n");

    //start reading samples
    while (s < teinst->packet_thresh) {
        up_mdelay(teinst->sample_wait_ms); //wait 10ms to fill FIFO
        if (mpu_read(inst,fifo_count_h, 2, data) < 0 )
            return -1;
        fifo_count = (data[0] << 8) | data[1];
        packet_count = fifo_count / MAX_PACKET_LENGTH;
        if ((teinst->packet_thresh - s) < packet_count)
            packet_count = teinst->packet_thresh - s;
        read_size = packet_count * MAX_PACKET_LENGTH;

        //burst read from FIFO
        if (mpu_read(inst,fifo_r_w, read_size, data) < 0 )
            return -1;
        ind = 0;
        for (ii = 0; ii < packet_count; ii++) {
            int16_t accel_cur[3], gyro_cur[3];
            accel_cur[0] = ((int16_t)data[ind + 0] << 8) | data[ind + 1];
            accel_cur[1] = ((int16_t)data[ind + 2] << 8) | data[ind + 3];
            accel_cur[2] = ((int16_t)data[ind + 4] << 8) | data[ind + 5];
            accel[0] += (long)accel_cur[0];
            accel[1] += (long)accel_cur[1];
            accel[2] += (long)accel_cur[2];
            gyro_cur[0] = (((int16_t)data[ind + 6] << 8) | data[ind + 7]);
            gyro_cur[1] = (((int16_t)data[ind + 8] << 8) | data[ind + 9]);
            gyro_cur[2] = (((int16_t)data[ind + 10] << 8) | data[ind + 11]);
            gyro[0] += (long)gyro_cur[0];
            gyro[1] += (long)gyro_cur[1];
            gyro[2] += (long)gyro_cur[2];
            ind += MAX_PACKET_LENGTH;
        }
        s += packet_count;
    }

    invdbg("Samples: %d\n", s);

    //stop FIFO
    data[0] = 0;
    if (mpu_write(inst,INV_MPU_FIFO_EN,data,1) < 0 )
        return -1;

    gyro[0] = (long)(((long long)gyro[0]<<16) / teinst->gyro_sens / s);
    gyro[1] = (long)(((long long)gyro[1]<<16) / teinst->gyro_sens / s);
    gyro[2] = (long)(((long long)gyro[2]<<16) / teinst->gyro_sens / s);
    accel[0] = (long)(((long long)accel[0]<<16) / teinst->accel_sens / s);
    accel[1] = (long)(((long long)accel[1]<<16) / teinst->accel_sens / s);
    accel[2] = (long)(((long long)accel[2]<<16) / teinst->accel_sens / s);
    /* remove gravity from bias calculation */
    if (accel[2] > 0L)
        accel[2] -= 65536L;
    else
        accel[2] += 65536L;


    invdbg("Accel offset data HWST bit=%d: %7.4f %7.4f %7.4f\r\n", hw_test, accel[0]/65536.f, accel[1]/65536.f, accel[2]/65536.f);
    invdbg("Gyro offset data HWST bit=%d: %7.4f %7.4f %7.4f\r\n", hw_test, gyro[0]/65536.f, gyro[1]/65536.f, gyro[2]/65536.f);

    return 0;
}
/*
 *  Name: mpu_run_6500_self_test
 *  
 *  Trigger gyro/accel/compass self-test for MPU6500/MPU9250
 *  On success/error, the self-test returns a mask representing the sensor(s)
 *  that failed. For each bit, a one (1) represents a "pass" case; conversely,
 *  a zero (0) indicates a failure.
 *
 *  \n The mask is defined as follows:
 *  \n Bit 0:   Gyro.
 *  \n Bit 1:   Accel.
 *  \n Bit 2:   Compass.
 *
 * Note this following register will be modified Save and restore by your self
 *  accel_fsr 
 *  fifo_sensors 
 *  sensors_on 
 *  gyro_fsr 
 *  sample_rate 
 *  lpf
 *  dmp_was_on
 * 
 *  gyro        Gyro biases in q16 format.
 *  accel       Accel biases (if applicable) in q16 format.
 *  Result mask (see above).
 */
int mpu_run_6500_self_test(long *gyro, long *accel)
{
    const uint8_t tries = 2;
    long gyro_st[3], accel_st[3];
    uint8_t accel_result, gyro_result;
#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
    uint8_t compass_result;
#endif
    int ii;

    int result;
    uint8_t accel_fsr, fifo_sensors, sensors_on;
    uint16_t gyro_fsr, sample_rate, lpf;
    uint8_t dmp_was_on;




    invdbg("Starting MPU6500 HWST!\r\n");

    if (inst->chip_cfg.dmp_on) {
        mpu_set_dmp_state(false);
        dmp_was_on = 1;
    } else
        dmp_was_on = 0;

    /* Get initial settings. */
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    mpu_get_lpf(&lpf);
    mpu_get_sample_rate(&sample_rate);
    sensors_on = inst->chip_cfg.sensors;
    mpu_get_fifo_config(&fifo_sensors);

    invdbg("Retrieving Biases\r\n");

    for (ii = 0; ii < tries; ii++)
        if (!get_st_6500_biases(gyro, accel, 0))
            break;
    if (ii == tries) {
        /* If we reach this point, we most likely encountered an I2C error.
         * We'll just report an error for all three sensors.
         */
        invdbg("Retrieving Biases Error - possible I2C error\n");

        result = 0;
        goto restore;
    }

    invdbg("Retrieving ST Biases\n");

    for (ii = 0; ii < tries; ii++)
        if (!get_st_6500_biases(gyro_st, accel_st, 1))
            break;
    if (ii == tries) {

        invdbg("Retrieving ST Biases Error - possible I2C error\n");

        /* Again, probably an I2C error. */
        result = 0;
        goto restore;
    }

    accel_result = accel_6500_self_test(accel, accel_st);
    invdbg("Accel Self Test Results: %d\n", accel_result);

    gyro_result = gyro_6500_self_test(gyro, gyro_st);
    invdbg("Gyro Self Test Results: %d\n", gyro_result);

    result = 0;
    if (!gyro_result)
        result |= 0x01;
    if (!accel_result)
        result |= 0x02;

#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
    compass_result = compass_self_test();
    invdbg("Compass Self Test Results: %d\n", compass_result);
    if (!compass_result)
        result |= 0x04;
#else
    result |= 0x04;
#endif
restore:
    invdbg("Exiting HWST\n");
    /* Set to invalid values to ensure no I2C writes are skipped. */
	inst->chip_cfg.gyro_fsr = 0xFF;
	inst->chip_cfg.accel_fsr = 0xFF;
	inst->chip_cfg.lpf = 0xFF;
	inst->chip_cfg.sample_rate = 0xFFFF;
	inst->chip_cfg.sensors = 0xFF;
	inst->chip_cfg.fifo_enable = 0xFF;
	inst->chip_cfg.clk_src = INV_CLK_PLL;
	mpu_set_gyro_fsr(gyro_fsr);
	mpu_set_accel_fsr(accel_fsr);
	mpu_set_lpf(lpf);
	mpu_set_sample_rate(sample_rate);
	mpu_set_sensors(sensors_on);
	mpu_configure_fifo(fifo_sensors);

    if (dmp_was_on)
        mpu_set_dmp_state(true);

    return result;
}
#endif
/*
 *  \n This function must be called with the device either face-up or face-down
 *  (z-axis is parallel to gravity).
 *  @param[out] gyro        Gyro biases in q16 format.
 *  @param[out] accel       Accel biases (if applicable) in q16 format.
 *  @return     Result mask (see above).
 */
int mpu_run_self_test(long *gyro, long *accel)
{
#ifdef CONFIG_SENSOR_MPU6050
    const uint8_t tries = 2;
    long gyro_st[3], accel_st[3];
    uint8_t accel_result, gyro_result;
#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
    uint8_t compass_result;
#endif
    int ii;
#endif
    int result;
    uint8_t accel_fsr, fifo_sensors, sensors_on;
    uint16_t gyro_fsr, sample_rate, lpf;
    uint8_t dmp_was_on;

    if (inst->chip_cfg.dmp_on) {
        mpu_set_dmp_state(false);
        dmp_was_on = 1;
    } else
        dmp_was_on = 0;

    /* Get initial settings. */
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    mpu_get_lpf(&lpf);
    mpu_get_sample_rate(&sample_rate);
    sensors_on = inst->chip_cfg.sensors;
    mpu_get_fifo_config(&fifo_sensors);

    /* For older chips, the self-test will be different. */
#if defined CONFIG_SENSOR_MPU6050
    for (ii = 0; ii < tries; ii++)
        if (!get_st_biases(gyro, accel, 0))
            break;
    if (ii == tries) {
        /* If we reach this point, we most likely encountered an I2C error.
         * We'll just report an error for all three sensors.
         */
        result = 0;
        goto restore;
    }
    for (ii = 0; ii < tries; ii++)
        if (!get_st_biases(gyro_st, accel_st, 1))
            break;
    if (ii == tries) {
        /* Again, probably an I2C error. */
        result = 0;
        goto restore;
    }
    accel_result = accel_self_test(accel, accel_st);
    gyro_result = gyro_self_test(gyro, gyro_st);

    result = 0;
    if (!gyro_result)
        result |= 0x01;
    if (!accel_result)
        result |= 0x02;

#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
    compass_result = compass_self_test();
    if (!compass_result)
        result |= 0x04;
#else
    result |= 0x04;
#endif
restore:
#elif defined MPU6500
    /* For now, this function will return a "pass" result for all three sensors
     * for compatibility with current test applications.
     */
    get_st_biases(gyro, accel, 0);
    result = 0x7;
#endif
    /* Set to invalid values to ensure no I2C writes are skipped. */
    inst->chip_cfg.gyro_fsr = 0xFF;
    inst->chip_cfg.accel_fsr = 0xFF;
    inst->chip_cfg.lpf = 0xFF;
    inst->chip_cfg.sample_rate = 0xFFFF;
    inst->chip_cfg.sensors = 0xFF;
    inst->chip_cfg.fifo_enable = 0xFF;
    inst->chip_cfg.clk_src = INV_CLK_PLL;
    mpu_set_gyro_fsr(gyro_fsr);
    mpu_set_accel_fsr(accel_fsr);
    mpu_set_lpf(lpf);
    mpu_set_sample_rate(sample_rate);
    mpu_set_sensors(sensors_on);
    mpu_configure_fifo(fifo_sensors);

    if (dmp_was_on)
        mpu_set_dmp_state(true);

    return result;
}

/**
 *  @brief      Write to the DMP memory.
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data        Bytes to write to memory.
 *  @return     0 if successful.
 */
int mpu_write_mem(uint16_t mem_addr, uint16_t length,
                  uint8_t *data)
{
    uint8_t tmp[2];

    if (!data)
        return -1;
    if (!inst->chip_cfg.sensors)
        return -1;

    tmp[0] = (uint8_t)(mem_addr >> 8);
    tmp[1] = (uint8_t)(mem_addr & 0xFF);

    /* Check bank boundaries. */
    if (tmp[1] + length > inst->hw->bank_size)
        return -1;

    if (mpu_write(inst,INV_MPU_BANK_SEL,tmp,2) < 0 )
        return -1;
    if (mpu_write(inst,INV_MPU_MEM_R_W,data,length) < 0 )
        return -1;
    return 0;
}

/**
 *  @brief      Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Bytes read from memory.
 *  @return     0 if successful.
 */
int mpu_read_mem(uint16_t mem_addr, uint16_t length,
                 uint8_t *data)
{
    uint8_t tmp[2];

    if (!data)
        return -1;
    if (!inst->chip_cfg.sensors)
        return -1;

    tmp[0] = (uint8_t)(mem_addr >> 8);
    tmp[1] = (uint8_t)(mem_addr & 0xFF);

    /* Check bank boundaries. */
    if (tmp[1] + length > inst->hw->bank_size)
        return -1;

    if (mpu_write(inst,INV_MPU_BANK_SEL,tmp,2) < 0 )
        return -1;
    if (mpu_read(inst,mem_r_w, length, data) < 0 )
        return -1;
    return 0;
}

/**
 *  @brief      Load and verify DMP image.
 *  @param[in]  length      Length of DMP image.
 *  @param[in]  firmware    DMP code.
 *  @param[in]  start_addr  Starting address of DMP code memory.
 *  @param[in]  sample_rate Fixed sampling rate used when DMP is enabled.
 *  @return     0 if successful.
 */
int mpu_load_firmware(uint16_t length, const uint8_t *firmware,
                      uint16_t start_addr, uint16_t sample_rate)
{
    uint16_t ii;
    uint16_t this_write;
    /* Must divide evenly into inst->hw->bank_size to avoid bank crossings. */
#define LOAD_CHUNK  (16)
    uint8_t cur[LOAD_CHUNK], tmp[2];

    if (inst->chip_cfg.dmp_loaded)
        /* DMP should only be loaded once. */
        return -1;

    if (!firmware)
        return -1;
    for (ii = 0; ii < length; ii += this_write) {
        this_write = min(LOAD_CHUNK, length - ii);
        if (mpu_write_mem(ii, this_write, (uint8_t*)&firmware[ii]))
            return -1;
        if (mpu_read_mem(ii, this_write, cur))
            return -1;
        if (memcmp(firmware+ii, cur, this_write))
            return -2;
    }

    /* Set program start address. */
    tmp[0] = start_addr >> 8;
    tmp[1] = start_addr & 0xFF;
    if (mpu_write(inst,INV_MPU_PRGM_START_H,tmp,2) < 0 )
        return -1;

    inst->chip_cfg.dmp_loaded = 1;
    inst->chip_cfg.dmp_sample_rate = sample_rate;
    return 0;
}

/**
 *  @brief      Enable/disable DMP support.
 *  @param[in]  enable  1 to turn on the DMP.
 *  @return     0 if successful.
 */
int mpu_set_dmp_state(bool enable)
{
    uint8_t tmp;
    if (inst->chip_cfg.dmp_on == enable)
        return 0;

    if (enable) {
        if (!inst->chip_cfg.dmp_loaded)
            return -1;
        /* Disable data ready interrupt. */
        set_int_enable(0);
        /* Disable bypass mode. */
        mpu_set_bypass(0);
        /* Keep constant sample rate, FIFO rate controlled by DMP. */
        mpu_set_sample_rate(inst->chip_cfg.dmp_sample_rate);
        /* Remove FIFO elements. */
        tmp = 0;
        i2c_write(inst->hw->addr, 0x23, 1, &tmp);
        inst->chip_cfg.dmp_on = 1;
        /* Enable DMP interrupt. */
        set_int_enable(1);
        mpu_reset_fifo();
    } else {
        /* Disable DMP interrupt. */
        set_int_enable(0);
        /* Restore FIFO settings. */
        tmp = inst->chip_cfg.fifo_enable;
        i2c_write(inst->hw->addr, 0x23, 1, &tmp);
        inst->chip_cfg.dmp_on = 0;
        mpu_reset_fifo();
    }
    return 0;
}

/**
 *  @brief      Get DMP state.
 *  @param[out] enabled 1 if enabled.
 *  @return     0 if successful.
 */
int mpu_get_dmp_state(bool *enabled)
{
    enabled[0] = inst->chip_cfg.dmp_on;
    return 0;
}

#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
/* This initialization is similar to the one in ak8975.c. */
static int setup_compass(void)
{
    uint8_t data[4], akm_addr;

    mpu_set_bypass(1);

    /* Find compass. Possible addresses range from 0x0C to 0x0F. */
    for (akm_addr = 0x0C; akm_addr <= 0x0F; akm_addr++) {
        int result;
        result = i2c_read(akm_addr, AKM_REG_WHOAMI, 1, data);
        if (!result && (data[0] == AKM_WHOAMI))
            break;
    }

    if (akm_addr > 0x0F) {
        /* TODO: Handle this case in all compass-related functions. */
        log_e("Compass not found.\n");
        return -1;
    }

    inst->chip_cfg.compass_addr = akm_addr;

    data[0] = AKM_POWER_DOWN;
    if (i2c_write(inst->chip_cfg.compass_addr, AKM_REG_CNTL, 1, data))
        return -1;
    up_mdelay(1);

    data[0] = AKM_FUSE_ROM_ACCESS;
    if (i2c_write(inst->chip_cfg.compass_addr, AKM_REG_CNTL, 1, data))
        return -1;
    up_mdelay(1);

    /* Get sensitivity adjustment data from fuse ROM. */
    if (i2c_read(inst->chip_cfg.compass_addr, AKM_REG_ASAX, 3, data))
        return -1;

    data[0] = AKM_POWER_DOWN;
    if (i2c_write(inst->chip_cfg.compass_addr, AKM_REG_CNTL, 1, data))
        return -1;
    up_mdelay(1);

    mpu_set_bypass(0);

    /* Set up master mode, master clock, and ES bit. */
    data[0] = 0x40;
    if (mpu_write(inst,INV_MPU_I2C_MST,data,1) < 0 )
        return -1;

    /* Slave 0 reads from AKM data registers. */
    data[0] = BIT_I2C_READ | inst->chip_cfg.compass_addr;
    if (mpu_write(inst,INV_MPU_S0_ADDR,data,1) < 0 )
        return -1;

    /* Compass reads start at this register. */
    data[0] = AKM_REG_ST1;
    if (mpu_write(inst,INV_MPU_S0_REG,data,1) < 0 )
        return -1;

    /* Enable slave 0, 8-byte reads. */
    data[0] = BIT_SLAVE_EN | 8;
    if (mpu_write(inst,INV_MPU_S0_CTRL,data,1) < 0 )
        return -1;

    /* Slave 1 changes AKM measurement mode. */
    data[0] = inst->chip_cfg.compass_addr;
    if (mpu_write(inst,INV_MPU_S1_ADDR,data,1) < 0 )
        return -1;

    /* AKM measurement mode register. */
    data[0] = AKM_REG_CNTL;
    if (mpu_write(inst,INV_MPU_S1_REG,data,1) < 0 )
        return -1;

    /* Enable slave 1, 1-byte writes. */
    data[0] = BIT_SLAVE_EN | 1;
    if (mpu_write(inst,INV_MPU_S1_CTRL,data,1) < 0 )
        return -1;

    /* Set slave 1 data. */
    data[0] = AKM_SINGLE_MEASUREMENT;
    if (mpu_write(inst,INV_MPU_S1_DO,data,1) < 0 )
        return -1;

    /* Trigger slave 0 and slave 1 actions at each sample. */
    data[0] = 0x03;
    if (mpu_write(inst,INV_MPU_I2C_DELAY_CTRL,data,1) < 0 )
        return -1;

#ifdef MPU9150
    /* For the MPU9150, the auxiliary I2C bus needs to be set to VDD. */
    data[0] = BIT_I2C_MST_VDDIO;
    if (mpu_write(inst,INV_MPU_YG_OFFS_TC,data,1) < 0 )
        return -1;
#endif

    return 0;
}
#endif


/**
 *  @brief      Get the compass full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_compass_fsr(uint16_t *fsr)
{
#ifdef CONFIG_SENSOR_AK89XX_SECONDARY
    fsr[0] = INV_AK89_FSR;
    return 0;
#else
    return -1;
#endif
}

/**
 *  @brief      Enters LP accel motion interrupt mode.
 *  The behaviour of this feature is very different between the MPU6050 and the
 *  MPU6500. Each chip's version of this feature is explained below.
 *
 *  \n The hardware motion threshold can be between 32mg and 8160mg in 32mg
 *  increments.
 *
 *  \n Low-power accel mode supports the following frequencies:
 *  \n 1.25Hz, 5Hz, 20Hz, 40Hz
 *
 *  \n MPU6500:
 *  \n Unlike the MPU6050 version, the hardware does not "lock in" a reference
 *  sample. The hardware monitors the accel data and detects any large change
 *  over a int16_t period of time.
 *
 *  \n The hardware motion threshold can be between 4mg and 1020mg in 4mg
 *  increments.
 *
 *  \n MPU6500 Low-power accel mode supports the following frequencies:
 *  \n 1.25Hz, 2.5Hz, 5Hz, 10Hz, 20Hz, 40Hz, 80Hz, 160Hz, 320Hz, 640Hz
 *
 *  \n\n NOTES:
 *  \n The driver will round down @e thresh to the nearest supported value if
 *  an unsupported threshold is selected.
 *  \n To select a fractional wake-up frequency, round down the value passed to
 *  @e lpa_freq.
 *  \n The MPU6500 does not support a delay parameter. If this function is used
 *  for the MPU6500, the value passed to @e time will be ignored.
 *  \n To disable this mode, set @e lpa_freq to zero. The driver will restore
 *  the previous configuration.
 *
 *  @param[in]  thresh      Motion threshold in mg.
 *  @param[in]  time        Duration in milliseconds that the accel data must
 *                          exceed @e thresh before motion is reported.
 *  @param[in]  lpa_freq    Minimum sampling rate, or zero to disable.
 *  @return     0 if successful.
 */
int mpu_lp_motion_interrupt(struct mpu_inst_s* inst,uint16_t thresh, 
                            uint8_t time, uint8_t lpa_freq)
{

#if defined MPU6500
    uint8_t data[3];
#endif
    if (lpa_freq) {
#if defined MPU6500
        uint8_t thresh_hw;

        /* 1LSb = 4mg. */
        if (thresh > 1020)
            thresh_hw = 255;
        else if (thresh < 4)
            thresh_hw = 1;
        else
            thresh_hw = thresh >> 2;
#endif

        if (!time)
            /* Minimum duration must be 1ms. */
            time = 1;

#if defined MPU6500
        if (lpa_freq > 640)
            /* At this point, the chip has not been re-configured, so the
             * function can safely exit.
             */
            return -1;
#endif

        if (!inst->chip_cfg.int_motion_only) {
            /* Store current settings for later. */
            if (inst->chip_cfg.dmp_on) {
                mpu_set_dmp_state(false);
                inst->chip_cfg.cache.dmp_on = 1;
            } else
                inst->chip_cfg.cache.dmp_on = 0;
            mpu_get_gyro_fsr(&inst->chip_cfg.cache.gyro_fsr);
            mpu_get_accel_fsr(&inst->chip_cfg.cache.accel_fsr);
            mpu_get_lpf(&inst->chip_cfg.cache.lpf);
            mpu_get_sample_rate(&inst->chip_cfg.cache.sample_rate);
            inst->chip_cfg.cache.sensors_on = inst->chip_cfg.sensors;
            mpu_get_fifo_config(&inst->chip_cfg.cache.fifo_sensors);
        }

#if defined MPU6500
        /* Disable hardware interrupts. */
        set_int_enable(0);

        /* Enter full-power accel-only mode, no FIFO/DMP. */
        data[0] = 0;
        data[1] = 0;
        data[2] = BIT_STBY_XYZG;
        if (mpu_write(inst,INV_MPU_USER_CTRL,data,3) < 0 )
            goto lp_int_restore;

        /* Set motion threshold. */
        data[0] = thresh_hw;
        if (mpu_write(inst,INV_MPU_MOTION_THR,data,1) < 0 )
            goto lp_int_restore;

        /* Set wake frequency. */
        if (lpa_freq == 1)
            data[0] = INV_LPA_1_25HZ;
        else if (lpa_freq == 2)
            data[0] = INV_LPA_2_5HZ;
        else if (lpa_freq <= 5)
            data[0] = INV_LPA_5HZ;
        else if (lpa_freq <= 10)
            data[0] = INV_LPA_10HZ;
        else if (lpa_freq <= 20)
            data[0] = INV_LPA_20HZ;
        else if (lpa_freq <= 40)
            data[0] = INV_LPA_40HZ;
        else if (lpa_freq <= 80)
            data[0] = INV_LPA_80HZ;
        else if (lpa_freq <= 160)
            data[0] = INV_LPA_160HZ;
        else if (lpa_freq <= 320)
            data[0] = INV_LPA_320HZ;
        else
            data[0] = INV_LPA_640HZ;
        if (mpu_write(inst,INV_MPU_LP_ACCEL_ODR,data,1) < 0 )
            goto lp_int_restore;

        /* Enable motion interrupt (MPU6500 version). */
        data[0] = BITS_WOM_EN;
        if (mpu_write(inst,INV_MPU_ACCEL_INTEL,data,1) < 0 )
            goto lp_int_restore;

        /* Enable cycle mode. */
        data[0] = BIT_LPA_CYCLE;
        if (mpu_write(inst,INV_MPU_PWR_MGMT_1,data,1) < 0 )
            goto lp_int_restore;

        /* Enable interrupt. */
        data[0] = BIT_MOT_INT_EN;
        if (mpu_write(inst,INV_MPU_INT_ENABLE,data,1) < 0 )
            goto lp_int_restore;

        inst->chip_cfg.int_motion_only = 1;
        return 0;
#endif
    } else {
        /* Don't "restore" the previous state if no state has been saved. */
        int ii;
        char *cache_ptr = (char*)&inst->chip_cfg.cache;
        for (ii = 0; ii < sizeof(inst->chip_cfg.cache); ii++) {
            if (cache_ptr[ii] != 0)
                goto lp_int_restore;
        }
        /* If we reach this point, motion interrupt mode hasn't been used yet. */
        return -1;
    }
lp_int_restore:
    /* Set to invalid values to ensure no I2C writes are skipped. */
    inst->chip_cfg.gyro_fsr = 0xFF;
    inst->chip_cfg.accel_fsr = 0xFF;
    inst->chip_cfg.lpf = 0xFF;
    inst->chip_cfg.sample_rate = 0xFFFF;
    inst->chip_cfg.sensors = 0xFF;
    inst->chip_cfg.fifo_enable = 0xFF;
    inst->chip_cfg.clk_src = INV_CLK_PLL;
    mpu_set_sensors(inst->chip_cfg.cache.sensors_on);
    mpu_set_gyro_fsr(inst->chip_cfg.cache.gyro_fsr);
    mpu_set_accel_fsr(inst->chip_cfg.cache.accel_fsr);
    mpu_set_lpf(inst->chip_cfg.cache.lpf);
    mpu_set_sample_rate(inst->chip_cfg.cache.sample_rate);
    mpu_configure_fifo(inst->chip_cfg.cache.fifo_sensors);

    if (inst->chip_cfg.cache.dmp_on)
        mpu_set_dmp_state(true);

#ifdef MPU6500
    /* Disable motion interrupt (MPU6500 version). */
    data[0] = 0;
    if (mpu_write(inst,INV_MPU_ACCEL_INTEL,data,1) < 0 )
        goto lp_int_restore;
#endif

    inst->chip_cfg.int_motion_only = 0;
    return 0;
}


#endif /* (( defined CONFIG_SENSOR_MPU6050)||(defined CONFIG_SENSOR_MPU9150)||
        * ( defined CONFIG_SENSOR_MPU6500 )||(defined CONFIG_SENSOR_MPU9250))
        */
