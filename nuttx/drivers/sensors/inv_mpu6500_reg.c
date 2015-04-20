/****************************************************************************
 * drivers/sensors/inv_mpu6500_reg.c
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
#include <nuttx/arch.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "nuttx/sensors/inv_mpu.h"


/*******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

#if ( ( defined CONFIG_SENSOR_MPU6500 ) || \
      ( defined CONFIG_SENSOR_MPU9250 ) )

/* Configuration **************************************************************/

#include "inv_mpu6500_reg.h"

/* Debug **********************************************************************/

#  define regdbg  dbg

/* BIT MASK *******************************************************************/


/*******************************************************************************
 * Private Types
 ******************************************************************************/

const struct mpu_reg_desc_s mpu6500_reg_desc[] =
{
    {"INV_MPU_SELFTEST_X_GYRO",     INV_MPU_SELFTEST_X_GYRO     },
    {"INV_MPU_SELFTEST_Y_GYRO",     INV_MPU_SELFTEST_Y_GYRO     },
    {"INV_MPU_SELFTEST_Z_GYRO",     INV_MPU_SELFTEST_Z_GYRO     },

    {"INV_MPU_SELFTEST_X_ACCEL",    INV_MPU_SELFTEST_X_ACCEL    },
    {"INV_MPU_SELFTEST_Y_ACCEL",    INV_MPU_SELFTEST_Y_ACCEL    },
    {"INV_MPU_SELFTEST_Z_ACCEL",    INV_MPU_SELFTEST_Z_ACCEL    },

    {"INV_MPU_WHO_AM_I",        INV_MPU_WHO_AM_I        },
    {"INV_MPU_RATE_DIV",        INV_MPU_RATE_DIV        },
    {"INV_MPU_LPF",             INV_MPU_LPF             },
    {"INV_MPU_PROD_ID",         INV_MPU_PROD_ID         },
    {"INV_MPU_USER_CTRL",       INV_MPU_USER_CTRL       },
    {"INV_MPU_FIFO_EN",         INV_MPU_FIFO_EN         },
    {"INV_MPU_GYRO_CFG",        INV_MPU_GYRO_CFG        },
    {"INV_MPU_ACCEL_CFG",       INV_MPU_ACCEL_CFG       },
    {"INV_MPU_ACCEL_CFG2",      INV_MPU_ACCEL_CFG2      },
    {"INV_MPU_LP_ACCEL_ODR",    INV_MPU_LP_ACCEL_ODR    },
    {"INV_MPU_MOTION_THR",      INV_MPU_MOTION_THR      },
    {"INV_MPU_MOTION_DUR",      INV_MPU_MOTION_DUR      },
    {"INV_MPU_FIFO_COUNT_H",    INV_MPU_FIFO_COUNT_H    },
    {"INV_MPU_FIFO_R_W",        INV_MPU_FIFO_R_W        },
    {"INV_MPU_RAW_GYRO",        INV_MPU_RAW_GYRO        },
    {"INV_MPU_RAW_ACCEL",       INV_MPU_RAW_ACCEL       },
    {"INV_MPU_TEMP",            INV_MPU_TEMP            },
    {"INV_MPU_INT_ENABLE",      INV_MPU_INT_ENABLE      },
    {"INV_MPU_DMP_INT_STATUS",  INV_MPU_DMP_INT_STATUS  },
    {"INV_MPU_INT_STATUS",      INV_MPU_INT_STATUS      },
    {"INV_MPU_ACCEL_INTEL",     INV_MPU_ACCEL_INTEL     },
    {"INV_MPU_PWR_MGMT_1",      INV_MPU_PWR_MGMT_1      },
    {"INV_MPU_PWR_MGMT_2",      INV_MPU_PWR_MGMT_2      },
    {"INV_MPU_INT_PIN_CFG",     INV_MPU_INT_PIN_CFG     },
    {"INV_MPU_MEM_R_W",         INV_MPU_MEM_R_W         },

    {"INV_MPU_I2C_MST",         INV_MPU_I2C_MST         },
    {"INV_MPU_BANK_SEL",        INV_MPU_BANK_SEL        },
    {"INV_MPU_MEM_START_ADDR",  INV_MPU_MEM_START_ADDR  },
    {"INV_MPU_PRGM_START_H",    INV_MPU_PRGM_START_H    },

    /* Gyroscope offset */

    {"INV_MPU_XG_OFFSET_H",     INV_MPU_XG_OFFSET_H     },
    {"INV_MPU_XG_OFFSET_L",     INV_MPU_XG_OFFSET_L     },
    {"INV_MPU_YG_OFFSET_H",     INV_MPU_YG_OFFSET_H     },
    {"INV_MPU_YG_OFFSET_L",     INV_MPU_YG_OFFSET_L     },
    {"INV_MPU_ZG_OFFSET_H",     INV_MPU_ZG_OFFSET_H     },
    {"INV_MPU_ZG_OFFSET_L",     INV_MPU_ZG_OFFSET_L     },

    /* Accelerometer offset */

    {"INV_MPU_XA_OFFSET_H",     INV_MPU_XA_OFFSET_H     },
    {"INV_MPU_XA_OFFSET_L",     INV_MPU_XA_OFFSET_L     },
    {"INV_MPU_YA_OFFSET_H",     INV_MPU_YA_OFFSET_H     },
    {"INV_MPU_YA_OFFSET_L",     INV_MPU_YA_OFFSET_L     },
    {"INV_MPU_ZA_OFFSET_H",     INV_MPU_ZA_OFFSET_H     },
    {"INV_MPU_ZA_OFFSET_L",     INV_MPU_ZA_OFFSET_L     },
    {"",                        0x00                    }
};


#endif /* ((defined CONFIG_SENSOR_MPU6500)||(defined CONFIG_SENSOR_MPU9250)) */
