/********************************************************************************************
 * include/nuttx/sensors/mpu9250.h
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_MPU9250_H
#define __INCLUDE_NUTTX_SENSORS_MPU9250_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <nuttx/i2c.h>
#include <nuttx/spi/spi.h>

#include <nuttx/irq.h>

#if defined(CONFIG_SENSORS_MPU9250)

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/
/* Configuration ****************************************************************************/
/* Prerequisites:
 *
 * CONFIG_SENSORS_MPU9250
 *   Enables support for the MPU9250 driver
 * CONFIG_MPU9250_SPI
 *   Enables support for the SPI interface (not currenly tested)
 * CONFIG_MPU9250_I2C
 *   Enables support for the I2C interface
 * CONFIG_MPU9250_ACTIVELOW
 *    The MPU9250 interrupt will be inverted. Instead starting low and
 *    going high, it will start high and will go low when an interrupt
 *    is fired. Default:  Active high/rising edge.
 * CONFIG_MPU9250_REGDEBUG
 *   Enable very low register-level debug output.  Requires CONFIG_DEBUG.
 */

/* The MPU9250 interfaces with the target CPU via a I2C or SPI interface. The pin IN_1
 * allows the selection of interface protocol at reset state.
 */

#if !defined(CONFIG_MPU9250_SPI) && !defined(CONFIG_MPU9250_I2C)
#  error "One of CONFIG_MPU9250_SPI or CONFIG_MPU9250_I2C must be defined"
#endif

#if defined(CONFIG_MPU9250_SPI) && defined(CONFIG_MPU9250_I2C)
#  error "Only one of CONFIG_MPU9250_SPI or CONFIG_MPU9250_I2C can be defined"
#endif

/* Check for some required settings.  This can save the user a lot of time
 * in getting the right configuration.
 */

#ifdef CONFIG_MPU9250_I2C
#  ifndef CONFIG_I2C
#    error "CONFIG_I2C is required in the I2C support"
#  endif
#  ifndef CONFIG_I2C_TRANSFER
#    error "CONFIG_I2C_TRANSFER is required in the I2C configuration"
#  endif
#endif

/* I2C frequency */

#define MPU9250_I2C_MAXFREQUENCY    400000       /* 400KHz */

/* SPI **************************************************************************************/
/* The device always operates in mode 0 */

#define MPU9250_SPI_MODE            SPIDEV_MODE0 /* Mode 0 */

/* SPI frequency */

#define MPU9250_SPI_MAXFREQUENCY    5000000       /* 5MHz */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/* Form of the GPIO "interrupt handler" callback. Callbacks do not occur from an interrupt
 * handler but rather from the context of the worker thread with interrupts enabled.
 */

typedef void (*mpu9250_handler_t)(FAR struct mpu9250_config_s *config, FAR void *arg);

/* A reference to a structure of this type must be passed to the MPU9250 driver when the
 * driver is instantiated. This structure provides information about the configuration of the
 * MPU9250 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by the driver
 * and is presumed to persist while the driver is active. The memory must be writable
 * because, under certain circumstances, the driver may modify the frequency.
 */

struct mpu9250_config_s
{
  /* Device characterization */

  uint32_t frequency;  /* I2C or SPI frequency */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the MPU9250 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach the MPU9250 interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   * clear   - Acknowledge/clear any pending GPIO interrupt
   */

  int  (*attach)(FAR struct mpu9250_config_s *state, mpu9250_handler_t handler,
                 FAR void *arg);
  void (*enable)(FAR struct mpu9250_config_s *state, bool enable);
  void (*clear)(FAR struct mpu9250_config_s *state);
};

typedef FAR void *MPU9250_HANDLE;

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/********************************************************************************************
 * Name: mpu9250_instantiate
 *
 * Description:
 *   Instantiate and configure the MPU9250 device driver to use the provided I2C or SPI
 *   device instance.
 *
 * Input Parameters:
 *   dev     - An I2C or SPI driver instance
 *   config  - Persistant board configuration data
 *
 * Returned Value:
 *   A non-zero handle is returned on success.  This handle may then be used to configure
 *   the MPU9250 driver as necessary.  A NULL handle value is returned on failure.
 *
 ********************************************************************************************/

#ifdef CONFIG_MPU9250_SPI
EXTERN MPU9250_HANDLE mpu9250_instantiate(FAR struct spi_dev_s *dev,
                                            FAR struct mpu9250_config_s *config);
#else
EXTERN MPU9250_HANDLE mpu9250_instantiate(FAR struct i2c_dev_s *dev,
                                            FAR struct mpu9250_config_s *config);
#endif

/********************************************************************************************
 * Name: mpu9250_register
 *
 * Description:
 *  This function will register the accelerometer driver
 *  where N is the minor device number
 *
 * Input Parameters:
 *   handle    - The handle previously returned by mpu9250_instantiate
 *   path      - path to register this device
 *   minor     - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is returned to indicate
 *   the nature of the failure.
 *
 ********************************************************************************************/

EXTERN int mpu9250_register(MPU9250_HANDLE handle,const char* path, int minor);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_MPU9250 */
#endif /* __INCLUDE_NUTTX_INPUT_MPU9250_H */
