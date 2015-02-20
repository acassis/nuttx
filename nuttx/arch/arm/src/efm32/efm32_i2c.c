/************************************************************************************
 * arch/arm/src/efm32/efm32_i2c.c
 * EFM32 I2C Hardware Layer - Device Driver
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
 ******************************************************************************/

/* Supports:
 *  - Master operation, 100 kHz (standard) and 400 kHz (full speed)
 * TODO:
 *  - Multiple instances (shared bus)
 *  - Interrupt based operation
 *
 * Structure naming:
 *  - Device: structure as defined by the nuttx/i2c/i2c.h
 *  - Instance: represents each individual access to the I2C driver, obtained by
 *     the i2c_init(); it extends the Device structure from the nuttx/i2c/i2c.h;
 *     Instance points to OPS, to common I2C Hardware private data and contains
 *     its own private data, as frequency, address, mode of operation (in the
 *     future)
 *  - Private: Private data of an I2C Hardware
 *
 */

/*******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/i2c.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include "up_arch.h"

#include "efm32_gpio.h"
#include "chip/efm32_cmu.h"
#include "chip/efm32_i2c.h"

/* At least one I2C peripheral must be enabled */

#if defined(CONFIG_EFM32_I2C0) || defined(CONFIG_EFM32_I2C1) 

#if ! defined(CONFIG_EFM32_EFM32GG) 
#   warning "Not tested"
#endif

/*******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/
/* Configuration **************************************************************/
/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.  Instead,
 * CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in seconds and milliseconds */

#if !defined(CONFIG_EFM32_I2CTIMEOSEC) && !defined(CONFIG_EFM32_I2CTIMEOMS)
#  define CONFIG_EFM32_I2CTIMEOSEC 0
#  define CONFIG_EFM32_I2CTIMEOMS  500   /* Default is 500 milliseconds */
#elif !defined(CONFIG_EFM32_I2CTIMEOSEC)
#  define CONFIG_EFM32_I2CTIMEOSEC 0     /* User provided milliseconds */
#elif !defined(CONFIG_EFM32_I2CTIMEOMS)
#  define CONFIG_EFM32_I2CTIMEOMS  0     /* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_EFM32_I2CTIMEOTICKS
#  define CONFIG_EFM32_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_EFM32_I2CTIMEOSEC) + MSEC2TICK(CONFIG_EFM32_I2CTIMEOMS))
#endif

#ifndef CONFIG_EFM32_I2C_DYNTIMEO_STARTSTOP
#  define CONFIG_EFM32_I2C_DYNTIMEO_STARTSTOP TICK2USEC(CONFIG_EFM32_I2CTIMEOTICKS)
#endif

/* Macros to convert a I2C pin to a GPIO output */

#define I2C_OUTPUT (GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_CNF_OUTOD )

#define MKI2C_OUTPUT(p) (((p) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | I2C_OUTPUT)

/* Debug ****************************************************************************/
/* CONFIG_DEBUG_I2C + CONFIG_DEBUG enables general I2C debug output. */

#ifdef CONFIG_DEBUG_I2C
#  define i2cdbg dbg
#  define i2cvdbg vdbg
#else
#  define i2cdbg(x...)
#  define i2cvdbg(x...)
#endif

/* I2C event trace logic.  NOTE:  trace uses the internal, non-standard, 
 * low-level debug interface syslog() but does not require that any other debug
 * is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#  define efm32_i2c_tracereset(p)
#  define efm32_i2c_tracenew(p,i2c_reg_state,i2c_reg_if)
#  define efm32_i2c_traceevent(p,e,a)
#  define efm32_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

/* Error flags indicating I2C transfer has failed somehow. 
 * Notice that I2C_IF_TXOF (transmit overflow) is not really possible with 
 * this SW supporting master mode. Likewise for I2C_IF_RXUF (receive underflow) 
 * RXUF is only likely to occur with this SW if using a debugger peeking into 
 * RXDATA register. Thus, we ignore those types of fault.
 */ 
#define I2C_IF_ERRORS    (I2C_IF_BUSERR | I2C_IF_ARBLOST)

/************************************************************************************
 * Private Types
 ************************************************************************************/
/* Interrupt state */

enum efm32_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};


/* Trace events */

enum efm32_trace_e
{
    I2CEVENT_NONE,       /* Nothing */
    I2CEVENT_IDLE,       /* Mode IDLE for I2C_EVENT */
    I2CEVENT_WAIT,       /* Mode WAIT for I2C_EVENT */
    I2CEVENT_START,      /* Mode START for I2C_EVENT */
    I2CEVENT_ADDR,       /* Mode ADDR for I2C_EVENT */
    I2CEVENT_ADDRACK,    /* Mode ADDRACK for I2C_EVENT */
    I2CEVENT_DATATX,     /* Mode DATA for I2C_EVENT */
    I2CEVENT_DATATXACK,  /* Mode DATAACK for I2C_EVENT */
    I2CEVENT_DATARX,     /* Mode DATA for I2C_EVENT */
    I2CEVENT_DATARXACK,  /* Mode DATAACK for I2C_EVENT */
    I2CEVENT_STOP,       /* Mode DATAACK for I2C_EVENT */
    I2CEVENT_ERROR       /* Mode ERROR for I2C_EVENT */
};

/* Trace data */

struct efm32_trace_s
{
  uint32_t i2c_reg_state;     /* I2C register I2Cx_STATES */
  uint32_t i2c_reg_if;         /* I2C register I2Cx_IF */
  uint32_t count;              /* Interrupt count when status change */
  enum efm32_intstate_e event; /* Last event that occurred with this status */
  uint32_t parm;               /* Parameter associated with the event */
  uint32_t time;               /* First of event or first status */
};

/* I2C Device hardware configuration */

struct efm32_i2c_config_s
{
  uint32_t base;              /* I2C base address */
  uint32_t clk_bit;           /* Clock enable bit */
  uint32_t route;             /* route location of I2C */
  uint32_t reset_bit;         /* Reset bit */
  uint32_t scl_pin;           /* GPIO configuration for SCL as SCL */
  uint32_t sda_pin;           /* GPIO configuration for SDA as SDA */
#ifndef CONFIG_I2C_POLLED
  int (*isr)(int, void *);    /* Interrupt handler */
  uint32_t irq;               /* Event IRQ */
#endif
};

/* I2C Device Private Data */

struct efm32_i2c_priv_s
{
  const struct efm32_i2c_config_s *config; /* Port configuration */
  int refs;                    /* Referernce count */
  sem_t sem_excl;              /* Mutual exclusion semaphore */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif

  volatile uint8_t  intstate;       /* Interrupt handshake */
  volatile uint32_t i2c_reg_if;     /* Current state of I2Cx_IF register. */
  volatile uint32_t i2c_reg_state; /* Current state of I2Cx_STATES register. */

  uint8_t msgc;                /* Message count */
  struct i2c_msg_s *msgv;      /* Message list */
  uint8_t *ptr;                /* Current message buffer */
  int dcnt;                    /* Current message length */
  uint32_t flags;              /* Current message flags */

  /* I2C trace support */

#ifdef CONFIG_I2C_TRACE
  int tndx;                    /* Trace array index */
  uint32_t start_time;         /* Time when the trace was started */

  /* The actual trace data */

  struct efm32_trace_s trace[CONFIG_I2C_NTRACE];
#endif

};

/* I2C Device, Instance */

struct efm32_i2c_inst_s
{
  const struct i2c_ops_s  *ops;  /* Standard I2C operations */
  struct efm32_i2c_priv_s *priv; /* Common driver private data structure */

  uint32_t    frequency;   /* Frequency used in this instantiation */
  int         address;     /* Address used in this instantiation */
  uint32_t    flags;       /* Flags used in this instantiation */
};

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static inline uint32_t efm32_i2c_getreg(FAR struct efm32_i2c_priv_s *priv,
                                        uint8_t offset);
static inline void efm32_i2c_putreg(FAR struct efm32_i2c_priv_s *priv, 
                                    uint8_t offset, uint32_t value);
static inline void efm32_i2c_modifyreg(FAR struct efm32_i2c_priv_s *priv,
                                       uint8_t offset, uint32_t clearbits,
                                       uint32_t setbits);
static inline void efm32_i2c_sem_wait(FAR struct i2c_dev_s *dev);

#ifdef CONFIG_EFM32_I2C_DYNTIMEOUT
static useconds_t efm32_i2c_tousecs(int msgc, FAR struct i2c_msg_s *msgs);
#endif /* CONFIG_EFM32_I2C_DYNTIMEOUT */

static inline int  efm32_i2c_sem_waitdone(  FAR struct efm32_i2c_priv_s *priv);
static inline void efm32_i2c_sem_post(      FAR struct i2c_dev_s *dev);
static inline void efm32_i2c_sem_init(      FAR struct i2c_dev_s *dev);
static inline void efm32_i2c_sem_destroy(   FAR struct i2c_dev_s *dev);

#ifdef CONFIG_I2C_TRACE
static void efm32_i2c_tracereset(   FAR struct efm32_i2c_priv_s *priv);
static void efm32_i2c_tracenew(     FAR struct efm32_i2c_priv_s *priv, 
                                    uint32_t i2c_reg_state, 
                                    uint32_t i2c_reg_if);
static void efm32_i2c_traceevent(FAR struct efm32_i2c_priv_s *priv,
                                 enum efm32_trace_e event, uint32_t parm);
static void efm32_i2c_tracedump(FAR struct efm32_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */

static void efm32_i2c_setclock(FAR struct efm32_i2c_priv_s *priv,
                               uint32_t frequency);

static int efm32_i2c_isr(struct efm32_i2c_priv_s * priv);

#ifndef CONFIG_I2C_POLLED
#ifdef CONFIG_EFM32_I2C0
static int efm32_i2c0_isr(int irq, void *context);
#endif
#ifdef CONFIG_EFM32_I2C1
static int efm32_i2c1_isr(int irq, void *context);
#endif
#endif /* !CONFIG_I2C_POLLED */

static void efm32_i2c_reset(FAR struct efm32_i2c_priv_s *priv);
static int efm32_i2c_init(FAR struct efm32_i2c_priv_s *priv, int frequency);
static int efm32_i2c_deinit(FAR struct efm32_i2c_priv_s *priv);
static uint32_t efm32_i2c_setfrequency(FAR struct i2c_dev_s *dev,
                                       uint32_t frequency);
static int efm32_i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits);
static int efm32_i2c_process(FAR struct i2c_dev_s *dev, 
                             FAR struct i2c_msg_s *msgs, int count);
static int efm32_i2c_write(FAR struct i2c_dev_s *dev, const uint8_t *buffer,
                           int buflen);
static int efm32_i2c_read(FAR struct i2c_dev_s *dev, uint8_t *buffer, 
                          int buflen);

#ifdef CONFIG_I2C_WRITEREAD
static int efm32_i2c_writeread(FAR struct i2c_dev_s *dev,
                               const uint8_t *wbuffer, int wbuflen,
                               uint8_t *buffer, int buflen);
#endif /* CONFIG_I2C_WRITEREAD */

#ifdef CONFIG_I2C_TRANSFER
static int efm32_i2c_transfer(FAR struct i2c_dev_s *dev, 
                              FAR struct i2c_msg_s *msgs, int count);
#endif /* CONFIG_I2C_TRANSFER */

/*******************************************************************************
 * Private Data
 ******************************************************************************/

/* Trace events strings */

#ifdef CONFIG_I2C_TRACE
static const char *g_trace_names[] =
{
  "NONE      ",
  "SENDADDR  ",
  "SENDBYTE  ",
  "ITBUFEN   ",
  "RCVBYTE   ",
  "REITBUFEN ",
  "DISITBUFEN",
  "BTFNOSTART",
  "BTFRESTART",
  "BTFSTOP   ",
  "ERROR     "
};
#endif

/* I2C device structures */

#ifdef CONFIG_EFM32_I2C0
static const struct efm32_i2c_config_s efm32_i2c0_config =
{
  .base       = EFM32_I2C0_BASE,
  .clk_bit    = CMU_HFPERCLKEN0_I2C0,
  .route      = BOARD_I2C1_ROUTE_LOCATION,
  .scl_pin    = BOARD_I2C0_SCL,
  .sda_pin    = BOARD_I2C0_SDA,
#ifndef CONFIG_I2C_POLLED
  .isr        = efm32_i2c0_isr,
  .irq        = EFM32_IRQ_I2C0
#endif
};

static struct efm32_i2c_priv_s efm32_i2c0_priv =
{
  .config     = &efm32_i2c0_config,
  .refs       = 0,
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
};
#endif

#ifdef CONFIG_EFM32_I2C1
static const struct efm32_i2c_config_s efm32_i2c1_config =
{
  .base       = EFM32_I2C1_BASE,
  .clk_bit    = CMU_HFPERCLKEN0_I2C1,
  .route      = BOARD_I2C1_ROUTE_LOCATION,
  .scl_pin    = BOARD_I2C1_SCL,
  .sda_pin    = BOARD_I2C1_SDA,
#ifndef CONFIG_I2C_POLLED
  .isr        = efm32_i2c1_isr,
  .irq        = EFM32_IRQ_I2C1
#endif
};

static struct efm32_i2c_priv_s efm32_i2c1_priv =
{
  .config     = &efm32_i2c1_config,
  .refs       = 0,
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .dcnt       = 0,
  .flags      = 0,
};
#endif


/* Device Structures, Instantiation */

static const struct i2c_ops_s efm32_i2c_ops =
{
  .setfrequency       = efm32_i2c_setfrequency,
  .setaddress         = efm32_i2c_setaddress,
  .write              = efm32_i2c_write,
  .read               = efm32_i2c_read
#ifdef CONFIG_I2C_WRITEREAD
  , .writeread        = efm32_i2c_writeread
#endif
#ifdef CONFIG_I2C_TRANSFER
  , .transfer         = efm32_i2c_transfer
#endif
#ifdef CONFIG_I2C_SLAVE
  , .setownaddress    = efm32_i2c_setownaddress,
    .registercallback = efm32_i2c_registercallback
#endif
};

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*******************************************************************************
 * Name: efm32_i2c_getreg
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ******************************************************************************/

static inline uint32_t efm32_i2c_getreg(FAR struct efm32_i2c_priv_s *priv,
                                        uint8_t offset)
{
  return getreg32(priv->config->base + offset);
}

/*******************************************************************************
 * Name: efm32_i2c_putreg
 *
 * Description:
 *  Put a 16-bit register value by offset
 *
 ******************************************************************************/

static inline void efm32_i2c_putreg(FAR struct efm32_i2c_priv_s *priv, 
                                    uint8_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/*******************************************************************************
 * Name: efm32_i2c_modifyreg
 *
 * Description:
 *   Modify a 16-bit register value by offset
 *
 ******************************************************************************/

static inline void efm32_i2c_modifyreg(FAR struct efm32_i2c_priv_s *priv,
                                       uint8_t offset, uint32_t clearbits,
                                       uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

/*******************************************************************************
 * Name: efm32_i2c_sem_wait
 *
 * Description:
 *   Take the exclusive access, waiting as necessary
 *
 ******************************************************************************/

static inline void efm32_i2c_sem_wait(FAR struct i2c_dev_s *dev)
{
  while (sem_wait(&((struct efm32_i2c_inst_s *)dev)->priv->sem_excl) != 0)
    {
      ASSERT(errno == EINTR);
    }
}

/*******************************************************************************
 * Name: efm32_i2c_tousecs
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be 
 *   processed.
 *
 ******************************************************************************/

#ifdef CONFIG_EFM32_I2C_DYNTIMEO
static useconds_t efm32_i2c_tousecs(int msgc, FAR struct i2c_msg_s *msgs)
{
  size_t bytecount = 0;
  int i;

  /* Count the number of bytes left to process */

  for (i = 0; i < msgc; i++)
    {
      bytecount += msgs[i].length;
    }

  /* Then return a number of microseconds based on a user provided scaling
   * factor.
   */

  return (useconds_t)(CONFIG_EFM32_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/*******************************************************************************
 * Name: efm32_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ******************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline int efm32_i2c_sem_waitdone(FAR struct efm32_i2c_priv_s *priv)
{
  struct timespec abstime;
  int ret;

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * sem_timedwait() sleeps.
   */

  priv->intstate = INTSTATE_WAITING;


  do
    {
      /* Get the current time */

      (void)clock_gettime(CLOCK_REALTIME, &abstime);

      /* Calculate a time in the future */

#if CONFIG_EFM32_I2CTIMEOSEC > 0
      abstime.tv_sec += CONFIG_EFM32_I2CTIMEOSEC;
#endif

      /* Add a value proportional to the number of bytes in the transfer */

#ifdef CONFIG_EFM32_I2C_DYNTIMEO
      abstime.tv_nsec += 1000 * efm32_i2c_tousecs(priv->msgc, priv->msgv);
      if (abstime.tv_nsec > 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }

#elif CONFIG_EFM32_I2CTIMEOMS > 0
      abstime.tv_nsec += CONFIG_EFM32_I2CTIMEOMS * 1000 * 1000;
      if (abstime.tv_nsec > 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }
#endif

      /* Enable I2C interrupts */

      efm32_i2c_putreg(priv,EFM32_I2C_IEN_OFFSET, I2C_IF_BUSHOLD | 
                       I2C_IF_ERRORS);
      /* add this to improve speed use this but efm32_i2c_isr should be retested
       * efm32_i2c_putreg(priv,EFM32_I2C_IEN_OFFSET, I2C_IF_NACK | I2C_IF_ACK | 
       *                  I2C_IF_MSTOP | I2C_IF_RXDATAV | I2C_IF_ERRORS);
       */

      /* Wait until either the transfer is complete or the timeout expires */

      ret = sem_timedwait(&priv->sem_isr, &abstime);

      /* Disable I2C interrupts */

      efm32_i2c_putreg(priv,EFM32_I2C_IEN_OFFSET, 0);

      if (ret != OK && errno != EINTR)
        {
          /* Break out of the loop on irrecoverable errors.  This would
           * include timeouts and mystery errors reported by sem_timedwait.
           * NOTE that we try again if we are awakened by a signal (EINTR).
           */

          break;
        }
    }

  /* Loop until the interrupt level transfer is complete. */

  while (priv->intstate != INTSTATE_DONE);

  /* Set the interrupt state back to IDLE */

  priv->intstate = INTSTATE_IDLE;

  return ret;
}
#else
static inline int efm32_i2c_sem_waitdone(FAR struct efm32_i2c_priv_s *priv)
{
  uint32_t timeout;
  uint32_t start;
  uint32_t elapsed;
  int ret;

  /* Get the timeout value */

#ifdef CONFIG_EFM32_I2C_DYNTIMEO
  timeout = USEC2TICK(efm32_i2c_tousecs(priv->msgc, priv->msgv));
#else
  timeout = CONFIG_EFM32_I2CTIMEOTICKS;
#endif

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * sem_timedwait() sleeps.
   */

  priv->intstate = INTSTATE_WAITING;
  start = clock_systimer();

  do
    {
      /* Poll by simply calling the timer interrupt handler until it
       * reports that it is done.
       */

      efm32_i2c_isr(priv);

      /* Calculate the elapsed time */

      elapsed = clock_systimer() - start;
    }

  /* Loop until the transfer is complete. */

  while (priv->intstate != INTSTATE_DONE && elapsed < timeout);

  i2cvdbg("intstate: %d elapsed: %d threshold: %d I2Cx_STATES: %08x "
          "I2Cx_IF: %08x\n", priv->intstate, elapsed, timeout,
          priv->i2c_reg_state,priv->i2c_reg_if);

  /* Set the interrupt state back to IDLE */

  ret = priv->intstate == INTSTATE_DONE ? OK : -ETIMEDOUT;

  priv->intstate = INTSTATE_IDLE;

  return ret;
}
#endif


/*******************************************************************************
 * Name: efm32_i2c_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ******************************************************************************/

static inline void efm32_i2c_sem_post(FAR struct i2c_dev_s *dev)
{
  sem_post(&((struct efm32_i2c_inst_s *)dev)->priv->sem_excl);
}

/*******************************************************************************
 * Name: efm32_i2c_sem_init
 *
 * Description:
 *   Initialize semaphores
 *
 ******************************************************************************/

static inline void efm32_i2c_sem_init(FAR struct i2c_dev_s *dev)
{
  sem_init(&((struct efm32_i2c_inst_s *)dev)->priv->sem_excl, 0, 1);
#ifndef CONFIG_I2C_POLLED
  sem_init(&((struct efm32_i2c_inst_s *)dev)->priv->sem_isr, 0, 0);
#endif
}

/*******************************************************************************
 * Name: efm32_i2c_sem_destroy
 *
 * Description:
 *   Destroy semaphores.
 *
 ******************************************************************************/

static inline void efm32_i2c_sem_destroy(FAR struct i2c_dev_s *dev)
{
  sem_destroy(&((struct efm32_i2c_inst_s *)dev)->priv->sem_excl);
#ifndef CONFIG_I2C_POLLED
  sem_destroy(&((struct efm32_i2c_inst_s *)dev)->priv->sem_isr);
#endif
}

/*******************************************************************************
 * Name: efm32_i2c_trace*
 *
 * Description:
 *   I2C trace instrumentation
 *
 ******************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void efm32_i2c_traceclear(FAR struct efm32_i2c_priv_s *priv)
{
  struct efm32_trace_s *trace = &priv->trace[priv->tndx];

  trace->i2c_reg_if     = 0;      /* I2C I2Cx_IF register */
  trace->i2c_reg_state = 0;      /* I2C I2Cx_STATES register */
  trace->count  = 0;              /* Interrupt count when status change */
  trace->event  = I2CEVENT_NONE;  /* Last event that occurred with this status*/
  trace->parm   = 0;              /* Parameter associated with the event */
  trace->time   = 0;              /* Time of first status or event */
}

static void efm32_i2c_tracereset(FAR struct efm32_i2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx       = 0;
  priv->start_time = clock_systimer();
  efm32_i2c_traceclear(priv);
}

static void efm32_i2c_tracenew(FAR struct efm32_i2c_priv_s *priv, 
                               uint32_t i2c_reg_state,
                               uint32_t i2c_reg_if)
{
  struct efm32_trace_s *trace = &priv->trace[priv->tndx];

  /* Is the current entry uninitialized?  Has the states/pending changed? */

  if (trace->count == 0 || i2c_reg_if != trace->i2c_reg_if || \
      i2c_reg_state != trace->i2c_reg_state )
    {
      /* Yes.. Was it the states/pending changed?  */

      if (trace->count != 0)
        {
          /* Yes.. bump up the trace index (unless we are out of trace 
           * entries) 
           */

          if (priv->tndx >= (CONFIG_I2C_NTRACE-1))
            {
              i2cdbg("Trace table overflow\n");
              return;
            }

          priv->tndx++;
          trace = &priv->trace[priv->tndx];
        }

      /* Initialize the new trace entry */

      efm32_i2c_traceclear(priv);
      trace->i2c_reg_state  = i2c_reg_state;
      trace->i2c_reg_if     = i2c_reg_if;
      trace->count          = 1;
      trace->time           = clock_systimer();
    }
  else
    {
      /* Just increment the count of times that we have seen this 
       * states/pending 
       */

      trace->count++;
    }
}

static void efm32_i2c_traceevent(FAR struct efm32_i2c_priv_s *priv,
                                 enum efm32_trace_e event, uint32_t parm)
{
  struct efm32_trace_s *trace;

  if (event != I2CEVENT_NONE)
    {
      trace = &priv->trace[priv->tndx];

      /* Initialize the new trace entry */

      trace->event  = event;
      trace->parm   = parm;

      /* Bump up the trace index (unless we are out of trace entries) */

      if (priv->tndx >= (CONFIG_I2C_NTRACE-1))
        {
          i2cdbg("Trace table overflow\n");
          return;
        }

      priv->tndx++;
      efm32_i2c_traceclear(priv);
    }
}

static void efm32_i2c_tracedump(FAR struct efm32_i2c_priv_s *priv)
{
  struct efm32_trace_s *trace;
  int i;

  syslog(LOG_DEBUG, "Elapsed time: %d\n",
         clock_systimer() - priv->start_time);

  for (i = 0; i <= priv->tndx; i++)
    {
      trace = &priv->trace[i];
      syslog(LOG_DEBUG,
             "%2d. I2Cx_STATE: %08x I2Cx_PENDING: %08x COUNT: %3d "
             "EVENT: %s(%2d) PARM: %08x TIME: %d\n",
             i+1, trace->i2c_reg_state, trace->i2c_reg_if, trace->count,
             g_trace_names[trace->event], trace->event, trace->parm,
             trace->time - priv->start_time);
    }
}
#endif /* CONFIG_I2C_TRACE */

/*******************************************************************************
 * Name: efm32_i2c_setclock
 *
 * Description:
 *   Set the I2C clock
 *
 ******************************************************************************/

static void efm32_i2c_setclock(FAR struct efm32_i2c_priv_s *priv, 
                               uint32_t frequency)
{
  uint32_t div;

  /* Set the CLHR (clock low to high ratio). */
  efm32_i2c_modifyreg(priv,EFM32_I2C_CTRL_OFFSET,_I2C_CTRL_CLHR_MASK,
#if   defined(CONFIG_EFM32_I2C_CLHR_FAST)
                      _I2C_CTRL_CLHR_FAST       /* Ratio is 11:3 */
#elif defined(CONFIG_EFM32_I2C_CLHR_ASYMMETRIC)
                      _I2C_CTRL_CLHR_ASYMMETRIC /* Ratio is  6:3 */
#else /* CLHR STANDARD */
                      _I2C_CTRL_CLHR_STANDARD   /* Ratio is  4:4 */
#endif
                     <<_I2C_CTRL_CLHR_SHIFT);

  /* Frequency is given by fSCL = fHFPERCLK/((Nlow + Nhigh)(DIV + 1) + 4), thus
   * DIV = ((fHFPERCLK - 4fSCL)/((Nlow + Nhigh)fSCL)) - 1 
   */

#if   defined(CONFIG_EFM32_I2C_CLHR_FAST)
#define n (11 + 6) /* Ratio is 11:3 */
#elif defined(CONFIG_EFM32_I2C_CLHR_ASYMMETRIC)
#define n ( 6 + 3) /* Ratio is  6:3 */
#else /* CLHR STANDARD */
#define n ( 4 + 4) /* Ratio is  4:4 */
#endif

  div = (BOARD_HFPERCLK_FREQUENCY - (4 * frequency)) / (n * frequency);

  /* Clock divisor must be at least 1 in slave mode according to reference */
  /* manual (in which case there is normally no need to set bus frequency). */
  if ((efm32_i2c_getreg(priv,EFM32_I2C_CTRL_OFFSET) & I2C_CTRL_SLAVE) && !div)
  {
    div = 1;
  }

  DEBUGASSERT(div <= (_I2C_CLKDIV_DIV_MASK>>_I2C_CLKDIV_DIV_SHIFT));

  efm32_i2c_putreg(priv,EFM32_I2C_CLKDIV_OFFSET,div);

}


/*******************************************************************************
 * Name: efm32_i2c_isr_finished
 *
 * Description:
 *  function that correctly finish transfert isr/polling
 *
 ******************************************************************************/
void efm32_i2c_isr_finished(struct efm32_i2c_priv_s *priv)
{
#ifndef CONFIG_I2C_POLLED

    /* Is there a thread waiting for this event (there should be) */

    if (priv->intstate == INTSTATE_WAITING)
    {
        /* Yes.. inform the thread that the transfer is complete
         * and wake it up.
         */

        sem_post(&priv->sem_isr);
        priv->intstate = INTSTATE_DONE;
    }
#else
    priv->intstate = INTSTATE_DONE;
#endif
}

/*******************************************************************************
 * Name: efm32_i2c_isr
 *
 * Description:
 *  Common Interrupt Service Routine
 *
 ******************************************************************************/

static int efm32_i2c_isr(struct efm32_i2c_priv_s *priv)
{

    uint32_t i2c_reg_if   = efm32_i2c_getreg(priv, EFM32_I2C_IF_OFFSET);
    uint32_t i2c_reg_state    = efm32_i2c_getreg(priv, EFM32_I2C_STATE_OFFSET);

    priv->i2c_reg_if      = i2c_reg_if;
    priv->i2c_reg_state   = i2c_reg_state;

    /* Check for new trace setup */

    efm32_i2c_tracenew(priv, i2c_reg_state, i2c_reg_if);

    /* Check for errors, in which case, stop the transfer and return
     * Note that in master reception mode AF becomes set on last byte
     * since ACK is not returned. We should ignore this error.
     */

    if ((i2c_reg_if & I2C_IF_ERRORS) != 0)
    {
        efm32_i2c_traceevent(priv, I2CEVENT_ERROR, 0);

        efm32_i2c_isr_finished(priv);
        return OK;
    }


    switch ( i2c_reg_state & _I2C_STATE_STATE_MASK >> _I2C_STATE_STATE_SHIFT)
    {
        case _I2C_STATE_STATE_START: 

            efm32_i2c_traceevent(priv, I2CEVENT_ADDR, priv->msgc);

            /* We check for msgc > 0 here as an unexpected interrupt */

            if (priv->msgc > 0 && priv->msgv != NULL)
            {
                int tmp;
                /* Get run-time data */

                priv->ptr   = priv->msgv->buffer;
                priv->dcnt  = priv->msgv->length;
                priv->flags = priv->msgv->flags;

                /* Send address byte */

                if ( priv->flags & I2C_M_TEN)
                {
                    if ( priv->flags & I2C_M_READ)
                        tmp = I2C_READADDR10H(priv->flags);
                    else
                        tmp = I2C_WRITEADDR10H(priv->flags);
                }
                else
                {
                    if ( priv->flags & I2C_M_READ)
                        tmp = I2C_READADDR8(priv->flags);
                    else
                        tmp = I2C_WRITEADDR8(priv->flags);
                }

                efm32_i2c_putreg(priv, EFM32_I2C_TXDATA_OFFSET, tmp);
            }
            else
            {
                /* Not more message send stop to free bus */

                efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_STOP);
            }

        case _I2C_STATE_STATE_ADDRACK: 

            efm32_i2c_traceevent(priv, I2CEVENT_ADDRACK, priv->msgc);

            if  ( i2c_reg_state & _I2C_STATE_NACKED_MASK )
            {
                /* Device not ack */

                efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, I2C_CMD_STOP);

                efm32_i2c_isr_finished(priv);

                return OK;
            }
            else
            {
                if ( priv->flags & I2C_M_TEN)
                {
                    int tmp;

                    /* Send 2nd address byte */

                    if ( priv->flags & I2C_M_READ)
                        tmp = I2C_READADDR10H(priv->flags);
                    else
                        tmp = I2C_WRITEADDR10H(priv->flags);
                    efm32_i2c_putreg(priv, EFM32_I2C_TXDATA_OFFSET, tmp);
                }
                else
                {
                    if (priv->dcnt > 0)
                    {
                        if (priv->flags & I2C_M_READ)
                        {
                            efm32_i2c_getreg(priv, EFM32_I2C_RXDATA_OFFSET);
                        }
                        else
                        {
                            priv->dcnt--;
                            efm32_i2c_putreg(priv, EFM32_I2C_TXDATA_OFFSET,
                                             *(priv->ptr++));
                        }
                    }
                    else
                    {
                        efm32_i2c_putreg(priv, EFM32_I2C_CMD_OFFSET, 
                                         I2C_CMD_STOP);

                        efm32_i2c_isr_finished(priv);

                        return OK;
                    }
                }


            }


    }



#if 0


    /* Was start bit sent */

    if ((status & I2C_SR1_SB) != 0)
    {
    }

    /* In 10-bit addressing mode, was first byte sent */

    else if ((status & I2C_SR1_ADD10) != 0)
    {
        /* TODO: Finish 10-bit mode addressing.
         *
         * For now just clear ISR by writing to DR register. As we don't do
         * 10 bit addressing this must be a spurious ISR
         */

        efm32_i2c_putreg(priv, EFM32_I2C_DR_OFFSET, 0);
    }

    /* Was address sent, continue with either sending or reading data */

    else if ((priv->flags & I2C_M_READ) == 0 && \
             (status & (I2C_SR1_ADDR | I2C_SR1_TXE)) != 0)
    {
        if (priv->dcnt > 0)
        {
            /* Send a byte */

            efm32_i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->dcnt);
            efm32_i2c_putreg(priv, EFM32_I2C_DR_OFFSET, *priv->ptr++);
            priv->dcnt--;
        }
    }

    else if ((priv->flags & I2C_M_READ) != 0 && (status & I2C_SR1_ADDR) != 0)
    {
        /* Enable RxNE and TxE buffers in order to receive one or multiple 
         * bytes 
         */

#ifndef CONFIG_I2C_POLLED
        efm32_i2c_traceevent(priv, I2CEVENT_ITBUFEN, 0);
        efm32_i2c_modifyreg(priv, EFM32_I2C_CR2_OFFSET, 0, I2C_CR2_ITBUFEN);
#endif
    }

    /* More bytes to read */

    else if ((status & I2C_SR1_RXNE) != 0)
    {
        /* Read a byte, if dcnt goes < 0, then read dummy bytes to ack ISRs */

        if (priv->dcnt > 0)
        {
            efm32_i2c_traceevent(priv, I2CEVENT_RCVBYTE, priv->dcnt);

            /* No interrupts or context switches may occur in the following
             * sequence.  Otherwise, additional bytes may be sent by the
             * device.
             */

#ifdef CONFIG_I2C_POLLED
            irqstate_t state = irqsave();
#endif
            /* Receive a byte */

            *priv->ptr++ = efm32_i2c_getreg(priv, EFM32_I2C_DR_OFFSET);

            /* Disable acknowledge when last byte is to be received */

            priv->dcnt--;
            if (priv->dcnt == 1)
            {
                efm32_i2c_modifyreg(priv, EFM32_I2C_CR1_OFFSET, I2C_CR1_ACK, 0);
            }

#ifdef CONFIG_I2C_POLLED
            irqrestore(state);
#endif
        }
        else
        {
            /* Throw away the unexpected byte */

            efm32_i2c_getreg(priv, EFM32_I2C_DR_OFFSET);
        }
    }
    else if (status & I2C_SR1_TXE)
    {
        /* This should never happen, but it does happen occasionally with lots
         * of noise on the bus. It means the peripheral is expecting more data
         * bytes, but we don't have any to give.
         */

        efm32_i2c_putreg(priv, EFM32_I2C_DR_OFFSET, 0);
    }
    else if (status & I2C_SR1_BTF)
    {
        /* We should have handled all cases where this could happen above, but
         * just to ensure it gets ACKed, lets clear it here
         */

        efm32_i2c_getreg(priv, EFM32_I2C_DR_OFFSET);
    }
    else if (status & I2C_SR1_STOPF)
    {
        /* We should never get this, as we are a master not a slave. Write CR1
         * with its current value to clear the error
         */

        efm32_i2c_modifyreg(priv, EFM32_I2C_CR1_OFFSET, 0, 0);
    }

    /* Do we have more bytes to send, enable/disable buffer interrupts
     * (these ISRs could be replaced by DMAs)
     */

#ifndef CONFIG_I2C_POLLED
    if (priv->dcnt > 0)
    {
        efm32_i2c_traceevent(priv, I2CEVENT_REITBUFEN, 0);
        efm32_i2c_modifyreg(priv, EFM32_I2C_CR2_OFFSET, 0, I2C_CR2_ITBUFEN);
    }
    else if (priv->dcnt == 0)
    {
        efm32_i2c_traceevent(priv, I2CEVENT_DISITBUFEN, 0);
        efm32_i2c_modifyreg(priv, EFM32_I2C_CR2_OFFSET, I2C_CR2_ITBUFEN, 0);
    }
#endif

    /* Was last byte received or sent?  Hmmm... the F2 and F4 seems to differ from
     * the F1 in that BTF is not set after data is received (only RXNE).
     */

#if defined(CONFIG_EFM32_EFM32F20XX) || defined(CONFIG_EFM32_EFM32F40XX) || \
    defined(CONFIG_EFM32_EFM32L15XX)
    if (priv->dcnt <= 0 && (status & (I2C_SR1_BTF|I2C_SR1_RXNE)) != 0)
#else
        if (priv->dcnt <= 0 && (status & I2C_SR1_BTF) != 0)
#endif
        {
            efm32_i2c_getreg(priv, EFM32_I2C_DR_OFFSET);    /* ACK ISR */

            /* Do we need to terminate or restart after this byte?
             * If there are more messages to send, then we may:
             *
             *  - continue with repeated start
             *  - or just continue sending writeable part
             *  - or we close down by sending the stop bit
             */

            if (priv->msgc > 0)
            {
                if (priv->msgv->flags & I2C_M_NORESTART)
                {
                    efm32_i2c_traceevent(priv, I2CEVENT_BTFNOSTART, priv->msgc);
                    priv->ptr   = priv->msgv->buffer;
                    priv->dcnt  = priv->msgv->length;
                    priv->flags = priv->msgv->flags;
                    priv->msgv++;
                    priv->msgc--;

                    /* Restart this ISR! */

#ifndef CONFIG_I2C_POLLED
                    efm32_i2c_modifyreg(priv, EFM32_I2C_CR2_OFFSET, 0, 
                                        I2C_CR2_ITBUFEN);
#endif
                }
                else
                {
                    efm32_i2c_traceevent(priv, I2CEVENT_BTFRESTART, priv->msgc);
                    efm32_i2c_putreg(priv,EFM32_I2C_CMD_OFFSET,I2C_CMD_START);
                }
            }
            else if (priv->msgv)
            {
                efm32_i2c_traceevent(priv, I2CEVENT_BTFSTOP, 0);
                efm32_i2c_sendstop(priv);

                /* Is there a thread waiting for this event (there should be) */

#ifndef CONFIG_I2C_POLLED
                if (priv->intstate == INTSTATE_WAITING)
                {
                    /* Yes.. inform the thread that the transfer is complete
                     * and wake it up.
                     */

                    sem_post(&priv->sem_isr);
                    priv->intstate = INTSTATE_DONE;
                }
#else
                priv->intstate = INTSTATE_DONE;
#endif

                /* Mark that we have stopped with this transaction */

                priv->msgv = NULL;
            }
        }
end:
#endif



    return OK;
}

#ifndef CONFIG_I2C_POLLED
/*******************************************************************************
 * Name: efm32_i2c0_isr
 *
 * Description:
 *   I2C0 interrupt service routine
 *
 ******************************************************************************/

#ifdef CONFIG_EFM32_I2C0
static int efm32_i2c0_isr(int irq, void *context)
{
  return efm32_i2c_isr(&efm32_i2c0_priv);
}
#endif

/*******************************************************************************
 * Name: efm32_i2c1_isr
 *
 * Description:
 *   I2C1 interrupt service routine
 *
 ******************************************************************************/

#ifdef CONFIG_EFM32_I2C1
static int efm32_i2c1_isr(int irq, void *context)
{
  return efm32_i2c_isr(&efm32_i2c1_priv);
}
#endif

#endif

/*******************************************************************************
 * Private Initialization and Deinitialization
 ******************************************************************************/
/*******************************************************************************
 * Name: efm32_i2c_reset
 *
 * Description:
 *   Reset I2C to same state as after a HW reset.
 *
 ******************************************************************************/

static void efm32_i2c_reset(FAR struct efm32_i2c_priv_s *priv)
{
  efm32_i2c_putreg(priv,EFM32_I2C_CTRL_OFFSET     ,_I2C_CTRL_RESETVALUE     );
  efm32_i2c_putreg(priv,EFM32_I2C_CLKDIV_OFFSET   ,_I2C_CLKDIV_RESETVALUE   );
  efm32_i2c_putreg(priv,EFM32_I2C_SADDR_OFFSET    ,_I2C_SADDR_RESETVALUE    );
  efm32_i2c_putreg(priv,EFM32_I2C_SADDRMASK_OFFSET,_I2C_SADDRMASK_RESETVALUE);
  efm32_i2c_putreg(priv,EFM32_I2C_IEN_OFFSET      ,_I2C_IEN_RESETVALUE      );
  efm32_i2c_putreg(priv,EFM32_I2C_IFC_OFFSET      ,_I2C_IFC_MASK            );
  /* Do not reset route register, setting should be done independently */
}


/*******************************************************************************
 * Name: efm32_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *   Prepare and start an I2C transfer (single master mode only).
 *
 ******************************************************************************/

static int efm32_i2c_init(FAR struct efm32_i2c_priv_s *priv, int frequency )
{
  /* Power-up and configure GPIOs */


  /* Enable power and reset the peripheral */

  /* Enable I2C Clock */

  modifyreg32(EFM32_CMU_HFPERCLKEN0, 0, priv->config->clk_bit);

  /* reset all resgister */

  efm32_i2c_reset(priv); 

  /* Configure pins */

  if (efm32_configgpio(priv->config->scl_pin) < 0)
    {
      return ERROR;
    }

  if (efm32_configgpio(priv->config->sda_pin) < 0)
    {
      return ERROR;
    }

  /* Attach ISRs */

#ifndef CONFIG_I2C_POLLED
  irq_attach(priv->config->irq, priv->config->isr);
  up_enable_irq(priv->config->irq);
#endif

  efm32_i2c_setclock(priv, 100000);

  /* Enable I2C */

  efm32_i2c_putreg(priv,EFM32_I2C_CTRL_OFFSET,I2C_CTRL_EN);

  /* send abort CMD to be done after reset 
   * if i2c peripheral timeout is not used 
   */

  efm32_i2c_putreg(priv,EFM32_I2C_CMD_OFFSET,I2C_CMD_ABORT);

  return OK;

}

/*******************************************************************************
 * Name: efm32_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ******************************************************************************/

static int efm32_i2c_deinit(FAR struct efm32_i2c_priv_s *priv)
{
  /* Disable I2C */

  efm32_i2c_reset(priv);

  /* Unconfigure GPIO pins */

  efm32_unconfiggpio(priv->config->scl_pin);
  efm32_configgpio(priv->config->sda_pin);

  /* Disable and detach interrupts */

#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->irq);
  irq_detach(priv->config->irq);
#endif

  /* Disable clocking */

  modifyreg32(EFM32_CMU_HFPERCLKEN0, priv->config->clk_bit, 0);
  return OK;
}

/*******************************************************************************
 * Device Driver Operations
 ******************************************************************************/

/*******************************************************************************
 * Name: efm32_i2c_setfrequency
 *
 * Description:
 *   Set the I2C frequency
 *
 ******************************************************************************/

static uint32_t efm32_i2c_setfrequency(FAR struct i2c_dev_s *dev, 
                                       uint32_t frequency)
{
  efm32_i2c_sem_wait(dev);

  ((struct efm32_i2c_inst_s *)dev)->frequency = frequency;

  efm32_i2c_sem_post(dev);
  return frequency;
}

/*******************************************************************************
 * Name: efm32_i2c_setaddress
 *
 * Description:
 *   Set the I2C slave address
 *
 ******************************************************************************/

static int efm32_i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits)
{
  efm32_i2c_sem_wait(dev);

  ((struct efm32_i2c_inst_s *)dev)->address = addr;
  ((struct efm32_i2c_inst_s *)dev)->flags   = (nbits == 10) ? I2C_M_TEN : 0;

  efm32_i2c_sem_post(dev);
  return OK;
}

/*******************************************************************************
 * Name: efm32_i2c_process
 *
 * Description:
 *   Common I2C transfer logic
 *
 ******************************************************************************/

static int efm32_i2c_process(FAR struct i2c_dev_s *dev, 
                             FAR struct i2c_msg_s *msgs, int count)
{
  struct efm32_i2c_inst_s *inst = (struct efm32_i2c_inst_s *)dev;
  FAR struct efm32_i2c_priv_s *priv = inst->priv;
  int errval = 0;

  ASSERT(count);

  /* Ensure that address or flags don't change meanwhile */

  efm32_i2c_sem_wait(dev);

  /* Reset ptr and dcnt to ensure an unexpected data interrupt doesn't
   * overwrite stale data.
   */

  priv->dcnt = 0;
  priv->ptr = NULL;

  priv->msgv = msgs;
  priv->msgc = count;

  /* Reset I2C trace logic */

  efm32_i2c_tracereset(priv);

  /* Set I2C clock frequency (on change it toggles I2C_CR1_PE !) */

  efm32_i2c_setclock(priv, inst->frequency);

#if 0
  /* Check if in busy state. Since this SW assumes single master, we can */
  /* just issue an abort. The BUSY state is normal after a reset. */
  if (i2c->STATE & I2C_STATE_BUSY)
  {
    i2c->CMD = I2C_CMD_ABORT;
  }
#endif

  /* Ensure buffers are empty */
  efm32_i2c_putreg(priv,EFM32_I2C_CMD_OFFSET,I2C_CMD_CLEARPC | I2C_CMD_CLEARTX);
  if (efm32_i2c_getreg(priv,EFM32_I2C_IF_OFFSET) & I2C_IF_RXDATAV)
  {
    (void)efm32_i2c_getreg(priv,EFM32_I2C_RXDATA_OFFSET);
  }

  /* Clear all pending interrupts prior to starting transfer. */
  efm32_i2c_putreg(priv,EFM32_I2C_IFC_OFFSET,_I2C_IFC_MASK);


  /* Trigger start condition, then the process moves into the ISR.  I2C
   * interrupts will be enabled within efm32_i2c_waitdone().
   */

  priv->i2c_reg_if = 0;
  priv->i2c_reg_state = 0;
  efm32_i2c_putreg(priv,EFM32_I2C_CMD_OFFSET,I2C_CMD_START);

  /* Wait for an ISR, if there was a timeout, fetch latest status to get
   * the BUSY flag.
   */

  if (efm32_i2c_sem_waitdone(priv) < 0)
    {
      errval = ETIMEDOUT;

      i2cdbg("Timed out: I2Cx_STATE: 0x%04x I2Cx_STATUS: 0x%08x\n",
             efm32_i2c_getreg(priv, EFM32_I2C_STATE_OFFSET),
             efm32_i2c_getreg(priv, EFM32_I2C_STATUS_OFFSET)
             );

       /* abort */

      efm32_i2c_putreg(priv,EFM32_I2C_CMD_OFFSET,I2C_CMD_ABORT);

      /* Clear busy flag in case of timeout */

      //status = priv->status & 0xffff;
    }
  else
    {
      /* clear SR2 (BUSY flag) as we've done successfully */

      //status = priv->status & 0xffff;
    }

  /* Check for error status conditions */
#if 0
  if ((status & I2C_SR1_ERRORMASK) != 0)
    {
      /* I2C_SR1_ERRORMASK is the 'OR' of the following individual bits: */

      if (status & I2C_SR1_BERR)
        {
          /* Bus Error */

          errval = EIO;
        }
      else if (status & I2C_SR1_ARLO)
        {
          /* Arbitration Lost (master mode) */

          errval = EAGAIN;
        }
      else if (status & I2C_SR1_AF)
        {
          /* Acknowledge Failure */

          errval = ENXIO;
        }
      else if (status & I2C_SR1_OVR)
        {
          /* Overrun/Underrun */

          errval = EIO;
        }
      else if (status & I2C_SR1_PECERR)
        {
          /* PEC Error in reception */

          errval = EPROTO;
        }
      else if (status & I2C_SR1_TIMEOUT)
        {
          /* Timeout or Tlow Error */

          errval = ETIME;
        }

      /* This is not an error and should never happen since SMBus is not 
       * enabled 
       */

      else /* if (status & I2C_SR1_SMBALERT) */
        {
          /* SMBus alert is an optional signal with an interrupt line for 
           *  devices that want to trade their ability to master for a pin.
           */

          errval = EINTR;
        }
    }

  /* This is not an error, but should not happen.  The BUSY signal can hang,
   * however, if there are unhealthy devices on the bus that need to be reset.
   * NOTE:  We will only see this busy indication if efm32_i2c_sem_waitdone()
   * fails above;  Otherwise it is cleared.
   */

  else if ((status & (I2C_SR2_BUSY << 16)) != 0)
    {
      /* I2C Bus is for some reason busy */

      errval = EBUSY;
    }
#endif

  /* Dump the trace result */

  efm32_i2c_tracedump(priv);

  /* Ensure that any ISR happening after we finish can't overwrite any user 
   * data 
   */

  priv->dcnt = 0;
  priv->ptr = NULL;

  efm32_i2c_sem_post(dev);

  return -errval;
}

/*******************************************************************************
 * Name: efm32_i2c_write
 *
 * Description:
 *   Write I2C data
 *
 ******************************************************************************/

static int efm32_i2c_write(FAR struct i2c_dev_s *dev, const uint8_t *buffer, 
                           int buflen)
{
  struct i2c_msg_s msgv =
  {
    .addr   = ((struct efm32_i2c_inst_s *)dev)->address,
    .flags  = ((struct efm32_i2c_inst_s *)dev)->flags,
    .buffer = (uint8_t *)buffer,
    .length = buflen
  };

  return efm32_i2c_process(dev, &msgv, 1);
}

/*******************************************************************************
 * Name: efm32_i2c_read
 *
 * Description:
 *   Read I2C data
 *
 ******************************************************************************/

int efm32_i2c_read(FAR struct i2c_dev_s *dev, uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msgv =
  {
    .addr   = ((struct efm32_i2c_inst_s *)dev)->address,
    .flags  = ((struct efm32_i2c_inst_s *)dev)->flags | I2C_M_READ,
    .buffer = buffer,
    .length = buflen
  };

  return efm32_i2c_process(dev, &msgv, 1);
}

/*******************************************************************************
 * Name: efm32_i2c_writeread
 *
 * Description:
 *  Read then write I2C data
 *
 ******************************************************************************/

#ifdef CONFIG_I2C_WRITEREAD
static int efm32_i2c_writeread(FAR struct i2c_dev_s *dev,
                               const uint8_t *wbuffer, int wbuflen,
                               uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msgv[2] =
  {
    {
      .addr   = ((struct efm32_i2c_inst_s *)dev)->address,
      .flags  = ((struct efm32_i2c_inst_s *)dev)->flags,
      .buffer = (uint8_t *)wbuffer,          /* This is really ugly, sorry 
                                              * const ... 
                                              */
      .length = wbuflen
    },
    {
      .addr   = ((struct efm32_i2c_inst_s *)dev)->address,
      .flags  = ((struct efm32_i2c_inst_s *)dev)->flags | \
                ((buflen>0) ? I2C_M_READ : I2C_M_NORESTART),
      .buffer = buffer,
      .length = (buflen>0) ? buflen : -buflen
    }
  };

  return efm32_i2c_process(dev, msgv, 2);
}
#endif

/*******************************************************************************
 * Name: efm32_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ******************************************************************************/

#ifdef CONFIG_I2C_TRANSFER
static int efm32_i2c_transfer(FAR struct i2c_dev_s *dev, 
                              FAR struct i2c_msg_s *msgs, int count)
{
  return efm32_i2c_process(dev, msgs, count);
}
#endif

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/*******************************************************************************
 * Name: up_i2cinitialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ******************************************************************************/

FAR struct i2c_dev_s *up_i2cinitialize(int port)
{
  struct efm32_i2c_priv_s * priv = NULL;  /* Private data of device with 
                                           * multiple instances 
                                           */
  struct efm32_i2c_inst_s * inst = NULL;  /* Device, single instance */
  int irqs;

  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_EFM32_I2C0
    case 0:
      priv = (struct efm32_i2c_priv_s *)&efm32_i2c0_priv;
      break;
#endif
#ifdef CONFIG_EFM32_I2C1
    case 1:
      priv = (struct efm32_i2c_priv_s *)&efm32_i2c1_priv;
      break;
#endif
    default:
      return NULL;
    }

  /* Allocate instance */

  if (!(inst = kmm_malloc(sizeof(struct efm32_i2c_inst_s))))
    {
      return NULL;
    }

  /* Initialize instance */

  inst->ops       = &efm32_i2c_ops;
  inst->priv      = priv;
  inst->frequency = 100000;
  inst->address   = 0;
  inst->flags     = 0;

  /* Initialize private data for the first time, increment reference count,
   * power-up hardware and configure GPIOs.
   */

  irqs = irqsave();

  if ((volatile int)priv->refs++ == 0)
    {
      efm32_i2c_sem_init((struct i2c_dev_s *)inst);
      efm32_i2c_init(priv,inst->frequency);
    }

  irqrestore(irqs);
  return (struct i2c_dev_s *)inst;
}

/*******************************************************************************
 * Name: up_i2cuninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ******************************************************************************/

int up_i2cuninitialize(FAR struct i2c_dev_s * dev)
{
  int irqs;

  ASSERT(dev);

  /* Decrement reference count and check for underflow */

  if (((struct efm32_i2c_inst_s *)dev)->priv->refs == 0)
    {
      return ERROR;
    }

  irqs = irqsave();

  if (--((struct efm32_i2c_inst_s *)dev)->priv->refs)
    {
      irqrestore(irqs);
      kmm_free(dev);
      return OK;
    }

  irqrestore(irqs);

  /* Disable power and other HW resource (GPIO's) */

  efm32_i2c_deinit(((struct efm32_i2c_inst_s *)dev)->priv);

  /* Release unused resources */

  efm32_i2c_sem_destroy((struct i2c_dev_s *)dev);

  kmm_free(dev);
  return OK;
}

/*******************************************************************************
 * Name: up_i2creset
 *
 * Description:
 *   Reset an I2C bus
 *
 ******************************************************************************/

#ifdef CONFIG_I2C_RESET
int up_i2creset(FAR struct i2c_dev_s * dev)
{
  struct efm32_i2c_priv_s * priv;
  unsigned int clock_count;
  unsigned int stretch_count;
  uint32_t scl_gpio;
  uint32_t sda_gpio;
  int ret = ERROR;

  ASSERT(dev);

  /* Get I2C private structure */

  priv = ((struct efm32_i2c_inst_s *)dev)->priv;

  /* Our caller must own a ref */

  ASSERT(priv->refs > 0);

  /* Lock out other clients */

  efm32_i2c_sem_wait(dev);

  /* De-init the port */

  efm32_i2c_deinit(priv);

  /* Use GPIO configuration to un-wedge the bus */

  scl_gpio = MKI2C_OUTPUT(priv->config->scl_pin);
  sda_gpio = MKI2C_OUTPUT(priv->config->sda_pin);

  efm32_configgpio(scl_gpio);
  efm32_configgpio(sda_gpio);

  /* Let SDA go high */

  efm32_gpiowrite(sda_gpio, 1);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!efm32_gpioread(sda_gpio))
    {
      /* Give up if we have tried too hard */

      if (clock_count++ > 10)
        {
          goto out;
        }

      /* Sniff to make sure that clock stretching has finished.
       *
       * If the bus never relaxes, the reset has failed.
       */

      stretch_count = 0;
      while (!efm32_gpioread(scl_gpio))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      efm32_gpiowrite(scl_gpio, 0);
      up_udelay(10);

      /* Drive SCL high again */

      efm32_gpiowrite(scl_gpio, 1);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  efm32_gpiowrite(sda_gpio, 0);
  up_udelay(10);
  efm32_gpiowrite(scl_gpio, 0);
  up_udelay(10);
  efm32_gpiowrite(scl_gpio, 1);
  up_udelay(10);
  efm32_gpiowrite(sda_gpio, 1);
  up_udelay(10);

  /* Revert the GPIO configuration. */

  efm32_unconfiggpio(sda_gpio);
  efm32_unconfiggpio(scl_gpio);

  /* Re-init the port */

  efm32_i2c_init(priv,((struct efm32_i2c_inst_s *)dev)->frequency);
  ret = OK;

out:

  /* Release the port for re-use by other clients */

  efm32_i2c_sem_post(dev);
  return ret;
}
#endif /* CONFIG_I2C_RESET */

#endif /* CONFIG_EFM32_I2C0 || CONFIG_EFM32_I2C1 */

