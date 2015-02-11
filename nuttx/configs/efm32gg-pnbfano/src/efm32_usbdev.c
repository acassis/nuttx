/************************************************************************************
 * configs/efm32gg-pnbfano/src/efm32_usbdev.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <chip/efm32_usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include "up_arch.h"

#ifdef speLOG_EFM32_USBDEV
#   undef  speLOG
#   define speLOG speLOG_EFM32_USBDEV
#endif

#ifndef speLOG
#   define speLOG syslog
#endif

#ifdef CONFIG_USBDEV
#   include <nuttx/usb/usbdev.h>
#endif

#ifdef CONFIG_USBDEV_TRACE
#   include <nuttx/usb/usbdev_trace.h>

#ifndef TRACE_BITSET
#  define TRACE_BITSET              (TRACE_ALLBITS)
#endif

#endif


/******************************************************************************\
 * Definitions
 ******************************************************************************/

/* Configuration **************************************************************/
#define __USBMSC_NLUNS      1
#define __USBMSC_DEVPATH1   "/dev/mmcsd0"

/******************************************************************************\
 * Private Functions
 ******************************************************************************/

static int efm32_usbmsc_connect(void);
static int efm32_usbmsc_discon(void);

/******************************************************************************\
 * Private Data
 ******************************************************************************/

static void *handle;

/******************************************************************************\
 * Name: usbmsc_enumerate
 ******************************************************************************/

#if CONFIG_USBDEV_TRACE
static int usbmsc_enumerate(struct usbtrace_s *trace, void *arg)
{
  switch (trace->event)
    {
    case TRACE_DEVINIT:
      speLOG(LOG_DEBUG,"USB controller initialization: %04x\n", trace->value);
      break;

    case TRACE_DEVUNINIT:
      speLOG(LOG_DEBUG,"USB controller un-initialization: %04x\n", trace->value);
      break;

    case TRACE_DEVREGISTER:
      speLOG(LOG_DEBUG,"usbdev_register(): %04x\n", trace->value);
      break;

    case TRACE_DEVUNREGISTER:
      speLOG(LOG_DEBUG,"usbdev_unregister(): %04x\n", trace->value);
      break;

    case TRACE_EPCONFIGURE:
      speLOG(LOG_DEBUG,"Endpoint configure(): %04x\n", trace->value);
      break;

    case TRACE_EPDISABLE:
      speLOG(LOG_DEBUG,"Endpoint disable(): %04x\n", trace->value);
      break;

    case TRACE_EPALLOCREQ:
      speLOG(LOG_DEBUG,"Endpoint allocreq(): %04x\n", trace->value);
      break;

    case TRACE_EPFREEREQ:
      speLOG(LOG_DEBUG,"Endpoint freereq(): %04x\n", trace->value);
      break;

    case TRACE_EPALLOCBUFFER:
      speLOG(LOG_DEBUG,"Endpoint allocbuffer(): %04x\n", trace->value);
      break;

    case TRACE_EPFREEBUFFER:
      speLOG(LOG_DEBUG,"Endpoint freebuffer(): %04x\n", trace->value);
      break;

    case TRACE_EPSUBMIT:
      speLOG(LOG_DEBUG,"Endpoint submit(): %04x\n", trace->value);
      break;

    case TRACE_EPCANCEL:
      speLOG(LOG_DEBUG,"Endpoint cancel(): %04x\n", trace->value);
      break;

    case TRACE_EPSTALL:
      speLOG(LOG_DEBUG,"Endpoint stall(true): %04x\n", trace->value);
      break;

    case TRACE_EPRESUME:
      speLOG(LOG_DEBUG,"Endpoint stall(false): %04x\n", trace->value);
      break;

    case TRACE_DEVALLOCEP:
      speLOG(LOG_DEBUG,"Device allocep(): %04x\n", trace->value);
      break;

    case TRACE_DEVFREEEP:
      speLOG(LOG_DEBUG,"Device freeep(): %04x\n", trace->value);
      break;

    case TRACE_DEVGETFRAME:
      speLOG(LOG_DEBUG,"Device getframe(): %04x\n", trace->value);
      break;

    case TRACE_DEVWAKEUP:
      speLOG(LOG_DEBUG,"Device wakeup(): %04x\n", trace->value);
      break;

    case TRACE_DEVSELFPOWERED:
      speLOG(LOG_DEBUG,"Device selfpowered(): %04x\n", trace->value);
      break;

    case TRACE_DEVPULLUP:
      speLOG(LOG_DEBUG,"Device pullup(): %04x\n", trace->value);
      break;

    case TRACE_CLASSBIND:
      speLOG(LOG_DEBUG,"Class bind(): %04x\n", trace->value);
      break;

    case TRACE_CLASSUNBIND:
      speLOG(LOG_DEBUG,"Class unbind(): %04x\n", trace->value);
      break;

    case TRACE_CLASSDISCONNECT:
      speLOG(LOG_DEBUG,"Class disconnect(): %04x\n", trace->value);
      break;

    case TRACE_CLASSSETUP:
      speLOG(LOG_DEBUG,"Class setup(): %04x\n", trace->value);
      break;

    case TRACE_CLASSSUSPEND:
      speLOG(LOG_DEBUG,"Class suspend(): %04x\n", trace->value);
      break;

    case TRACE_CLASSRESUME:
      speLOG(LOG_DEBUG,"Class resume(): %04x\n", trace->value);
      break;

    case TRACE_CLASSRDCOMPLETE:
      speLOG(LOG_DEBUG,"Class RD request complete: %04x\n", trace->value);
      break;

    case TRACE_CLASSWRCOMPLETE:
      speLOG(LOG_DEBUG,"Class WR request complete: %04x\n", trace->value);
      break;

    default:
      switch (TRACE_ID(trace->event))
        {
        case TRACE_CLASSAPI_ID:        /* Other class driver system API calls */
          speLOG(LOG_DEBUG,"Class API call %d: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_CLASSSTATE_ID:      /* Track class driver state changes */
          speLOG(LOG_DEBUG,"Class state %d: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_INTENTRY_ID:        /* Interrupt handler entry */
          speLOG(LOG_DEBUG,"Interrupt %d entry: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_INTDECODE_ID:       /* Decoded interrupt trace->event */
          speLOG(LOG_DEBUG,"Interrupt decode %d: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_INTEXIT_ID:         /* Interrupt handler exit */
          speLOG(LOG_DEBUG,"Interrupt %d exit: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_OUTREQQUEUED_ID:    /* Request queued for OUT endpoint */
          speLOG(LOG_DEBUG,"EP%d OUT request queued: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_INREQQUEUED_ID:     /* Request queued for IN endpoint */
          speLOG(LOG_DEBUG,"EP%d IN request queued: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_READ_ID:            /* Read (OUT) action */
          speLOG(LOG_DEBUG,"EP%d OUT read: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_WRITE_ID:           /* Write (IN) action */
          speLOG(LOG_DEBUG,"EP%d IN write: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_COMPLETE_ID:        /* Request completed */
          speLOG(LOG_DEBUG,"EP%d request complete: %04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_DEVERROR_ID:        /* USB controller driver error event */
          speLOG(LOG_DEBUG,"Controller error: %02x:%04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        case TRACE_CLSERROR_ID:        /* USB class driver error event */
          speLOG(LOG_DEBUG,"Class error: %02x:%04x\n",
                 TRACE_DATA(trace->event), trace->value);
          break;

        default:
          speLOG(LOG_DEBUG,"Unrecognized event: %02x:%02x:%04x\n",
                  TRACE_ID(trace->event) >> 8,
                  TRACE_DATA(trace->event), trace->value);
          break;
        }
    }
  return 0;
}
#endif


static int efm32_usbmsc_connect(void)
{
#ifdef CONFIG_USBMSC
    int ret = 0;

    if ( ret >= 0 )
    {
        speLOG(LOG_INFO,"Configuring with NLUNS=%d\n", __USBMSC_NLUNS);
        ret = usbmsc_configure(__USBMSC_NLUNS, &handle);
        if (ret < 0)
        {
            speLOG(LOG_ERR,"usbmsc_configure failed: %d\n", -ret);
        }
    }

    if ( ret >= 0 )
    {
        speLOG(LOG_INFO,"Bind LUN=0 to %s\n", __USBMSC_DEVPATH1);
        ret = usbmsc_bindlun(handle, __USBMSC_DEVPATH1, 0, 0, 0, false);
        if (ret < 0)
        {
            speLOG(LOG_ERR,"usbmsc_bindlun failed for LUN 0 using %s: %d\n",
                   __USBMSC_DEVPATH1, -ret);
        }
    }

    if ( ret >= 0 )
    {
        speLOG(LOG_INFO,"export luns\n");
        ret = usbmsc_exportluns(handle);
        if (ret < 0)
        {
            speLOG(LOG_ERR,"usbmsc_exportluns failed: %d\n", -ret);
        }
    }



#endif

    if ( ret < 0 )
    {
        efm32_usbmsc_discon();
        return -1;
    }

    return 0;
}

static int efm32_usbmsc_discon(void)
{
#ifdef CONFIG_USBMSC
    if ( handle )
    {
        speLOG(LOG_ERR,"USB uninitialization\n");
        usbmsc_uninitialize(handle);
        handle = NULL;
    }
#endif

    return 0;
}
/******************************************************************************
 * Public Functions
 *****************************************************************************/

int efm32_usbdev_init(void)
{
#ifdef CONFIG_USBDEV_TRACE
    usbtrace_enable(TRACE_BITSET);
#endif

    /* Enable voltage regulator output sensing to detect usb plug/unplug */

    modifyreg32(EFM32_USB_CTRL,0,_USB_CTRL_VREGOSEN_MASK);

    return 0;
}

void efm32_usbdev_slow_poll(void)
{
    int ret;

    /* check USB regulator output sensing only if Active */

    if ( getreg32(EFM32_USB_CTRL) & _USB_CTRL_VREGOSEN_MASK )
    {
        /* Check regulator output */
        if ( getreg32(EFM32_USB_STATUS) & _USB_STATUS_VREGOS_MASK )
        {
            if ( handle == NULL )
            {
                efm32_usbmsc_connect();
            }
        }
        else
        {
            if ( handle != NULL )
            {
                efm32_usbmsc_discon();
            }
        }
    }
    else
    {
        speLOG(LOG_WARNING,"USB regulator output sensing disabled !\n");
    }

#ifdef CONFIG_USBDEV_TRACE
    speLOG(LOG_DEBUG,"USB TRACE DATA:\n");
    ret = usbtrace_enumerate(usbmsc_enumerate, NULL);

    if ( ret < 0 )
        speLOG(LOG_ERR,"USB TRACE DATA Failed %d\n",ret);
#endif

}



/************************************************************************************
 * Name:  efm32_usbsuspend
 *
 * Description:
 *   Board logic must provide the stm32_usbsuspend logic if the USBDEV driver is
 *   used.  This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power, etc.
 *   while the USB is suspended.
 *
 ************************************************************************************/

void efm32_usbsuspend(FAR struct usbdev_s *dev, bool resume)
{
  ulldbg("resume: %d\n", resume);
}

