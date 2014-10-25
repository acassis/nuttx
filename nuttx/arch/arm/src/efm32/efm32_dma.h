/************************************************************************************
 * arch/arm/src/efm32/efm32_dma.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_EFM32_EFM32_DMA_H
#define __ARCH_ARM_SRC_EFM32_EFM32_DMA_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "chip.h"
#include "chip/efm32_dma.h"

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* DMA_HANDLE provides an opaque are reference that can be used to represent a DMA
 * channel.
 */

typedef FAR void *DMA_HANDLE;

/* Description:
 *   This is the type of the callback that is used to inform the user of the the
 *   completion of the DMA.
 *
 * Input Parameters:
 *   handle - Refers tot he DMA channel or stream
 *   status - A bit encoded value that provides the completion status.  See the
 *            DMASTATUS_* definitions above.
 *   arg    - A user-provided value that was provided when efm32_dmastart() was
 *            called.
 */

typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t status, void *arg);

#ifdef CONFIG_DEBUG_DMA
struct efm32_dmaregs_s
{
  uint32_t status;            /* DMA Status Registers */
  uint32_t ctrlbase;          /* Channel Control Data Base Pointer Register */
  uint32_t altctrlbase;       /* Channel Alternate Control Data Base Pointer Register */
  uint32_t chwaitstatus;      /* Channel Wait on Request Status Register */
  uint32_t chusebursts;       /* Channel Useburst Set Register */
  uint32_t chreqmasks;        /* Channel Request Mask Set Register */
  uint32_t chens;             /* Channel Enable Set Register */
  uint32_t chalts;            /* Channel Alternate Set Register */
  uint32_t chpris;            /* Channel Priority Set Register */
  uint32_t errorc;            /* Bus Error Clear Register */
  uint32_t chreqstatus;       /* Channel Request Status */
  uint32_t chsreqstatus;      /* Channel Single Request Status */
  uint32_t ifr;               /* Interrupt Flag Register */
  uint32_t ien;               /* Interrupt Enable register */
#if defined(CONFIG_EFM32_EFM32GG)
  uint32_t ctrl;              /* DMA Control Register */
  uint32_t rds;               /* DMA Retain Descriptor State */
  uint32_t loop0;             /* Channel 0 Loop Register */
  uint32_t loop1;             /* Channel 1 Loop Register */
  uint32_t rect0;             /* Channel 0 Rectangle Register */
#endif
  uint32_t chcgrl             /* Channel n Control Register */
};
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/****************************************************************************
 * Name: efm32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'chan' argument.
 *   DMA channels are shared on the EFM32:  Devices sharing the same DMA
 *   channel cannot do DMA concurrently!  See the DMACHAN_* definitions in
 *   efm32_dma.h.
 *
 *   If the DMA channel is not available, then efm32_dmachannel() will wait
 *   until the holder of the channel relinquishes the channel by calling
 *   efm32_dmafree().  WARNING: If you have two devices sharing a DMA
 *   channel and the code never releases the channel, the efm32_dmachannel
 *   call for the other will hang forever in this function!  Don't let your
 *   design do that!
 *
 *   Hmm.. I suppose this interface could be extended to make a non-blocking
 *   version.  Feel free to do that if that is what you need.
 *
 * Input parameter:
 *   chan - Identifies the channel resource
 *
 * Returned Value:
 *   Provided that 'chan' is valid, this function ALWAYS returns a non-NULL,
 *   void* DMA channel handle.  (If 'chan' is invalid, the function will
 *   assert if debug is enabled or do something ignorant otherwise).
 *
 * Assumptions:
 *   - The caller does not hold he DMA channel.
 *   - The caller can wait for the DMA channel to be freed if it is no
 *     available.
 *
 ****************************************************************************/

DMA_HANDLE efm32_dmachannel(unsigned int chan);

/****************************************************************************
 * Name: efm32_dmafree
 *
 * Description:
 *   Release a DMA channel.  If another thread is waiting for this DMA channel
 *   in a call to efm32_dmachannel, then this function will re-assign the
 *   DMA channel to that thread and wake it up.  NOTE:  The 'handle' used
 *   in this argument must NEVER be used again until efm32_dmachannel() is
 *   called again to re-gain access to the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void efm32_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: efm32_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

void efm32_dmasetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                    size_t ntransfers, uint32_t ccr);

/****************************************************************************
 * Name: efm32_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void efm32_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                    bool half);

/****************************************************************************
 * Name: efm32_dmastop
 *
 * Description:
 *   Cancel the DMA.  After efm32_dmastop() is called, the DMA channel is
 *   reset and efm32_dmasetup() must be called before efm32_dmastart() can be
 *   called again
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *
 ****************************************************************************/

void efm32_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: efm32_dmaresidual
 *
 * Description:
 *   Returns the number of bytes remaining to be transferred
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *
 ****************************************************************************/

size_t efm32_dmaresidual(DMA_HANDLE handle);

/****************************************************************************
 * Name: efm32_dmacapable
 *
 * Description:
 *   Check if the DMA controller can transfer data to/from given memory
 *   address with the given configuration. This depends on the internal
 *   connections in the ARM bus matrix of the processor. Note that this
 *   only applies to memory addresses, it will return false for any peripheral
 *   address.
 *
 * Returned value:
 *   True, if transfer is possible.
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_DMACAPABLE
bool efm32_dmacapable(uintptr_t maddr, uint32_t count, uint32_t ccr);
#else
#  define efm32_dmacapable(maddr, count, ccr) (true)
#endif

/****************************************************************************
 * Name: efm32_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void efm32_dmasample(DMA_HANDLE handle, struct efm32_dmaregs_s *regs);
#else
#  define efm32_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: efm32_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void efm32_dmadump(DMA_HANDLE handle, const struct efm32_dmaregs_s *regs,
                   const char *msg);
#else
#  define efm32_dmadump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_EFM32_EFM32_DMA_H */
