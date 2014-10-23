/*******************************************************************************
 * arch/arm/src/efm32/em_bitband.h
 * 
 *    (C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *   Copyright (C) 2014 Pierre-noel Bouteville . All rights reserved.
 *   Author: Pierre-noel Bouteville <pnb990@gmail.com>
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


#ifndef __ARCH_ARM_SRC_EFM32_EM_BITBAND_H
#define __ARCH_ARM_SRC_EFM32_EM_BITBAND_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * brief
 *   Perform bit-band operation on peripheral memory location.
 *
 * details
 *   Bit-banding provides atomic read-modify-write cycle for single bit
 *   modification. Please refer to the reference manual for further details
 *   about bit-banding.
 *
 * note
 *   This function is only atomic on cores which fully support bitbanding.
 *
 * param[in] addr Peripheral address location to modify bit in.
 *
 * param[in] bit Bit position to modify, 0-31.
 *
 * param[in] val Value to set bit to, 0 or 1.
 ******************************************************************************/
static inline void BITBAND_Peripheral(volatile uint32_t *addr,
                                        uint32_t bit,
                                        uint32_t val)
{
#if defined(BITBAND_PER_BASE)
  uint32_t tmp =
    BITBAND_PER_BASE + (((uint32_t)addr - PER_MEM_BASE) * 32) + (bit * 4);

  *((volatile uint32_t *)tmp) = (uint32_t)val;
#else
  uint32_t tmp = *addr;
  /* Make sure val is not more than 1, because we only want to set one bit. */
  val &= 0x1;
  *addr = (tmp & ~(1 << bit)) | (val << bit);
#endif /* defined(BITBAND_PER_BASE) */
}


/*******************************************************************************
 * brief
 *   Perform a read operation on the peripheral bit-band memory location.
 *
 * details
 *   This function reads a single bit from the peripheral bit-band alias region.
 *   Bit-banding provides atomic read-modify-write cycle for single bit
 *   modification. Please refer to the reference manual for further details
 *   about bit-banding.
 *
 * param[in] addr   Peripheral address location to read.
 *
 * param[in] bit    Bit position to read, 0-31.
 *
 * return           Value of the requested bit.
 ******************************************************************************/
static inline uint32_t BITBAND_PeripheralRead(volatile uint32_t *addr,
                                                uint32_t bit)
{
#if defined(BITBAND_PER_BASE)
  uint32_t tmp =
    BITBAND_PER_BASE + (((uint32_t)addr - PER_MEM_BASE) * 32) + (bit * 4);

  return *((volatile uint32_t *)tmp);
#else
  return ((*addr) >> bit) & 1;
#endif /* defined(BITBAND_PER_BASE) */
}


/*******************************************************************************
 * brief
 *   Perform bit-band operation on SRAM memory location.
 *
 * details
 *   Bit-banding provides atomic read-modify-write cycle for single bit
 *   modification. Please refer to the reference manual for further details
 *   about bit-banding.
 *
 * note
 *   This function is only atomic on cores which fully support bitbanding.
 *
 * param[in] addr SRAM address location to modify bit in.
 *
 * param[in] bit Bit position to modify, 0-31.
 *
 * param[in] val Value to set bit to, 0 or 1.
 ******************************************************************************/
static inline void BITBAND_SRAM(uint32_t *addr, uint32_t bit, uint32_t val)
{
#if defined(BITBAND_RAM_BASE)
  uint32_t tmp =
    BITBAND_RAM_BASE + (((uint32_t)addr - RAM_MEM_BASE) * 32) + (bit * 4);

  *((volatile uint32_t *)tmp) = (uint32_t)val;
#else
  uint32_t tmp = *addr;
  /* Make sure val is not more than 1, because we only want to set one bit. */
  val &= 0x1;
  *addr = (tmp & ~(1 << bit)) | (val << bit);
#endif /* defined(BITBAND_RAM_BASE) */
}


/*******************************************************************************
 * brief
 *   Read a single bit from the SRAM bit-band alias region.
 *
 * details
 *   This function reads a single bit from the SRAM bit-band alias region.
 *   Bit-banding provides atomic read-modify-write cycle for single bit
 *   modification. Please refer to the reference manual for further details
 *   about bit-banding.
 *
 * param[in] addr    SRAM address location to modify bit in.
 *
 * param[in] bit     Bit position to modify, 0-31.
 *
 * return            Value of the requested bit.
 ******************************************************************************/
static inline uint32_t BITBAND_SRAMRead(uint32_t *addr, uint32_t bit)
{
#if defined(BITBAND_RAM_BASE)
  uint32_t tmp =
    BITBAND_RAM_BASE + (((uint32_t)addr - RAM_MEM_BASE) * 32) + (bit * 4);

  return *((volatile uint32_t *)tmp);
#else
  return ((*addr) >> bit) & 1;
#endif /* defined(BITBAND_RAM_BASE) */
}


#ifdef __cplusplus
}
#endif

#endif /* __EM_BITBAND_H */
