/************************************************************************************
 * arch/arm/src/efm32/cm3_asm.h
 *
 * Copyright (c) 2009 - 2013 ARM LIMITED
 *
 *   All rights reserved.
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *   - Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   - Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   - Neither the name of ARM nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without
 *     specific prior written permission.
 *   *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 *   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 *   ---------------------------------------------------------------------------
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
 ************************************************************************************/


#ifndef __ARCH_ARM_SRC_EFM32_CM3_ASM_H_
#define __ARCH_ARM_SRC_EFM32_CM3_ASM_H_


/* GNU gcc specific functions */

/* Define macros for porting to both thumb1 and thumb2.
 * For thumb1, use low register (r0-r7), specified by constrant "l"
 * Otherwise, use general registers, specified by constrant "r" */
#if defined (__thumb__) && !defined (__thumb2__)
#define __CMSIS_GCC_OUT_REG(r) "=l" (r)
#define __CMSIS_GCC_USE_REG(r) "l" (r)
#else
#define __CMSIS_GCC_OUT_REG(r) "=r" (r)
#define __CMSIS_GCC_USE_REG(r) "r" (r)
#endif

/* name: __NOP
 *
 *  No Operation does nothing. This instruction can be used for code alignment purposes.
 */
__attribute__( ( always_inline ) ) static inline void __NOP(void)
{
  __asm volatile ("nop");
}


/*  name: __WFI
 *
 *  Wait For Interrupt is a hint instruction that suspends execution
 *  until one of a number of events occurs.
 */
__attribute__( ( always_inline ) ) static inline void __WFI(void)
{
  __asm volatile ("wfi");
}


/* name: __WFE
 *
 * Wait For Event is a hint instruction that permits the processor to enter
 * a low-power state until one of a number of events occurs.
 */
__attribute__( ( always_inline ) ) static inline void __WFE(void)
{
  __asm volatile ("wfe");
}


/* name: __SEV
 *
 *  Send Event is a hint instruction. It causes an event to be signaled to the CPU.
 */
__attribute__( ( always_inline ) ) static inline void __SEV(void)
{
  __asm volatile ("sev");
}


/* name: ISB
 *
 *  Instruction Synchronization Barrier flushes the pipeline in the processor,
 *  so that all instructions following the ISB are fetched from cache or
 *  memory, after the instruction has been completed.
 */
__attribute__( ( always_inline ) ) static inline void __ISB(void)
{
  __asm volatile ("isb");
}


/* name: __DSB
 *
 *  This function acts as a special kind of Data Memory Barrier.
 *  It completes when all explicit memory accesses before this instruction complete.
 */
__attribute__( ( always_inline ) ) static inline void __DSB(void)
{
  __asm volatile ("dsb");
}


/* name: __DMB
 *
 *  This function ensures the apparent order of the explicit memory operations before
 *  and after the instruction, without ensuring their completion.
 */
__attribute__( ( always_inline ) ) static inline void __DMB(void)
{
  __asm volatile ("dmb");
}


/* name: __REV
 *
 *  This function reverses the byte order in integer value.
 *
 *  param value:    
 *      Value to reverse
 *  return :        
 *      Reversed value
 */
__attribute__( ( always_inline ) ) static inline uint32_t __REV(uint32_t value)
{
#if (__GNUC__ > 4) || (__GNUC__ == 4 && __GNUC_MINOR__ >= 5)
  return __builtin_bswap32(value);
#else
  uint32_t result;

  __asm volatile ("rev %0, %1" : __CMSIS_GCC_OUT_REG (result) : __CMSIS_GCC_USE_REG (value) );
  return(result);
#endif
}


/* name: __REV16
 *
 * This function reverses the byte order in two unsigned short values.
 *
 * param value:
 *     Value to reverse
 * return: 
 *     Reversed value
 */
__attribute__( ( always_inline ) ) static inline uint32_t __REV16(uint32_t value)
{
  uint32_t result;

  __asm volatile ("rev16 %0, %1" : __CMSIS_GCC_OUT_REG (result) : __CMSIS_GCC_USE_REG (value) );
  return(result);
}


/* name: __REVSH
 *
 * This function reverses the byte order in a signed short value with sign extension to integer.
 *
 * param value:
 *     Value to reverse
 * return:
 *     Reversed value
 */
__attribute__( ( always_inline ) ) static inline int32_t __REVSH(int32_t value)
{
#if (__GNUC__ > 4) || (__GNUC__ == 4 && __GNUC_MINOR__ >= 8)
  return (short)__builtin_bswap16(value);
#else
  uint32_t result;

  __asm volatile ("revsh %0, %1" : __CMSIS_GCC_OUT_REG (result) : __CMSIS_GCC_USE_REG (value) );
  return(result);
#endif
}


/* name: __ROR
 *
 * This function Rotate Right (immediate) provides the value of the contents of a register rotated by a variable number of bits.
 *
 * param opt1:  
 *     Value to rotate
 * param opt2:  
 *     Number of Bits to rotate
 * return:
 *     Rotated value
 */
__attribute__( ( always_inline ) ) static inline uint32_t __ROR(uint32_t value, uint32_t nbits)
{
  return (value >> nbits) | (value << (32 - nbits)); 
}


/* name: __BKPT
 *
 * This function causes the processor to enter Debug state.
 * Debug tools can use this to investigate system state when the instruction at a particular address is reached.
 *
 * note:
 *     If required, a debugger can use it to store additional information about the breakpoint.
 *
 * param value:
 *     Value is ignored by the processor.
 * return: none
 */
#define __BKPT(value)                       __asm volatile ("bkpt "#value)


/* name __RBIT
 *
 *  This function reverses the bit order of the given value.
 *
 *  param value:
 *      Value to reverse
 *  return:              
 *      Reversed value
 */
__attribute__( ( always_inline ) ) static inline uint32_t __RBIT(uint32_t value)
{
  uint32_t result;

   __asm volatile ("rbit %0, %1" : "=r" (result) : "r" (value) );
   return(result);
}


/* name: __LDREXB
 *
 *  This function performs a exclusive LDR command for 8 bit value.
 *
 *  param ptr:
 *      Pointer to data
 *  return 
 *      Value of type uint8_t at (*ptr)
 */
__attribute__( ( always_inline ) ) static inline uint8_t __LDREXB(volatile uint8_t *ptr)
{
    uint32_t result;

#if (__GNUC__ > 4) || (__GNUC__ == 4 && __GNUC_MINOR__ >= 8)
   __asm volatile ("ldrexb %0, %1" : "=r" (result) : "Q" (*ptr) );
#else
    /* Prior to GCC 4.8, "Q" will be expanded to [rx, #0] which is not
       accepted by assembler. So has to use following less efficient pattern.
    */
   __asm volatile ("ldrexb %0, [%1]" : "=r" (result) : "r" (ptr) : "memory" );
#endif
   return(result);
}


/* name:__LDREXH
 *
 *  This function performs a exclusive LDR command for 16 bit values.
 *
 *  param ptr:
 *      Pointer to data
 *  return:        
 *      Value of type uint16_t at (*ptr)
 */
__attribute__( ( always_inline ) ) static inline uint16_t __LDREXH(volatile uint16_t *ptr)
{
    uint32_t result;

#if (__GNUC__ > 4) || (__GNUC__ == 4 && __GNUC_MINOR__ >= 8)
   __asm volatile ("ldrexh %0, %1" : "=r" (result) : "Q" (*ptr) );
#else
    /* Prior to GCC 4.8, "Q" will be expanded to [rx, #0] which is not
       accepted by assembler. So has to use following less efficient pattern.
    */
   __asm volatile ("ldrexh %0, [%1]" : "=r" (result) : "r" (ptr) : "memory" );
#endif
   return(result);
}


/* name: __LDREXW
 *
 *  This function performs a exclusive LDR command for 32 bit values.
 *
 *  param ptr:  
 *      Pointer to data
 *  return
 *      Value of type uint32_t at (*ptr)
 */
__attribute__( ( always_inline ) ) static inline uint32_t __LDREXW(volatile uint32_t *ptr)
{
    uint32_t result;

   __asm volatile ("ldrex %0, %1" : "=r" (result) : "Q" (*ptr) );
   return(result);
}


/* name: __STREXB
 *
 *  This function performs a exclusive STR command for 8 bit values.
 *
 *  param value:  
 *      Value to store
 *  param   ptr:  
 *      Pointer to location
 *  return:
 *      0  Function succeeded
 *      1  Function failed
 */
__attribute__( ( always_inline ) ) static inline uint32_t __STREXB(uint8_t value, volatile uint8_t *ptr)
{
   uint32_t result;

   __asm volatile ("strexb %0, %2, %1" : "=&r" (result), "=Q" (*ptr) : "r" (value) );
   return(result);
}


/* name: __STREXH
 *
 *  This function performs a exclusive STR command for 16 bit values.
 *
 *  param value  
 *      Value to store
 *  param ptr:  
 *      Pointer to location
 *  return:
 *      0  Function succeeded
 *      1  Function failed
 */
__attribute__( ( always_inline ) ) static inline uint32_t __STREXH(uint16_t value, volatile uint16_t *ptr)
{
   uint32_t result;

   __asm volatile ("strexh %0, %2, %1" : "=&r" (result), "=Q" (*ptr) : "r" (value) );
   return(result);
}


/* name: __STREXW
 *
 *  This function performs a exclusive STR command for 32 bit values.
 *
 *  param value:
 *      Value to store
 *  param ptr:
 *      Pointer to location
 *  return:
 *      0  Function succeeded
 *      1  Function failed
 */
__attribute__( ( always_inline ) ) static inline uint32_t __STREXW(uint32_t value, volatile uint32_t *ptr)
{
   uint32_t result;

   __asm volatile ("strex %0, %2, %1" : "=&r" (result), "=Q" (*ptr) : "r" (value) );
   return(result);
}


/* name: __CLREX
 *
 *  This function removes the exclusive lock which is created by LDREX.
 *
 */
__attribute__( ( always_inline ) ) static inline void __CLREX(void)
{
  __asm volatile ("clrex" ::: "memory");
}


/* name: __SSAT
 *
 *  This function saturates a signed value.
 *
 *  param value:
 *      Value to be saturated
 *  param sat:
 *      Bit position to saturate to (1..32)
 *  return:             
 *      Saturated value
 */
#define __SSAT(value,sat) \
({                          \
  uint32_t __RES, __ARG1 = (value); \
  __asm ("ssat %0, %1, %2" : "=r" (__RES) :  "I" (sat), "r" (value) ); \
  __RES; \
 })


/* name: __USAT
 *
 *  This function saturates an unsigned value.
 *
 *  param value  
 *      Value to be saturated
 *  param sat  
 *      Bit position to saturate to (0..31)
 *  return:
 *      Saturated value
 */
#define __USAT(value,sat) \
({                          \
  uint32_t __RES, __ARG1 = (value); \
  __asm ("usat %0, %1, %2" : "=r" (__RES) :  "I" (sat), "r" (value) ); \
  __RES; \
 })


/* name: __CLZ
 *
 *  This function counts the number of leading zeros of a data value.
 *
 *  param value:
 *      Value to count the leading zeros
 *  return:            
 *      number of leading zeros in value
 */
__attribute__( ( always_inline ) ) static inline uint8_t __CLZ(uint32_t value)
{
   uint32_t result;

  __asm volatile ("clz %0, %1" : "=r" (result) : "r" (value) );
  return(result);
}


#endif 
