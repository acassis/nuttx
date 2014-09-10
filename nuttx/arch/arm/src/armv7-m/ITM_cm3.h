/******************************************************************************
 * arch/arm/src/armv7-m/nvic.h
 *
 *   Copyright (c) 2009 - 2013 ARM LIMITED
 *
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ARM nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *  *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
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



#ifndef __ARCH_ARM_SRC_COMMON_ARMV7_M_ITM_CM3_H
#define __ARCH_ARM_SRC_COMMON_ARMV7_M_ITM_CM3_H

#include "stdint.h"

/* brief  Structure type to access the Instrumentation Trace Macrocell Register (ITM).  */
#define ITM_BASE      (0xE0000000UL)
/* i ITM port used : 0-31 */
#define ITM_PORT(i)   (ITM_BASE+(i*4)) /* Stimulus Port 32-bit                  */
#define ITM_TER       (ITM_BASE+0xE00) /* Trace Enable Register                 */
#define ITM_TPR       (ITM_BASE+0xE40) /* Trace Privilege Register              */
#define ITM_TCR       (ITM_BASE+0xE80) /* Trace Control Register                */
#define ITM_IWR       (ITM_BASE+0xEF8) /* Integration Write Register            */
#define ITM_IRR       (ITM_BASE+0xEFC) /* Integration Read Register             */
#define ITM_IMCR      (ITM_BASE+0xF00) /* Integration Mode Control Register     */
#define ITM_LAR       (ITM_BASE+0xFB0) /* Lock Access Register                  */
#define ITM_LSR       (ITM_BASE+0xFB4) /* Lock Status Register                  */
#define ITM_PID4      (ITM_BASE+0xFD0) /* Peripheral Identification Register #4 */
#define ITM_PID5      (ITM_BASE+0xFD4) /* Peripheral Identification Register #5 */
#define ITM_PID6      (ITM_BASE+0xFD8) /* Peripheral Identification Register #6 */
#define ITM_PID7      (ITM_BASE+0xFDC) /* Peripheral Identification Register #7 */
#define ITM_PID0      (ITM_BASE+0xFE0) /* Peripheral Identification Register #0 */
#define ITM_PID1      (ITM_BASE+0xFE4) /* Peripheral Identification Register #1 */
#define ITM_PID2      (ITM_BASE+0xFE8) /* Peripheral Identification Register #2 */
#define ITM_PID3      (ITM_BASE+0xFEC) /* Peripheral Identification Register #3 */
#define ITM_CID0      (ITM_BASE+0xFF0) /* Component  Identification Register #0 */
#define ITM_CID1      (ITM_BASE+0xFF4) /* Component  Identification Register #1 */
#define ITM_CID2      (ITM_BASE+0xFF8) /* Component  Identification Register #2 */
#define ITM_CID3      (ITM_BASE+0xFFC) /* Component  Identification Register #3 */

#define ITM_TPR_PRIVMASK_Pos                0                                             /* ITM TPR: PRIVMASK Position */
#define ITM_TPR_PRIVMASK_Msk               (0xFUL << ITM_TPR_PRIVMASK_Pos)                /* ITM TPR: PRIVMASK Mask */

#define ITM_TCR_BUSY_Pos                   23                                             /* ITM TCR: BUSY Position */
#define ITM_TCR_BUSY_Msk                   (1UL << ITM_TCR_BUSY_Pos)                      /* ITM TCR: BUSY Mask */

#define ITM_TCR_TraceBusID_Pos             16                                             /* ITM TCR: ATBID Position */
#define ITM_TCR_TraceBusID_Msk             (0x7FUL << ITM_TCR_TraceBusID_Pos)             /* ITM TCR: ATBID Mask */

#define ITM_TCR_GTSFREQ_Pos                10                                             /* ITM TCR: Global timestamp frequency Position */
#define ITM_TCR_GTSFREQ_Msk                (3UL << ITM_TCR_GTSFREQ_Pos)                   /* ITM TCR: Global timestamp frequency Mask */

#define ITM_TCR_TSPrescale_Pos              8                                             /* ITM TCR: TSPrescale Position */
#define ITM_TCR_TSPrescale_Msk             (3UL << ITM_TCR_TSPrescale_Pos)                /* ITM TCR: TSPrescale Mask */

#define ITM_TCR_SWOENA_Pos                  4                                             /* ITM TCR: SWOENA Position */
#define ITM_TCR_SWOENA_Msk                 (1UL << ITM_TCR_SWOENA_Pos)                    /* ITM TCR: SWOENA Mask */

#define ITM_TCR_DWTENA_Pos                  3                                             /* ITM TCR: DWTENA Position */
#define ITM_TCR_DWTENA_Msk                 (1UL << ITM_TCR_DWTENA_Pos)                    /* ITM TCR: DWTENA Mask */

#define ITM_TCR_SYNCENA_Pos                 2                                             /* ITM TCR: SYNCENA Position */
#define ITM_TCR_SYNCENA_Msk                (1UL << ITM_TCR_SYNCENA_Pos)                   /* ITM TCR: SYNCENA Mask */

#define ITM_TCR_TSENA_Pos                   1                                             /* ITM TCR: TSENA Position */
#define ITM_TCR_TSENA_Msk                  (1UL << ITM_TCR_TSENA_Pos)                     /* ITM TCR: TSENA Mask */

#define ITM_TCR_ITMENA_Pos                  0                                             /* ITM TCR: ITM Enable bit Position */
#define ITM_TCR_ITMENA_Msk                 (1UL << ITM_TCR_ITMENA_Pos)                    /* ITM TCR: ITM Enable bit Mask */

#define ITM_IWR_ATVALIDM_Pos                0                                             /* ITM IWR: ATVALIDM Position */
#define ITM_IWR_ATVALIDM_Msk               (1UL << ITM_IWR_ATVALIDM_Pos)                  /* ITM IWR: ATVALIDM Mask */

#define ITM_IRR_ATREADYM_Pos                0                                             /* ITM IRR: ATREADYM Position */
#define ITM_IRR_ATREADYM_Msk               (1UL << ITM_IRR_ATREADYM_Pos)                  /* ITM IRR: ATREADYM Mask */

#define ITM_IMCR_INTEGRATION_Pos            0                                             /* ITM IMCR: INTEGRATION Position */
#define ITM_IMCR_INTEGRATION_Msk           (1UL << ITM_IMCR_INTEGRATION_Pos)              /* ITM IMCR: INTEGRATION Mask */

#define ITM_LSR_ByteAcc_Pos                 2                                             /* ITM LSR: ByteAcc Position */
#define ITM_LSR_ByteAcc_Msk                (1UL << ITM_LSR_ByteAcc_Pos)                   /* ITM LSR: ByteAcc Mask */

#define ITM_LSR_Access_Pos                  1                                             /* ITM LSR: Access Position */
#define ITM_LSR_Access_Msk                 (1UL << ITM_LSR_Access_Pos)                    /* ITM LSR: Access Mask */

#define ITM_LSR_Present_Pos                 0                                             /* ITM LSR: Present Position */
#define ITM_LSR_Present_Msk                (1UL << ITM_LSR_Present_Pos)                   /* ITM LSR: Present Mask */




/* ##################################### Debug In/Output function ########################################### */

#ifdef __cplusplus
extern "C"
{
#endif

extern volatile int32_t ITM_RxBuffer;                    /* External variable to receive characters.                         */
#define                 ITM_RXBUFFER_EMPTY    0x5AA55AA5 /* Value identifying ITM_RxBuffer is ready for next character. */


extern uint32_t    ITM_SendChar    (uint32_t ch);
extern int32_t     ITM_ReceiveChar (void);
extern int32_t     ITM_CheckChar   (void);

#ifdef __cplusplus
}
#endif

#endif
