/*============================================================================
* Super-Simple Tasker (SST0/C) port to ARM Cortex-M
*
* Copyright (C) 2005-2023 Quantum Leaps, <state-machine.com>.
*
* SPDX-License-Identifier: MIT
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.
============================================================================*/
#ifndef SST_PORT_H_
#define SST_PORT_H_

#include <stm32f7xx.h>
#define SST_PORT_TASK_ATTR SST_TaskPrio prio;

#define SST_PORT_MAX_TASK 32U

/* SST-PORT disabling/enabling interrupts */
#define SST_PORT_INT_DISABLE() __asm volatile ("cpsid i")
#define SST_PORT_INT_ENABLE()  __asm volatile ("cpsie i")

// Define BASEPRI priority level (0-15, lower number = higher priority)
#define BASE_PRI 2

// Macro để vào critical section
#define SST_ENTER_CRITICAL() do { \
    __set_BASEPRI(BASE_PRI << (8 - __NVIC_PRIO_BITS)); \
} while (0)

// Macro để thoát critical section
#define SST_EXIT_CRITICAL() do { \
    __set_BASEPRI(0); \
} while (0)

/* SST-PORT critical section */
#define SST_PORT_CRIT_STAT
//#define SST_PORT_CRIT_ENTRY() SST_PORT_INT_DISABLE()
//#define SST_PORT_CRIT_EXIT()  SST_PORT_INT_ENABLE()
#define SST_PORT_CRIT_ENTRY() SST_ENTER_CRITICAL()
#define SST_PORT_CRIT_EXIT()  SST_EXIT_CRITICAL()

typedef uint32_t SST_ReadySet;

/* special idle callback to handle the "idle condition" in SST0 */
void SST_onIdleCond(void);

/* the SST scheduler lock key type */
typedef uint32_t SST_LockKey;

#if (__ARM_ARCH == 6) /* ARMv6-M? */

/* SST_LOG2() implementation for ARMv6-M (no CLZ instruction) */
static inline uint_fast8_t SST_LOG2(uint32_t x) {
    static uint8_t const log2LUT[16] = {
        0U, 1U, 2U, 2U, 3U, 3U, 3U, 3U,
        4U, 4U, 4U, 4U, 4U, 4U, 4U, 4U
    };
    uint_fast8_t n = 0U;
    SST_ReadySet tmp;

    #if (SST_PORT_MAX_TASK > 16U)
    tmp = (SST_ReadySet)(x >> 16U);
    if (tmp != 0U) {
        n += 16U;
        x = tmp;
    }
    #endif
    #if (SST_PORT_MAX_TASK > 8U)
    tmp = (x >> 8U);
    if (tmp != 0U) {
        n += 8U;
        x = tmp;
    }
    #endif
    tmp = (x >> 4U);
    if (tmp != 0U) {
        n += 4U;
        x = tmp;
    }
    return n + log2LUT[x];
}

#else /* ARMv7-M+ have CLZ instruction for fast LOG2 computations */

#if defined __ARMCC_VERSION
    #define SST_LOG2(x_) ((uint_fast8_t)(32U - __builtin_clz((unsigned)(x_))))
#elif defined __GNUC__
    #define SST_LOG2(x_) ((uint_fast8_t)(32U - __builtin_clz((unsigned)(x_))))
#elif defined __ICCARM__
    #include <intrinsics.h>
    #define SST_LOG2(x_) ((uint_fast8_t)(32U - __CLZ((unsigned long)(x_))))
#endif /* compiler type */

#endif

#endif /* SST_PORT_H_ */
