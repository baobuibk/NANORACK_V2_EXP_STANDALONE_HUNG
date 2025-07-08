/*===========================================================================
* Super-Simple Tasker (SST0/C)
*
* Copyright (C) 2006-2023 Quantum Leaps, <state-machine.com>.
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
===========================================================================*/
#include "sst.h"        /* Super-Simple Tasker (SST) */
#include "dbc_assert.h" /* Design By Contract (DBC) assertions */

DBC_MODULE_NAME("sst0") /* for DBC assertions in this module */

/* bitmask of all SST tasks in the system */
static SST_ReadySet task_readySet;
static uint32_t SST_tick;
/* array of all SST task pointers in the system */
static SST_Task *task_registry[SST_PORT_MAX_TASK + 1U];

/*..........................................................................*/
void SST_init(void) {
    SST_tick = 0U; /* initialize the SST tick counter */
}
/*..........................................................................*/
int SST_Task_run(void) {
    SST_onStart(); /* configure and start the interrupts */

    SST_PORT_INT_DISABLE();
    for (;;) { /* event loop of the SST0 kernel */

        if (task_readySet != 0U) { /* any SST tasks ready to run? */
            uint_fast8_t const p = SST_LOG2(task_readySet);
            SST_Task * const task = task_registry[p];
            SST_PORT_INT_ENABLE();

            /* the task must have some events in the queue */
            DBC_ASSERT(100, task->nUsed > 0U);

            /* get the event out of the queue */
            /* NOTE: no critical section because task->tail is accessed only
            * from this task
            */
            // SST_Evt const *e = task->qBuf[task->tail];
            // if (task->tail == 0U) { /* need to wrap the tail? */
            //     task->tail = task->end; /* wrap around */
            // }
            // else {
            //     --task->tail;
            // }
            circular_buffer_pop(task->evt_queue, (void *)task->current_evt);
            SST_PORT_INT_DISABLE();
            //if ((--task->nUsed) == 0U) { /* no more events in the queue? */
            if (circular_buffer_is_empty(task->evt_queue)) { /* no more events? */
                task_readySet &= ~(1U << (p - 1U));
            }
            SST_PORT_INT_ENABLE();

            /* dispatch the received event to the task */
            (*task->dispatch)(task, task->current_evt); /* NOTE: virtual call */
            /* TBD: implement event recycling */
        }
        else { /* no SST tasks are ready to run --> idle */

            /* SST_onIdleCond() must be called with interrupts DISABLED
            * because the determination of the idle condition (all event
            * queues empty) can change at any time by an interrupt posting
            * events to a queue.
            *
            * NOTE: SST_onIdleCond() MUST enable interrupts internally,
            * ideally at the same time as putting the CPU into a power-
            * saving mode.
            */
            SST_onIdleCond();

            SST_PORT_INT_DISABLE(); /* disable before looping back */
        }
    }
#ifdef __GNUC__ /* GNU compiler? */
    return 0;
#endif
}
/*..........................................................................*/
void SST_Task_ctor(
    SST_Task * const me,
    SST_Handler init,
    SST_Handler dispatch,
    SST_Evt const * const current_evt,
    circular_buffer_t * const evt_queue)
{
    me->init = init;
    me->dispatch = dispatch;
    me->current_evt = current_evt; /*!< current event */
    me->evt_queue = evt_queue; /*!< circular buffer for the event queue */
}
/*..........................................................................*/
void SST_Task_start(
    SST_Task * const me,
    SST_TaskPrio prio)
{
    /*! @pre
    * - the priority must be in range
    * - the queue storage must be provided
    * - the queue length must not be zero
    * - the priority must not be in use
    */
//     DBC_REQUIRE(200,
//         (0U < prio) && (prio <= SST_PORT_MAX_TASK)
//         && (qBuf != (SST_Evt const **)0) && (qLen > 0U)
//         && (task_registry[prio] == (SST_Task *)0));

     me->prio  = prio;
    // me->qBuf  = qBuf;
    // me->end   = qLen - 1U;
    // me->head  = 0U;
    // me->tail  = 0U;
    // me->nUsed = 0U;
    DBC_REQUIRE(200,
        (0U <= prio) && (prio < SST_PORT_MAX_TASK)
        && (task_registry[prio] == (SST_Task *)0));
    task_registry[prio] = me;
    //me->nUsed = 0U; /* no events in the queue */
    /* initialize this task with the initialization event */
    (*me->init)(me, me->current_evt); /* NOTE: virtual call */
    /* TBD: implement event recycling */
}
/*..........................................................................*/
void SST_Task_post(SST_Task * const me, SST_Evt const * const e) {
    /*! @pre the queue must be sized adequately and cannot overflow */
    //DBC_REQUIRE(300, me->nUsed <= me->end);

//    SST_PORT_CRIT_STAT

    // me->qBuf[me->head] = e; /* insert event into the queue */
    // if (me->head == 0U) {   /* need to wrap the head? */
    //     me->head = me->end; /* wrap around */
    // }
    // else {
    //     --me->head;
    // }
    circular_buffer_push(me->evt_queue, e); /* insert event into the queue */
    //++me->nUsed;
    SST_PORT_CRIT_ENTRY();
    task_readySet |= (1U << (me->prio - 1U));
    SST_PORT_CRIT_EXIT();
}

/*--------------------------------------------------------------------------*/
static SST_TimeEvt *timeEvt_head = (SST_TimeEvt *)0;

/*..........................................................................*/
void SST_TimeEvt_ctor(
    SST_TimeEvt * const me,
    SST_Signal sig,
    SST_Task *task)
{
    me->super.sig = sig;
    me->task = task;
    me->ctr = 0U;
    me->interval = 0U;

    /* insert time event "me" into the linked-list */
    me->next = timeEvt_head;
    timeEvt_head = me;
}
/*..........................................................................*/
void SST_TimeEvt_arm(
    SST_TimeEvt * const me,
    SST_TCtr ctr,
    SST_TCtr interval)
{
    SST_PORT_CRIT_STAT
    SST_PORT_CRIT_ENTRY();
    me->ctr = ctr;
    me->interval = interval;
    SST_PORT_CRIT_EXIT();
}
/*..........................................................................*/
bool SST_TimeEvt_disarm(SST_TimeEvt * const me) {
    SST_PORT_CRIT_STAT
    SST_PORT_CRIT_ENTRY();
    bool status = (me->ctr != 0U);
    me->ctr = 0U;
    me->interval = 0U;
    SST_PORT_CRIT_EXIT();
    return status;
}
/*..........................................................................*/
void SST_TimeEvt_tick(void) {
    SST_tick++; /* increment the SST tick counter */
    for (SST_TimeEvt *t = timeEvt_head;
         t != (SST_TimeEvt *)0;
         t = t->next)
    {
        SST_PORT_CRIT_STAT
        SST_PORT_CRIT_ENTRY();
        if (t->ctr == 0U) { /* disarmed? (most frequent case) */
            SST_PORT_CRIT_EXIT();
        }
        else if (t->ctr == 1U) { /* expiring? */
            t->ctr = t->interval;
            SST_PORT_CRIT_EXIT();

            SST_Task_post(t->task, &t->super);
        }
        else { /* timing out */
            --t->ctr;
            SST_PORT_CRIT_EXIT();
        }
    }
}

uint32_t SST_getTick(void) {
    return SST_tick;
}
