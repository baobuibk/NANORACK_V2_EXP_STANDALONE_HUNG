/*
 * min_shell.h
 *
 *  Created on: Jun 13, 2025
 *      Author: Admin
 */

#ifndef MIN_SHELL_H_
#define MIN_SHELL_H_

#include "sst.h"
#include  "min_shell.h"
#include "fsm.h"
#include "min.h"
#include "uart_stdio.h"


typedef struct min_shell_task_t min_shell_task_t;
typedef struct min_shell_evt_t min_shell_evt_t;
typedef struct min_shell_task_init_t min_shell_task_init_t;
typedef state_t (*min_shell_state_handler_t)(min_shell_task_t * const me, min_shell_evt_t const * const e);
struct min_shell_task_t {
	SST_Task super; // Base task structure
    min_shell_state_handler_t state; // Current state handler
    // Add additional fields if needed for your shell task
    struct min_context *min_context; // Pointer to the context for the shell task
    UART_stdio_t *min_shell_uart; // Pointer to the UART for MIN communication
    min_shell_evt_t const *current_evt; // Pointer to the current event being processed
    SST_TimeEvt         min_poll_timer; // Timer for periodic polling
};
struct min_shell_evt_t {
	SST_Evt super; // Base event structure
    // Add additional fields if needed for your shell events
    uint8_t *data_buff; // Pointer to the data associated with the event
    uint32_t data_len; // Length of the data in the event
};
struct min_shell_task_init_t {
    min_shell_state_handler_t init_state; // Initial state handler
    struct min_context *min_context; // Pointer to the context for the shell task
    UART_stdio_t *min_shell_uart; // Pointer to the UART for MIN communication
    min_shell_evt_t const *current_evt; // Pointer to the current event being processed
    circular_buffer_t *min_shell_event_buffer; // Pointer to the circular buffer for events
};
void min_shell_task_ctor_singleton();
#endif /* APP_MIN_SHELL_MIN_SHELL_H_ */
