/*
 * min_shell.c
 *
 *  Created on: Jun 13, 2025
 *      Author: Admin
 */
#include "min_shell.h"
#include "min_shell_command.h"
#include "min.h"
#include "stddef.h"
#include "app_signals.h"
#include "configs.h"
#include "DBC_assert.h"
#include "board.h"

DBC_MODULE_NAME("min_shell")

#define MIN_SHELL_POLL_INTERVAL_MS 10 // Polling interval in milliseconds
#define MIN_SHELL_TASK_NUM_EVENTS 10
#define MIN_SHELL_RECEIVED_DATA_BUFFER_SIZE 256
min_shell_task_t min_shell_inst;

//static struct min_context min_ctx;

static MIN_Context_t min_ctx;
static MIN_Context_t *registered_contexts[MAX_MIN_CONTEXTS] = {0};

min_shell_evt_t min_shell_current_event = {0}; // Current event being processed
min_shell_evt_t min_shell_task_event_buffer[MIN_SHELL_TASK_NUM_EVENTS] = {0}; // Array to hold shell events
circular_buffer_t min_shell_task_event_queue = {0}; // Circular buffer to hold shell events
static state_t min_shell_state_process_handler(min_shell_task_t * const me, min_shell_evt_t const * const e);
static void min_shell_task_dispatch(min_shell_task_t * const me, min_shell_evt_t * const e) ;

static uint8_t min_shell_received_data[MIN_SHELL_RECEIVED_DATA_BUFFER_SIZE];
static uint32_t min_shell_received_length = MIN_SHELL_RECEIVED_DATA_BUFFER_SIZE;
static min_shell_evt_t const entry_evt = {.super = {.sig = SIG_ENTRY} };
static min_shell_evt_t const exit_evt = {.super = {.sig = SIG_EXIT} };

#define MIN_SHELL_UART_BUFFER_SIZE 256


static circular_char_buffer_t rx_buffer;
static circular_char_buffer_t tx_buffer;
static uint8_t rx_static_buffer[MIN_SHELL_UART_BUFFER_SIZE];
static uint8_t tx_static_buffer[MIN_SHELL_UART_BUFFER_SIZE];
UART_stdio_t min_shell_stdio; // UART for MIN communication


void MIN_Context_Init(MIN_Context_t *ctx, uint8_t port) {
    min_init_context(&ctx->min_ctx, port);
    min_transport_reset(&ctx->min_ctx, true);
    ctx->last_poll_time = min_time_ms();
    ctx->timeout_triggered = false;
#ifdef AUTO_REINIT_ON_TIMEOUT
    ctx->auto_reinit = true;
#endif
    if (port < MAX_MIN_CONTEXTS) {
        registered_contexts[port] = ctx;
    }
}

void MIN_ReInit(MIN_Context_t *ctx)
{
    min_init_context(&ctx->min_ctx, ctx->min_ctx.port);
    min_transport_reset(&ctx->min_ctx, true);
    ctx->last_poll_time = min_time_ms();
    ctx->timeout_triggered = false;
}

void MIN_RegisterTimeoutCallback(MIN_Context_t *ctx, void (*callback)(MIN_Context_t *ctx))
{
    ctx->timeout_callback = callback;
}

void MIN_App_Poll(MIN_Context_t *ctx, const uint8_t *rx_data, uint32_t rx_len) {
    min_poll(&ctx->min_ctx, rx_data, rx_len);
    uint32_t now = min_time_ms();
    if (ctx->min_ctx.transport_fifo.n_frames > 0) {
        struct transport_frame *oldest = &ctx->min_ctx.transport_fifo.frames[ctx->min_ctx.transport_fifo.head_idx];
        if ((now - oldest->last_sent_time_ms) > MIN_FRAME_TIMEOUT_MS) {
            if (!ctx->timeout_triggered) {
                if (ctx->timeout_callback) {
                    ctx->timeout_callback(ctx);
                }
                ctx->timeout_triggered = true;
#ifdef AUTO_REINIT_ON_TIMEOUT
                if (ctx->auto_reinit) {
                    MIN_ReInit(ctx);
                }
#endif
            }
        } else {
            ctx->timeout_triggered = false;
        }
    }
    ctx->last_poll_time = now;
}


void MIN_Timeout_Handler(MIN_Context_t *ctx) {
	min_debug_print("MIN-Timeout!\r\n");
}



// Khởi tạo bộ đệm vòng (item_size = 1 byte)
void min_shell_stdio_init(void)
{
	circular_char_buffer_init(&rx_buffer, rx_static_buffer, MIN_SHELL_UART_BUFFER_SIZE);
	circular_char_buffer_init(&tx_buffer, tx_static_buffer,  MIN_SHELL_UART_BUFFER_SIZE);
    // Khởi tạo UART_Stdio
    uart_stdio_init(&min_shell_stdio, MIN_SHELL_UART, &rx_buffer, &tx_buffer);

    // Kích hoạt UART
    uart_stdio_active(&min_shell_stdio);
}

static void min_shell_task_init(min_shell_task_t * const me, min_shell_evt_t const * const e) {
    DBC_ASSERT(1u, me != NULL);
    me->min_context = &min_ctx; // Assign the context to the task
    MIN_Context_Init(me->min_context, EXP_MIN_PORT);
    MIN_RegisterTimeoutCallback(&min_ctx, MIN_Timeout_Handler);
    SST_TimeEvt_arm(&me->min_poll_timer, MIN_SHELL_POLL_INTERVAL_MS, MIN_SHELL_POLL_INTERVAL_MS); // Re-arm the timer
}

static void min_shell_task_dispatch(min_shell_task_t * const me, min_shell_evt_t * const e) {
    DBC_ASSERT(4u, me != NULL);
    DBC_ASSERT(5u, e != NULL);

    min_shell_state_handler_t prev_state = me->state; /* save for later */
    state_t status = (me->state)(me, e);

    if (status == TRAN_STATUS) { /* transition taken? */
        (prev_state)(me, &exit_evt);
        (me->state)(me, &entry_evt);
    }
}

static void min_shell_task_ctor(min_shell_task_t * const me, min_shell_task_init_t * const init) {
    DBC_ASSERT(2u, me != NULL);
    SST_Task_ctor(&me->super, (SST_Handler) min_shell_task_init, (SST_Handler) min_shell_task_dispatch,
                  (SST_Evt *) init->current_evt, init->min_shell_event_buffer);
    SST_TimeEvt_ctor(&me->min_poll_timer, EVT_MIN_POLL, &(me->super)); // Initialize the polling timer
    me->state = init->init_state; // Set the initial state
    me->min_context = init->min_context; // Set the MIN context
    me->min_shell_uart = init->min_shell_uart; // Set the UART for MIN communication
    SST_TimeEvt_disarm(&me->min_poll_timer); // Disarm the polling timer
}

void min_shell_task_ctor_singleton(void)
{

    min_shell_stdio_init(); // Initialize the UART for MIN communication
    circular_buffer_init(&min_shell_task_event_queue,(uint8_t *)min_shell_task_event_buffer,sizeof(min_shell_task_event_buffer),MIN_SHELL_TASK_NUM_EVENTS,sizeof(min_shell_evt_t));
    min_shell_task_init_t min_shell_task_init =
    {
        .init_state = min_shell_state_process_handler, // Set the initial state to process handler
        .min_context = &min_ctx, // Pointer to the MIN context
        .min_shell_uart = &min_shell_stdio, // Pointer to the UART for MIN communication
        .current_evt = &min_shell_current_event, // Pointer to the current event being processed
        .min_shell_event_buffer = &min_shell_task_event_queue // Pointer to the circular buffer for events
    };
    min_shell_task_ctor(&min_shell_inst, &min_shell_task_init);
}
void min_shell_task_start(uint8_t priority)
{
    SST_Task_start(&min_shell_inst.super,priority);
 
}
static state_t min_shell_state_process_handler(min_shell_task_t * const me, min_shell_evt_t const * const e)
{
    DBC_ASSERT(3u, me != NULL);
    switch (e->super.sig)
    {
    	case SIG_ENTRY:
    		SST_TimeEvt_arm(&me->min_poll_timer, MIN_SHELL_POLL_INTERVAL_MS, MIN_SHELL_POLL_INTERVAL_MS);
    		return HANDLED_STATUS;
        case EVT_MIN_POLL:

        	min_shell_received_length = MIN_SHELL_RECEIVED_DATA_BUFFER_SIZE;
            if(uart_stdio_read(me->min_shell_uart, min_shell_received_data, &min_shell_received_length))
			{
            	MIN_App_Poll(me->min_context, min_shell_received_data, min_shell_received_length);
			}

            MIN_App_Poll(me->min_context, NULL, 0);
            return HANDLED_STATUS;

        default:
            return IGNORED_STATUS;
    }
}





////////////////////////////////// CALLBACKS ///////////////////////////////////

// Tell MIN how much space there is to write to the serial port. This is used
// inside MIN to decide whether to bother sending a frame or not.
uint16_t min_tx_space(uint8_t port)
{
  // Ignore 'port' because we have just one context. But in a bigger application
  // with multiple ports we could make an array indexed by port to select the serial
  // port we need to use.
  return circular_char_buffer_get_free_space(min_shell_inst.min_shell_uart->rx_buffer);
}

// Send a character on the designated port.
void min_tx_byte(uint8_t port, uint8_t byte)
{
  // Ignore 'port' because we have just one context.
	uart_stdio_write_char(min_shell_inst.min_shell_uart, byte);

}

// Tell MIN the current time in milliseconds.
uint32_t min_time_ms(void)
{
  return SST_getTick();
}

//callback function to handle incoming MIN frames

static MIN_ResponseHandler response_handler = NULL;

void MIN_RegisterResponseHandler(MIN_ResponseHandler handler)
{
    response_handler = handler;
}

void min_application_handler(uint8_t min_id, uint8_t const *min_payload, uint8_t len_payload, uint8_t port) {
	if (port >= MAX_MIN_CONTEXTS)
	{
		return;
	}
	MIN_Context_t *ctx = registered_contexts[port];
    const MIN_Command_t *command_table = MIN_GetCommandTable();
    int table_size = MIN_GetCommandTableSize();
    for (int i = 0; i < table_size; i++) {
        if (command_table[i].id == min_id) {
            command_table[i].handler(ctx, min_payload, len_payload);
            return;
        }
    }
}

void MIN_Send(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *payload, uint8_t len) {
    if (min_queue_has_space_for_frame(&ctx->min_ctx, len)) {
        min_queue_frame(&ctx->min_ctx, min_id, payload, len);
    }
}



void UART7_IRQHandler(void)
{
	uart_stdio_rx_callback(min_shell_inst.min_shell_uart);
	uart_stdio_tx_callback(min_shell_inst.min_shell_uart);
}

//void min_shell_rx_callback(void) // Callback xử lý ngắt nhận
//{
//    // Read data from the UART and process it
//    if (LL_USART_IsActiveFlag_RXNE(MIN_SHELL_UART)) {
//    	min_shell_received_data[0] = LL_USART_ReceiveData8(MIN_SHELL_UART);
//
//    	has_data_evt.data_len = 1;
//        SST_Task_post(&min_shell_inst.super, (SST_Evt *)&has_data_evt); // Post event to shell task
//    }
//}

