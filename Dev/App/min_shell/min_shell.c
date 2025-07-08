/*
 * min_shell.c
 *
 *  Created on: Jun 13, 2025
 *      Author: Admin
 */
#include "min_shell.h"
#include "min_shell_command.h"

#include "stddef.h"
#include "app_signals.h"
#include "configs.h"
#include "DBC_assert.h"
#include "board.h"

DBC_MODULE_NAME("min_shell")

#define MIN_SHELL_POLL_INTERVAL_MS 10 // Polling interval in milliseconds
#define MIN_SHELL_TASK_NUM_EVENTS 30
#define MIN_SHELL_RECEIVED_DATA_BUFFER_SIZE 256
min_shell_task_t min_shell_inst;

static struct min_context min_ctx;

min_shell_evt_t min_shell_current_event = {0}; // Current event being processed
min_shell_evt_t min_shell_task_event_buffer[MIN_SHELL_TASK_NUM_EVENTS] = {0}; // Array to hold shell events
circular_buffer_t min_shell_task_event_queue = {0}; // Circular buffer to hold shell events
static state_t min_shell_state_process_handler(min_shell_task_t * const me, min_shell_evt_t const * const e);
static void min_shell_task_dispatch(min_shell_task_t * const me, min_shell_evt_t * const e) ;

 static uint8_t          min_shell_received_data[MIN_SHELL_RECEIVED_DATA_BUFFER_SIZE];
 static min_shell_evt_t const entry_evt = {.super = {.sig = SIG_ENTRY} };
 static min_shell_evt_t const exit_evt = {.super = {.sig = SIG_EXIT} };
 static min_shell_evt_t has_data_evt = {.super = {.sig = EVT_MIN_SHELL_HAS_DATA},
                                            .data_buff = min_shell_received_data,
                                            .data_len = sizeof(min_shell_received_data)};

#define MIN_SHELL_UART_BUFFER_SIZE 256


static circular_char_buffer_t rx_buffer;
static circular_char_buffer_t tx_buffer;
static uint8_t rx_static_buffer[MIN_SHELL_UART_BUFFER_SIZE];
static uint8_t tx_static_buffer[MIN_SHELL_UART_BUFFER_SIZE];
UART_stdio_t min_shell_uart; // UART for MIN communication

static void min_init() {

    // Initialize the MIN context for the shell task
    min_init_context(&min_ctx, 0);
    
}

// Khởi tạo bộ đệm vòng (item_size = 1 byte)
void min_shell_stdio_init(void)
{
	circular_char_buffer_init(&rx_buffer, rx_static_buffer, MIN_SHELL_UART_BUFFER_SIZE);
	circular_char_buffer_init(&tx_buffer, tx_static_buffer,  MIN_SHELL_UART_BUFFER_SIZE);
    // Khởi tạo UART_Stdio
    uart_stdio_init(&min_shell_uart, MIN_SHELL_UART, &rx_buffer, &tx_buffer);
    // Kích hoạt UART
    uart_stdio_active(&min_shell_uart);
}
static void min_shell_task_init(min_shell_task_t * const me, min_shell_evt_t const * const e) {
    DBC_ASSERT(1u, me != NULL);
    // Initialize the MIN shell task
    me->min_context = &min_ctx; // Assign the context to the task
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
void min_shell_task_ctor_singleton() {
    // Initialize the MIN shell task
    min_init();
    min_shell_stdio_init(); // Initialize the UART for MIN communication
    circular_buffer_init(&min_shell_task_event_queue,(uint8_t *)min_shell_task_event_buffer,sizeof(min_shell_task_event_buffer),MIN_SHELL_TASK_NUM_EVENTS,sizeof(min_shell_evt_t));

    min_shell_task_init_t min_shell_task_init = {
        .init_state = min_shell_state_process_handler, // Set the initial state to process handler
        .min_context = &min_ctx, // Pointer to the MIN context
        .min_shell_uart = &min_shell_uart, // Pointer to the UART for MIN communication
        .current_evt = &min_shell_current_event, // Pointer to the current event being processed
        .min_shell_event_buffer = &min_shell_task_event_queue // Pointer to the circular buffer for events
    };
    min_shell_task_ctor(&min_shell_inst, &min_shell_task_init);
}
void min_shell_task_start(void) {
    // Start the MIN shell task
    SST_Task_start(&min_shell_inst.super,2);
 
}
static state_t min_shell_state_process_handler(min_shell_task_t * const me, min_shell_evt_t const * const e)
{
    DBC_ASSERT(3u, me != NULL);
    switch (e->super.sig) {
        case EVT_MIN_POLL: {
            // Handle periodic polling event
            min_poll(me->min_context,NULL,0);
            SST_TimeEvt_arm(&me->min_poll_timer, MIN_SHELL_POLL_INTERVAL_MS, 0); // Re-arm the timer
            return HANDLED_STATUS;
        }

        case EVT_MIN_SHELL_HAS_DATA: {
            // Handle incoming data from MIN
            min_poll(me->min_context,e->data_buff,e->data_len);
            SST_TimeEvt_arm(&me->min_poll_timer, MIN_SHELL_POLL_INTERVAL_MS, 0); // Re-arm the timer
            return HANDLED_STATUS;

        }

        default: {
            // Handle other events if necessary
            return IGNORED_STATUS;
        }
    }
    return HANDLED_STATUS;
}

////////////////////////////////// CALLBACKS ///////////////////////////////////

// Tell MIN how much space there is to write to the serial port. This is used
// inside MIN to decide whether to bother sending a frame or not.
uint16_t min_tx_space(uint8_t port)
{
  // Ignore 'port' because we have just one context. But in a bigger application
  // with multiple ports we could make an array indexed by port to select the serial
  // port we need to use.
  return circular_char_buffer_get_free_space(min_shell_uart.tx_buffer);
}

// Send a character on the designated port.
void min_tx_byte(uint8_t port, uint8_t byte)
{
  // Ignore 'port' because we have just one context.
	uart_stdio_write_char(&min_shell_uart, byte);
}

// Tell MIN the current time in milliseconds.
uint32_t min_time_ms(void)
{
  return SST_getTick();
}

//callback function to handle incoming MIN frames
void min_application_handler(uint8_t min_id, uint8_t const *min_payload, uint8_t len_payload, uint8_t port) {
    if (min_id == 0x01 && len_payload >= 1) {
        uint8_t response[255];
        uint8_t resp_len = 0;
        cmd_process(min_payload, len_payload, response, &resp_len);
//        min_queue_frame(min_shell_inst.min_context, 0x02, response, resp_len);
    }
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
// Callback xử lý ngắt nhận

void min_shell_rx_callback(void) {
    // Read data from the UART and process it
    if (LL_USART_IsActiveFlag_RXNE(MIN_SHELL_UART)) {
    	min_shell_received_data[0] = LL_USART_ReceiveData8(MIN_SHELL_UART);

    	has_data_evt.data_len = 1;
        SST_Task_post(&min_shell_inst.super, (SST_Evt *)&has_data_evt); // Post event to shell task
    }
}
//void USART2_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART2_IRQn 0 */
////	CLI_UART_stdio_rx_callback();
////	uart_stdio_tx_callback(&uart_stdio);
//	min_shell_rx_callback();
//	uart_stdio_tx_callback(&min_shell_uart);
//
//  /* USER CODE END USART2_IRQn 0 */
//  /* USER CODE BEGIN USART2_IRQn 1 */
//
//  /* USER CODE END USART2_IRQn 1 */
//}
