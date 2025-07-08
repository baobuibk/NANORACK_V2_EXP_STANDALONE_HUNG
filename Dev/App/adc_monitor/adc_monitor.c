/*
 * temperature_monitor.c
 *
 *  Created on: Jun 21, 2025
 *      Author: Admin
 */

#include "adc_monitor.h"
#include "bsp_ntc.h"
#include "bsp_laser.h"
#include "dbc_assert.h"
#include "app_signals.h"
#include "error_codes.h"
#include "uart_dbg.h"
#include "stddef.h"
#include "ntc.h"

DBC_MODULE_NAME("adc_monitor")

monitor_task_t monitor_task_inst ;
#define MONITOR_TASK_NUM_EVENTS 4
monitor_evt_t current_monitor_e = {0}; // Current event being processed
monitor_evt_t monitor_e_buffer[MONITOR_TASK_NUM_EVENTS] = {0}; // Array to hold shell events
circular_buffer_t monitor_e_queue = {0}; // Circular buffer to hold shell events


static state_t monitor_state_process_handler(monitor_task_t * const me, monitor_evt_t const * const e);


static void monitor_task_init(monitor_task_t * const me, monitor_evt_t const * const e)
{
	bsp_ntc_adc_init();
	bsp_laser_adc_init();
	SST_TimeEvt_arm(&me->monitor_task_timer, 10, 10); //trigger every 10 tick
}


void monitor_task_ctor(monitor_task_t * const me, monitor_init_t const * const init) {
    DBC_ASSERT(0u, me != NULL);
    SST_Task_ctor(&me->super, (SST_Handler) monitor_task_init, (SST_Handler)monitor_state_process_handler, \
                                (SST_Evt *)init->current_evt, init->event_buffer);
    SST_TimeEvt_ctor(&me->monitor_task_timer, EVT_MONITOR_NTC_TRIGGER_TIME, &(me->super));

    me->state = init->init_state;

}

void monitor_task_ctor_singleton()
{
 circular_buffer_init(&monitor_e_queue,(uint8_t *)&monitor_e_buffer,sizeof(monitor_e_buffer),MONITOR_TASK_NUM_EVENTS,sizeof(monitor_evt_t));
 monitor_init_t init = {
		 .init_state = monitor_state_process_handler,
		 .event_buffer = &monitor_e_queue,
		 .current_evt = &current_monitor_e
 };
 monitor_task_ctor(&monitor_task_inst, &init);
}

void monitor_task_start(uint8_t priority)
{
	SST_Task_start(&monitor_task_inst.super,priority);
}
static state_t monitor_state_process_handler(monitor_task_t * const me, monitor_evt_t const * const e)
{
	switch (e->super.sig)
	{
		case EVT_MONITOR_NTC_TRIGGER_TIME:
		{
			bsp_ntc_trigger_adc();
			bsp_laser_trigger_adc();
			return HANDLED_STATUS;
		}

		case EVT_MONITOR_NTC_ADC_COMPLETED:
		{
			for (uint8_t i = 0; i < 8; i++ )
			{
				me->adc_data.ntc_temperature[i] = bsp_ntc_get_temperature(i);
			}
			return HANDLED_STATUS;
		}

		case EVT_MONITOR_LD_ADC_COMPLETED:
		{
			for (uint8_t i = 0; i < 2; i++ )
			{
				me->adc_data.laser_current[i] = bsp_laser_get_current(i);
			}
		}
		default:
			return IGNORED_STATUS;
	}
}

int16_t laser_monitor_get_laser_current(uint32_t channel)
{
	return monitor_task_inst.adc_data.laser_current[channel];
}

int16_t temperature_monitor_get_ntc_temperature(uint32_t channel)
{
	return monitor_task_inst.adc_data.ntc_temperature[channel];
}
