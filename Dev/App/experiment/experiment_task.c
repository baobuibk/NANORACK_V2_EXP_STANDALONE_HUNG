/*
 * experiment_task.c
 *
 *  Created on: Jun 25, 2025
 *      Author: Admin
 */
#include "experiment_task.h"
#include "bsp_photodiode.h"
#include "bsp_laser.h"
#include "bsp_spi_ram.h"
#include "dbc_assert.h"
#include "app_signals.h"
#include "error_codes.h"
#include "uart_dbg.h"
#include "stddef.h"
#include "string.h"
#include "shell.h"

DBC_MODULE_NAME("experiment_task")

#define EXPERIMENT_TASK_NUM_EVENTS  	2
#define EXPERIMENT_TASK_AQUI_TIMEOUT	20000
#define EXPERIMENT_CHUNK_SIZE			(4*1024)
experiment_task_t experiment_task_inst;
extern shell_task_t shell_task_inst ;
static shell_task_t *pshell_task_instt = &shell_task_inst;

static experiment_evt_t const entry_evt = {.super = {.sig = SIG_ENTRY} };
static experiment_evt_t const exit_evt = {.super = {.sig = SIG_EXIT} };
static experiment_evt_t const start_measuring_evt = {.super = {.sig = EVT_EXPERIMENT_START_MEASURE} };
static experiment_evt_t const start_sending_evt = {.super = {.sig = EVT_EXPERIMENT_START_SENDING} };

static shell_evt_t const uart_send_buffer_bin_evt = {.super = {.sig = EVT_SHELL_SEND_BUFFER_BINARY},};
static shell_evt_t const uart_send_crc_evt = {.super = {.sig = EVT_SHELL_SEND_CRC},};


static data_profile_t remain_data_profile;
static uint16_t ram_buffer[EXPERIMENT_CHUNK_SIZE];
static uint16_t batch_size;

experiment_evt_t experiment_task_current_event = {0}; // Current event being processed
experiment_evt_t experiment_task_event_buffer[EXPERIMENT_TASK_NUM_EVENTS];

circular_buffer_t experiment_task_event_queue = {0}; // Circular buffer to hold shell events

static state_t experiment_task_state_manual_handler(experiment_task_t * const me, experiment_evt_t const * const e);
static state_t experiment_task_state_data_aqui_handler(experiment_task_t * const me, experiment_evt_t const * const e);
static state_t experiment_task_state_data_send_handler(experiment_task_t * const me, experiment_evt_t const * const e);

static void experiment_task_init(experiment_task_t * const me,experiment_evt_t const * const e)
{
	//DBG(DBG_LEVEL_INFO,"experiment_task init\r\n");
	bsp_laser_init();
	bsp_spi_ram_init();
	bsp_photodiode_init();
	me->laser_spi_mode = 1;
	me->photodiode_mode = ADC_MODE;
}
static void experiment_task_dispatch(experiment_task_t * const me,experiment_evt_t const * const e)
{
    experiment_task_handler_t prev_state = me->state; /* save for later */
    state_t status = (me->state)(me, e);

    if (status == TRAN_STATUS) { /* transition taken? */
        (prev_state)(me, &exit_evt);
        (me->state)(me, &entry_evt);
    }
}
void experiment_task_ctor(experiment_task_t * const me, experiment_task_init_t const * const init) {
    DBC_ASSERT(0u, me != NULL);
    SST_Task_ctor(&me->super, (SST_Handler) experiment_task_init, (SST_Handler)experiment_task_dispatch, \
                                (SST_Evt *)init->current_evt, init->event_buffer);
    me->state = init->init_state;
    SST_TimeEvt_ctor(&me->timeout_timer, EVT_EXPERIMENT_DATA_AQUISITION_TIMEOUT, &(me->super));
    SST_TimeEvt_disarm(&me->timeout_timer); // Disarm the timeout timer
    me->sub_state = init->sub_state;
}
void experiment_task_singleton_ctor(void)
{
	circular_buffer_init(&experiment_task_event_queue, (uint8_t * )&experiment_task_event_buffer, sizeof(experiment_task_event_buffer), EXPERIMENT_TASK_NUM_EVENTS, sizeof(experiment_evt_t));
	experiment_task_init_t init = {
			.init_state = experiment_task_state_manual_handler,
			.current_evt = &experiment_task_current_event,
			.event_buffer = &experiment_task_event_queue,
			.sub_state = NO_SUBSTATE
	};
	experiment_task_ctor(&experiment_task_inst,&init);
}
void experiment_task_start(uint8_t priority)
{
	SST_Task_start(&experiment_task_inst.super,priority);
}

//only handle command from shell
static state_t experiment_task_state_manual_handler(experiment_task_t * const me, experiment_evt_t const * const e)
{
	switch (e->super.sig)
	{
		case SIG_ENTRY:
		{
			//DBG(DBG_LEVEL_INFO,"entry experiment_task_state_manual_handler\r\n");
			SST_TimeEvt_disarm(&me->timeout_timer); //disable the timeout
			return HANDLED_STATUS;
		}

		case EVT_EXPERIMENT_START_MEASURE:
			{

				me->state = experiment_task_state_data_aqui_handler;
				return TRAN_STATUS;
			}
		case EVT_EXPERIMENT_START_SENDING:
			{
				//memcpy((void *)&me->data_profile, e->payload, sizeof(data_profile_t));

				me->state = experiment_task_state_data_send_handler;
				return TRAN_STATUS;
			}
			default:
				return IGNORED_STATUS;
		}
	}



static state_t experiment_task_state_data_aqui_handler(experiment_task_t * const me, experiment_evt_t const * const e)
{
	switch (e->super.sig)
	{
		case SIG_ENTRY:
		{
			DBG(DBG_LEVEL_INFO,"Start Sampling...\r\n");
			//DBG(DBG_LEVEL_INFO,"entry experiment_task_state_data_aqui_handler\r\n");
			SST_TimeEvt_arm(&me->timeout_timer, EXPERIMENT_TASK_AQUI_TIMEOUT, 0);
	//      Switch the photodiode on
			experiment_task_photodiode_switchon(me, me->profile.pos);
			//DBG(DBG_LEVEL_INFO,"switch on photo %d\r\n", me->profile.pos);
	//		Switch the SPI to ADC supported mode
			experiment_task_photo_ADC_prepare_SPI(me);
			//DBG(DBG_LEVEL_INFO,"switch on photo %d and change SPI mode to SPI ADC Mode\r\n", me->profile.pos);
	//		Prepare the timer for sampling
			bsp_photodiode_time_t init_photo_time;
			init_photo_time.pre_time = me->profile.pre_time ;
			init_photo_time.sampling_time = me->profile.experiment_time ;
			init_photo_time.post_time = me->profile.post_time ;
			init_photo_time.sampling_rate = me->profile.sampling_rate;
			init_photo_time.pos = me->profile.pos;
			bsp_laser_int_set_current(me->profile.laser_percent);
			bsp_photo_set_time(& init_photo_time);
			bsp_photodiode_sample_start();
			me->sub_state = S_PRE_SAMPLING;
			return HANDLED_STATUS;
		}
		case SIG_EXIT:
		{
			//DBG(DBG_LEVEL_INFO,"exit experiment_task_state_data_aqui_handler\r\n");
			SST_TimeEvt_disarm(&me->timeout_timer);
			return HANDLED_STATUS;
		}
		case EVT_EXPERIMENT_FINISH_PRE_SAMPLING:
		{
			//DBG(DBG_LEVEL_INFO,"EXPERIMENT_FINISH_PRE_SAMPLING\r\n");
			if (me->sub_state == S_PRE_SAMPLING)
			{
				me->sub_state = S_DATA_SAMPLING;
			}
			else me->sub_state = S_AQUI_ERROR;

			return HANDLED_STATUS;
		}
		case EVT_EXPERIMENT_FINISH_SAMPLING:
		{
			//DBG(DBG_LEVEL_INFO,"EXPERIMENT_FINISH_SAMPLING\r\n");
			if (me->sub_state == S_DATA_SAMPLING)
			{
				me->sub_state = S_POST_SAMPLING;
			}
			else me->sub_state = S_AQUI_ERROR;
			return HANDLED_STATUS;
		}
		case EVT_EXPERIMENT_FINISH_POST_SAMPLING:
		{
			if (me->sub_state == S_POST_SAMPLING)
			{
				me->sub_state = NO_SUBSTATE;
			}
			else me->sub_state = S_AQUI_ERROR;
			DBG(DBG_LEVEL_INFO,"Sampling Done!\r\n");
			me->state = experiment_task_state_manual_handler;
			return TRAN_STATUS;
		}
		case EVT_EXPERIMENT_DATA_AQUISITION_TIMEOUT:
		{
			me->sub_state = S_AQUI_TIMEOUT;
			me->state = experiment_task_state_manual_handler;
			return TRAN_STATUS;
		}
		default:
			return IGNORED_STATUS;
	}
 }
static state_t experiment_task_state_data_send_handler(experiment_task_t * const me, experiment_evt_t const * const e)
{
	switch (e->super.sig)
	{
		case SIG_ENTRY:
		{
			//DBG(DBG_LEVEL_INFO,"entry experiment_task_state_data_send_handler\r\n");
			SST_TimeEvt_disarm(&me->timeout_timer); //disable the timeout
		    remain_data_profile.num_data = me->data_profile.num_data;
		    remain_data_profile.start_address = me->data_profile.start_address;
		    SST_Task_post((SST_Task *)&shell_task_inst.super, (SST_Evt *)&uart_send_buffer_bin_evt);
			return HANDLED_STATUS;
		}

		case EVT_EXPERIMENT_DONE_SEND_HEADER:
		{
		    batch_size = (remain_data_profile.num_data > EXPERIMENT_CHUNK_SIZE) ? EXPERIMENT_CHUNK_SIZE : remain_data_profile.num_data;
			bsp_spi_ram_read_dma(remain_data_profile.start_address * 2, batch_size * 2, (uint8_t *)ram_buffer);
			return HANDLED_STATUS;
		}

		case EVT_EXPERIMENT_DONE_READ_RAM:
		{
			shell_send_buffer(pshell_task_instt, ram_buffer, batch_size, me->data_profile.mode);
			return HANDLED_STATUS;
		}

		case EVT_EXPERIMENT_DONE_SEND_CHUNK:
		{
			remain_data_profile.num_data -= batch_size;
			remain_data_profile.start_address += batch_size;
			if(remain_data_profile.num_data > 0)
			{
				batch_size = (remain_data_profile.num_data > EXPERIMENT_CHUNK_SIZE) ? EXPERIMENT_CHUNK_SIZE : remain_data_profile.num_data;
				bsp_spi_ram_read_dma(remain_data_profile.start_address * 2, batch_size * 2, (uint8_t *)ram_buffer);
				return HANDLED_STATUS;
			}
			else
			{
				SST_Task_post((SST_Task *)&shell_task_inst.super, (SST_Evt *)&uart_send_crc_evt);
				me->state = experiment_task_state_manual_handler;
				return TRAN_STATUS;
			}
		}

		default:
			return IGNORED_STATUS;
	}
}


uint32_t experiment_task_get_ram_data(experiment_task_t * const me, uint32_t start_addr, uint32_t num_data, uint8_t mode)	// mode0: send ASCII, mode1: send binary
{
	me->data_profile.total_data = num_data;
	me->data_profile.start_address = start_addr;
	me->data_profile.num_data = num_data;
	me->data_profile.mode = mode;
	pshell_task_instt->crc = 0xffff;
	SST_Task_post(&me->super, (SST_Evt *)&start_sending_evt);
	return ERROR_OK;
}

uint32_t experiment_task_data_transfer(experiment_task_t * const me)
{

	me->data_profile.total_data = me->num_data_real;
	me->data_profile.start_address = 0;
	me->data_profile.num_data = me->num_data_real;
	me->data_profile.mode = 1;
	pshell_task_instt->crc = 0xffff;
	SST_Task_post(&me->super, (SST_Evt *)&start_sending_evt);
	return ERROR_OK;
}

uint32_t experiment_task_laser_set_current(experiment_task_t * const me, uint32_t laser_id, uint32_t percent)
{
	if ((laser_id > 1) || (percent > 100)) return ERROR_NOT_SUPPORTED;
	if(me->laser_spi_mode != 0)
	{
		bsp_laser_set_spi_mode(SPI_MODE_0);
		me->laser_spi_mode = 0;
	}
	bsp_laser_set_current(laser_id, percent);
	me->laser_current[laser_id] = percent;
	return ERROR_OK;
}

uint32_t experiment_task_laser_get_current(experiment_task_t * const me, uint32_t laser_id)
{
	uint32_t index;
	if (laser_id > 0) index = 1; else index = 0;
	return me->laser_current[index];
}
uint32_t experiment_task_int_laser_switchon(experiment_task_t * const me, uint32_t laser_id)
{
	if (laser_id > INTERNAL_CHAIN_CHANNEL_NUM) return ERROR_NOT_SUPPORTED;
	if(me->laser_spi_mode != 1)
	{
		bsp_laser_set_spi_mode(SPI_MODE_1);
		me->laser_spi_mode = 1;
	}
	bsp_laser_int_switch_on(laser_id);
	me->int_laser_pos = laser_id;
	return ERROR_OK;
}
uint32_t experiment_task_int_laser_switchoff(experiment_task_t * const me)
{
	if(me->laser_spi_mode != 1)
	{
		bsp_laser_set_spi_mode(SPI_MODE_1);
		me->laser_spi_mode = 1;
	}
	bsp_laser_int_switch_off_all();
	me->int_laser_pos = 0xFF;
	return ERROR_OK;
}
uint32_t experiment_task_ext_laser_switchon(experiment_task_t * const me, uint32_t laser_id)
{
	if (laser_id > EXTERNAL_CHAIN_CHANNEL_NUM) return ERROR_NOT_SUPPORTED;
	if(me->laser_spi_mode != 1)
	{
		bsp_laser_set_spi_mode(SPI_MODE_1);
		me->laser_spi_mode = 1;
	}
	bsp_laser_ext_switch_on(laser_id);
	me->ext_laser_pos = laser_id;
	return ERROR_OK;
}
uint32_t experiment_task_ext_laser_switchoff(experiment_task_t * const me)
{
	if(me->laser_spi_mode != 1)
	{
		bsp_laser_set_spi_mode(SPI_MODE_1);
		me->laser_spi_mode = 1;
	}
	bsp_laser_ext_switch_off_all();
	me->ext_laser_pos = 0xFF;
	return ERROR_OK;
}

uint32_t experiment_task_photodiode_switchon(experiment_task_t * const me, uint32_t photo_id)
{
	if (photo_id > INTERNAL_CHAIN_CHANNEL_NUM) return ERROR_NOT_SUPPORTED;
//	if(me->photodiode_mode != SW_MODE)
//	{
		bsp_photodiode_sw_spi_change_mode();
//		me->photodiode_mode = SW_MODE;
//	}
	bsp_photo_switch_on(photo_id);
	me->photo_pos = photo_id;
	return ERROR_OK;
}
uint32_t experiment_task_photodiode_switchoff(experiment_task_t * const me)
{
//	if(me->photodiode_mode != SW_MODE)
//	{
		bsp_photodiode_sw_spi_change_mode();
//		me->photodiode_mode = SW_MODE;
//	}
	bsp_photo_switch_off_all();
	me->photo_pos = 0xFF; //photo is OFF
	return ERROR_OK;
}

uint32_t experiment_task_photo_ADC_prepare_SPI(experiment_task_t * const me)
{
//	if(me->photodiode_mode != ADC_MODE)
//	{
		bsp_photodiode_adc_spi_change_mode();
//		me->photodiode_mode = ADC_MODE;
//	}
	return ERROR_OK;
}

uint32_t experiment_task_set_profile(experiment_task_t * me,experiment_profile_t * profile)
{
	if ((profile->sampling_rate == 0 ) || (profile->sampling_rate > 1000)) return ERROR_NOT_SUPPORTED;
	if ((profile->pos == 0 ) || (profile->pos > 36)) return ERROR_NOT_SUPPORTED;
	if ((profile->laser_percent > 100 ) ) return ERROR_NOT_SUPPORTED;
	if (profile->num_sample > 2048) return ERROR_NOT_SUPPORTED;
	if (profile->period == 0) return ERROR_NOT_SUPPORTED;
	me->profile = *profile;
	return ERROR_OK;

}
void experiment_task_get_profile(experiment_task_t * me, experiment_profile_t * profile)
{
	*profile = me->profile;
}

uint32_t experiment_start_measuring(experiment_task_t * const me)
{
	experiment_profile_t * profile = &me->profile;
	if ((profile->sampling_rate ==0 ) || (profile->sampling_rate > 1000)) return ERROR_NOT_SUPPORTED;
	if ((profile->pos ==0 ) || (profile->pos > 36)) return ERROR_NOT_SUPPORTED;
	if ((profile->laser_percent > 100 ) ) return ERROR_NOT_SUPPORTED;
	if (profile->num_sample > 2048) return ERROR_NOT_SUPPORTED;
	//if (((profile->pre_time + profile->experiment_time + profile->post_time ) * profile->sampling_rate) > 2048*1000000) return ERROR_NOT_SUPPORTED;
	SST_Task_post(&me->super, (SST_Evt *)&start_measuring_evt);
	return ERROR_OK;
}


