/*
 * experiment_task.h
 *
 *  Created on: Jun 25, 2025
 *      Author: Admin
 */

#ifndef APP_EXPERIMENT_EXPERIMENT_TASK_H_
#define APP_EXPERIMENT_EXPERIMENT_TASK_H_

#include "sst.h"
#include "fsm.h"

#define EXPERIMENT_COMMAND_PAYLOAD_LENGTH 16
typedef struct experiment_task_t experiment_task_t;
typedef struct experiment_evt_t experiment_evt_t;
typedef struct experiment_task_init_t experiment_task_init_t;
typedef struct experiment_profile_t experiment_profile_t;
typedef struct data_profile_t  data_profile_t;

typedef state_t (*experiment_task_handler_t)(experiment_task_t  * const me, experiment_evt_t const * const e);
enum {S_PRE_SAMPLING, S_DATA_SAMPLING, S_POST_SAMPLING, S_AQUI_ERROR,S_AQUI_TIMEOUT,NO_SUBSTATE};
enum {EXPERIMENT_TEMPERATURE_AQUI_START,EXPERIMENT_TEMPERATURE_AQUI_SEND_DATA};

struct experiment_evt_t{
	SST_Evt super;
	uint8_t payload[EXPERIMENT_COMMAND_PAYLOAD_LENGTH];
};

struct experiment_profile_t{
	uint16_t sampling_rate;
	uint8_t pos;
	uint8_t	laser_percent;
	uint32_t pre_time; //time before switching
	uint32_t experiment_time;//time when switch the laser on
	uint32_t post_time; //time after switching off the laser
	uint32_t num_sample;
	uint32_t period;
};
struct data_profile_t{
	uint32_t start_address;
	uint32_t num_data;
	uint8_t	destination; //0: send to UART console, 1: to SPI
};
struct experiment_task_t{
	SST_Task super;
	experiment_task_handler_t state;
	SST_TimeEvt timeout_timer;
	experiment_profile_t profile;
	data_profile_t data_profile;
	uint8_t	sub_state;
	uint8_t  laser_current[2]; //current for internal/external laser
	uint8_t  int_laser_pos; //0-36, 0xFF for switched OFF all
	uint8_t  ext_laser_pos; //0-8, 0xFF for switched OFF all
	uint8_t	 laser_spi_mode; // SPI mode is 0, 1, 2, 3
	uint8_t  photo_pos;		//0-36, 0xFF for switched OFF all
	uint16_t photo_value;	//photo adc 16bit value
	enum {ADC_MODE, SW_MODE} photodiode_mode;
};

struct experiment_task_init_t {
	experiment_task_handler_t init_state;
	experiment_evt_t * current_evt;
	circular_buffer_t * event_buffer;
	uint8_t	sub_state;
};
void experiment_task_singleton_ctor(void);
void experiment_task_start(uint8_t priority);
uint32_t experiment_task_laser_set_current(experiment_task_t * const me, uint32_t laser_id, uint32_t percent);
uint32_t experiment_task_laser_get_current(experiment_task_t * const me, uint32_t laser_id);
uint32_t experiment_task_int_laser_switchon(experiment_task_t * const me, uint32_t laser_id);
uint32_t experiment_task_ext_laser_switchon(experiment_task_t * const me, uint32_t laser_id);
uint32_t experiment_task_photodiode_switchon(experiment_task_t * const me, uint32_t photo_id);
uint32_t experiment_task_photodiode_switchoff(experiment_task_t * const me);
uint32_t experiment_task_ext_laser_switchoff(experiment_task_t * const me);
uint32_t experiment_task_int_laser_switchoff(experiment_task_t * const me);

uint32_t experiment_task_set_profile(experiment_task_t * me,experiment_profile_t * profile);
uint32_t experiment_task_get_ram_data(experiment_task_t * const me, uint32_t start_addr, uint32_t num_data);
void experiment_task_get_profile(experiment_task_t * me, experiment_profile_t * profile);
uint32_t experiment_start_measuring(experiment_task_t * const me);
uint32_t experiment_task_photo_ADC_prepare_SPI(experiment_task_t * const me);

#endif /* APP_EXPERIMENT_EXPERIMENT_TASK_H_ */
