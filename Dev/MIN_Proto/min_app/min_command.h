/*
 * min_command.h
 *
 *  Created on: May 5, 2025
 *      Author: HTSANG
 */

#ifndef MIN_PROTO_MIN_APP_MIN_COMMAND_H_
#define MIN_PROTO_MIN_APP_MIN_COMMAND_H_

#include <stdint.h>
#include "min_app.h"

// =================================================================
// Min IDs
// =================================================================
#define MIN_SET		0x20
#define MIN_GET		0x21

// =================================================================
// Status data
// =================================================================
#define ON			0x01
#define OFF			0x00

// =================================================================
// Command IDs (Maximum ID: 0xFF)
// =================================================================
#define NTC_TEMP_GET_CMD             0x01
#define PWR_5V_SET_CMD               0x02
#define PWR_5V_GET_CMD               0x03
#define TEC_INIT_CMD                 0x04
#define TEC_STATUS_GET_CMD           0x05
#define TEC_VOLT_SET_CMD             0x06
#define TEC_VOLT_GET_CMD             0x07
#define TEC_DIR_SET_CMD              0x08
#define TEC_DIR_GET_CMD              0x09
#define HTR_DUTY_SET_CMD             0x0A
#define HTR_DUTY_GET_CMD             0x0B
#define REF_TEMP_SET_CMD             0x0C
#define REF_TEMP_GET_CMD             0x0D
#define REF_NTC_SET_CMD              0x0E
#define REF_NTC_GET_CMD              0x0F
#define AUTO_TEC_SET_CMD             0x10
#define AUTO_TEC_GET_CMD             0x11
#define AUTO_HTR_SET_CMD             0x12
#define AUTO_HTR_GET_CMD             0x13
#define AUTO_TEMP_SET_CMD            0x14
#define AUTO_TEMP_GET_CMD            0x15
#define LSM_SENS_GET_CMD             0x16
#define H3L_SENS_GET_CMD             0x17
#define BME_SENS_GET_CMD             0x18
#define LASER_INT_IND_SET_CMD        0x19
#define LASER_INT_IND_GET_CMD        0x1A
#define LASER_INT_DAC_SET_CMD        0x1B
#define LASER_INT_DAC_GET_CMD        0x1C
#define LASER_INT_AUTO_SET_CMD       0x1D


#define OVER				  0x3B
#define ACK					  0x3C
#define WRONG				  0x3D
#define FAIL				  0x3E
#define	DONE				  0x3F

#define MIN_ACK					0x06
#define MIN_NAK					0x15

/**
 * @brief Command handler function type.
 * @param ctx Pointer to the MIN context.
 * @param payload Pointer to the received payload data.
 * @param len Length of the payload.
 */
typedef void (*MIN_CommandHandler)(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload);

/**
 * @brief Structure to map command IDs to their handlers.
 */
typedef struct MIN_Command {
    uint8_t CMD;                   ///< Command ID
    MIN_CommandHandler handler;    ///< Handler function for the command
} MIN_Command_t;

/**
 * @brief Gets the command table.
 * @return Pointer to the command table.
 */
const MIN_Command_t *MIN_GetCommandTable(void);

/**
 * @brief Gets the size of the command table.
 * @return Number of entries in the command table.
 */
int MIN_GetCommandTableSize(void);

#endif /* MIN_PROTO_MIN_APP_MIN_COMMAND_H_ */
