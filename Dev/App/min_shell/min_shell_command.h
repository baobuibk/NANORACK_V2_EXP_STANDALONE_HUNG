/*
 * min_shell_command.h
 *
 *  Created on: Jun 13, 2025
 *      Author: Admin
 */

#ifndef APP_MIN_SHELL_MIN_SHELL_COMMAND_H_
#define APP_MIN_SHELL_MIN_SHELL_COMMAND_H_


#include "min_shell.h"

#include <stdint.h>


// =================================================================
// Command IDs (Maximum ID: 63)
// =================================================================
#define READ_TEMP_CMD         0x99  ///< Request temperature reading
#define TEMP_RESPONSE         0x99  ///< Response with temperature data
#define CONTROL_TEMP_CMD      0x03  ///< Control temperature setting
#define CONTROL_TEMP_ACK      0x04  ///< Acknowledgment for control command
#define HEARTBEAT_CMD         0x05  ///< Heartbeat request
#define HEARTBEAT_ACK         0x06  ///< Heartbeat acknowledgment
#define GET_STATUS_CMD        0x07  ///< Request system status
#define STATUS_RESPONSE       0x08  ///< Response with status data
#define RESET_CMD             0x09  ///< Reset system

#define DUMMY_CMD_1           0x0C  ///< Dummy command 1
#define DUMMY_CMD_2           0x0D  ///< Dummy command 2
#define CUSTOM_CMD_1          0x0E  ///< Custom command 1
#define CUSTOM_CMD_1_ACK      0x0F  ///< Acknowledgment for custom command 1
#define CUSTOM_CMD_2          0x10  ///< Custom command 2
#define CUSTOM_CMD_2_ACK      0x11  ///< Acknowledgment for custom command 2

#define COLLECT_DATA_CMD	  0x12
#define PRE_CHUNK_CMD		  0x13
#define PRE_CHUNK_ACK		  0x14
#define PRE_DATA_CMD		  0x15
#define PRE_DATA_ACK		  0x16
#define SAMPLERATE_SET_CMD 	  0x17
#define SAMPLERATE_GET_CMD    0x18
#define SAMPLERATE_GET_ACK	  0x19
#define COLLECT_PACKAGE_CMD   0x1A
#define COLLECT_PACKAGE_ACK   0x1B

#define OVER				  0x3B
#define GOT_IT				  0x3C
#define WRONG				  0x3D
#define FAIL				  0x3E
#define	DONE				  0x3F
//----------------------------------------------------------------
#define TEST_CONNECTION_CMD                         0x01
#define TEST_CONNECTION_ACK                         0x02

#define SET_TEMP_PROFILE_CMD                        0x03
#define SET_TEMP_PROFILE_ACK                        0x04

#define START_TEMP_PROFILE_CMD                      0x05
#define START_TEMP_PROFILE_ACK                      0x06

#define STOP_TEMP_PROFILE_CMD                       0x07
#define STOP_TEMP_PROFILE_ACK                       0x08

#define SET_OVERRIDE_TEC_PROFILE_CMD                0x09
#define SET_OVERRIDE_TEC_PROFILE_ACK                0x0A

#define START_OVERRIDE_TEC_PROFILE_CMD              0x0B
#define START_OVERRIDE_TEC_PROFILE_ACK              0x0C

#define STOP_OVERRIDE_TEC_PROFILE_CMD               0x0D
#define STOP_OVERRIDE_TEC_PROFILE_ACK               0x0E

#define SET_SAMPLING_PROFILE_CMD                    0x11
#define SET_SAMPLING_PROFILE_ACK                    0x12

#define SET_LASER_INTENSITY_CMD                     0x13
#define SET_LASER_INTENSITY_ACK                     0x14

#define SET_POSITION_CMD                            0x15
#define SET_POSITION_ACK                            0x16

#define START_SAMPLING_CYCLE_CMD                    0x17
#define START_SAMPLING_CYCLE_ACK                    0x18

#define GET_INFO_SAMPLE_CMD                         0x19
#define GET_INFO_SAMPLE_ACK                         0x1A

#define GET_CHUNK_CMD                               0x1B
#define GET_CHUNK_ACK                               0x1C

#define SET_EXT_LASER_PROFILE_CMD                   0x1D
#define SET_EXT_LASER_PROFILE_ACK                   0x1E

#define TURN_ON_EXT_LASER_CMD                       0x21
#define TURN_ON_EXT_LASER_ACK                       0x22

#define TURN_OFF_EXT_LASER_CMD                      0x23
#define TURN_OFF_EXT_LASER_ACK                      0x24

//-------------------------------------------------------
#define CUSTOM_COMMAND_CMD							0x36
#define CUSTOM_COMMAND_ACK							0x37
//----
#define PING_CMD                                    0x38  ///< Ping request
#define PONG_CMD                                    0x39  ///< Pong response
//----
#define	MIN_RESP_NAK		                        0x3A
#define	MIN_RESP_ACK		                        0x3B
//----
#define	MIN_RESP_WRONG		                        0x3C
#define	MIN_RESP_DONE		                        0x3D
//----
#define MIN_RESP_FAIL		                        0x3E
#define	MIN_RESP_OK			                        0x3F


/**
 * @brief Command handler function type.
 * @param ctx Pointer to the MIN context.
 * @param payload Pointer to the received payload data.
 * @param len Length of the payload.
 */
typedef void (*MIN_CommandHandler)(MIN_Context_t *ctx, const uint8_t *payload, uint8_t len);

/**
 * @brief Structure to map command IDs to their handlers.
 */
typedef struct {
    uint8_t id;                    ///< Command ID
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






#endif /* APP_MIN_SHELL_MIN_SHELL_COMMAND_H_ */
