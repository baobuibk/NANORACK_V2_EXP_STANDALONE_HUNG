/*
 * min_shell_command.h
 *
 *  Created on: Jun 13, 2025
 *      Author: Admin
 */

#ifndef APP_MIN_SHELL_MIN_SHELL_COMMAND_H_
#define APP_MIN_SHELL_MIN_SHELL_COMMAND_H_



#include <stdint.h>

// Callback type for command processing
typedef void (*pfnCmdLine)(uint8_t const *params, uint8_t param_len, uint8_t *response, uint8_t *resp_len);

// Structure for command table entry
typedef struct {
    uint8_t cmd;          // Command code
    uint8_t expected_param_len; // Expected parameter length
    pfnCmdLine pfn_cmd;   // Function pointer to command handler
    const char *pc_help;  // Help text for the command
} t_min_cmd_line_entry;

// Process command from MIN payload
void cmd_process(uint8_t const *data, uint8_t len, uint8_t *response, uint8_t *resp_len);



#endif /* APP_MIN_SHELL_MIN_SHELL_COMMAND_H_ */
