#include "min_shell_command.h"

// Command handlers
static void cmd_turn_on_led(uint8_t const *params, uint8_t param_len, uint8_t *response, uint8_t *resp_len) {
    if (param_len == 1) {
        uint8_t led_id = params[0];
        // HAL_GPIO_WritePin(led_id == 1 ? GPIOA : GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        response[0] = 0x00; // OK
        *resp_len = 1;
    } else {
        response[0] = 0x01; // ERROR
        *resp_len = 1;
    }
}

static void cmd_read_sensor(uint8_t const *params, uint8_t param_len, uint8_t *response, uint8_t *resp_len) {
    if (param_len == 1) {
        uint8_t sensor_id = params[0];
        uint16_t value = (sensor_id == 1) ? 2500 : 3000; // Giả lập (16-bit)
        response[0] = 0x00; // OK
        response[1] = (uint8_t)(value & 0xFF);        // LSB
        response[2] = (uint8_t)((value >> 8) & 0xFF); // MSB
        *resp_len = 3;
    } else {
        response[0] = 0x01; // ERROR
        *resp_len = 1;
    }
}

static void cmd_set_pwm(uint8_t const *params, uint8_t param_len, uint8_t *response, uint8_t *resp_len) {
    if (param_len == 3) {
        uint8_t channel = params[0];
        uint16_t frequency = params[1] | ((uint16_t)params[2] << 8); // Little-endian
        // HAL_TIM_PWM_SetFrequency(channel, frequency);
        response[0] = 0x00; // OK
        *resp_len = 1;
    } else {
        response[0] = 0x01; // ERROR
        *resp_len = 1;
    }
}

// Command table
static t_min_cmd_line_entry commands[] = {
    {0x01, 1, cmd_turn_on_led, "turn_on_led <led_id:8-bit> - Turn on LED"},
    {0x03, 1, cmd_read_sensor, "read_sensor <sensor_id:8-bit> - Read sensor value (16-bit)"},
    {0x05, 3, cmd_set_pwm, "set_pwm <channel:8-bit><frequency:16-bit> - Set PWM frequency"}
};
#define NUM_COMMANDS (sizeof(commands) / sizeof(commands[0]))

// Process command
void cmd_process(uint8_t const *data, uint8_t len, uint8_t *response, uint8_t *resp_len) {
    if (len < 1) {
        response[0] = 0x01; // ERROR: No command code
        *resp_len = 1;
        return;
    }

    uint8_t code = data[0];
    uint8_t param_len = len - 1;
    uint8_t const *params = data + 1;

    for (uint32_t i = 0; i < NUM_COMMANDS; i++) {
        if (code == commands[i].cmd) {
            if (param_len == commands[i].expected_param_len) {
                commands[i].pfn_cmd(params, param_len, response, resp_len);
            } else {
                response[0] = 0x01; // ERROR: Invalid param length
                *resp_len = 1;
            }
            return;
        }
    }

    response[0] = 0x01; // Command not found
    *resp_len = 1;
}



