/*
 * min_command.c
 *
 *  Created on: May 5, 2025
 *      Author: HTSANG
 */

#include "min_command.h"
#include <string.h>
#include "stdio.h"
#include "main.h"
#include "lt8722.h"
#include "board.h"
#include "uart_driver.h"
//#include "temperature.h"



#define MAX_SIZE (100 * 1024) // 100KB
#define RAM_D2_200KB_SIZE  (200 * 1024) // 200KB

#define SPI_RAM_START ((uint8_t*)&_schunk_data)
#define SPI_RAM_SIZE  (10 * 1024)
#define MAX_CHUNKS 20

//extern uint32_t _scustom_data;
//extern uint32_t _ecustom_data;

extern uint8_t _schunk_data[];
extern uint8_t _echunk_data[];

uint32_t g_total_size;
uint16_t g_chunk_size;

uint32_t g_sample_rate;
uint16_t g_chunk_crcs[MAX_CHUNKS];
// =================================================================
// Command Handlers
// =================================================================

//static void MIN_Handler_COLLECT_DATA(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
//    printf("min_payload (%u bytes):", len_payload);
//    for (uint8_t i = 0; i < len_payload; i++) {
//        printf(" %02X", min_payload[i]);
//    }
//    printf("\r\n");
//
//    if (len_payload < 6) {
//        printf("Invalid min_payload len_payloadgth.\r\n");
//        return;
//    }
//    uint16_t chunk_size = (min_payload[1] << 8) | min_payload[0];
//    uint32_t sample = (min_payload[5] << 24) | (min_payload[4] << 16) | (min_payload[3] << 8) | min_payload[2];
//
//    printf("Message -Hex: chunk_size = %u (0x%04X), sample = %lu (0x%08lX)\r\n",
//           chunk_size, chunk_size, (unsigned long)sample, (unsigned long)sample);
//
//    MIN_Send(ctx, GOT_IT, NULL, 0);
//}

//static uint16_t UpdateCRC16_XMODEM(uint16_t crc, uint8_t byte) {
//    const uint16_t polynomial = 0x1021; // CRC16 XMODEM
//    crc ^= (uint16_t)byte << 8;
//    for (uint8_t bit = 0; bit < 8; bit++) {
//        if (crc & 0x8000) {
//            crc = (crc << 1) ^ polynomial;
//        } else {
//            crc <<= 1;
//        }
//    }
//    return crc;
//}

//static void MIN_Handler_SAMPLERATE_SET(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
//    if (len_payload < 4) {
//        printf("Invalid min_payload len_payloadgth.\r\n");
//        return;
//    }
//
//    g_sample_rate = (min_payload[3] << 24) | (min_payload[2] << 16) | (min_payload[1] << 8) | min_payload[0];
//
//    printf("Sample rate set to %lu Hz\r\n", (unsigned long)g_sample_rate);
//    MIN_Send(ctx, DONE, NULL, 0);
//}
//
//static void MIN_Handler_SAMPLERATE_GET(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
//    (void)min_payload; (void)len_payload;
//    uint8_t response[4];
//
//    response[0] = (uint8_t)(g_sample_rate & 0xFF);
//    response[1] = (uint8_t)((g_sample_rate >> 8) & 0xFF);
//    response[2] = (uint8_t)((g_sample_rate >> 16) & 0xFF);
//    response[3] = (uint8_t)((g_sample_rate >> 24) & 0xFF);
//
//    printf("Sample rate retrieved: %lu Hz\r\n", (unsigned long)g_sample_rate);
//    MIN_Send(ctx, SAMPLERATE_GET_ACK, response, sizeof(response));
//}

//static void MIN_Handler_COLLECT_DATA(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
//
//	toOBC_SetState(toOBC_BUSY);
//
//    if (len_payload < 4) {
//        printf("Invalid min_payload len_payloadgth.\r\n");
//    	toOBC_SetState(toOBC_ERROR);
//        return;
//    }
//
//    uint32_t sample = (min_payload[3] << 24) | (min_payload[2] << 16) | (min_payload[1] << 8) | min_payload[0];
//    uint32_t total_size = sample * 2;
//
//    if (total_size > RAM_D2_200KB_SIZE) {
//        printf("Data size exceeds available RAM (%lu > %lu).\r\n",
//               (unsigned long)total_size, (unsigned long)RAM_D2_200KB_SIZE);
//    	toOBC_SetState(toOBC_ERROR);
//        return;
//    }
//
//    if (CSP_QSPI_Read(RAM_D2_200KB_START, 0, total_size) != HAL_OK) {
//        printf("Error reading QSPI Flash.\r\n");
//    	toOBC_SetState(toOBC_ERROR);
//        return;
//    }
//
//    g_total_size = total_size;
//
//    printf("Collected %lu samples, total size: %lu bytes\r\n",
//           (unsigned long)sample, (unsigned long)total_size);
//    MIN_Send(ctx, DONE, NULL, 0);
//	toOBC_SetState(toOBC_READYSEND);
//}

//static void MIN_Handler_PRE_DATA(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
//    if (len_payload < 2) {
//        printf("Invalid min_payload len_payloadgth.\r\n");
//        return;
//    }
//
//    if (g_total_size == 0) {
//        printf("No data collected yet.\r\n");
//        return;
//    }
//
//    uint16_t chunk_size = (min_payload[1] << 8) | min_payload[0];
//    g_chunk_size = chunk_size;
//
//    uint32_t num_chunks = (g_total_size + g_chunk_size - 1) / g_chunk_size;
//    if (num_chunks > MAX_CHUNKS) {
//        printf("Too many chunks (%lu > %d).\r\n", (unsigned long)num_chunks, MAX_CHUNKS);
//        return;
//    }
//
//    memset(g_chunk_crcs, 0, sizeof(g_chunk_crcs));
//
//    uint8_t response[1 + 2 * num_chunks];
//    response[0] = (uint8_t)num_chunks;
//
//    for (uint32_t i = 0; i < num_chunks; i++) {
//        uint32_t offset = i * g_chunk_size;
//        uint32_t size = (i == num_chunks - 1) ? (g_total_size % g_chunk_size) : g_chunk_size;
//        if (size == 0) size = g_chunk_size;
//        uint16_t crc = 0x0000;
//        for (uint32_t j = 0; j < size; j++) {
//            crc = UpdateCRC16_XMODEM(crc, RAM_D2_200KB_START[offset + j]);
//        }
//        g_chunk_crcs[i] = crc;
//        response[1 + 2 * i] = (uint8_t)(crc >> 8);
//        response[2 + 2 * i] = (uint8_t)(crc & 0xFF);
//    }
//
//    MIN_Send(ctx, PRE_DATA_ACK, response, sizeof(response));
//    printf("Pre Data Success: %lu Chunks\r\n", (unsigned long)num_chunks);
//    for (uint32_t i = 0; i < num_chunks; i++) {
//        printf("%lu -> CRC16: 0x%04X\r\n", (unsigned long)i, g_chunk_crcs[i]);
//    }
//}

//static void MIN_Handler_PRE_CHUNK(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
//    if (len_payload < 1) {
//        printf("Invalid min_payload len_payloadgth.\r\n");
//        return;
//    }
//
//    uint8_t chunk_num = min_payload[0];
//    uint32_t num_chunks = (g_total_size + g_chunk_size - 1) / g_chunk_size;
//
//    if (chunk_num >= num_chunks) {
//        printf("Invalid chunk number (%u >= %lu).\r\n", chunk_num, (unsigned long)num_chunks);
//        return;
//    }
//
//    uint32_t offset = chunk_num * g_chunk_size;
//    uint32_t size = (chunk_num == num_chunks - 1) ? (g_total_size % g_chunk_size) : g_chunk_size;
//    if (size == 0) size = g_chunk_size;
//
//    memset(SPI_RAM_START, 0, SPI_RAM_SIZE);
//    memcpy(SPI_RAM_START, RAM_D2_200KB_START + offset, size);
//
//    SPI_SlaveDevice_ResetDMA((uint32_t)SPI_RAM_START, g_chunk_size);
//
//    uint8_t response[2];
//    response[0] = (uint8_t)(g_chunk_crcs[chunk_num] >> 8);
//    response[1] = (uint8_t)(g_chunk_crcs[chunk_num] & 0xFF);
//
//    MIN_Send(ctx, PRE_CHUNK_ACK, response, sizeof(response));
//    printf("Prepared chunk %u for SPI read, size: %lu bytes, CRC16: 0x%04X\r\n",
//           chunk_num, (unsigned long)size, g_chunk_crcs[chunk_num]);
//}

//static void MIN_Handler_COLLECT_PACKAGE(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
//    (void)min_payload; (void)len_payload;
//    const uint32_t package_size = SPI_RAM_SIZE;
//    const uint32_t sample_count = package_size / 2;
//
//    memset(SPI_RAM_START, 0, SPI_RAM_SIZE);
//
//    if (CSP_QSPI_Read(SPI_RAM_START, 0, SPI_RAM_SIZE) != HAL_OK) {
//        printf("Error reading QSPI Flash.\r\n");
//        return;
//    }
//
//    uint16_t crc = 0x0000;
//    for (uint32_t i = 0; i < package_size; i++) {
//        crc = UpdateCRC16_XMODEM(crc, SPI_RAM_START[i]);
//    }
//
//    uint8_t response[2];
//    response[0] = (uint8_t)(crc >> 8);
//    response[1] = (uint8_t)(crc & 0xFF);
//
//    printf("Collected package: %lu samples, size: %lu bytes, CRC16: 0x%04X\r\n",
//           (unsigned long)sample_count, (unsigned long)package_size, crc);
//    MIN_Send(ctx, COLLECT_PACKAGE_ACK, response, sizeof(response));
//}

void MIN_Handler_DEV_STATUS_GET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
	return;
}

void MIN_Handler_NTC_TEMP_GET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
//    uint8_t response[5];
//    response[0] = *(min_payload);
//    response[1] = '2';
//    response[2] = '5';
//    response[3] = '.';
//    response[4] = '3';


//	uint8_t option = *(min_payload + 1);
//	NTC_get_temperature(NTC_Temperature);
//	if (option == 0x0F) {
//	    uint8_t response[17] = {*(min_payload)};
//		for (uint8_t i = 0; i < 8; i++) {
//			response[2*i + 1] = (uint8_t)((NTC_Temperature[i] >> 8) & 0xFF);
//			response[2*i + 2] = (uint8_t)(NTC_Temperature[i] & 0xFF);
//		}
//	    MIN_Send(ctx, min_id, (const uint8_t *)response, 17);
//	    return;
//	}
//	else if (option == 0x01 || option == 0x02 || option == 0x04 || option == 0x08 ||
//			 option == 0x10 || option == 0x20 || option == 0x40 || option == 0x80) {
//		uint8_t ntc_temp_H = 0;
//		uint8_t ntc_temp_L = 0;
//		for (uint8_t i = 0; i < 8; i++) {
//			if ((option >> i) & 0x01) {
//				ntc_temp_H = (uint8_t)((NTC_Temperature[i] >> 8) & 0xFF);
//				ntc_temp_L = (uint8_t)(NTC_Temperature[i] & 0xFF);
//				break;
//			}
//		}
//		uint8_t response[3] = {*(min_payload), ntc_temp_H, ntc_temp_L};
//		MIN_Send(ctx, min_id, (const uint8_t *)response, 3);
//		return;
//	}
//	else {
//		uint8_t response[2] = {*(min_payload), MIN_NAK};
//		MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
//		return;
//	}
}

void MIN_Handler_PWR_5V_SET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
	if (*(min_payload + 1))
		LL_GPIO_SetOutputPin(EF_5_EN_GPIO_Port, EF_5_EN_Pin);
	else
		LL_GPIO_ResetOutputPin(EF_5_EN_GPIO_Port, EF_5_EN_Pin);

	uint8_t response[2]= {*(min_payload), MIN_ACK};
    MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_PWR_5V_GET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
    uint8_t pwr_status = LL_GPIO_IsOutputPinSet(EF_5_EN_GPIO_Port, EF_5_EN_Pin) & 0x01;
    uint8_t response[2] = {*(min_payload), pwr_status};
    MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_TEC_INIT_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
//	uint8_t response[2] = {*(min_payload), MIN_ACK};
//	uint8_t option = *(min_payload + 1);
//	if (option == 0x0F || option == 0x01 || option == 0x02 || option == 0x04 || option == 0x08)
//		LT8722_Status = (LT8722_Status & 0xF0) | option;
//	else
//		response[1] = MIN_NAK;
//    MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_TEC_STATUS_GET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
//    uint8_t status = (LT8722_Status >> 4) & 0x0F;
//    uint8_t response[2] = {*(min_payload), status};
//    MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_TEC_VOLT_SET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
	uint8_t response[2] = {*(min_payload), MIN_ACK};
	uint16_t volt[4];
	volt[0] = ((uint16_t)*(min_payload + 1) << 8) | (*(min_payload + 2));
	volt[1] = ((uint16_t)*(min_payload + 3) << 8) | (*(min_payload + 4));
	volt[2] = ((uint16_t)*(min_payload + 5) << 8) | (*(min_payload + 6));
	volt[3] = ((uint16_t)*(min_payload + 7) << 8) | (*(min_payload + 8));
	for (uint8_t i = 0; i < 4; i++) {
		temperature_set_tec_vol(i, volt[i]);
	}
    MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_TEC_VOLT_GET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
	uint8_t option = *(min_payload + 1);
	if (option == 0x0F) {
	    uint16_t volt_0 = temperature_get_tec_vol_set(0);
	    uint16_t volt_1 = temperature_get_tec_vol_set(1);
	    uint16_t volt_2 = temperature_get_tec_vol_set(2);
	    uint16_t volt_3 = temperature_get_tec_vol_set(3);
	    uint8_t response[9] = {*(min_payload), (volt_0 >> 8) & 0xFF, volt_0 & 0xFF, (volt_1 >> 8) & 0xFF, volt_1 & 0xFF, (volt_2 >> 8) & 0xFF, volt_2 & 0xFF, (volt_3 >> 8) & 0xFF, volt_3 & 0xFF};
	    MIN_Send(ctx, min_id, (const uint8_t *)response, 9);
	}
	else if (option == 0x01 || option == 0x02 || option == 0x04 || option == 0x08) {
		uint16_t volt = 0;
		for (uint8_t i = 0; i < 4; i++) {
			if ((option >> i) & 0x01) {
				volt = temperature_get_tec_vol_set(i);
				break;
			}
		}
		uint8_t response[3] = {*(min_payload), (volt >> 8) & 0xFF, volt & 0xFF};
		MIN_Send(ctx, min_id, (const uint8_t *)response, 3);
	}
	else {
		uint8_t response[2] = {*(min_payload), MIN_NAK};
		MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
		return;
	}
}

void MIN_Handler_TEC_DIR_SET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
	uint8_t dir = (uint8_t)*(min_payload + 1);
	temperature_set_tec_dir((tec_dir_t)(dir & 0x01), (tec_dir_t)((dir >> 1) & 0x01), (tec_dir_t)((dir >> 2) & 0x01), (tec_dir_t)((dir >> 3) & 0x01));
	uint8_t response[2] = {*(min_payload), MIN_ACK};
    MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_TEC_DIR_GET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
//    uint8_t tec_dir = 0;
//    temperature_get_tec_dir(&tec_dir);
//    uint8_t response[2] = {*(min_payload), tec_dir};
//    MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_HTR_DUTY_SET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
	uint8_t duty[4];
	duty[0] = (uint8_t)*(min_payload + 1);
	duty[1] = (uint8_t)*(min_payload + 2);
	duty[2] = (uint8_t)*(min_payload + 3);
	duty[3] = (uint8_t)*(min_payload + 4);
	for (uint8_t i = 0; i < 4; i++) {
		temperature_set_heater_duty(i, duty[i]);
	}

	uint8_t response[2] = {*(min_payload), MIN_ACK};
    MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_HTR_DUTY_GET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
	uint8_t option = *(min_payload + 1);
	if (option == 0x0F) {
		uint8_t duty_0 = temperature_get_heater_duty(0);
		uint8_t duty_1 = temperature_get_heater_duty(1);
		uint8_t duty_2 = temperature_get_heater_duty(2);
		uint8_t duty_3 = temperature_get_heater_duty(3);
	    uint8_t response[5] = {*(min_payload), duty_0, duty_1, duty_2, duty_3};
	    MIN_Send(ctx, min_id, (const uint8_t *)response, 5);
	}
	else if (option == 0x01 || option == 0x02 || option == 0x04 || option == 0x08) {
		uint8_t duty = 0;
		for (uint8_t i = 0; i < 4; i++) {
			if ((option >> i) & 0x01) {
				duty = temperature_get_heater_duty(i);
				break;
			}
		}
		uint8_t response[2] = {*(min_payload), duty};
		MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
	}
	else {
		uint8_t response[2] = {*(min_payload), MIN_NAK};
		MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
		return;
	}
}

void MIN_Handler_REF_TEMP_SET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
	uint16_t temp_setpoint = ((uint16_t)*(min_payload + 1) << 8) | (*(min_payload + 2));
	temperature_set_setpoint(temp_setpoint);
	uint8_t response[2] = {*(min_payload), MIN_ACK};
    MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_REF_TEMP_GET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
	uint16_t temp_setpoint = temperature_get_setpoint();
	uint8_t response[3] = {*(min_payload), (uint8_t)((temp_setpoint >> 8) & 0xFF), (uint8_t)(temp_setpoint & 0xFF)};
    MIN_Send(ctx, min_id, (const uint8_t *)response, 3);
}

void MIN_Handler_REF_NTC_SET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
	uint8_t tempo = *(min_payload + 1);
	uint8_t ntc_ref = 0;
	if (tempo == 0x01 || tempo == 0x02 || tempo == 0x04 || tempo == 0x08 ||
		tempo == 0x10 || tempo == 0x20 || tempo == 0x40 || tempo == 0x80) {
		for (uint8_t i = 0; i < 8; i++) {
			if ((tempo >> i) & 0x01) {
				ntc_ref = i;
				break;
			}
		}
		temperature_set_ntc_ref(ntc_ref);
	}
	else {
		uint8_t response[2] = {*(min_payload), MIN_NAK};
		MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
		return;
	}

	uint8_t response[2] = {*(min_payload), MIN_ACK};
	MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_REF_NTC_GET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
	uint8_t ntc_ref = 0;
	temperature_get_ntc_ref(&ntc_ref);
	ntc_ref = 0x01 << ntc_ref;
	uint8_t response[2] = {*(min_payload), ntc_ref};
	MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_AUTO_TEC_SET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
//	uint8_t tec_ena = (uint8_t)*(min_payload + 1);
//	temperature_set_tec_auto(tec_ena & 0x0F);
//
//	uint8_t response[2] = {*(min_payload), MIN_ACK};
//    MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_AUTO_TEC_GET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
//	uint8_t tec_ena = 0;
//	temperature_get_tec_auto(&tec_ena);
//
//	uint8_t response[2] = {*(min_payload), tec_ena};
//	MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_AUTO_HTR_SET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
//	uint8_t heater_ena = (uint8_t)*(min_payload + 1);
//	temperature_set_heater_auto(heater_ena & 0x0F);
//
//	uint8_t response[2] = {*(min_payload), MIN_ACK};
//    MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_AUTO_HTR_GET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
//	uint8_t heater_ena = 0;
//	temperature_get_heater_auto(&heater_ena);
//
//	uint8_t response[2] = {*(min_payload), heater_ena};
//	MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_AUTO_TEMP_SET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
	uint8_t Temp_auto = *(min_payload + 1) ? 1 : 0;
	temperature_set_auto_ctrl(Temp_auto);

	uint8_t response[2] = {*(min_payload), MIN_ACK};
    MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_AUTO_TEMP_GET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
	uint8_t Temp_auto = 0;
	temperature_get_auto_ctrl(&Temp_auto);

	uint8_t response[2] = {*(min_payload), Temp_auto};
	MIN_Send(ctx, min_id, (const uint8_t *)response, 2);
}

void MIN_Handler_LSM_SENS_GET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
	return;
}

void MIN_Handler_H3L_SENS_GET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
	return;
}

void MIN_Handler_BME_SENS_GET_CMD(MIN_Context_t *ctx, uint8_t min_id, const uint8_t *min_payload, uint8_t len_payload) {
	return;
}

// =================================================================
// Command Table
// =================================================================

static const MIN_Command_t command_table[] = {
    { NTC_TEMP_GET_CMD,         MIN_Handler_NTC_TEMP_GET_CMD },
    { PWR_5V_SET_CMD,           MIN_Handler_PWR_5V_SET_CMD },
    { PWR_5V_GET_CMD,           MIN_Handler_PWR_5V_GET_CMD },
    { TEC_INIT_CMD,             MIN_Handler_TEC_INIT_CMD },
    { TEC_STATUS_GET_CMD,       MIN_Handler_TEC_STATUS_GET_CMD },
    { TEC_VOLT_SET_CMD,         MIN_Handler_TEC_VOLT_SET_CMD },
    { TEC_VOLT_GET_CMD,         MIN_Handler_TEC_VOLT_GET_CMD },
    { TEC_DIR_SET_CMD,          MIN_Handler_TEC_DIR_SET_CMD },
    { TEC_DIR_GET_CMD,          MIN_Handler_TEC_DIR_GET_CMD },
    { HTR_DUTY_SET_CMD,         MIN_Handler_HTR_DUTY_SET_CMD },
    { HTR_DUTY_GET_CMD,         MIN_Handler_HTR_DUTY_GET_CMD },
    { REF_TEMP_SET_CMD,         MIN_Handler_REF_TEMP_SET_CMD },
    { REF_TEMP_GET_CMD,         MIN_Handler_REF_TEMP_GET_CMD },
    { REF_NTC_SET_CMD,          MIN_Handler_REF_NTC_SET_CMD },
    { REF_NTC_GET_CMD,          MIN_Handler_REF_NTC_GET_CMD },
    { AUTO_TEC_SET_CMD,         MIN_Handler_AUTO_TEC_SET_CMD },
    { AUTO_TEC_GET_CMD,         MIN_Handler_AUTO_TEC_GET_CMD },
    { AUTO_HTR_SET_CMD,         MIN_Handler_AUTO_HTR_SET_CMD },
    { AUTO_HTR_GET_CMD,         MIN_Handler_AUTO_HTR_GET_CMD },
    { AUTO_TEMP_SET_CMD,        MIN_Handler_AUTO_TEMP_SET_CMD },
    { AUTO_TEMP_GET_CMD,        MIN_Handler_AUTO_TEMP_GET_CMD },
    { LSM_SENS_GET_CMD,         MIN_Handler_LSM_SENS_GET_CMD },
    { H3L_SENS_GET_CMD,         MIN_Handler_H3L_SENS_GET_CMD },
    { BME_SENS_GET_CMD,         MIN_Handler_BME_SENS_GET_CMD },
};


static const int command_table_size = sizeof(command_table) / sizeof(command_table[0]);

// =================================================================
// Helper Functions
// =================================================================

const MIN_Command_t *MIN_GetCommandTable(void) {
	return command_table;
}

int MIN_GetCommandTableSize(void) {
	return command_table_size;
}
