/*
 * bsp_spi_ram.c
 *
 *  Created on: Jun 25, 2025
 *      Author: Admin
 */
#include "bsp_spi_ram.h"
#include "IS66WVS4M8BLL.h"
#include "app_signals.h"
#include "experiment_task.h"
// Byte dummy cho DMA TX

static experiment_evt_t const done_read_ram_evt = {.super = {.sig = EVT_EXPERIMENT_DONE_READ_RAM}};
extern experiment_task_t experiment_task_inst;
// Khởi tạo cấu hình SRAM
IS66_t IS66WV = {
    .spi = SPI6,
    .cs_port = GPIOD,
    .cs_pin = LL_GPIO_PIN_7, // Dùng PD7 làm CS
    .remain_size = 0,
    .transfer_done = 0,
    .dma = DMA2, // Dùng DMA2
    .dma_stream_tx = LL_DMA_STREAM_5, // Stream 5 cho TX
    .dma_stream_rx = LL_DMA_STREAM_6, // Stream 6 cho RX
    .dma_channel = LL_DMA_CHANNEL_1 // Channel 1 chung
};

void bsp_spi_ram_init(void)
{
	SRAM_Initialize(&IS66WV);
}
void bsp_spi_ram_write_polling(uint32_t address, uint32_t size, uint8_t *buffer)
{
	SRAM_write_polling(&IS66WV, address, size, buffer);
}
void bsp_spi_ram_read_polling(uint32_t address, uint32_t size, uint8_t *buffer)
{
	SRAM_read_polling(&IS66WV, address, size, buffer);
}
void bsp_spi_ram_fast_read_polling(uint32_t address, uint32_t size, uint8_t *buffer)
{
	SRAM_fast_read_polling(&IS66WV, address, size, buffer);
}

void bsp_spi_ram_write_dma(uint32_t address, uint32_t size, uint8_t *buffer)
{
	IS66WV.address = address;
	IS66WV.remain_size = size;
	IS66WV.buffer = buffer;
	IS66WV.sram_mode = SRAM_MODE_WRITE;
	IS66WV.transfer_done = 0;
	SRAM_DMA_Transfer(&IS66WV, MAX_BURST_SIZE);
}

void bsp_spi_ram_read_dma(uint32_t address, uint32_t size, uint8_t *buffer)
{
	IS66WV.address = address;
	IS66WV.remain_size = size;
	IS66WV.buffer = buffer;
	IS66WV.sram_mode = SRAM_MODE_READ;
	IS66WV.transfer_done = 0;
	SRAM_DMA_Transfer(&IS66WV, MAX_BURST_SIZE);
}

void bsp_spi_ram_read_id(uint8_t * buffer)
{
	SRAM_read_id(&IS66WV,  buffer);
}

uint8_t bsp_spi_ram_is_transfer_done(void)
{
	return SRAM_IsTransferDone(&IS66WV);
}
// Hàm xử lý ngắt DMA RX (SPI2_RX)
void DMA2_Stream6_IRQHandler(void)
{
	DMA2->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTCIF5;
	IS66WV.cs_port->BSRR = IS66WV.cs_pin;				// CS_HIGH

	// Kiểm tra và truyền khối tiếp theo
	if (IS66WV.remain_size > 0)
	{
		IS66WV.buffer += IS66WV.prev_size;
		SRAM_DMA_Transfer(&IS66WV, (IS66WV.remain_size > MAX_BURST_SIZE) ? MAX_BURST_SIZE : IS66WV.remain_size);
	}
	else
	{
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_6);
		LL_SPI_DisableDMAReq_TX(SPI6);
		LL_SPI_DisableDMAReq_RX(SPI6);
		IS66WV.transfer_done = 1;
		if(!IS66WV.sram_mode)		// If mode read
		{
			SST_Task_post((SST_Task *)&experiment_task_inst.super, (SST_Evt *)&done_read_ram_evt);
		}
	}

}
