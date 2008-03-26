/*
 *  Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/init.h>
#include <linux/device.h>
#include <asm/dma.h>
#include <asm/arch/hardware.h>

#include "serial.h"

struct mxc_sdma_info_entry_s {
	mxc_dma_device_t device;
	mxc_sdma_channel_params_t *chnl_info;
};

static mxc_sdma_channel_params_t mxc_sdma_uart1_rx_params = {
	.chnl_params = {
			.watermark_level = UART1_UFCR_RXTL,
			.per_address = UART1_BASE_ADDR,
			.peripheral_type = UART,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_UART1_RX,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_UART1_RX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_uart1_tx_params = {
	.chnl_params = {
			.watermark_level = UART1_UFCR_TXTL,
			.per_address = UART1_BASE_ADDR + MXC_UARTUTXD,
			.peripheral_type = UART,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_UART1_TX,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_UART1_TX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_uart2_rx_params = {
	.chnl_params = {
			.watermark_level = UART2_UFCR_RXTL,
			.per_address = UART2_BASE_ADDR,
			.peripheral_type = UART,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_UART2_RX,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_UART2_RX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_uart2_tx_params = {
	.chnl_params = {
			.watermark_level = UART2_UFCR_TXTL,
			.per_address = UART2_BASE_ADDR + MXC_UARTUTXD,
			.peripheral_type = UART,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_UART2_TX,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_UART2_TX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_uart3_rx_params = {
	.chnl_params = {
			.watermark_level = UART3_UFCR_RXTL,
			.per_address = UART3_BASE_ADDR,
			.peripheral_type = UART_SP,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_UART3_RX,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_UART3_RX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_uart3_tx_params = {
	.chnl_params = {
			.watermark_level = UART3_UFCR_TXTL,
			.per_address = UART3_BASE_ADDR + MXC_UARTUTXD,
			.peripheral_type = UART_SP,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_UART3_TX,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_UART3_TX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_memory_params = {
	.chnl_params = {
			.peripheral_type = MEMORY,
			.transfer_type = emi_2_emi,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_MEMORY,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static struct mxc_sdma_info_entry_s mxc_sdma_active_dma_info[] = {
	{MXC_DMA_UART1_RX, &mxc_sdma_uart1_rx_params},
	{MXC_DMA_UART1_TX, &mxc_sdma_uart1_tx_params},
	{MXC_DMA_UART2_RX, &mxc_sdma_uart2_rx_params},
	{MXC_DMA_UART2_TX, &mxc_sdma_uart2_tx_params},
	{MXC_DMA_UART3_RX, &mxc_sdma_uart3_rx_params},
	{MXC_DMA_UART3_TX, &mxc_sdma_uart3_tx_params},
	{MXC_DMA_MEMORY, &mxc_sdma_memory_params},
};

static int mxc_sdma_info_entrys =
    sizeof(mxc_sdma_active_dma_info) / sizeof(mxc_sdma_active_dma_info[0]);
/*!
 * This functions Returns the SDMA paramaters associated for a module
 *
 * @param channel_id the ID of the module requesting DMA
 * @return returns the sdma parameters structure for the device
 */
mxc_sdma_channel_params_t *mxc_sdma_get_channel_params(mxc_dma_device_t
						       channel_id)
{
	struct mxc_sdma_info_entry_s *p = mxc_sdma_active_dma_info;
	int i;

	for (i = 0; i < mxc_sdma_info_entrys; i++, p++) {
		if (p->device == channel_id)
			return p->chnl_info;
	}
	return NULL;
}

EXPORT_SYMBOL(mxc_sdma_get_channel_params);

/*!
 * This functions marks the SDMA channels that are statically allocated
 *
 * @param chnl the channel array used to store channel information
 */
void mxc_get_static_channels(mxc_dma_channel_t *chnl)
{
	/* No channels statically allocated for MX35 */
}

EXPORT_SYMBOL(mxc_get_static_channels);
