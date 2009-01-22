/* -*- linux-c -*-
 *
 * drivers/fims/fim_sdio.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 *  !Revision:   $Revision: 1.20 $
 *  !Author:     Luis Galdos
 *  !Descr:      
 *  !References:
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/scatterlist.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/delay.h>

#include <asm/scatterlist.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <mach/gpio.h>
#include <mach/regs-sys-ns921x.h>
#include <mach/regs-iohub-ns921x.h>
#include <mach/regs-io-ns921x.h>
#include <asm/delay.h>


/* For registering the FIM-driver */
#include <mach/fim-ns921x.h>


/*
 * If the driver is going to be loaded as a built-in driver, then include the header
 * file with the firmware, otherwise set the name of the binary file that should
 * be read with the help of the firmware-subsystem
 */
#if !defined(MODULE)
#include "fim_sdio.h"
extern const unsigned char fim_sdio_firmware[];
#define FIM_SDIO_FIRMWARE_FILE			(NULL)
#define FIM_SDIO_FIRMWARE_CODE			fim_sdio_firmware
#else
const unsigned char *fim_sdio_firmware = NULL;
#define FIM_SDIO_FIRMWARE_FILE			"fim_sdio.bin"
#define FIM_SDIO_FIRMWARE_CODE			(NULL)
#endif

/* Driver informations */
#define DRIVER_VERSION				"0.2"
#define DRIVER_AUTHOR				"Luis Galdos"
#define DRIVER_DESC				"FIM SDIO driver"
#define FIM_SDIO_DRIVER_NAME			"fim-sdio"
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

/* Registers with status information */
#define FIM_SD_GPIOS_REG			0x02
#define FIM_SD_GPIOS_REG_CD			0x01
#define FIM_SD_GPIOS_REG_WP			0x02
#define FIM_SD_CARD_STATREG			0x00

/* Interrupts from the FIM to the driver */
#define FIM_SD_INTARM_CARD_DAT1			0x01
#define FIM_SD_INTARM_CARD_DETECTED		0x02


/* Macros for the SDIO-interface to the FIM-firmware */
#define SDIO_HOST_TX_HDR			0x40
#define SDIO_HOST_CMD_MASK			0x3f
#define SDIO_FIFO_TX_48RSP			0x01
#define	SDIO_FIFO_TX_136RSP			0x02
#define SDIO_FIFO_TX_BW4			0x04
#define SDIO_FIFO_TX_BLKWR			0x08
#define SDIO_FIFO_TX_BLKRD			0x10
#define SDIO_FIFO_TX_DISCRC			0x20

/* User specified macros */
#define FIM_SD_TIMEOUT_MS			2000
#define FIM_SD_TX_CMD_LEN			5
#define FIM_SD_MAX_RESP_LENGTH			17

/* Status bits from the PIC-firmware */
#define FIM_SD_RX_RSP				0x01
#define FIM_SD_RX_BLKRD				0x02
#define FIM_SD_RX_TIMEOUT			0x04

/* FIM SD control registers */
#define FIM_SD_REG_CLOCK_DIVISOR		0
#define FIM_SD_REG_INTERRUPT			1

/* Internal flags for the request function */
#define FIM_SD_REQUEST_NEW			0x00
#define FIM_SD_REQUEST_CMD			0x01
#define FIM_SD_REQUEST_STOP			0x02
#define FIM_SD_SET_BUS_WIDTH			0x04

/* Macros for the DMA-configuraton */
#define FIM_SD_DMA_BUFFER_SIZE			PAGE_SIZE
#define FIM_SD_DMA_RX_BUFFERS			21
#define FIM_SD_DMA_TX_BUFFERS			10

#if 0
# define FIM_SD_MULTI_BLOCK
# define FIM_SD_MAX_BLOCKS			2
#else
# define FIM_SD_MAX_BLOCKS			1
#endif

#define printk_err(fmt, args...)                printk(KERN_ERR "[ ERROR ] fim-sdio: " fmt, ## args)
#define printk_info(fmt, args...)               printk(KERN_INFO "fim-sdio: " fmt, ## args)

#if 0
#define FIM_SD_DEBUG
#endif

#ifdef FIM_SD_DEBUG
#  define printk_debug(fmt, args...)		printk(KERN_DEBUG "fim-sdio: %s() " fmt, __FUNCTION__ , ## args)
#else
#  define printk_debug(fmt, args...)
#endif

/* If enabled will generate an CRC error in the function that checks it */
#if 0
#define FIM_SD_FORCE_CRC
#endif

/*
 * GPIO configuration
 * Please note that the write protect GPIO must be under the index [4], then
 * the driver will read its status (the firmware doesn't support it)
 */
#define FIM_SDIO_MAX_GPIOS			8

/* Values for the block read state machine */
enum fim_blkrd_state {
	BLKRD_STATE_IDLE		= 0,
	BLKRD_STATE_WAIT_ACK		= 1, /* Waiting for the block read ACK */
	BLKRD_STATE_WAIT_DATA		= 2, /* Waiting for the block read data */
	BLKRD_STATE_WAIT_CRC		= 3, /* Waiting for the CRC */
	BLKRD_STATE_HAVE_DATA		= 4, /* Have block read data with the CRC */
	BLKRD_STATE_TIMEOUTED		= 5, /* Timeout response from the PIC */
	BLKRD_STATE_CRC_ERR		= 6, /* Compared CRC (PIC and card) differs */
};


/* Values for the command state machine */
enum fim_cmd_state {
	CMD_STATE_IDLE			= 0,
	CMD_STATE_WAIT_ACK		= 1, /* Waiting for the response ACK */
	CMD_STATE_WAIT_DATA		= 2, /* Waiting for the response data */
	CMD_STATE_HAVE_RSP		= 3, /* Have response data */
	CMD_STATE_TIMEOUTED		= 4, /* Timeout response from the PIC */
	CMD_STATE_CRC_ERR		= 5, /* Compared CRC (PIC and card) differs */
};

/*
 * Internal port structure for the FIM-SDIO host controller
 * wp_gpio : GPIO to use for reading the write protect line
 */
struct fim_sdio_t {
	struct fim_driver fim;
	unsigned int flags;
	struct device *device;
	int index;

	struct semaphore sem;
	struct fim_gpio_t gpios[FIM_SDIO_MAX_GPIOS];
	struct fim_buffer_t *buf;
	struct mmc_command *mmc_cmd;
	struct timer_list mmc_timer;
	struct mmc_host *mmc;
	struct mmc_request *mmc_req;
	spinlock_t mmc_lock;

	enum fim_cmd_state cmd_state;
	enum fim_blkrd_state blkrd_state;

	int trans_blocks;
	int trans_sg;
	int reg;

	int wp_gpio;
};

/*
 * Transfer command structure for the card
 * opctl : Control byte for the PIC
 * blksz : Block size
 * cmd   : Command to send to the card
 */
struct fim_sd_tx_cmd_t {
	unsigned char opctl;
	unsigned char blksz_msb;
        unsigned char blksz_lsb;
        unsigned char cmd[FIM_SD_TX_CMD_LEN];
}__attribute__((__packed__));


/*
 * Response receive structure from the Card
 * resp  : Card response, with a length of 5 or 17 as appropriate
 * stat  : Opcode of the executed command
 * crc   : CRC
 */
struct fim_sd_rx_resp_t {
        unsigned char stat;
        unsigned char resp[FIM_SD_MAX_RESP_LENGTH];
        unsigned char crc;
}__attribute__((__packed__));


/* Main structure for the available ports */
struct fim_sdios_t {
	int fims;
	struct fim_sdio_t *ports;
};


static struct fim_sdios_t *fim_sdios;


/* Internal functions */
static void fim_sd_process_next(struct fim_sdio_t *port);
inline static struct fim_sdio_t *get_port_from_mmc(struct mmc_host *mmc);
inline static void *fim_sd_dma_to_sg(struct fim_sdio_t *port, struct mmc_data *data,
				     unsigned char *dma_buf, int dma_len);
static void fim_sd_set_clock(struct fim_sdio_t *port, long int clockrate);
static struct fim_buffer_t *fim_sd_alloc_cmd(struct fim_sdio_t *port);
static int fim_sd_send_command(struct fim_sdio_t *port, struct mmc_command *cmd);



/* Return the corresponding port structure */
inline static struct fim_sdio_t *get_port_from_mmc(struct mmc_host *mmc)
{
	if (!mmc)
		return NULL;

	return (struct fim_sdio_t *)mmc->private[0];
}


/* Timeout function called when a MMC-command doesn't return correctly */
static void fim_sd_cmd_timeout(unsigned long data)
{
	struct fim_sdio_t *port;

	port = (struct fim_sdio_t *)data;

	/* If the command pointer isn't NULL then a timeout has ocurred */
	spin_lock(&port->mmc_lock);
	if (port->mmc_cmd) {
		printk_err("A MMC-command timeout ocurred\n");
		port->mmc_cmd->error = -ETIMEDOUT;
		fim_sd_process_next(port);
	}	
	spin_unlock(&port->mmc_lock);
}


inline static int fim_sd_card_plugged(struct fim_sdio_t *port)
{
	struct fim_driver *fim;
	unsigned int val;
	
	fim = &port->fim;
	fim_get_stat_reg(fim, FIM_SD_CARD_STATREG, &val);
	return !val;
}



/*
 * Handler for the incoming FIM-interrupts. Available interrupts:
 * - Card detection (
 */
static void fim_sd_isr(struct fim_driver *driver, int irq, unsigned char code,
		       unsigned int rx_fifo)
{
	struct fim_sdio_t *port;
	
	port = driver->driver_data;

	switch (code) {
	case FIM_SD_INTARM_CARD_DETECTED:
		if (fim_sd_card_plugged(port)) {
			printk_debug("SD-card detected\n");
		} else {
			printk_debug("SD-card removed\n");
		}
		mmc_detect_change(port->mmc, msecs_to_jiffies(100));
		break;
	case FIM_SD_INTARM_CARD_DAT1:
		printk_err("SDIO-interrupt received, but not supported up now.\n");
		break;
	default:
		printk_err("Unknown IRQ %i | FIM %i | %x\n",
			   code, port->fim.picnr, rx_fifo);
		break;
	}
}



/*
 * This is the TX-callback that the API call after a DMA-package was closed
 * The fim buffer structure contains our internal private data
 * Free the allocated FIM-buffer that was used for sending the DMA-data
 */
static void fim_sd_tx_isr(struct fim_driver *driver, int irq,
			  struct fim_buffer_t *pdata)
{
	struct fim_buffer_t *buf;
	struct fim_sdio_t *port;

	port = (struct fim_sdio_t *)driver->driver_data;
	if (pdata->private) {
		buf = pdata->private;
		fim_free_buffer(&port->fim, buf);
	}
}

				 

/* @XXX: Remove this ugly code! */
inline static void fim_sd_parse_resp(struct mmc_command *cmd,
				     struct fim_sd_rx_resp_t *resp)
{
	unsigned char *ptr;
	ptr = (unsigned char *)cmd->resp;
	if (cmd->flags & MMC_RSP_136) {
		*ptr++ = resp->resp[3];
		*ptr++ = resp->resp[2];
		*ptr++ = resp->resp[1];
		*ptr++ = resp->resp[0];
		*ptr++ = resp->resp[7];
		*ptr++ = resp->resp[6];
		*ptr++ = resp->resp[5];
		*ptr++ = resp->resp[4];
		*ptr++ = resp->resp[11];
		*ptr++ = resp->resp[10];
		*ptr++ = resp->resp[9];
		*ptr++ = resp->resp[8];
		*ptr++ = resp->resp[15];
		*ptr++ = resp->resp[14];
		*ptr++ = resp->resp[13];
		*ptr++ = resp->resp[12];
	} else {
		*ptr++ = resp->resp[3];
		*ptr++ = resp->resp[2];
		*ptr++ = resp->resp[1];
		*ptr++ = resp->resp[0];
		*ptr++ = resp->resp[4];
		*ptr++ = resp->stat;
	}
}



/*
 * This function checks the CRC by block read transfer
 * The information about the length and content of the CRC was obtained
 * from the firmware-source code (sd.asm)
 */
inline static int fim_sd_check_blkrd_crc(struct fim_sdio_t *port, unsigned char *data,
					 int length)
{
	int crc_len;
	unsigned char *pic_crc;	
	
	/*
	 * The CRC length depends on the bus width (see sd.asm)
	 * No CRC enabled : One byte (0x00)
	 * One bit bus    : Four bytes
	 * Four bit bus   : Eight bytes 
	 */
	if (!(port->mmc_cmd->flags & MMC_RSP_CRC)) {
		crc_len = 1;
		pic_crc = data;
	} else if (port->mmc->ios.bus_width == MMC_BUS_WIDTH_1) {		
		crc_len = 4;
		pic_crc = data + 2;
	} else {
		crc_len = 16;
		pic_crc = data + 8;
	}

	if (crc_len != length) {
		printk_err("Unexpected CRC length %i (expected %i)\n",
		       length, crc_len);
		return -EINVAL;
	}


        /*
	 * Code for forcing a CRC-error and the behavior of the MMC-layer
	 * crc_error = 10 : Error reading the partition table
	 * crc_error = 40 : Error by a block read transfer
	 */
#ifdef FIM_SD_FORCE_CRC
	static int crc_error = 0;
	if (crc_error == 40) {
		crc_error++;
		return 1;
	} else
		crc_error++;
#endif

	/* If the CRC is disabled, the PIC only appended a dummy Byte */
	if (crc_len == 1)
		return 0;

	return memcmp(data, pic_crc, crc_len >> 1);
}


inline static void fim_sd_print_crc(int length, unsigned char *crc)
{	
	/* Only four and six as length supported */
	if (length == 4)
		printk_info("CRC: %u\n", *((unsigned int *)crc));
	else
		printk_info("CRC: %lu\n", *((unsigned long *)crc));
}


/*
 * Called when a receive DMA-buffer was closed.
 * Unfortunately the data received from the PIC has different formats. Sometimes it
 * contains a response, sometimes data of a block read request and sometimes the CRC
 * of the read data. In the case of a read transfer it is really amazing, then
 * the transfer consists in four DMA-buffers.
 */
static void fim_sd_rx_isr(struct fim_driver *driver, int irq,
			  struct fim_buffer_t *pdata)
{
	struct fim_sdio_t *port;
	struct mmc_command *mmc_cmd;
	struct fim_sd_rx_resp_t *resp;
	int len, crc_len;
	unsigned char *crc_ptr;
	int is_ack;
	
	/* Get the correct port from the FIM-driver structure */
	len = pdata->length;
	port = (struct fim_sdio_t *)driver->driver_data;
	spin_lock(&port->mmc_lock);

	/*
	 * The timeout function can set the command structure to NULL, for this reason
	 * check here is we can handle the response correctly
	 */
	if ((mmc_cmd = port->mmc_cmd) == NULL) {
		printk_err("Timeouted command response?\n");
		goto exit_unlock;
	}
		
	/*
	 * Check the current state of the command and update it if required
	 * IMPORTANT: The buffer can contain response data or the data from a block
	 * read too, for this reason was implemented the state machine
	 */
	resp = (struct fim_sd_rx_resp_t *)pdata->data;
	is_ack = (pdata->length == 1) ? 1 : 0;
	
	printk_debug("CMD%i | PIC stat %x | CMD stat %i | BLKRD stat %i | Len %i\n",
	       mmc_cmd->opcode, resp->stat, port->cmd_state,
	       port->blkrd_state, pdata->length);

	/*
	 * By the ACKs the PIC will NOT send a timeout. Timeouts are only
	 * set by the response and and block read data
	 */
	if (is_ack && resp->stat & FIM_SD_RX_TIMEOUT) {
		mmc_cmd->error = -ETIMEDOUT;
		port->blkrd_state = BLKRD_STATE_HAVE_DATA;
		port->cmd_state = CMD_STATE_HAVE_RSP;

		/* Check the conditions for the BLOCK READ state machine */
	} else if (port->blkrd_state == BLKRD_STATE_WAIT_ACK && is_ack &&
		   resp->stat & FIM_SD_RX_BLKRD) {
		port->blkrd_state = BLKRD_STATE_WAIT_DATA;
		
		/* Check if the block read data has arrived */
	} else if (port->blkrd_state == BLKRD_STATE_WAIT_DATA && !is_ack) {
		crc_len = len - mmc_cmd->data->blksz;
		crc_ptr = pdata->data + mmc_cmd->data->blksz;
		port->blkrd_state = BLKRD_STATE_HAVE_DATA;
		
		if (fim_sd_check_blkrd_crc(port, crc_ptr, crc_len)) {
			mmc_cmd->error = -EILSEQ;
		} else {
			/* fim_print_fifo_status(&port->fim); */
			fim_sd_dma_to_sg(port, mmc_cmd->data,
					 pdata->data, pdata->length);
 		}

		/* Check if we have a multiple transfer read */
		port->trans_blocks -= 1;
		if (port->trans_blocks > 0)
			port->blkrd_state = BLKRD_STATE_WAIT_DATA;
		
		/* Check the conditions for the COMMAND state machine */
	} else if (is_ack && port->cmd_state == CMD_STATE_WAIT_ACK &&
		   resp->stat & FIM_SD_RX_RSP) {
		port->cmd_state = CMD_STATE_WAIT_DATA;
	} else if (!is_ack && port->cmd_state == CMD_STATE_WAIT_DATA) {
		fim_sd_parse_resp(mmc_cmd, resp);
		port->cmd_state = CMD_STATE_HAVE_RSP;

		/* Check for unexpected acks or opcodes */
	} else {
		
		/* @FIXME: Need a correct errror handling for this condition */
		printk_err("Unexpected RX stat (CMD%i | PIC stat %x | Length %i)\n",
		       mmc_cmd->opcode, resp->stat, pdata->length);
	}

	/*
	 * By errors set the two states machines to the end position for sending
	 * the error to the MMC-layer
	 */
	if (mmc_cmd->error) {
		port->cmd_state = CMD_STATE_HAVE_RSP;
		port->blkrd_state = BLKRD_STATE_HAVE_DATA;
	}
	
	/*
	 * Now evaluate if need to wait for another RX-interrupt or
	 * can send the request done to the MMC-layer
	 */
	if (port->cmd_state == CMD_STATE_HAVE_RSP &&
	    port->blkrd_state == BLKRD_STATE_HAVE_DATA)
		fim_sd_process_next(port);

 exit_unlock:
	spin_unlock(&port->mmc_lock);
}


/* Send a buffer over the FIM-API */
static int fim_sd_send_buffer(struct fim_sdio_t *port, struct fim_buffer_t *buf)
{
	struct fim_driver *fim;
	int retval;

	if (!buf || !port)
		return -EINVAL;
	
	fim = &port->fim;
	buf->private = buf;

	if ((retval = fim_send_buffer(fim, buf)))
		printk_err("FIM send buffer request failed.\n");

	return retval;
}


/* Returns a command buffer allocated from the FIM-API */
static struct fim_buffer_t *fim_sd_alloc_cmd(struct fim_sdio_t *port)
{
	struct fim_driver *fim;
	int length;

	if(!port)
		return NULL;
	
	fim = &port->fim;
	length = sizeof(struct fim_sd_tx_cmd_t);
	
	return fim_alloc_buffer(fim, length, GFP_KERNEL);
}


/* Returns a buffer allocated from the FIM-API */
static struct fim_buffer_t *fim_sd_alloc_buffer(struct fim_sdio_t *port, int length)
{
	struct fim_driver *fim;

	if (!port || length <= 0)
		return NULL;
	
	fim = &port->fim;
	return fim_alloc_buffer(fim, length, GFP_KERNEL);
}


static void fim_sd_free_buffer(struct fim_sdio_t *port, struct fim_buffer_t *buf)
{
	struct fim_driver *fim;

	if (!port || !buf)
		return;
	
	fim = &port->fim;
	fim_free_buffer(fim, buf);
}


/*
 * Copy the data from the MMC-layer (scatter list) to the DMA-buffer for the FIM-API
 * Since we have only support for single block reads, some sanity checks
 * are not implemented
 */
inline static void fim_sd_sg_to_dma(struct fim_sdio_t *port, struct mmc_data *data,
				   struct fim_buffer_t *buf)
{
	unsigned int len, cnt, dma_len;
	struct scatterlist *sg;
	unsigned char *sg_buf;
	unsigned char *dma_buf;
	int process;
	
	sg = data->sg;
	len = data->sg_len;
	dma_buf = buf->data;
	dma_len = 0;

	/* Need a correct error handling */
	if (len > 1) {
		printk_err("The FIM-SD host only supports single block\n");
		len = 1;
	}
	
	/* @XXX: Check the correctness of the memcpy operation */
	for (cnt = 0; cnt < len && dma_len <= buf->length; cnt++) {
                sg_buf = sg_virt(&sg[cnt]);
		process = (buf->length >= sg[cnt].length) ? sg[cnt].length : buf->length;
                memcpy(dma_buf, sg_buf, process);
		dma_buf += process;
		dma_len += process;
		data->bytes_xfered += process;
	}
}


/*
 * Function called when RD data has arrived
 * Return value is the pointer of the last byte copied to the scatterlist, it can
 * be used for appending more data (e.g. in multiple block read transfers)
 */
inline static void *fim_sd_dma_to_sg(struct fim_sdio_t *port, struct mmc_data *data,
				     unsigned char *dma_buf, int dma_len)
{
	unsigned int len, cnt, process;
	struct scatterlist *sg;
	char *sg_buf;
	
	sg = data->sg;
	len = data->sg_len;

        /* Need a correct error handling */
#if !defined(FIM_SD_MULTI_BLOCK)
	if (len > 1) {
		printk_err("The FIM-SD host driver only supports single block\n");
		len = 1;
	}
#endif
	
	/* This loop was tested only with single block transfers */
	sg_buf = NULL;
	for (cnt = port->trans_sg; cnt < len && dma_len > 0; cnt++) {
		process = dma_len > sg[cnt].length ? sg[cnt].length : dma_len;
		sg_buf = sg_virt(&sg[cnt]);
                memcpy(sg_buf, dma_buf, process);
		dma_buf += process;
		dma_len -= process;
		data->bytes_xfered += process;
		sg_buf += process;
		port->trans_sg += 1;
	}
	
	return sg_buf;
}


/* This function will send the command to the PIC using the TX-DMA buffers */
static int fim_sd_send_command(struct fim_sdio_t *port, struct mmc_command *cmd)
{
	struct mmc_data *data;
	struct fim_buffer_t *buf;
	struct fim_sd_tx_cmd_t *txcmd;
	unsigned int block_length, blocks;
	int retval, length;
	
	/* @TODO: Send an error response to the MMC-core */
	if (!(buf = fim_sd_alloc_cmd(port))) {
		printk_err("No memory available for a new CMD?\n");
		return -ENOMEM;
	}

	/* Use the buffer data for the TX-command */
	txcmd = (struct fim_sd_tx_cmd_t *)buf->data;
	txcmd->opctl = 0;
	
	/*
	 * Set the internal flags for the next response sequences
	 * Assume that we will wait for a command response (not block read).
	 * By block reads the flag will be modified inside the if-condition
	 */
	port->cmd_state = CMD_STATE_WAIT_ACK;
	port->blkrd_state = BLKRD_STATE_HAVE_DATA;
	if ((data = cmd->data) != NULL) {
		block_length = data->blksz;
		blocks = data->blocks;

#if !defined(FIM_SD_MULTI_BLOCK)
		if (blocks != 1) {
			printk_err("Only supports single block transfer (%i)\n", blocks);
			cmd->error = -EILSEQ;
			fim_sd_process_next(port);
			return -EILSEQ;
		}
#endif

		printk_debug("New block transfer with %i blocks\n", blocks);

		/* Reset the scatter list position */
		port->trans_sg = 0;
		port->trans_blocks = blocks;
		fim_set_ctrl_reg(&port->fim, 2, blocks);
		
		/* Check if the transfer request is for reading or writing */
		if (cmd->data->flags & MMC_DATA_READ) {
			txcmd->opctl |= SDIO_FIFO_TX_BLKRD;
			port->blkrd_state = BLKRD_STATE_WAIT_ACK;
		} else
			txcmd->opctl |= SDIO_FIFO_TX_BLKWR;
	} else {
		block_length = 0;
		blocks = 0;
	}

	/* Set the correct expected response length */
	if (cmd->flags & MMC_RSP_136)
		txcmd->opctl |= SDIO_FIFO_TX_136RSP;
	else
		txcmd->opctl |= SDIO_FIFO_TX_48RSP;

	/* Set the correct CRC configuration */
	if (!(cmd->flags & MMC_RSP_CRC)) {
		printk_debug("CRC is disabled\n");
		txcmd->opctl |= SDIO_FIFO_TX_DISCRC;		
	}

	/* Set the correct bus width */
	if (port->mmc->ios.bus_width == MMC_BUS_WIDTH_4) {
		printk_debug("Bus width has four bits\n");
		txcmd->opctl |= SDIO_FIFO_TX_BW4;
	}

	txcmd->blksz_msb = (block_length >> 8);
	txcmd->blksz_lsb =  block_length;
	txcmd->cmd[0] = SDIO_HOST_TX_HDR | (cmd->opcode & SDIO_HOST_CMD_MASK);
	txcmd->cmd[1] = cmd->arg >> 24;
	txcmd->cmd[2] = cmd->arg >> 16;
	txcmd->cmd[3] = cmd->arg >> 8;
	txcmd->cmd[4] = cmd->arg;

	/*
	 * Store the private data for the callback function
	 * If an error ocurrs when sending the buffer, the timeout function will
	 * send the error to the MMC-layer
	 */
	port->buf = buf;
	port->mmc_cmd = cmd;
	buf->private = port;
	mod_timer(&port->mmc_timer, jiffies + msecs_to_jiffies(FIM_SD_TIMEOUT_MS));
	if ((retval = fim_sd_send_buffer(port, buf))) {
		printk_err("MMC command %i (err %i)\n", cmd->opcode,
			   retval);
		fim_sd_free_buffer(port, buf);
		goto exit_ok;
	}

	/*
	 * If we have a write command then fill a next buffer and send it
	 * @TODO: We need here an error handling, then otherwise we have started a
	 * WR-transfer but have no transfer data (perhaps not too critical?)
	 */
	if (data && data->flags & MMC_DATA_WRITE) {
		length = data->blksz * data->blocks;
		if (!(buf = fim_sd_alloc_buffer(port, length))) {
			printk_err("Buffer alloc BLKWR failed, %i\n", length);
			goto exit_ok;
		}

		buf->private = port;
		fim_sd_sg_to_dma(port, data, buf);
		if ((retval = fim_sd_send_buffer(port, buf))) {
			printk_err("Send BLKWR-buffer failed, %i\n", retval);
			fim_sd_free_buffer(port, buf);
		}
	}
	
 exit_ok:
	return 0;
}



/*
 * This function will be called from the request function and from the ISR callback
 * when a command was executed
 * In some cases we don't need to pass the command response to the Linux-layer (e.g.
 * by the configuration of the bus width)
 */
static void fim_sd_process_next(struct fim_sdio_t *port)
{
	if (port->flags == FIM_SD_REQUEST_NEW) {
		port->flags = FIM_SD_REQUEST_CMD;
		fim_sd_send_command(port, port->mmc_req->cmd);
	} else if ((!(port->flags & FIM_SD_REQUEST_STOP)) && port->mmc_req->stop) {
		port->flags = FIM_SD_REQUEST_STOP;
		fim_sd_send_command(port, port->mmc_req->stop);
	} else {
		/* By timeouts the core might retry sending another command */
		port->mmc_cmd = NULL;
		mmc_request_done(port->mmc, port->mmc_req);
	}
}



/*
 * Called for processing three main request types:
 * command : Submit the command to the PIC and returns inmediately
 * stop    : Request for stopping an already started request?
 * data    : For data transfer
 */
static void fim_sd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct fim_sdio_t *port;

	if (!(port = get_port_from_mmc(mmc)))
		return;

	/*
	 * Wait if the timeout function is running or the RX-callback is active
	 */
	spin_lock(&port->mmc_lock);
	port->mmc_req = mrq;
	port->flags = FIM_SD_REQUEST_NEW;	
	fim_sd_process_next(port);
	spin_unlock(&port->mmc_lock);
}



/* Set the transfer clock using the pre-defined values */
static void fim_sd_set_clock(struct fim_sdio_t *port, long int clockrate)
{
	unsigned int clkdiv;

	switch (clockrate) {
	case 0:
		printk(KERN_DEBUG "@TODO: Disable the clock for 0Hz\n");
		clkdiv = 0;
		clockrate = -1;
		break;
	case 320000:
		clkdiv = 0x70;
		break;
	case 25000000:
		clkdiv = 0x04;
		break;
	default:
		printk_err("Unknow clock rate, %ld\n", clockrate);
		clockrate = -1;
		break;
	}

	if (clockrate >= 0) {
		printk_debug("Setting the clock to %ld (%x)\n",
			     clockrate, clkdiv);
		fim_set_ctrl_reg(&port->fim, FIM_SD_REG_CLOCK_DIVISOR, clkdiv);
	}
}


/*
 * Called by the core system for setting the available modes and clock speed
 * 
 */
static void fim_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct fim_sdio_t *port;
	
	if (!(port = get_port_from_mmc(mmc)))
		return;

	/*
	 * The FIM-board doesn't have a power switch for the card, but probably the
	 * next revision will include it, so that we can control the switch from here
	 */
	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		/* fim_set_ctrl_reg(&port->fim, FIM_SD_REG_INTERRUPT, 1); */
		break;
	case MMC_POWER_UP:
		fim_set_ctrl_reg(&port->fim, FIM_SD_REG_INTERRUPT, 1);
		break;
	case MMC_POWER_ON:
		fim_set_ctrl_reg(&port->fim, FIM_SD_REG_INTERRUPT, 1);
		break;
	}

	fim_sd_set_clock(port, ios->clock);
}


/*
 * Return the read only status of the plugged card
 * Since the FIM-firmware doesn't include this GPIO, we will read it at this point
 */
static int fim_sd_get_ro(struct mmc_host *mmc)
{
	struct fim_sdio_t *port;

	if (!(port = get_port_from_mmc(mmc))) {
		printk_err("No FIM-port by a registered MMC-host?\n");
		return -ENODEV;
	}

	return gpio_get_value_ns921x(port->wp_gpio);
}


static void fim_sd_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct fim_sdio_t *port;
	
	printk_debug("Enabling the IRQ (%i)\n", enable);

	if (!(port = get_port_from_mmc(mmc)))
		return;
	
	if (enable)
		fim_set_ctrl_reg(&port->fim, FIM_SD_REG_INTERRUPT, 1);
	else
		fim_set_ctrl_reg(&port->fim, FIM_SD_REG_INTERRUPT, 0);
}


/* Available driver host operations */
static const struct mmc_host_ops fim_sd_ops = {
	.request = fim_sd_request,
	.set_ios = fim_sd_set_ios,
	.get_ro  = fim_sd_get_ro,
	.enable_sdio_irq = fim_sd_enable_sdio_irq,
};


/* Called normally only when exiting the driver */
static int fim_sdio_unregister_port(struct fim_sdio_t *port)
{
	int cnt;
	
	if (!port || !port->reg)
		return -ENODEV;

	printk_info("Removing the FIM MMC host %i\n", port->index);
	mmc_remove_host(port->mmc);
	mmc_free_host(port->mmc);

	del_timer_sync(&port->mmc_timer);
		
	fim_unregister_driver(&port->fim);

	for (cnt=0; cnt < FIM_SDIO_MAX_GPIOS; cnt++) {
		printk_debug("Freeing GPIO %i\n", port->gpios[cnt].nr);
		if (port->gpios[cnt].nr == FIM_LAST_GPIO)
			break;		
		else if (port->gpios[cnt].nr == FIM_GPIO_DONT_USE)
			continue;
		else
			gpio_free(port->gpios[cnt].nr);
	}

	port->reg = 0;
	return 0;
}



/* Register the new FIM driver by the FIM-API */
static int fim_sdio_register_port(struct device *dev, struct fim_sdio_t *port,
				  int picnr, struct fim_gpio_t gpios[])
{
	int retval;
	int cnt, func;
	struct fim_dma_cfg_t dma_cfg;

	/* Specific DMA configuration for the SD-host driver */
	dma_cfg.rxnr = FIM_SD_DMA_RX_BUFFERS;
	dma_cfg.txnr = FIM_SD_DMA_TX_BUFFERS;
	dma_cfg.rxsz = FIM_SD_DMA_BUFFER_SIZE;
	dma_cfg.txsz = FIM_SD_DMA_BUFFER_SIZE;

	port->index = picnr;
	port->fim.picnr = picnr;
	port->fim.driver.name = FIM_SDIO_DRIVER_NAME;
	port->fim.driver_data = port;
	port->fim.fim_isr = fim_sd_isr;
	port->fim.dma_tx_isr = fim_sd_tx_isr;
	port->fim.dma_rx_isr = fim_sd_rx_isr;
	port->fim.driver_data = port;
	port->fim.dma_cfg = &dma_cfg;

	/* Check if have a firmware code for using to */
	port->fim.fw_name = FIM_SDIO_FIRMWARE_FILE;
	port->fim.fw_code = FIM_SDIO_FIRMWARE_CODE;
	retval = fim_register_driver(&port->fim);
	if (retval) {
		printk_err("Couldn't register the FIM driver.\n");
		return retval;
	}

	/* Request the corresponding GPIOs (@XXX: Check the returned values) */
	for (cnt=0; cnt < FIM_SDIO_MAX_GPIOS; cnt++) {

		if (gpios[cnt].nr == FIM_LAST_GPIO)
			break;
		
		if (gpios[cnt].nr == FIM_GPIO_DONT_USE)
			continue;

		printk_debug("Requesting the GPIO %i (Function %i)\n",
			     gpios[cnt].nr, gpios[cnt].func);
		retval = gpio_request(gpios[cnt].nr, gpios[cnt].name);
		if (!retval) {

			func = gpios[cnt].func;
			
			/*
			 * Since the FIM-firmware doesn't support the WP-GPIO, we will
			 * read the status of the GPIO inside the driver. For this reason
			 * configure the WP-line as GPIO (Function three)
			 */
			if (gpios[cnt].nr == port->wp_gpio)
				func = NS921X_GPIO_FUNC_3;
				
			gpio_configure_ns921x_unlocked(gpios[cnt].nr,
						       NS921X_GPIO_INPUT,
						       NS921X_GPIO_DONT_INVERT,
						       func,
						       NS921X_GPIO_DISABLE_PULLUP);
		} else {
			/* Free the already requested GPIOs */
			printk_err("Couldn't request the GPIO %i\n", gpios[cnt].nr);
			while (cnt) gpio_free(gpios[--cnt].nr);
			goto exit_unreg_fim;
		}
	}

	/* First init the internal data */
	init_MUTEX(&port->sem);
	spin_lock_init(&port->mmc_lock);
	
	/* Configure and init the timer for the command timeouts */
	init_timer(&port->mmc_timer);
	port->mmc_timer.function = fim_sd_cmd_timeout;
	port->mmc_timer.data = (unsigned long)port;
	
	port->mmc = mmc_alloc_host(sizeof(struct fim_sdio_t *), port->fim.dev);
	if (!port->mmc) {
		printk_err("Alloc MMC host by the FIM %i failed.\n", picnr);
		retval = -ENOMEM;
		goto exit_free_gpios;
	}

	/* These are the default values for this SD-host */
	port->mmc->ops = &fim_sd_ops;

	/* Supported physical properties of the FIM-host (see the PIC-firmware code) */
	port->mmc->f_min = 320000;
	port->mmc->f_max = 25000000;
	port->mmc->ocr_avail = MMC_VDD_33_34 | MMC_VDD_32_33;
	port->mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ;

	/* Maximum number of blocks in one req */
	port->mmc->max_blk_count = FIM_SD_MAX_BLOCKS;
	port->mmc->max_blk_size = FIM_SD_DMA_BUFFER_SIZE;
	/* The maximum per SG entry depends on the buffer size */
	port->mmc->max_seg_size = FIM_SD_DMA_BUFFER_SIZE;
	
	/* Save our port structure into the private pointer */
	port->mmc->private[0] = (unsigned long)port;
	retval = mmc_add_host(port->mmc);
	if (retval) {
		printk_err("Couldn't add the MMC host\n");
		goto exit_free_host;
	}

	memcpy(port->gpios, gpios, sizeof(struct fim_gpio_t) * FIM_SDIO_MAX_GPIOS);
	port->reg = 1;
	dev_set_drvdata(dev, port);
	
	/* And enable the FIM-interrupt */
	fim_enable_irq(&port->fim);
	return 0;

 exit_free_host:
	mmc_free_host(port->mmc);

 exit_free_gpios:
	for (cnt=0; gpios[cnt].nr < FIM_SDIO_MAX_GPIOS; cnt++) {
		if (gpios[cnt].nr == FIM_LAST_GPIO)
			break;
		if (gpios[cnt].nr != FIM_GPIO_DONT_USE)
			gpio_free(gpios[cnt].nr);
	}
	
 exit_unreg_fim:
	fim_unregister_driver(&port->fim);

	return retval;
}



static int __devinit fim_sdio_probe(struct platform_device *pdev)
{
	struct fim_sdio_t *port;
	struct fim_sdio_platform_data *pdata;
	struct fim_gpio_t gpios[FIM_SDIO_MAX_GPIOS];
	int retval;

	printk_debug("Probing a new device with ID %i\n", pdev->id);
	
	pdata = pdev->dev.platform_data;
	if (!pdata)
		return -ENXIO;

	if (pdata->fim_nr < 0 || pdata->fim_nr >= fim_sdios->fims) {
		printk_err("Invalid device FIM number %i\n", pdata->fim_nr);
		return -ENODEV;
	}

	port = fim_sdios->ports + pdata->fim_nr;

	/* Get the GPIOs-table from the platform data structure */
	gpios[0].nr   = pdata->d0_gpio_nr;
	gpios[0].func = pdata->d0_gpio_func;
	gpios[1].nr   = pdata->d1_gpio_nr;
	gpios[1].func = pdata->d1_gpio_func;
	gpios[2].nr   = pdata->d2_gpio_nr;
	gpios[2].func = pdata->d2_gpio_func;
	gpios[3].nr   = pdata->d3_gpio_nr;
	gpios[3].func = pdata->d3_gpio_func;
	gpios[4].nr   = pdata->wp_gpio_nr;
	gpios[4].func = pdata->wp_gpio_func;
	gpios[5].nr   = pdata->cd_gpio_nr;
	gpios[5].func = pdata->cd_gpio_func;
	gpios[6].nr   = pdata->clk_gpio_nr;
	gpios[6].func = pdata->clk_gpio_func;
	gpios[7].nr   = pdata->cmd_gpio_nr;
	gpios[7].func = pdata->cmd_gpio_func;
	port->wp_gpio = pdata->wp_gpio_nr;	
	
	retval = fim_sdio_register_port(&pdev->dev, port,
					pdata->fim_nr,
					gpios);
	
	return retval;
}


static int __devexit fim_sdio_remove(struct platform_device *pdev)
{
	struct fim_sdio_t *port;
	int retval;
	
	port = dev_get_drvdata(&pdev->dev);

	retval = fim_sdio_unregister_port(port);
	if (!retval)
		dev_set_drvdata(&pdev->dev, NULL);

	return retval;
}


static struct platform_driver fim_sdio_platform_driver = {
	.probe	= fim_sdio_probe,
	.remove	= __devexit_p(fim_sdio_remove),
	.driver	= {
		   .owner = THIS_MODULE,
		   .name  = FIM_SDIO_DRIVER_NAME,
	},
};



/*
 * This is the function that will be called when the module is loaded
 * into the kernel space
 */
static int __init fim_sdio_init(void)
{
	int retval;
	int nrpics;
	
	printk_debug("Starting the FIM SDIO driver.\n");	

	/* Get the number of available FIMs */
	nrpics = fim_number_pics();
	fim_sdios = kzalloc(sizeof(struct fim_sdios_t) +
			    (nrpics * sizeof(struct fim_sdio_t)), GFP_KERNEL);
	if (!fim_sdios)
		return -ENOMEM;

	fim_sdios->fims = nrpics;
	fim_sdios->ports = (void *)fim_sdios + sizeof(struct fim_sdios_t);	


	retval = platform_driver_register(&fim_sdio_platform_driver);
	if (retval)
		goto exit_free_ports;
	
	printk_info(DRIVER_DESC " v" DRIVER_VERSION "\n");
	return 0;
	
 exit_free_ports:
	kfree(fim_sdios);
	
	return retval;
}



/*
 * Free the requested resources (GPIOs, memory, drivers, etc.)
 * The following steps MUST be followed when unregistering the driver:
 * - First remove and free the MMC-host
 * - Unregister the FIM-driver (will free the DMA-channel)
 * - Free the GPIOs at last
 */
static void __exit fim_sdio_exit(void)
{
	printk_info("Removing the FIM SDIO driver\n");
	platform_driver_unregister(&fim_sdio_platform_driver);		
	kfree(fim_sdios);
}


module_init(fim_sdio_init);
module_exit(fim_sdio_exit);

