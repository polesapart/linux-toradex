/*
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mx_sdhci.c
 *
 * @brief Driver for the Freescale Semiconductor MXC eSDHC modules.
 *
 * This driver code is based on sdhci.c, by Pierre Ossman <drzeus@drzeus.cx>");
 * This driver supports Enhanced Secure Digital Host Controller
 * modules eSDHC of MXC. eSDHC is also referred as enhanced MMC/SD
 * controller.
 *
 * @ingroup MMC_SD
 */

#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>

#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/clk.h>
#include <linux/regulator/regulator.h>

#include <asm/dma.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>
#include <asm/arch/mmc.h>
#ifdef CONFIG_MACH_MX35_3DS
#include <asm/arch/pmic_external.h>
#endif

#include "mx_sdhci.h"

#define DRIVER_NAME "mxsdhci"

#define DBG(f, x...) \
	pr_debug(DRIVER_NAME " [%s()]: " f, __func__, ## x)

/*
 * Different quirks to handle when the hardware deviates from a strict
 * interpretation of the SDHCI specification.
 */

/* Controller doesn't honor resets unless we touch the clock register */
#define SDHCI_QUIRK_CLOCK_BEFORE_RESET			(1<<0)
/* Controller has bad caps bits, but really supports DMA */
#define SDHCI_QUIRK_FORCE_DMA				(1<<1)
/* Controller doesn't like some resets when there is no card inserted. */
#define SDHCI_QUIRK_NO_CARD_NO_RESET			(1<<2)
/* Controller doesn't like clearing the power reg before a change */
#define SDHCI_QUIRK_SINGLE_POWER_WRITE			(1<<3)
/* Controller has flaky internal state so reset it on each ios change */
#define SDHCI_QUIRK_RESET_CMD_DATA_ON_IOS		(1<<4)
/* Controller has an unusable DMA engine */
#define SDHCI_QUIRK_BROKEN_DMA				(1<<5)
/* Controller can only DMA from 32-bit aligned addresses */
#define SDHCI_QUIRK_32BIT_DMA_ADDR			(1<<6)
/* Controller can only DMA chunk sizes that are a multiple of 32 bits */
#define SDHCI_QUIRK_32BIT_DMA_SIZE			(1<<7)
/* Controller needs to be reset after each request to stay stable */
#define SDHCI_QUIRK_RESET_AFTER_REQUEST			(1<<8)
/* Controller only support the PIO */
#define SDHCI_QUIRK_ONLY_PIO 				(1<<9)
/* Controller support the External DMA */
#define SDHCI_QUIRK_EXTERNAL_DMA_MODE			(1<<10)
/* Controller support the Internal Simple DMA */
#define SDHCI_QUIRK_INTERNAL_SIMPLE_DMA			(1<<11)
/* Controller support the Internal Advanced DMA */
#define SDHCI_QUIRK_INTERNAL_ADVANCED_DMA 		(1<<12)

/*
 * defines the mxc flags refer to the special hw pre-conditons and behavior
 */
static unsigned int mxc_quirks;
static unsigned int debug_quirks = SDHCI_QUIRK_ONLY_PIO;

static void sdhci_prepare_data(struct sdhci_host *, struct mmc_data *);
static void sdhci_finish_data(struct sdhci_host *);

static void sdhci_send_command(struct sdhci_host *, struct mmc_command *);
static void sdhci_finish_command(struct sdhci_host *);

/* Used to active the SD bus */
extern void gpio_sdhc_active(int module);
extern void gpio_sdhc_inactive(int module);
static void sdhci_dma_irq(void *devid, int error, unsigned int cnt);

#ifdef CONFIG_MACH_MX35_3DS
static pmic_event_callback_t sdhci_pmic_cd_callback;
static pmic_event_callback_t sdhci_pmic_wp_callback;
#endif
static void sdhci_dumpregs(struct sdhci_host *host)
{
	printk(KERN_DEBUG DRIVER_NAME
	       ": ============== REGISTER DUMP ==============\n");

	printk(KERN_DEBUG DRIVER_NAME ": Sys addr: 0x%08x | Version:  0x%08x\n",
	       readl(host->ioaddr + SDHCI_DMA_ADDRESS),
	       readl(host->ioaddr + SDHCI_HOST_VERSION));
	printk(KERN_DEBUG DRIVER_NAME ": Blk size: 0x%08x | Blk cnt:  0x%08x\n",
	       (readl(host->ioaddr + SDHCI_BLOCK_SIZE) & 0xFFFF),
	       (readl(host->ioaddr + SDHCI_BLOCK_COUNT) >> 16));
	printk(KERN_DEBUG DRIVER_NAME ": Argument: 0x%08x | Trn mode: 0x%08x\n",
	       readl(host->ioaddr + SDHCI_ARGUMENT),
	       readl(host->ioaddr + SDHCI_TRANSFER_MODE));
	printk(KERN_DEBUG DRIVER_NAME ": Present:  0x%08x | Host ctl: 0x%08x\n",
	       readl(host->ioaddr + SDHCI_PRESENT_STATE),
	       readl(host->ioaddr + SDHCI_HOST_CONTROL));
	printk(KERN_DEBUG DRIVER_NAME ": Clock:    0x%08x\n",
	       readl(host->ioaddr + SDHCI_CLOCK_CONTROL));
	printk(KERN_DEBUG DRIVER_NAME ": Int stat: 0x%08x\n",
	       readl(host->ioaddr + SDHCI_INT_STATUS));
	printk(KERN_DEBUG DRIVER_NAME ": Int enab: 0x%08x | Sig enab: 0x%08x\n",
	       readl(host->ioaddr + SDHCI_INT_ENABLE),
	       readl(host->ioaddr + SDHCI_SIGNAL_ENABLE));
	printk(KERN_DEBUG DRIVER_NAME ": Caps:     0x%08x\n",
	       readl(host->ioaddr + SDHCI_CAPABILITIES));

	printk(KERN_DEBUG DRIVER_NAME
	       ": ===========================================\n");
}

/*****************************************************************************\
 *                                                                           *
 * Low level functions                                                       *
 *                                                                           *
\*****************************************************************************/

static void sdhci_reset(struct sdhci_host *host, u8 mask)
{
	unsigned long timeout;
	unsigned long mask_u32 = mask;

	if (host->chip->quirks & SDHCI_QUIRK_NO_CARD_NO_RESET) {
		if (!(readl(host->ioaddr + SDHCI_PRESENT_STATE) &
		      SDHCI_CARD_PRESENT))
			return;
	}

	mask_u32 = readl(host->ioaddr + SDHCI_CLOCK_CONTROL) | (mask_u32 << 24);
	writel(mask_u32, host->ioaddr + SDHCI_CLOCK_CONTROL);

	if (mask & SDHCI_RESET_ALL)
		host->clock = 0;

	/* Wait max 100 ms */
	timeout = 100;

	/* hw clears the bit when it's done */
	while ((readl(host->ioaddr + SDHCI_CLOCK_CONTROL) >> 24) & mask) {
		if (timeout == 0) {
			printk(KERN_ERR "%s: Reset 0x%x never completed.\n",
			       mmc_hostname(host->mmc), (int)mask);
			sdhci_dumpregs(host);
			return;
		}
		timeout--;
		mdelay(1);
	}
	/*
	 * The INT_EN SIG_EN regs have been modified after reset.
	 * re-configure them ag.
	 */
	mask_u32 = SDHCI_INT_DMAERR |
	    SDHCI_INT_DATA_END_BIT | SDHCI_INT_DATA_CRC |
	    SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_INDEX |
	    SDHCI_INT_END_BIT | SDHCI_INT_CRC | SDHCI_INT_TIMEOUT |
	    SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL |
	    SDHCI_INT_DMA_END | SDHCI_INT_DATA_END | SDHCI_INT_RESPONSE;

	if (host->flags & SDHCI_USE_DMA)
		mask_u32 &= ~(SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL);
	writel(mask_u32, host->ioaddr + SDHCI_INT_ENABLE);
	writel(mask_u32, host->ioaddr + SDHCI_SIGNAL_ENABLE);
}

static void sdhci_init(struct sdhci_host *host)
{
	u32 intmask;

	sdhci_reset(host, SDHCI_RESET_ALL);

	intmask = SDHCI_INT_DMAERR |
	    SDHCI_INT_DATA_END_BIT | SDHCI_INT_DATA_CRC |
	    SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_INDEX |
	    SDHCI_INT_END_BIT | SDHCI_INT_CRC | SDHCI_INT_TIMEOUT |
	    SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL |
	    SDHCI_INT_DMA_END | SDHCI_INT_DATA_END | SDHCI_INT_RESPONSE;

	if (host->flags & SDHCI_USE_DMA)
		intmask &= ~(SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL);
	/* Configure the WML rege */
	writel(SDHCI_WML_4_WORDS, host->ioaddr + SDHCI_WML);
	writel(intmask, host->ioaddr + SDHCI_INT_ENABLE);
	writel(intmask, host->ioaddr + SDHCI_SIGNAL_ENABLE);
}

static void sdhci_activate_led(struct sdhci_host *host)
{
	u32 ctrl;

	ctrl = readl(host->ioaddr + SDHCI_HOST_CONTROL);
	ctrl |= SDHCI_CTRL_LED;
	writel(ctrl, host->ioaddr + SDHCI_HOST_CONTROL);
}

static void sdhci_deactivate_led(struct sdhci_host *host)
{
	u32 ctrl;

	ctrl = readl(host->ioaddr + SDHCI_HOST_CONTROL);
	ctrl &= ~SDHCI_CTRL_LED;
	writel(ctrl, host->ioaddr + SDHCI_HOST_CONTROL);
}

/*****************************************************************************\
 *                                                                           *
 * Core functions                                                            *
 *                                                                           *
\*****************************************************************************/

static inline char *sdhci_sg_to_buffer(struct sdhci_host *host)
{
	return sg_virt(host->cur_sg);
}

static inline int sdhci_next_sg(struct sdhci_host *host)
{
	/*
	 * Skip to next SG entry.
	 */
	host->cur_sg++;
	host->num_sg--;

	/*
	 * Any entries left?
	 */
	if (host->num_sg > 0) {
		host->offset = 0;
		host->remain = host->cur_sg->length;
	}

	return host->num_sg;
}

static void sdhci_read_block_pio(struct sdhci_host *host)
{
	int blksize, chunk_remain;
	u32 data;
	char *buffer;
	int size;

	DBG("PIO reading\n");
	udelay(100);		/* 100 us delay is needed */

	blksize = host->data->blksz;
	chunk_remain = 0;
	data = 0;

	buffer = sdhci_sg_to_buffer(host) + host->offset;

	while (blksize) {
		if (chunk_remain == 0) {
			data = readl(host->ioaddr + SDHCI_BUFFER);
			chunk_remain = min(blksize, 4);
		}

		size = min(host->remain, chunk_remain);

		chunk_remain -= size;
		blksize -= size;
		host->offset += size;
		host->remain -= size;

		while (size) {
			*buffer = data & 0xFF;
			buffer++;
			data >>= 8;
			size--;
		}

		if (host->remain == 0) {
			if (sdhci_next_sg(host) == 0) {
				BUG_ON(blksize != 0);
				return;
			}
			buffer = sdhci_sg_to_buffer(host);
		}
	}
}

static void sdhci_write_block_pio(struct sdhci_host *host)
{
	int blksize, chunk_remain;
	u32 data;
	char *buffer;
	int bytes, size;

	DBG("PIO writing\n");

	blksize = host->data->blksz;
	chunk_remain = 4;
	data = 0;

	bytes = 0;
	buffer = sdhci_sg_to_buffer(host) + host->offset;

	while (blksize) {
		size = min(host->remain, chunk_remain);

		chunk_remain -= size;
		blksize -= size;
		host->offset += size;
		host->remain -= size;

		while (size) {
			data >>= 8;
			data |= (u32) *buffer << 24;
			buffer++;
			size--;
		}

		if (chunk_remain == 0) {
			writel(data, host->ioaddr + SDHCI_BUFFER);
			chunk_remain = min(blksize, 4);
		}

		if (host->remain == 0) {
			if (sdhci_next_sg(host) == 0) {
				BUG_ON(blksize != 0);
				return;
			}
			buffer = sdhci_sg_to_buffer(host);
		}
	}
}

static void sdhci_transfer_pio(struct sdhci_host *host)
{
	u32 mask;

	BUG_ON(!host->data);

	if (host->num_sg == 0)
		return;

	if (host->data->flags & MMC_DATA_READ)
		mask = SDHCI_DATA_AVAILABLE;
	else
		mask = SDHCI_SPACE_AVAILABLE;

	while (readl(host->ioaddr + SDHCI_PRESENT_STATE) & mask) {
		if (host->data->flags & MMC_DATA_READ)
			sdhci_read_block_pio(host);
		else
			sdhci_write_block_pio(host);

		if (host->num_sg == 0)
			break;
	}

	DBG("PIO transfer complete.\n");
}

static void sdhci_prepare_data(struct sdhci_host *host, struct mmc_data *data)
{
	u32 count;
	unsigned target_timeout, current_timeout;

	WARN_ON(host->data);

	if (data == NULL)
		return;

	/* Sanity checks */
	BUG_ON(data->blksz * data->blocks > 524288);
	BUG_ON(data->blksz > host->mmc->max_blk_size);
	BUG_ON(data->blocks > 65535);

	host->data = data;
	host->data_early = 0;

	/* timeout in us */
	target_timeout = data->timeout_ns / 1000 +
	    data->timeout_clks / host->clock;

	/*
	 * Figure out needed cycles.
	 * We do this in steps in order to fit inside a 32 bit int.
	 * The first step is the minimum timeout, which will have a
	 * minimum resolution of 6 bits:
	 * (1) 2^13*1000 > 2^22,
	 * (2) host->timeout_clk < 2^16
	 *     =>
	 *     (1) / (2) > 2^6
	 */
	count = 0;
	current_timeout = (1 << 13) * 1000 / host->timeout_clk;
	while (current_timeout < target_timeout) {
		count++;
		current_timeout <<= 1;
		if (count >= 0xF)
			break;
	}

	if (count >= 0xF) {
		printk(KERN_WARNING "%s: Too large timeout requested!\n",
		       mmc_hostname(host->mmc));
		count = 0xE;
	}

	count =
	    (count << 16) | (readl(host->ioaddr + SDHCI_CLOCK_CONTROL) &
			     0xFFF0FFFF);
	writel(count, host->ioaddr + SDHCI_CLOCK_CONTROL);

	if (host->chip->quirks & SDHCI_QUIRK_EXTERNAL_DMA_MODE) {
		host->dma_size = data->blocks * data->blksz;
		DBG("%s:Request bytes to transfer:%d\n", DRIVER_NAME,
		    host->dma_size);
		if (host->dma_size <= (64 << host->mmc->ios.bus_width))
			goto NO_EX_DMA;

		if (data->blksz & 0x3) {
			printk(KERN_ERR
			       "mxc_mci: block size not multiple of 4 bytes\n");
		}

		if (data->flags & MMC_DATA_READ)
			host->dma_dir = DMA_FROM_DEVICE;
		else
			host->dma_dir = DMA_TO_DEVICE;

		host->dma_len = dma_map_sg(mmc_dev(host->mmc), data->sg,
					   data->sg_len, host->dma_dir);

		if (data->flags & MMC_DATA_READ) {
			mxc_dma_sg_config(host->dma, data->sg, data->sg_len,
					  host->dma_size, MXC_DMA_MODE_READ);
		} else {
			mxc_dma_sg_config(host->dma, data->sg, data->sg_len,
					  host->dma_size, MXC_DMA_MODE_WRITE);
		}
	}
NO_EX_DMA:

	if (host->flags & SDHCI_USE_DMA)
		host->flags |= SDHCI_REQ_USE_DMA;

	if (unlikely((host->flags & SDHCI_REQ_USE_DMA) &&
		     (host->chip->quirks & SDHCI_QUIRK_32BIT_DMA_SIZE) &&
		     ((data->blksz * data->blocks) & 0x3))) {
		DBG("Reverting to PIO because of transfer size (%d)\n",
		    data->blksz * data->blocks);
		host->flags &= ~SDHCI_REQ_USE_DMA;
	}

	/*
	 * The assumption here being that alignment is the same after
	 * translation to device address space.
	 */
	if (unlikely((host->flags & SDHCI_REQ_USE_DMA) &&
		     (host->chip->quirks & SDHCI_QUIRK_32BIT_DMA_ADDR) &&
		     (data->sg->offset & 0x3))) {
		DBG("Reverting to PIO because of bad alignment\n");
		host->flags &= ~SDHCI_REQ_USE_DMA;
	}

	if (host->flags & SDHCI_REQ_USE_DMA) {
		int count;

		count =
		    dma_map_sg(&(host->chip->pdev)->dev, data->sg, data->sg_len,
			       (data->
				flags & MMC_DATA_READ) ? DMA_FROM_DEVICE :
			       DMA_TO_DEVICE);
		BUG_ON(count != 1);

		writel(sg_dma_address(data->sg),
		       host->ioaddr + SDHCI_DMA_ADDRESS);
		DBG("configure the sg DMA, %s\n", (data->flags & MMC_DATA_READ)
		    ? "DMA_FROM_DEIVCE" : "DMA_TO_DEVICE");
	} else {
		host->cur_sg = data->sg;
		host->num_sg = data->sg_len;

		host->offset = 0;
		host->remain = host->cur_sg->length;
	}

	/* We do not handle DMA boundaries, so set it to max (512 KiB) */
	writel((data->blocks << 16) | SDHCI_MAKE_BLKSZ(7, data->blksz),
	       host->ioaddr + SDHCI_BLOCK_SIZE);
}

static void sdhci_finish_data(struct sdhci_host *host)
{
	struct mmc_data *data;
	u16 blocks;

	BUG_ON(!host->data);

	data = host->data;
	host->data = NULL;

	if (host->flags & SDHCI_REQ_USE_DMA) {
		dma_unmap_sg(&(host->chip->pdev)->dev, data->sg, data->sg_len,
			     (data->
			      flags & MMC_DATA_READ) ? DMA_FROM_DEVICE :
			     DMA_TO_DEVICE);
	}
	if (host->chip->quirks & SDHCI_QUIRK_EXTERNAL_DMA_MODE) {
		if (host->dma_size > (64 << host->mmc->ios.bus_width)) {
			dma_unmap_sg(mmc_dev(host->mmc), data->sg,
				     host->dma_len, host->dma_dir);
		}
	}

	/*
	 * Controller doesn't count down when in single block mode.
	 */
	if (data->blocks == 1)
		blocks = (data->error == 0) ? 0 : 1;
	else
		blocks = readl(host->ioaddr + SDHCI_BLOCK_COUNT) >> 16;
	data->bytes_xfered = data->blksz * (data->blocks - blocks);

	if (!data->error && blocks) {
		printk(KERN_ERR "%s: Controller signalled completion even "
		       "though there were blocks left.\n",
		       mmc_hostname(host->mmc));
		data->error = -EIO;
	}

	if (data->stop) {
		/*
		 * The controller needs a reset of internal state machines
		 * upon error conditions.
		 */
		if (data->error) {
			sdhci_reset(host, SDHCI_RESET_CMD);
			sdhci_reset(host, SDHCI_RESET_DATA);
		}

		sdhci_send_command(host, data->stop);
	} else
		tasklet_schedule(&host->finish_tasklet);
}

static void sdhci_send_command(struct sdhci_host *host, struct mmc_command *cmd)
{
	int flags;
	u32 mask;
	u32 mode = 0;
	unsigned long timeout;

	WARN_ON(host->cmd);

	/* Wait max 10 ms */
	timeout = 10;

	mask = SDHCI_CMD_INHIBIT;
	if ((cmd->data != NULL) || (cmd->flags & MMC_RSP_BUSY))
		mask |= SDHCI_DATA_INHIBIT;

	/* We shouldn't wait for data inihibit for stop commands, even
	   though they might use busy signaling */
	if (host->mrq->data && (cmd == host->mrq->data->stop))
		mask &= ~SDHCI_DATA_INHIBIT;

	while (readl(host->ioaddr + SDHCI_PRESENT_STATE) & mask) {
		if (timeout == 0) {
			printk(KERN_ERR "%s: Controller never released "
			       "inhibit bit(s).\n", mmc_hostname(host->mmc));
			sdhci_dumpregs(host);
			cmd->error = -EIO;
			tasklet_schedule(&host->finish_tasklet);
			return;
		}
		timeout--;
		mdelay(1);
	}

	mod_timer(&host->timer, jiffies + 10 * HZ);

	host->cmd = cmd;

	sdhci_prepare_data(host, cmd->data);

	writel(cmd->arg, host->ioaddr + SDHCI_ARGUMENT);

	/* Set up the transfer mode */
	if (cmd->data != NULL) {
		mode = SDHCI_TRNS_BLK_CNT_EN | SDHCI_TRNS_DPSEL;
		if (cmd->data->blocks > 1)
			mode |= SDHCI_TRNS_MULTI;
		if (cmd->data->flags & MMC_DATA_READ)
			mode |= SDHCI_TRNS_READ;
		else
			mode &= ~SDHCI_TRNS_READ;
		if (host->flags & SDHCI_USE_DMA)
			mode |= SDHCI_TRNS_DMA;
		DBG("Prepare data completely in %s transfer mode.\n",
		    (host->flags & SDHCI_USE_DMA) ? "INTERNAL DMA" : "PIO");
	}

	if ((cmd->flags & MMC_RSP_136) && (cmd->flags & MMC_RSP_BUSY)) {
		printk(KERN_ERR "%s: Unsupported response type!\n",
		       mmc_hostname(host->mmc));
		cmd->error = -EINVAL;
		tasklet_schedule(&host->finish_tasklet);
		return;
	}

	if (!(cmd->flags & MMC_RSP_PRESENT))
		flags = SDHCI_CMD_RESP_NONE;
	else if (cmd->flags & MMC_RSP_136)
		flags = SDHCI_CMD_RESP_LONG;
	else if (cmd->flags & MMC_RSP_BUSY)
		flags = SDHCI_CMD_RESP_SHORT_BUSY;
	else
		flags = SDHCI_CMD_RESP_SHORT;

	if (cmd->flags & MMC_RSP_CRC)
		flags |= SDHCI_CMD_CRC;
	if (cmd->flags & MMC_RSP_OPCODE)
		flags |= SDHCI_CMD_INDEX;
	if (cmd->data)
		flags |= SDHCI_CMD_DATA;

	mode |= SDHCI_MAKE_CMD(cmd->opcode, flags);
	writel(mode, host->ioaddr + SDHCI_TRANSFER_MODE);
	DBG("Complete sending cmd, tansfer mode is 0x%x.\n",
	    readl(host->ioaddr + SDHCI_TRANSFER_MODE));
}

static void sdhci_finish_command(struct sdhci_host *host)
{
	int i;

	BUG_ON(host->cmd == NULL);

	if (host->cmd->flags & MMC_RSP_PRESENT) {
		if (host->cmd->flags & MMC_RSP_136) {
			/* CRC is stripped so we need to do some shifting. */
			for (i = 0; i < 4; i++) {
				host->cmd->resp[i] = readl(host->ioaddr +
							   SDHCI_RESPONSE + (3 -
									     i)
							   * 4) << 8;
				if (i != 3)
					host->cmd->resp[i] |=
					    readb(host->ioaddr +
						  SDHCI_RESPONSE + (3 - i) * 4 -
						  1);
			}
		} else {
			host->cmd->resp[0] =
			    readl(host->ioaddr + SDHCI_RESPONSE);
		}
	}

	host->cmd->error = 0;

	if (host->data && host->data_early)
		sdhci_finish_data(host);

	if (!host->cmd->data)
		tasklet_schedule(&host->finish_tasklet);

	host->cmd = NULL;
}

static void sdhci_set_clock(struct sdhci_host *host, unsigned int clock)
{
	/*This variable holds the value of clock divider, prescaler */
	int div = 0, prescaler = 0;
	int clk_rate = clk_get_rate(host->clk);
	u32 clk;
	unsigned long timeout;

	if (clock == host->clock)
		return;

	clk = readl(host->ioaddr + SDHCI_CLOCK_CONTROL) & ~SDHCI_CLOCK_MASK;
	writel(clk, host->ioaddr + SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		goto out;
	/*
	 * when prescaler = 16, CLK_20M = CLK_DIV / 2
	 */
	if (clock == host->min_clk)
		prescaler = 16;
	else
		prescaler = 0;
	/* div=1, CLK_DIV = ipg_perclk/2 */
	while (prescaler <= 0x80) {
		for (div = 1; div <= 0xF; div++) {
			int x;
			if (prescaler != 0)
				x = (clk_rate / (div + 1)) / (prescaler * 2);
			else
				x = clk_rate / (div + 1);

			DBG("x=%d, clock=%d %d\n", x, clock, div);
			if (x <= clock)
				break;
		}
		if (div < 0x10)
			break;
		if (prescaler == 0)
			prescaler = 1;
		else
			prescaler <<= 1;
	}
	DBG("prescaler = 0x%x, divider = 0x%x\n", prescaler, div);
	clk |= (prescaler << 8) | (div << 4);

	/* Configure the clock control register */
	clk |=
	    (readl(host->ioaddr + SDHCI_CLOCK_CONTROL) & (~SDHCI_CLOCK_MASK));
	writel(clk, host->ioaddr + SDHCI_CLOCK_CONTROL);

	/* Wait max 10 ms */
	timeout = 10;
	while (timeout > 0) {
		timeout--;
		mdelay(1);
	}

out:
	host->clock = clock;
}

static void sdhci_set_power(struct sdhci_host *host, unsigned short power)
{
	int voltage = 0;

	/* There is no PWR CTL REG*/
	if (host->power == power)
		return;

	if (host->regulator_mmc) {
		if (power == (unsigned short)-1) {
			regulator_disable(host->regulator_mmc);
			pr_debug("mmc power off\n");
		} else {
			if (power == 7)
				voltage = 1800000;
			else if (power >= 8)
				voltage = 2000000 + (power - 8) * 100000;
			regulator_set_voltage(host->regulator_mmc, voltage);

			if (regulator_enable(host->regulator_mmc) == 0) {
				pr_debug("mmc power on\n");
				msleep(300);
			}
		}
	}

	host->power = power;
}

/*****************************************************************************\
 *                                                                           *
 * MMC callbacks                                                             *
 *                                                                           *
\*****************************************************************************/

static void sdhci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct sdhci_host *host;
	unsigned long flags;
	int card_gpio_status;

	host = mmc_priv(mmc);

	spin_lock_irqsave(&host->lock, flags);

	WARN_ON(host->mrq != NULL);

	sdhci_activate_led(host);

	host->mrq = mrq;
#ifdef CONFIG_MACH_MX35_3DS
	if (!(host->flags & SDHCI_CD_PRESENT))
#else
	card_gpio_status = host->plat_data->status(host->mmc->parent);
	if (card_gpio_status)
#endif
	{
		host->mrq->cmd->error = -ENOMEDIUM;
		tasklet_schedule(&host->finish_tasklet);
	} else
		sdhci_send_command(host, mrq->cmd);

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

static void sdhci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sdhci_host *host;
	unsigned long flags;
	u32 ctrl;
	mxc_dma_device_t dev_id = 0;

	DBG("%s: clock %u, bus %lu, power %u, vdd %u\n", DRIVER_NAME,
	    ios->clock, 1UL << ios->bus_width, ios->power_mode, ios->vdd);

	host = mmc_priv(mmc);

	spin_lock_irqsave(&host->lock, flags);

	/*
	 * Reset the chip on each power off.
	 * Should clear out any weird states.
	 */
	if (ios->power_mode == MMC_POWER_OFF) {
		writel(0, host->ioaddr + SDHCI_SIGNAL_ENABLE);
		sdhci_init(host);
	}

	if (host->chip->quirks & SDHCI_QUIRK_EXTERNAL_DMA_MODE) {
		host->dma_dir = DMA_NONE;
		if (mmc->ios.bus_width != host->mode) {
			mxc_dma_free(host->dma);
			if (mmc->ios.bus_width == MMC_BUS_WIDTH_4) {
				if (host->id == 0)
					dev_id = MXC_DMA_MMC1_WIDTH_4;
				else
					dev_id = MXC_DMA_MMC2_WIDTH_4;

				/* Configure the WML rege */
				writel(SDHCI_WML_64_WORDS,
				       host->ioaddr + SDHCI_WML);
			} else {
				if (host->id == 0)
					dev_id = MXC_DMA_MMC1_WIDTH_1;
				else
					dev_id = MXC_DMA_MMC2_WIDTH_1;

				/* Configure the WML rege */
				writel(SDHCI_WML_16_WORDS,
				       host->ioaddr + SDHCI_WML);
			}
			DBG("%d dma chan request\n", dev_id);
			host->dma = mxc_dma_request(dev_id, "MXC MMC");
			if (host->dma < 0)
				DBG("Cannot allocate MMC DMA channel\n");
			mxc_dma_callback_set(host->dma, sdhci_dma_irq,
					     (void *)host);
		}
	}

	host->mode = mmc->ios.bus_width;

	sdhci_set_clock(host, ios->clock);

	if (ios->power_mode == MMC_POWER_OFF)
		sdhci_set_power(host, -1);
	else
		sdhci_set_power(host, ios->vdd);

	ctrl = readl(host->ioaddr + SDHCI_HOST_CONTROL);

	if (ios->bus_width == MMC_BUS_WIDTH_4)
		ctrl |= SDHCI_CTRL_4BITBUS;
	else
		ctrl &= ~SDHCI_CTRL_4BITBUS;

	/* Only 4bit bus mode is enabled now :( */
#if 0
	if (ios->timing == MMC_TIMING_SD_HS)
		ctrl |= SDHCI_CTRL_HISPD;
	else
		ctrl &= ~SDHCI_CTRL_HISPD;
#endif

	writel(ctrl, host->ioaddr + SDHCI_HOST_CONTROL);

	/*
	 * Some (ENE) controllers go apeshit on some ios operation,
	 * signalling timeout and CRC errors even on CMD0. Resetting
	 * it on each ios seems to solve the problem.
	 */
	if (host->chip->quirks & SDHCI_QUIRK_RESET_CMD_DATA_ON_IOS)
		sdhci_reset(host, SDHCI_RESET_CMD | SDHCI_RESET_DATA);

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

static int sdhci_get_ro(struct mmc_host *mmc)
{
	struct sdhci_host *host;
	unsigned long flags;
	int present;

	host = mmc_priv(mmc);

	spin_lock_irqsave(&host->lock, flags);

	present = readl(host->ioaddr + SDHCI_PRESENT_STATE);

	spin_unlock_irqrestore(&host->lock, flags);

	return !(present & SDHCI_WRITE_PROTECT);
}

static void sdhci_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct sdhci_host *host;
	unsigned long flags;
	u32 ier;

	host = mmc_priv(mmc);

	spin_lock_irqsave(&host->lock, flags);

	ier = readl(host->ioaddr + SDHCI_INT_ENABLE);

	ier &= ~SDHCI_INT_CARD_INT;
	if (enable)
		ier |= SDHCI_INT_CARD_INT;

	writel(ier, host->ioaddr + SDHCI_INT_ENABLE);
	writel(ier, host->ioaddr + SDHCI_SIGNAL_ENABLE);

	mmiowb();

	spin_unlock_irqrestore(&host->lock, flags);
}

static const struct mmc_host_ops sdhci_ops = {
	.request = sdhci_request,
	.set_ios = sdhci_set_ios,
	.get_ro = sdhci_get_ro,
	.enable_sdio_irq = sdhci_enable_sdio_irq,
};

/*****************************************************************************\
*                                                                           *
* Tasklets                                                                  *
*                                                                           *
\*****************************************************************************/

static void sdhci_tasklet_card(unsigned long param)
{
	struct sdhci_host *host;
	unsigned long flags;

	host = (struct sdhci_host *)param;

	spin_lock_irqsave(&host->lock, flags);

	if (!(host->flags & SDHCI_CD_PRESENT)) {
		if (host->mrq) {
			printk(KERN_ERR
			       "%s: Card removed during transfer!\n",
			       mmc_hostname(host->mmc));
			printk(KERN_ERR
			       "%s: Resetting controller.\n",
			       mmc_hostname(host->mmc));

			sdhci_reset(host, SDHCI_RESET_CMD);
			sdhci_reset(host, SDHCI_RESET_DATA);

			host->mrq->cmd->error = -ENOMEDIUM;
			tasklet_schedule(&host->finish_tasklet);
		}
	}

	spin_unlock_irqrestore(&host->lock, flags);

	mmc_detect_change(host->mmc, msecs_to_jiffies(500));
}

static void sdhci_tasklet_finish(unsigned long param)
{
	struct sdhci_host *host;
	unsigned long flags;
	struct mmc_request *mrq;

	host = (struct sdhci_host *)param;

	spin_lock_irqsave(&host->lock, flags);

	del_timer(&host->timer);

	mrq = host->mrq;

	/*
	 * The controller needs a reset of internal state machines
	 * upon error conditions.
	 */
	if (mrq->cmd->error ||
	    (mrq->data && (mrq->data->error ||
			   (mrq->data->stop && mrq->data->stop->error)))
	    || (host->chip->quirks & SDHCI_QUIRK_RESET_AFTER_REQUEST)) {

		/* Some controllers need this kick or reset won't work here */
		if (host->chip->quirks & SDHCI_QUIRK_CLOCK_BEFORE_RESET) {
			unsigned int clock;

			/* This is to force an update */
			clock = host->clock;
			host->clock = 0;
			sdhci_set_clock(host, clock);
		}

		/* Spec says we should do both at the same time, but Ricoh
		   controllers do not like that. */
		sdhci_reset(host, SDHCI_RESET_CMD);
		sdhci_reset(host, SDHCI_RESET_DATA);
	}

	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;

	sdhci_deactivate_led(host);

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);

	mmc_request_done(host->mmc, mrq);
}

static void sdhci_timeout_timer(unsigned long data)
{
	struct sdhci_host *host;
	unsigned long flags;

	host = (struct sdhci_host *)data;

	spin_lock_irqsave(&host->lock, flags);

	if (host->mrq) {
		printk(KERN_ERR
		       "%s: Timeout waiting for hardware "
		       "interrupt.\n", mmc_hostname(host->mmc));
		sdhci_dumpregs(host);

		if (host->data) {
			host->data->error = -ETIMEDOUT;
			sdhci_finish_data(host);
		} else {
			if (host->cmd)
				host->cmd->error = -ETIMEDOUT;
			else
				host->mrq->cmd->error = -ETIMEDOUT;

			tasklet_schedule(&host->finish_tasklet);
		}
	}

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

/*****************************************************************************\
*                                                                           *
* Interrupt handling                                                        *
*                                                                           *
\*****************************************************************************/

static void sdhci_cmd_irq(struct sdhci_host *host, u32 intmask)
{
	BUG_ON(intmask == 0);

	if (!host->cmd) {
		printk(KERN_ERR
		       "%s: Got command interrupt 0x%08x even "
		       "though no command operation was in progress.\n",
		       mmc_hostname(host->mmc), (unsigned)intmask);
		sdhci_dumpregs(host);
		return;
	}

	if (intmask & SDHCI_INT_TIMEOUT)
		host->cmd->error = -ETIMEDOUT;
	else if (intmask & (SDHCI_INT_CRC | SDHCI_INT_END_BIT |
			    SDHCI_INT_INDEX))
		host->cmd->error = -EILSEQ;

	if (host->cmd->error)
		tasklet_schedule(&host->finish_tasklet);
	else if (intmask & SDHCI_INT_RESPONSE)
		sdhci_finish_command(host);
}

static void sdhci_data_irq(struct sdhci_host *host, u32 intmask)
{
	BUG_ON(intmask == 0);

	if (!host->data) {
		/*
		 * A data end interrupt is sent together with the response
		 * for the stop command.
		 */
		if (intmask & SDHCI_INT_DATA_END)
			return;

		printk(KERN_ERR
		       "%s: Got data interrupt 0x%08x even "
		       "though no data operation was in progress.\n",
		       mmc_hostname(host->mmc), (unsigned)intmask);
		sdhci_dumpregs(host);

		return;
	}

	if (intmask & SDHCI_INT_DATA_TIMEOUT)
		host->data->error = -ETIMEDOUT;
	else if (intmask & (SDHCI_INT_DATA_CRC | SDHCI_INT_DATA_END_BIT))
		host->data->error = -EILSEQ;

	if (host->data->error)
		sdhci_finish_data(host);
	else {
		if (host->chip->quirks & SDHCI_QUIRK_EXTERNAL_DMA_MODE) {
			/* Use DMA if transfer size is greater than fifo size */
			if (intmask & (SDHCI_INT_DATA_AVAIL |
				       SDHCI_INT_SPACE_AVAIL)) {
				if (host->dma_size > (64 <<
						      host->
						      mmc->ios.bus_width)) {
					if (mxc_dma_enable(host->dma) < 0)
						printk(KERN_ERR
						       "ENABLE SDMA ERR.\n");
				} else {
					sdhci_transfer_pio(host);
				}

			}
		} else {
			if (intmask & (SDHCI_INT_DATA_AVAIL |
				       SDHCI_INT_SPACE_AVAIL))
				sdhci_transfer_pio(host);
		}

		/*
		 * We currently don't do anything fancy with DMA
		 * boundaries, but as we can't disable the feature
		 * we need to at least restart the transfer.
		 */
		if (intmask & SDHCI_INT_DMA_END)
			writel(readl
			       (host->ioaddr +
				SDHCI_DMA_ADDRESS),
			       host->ioaddr + SDHCI_DMA_ADDRESS);

		if (intmask & SDHCI_INT_DATA_END) {
			if (host->cmd) {
				/*
				 * Data managed to finish before the
				 * command completed. Make sure we do
				 * things in the proper order.
				 */
				host->data_early = 1;
			} else {
			/*
			 * There are the DATA END INT when writing is
			 * not complete. Double check on it.
			 */
			intmask = readl(host->ioaddr + SDHCI_PRESENT_STATE);
			if (intmask & SDHCI_DATA_ACTIVE) {
				printk(KERN_ERR
					"ERR DATA END INT in writing.\n");
				return;
			}
				sdhci_finish_data(host);
			}
		}
	}
}

/*!
* This function is called by DMA Interrupt Service Routine to indicate
* requested DMA transfer is completed.
*
* @param   devid  pointer to device specific structure
* @param   error any DMA error
* @param   cnt   amount of data that was transferred
*/
static void sdhci_dma_irq(void *devid, int error, unsigned int cnt)
{
	struct sdhci_host *host = devid;

	if (host->chip->quirks & SDHCI_QUIRK_EXTERNAL_DMA_MODE) {
		/*
		 * Stop the DMA transfer here, the data_irq would be called
		 * to process the others
		 */
		mxc_dma_disable(host->dma);

		if (error) {
			DBG("Error in DMA transfer\n");
			return;
		}
		DBG("%s: Transfered bytes:%d\n", DRIVER_NAME, cnt);
	}
}

#ifdef CONFIG_MACH_MX35_3DS
/*!
* Card detection interrupt callback routine registered to handle the
* SDHC interrupts. This interrupt callback handles card insertion
* and card removal interrupts. when the PMIC events mechanis is used.
*
* @param   irq    the interrupt number
* @param   devid  driver private data
*
* @return  The function returns \b IRQ_RETVAL(1)
*/
static void sdhci_pmic_cd_func(void *devid)
{
	int cd_status = 0;
	unsigned long flags;
	struct sdhci_host *host = devid;

	if (host->flags & SDHCI_CD_PRESENT)
		host->flags &= ~SDHCI_CD_PRESENT;
	else
		host->flags |= SDHCI_CD_PRESENT;

	/* Detect there is a card in slot or not */
	DBG("%s: %s cd_status=%d %s\n", DRIVER_NAME,
	    host->slot_descr, (host->flags & SDHCI_CD_PRESENT),
	    (host->flags & SDHCI_CD_PRESENT)
	    ? "inserted" : "removed");

	if (0 == cd_status) {
#if 0
		host->flags |= SDHCI_CD_PRESENT;
#endif
		mmc_detect_change(host->mmc, msecs_to_jiffies(500));
	} else {
#if 0
		host->flags &= ~SDHCI_CD_PRESENT;
#endif
		if (host->mrq) {
			printk(KERN_ERR
			       "%s: Card removed during transfer!\n",
			       mmc_hostname(host->mmc));
			printk(KERN_ERR
			       "%s: Resetting controller.\n",
			       mmc_hostname(host->mmc));

			spin_lock_irqsave(&host->lock, flags);
			sdhci_reset(host, SDHCI_RESET_CMD);
			sdhci_reset(host, SDHCI_RESET_DATA);

			host->mrq->cmd->error = -ENOMEDIUM;
			tasklet_schedule(&host->finish_tasklet);
			spin_unlock_irqrestore(&host->lock, flags);
		}
		mmc_detect_change(host->mmc, msecs_to_jiffies(100));
	}
}

/*!
* Write protect interrupt callback routine registered to check the SDHC write
* protected or not. when the PMIC events mechanis is used.
*
* @param
* @param
*
* @return
*/
static void sdhci_pmic_wp_func(void *devid)
{
	int wp_status = 0;
	struct sdhci_host *host = devid;

	/* Detect the card in slot is write protected or not */
	if (0 == wp_status)
		host->flags |= SDHCI_WP_ENABLED;
	else
		host->flags &= ~SDHCI_WP_ENABLED;
	host->flags |= SDHCI_WP_ENABLED;
}
#else
/*!
* Card detection interrupt service routine registered to handle
* the SDHC interrupts. This interrupt routine handles card
* insertion and card removal interrupts.
*
* @param   irq    the interrupt number
* @param   devid  driver private data
*
* @return  The function returns \b IRQ_RETVAL(1)
*/
static irqreturn_t sdhci_cd_irq(int irq, void *devid)
{
	struct sdhci_host *host = devid;
	int card_gpio_status = host->plat_data->status(host->mmc->parent);
	unsigned long flags;

	DBG("%s: %s status=%d %s\n", DRIVER_NAME,
	    host->slot_descr, card_gpio_status,
	    card_gpio_status ? "removed" : "inserted");

	if (card_gpio_status == host->plat_data->card_inserted_state) {
		host->flags &= ~SDHCI_CD_PRESENT;
		if (host->mrq) {
			printk(KERN_ERR
			       "%s: Card removed during transfer!\n",
			       mmc_hostname(host->mmc));
			printk(KERN_ERR
			       "%s: Resetting controller.\n",
			       mmc_hostname(host->mmc));

			spin_lock_irqsave(&host->lock, flags);
			sdhci_reset(host, SDHCI_RESET_CMD);
			sdhci_reset(host, SDHCI_RESET_DATA);

			host->mrq->cmd->error = -ENOMEDIUM;
			tasklet_schedule(&host->finish_tasklet);
			spin_unlock_irqrestore(&host->lock, flags);
		}
		mmc_detect_change(host->mmc, msecs_to_jiffies(100));
	} else {
		host->flags |= SDHCI_CD_PRESENT;
		mmc_detect_change(host->mmc, msecs_to_jiffies(500));
	}

	do {
		card_gpio_status = host->plat_data->status(host->mmc->parent);
		if (card_gpio_status)
			set_irq_type(host->detect_irq, IRQT_FALLING);
		else
			set_irq_type(host->detect_irq, IRQT_RISING);
	} while (card_gpio_status !=
		 host->plat_data->status(host->mmc->parent));

	return IRQ_HANDLED;
}
#endif

static irqreturn_t sdhci_irq(int irq, void *dev_id)
{
	irqreturn_t result;
	struct sdhci_host *host = dev_id;
	u32 intmask;
	int cardint = 0;

	spin_lock(&host->lock);

	intmask = readl(host->ioaddr + SDHCI_INT_STATUS);

	if (!intmask || intmask == 0xffffffff) {
		result = IRQ_NONE;
		goto out;
	}

	DBG("*** %s got interrupt: 0x%08x\n", host->slot_descr, intmask);

	if (intmask & (SDHCI_INT_CARD_INSERT | SDHCI_INT_CARD_REMOVE)) {
		writel(intmask &
		       (SDHCI_INT_CARD_INSERT |
			SDHCI_INT_CARD_REMOVE),
		       host->ioaddr + SDHCI_INT_STATUS);
		tasklet_schedule(&host->card_tasklet);
	}

	intmask &= ~(SDHCI_INT_CARD_INSERT | SDHCI_INT_CARD_REMOVE);

	if (intmask & SDHCI_INT_CMD_MASK) {
		writel(intmask & SDHCI_INT_CMD_MASK,
		       host->ioaddr + SDHCI_INT_STATUS);
		sdhci_cmd_irq(host, intmask & SDHCI_INT_CMD_MASK);
	}

	if (intmask & SDHCI_INT_DATA_MASK) {
		writel(intmask & SDHCI_INT_DATA_MASK,
		       host->ioaddr + SDHCI_INT_STATUS);
		sdhci_data_irq(host, intmask & SDHCI_INT_DATA_MASK);
	}

	intmask &= ~(SDHCI_INT_CMD_MASK | SDHCI_INT_DATA_MASK);

	intmask &= ~SDHCI_INT_ERROR;

	if (intmask & SDHCI_INT_BUS_POWER) {
		printk(KERN_ERR
		       "%s: Card is consuming too much power!\n",
		       mmc_hostname(host->mmc));
		writel(SDHCI_INT_BUS_POWER, host->ioaddr + SDHCI_INT_STATUS);
	}

	intmask &= ~SDHCI_INT_BUS_POWER;

	if (intmask & SDHCI_INT_CARD_INT)
		cardint = 1;

	intmask &= ~SDHCI_INT_CARD_INT;

	if (intmask) {
		printk(KERN_ERR
		       "%s: Unexpected interrupt 0x%08x.\n",
		       mmc_hostname(host->mmc), intmask);
		sdhci_dumpregs(host);

		writel(intmask, host->ioaddr + SDHCI_INT_STATUS);
	}

	result = IRQ_HANDLED;

	mmiowb();
out:
	spin_unlock(&host->lock);

	/*
	 * We have to delay this as it calls back into the driver.
	 */
	if (cardint)
		mmc_signal_sdio_irq(host->mmc);

	return result;
}

/*****************************************************************************\
*                                                                           *
* Suspend/resume                                                            *
*                                                                           *
\*****************************************************************************/

#ifdef CONFIG_PM

static int sdhci_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct sdhci_chip *chip;
	int i, ret;

	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct sdhci_host *host = mmc_priv(mmc);

	chip = dev_get_drvdata(&pdev->dev);
	if (!chip)
		return 0;

	DBG("Suspending...\n");

	for (i = 0; i < chip->num_slots; i++) {
		if (!chip->hosts[i])
			continue;
		ret = mmc_suspend_host(chip->hosts[i]->mmc, state);
		if (ret) {
			for (i--; i >= 0; i--)
				mmc_resume_host(chip->hosts[i]->mmc);
			return ret;
		}
	}

	for (i = 0; i < chip->num_slots; i++) {
		if (!chip->hosts[i])
			continue;
		free_irq(chip->hosts[i]->irq, chip->hosts[i]);
	}

	clk_disable(host->clk);
	gpio_sdhc_inactive(pdev->id);

	return 0;
}

static int sdhci_resume(struct platform_device *pdev)
{
	struct sdhci_chip *chip;
	int i, ret;

	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct sdhci_host *host = mmc_priv(mmc);

	chip = dev_get_drvdata(&pdev->dev);
	if (!chip)
		return 0;

	DBG("Resuming...\n");

	gpio_sdhc_active(pdev->id);
	clk_enable(host->clk);

	for (i = 0; i < chip->num_slots; i++) {
		if (!chip->hosts[i])
			continue;
		ret =
		    request_irq(chip->hosts[i]->irq, sdhci_irq,
				IRQF_SHARED,
				chip->hosts[i]->slot_descr, chip->hosts[i]);
		if (ret)
			return ret;
		sdhci_init(chip->hosts[i]);
		mmiowb();
		ret = mmc_resume_host(chip->hosts[i]->mmc);
		if (ret)
			return ret;
	}

	return 0;
}

#else				/* CONFIG_PM */

#define sdhci_suspend NULL
#define sdhci_resume NULL

#endif				/* CONFIG_PM */

/*****************************************************************************\
*                                                                           *
* Device probing/removal                                                    *
*                                                                           *
\*****************************************************************************/

static int __devinit sdhci_probe_slot(struct platform_device
				      *pdev, int slot)
{
	struct mxc_mmc_platform_data *mmc_plat = pdev->dev.platform_data;
	int ret = 0;
	unsigned int version, caps;
	struct sdhci_chip *chip;
	struct mmc_host *mmc;
	struct sdhci_host *host;

	if (!mmc_plat)
		return -EINVAL;

	chip = dev_get_drvdata(&pdev->dev);
	BUG_ON(!chip);

	mmc = mmc_alloc_host(sizeof(struct mmc_host), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);

	host->mmc = mmc;
	host->dma = -1;
	host->plat_data = mmc_plat;
	if (!host->plat_data) {
		ret = -EINVAL;
		goto out0;
	}

	host->chip = chip;
	chip->hosts[slot] = host;

	/* Active the eSDHC bus */
	gpio_sdhc_active(pdev->id);

	/* Get the SDHC clock from clock system APIs */
	host->clk = clk_get(&pdev->dev, mmc_plat->clock_mmc);
	if (NULL != host->clk) {
		if (clk_enable(host->clk))
			printk(KERN_ERR "MXC MMC enable clock error.\n");
	} else
		printk(KERN_ERR "MXC MMC can't get clock.\n");
	DBG("SDHC:%d clock:%lu\n", pdev->id, clk_get_rate(host->clk));

	host->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!host->res) {
		ret = -ENOMEM;
		goto out1;
	}
	host->irq = platform_get_irq(pdev, 0);
	if (!host->irq) {
		ret = -ENOMEM;
		goto out1;
	}
#ifdef CONFIG_MACH_MX35_3DS
	/* Subcribe the PMIC event */
	sdhci_pmic_cd_callback.func = sdhci_pmic_cd_func;
	sdhci_pmic_cd_callback.param = host;
	CHECK_ERROR(pmic_event_subscribe
		    (EVENT_SD1_DET, sdhci_pmic_cd_callback));

	sdhci_pmic_wp_callback.func = sdhci_pmic_wp_func;
	sdhci_pmic_wp_callback.param = host;
	CHECK_ERROR(pmic_event_subscribe
		    (EVENT_SD1_WP, sdhci_pmic_wp_callback));
#else
	host->detect_irq = platform_get_irq(pdev, 1);
	if (!host->detect_irq) {
		ret = -ENOMEM;
		goto out1;
	}

	do {
		ret = host->plat_data->status(host->mmc->parent);
		if (ret)
			set_irq_type(host->detect_irq, IRQT_FALLING);
		else
			set_irq_type(host->detect_irq, IRQT_RISING);
	} while (ret != host->plat_data->status(host->mmc->parent));
#endif

	DBG("slot %d at 0x%x, irq %d\n", slot, host->res->start, host->irq);

	if (!request_mem_region(host->res->start,
				host->res->end -
				host->res->start + 1, pdev->name)) {
		printk(KERN_ERR "request_mem_region failed\n");
		ret = -ENOMEM;
		goto out1;
	}
	host->ioaddr = (void *)ioremap(host->res->start, host->res->end -
					host->res->start + 1);
	if (!host->ioaddr) {
		ret = -ENOMEM;
		goto out3;
	}

	sdhci_reset(host, SDHCI_RESET_ALL);

	version = readl(host->ioaddr + SDHCI_HOST_VERSION);
	version = (version & SDHCI_SPEC_VER_MASK) >> SDHCI_SPEC_VER_SHIFT;
	if (version != 1) {
		printk(KERN_ERR
		       "%s: Unknown controller version (%d). "
		       "You may experience problems.\n",
		       host->slot_descr, version);
	}

	caps = readl(host->ioaddr + SDHCI_CAPABILITIES);

	if (chip->quirks & SDHCI_QUIRK_FORCE_DMA)
		host->flags |= SDHCI_USE_DMA;
	else if (!(caps & SDHCI_CAN_DO_DMA))
		DBG("Controller doesn't have DMA capability\n");
	else if (chip->
		 quirks & (SDHCI_QUIRK_INTERNAL_ADVANCED_DMA |
			   SDHCI_QUIRK_INTERNAL_SIMPLE_DMA))
		host->flags |= SDHCI_USE_DMA;
	else
		host->flags &= ~SDHCI_USE_DMA;

	/*
	 * These definitions of eSDHC are not compatible with the SD Host
	 * Controller Spec v2.0
	 */
	host->min_clk = mmc_plat->min_clk;
	host->max_clk = mmc_plat->max_clk;
	host->timeout_clk = 32 * 1000;	/* Just set the value temply. */

	/*
	 * Set host parameters.
	 */
	mmc->ops = &sdhci_ops;
	mmc->f_min = host->min_clk;
	mmc->f_max = host->max_clk;
	mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_MULTIWRITE | MMC_CAP_SDIO_IRQ;

	if (caps & SDHCI_CAN_DO_HISPD)
		mmc->caps |= MMC_CAP_SD_HIGHSPEED;

	mmc->ocr_avail = 0;
	if (caps & SDHCI_CAN_VDD_330)
		mmc->ocr_avail |= MMC_VDD_32_33 | MMC_VDD_33_34;
	if (caps & SDHCI_CAN_VDD_300)
		mmc->ocr_avail |= MMC_VDD_29_30 | MMC_VDD_30_31;
	if (caps & SDHCI_CAN_VDD_180)
		mmc->ocr_avail |= MMC_VDD_165_195;

	if (mmc->ocr_avail == 0) {
		printk(KERN_ERR
		       "%s: Hardware doesn't report any "
		       "support voltages.\n", host->slot_descr);
		ret = -ENODEV;
		goto out3;
	}

	spin_lock_init(&host->lock);

	/*
	 * Maximum number of segments. Hardware cannot do scatter lists.
	 */
	if (host->flags & SDHCI_USE_DMA)
		mmc->max_hw_segs = 1;
	else
		mmc->max_hw_segs = 16;
	mmc->max_phys_segs = 16;

	/*
	 * Maximum number of sectors in one transfer. Limited by DMA boundary
	 * size (512KiB).
	 */
	mmc->max_req_size = 524288;

	/*
	 * Maximum segment size. Could be one segment with the maximum number
	 * of bytes.
	 */
	mmc->max_seg_size = mmc->max_req_size;

	/*
	 * Maximum block size. This varies from controller to controller and
	 * is specified in the capabilities register.
	 */
	mmc->max_blk_size =
	    (caps & SDHCI_MAX_BLOCK_MASK) >> SDHCI_MAX_BLOCK_SHIFT;
	if (mmc->max_blk_size > 3) {
		printk(KERN_ERR
		       "%s: Invalid maximum block size.\n", host->slot_descr);
		ret = -ENODEV;
		goto out3;
	}
	mmc->max_blk_size = 512 << mmc->max_blk_size;

	/*
	 * Maximum block count.
	 */
	mmc->max_blk_count = 65535;

	/*
	 * Init tasklets.
	 */
	tasklet_init(&host->card_tasklet,
		     sdhci_tasklet_card, (unsigned long)host);
	tasklet_init(&host->finish_tasklet,
		     sdhci_tasklet_finish, (unsigned long)host);

	setup_timer(&host->timer, sdhci_timeout_timer, (unsigned long)host);

#ifdef CONFIG_MACH_MX35_3DS
#else
	ret = request_irq(host->detect_irq, sdhci_cd_irq, 0, pdev->name, host);
	if (ret)
		goto out4;
#endif

	ret = request_irq(host->irq, sdhci_irq, IRQF_SHARED, pdev->name, host);
	if (ret)
		goto out5;

	sdhci_init(host);

#ifdef CONFIG_MMC_DEBUG
	sdhci_dumpregs(host);
#endif

	mmiowb();

	if (mmc_add_host(mmc) < 0)
		goto out6;
	printk(KERN_INFO "%s: SDHCI detect irq %d irq %d %s\n",
	       mmc_hostname(mmc), host->detect_irq, host->irq,
	       (host->flags & SDHCI_USE_DMA) ? "INTERNAL DMA" : "PIO");

	return 0;

out6:
	free_irq(host->irq, host);
out5:
#ifdef CONFIG_MACH_MX35_3DS
	CHECK_ERROR(pmic_event_unsubscribe
		    (EVENT_SD1_DET, sdhci_pmic_cd_callback));
	CHECK_ERROR(pmic_event_unsubscribe
		    (EVENT_SD1_WP, sdhci_pmic_wp_callback));
#else
	if (host->detect_irq)
		free_irq(host->detect_irq, host);
out4:
#endif
	del_timer_sync(&host->timer);
	tasklet_kill(&host->card_tasklet);
	tasklet_kill(&host->finish_tasklet);
out3:
	release_mem_region(host->res->start,
			   host->res->end - host->res->start + 1);
out1:
	clk_disable(host->clk);
	gpio_sdhc_inactive(pdev->id);
out0:
	mmc_free_host(mmc);
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static void sdhci_remove_slot(struct platform_device *pdev, int slot)
{
	struct sdhci_chip *chip;
	struct mmc_host *mmc;
	struct sdhci_host *host;

	chip = dev_get_drvdata(&pdev->dev);
	host = chip->hosts[slot];
	mmc = host->mmc;

	chip->hosts[slot] = NULL;

	mmc_remove_host(mmc);

	sdhci_reset(host, SDHCI_RESET_ALL);

#ifdef CONFIG_MACH_MX35_3DS
	pmic_event_unsubscribe(EVENT_SD1_DET, sdhci_pmic_cd_callback);
	pmic_event_unsubscribe(EVENT_SD1_WP, sdhci_pmic_wp_callback);
#else
	if (host->detect_irq)
		free_irq(host->detect_irq, host);
#endif
	free_irq(host->irq, host);
	if (chip->quirks & SDHCI_QUIRK_EXTERNAL_DMA_MODE)
		mxc_dma_free(host->dma);

	del_timer_sync(&host->timer);

	tasklet_kill(&host->card_tasklet);
	tasklet_kill(&host->finish_tasklet);

	release_mem_region(host->res->start,
			   host->res->end - host->res->start + 1);
	clk_disable(host->clk);
	mmc_free_host(mmc);
	gpio_sdhc_inactive(pdev->id);
}

static int sdhci_probe(struct platform_device *pdev)
{
	int ret = 0, i;
	u8 slots = 1;
	struct sdhci_chip *chip;

	printk(KERN_INFO DRIVER_NAME ": MXC SDHCI Controller Driver. \n");
	BUG_ON(pdev == NULL);

	chip = kzalloc(sizeof(struct sdhci_chip) +
		       sizeof(struct sdhci_host *) * slots, GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto err;
	}

	/* Distinguish different platform */
	if (machine_is_mx37_3ds()) {
		mxc_quirks = SDHCI_QUIRK_EXTERNAL_DMA_MODE;
	} else {
		mxc_quirks = SDHCI_QUIRK_INTERNAL_ADVANCED_DMA |
		    SDHCI_QUIRK_INTERNAL_SIMPLE_DMA;
	}
	chip->pdev = pdev;
	chip->quirks = mxc_quirks;

	if (debug_quirks)
		chip->quirks = debug_quirks;

	chip->num_slots = slots;
	dev_set_drvdata(&pdev->dev, chip);

	for (i = 0; i < slots; i++) {
		ret = sdhci_probe_slot(pdev, i);
		if (ret) {
			for (i--; i >= 0; i--)
				sdhci_remove_slot(pdev, i);
			goto free;
		}
	}

	return 0;

free:
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(chip);

err:
	return ret;
}

static int sdhci_remove(struct platform_device *pdev)
{
	int i;
	struct sdhci_chip *chip;

	chip = dev_get_drvdata(&pdev->dev);

	if (chip) {
		for (i = 0; i < chip->num_slots; i++)
			sdhci_remove_slot(pdev, i);

		dev_set_drvdata(&pdev->dev, NULL);

		kfree(chip);
	}

	return 0;
}

static struct platform_driver sdhci_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   },
	.probe = sdhci_probe,
	.remove = sdhci_remove,
	.suspend = sdhci_suspend,
	.resume = sdhci_resume,
};

/*****************************************************************************\
*                                                                           *
* Driver init/exit                                                          *
*                                                                           *
\*****************************************************************************/

static int __init sdhci_drv_init(void)
{
	printk(KERN_INFO DRIVER_NAME
	       ": MXC Secure Digital Host Controller Interface driver\n");
	return platform_driver_register(&sdhci_driver);
}

static void __exit sdhci_drv_exit(void)
{
	DBG("Exiting\n");

	platform_driver_unregister(&sdhci_driver);
}

module_init(sdhci_drv_init);
module_exit(sdhci_drv_exit);

module_param(debug_quirks, uint, 0444);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC Secure Digital Host Controller Interface driver");
MODULE_LICENSE("GPL");

MODULE_PARM_DESC(debug_quirks, "Force certain quirks.");
