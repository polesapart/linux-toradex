/*
 *  linux/drivers/mmc/s3cmci.h - Samsung S3C MCI driver
 *
 *  Copyright (C) 2004-2006 maintech GmbH, Thomas Kleffel <tk@maintech.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/mmc/mmc.h>
#include <asm/dma.h>

/* Use the old headers system */
#include <mach/regs-sdi.h>
#include <mach/regs-gpio.h>
#include <mach/gpio.h>
#include <asm/delay.h>
#include <linux/delay.h>

#include <asm/plat-s3c24xx/mci.h>

#include "s3cmci.h"

#define DRIVER_NAME "s3c-mci"

#define printk_err(fmt, args...)                printk(KERN_ERR "[ ERROR ] s3c2443-sdi: " fmt, ## args)
#define printk_info(fmt, args...)               printk(KERN_INFO "s3c2443-sdi: " fmt, ## args)
#define printk_dbg(fmt, args...)                printk(KERN_DEBUG "s3c2443-sdi: " fmt, ## args)

/* Enable/disable the debug messages */
#if 0
#define S3C2443_SDI_DEBUG
#endif

#ifdef S3C2443_SDI_DEBUG
#  define printk_debug(fmt, args...)		printk(KERN_DEBUG "s3c2443-sdi: %s() " fmt, __FUNCTION__ , ## args)
#else
#  define printk_debug(fmt, args...)
#endif


enum dbg_channels {
	dbg_err   = (1 << 0),
	dbg_debug = (1 << 1),
	dbg_info  = (1 << 2),
	dbg_irq   = (1 << 3),
	dbg_sg    = (1 << 4),
	dbg_dma   = (1 << 5),
	dbg_pio   = (1 << 6),
	dbg_fail  = (1 << 7),
	dbg_conf  = (1 << 8),
};

static const int dbgmap_err   = dbg_err | dbg_fail;
static const int dbgmap_info  = dbg_info | dbg_conf;
static const int dbgmap_debug = dbg_debug;

#define dbg(host, channels, args...)		  \
	do {					  \
	if (dbgmap_err & channels) 		  \
		dev_err(&host->pdev->dev, args);  \
	else if (dbgmap_info & channels)	  \
		dev_info(&host->pdev->dev, args); \
	else if (dbgmap_debug & channels)	  \
		dev_dbg(&host->pdev->dev, args);  \
	} while (0)

#define RESSIZE(ressource) (((ressource)->end - (ressource)->start)+1)

static struct s3c2410_dma_client s3cmci_dma_client = {
	.name		= "s3c-mci",
};

static void finalize_request(struct s3cmci_host *host);
static void s3cmci_send_request(struct mmc_host *mmc);
static void s3cmci_reset(struct s3cmci_host *host);

#ifdef CONFIG_MMC_DEBUG

static void dbg_dumpregs(struct s3cmci_host *host, char *prefix)
{
	u32 con, pre, cmdarg, cmdcon, cmdsta, r0, r1, r2, r3, timer, bsize;
	u32 datcon, datcnt, datsta, fsta, imask;

	con 	= readl(host->base + S3C2410_SDICON);
	pre 	= readl(host->base + S3C2410_SDIPRE);
	cmdarg 	= readl(host->base + S3C2410_SDICMDARG);
	cmdcon 	= readl(host->base + S3C2410_SDICMDCON);
	cmdsta 	= readl(host->base + S3C2410_SDICMDSTAT);
	r0 	= readl(host->base + S3C2410_SDIRSP0);
	r1 	= readl(host->base + S3C2410_SDIRSP1);
	r2 	= readl(host->base + S3C2410_SDIRSP2);
	r3 	= readl(host->base + S3C2410_SDIRSP3);
	timer 	= readl(host->base + S3C2410_SDITIMER);
	bsize 	= readl(host->base + S3C2410_SDIBSIZE);
	datcon 	= readl(host->base + S3C2410_SDIDCON);
	datcnt 	= readl(host->base + S3C2410_SDIDCNT);
	datsta 	= readl(host->base + S3C2410_SDIDSTA);
	fsta 	= readl(host->base + S3C2410_SDIFSTA);
	imask   = readl(host->base + host->sdiimsk);

	dbg(host, dbg_debug, "%s  CON:[%08x]  PRE:[%08x]  TMR:[%08x]\n",
				prefix, con, pre, timer);

	dbg(host, dbg_debug, "%s CCON:[%08x] CARG:[%08x] CSTA:[%08x]\n",
				prefix, cmdcon, cmdarg, cmdsta);

	dbg(host, dbg_debug, "%s DCON:[%08x] FSTA:[%08x]"
			       " DSTA:[%08x] DCNT:[%08x]\n",
				prefix, datcon, fsta, datsta, datcnt);

	dbg(host, dbg_debug, "%s   R0:[%08x]   R1:[%08x]"
			       "   R2:[%08x]   R3:[%08x]\n",
				prefix, r0, r1, r2, r3);
}

static void prepare_dbgmsg(struct s3cmci_host *host, struct mmc_command *cmd,
			   int stop)
{
	snprintf(host->dbgmsg_cmd, 300,
		 "#%u%s op:%i arg:0x%08x flags:0x08%x retries:%u",
		 host->ccnt, (stop ? " (STOP)" : ""),
		 cmd->opcode, cmd->arg, cmd->flags, cmd->retries);

	if (cmd->data) {
		snprintf(host->dbgmsg_dat, 300,
			 "#%u bsize:%u blocks:%u bytes:%u",
			 host->dcnt, cmd->data->blksz,
			 cmd->data->blocks,
			 cmd->data->blocks * cmd->data->blksz);
	} else {
		host->dbgmsg_dat[0] = '\0';
	}
}

static void dbg_dumpcmd(struct s3cmci_host *host, struct mmc_command *cmd,
			int fail)
{
	unsigned int dbglvl = fail ? dbg_fail : dbg_debug;

	if (!cmd)
		return;

	if (cmd->error == 0) {
		dbg(host, dbglvl, "CMD[OK] %s R0:0x%08x\n",
			host->dbgmsg_cmd, cmd->resp[0]);
	} else {
		dbg(host, dbglvl, "CMD[ERR %i] %s Status:%s\n",
			cmd->error, host->dbgmsg_cmd, host->status);
	}

	if (!cmd->data)
		return;

	if (cmd->data->error == 0) {
		dbg(host, dbglvl, "DAT[OK] %s\n", host->dbgmsg_dat);
	} else {
		dbg(host, dbglvl, "DAT[ERR %i] %s DCNT:0x%08x\n",
			cmd->data->error, host->dbgmsg_dat,
			readl(host->base + S3C2410_SDIDCNT));
	}
}
#else
static void dbg_dumpcmd(struct s3cmci_host *host,
			struct mmc_command *cmd, int fail) { }

static void prepare_dbgmsg(struct s3cmci_host *host, struct mmc_command *cmd,
			   int stop) { }

static void dbg_dumpregs(struct s3cmci_host *host, char *prefix) { }

#endif /* CONFIG_MMC_DEBUG */

static inline u32 enable_imask(struct s3cmci_host *host, u32 imask)
{
	u32 newmask;

	newmask = readl(host->base + host->sdiimsk);
	newmask |= imask;

	writel(newmask, host->base + host->sdiimsk);

	return newmask;
}

static inline u32 disable_imask(struct s3cmci_host *host, u32 imask)
{
	u32 newmask;

	newmask = readl(host->base + host->sdiimsk);
	newmask &= ~imask;

	writel(newmask, host->base + host->sdiimsk);

	return newmask;
}

static inline void clear_imask(struct s3cmci_host *host)
{
	ulong imsk, to_write = 0;
       
	imsk = readl(host->base + host->sdiimsk);
	if ((imsk & S3C2410_SDIIMSK_SDIOIRQ) && host->sdio_irq)
		to_write = S3C2410_SDIIMSK_SDIOIRQ;
	
	writel(to_write, host->base + host->sdiimsk);
}

static inline int get_data_buffer(struct s3cmci_host *host,
				  u32 *words, u32 **pointer)
{
	struct scatterlist *sg;

	if (host->pio_active == XFER_NONE)
		return -EINVAL;

	if ((!host->mrq) || (!host->mrq->data))
		return -EINVAL;

	if (host->pio_sgptr >= host->mrq->data->sg_len) {
		dbg(host, dbg_debug, "no more buffers (%i/%i)\n",
		      host->pio_sgptr, host->mrq->data->sg_len);
		return -EBUSY;
	}
	sg = &host->mrq->data->sg[host->pio_sgptr];

	*words = sg->length >> 2;
	*pointer = sg_virt(sg);

	host->pio_sgptr++;

	dbg(host, dbg_sg, "new buffer (%i/%i)\n",
	    host->pio_sgptr, host->mrq->data->sg_len);

	return 0;
}

static inline u32 fifo_count(struct s3cmci_host *host)
{
	u32 fifostat = readl(host->base + S3C2410_SDIFSTA);

	fifostat &= S3C2410_SDIFSTA_COUNTMASK;
	return fifostat >> 2;
}

static inline u32 fifo_free(struct s3cmci_host *host)
{
	u32 fifostat = readl(host->base + S3C2410_SDIFSTA);

	fifostat &= S3C2410_SDIFSTA_COUNTMASK;
	return (63 - fifostat) >> 2;
}

static void do_pio_read(struct s3cmci_host *host)
{
	int res;
	u32 fifo;
	void __iomem *from_ptr;

	/* write real prescaler to host, it might be set slow to fix */
	writel(host->prescaler, host->base + S3C2410_SDIPRE);

	from_ptr = host->base + host->sdidata;

	while ((fifo = fifo_count(host))) {
		if (!host->pio_bytes) {
			res = get_data_buffer(host, &host->pio_bytes,
					      &host->pio_ptr);
			if (res) {
				host->pio_active = XFER_NONE;
				host->complete_what = COMPLETION_FINALIZE;

				dbg(host, dbg_pio, "pio_read(): "
				    "complete (no more data).\n");
				return;
			}

			dbg(host, dbg_pio,
			    "pio_read(): new target: [%i]@[%p]\n",
			    host->pio_bytes, host->pio_ptr);
		}

		dbg(host, dbg_pio,
		    "pio_read(): fifo:[%02i] buffer:[%03i] dcnt:[%08X]\n",
		    fifo, host->pio_bytes,
		    readl(host->base + S3C2410_SDIDCNT));

		if (fifo > host->pio_bytes)
			fifo = host->pio_bytes;

		host->pio_bytes -= fifo;
		host->pio_count += fifo;

		while (fifo--)
			*(host->pio_ptr++) = readl(from_ptr);
	}

	if (!host->pio_bytes) {
		res = get_data_buffer(host, &host->pio_bytes, &host->pio_ptr);
		if (res) {
			dbg(host, dbg_pio,
			    "pio_read(): complete (no more buffers).\n");
			host->pio_active = XFER_NONE;
			host->complete_what = COMPLETION_FINALIZE;

			return;
		}
	}

	enable_imask(host,
		     S3C2410_SDIIMSK_RXFIFOHALF | S3C2410_SDIIMSK_RXFIFOLAST);
}

static void do_pio_write(struct s3cmci_host *host)
{
	void __iomem *to_ptr;
	int res;
	u32 fifo;

	to_ptr = host->base + host->sdidata;

	while ((fifo = fifo_free(host))) {
		if (!host->pio_bytes) {
			res = get_data_buffer(host, &host->pio_bytes,
							&host->pio_ptr);
			if (res) {
				dbg(host, dbg_pio,
				    "pio_write(): complete (no more data).\n");
				host->pio_active = XFER_NONE;

				return;
			}

			dbg(host, dbg_pio,
			    "pio_write(): new source: [%i]@[%p]\n",
			    host->pio_bytes, host->pio_ptr);

		}

		if (fifo > host->pio_bytes)
			fifo = host->pio_bytes;

		host->pio_bytes -= fifo;
		host->pio_count += fifo;

		while (fifo--)
			writel(*(host->pio_ptr++), to_ptr);
	}

	enable_imask(host, S3C2410_SDIIMSK_TXFIFOHALF);
}

static void pio_tasklet(unsigned long data)
{
	struct s3cmci_host *host = (struct s3cmci_host *) data;


	disable_irq(host->irq);

	if (host->pio_active == XFER_WRITE)
		do_pio_write(host);

	if (host->pio_active == XFER_READ)
		do_pio_read(host);

	if (host->complete_what == COMPLETION_FINALIZE) {
		clear_imask(host);
		if (host->pio_active != XFER_NONE) {
			dbg(host, dbg_err, "unfinished %s "
			    "- pio_count:[%u] pio_bytes:[%u]\n",
			    (host->pio_active == XFER_READ) ? "read" : "write",
			    host->pio_count, host->pio_bytes);

			if (host->mrq->data)
				host->mrq->data->error = -EINVAL;
		}

		finalize_request(host);
	} else
		enable_irq(host->irq);
}

/* The DMA-callback will call this function by errors or transfer completes */
static void dma_tasklet(unsigned long data)
{
	struct s3cmci_host *host = (struct s3cmci_host *) data;

	if (host->complete_what == COMPLETION_FINALIZE) {
		clear_imask(host);
		finalize_request(host);
	}
}

/*
 * ISR for SDI Interface IRQ
 * Communication between driver and ISR works as follows:
 *   host->mrq 			points to current request
 *   host->complete_what	Indicates when the request is considered done
 *     COMPLETION_CMDSENT	  when the command was sent
 *     COMPLETION_RSPFIN          when a response was received
 *     COMPLETION_XFERFINISH	  when the data transfer is finished
 *     COMPLETION_XFERFINISH_RSPFIN both of the above.
 *   host->complete_request	is the completion-object the driver waits for
 *
 * 1) Driver sets up host->mrq and host->complete_what
 * 2) Driver prepares the transfer
 * 3) Driver enables interrupts
 * 4) Driver starts transfer
 * 5) Driver waits for host->complete_rquest
 * 6) ISR checks for request status (errors and success)
 * 6) ISR sets host->mrq->cmd->error and host->mrq->data->error
 * 7) ISR completes host->complete_request
 * 8) ISR disables interrupts
 * 9) Driver wakes up and takes care of the request
 *
 * Note: "->error"-fields are expected to be set to 0 before the request
 *       was issued by mmc.c - therefore they are only set, when an error
 *       contition comes up
 */

static irqreturn_t s3cmci_irq(int irq, void *dev_id)
{
	struct s3cmci_host *host = dev_id;
	struct mmc_command *cmd;
	u32 mci_csta, mci_dsta, mci_fsta, mci_dcnt, mci_imsk;
	u32 mci_cclear, mci_dclear;
	unsigned long iflags;

	spin_lock_irqsave(&host->complete_lock, iflags);

	mci_csta = readl(host->base + S3C2410_SDICMDSTAT);
	mci_dsta = readl(host->base + S3C2410_SDIDSTA);
	mci_dcnt = readl(host->base + S3C2410_SDIDCNT);
	mci_fsta = readl(host->base + S3C2410_SDIFSTA);
	mci_imsk = readl(host->base + host->sdiimsk);
	mci_cclear = 0;
	mci_dclear = 0;

	printk_debug("IRQ: cmd 0x%08x | dsta 0x%08x | imsk 0x%08x\n",
		     mci_csta, mci_dsta, mci_imsk);

	if (mci_dsta & S3C2410_SDIDSTA_SDIOIRQDETECT) {
		printk_debug("SDIO IRQ detected\n");
		mmc_signal_sdio_irq(host->mmc);
		writel(S3C2410_SDIDSTA_SDIOIRQDETECT, host->base + S3C2410_SDIDSTA);
	}
	
	if ((host->complete_what == COMPLETION_NONE) ||
	    (host->complete_what == COMPLETION_FINALIZE)) {
		host->status = "nothing to complete";
		clear_imask(host);
		goto irq_out;
	}

	if (!host->mrq) {
		host->status = "no active mrq";
		clear_imask(host);
		goto irq_out;
	}

	cmd = host->cmd_is_stop ? host->mrq->stop : host->mrq->cmd;

	if (!cmd) {
		host->status = "no active cmd";
		clear_imask(host);
		goto irq_out;
	}

	if (!host->dodma) {
		if ((host->pio_active == XFER_WRITE) &&
		    (mci_fsta & S3C2410_SDIFSTA_TFDET)) {

			disable_imask(host, S3C2410_SDIIMSK_TXFIFOHALF);
			tasklet_schedule(&host->pio_tasklet);
			host->status = "pio tx";
		}

		if ((host->pio_active == XFER_READ) &&
		    (mci_fsta & S3C2410_SDIFSTA_RFDET)) {

			disable_imask(host,
				      S3C2410_SDIIMSK_RXFIFOHALF |
				      S3C2410_SDIIMSK_RXFIFOLAST);

			tasklet_schedule(&host->pio_tasklet);
			host->status = "pio rx";
		}
	}

	if (mci_csta & S3C2410_SDICMDSTAT_CMDTIMEOUT) {
		dbg(host, dbg_debug, "CMDSTAT: error CMDTIMEOUT\n");
		cmd->error = -ETIMEDOUT;
		host->status = "error: command timeout";
		goto fail_transfer;
	}

	if (mci_csta & S3C2410_SDICMDSTAT_CMDSENT) {

		/* @FIXME: Move the write-command to a correct place! (Luis G.) */
		mci_cclear |= S3C2410_SDICMDSTAT_CMDSENT;
		writel(mci_cclear, host->base + S3C2410_SDICMDSTAT);
		
		if (host->complete_what == COMPLETION_CMDSENT) {
			host->status = "ok: command sent";
			goto close_transfer;
		}
	}

	if (mci_csta & S3C2410_SDICMDSTAT_CRCFAIL) {
		if (cmd->flags & MMC_RSP_CRC) {
			if (host->mrq->cmd->flags & MMC_RSP_136) {
				dbg(host, dbg_irq,
				    "fixup: ignore CRC fail with long rsp\n");
			} else {
				/* note, we used to fail the transfer
				 * here, but it seems that this is just
				 * the hardware getting it wrong.
				 *
				 * cmd->error = -EILSEQ;
				 * host->status = "error: bad command crc";
				 * goto fail_transfer;
				*/
			}
		}

		mci_cclear |= S3C2410_SDICMDSTAT_CRCFAIL;
	}

	if (mci_csta & S3C2410_SDICMDSTAT_RSPFIN) {
		if (host->complete_what == COMPLETION_RSPFIN) {
			host->status = "ok: command response received";
			goto close_transfer;
		}

		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN)
			host->complete_what = COMPLETION_XFERFINISH;

		mci_cclear |= S3C2410_SDICMDSTAT_RSPFIN;
	}

	/* errors handled after this point are only relevant
	   when a data transfer is in progress */

	if (!cmd->data)
		goto clear_status_bits;

	/* Check for FIFO failure */
	if (host->is2440) {
		if (mci_fsta & S3C2440_SDIFSTA_FIFOFAIL) {
			dbg(host, dbg_err, "FIFO failure\n");
			host->mrq->data->error = -EILSEQ;
			host->status = "error: 2440 fifo failure";
			goto fail_transfer;
		}
	} else {
		if (mci_dsta & S3C2410_SDIDSTA_FIFOFAIL) {
			dbg(host, dbg_err, "FIFO failure\n");
			cmd->data->error = -EILSEQ;
			host->status = "error:  fifo failure";
			goto fail_transfer;
		}
	}

	if (mci_dsta & S3C2410_SDIDSTA_RXCRCFAIL) {
		dbg(host, dbg_err, "bad data crc (outgoing)\n");
		cmd->data->error = -EILSEQ;
		host->status = "error: bad data crc (outgoing)";
		goto fail_transfer;
	}

	if (mci_dsta & S3C2410_SDIDSTA_CRCFAIL) {
		dbg(host, dbg_err, "bad data crc (incoming)\n");
		cmd->data->error = -EILSEQ;
		host->status = "error: bad data crc (incoming)";
		goto fail_transfer;
	}

	if (mci_dsta & S3C2410_SDIDSTA_DATATIMEOUT) {
		dbg(host, dbg_err, "data timeout\n");
		cmd->data->error = -ETIMEDOUT;
		host->status = "error: data timeout";
		goto fail_transfer;
	}

	if (mci_dsta & S3C2410_SDIDSTA_XFERFINISH) {
		if (host->complete_what == COMPLETION_XFERFINISH) {
			host->status = "ok: data transfer completed";
			goto close_transfer;
		}

		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN)
			host->complete_what = COMPLETION_RSPFIN;

		mci_dclear |= S3C2410_SDIDSTA_XFERFINISH;
	}

clear_status_bits:
	writel(mci_cclear, host->base + S3C2410_SDICMDSTAT);
	writel(mci_dclear, host->base + S3C2410_SDIDSTA);

	goto irq_out;

fail_transfer:
	host->pio_active = XFER_NONE;

close_transfer:
	host->complete_what = COMPLETION_FINALIZE;

	clear_imask(host);

	/*
	 * If we have received an interrupt although we are waiting for the
	 * DMA-callback (cmd->data), then something went wrong with the last
	 * transfer. This happens when the card is removed before
	 * the card initialization was completed.
	 * (Luis Galdos)
	 */
	if (cmd->data && host->dodma && !cmd->data->error && !cmd->error) {
		host->dma_complete = 1;
		cmd->error = cmd->data->error = -EILSEQ;
		printk_debug("[SD] Waiting DMA (CMD %08x | DAT %08x)\n",
			     mci_csta, mci_dsta);
	}

	tasklet_schedule(&host->pio_tasklet);
	goto irq_out;

irq_out:
	dbg(host, dbg_irq,
	    "csta:0x%08x dsta:0x%08x fsta:0x%08x dcnt:0x%08x status:%s.\n",
	    mci_csta, mci_dsta, mci_fsta, mci_dcnt, host->status);

	spin_unlock_irqrestore(&host->complete_lock, iflags);
	return IRQ_HANDLED;

}

/*
 * ISR for the CardDetect Pin
*/

static int s3cmci_card_present(struct mmc_host *mmc);

static irqreturn_t s3cmci_irq_cd(int irq, void *dev_id)
{
	struct s3cmci_host *host = (struct s3cmci_host *)dev_id;
	int val;
	static int card_present;

	/*
	 * Get the current status of the GPIO for checking if the card state has
	 * really changed since the last interrupt. Otherwise it will generates
	 * more than one interrupt for the same state
	 * Luis Galdos
	 */
	val = s3cmci_card_present(host->mmc);
	if (val != card_present) {
		dbg(host, dbg_debug, "Card detect IRQ: %s\n", val ? "insert" : "remove");
		card_present = val;
		mmc_detect_change(host->mmc, msecs_to_jiffies(500));
	}

	return IRQ_HANDLED;
}

static void s3cmci_dma_done_callback(struct s3c2410_dma_chan *dma_ch,
				     void *buf_id, int size,
				     enum s3c2410_dma_buffresult result)
{
	struct s3cmci_host *host = buf_id;
	unsigned long iflags;
	u32 mci_csta, mci_dsta, mci_fsta, mci_dcnt;

	mci_csta = readl(host->base + S3C2410_SDICMDSTAT);
	mci_dsta = readl(host->base + S3C2410_SDIDSTA);
	mci_fsta = readl(host->base + S3C2410_SDIFSTA);
	mci_dcnt = readl(host->base + S3C2410_SDIDCNT);

	BUG_ON(!host->mrq);
	BUG_ON(!host->mrq->data);
	BUG_ON(!host->dmatogo);

	spin_lock_irqsave(&host->complete_lock, iflags);

	if (result != S3C2410_RES_OK) {
		struct mmc_request *mrq = host->mrq;
		
		if (mrq) {
			struct mmc_command *cmd;

			cmd = mrq->cmd;
			dbg(host, dbg_fail, "DMA FAILED: CMD%i csta=0x%08x dsta=0x%08x "
			    "fsta=0x%08x dcnt:0x%08x result:%i toGo:%u\n",
			    cmd->opcode, mci_csta, mci_dsta, mci_fsta,
			    mci_dcnt, result, host->dmatogo);
                } else 
			dbg(host, dbg_fail, "DMA FAILED: csta=0x%08x dsta=0x%08x "
			    "fsta=0x%08x dcnt:0x%08x result:0x%08x toGo:%u\n",
			    mci_csta, mci_dsta, mci_fsta,
			    mci_dcnt, result, host->dmatogo);

		goto fail_request;
	}

	host->dmatogo--;
	if (host->dmatogo) {
		printk_debug("DMA DONE  Size:%i DSTA:[%08x] "
			     "DCNT:[%08x] toGo:%u\n",
			     size, mci_dsta, mci_dcnt, host->dmatogo);

		if (mci_dsta & S3C2410_SDIDSTA_XFERFINISH) {
			printk_err("SDI has no more data, but DMA waits for more?\n");
			goto fail_request;
		}
		
		spin_unlock_irqrestore(&host->complete_lock, iflags);
		return;
	}

	printk_debug("DMA ENDE  Size:%i DSTA:[%08x] DCNT:[%08x]\n",
		     size, mci_dsta, mci_dcnt);

	host->dma_complete = 1;
	host->complete_what = COMPLETION_FINALIZE;

out:
	/* @FIXME: Check the performance modification through this delay */
	udelay(10);
	tasklet_schedule(&host->pio_tasklet);
	spin_unlock_irqrestore(&host->complete_lock, iflags);
	return;

fail_request:
	host->mrq->data->error = -EINVAL;
	host->complete_what = COMPLETION_FINALIZE;
	writel(0, host->base + host->sdiimsk);
	goto out;

}

static void finalize_request(struct s3cmci_host *host)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd = host->cmd_is_stop ? mrq->stop : mrq->cmd;
	int debug_as_failure = 0;

	spin_lock(&host->complete_lock);
	
	if (host->complete_what != COMPLETION_FINALIZE) {
		printk_debug("Nothing to complete!\n");
		goto exit_unlock;
	}

	if (!mrq) {
		printk_err("Empty MMC request found!\n");
		goto exit_unlock;
	}

	if (cmd->data && (cmd->error == 0) && (cmd->data->error == 0)) {
		if (host->dodma && (!host->dma_complete)) {
			printk_err("DMA complete missing (%i)\n", host->dma_complete);
			goto exit_unlock;
		}
	}

	/* Read response from controller. */
	cmd->resp[0] = readl(host->base + S3C2410_SDIRSP0);
	cmd->resp[1] = readl(host->base + S3C2410_SDIRSP1);
	cmd->resp[2] = readl(host->base + S3C2410_SDIRSP2);
	cmd->resp[3] = readl(host->base + S3C2410_SDIRSP3);

	writel(host->prescaler, host->base + S3C2410_SDIPRE);

	if (cmd->error)
		debug_as_failure = 1;

	if (cmd->data && cmd->data->error)
		debug_as_failure = 1;

	dbg_dumpcmd(host, cmd, debug_as_failure);

	/* Cleanup controller */
	writel(0, host->base + S3C2410_SDICMDARG);
	writel(0, host->base + S3C2410_SDICMDCON);
	writel(S3C2410_SDIDCON_STOP, host->base + S3C2410_SDIDCON);
	if (!host->sdio_irq) {
		writel(0, host->base + host->sdiimsk);
	}
	
	if (cmd->data && cmd->error)
		cmd->data->error = cmd->error;

	if (cmd->data && cmd->data->stop && (!host->cmd_is_stop)) {
		host->cmd_is_stop = 1;
		s3cmci_send_request(host->mmc);
		goto exit_unlock;
	}

	/* If we have no data transfer we are finished here */
	if (!mrq->data)
		goto request_done;

	/* Calulate the amout of bytes transfer if there was no error */
	if (mrq->data->error == 0) {
		mrq->data->bytes_xfered =
			(mrq->data->blocks * mrq->data->blksz);
	} else {
		mrq->data->bytes_xfered = 0;
	}

	/* If we had an error while transfering data we flush the
	 * DMA channel and the fifo to clear out any garbage. */
	if (mrq->data->error != 0) {
		if (host->dodma)
			s3c2410_dma_ctrl(host->dma, S3C2410_DMAOP_FLUSH);

		if (host->is2440) {
			/* Clear failure register and reset fifo. */
			writel(S3C2440_SDIFSTA_FIFORESET |
			       S3C2440_SDIFSTA_FIFOFAIL,
			       host->base + S3C2410_SDIFSTA);
		} else {
			u32 mci_con;

			/* reset fifo */
			mci_con = readl(host->base + S3C2410_SDICON);
			mci_con |= S3C2410_SDICON_FIFORESET;

			writel(mci_con, host->base + S3C2410_SDICON);
		}
	}

request_done:
	host->complete_what = COMPLETION_NONE;
	host->mrq = NULL;
	mmc_request_done(host->mmc, mrq);

 exit_unlock:
	spin_unlock(&host->complete_lock);

}

static void s3cmci_dma_setup(struct s3cmci_host *host,
			     enum s3c2410_dmasrc source)
{
	static enum s3c2410_dmasrc last_source = -1;
	static int setup_ok = 0;

	if (last_source == source)
		return;

	last_source = source;

	s3c2410_dma_devconfig(host->dma, source,
			      S3C2410_DISRCC_INC | S3C2410_DISRCC_APB,
			      host->mem->start + host->sdidata);

	if (!setup_ok) {
		printk_debug("Setting up the DMA channel!\n");
		s3c2410_dma_config(host->dma,
				   4,
				   S3C2410_DCON_HANDSHAKE |
				   S3C2410_DCON_SYNC_PCLK |
				   S3C2410_DCON_HWTRIG);

		s3c2410_dma_set_buffdone_fn(host->dma,
					    s3cmci_dma_done_callback);
		s3c2410_dma_ctrl(host->dma, S3C2410_DMAOP_FLUSH);
		setup_ok = 1;
	}
}

static void s3cmci_send_command(struct s3cmci_host *host,
					struct mmc_command *cmd)
{
	u32 ccon, imsk;

	imsk  = S3C2410_SDIIMSK_CRCSTATUS | S3C2410_SDIIMSK_CMDTIMEOUT |
		S3C2410_SDIIMSK_RESPONSEND | S3C2410_SDIIMSK_CMDSENT |
		S3C2410_SDIIMSK_RESPONSECRC;

	enable_imask(host, imsk);

	if (cmd->data)
		host->complete_what = COMPLETION_XFERFINISH_RSPFIN;
	else if (cmd->flags & MMC_RSP_PRESENT)
		host->complete_what = COMPLETION_RSPFIN;
	else
		host->complete_what = COMPLETION_CMDSENT;

	writel(cmd->arg, host->base + S3C2410_SDICMDARG);

	ccon  = cmd->opcode & S3C2410_SDICMDCON_INDEX;
	ccon |= S3C2410_SDICMDCON_SENDERHOST | S3C2410_SDICMDCON_CMDSTART;

	if (cmd->flags & MMC_RSP_PRESENT)
		ccon |= S3C2410_SDICMDCON_WAITRSP;

	if (cmd->flags & MMC_RSP_136)
		ccon |= S3C2410_SDICMDCON_LONGRSP;

	writel(ccon, host->base + S3C2410_SDICMDCON);
}

static int s3cmci_setup_data(struct s3cmci_host *host, struct mmc_data *data)
{
	u32 dcon, imsk, stoptries = 3;

	/* write DCON register */

	if (!data) {
		writel(0, host->base + S3C2410_SDIDCON);
		return 0;
	}

	if ((data->blksz & 3) != 0) {
		/* We cannot deal with unaligned blocks with more than
		 * one block being transfered. */

		if (data->blocks > 1)
			return -EINVAL;

		/* No support yet for non-word block transfers. */
		return -EINVAL;
	}

	while (readl(host->base + S3C2410_SDIDSTA) &
	       (S3C2410_SDIDSTA_TXDATAON | S3C2410_SDIDSTA_RXDATAON)) {

		dbg(host, dbg_err,
		    "mci_setup_data() transfer stillin progress.\n");

		writel(S3C2410_SDIDCON_STOP, host->base + S3C2410_SDIDCON);
		s3cmci_reset(host);

		if ((stoptries--) == 0) {
			dbg_dumpregs(host, "DRF");
			return -EINVAL;
		}
	}

	dcon  = data->blocks & S3C2410_SDIDCON_BLKNUM_MASK;

	if (host->dodma)
		dcon |= S3C2410_SDIDCON_DMAEN;

	if (host->bus_width == MMC_BUS_WIDTH_4)
		dcon |= S3C2410_SDIDCON_WIDEBUS;

	if (!(data->flags & MMC_DATA_STREAM))
		dcon |= S3C2410_SDIDCON_BLOCKMODE;

	if (data->flags & MMC_DATA_WRITE) {
		dcon |= S3C2410_SDIDCON_TXAFTERRESP;
		dcon |= S3C2410_SDIDCON_XFER_TXSTART;
	}

	if (data->flags & MMC_DATA_READ) {
		dcon |= S3C2410_SDIDCON_RXAFTERCMD;
		dcon |= S3C2410_SDIDCON_XFER_RXSTART;
	}

	if (host->is2440) {
		dcon |= S3C2440_SDIDCON_DS_WORD;
		dcon |= S3C2440_SDIDCON_DATSTART;
	}

	writel(dcon, host->base + S3C2410_SDIDCON);

	/* write BSIZE register */
	writel(data->blksz, host->base + S3C2410_SDIBSIZE);

	/* add to IMASK register */
	imsk = S3C2410_SDIIMSK_FIFOFAIL | S3C2410_SDIIMSK_DATACRC |
		S3C2410_SDIIMSK_DATATIMEOUT;

	/* By DMA-support we will use the DMA-callback for the transfer-complete */
	if (!host->dodma)
		imsk |= S3C2410_SDIIMSK_DATAFINISH;

	enable_imask(host, imsk);

	/* write TIMER register */

	if (host->is2440) {
		writel(0x007FFFFF, host->base + S3C2410_SDITIMER);
	} else {
		writel(0x0000FFFF, host->base + S3C2410_SDITIMER);

		/* FIX: set slow clock to prevent timeouts on read */
		if (data->flags & MMC_DATA_READ)
			writel(0xFF, host->base + S3C2410_SDIPRE);
	}

	return 0;
}

#define BOTH_DIR (MMC_DATA_WRITE | MMC_DATA_READ)

static int s3cmci_prepare_pio(struct s3cmci_host *host, struct mmc_data *data)
{
	int rw = (data->flags & MMC_DATA_WRITE) ? 1 : 0;

	BUG_ON((data->flags & BOTH_DIR) == BOTH_DIR);

	host->pio_sgptr = 0;
	host->pio_bytes = 0;
	host->pio_count = 0;
	host->pio_active = rw ? XFER_WRITE : XFER_READ;

	if (rw) {
		do_pio_write(host);
		enable_imask(host, S3C2410_SDIIMSK_TXFIFOHALF);
	} else {
		enable_imask(host, S3C2410_SDIIMSK_RXFIFOHALF
			     | S3C2410_SDIIMSK_RXFIFOLAST);
	}

	return 0;
}

static int s3cmci_prepare_dma(struct s3cmci_host *host, struct mmc_data *data)
{
	int dma_len, i;
	int rw = (data->flags & MMC_DATA_WRITE) ? 1 : 0;
	int retval;
	
	BUG_ON((data->flags & BOTH_DIR) == BOTH_DIR);

	printk_debug("New DMA transfer | %i Blks | %i Blksz\n",
		     data->blocks, data->blksz);

	s3c2410_dma_ctrl(host->dma, S3C2410_DMAOP_FLUSH);
	s3cmci_dma_setup(host, rw ? S3C2410_DMASRC_MEM : S3C2410_DMASRC_HW);

	dma_len = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
			     (rw) ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

	if (dma_len == 0) {
		printk_err("dma_map_sg failed, no memory available?\n");
		retval = -ENOMEM;
		goto exit_all;
	}

	host->dmatogo = dma_len;

	for (i = 0; i < dma_len; i++) {
		int res;

		printk_debug("Enqueue %i @ 0x%08x | %u bytes\n", i,
			sg_dma_address(&data->sg[i]),
			sg_dma_len(&data->sg[i]));

		res = s3c2410_dma_enqueue(host->dma, (void *) host,
					  sg_dma_address(&data->sg[i]),
					  sg_dma_len(&data->sg[i]));

		if (res) {
			s3c2410_dma_ctrl(host->dma, S3C2410_DMAOP_FLUSH);
			printk_err("Couldn't enqueue a DMA-buffer\n");
			retval = -EBUSY;
			goto exit_all;
		}
	}

	/*
	 * Disable the data counter interrupt, then the DMA-callback will be
	 * responsible for finalizing the request
	 */
	disable_imask(host, S3C2410_SDIIMSK_DATAFINISH |
		S3C2410_SDIIMSK_RXFIFOHALF |
		S3C2410_SDIIMSK_RXFIFOFULL |
		S3C2410_SDIIMSK_RXFIFOLAST |
		S3C2410_SDIIMSK_TXFIFOEMPTY |
		S3C2410_SDIIMSK_TXFIFOHALF);

	host->dma_complete = 0;
	s3c2410_dma_ctrl(host->dma, S3C2410_DMAOP_START);

	retval = 0;
	
 exit_all:
	
	return retval;
}

static void s3cmci_send_request(struct mmc_host *mmc)
{
	struct s3cmci_host *host = mmc_priv(mmc);
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd = host->cmd_is_stop ? mrq->stop : mrq->cmd;

	host->ccnt++;
	prepare_dbgmsg(host, cmd, host->cmd_is_stop);

	/* @XXX: Sending the switch command with data leads to a DMA-failure, why? */
	if (cmd->opcode == MMC_SWITCH && cmd->data)
		udelay(200);

	/* Clear command, data and fifo status registers
	   Fifo clear only necessary on 2440, but doesn't hurt on 2410
	*/
	writel(0xFFFFFFFF, host->base + S3C2410_SDICMDSTAT);
	writel(0xFFFFFFFF, host->base + S3C2410_SDIDSTA);
	writel(0xFFFFFFFF, host->base + S3C2410_SDIFSTA);

	if (cmd->data) {
		int res = s3cmci_setup_data(host, cmd->data);

		printk_debug("CMD%u: %s transfer | %i blks | %u blksz\n",
			     cmd->opcode,
			     (cmd->data->flags & MMC_DATA_READ) ? "RX" : "TX",
			     cmd->data->blocks, cmd->data->blksz);
		
		host->dcnt++;

		if (res) {
			dbg(host, dbg_err, "setup data error %d\n", res);
			cmd->error = res;
			cmd->data->error = res;

			mmc_request_done(mmc, mrq);
			return;
		}

		if (host->dodma)
			res = s3cmci_prepare_dma(host, cmd->data);
		else
			res = s3cmci_prepare_pio(host, cmd->data);

		if (res) {
			dbg(host, dbg_err, "data prepare error %d\n", res);
			cmd->error = res;
			cmd->data->error = res;

			mmc_request_done(mmc, mrq);
			return;
		}
	}

	/* Enable Interrupt */
	if (!host->dodma)
		enable_irq(host->irq);

	/* Send command */
	s3cmci_send_command(host, cmd);

	/*
	 * If the host supports DMA, then disable the data finish interrupts and
	 * the FIFO-interrupts, then the DMA-controller will handle the data
	 * transfer and will finish the transfer if no errors ocurrs
	 * @XXX: This is probably not the correct place for this operation
	 * (Luis Galdos)
	 */
	if (cmd->data && host->dodma)
		disable_imask(host, S3C2410_SDIIMSK_DATAFINISH |
			      S3C2410_SDIIMSK_RXFIFOHALF |
			      S3C2410_SDIIMSK_RXFIFOFULL |
			      S3C2410_SDIIMSK_RXFIFOLAST |
			      S3C2410_SDIIMSK_TXFIFOEMPTY |
			      S3C2410_SDIIMSK_TXFIFOHALF);

}

static int s3cmci_card_present(struct mmc_host *mmc)
{
	struct s3cmci_host *host = mmc_priv(mmc);
	struct s3c24xx_mci_pdata *pdata = host->pdata;
	int ret;

	if (pdata->gpio_detect == 0)
		return -ENOSYS;

	ret = s3c2410_gpio_getpin(pdata->gpio_detect) ? 0 : 1;
	return ret ^ pdata->detect_invert;
}

static void s3cmci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct s3cmci_host *host = mmc_priv(mmc);

	host->status = "mmc request";
	host->cmd_is_stop = 0;
	host->mrq = mrq;

	if (s3cmci_card_present(mmc) == 0) {
		dbg(host, dbg_debug, "%s: no medium present\n", __func__);
		host->mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
	} else
		s3cmci_send_request(mmc);
}

static void s3cmci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct s3cmci_host *host = mmc_priv(mmc);
	u32 mci_psc, mci_con;
	int waitclk;

	/* Set the power state */

	mci_con = readl(host->base + S3C2410_SDICON);

	switch (ios->power_mode) {
	case MMC_POWER_ON:
	case MMC_POWER_UP:
		if (host->pdata->set_power)
			host->pdata->set_power(ios->power_mode, ios->vdd);

		if (!host->is2440)
			mci_con |= S3C2410_SDICON_FIFORESET;

		break;

	case MMC_POWER_OFF:
	default:
		if (host->is2440)
			mci_con |= S3C2440_SDICON_SDRESET;

		if (host->pdata->set_power)
			host->pdata->set_power(ios->power_mode, ios->vdd);

		break;
	}

	/* Set CLOCK_ENABLE */
	if (ios->clock)
		mci_con |= S3C2410_SDICON_CLOCKTYPE;
	else
		mci_con &= ~S3C2410_SDICON_CLOCKTYPE;

	writel(mci_con, host->base + S3C2410_SDICON);


	/* Set clock */
	for (mci_psc = 0; mci_psc < 255; mci_psc++) {
		host->real_rate = host->clk_rate / (host->clk_div*(mci_psc+1));

		if (host->real_rate <= ios->clock)
			break;
	}

	if (mci_psc > 255)
		mci_psc = 255;


	/* If requested clock is 0, real_rate will be 0, too */
	if (ios->clock == 0)
		host->real_rate = 0;

	host->prescaler = mci_psc;
	writel(host->prescaler, host->base + S3C2410_SDIPRE);

	/*
	 * According to the data sheet first configure the register SDICON and
	 * wait at least 74 SDCLK before initializing the SD card.
	 * (Luis Galdos)
	 */
	waitclk = 1;
	if (host->real_rate)
		waitclk = 100 * (1000000 / host->real_rate);

	if (waitclk > 1000)
		mdelay(waitclk / 1000);
	else
		udelay(waitclk);

	
	if ((ios->power_mode == MMC_POWER_ON) ||
	    (ios->power_mode == MMC_POWER_UP)) {
		dbg(host, dbg_debug, "running at %lukHz (requested: %ukHz).\n",
			host->real_rate/1000, ios->clock/1000);
	} else {
		dbg(host, dbg_debug, "powered down.\n");
	}

	host->bus_width = ios->bus_width;
}

static void s3cmci_reset(struct s3cmci_host *host)
{
	u32 con = readl(host->base + S3C2410_SDICON);

	con |= S3C2440_SDICON_SDRESET;
	writel(con, host->base + S3C2410_SDICON);
}

static int s3cmci_get_ro(struct mmc_host *mmc)
{
	struct s3cmci_host *host = mmc_priv(mmc);
	struct s3c24xx_mci_pdata *pdata = host->pdata;
	int ret;

	if (pdata->gpio_wprotect == 0)
		return 0;

	ret = s3c2410_gpio_getpin(pdata->gpio_wprotect);

	if (pdata->wprotect_invert)
		ret = !ret;

	return ret;
}

/* Here only mask or unmask the SDIO interrupt */
static void s3cmci_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct s3cmci_host *host;
	ulong con, imsk;
	
	if (!(host = mmc_priv(mmc))) {
		printk_err("NULL host pointer found!\n");
		return;
	}
	
	printk_debug("%s SDIO IRQ\n", enable ? "Enabling" : "Disabling");
	
	con = readl(host->base + S3C2410_SDICON);
	imsk = readl(host->base + host->sdiimsk);
	
	if (enable) {
		host->sdio_irq = 1;
		imsk |= S3C2410_SDIIMSK_SDIOIRQ;
		con |= S3C2410_SDICON_SDIOIRQ;
	} else {
		host->sdio_irq = 0;
		
		imsk |= S3C2410_SDIIMSK_SDIOIRQ;
		con &= ~S3C2410_SDICON_SDIOIRQ;
	}
	
	writel(con, host->base + S3C2410_SDICON);
	writel(imsk, host->base + host->sdiimsk);
}

static struct mmc_host_ops s3cmci_ops = {
	.request	 = s3cmci_request,
	.set_ios	 = s3cmci_set_ios,
	.get_ro		 = s3cmci_get_ro,
	.enable_sdio_irq = s3cmci_enable_sdio_irq,
};

static struct s3c24xx_mci_pdata s3cmci_def_pdata = {
	/* This is currently here to avoid a number of if (host->pdata)
	 * checks. Any zero fields to ensure reaonable defaults are picked. */
};

static int __devinit s3cmci_probe(struct platform_device *pdev, int is2440)
{
	struct s3cmci_host *host;
	struct mmc_host	*mmc;
	int ret;

	mmc = mmc_alloc_host(sizeof(struct s3cmci_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto probe_out;
	}

	host = mmc_priv(mmc);
	host->mmc 	= mmc;
	host->pdev	= pdev;
	host->is2440	= is2440;

	host->pdata = pdev->dev.platform_data;
	if (!host->pdata) {
		pdev->dev.platform_data = &s3cmci_def_pdata;
		host->pdata = &s3cmci_def_pdata;
	}

	if (is2440) {
		host->sdiimsk	= S3C2440_SDIIMSK;
		host->sdidata	= S3C2440_SDIDATA;
		host->clk_div	= 1;
	} else {
		host->sdiimsk	= S3C2410_SDIIMSK;
		host->sdidata	= S3C2410_SDIDATA;
		host->clk_div	= 2;
	}

	/* Configure the DMA-support depending on the passed platform configuration */
	host->dodma		= (host->pdata->dma_enable) ? 1 : 0;
	host->complete_what 	= COMPLETION_NONE;
	host->pio_active 	= XFER_NONE;

	/* DMA-channel to use (include/asm-arm/arch-s3c2410/dma.h) */
	host->dma		= DMACH_SDI;

	spin_lock_init(&host->complete_lock);

	/* Depending on the DMA-support use different tasklet-functions */
	if (!host->dodma)
		tasklet_init(&host->pio_tasklet, pio_tasklet, (unsigned long) host);
	else
		tasklet_init(&host->pio_tasklet, dma_tasklet, (unsigned long) host);
	
	host->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!host->mem) {
		dev_err(&pdev->dev,
			"failed to get io memory region resouce.\n");

		ret = -ENOENT;
		goto probe_free_host;
	}

	host->mem = request_mem_region(host->mem->start,
				       RESSIZE(host->mem), pdev->name);

	if (!host->mem) {
		dev_err(&pdev->dev, "failed to request io memory region.\n");
		ret = -ENOENT;
		goto probe_free_host;
	}

	host->base = ioremap(host->mem->start, RESSIZE(host->mem));
	if (!host->base) {
		dev_err(&pdev->dev, "failed to ioremap() io memory region.\n");
		ret = -EINVAL;
		goto probe_free_mem_region;
	}

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq == 0) {
		dev_err(&pdev->dev, "failed to get interrupt resouce.\n");
		ret = -EINVAL;
		goto probe_iounmap;
	}

	if (request_irq(host->irq, s3cmci_irq, 0, DRIVER_NAME, host)) {
		dev_err(&pdev->dev, "failed to request mci interrupt.\n");
		ret = -ENOENT;
		goto probe_iounmap;
	}

	/* We get spurious interrupts even when we have set the IMSK
	 * register to ignore everything, so use disable_irq() to make
	 * ensure we don't lock the system with un-serviceable requests. */

	disable_irq(host->irq);

	host->irq_cd = s3c2410_gpio_getirq(host->pdata->gpio_detect);

	if (host->irq_cd >= 0) {
		if (request_irq(host->irq_cd, s3cmci_irq_cd,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				DRIVER_NAME, host)) {
			dev_err(&pdev->dev, "can't get card detect irq.\n");
			ret = -ENOENT;
			goto probe_free_irq;
		}
	} else {
		dev_warn(&pdev->dev, "host detect has no irq available\n");
		s3c2410_gpio_cfgpin(host->pdata->gpio_detect,
				    S3C2410_GPIO_INPUT);
	}

	if (host->pdata->gpio_wprotect)
		s3c2410_gpio_cfgpin(host->pdata->gpio_wprotect,
				    S3C2410_GPIO_INPUT);

	if (host->dodma) {
		if (s3c2410_dma_request(host->dma, &s3cmci_dma_client, NULL) < 0) {
			dev_err(&pdev->dev, "unable to get DMA channel.\n");
			ret = -EBUSY;
			goto probe_free_irq_cd;
		}
	}

	host->clk = clk_get(&pdev->dev, "sdi");
	if (IS_ERR(host->clk)) {
		dev_err(&pdev->dev, "failed to find clock source.\n");
		ret = PTR_ERR(host->clk);
		host->clk = NULL;
		goto probe_free_dmach;
	}

	ret = clk_enable(host->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clock source.\n");
		goto clk_free;
	}

	host->clk_rate = clk_get_rate(host->clk);

	mmc->ops 	= &s3cmci_ops;
	mmc->ocr_avail	= MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps	= MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ;
	mmc->f_min 	= host->clk_rate / (host->clk_div * 256);
	mmc->f_max 	= host->clk_rate / host->clk_div;

	if (host->pdata->ocr_avail)
		mmc->ocr_avail = host->pdata->ocr_avail;

	mmc->max_blk_count	= 4095;
	mmc->max_blk_size	= 4095;
	mmc->max_req_size	= 4095 * 512;
	mmc->max_seg_size	= mmc->max_req_size;

	mmc->max_phys_segs	= 128;
	mmc->max_hw_segs	= 128;

	dbg(host, dbg_debug,
	    "probe: mode:%s mapped mci_base:%p irq:%u irq_cd:%u dma:%u.\n",
	    (host->is2440?"2440":""),
	    host->base, host->irq, host->irq_cd, host->dma);

	ret = mmc_add_host(mmc);
	if (ret) {
		dev_err(&pdev->dev, "failed to add mmc host.\n");
		goto free_dmabuf;
	}

	/* @FIXME: Get the GPIO-list from the platform-data */
	s3c2410_gpio_cfgpin(S3C2410_GPE5, S3C2410_GPE5_SDCLK);
	s3c2410_gpio_cfgpin(S3C2410_GPE6, S3C2410_GPE6_SDCMD);
	s3c2410_gpio_cfgpin(S3C2410_GPE7, S3C2410_GPE7_SDDAT0);
	s3c2410_gpio_cfgpin(S3C2410_GPE8, S3C2410_GPE8_SDDAT1);
	s3c2410_gpio_cfgpin(S3C2410_GPE9, S3C2410_GPE9_SDDAT2);
	s3c2410_gpio_cfgpin(S3C2410_GPE10, S3C2410_GPE10_SDDAT3);
	
	platform_set_drvdata(pdev, mmc);
	dev_info(&pdev->dev, "initialisation done.\n");

        /* Enable the wakeup for this device (Luis Galdos) */
	device_init_wakeup(&pdev->dev, 1); 
        device_set_wakeup_enable(&pdev->dev, 0);
	
	enable_irq(host->irq);

	return 0;

 free_dmabuf:
	clk_disable(host->clk);

 clk_free:
	clk_put(host->clk);

 probe_free_dmach:
	if (host->dodma)
		s3c2410_dma_free(host->dma, &s3cmci_dma_client);
	
 probe_free_irq_cd:
	if (host->irq_cd >= 0)
		free_irq(host->irq_cd, host);

 probe_free_irq:
	free_irq(host->irq, host);

 probe_iounmap:
	iounmap(host->base);

 probe_free_mem_region:
	release_mem_region(host->mem->start, RESSIZE(host->mem));

 probe_free_host:
	mmc_free_host(mmc);
 probe_out:
	return ret;
}

static void s3cmci_shutdown(struct platform_device *pdev)
{
	struct mmc_host	*mmc = platform_get_drvdata(pdev);
	struct s3cmci_host *host = mmc_priv(mmc);

	if (host->irq_cd >= 0)
		free_irq(host->irq_cd, host);

	mmc_remove_host(mmc);
	clk_disable(host->clk);
}

/*
 * Don't remove the MMC-host when going to shutdown the system. This can lead to some
 * failures when the host has mounted a card with our root file system.
 * (Luis Galdos)
 */
static void s3cmci_2440_shutdown(struct platform_device *pdev)
{
	struct mmc_host	*mmc = platform_get_drvdata(pdev);
	struct s3cmci_host *host = mmc_priv(mmc);

	if (host->irq_cd >= 0)
		free_irq(host->irq_cd, host);

	clk_disable(host->clk);
}

static int __devexit s3cmci_remove(struct platform_device *pdev)
{
	struct mmc_host		*mmc  = platform_get_drvdata(pdev);
	struct s3cmci_host	*host = mmc_priv(mmc);

	s3cmci_shutdown(pdev);

	clk_put(host->clk);

	tasklet_disable(&host->pio_tasklet);

	/* Free the DMA-channel */
	if (host->dodma)
		s3c2410_dma_free(host->dma, &s3cmci_dma_client);

	free_irq(host->irq, host);

	iounmap(host->base);
	release_mem_region(host->mem->start, RESSIZE(host->mem));

	mmc_free_host(mmc);
	return 0;
}

static int __devinit s3cmci_2410_probe(struct platform_device *dev)
{
	return s3cmci_probe(dev, 0);
}

static int __devinit s3cmci_2412_probe(struct platform_device *dev)
{
	return s3cmci_probe(dev, 1);
}

static int __devinit s3cmci_2440_probe(struct platform_device *dev)
{
	return s3cmci_probe(dev, 1);
}

#ifdef CONFIG_PM

static int s3cmci_suspend(struct platform_device *pdev, pm_message_t state)
{
	int retval;
	struct mmc_host *mmc;
	struct s3cmci_host *host;

	mmc = platform_get_drvdata(pdev);
	host = mmc_priv(mmc);

 	retval = mmc_suspend_host(mmc, state);
	if (retval)
		goto exit_suspend;
	
	/* Check if the wakeup is enabled. IN that case configure it as ext wakeup */
	if (device_may_wakeup(&pdev->dev)) {
		struct s3c24xx_mci_pdata *pdata;
		unsigned long detect;

		pdata = host->pdata;
		
		retval = enable_irq_wake(host->irq_cd);
		if (retval) {
			dev_err(&pdev->dev, "Couldn't enable wake IRQ %i\n",
				host->irq_cd);
			goto exit_suspend;
		}

		/*
		 * Reconfigure the card for generating the wakeup when a new
		 * card is plugged into the slot
		 */
		detect = (pdata->detect_invert) ? (IRQF_TRIGGER_RISING) :
			(IRQF_TRIGGER_FALLING);
		set_irq_type(host->irq_cd, detect);
	}

	writel(S3C2440_SDICON_SDRESET | S3C2410_SDIDCON_STOP,
	       host->base + S3C2410_SDICON);
	
	clk_disable(host->clk);
exit_suspend:
	return retval;
}

static int s3cmci_resume(struct platform_device *pdev)
{
	struct s3cmci_host *host;
	struct mmc_host *mmc;
	int retval;

	mmc = platform_get_drvdata(pdev);
	host = mmc_priv(mmc);

	if (device_may_wakeup(&pdev->dev)) {
		retval = disable_irq_wake(host->irq_cd);
		if (retval) {
			dev_err(&pdev->dev, "Couldn't disable wake IRQ %i\n",
				host->irq_cd);
			goto exit_resume;
		}

		set_irq_type(host->irq_cd, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	}

	clk_enable(host->clk);
	
	/* @FIXME: Why do we need to free que DMA-channel? */
	s3c2410_dma_free(host->dma, &s3cmci_dma_client);
	s3c2410_dma_request(host->dma, &s3cmci_dma_client, NULL);

	/*
	 * By unsafe resumes we MUST check the card state at this point, then the
	 * higher MMC-layer is probably transferring some kind of data to the
	 * block device that doesn't exist any more.
	 */
#if defined(CONFIG_MMC_UNSAFE_RESUME)
	if (s3cmci_card_present(mmc))
		retval = mmc_resume_host(mmc);
	else {
		retval = 0;
		mmc_detect_change(mmc, msecs_to_jiffies(500));
	}
#else
	retval = mmc_resume_host(mmc);
#endif /* CONFIG_MMC_UNSAFE_RESUME */
	
exit_resume:
	return retval;
}

#else /* CONFIG_PM */
#define s3cmci_suspend NULL
#define s3cmci_resume NULL
#endif /* CONFIG_PM */


static struct platform_driver s3cmci_2410_driver = {
	.driver.name	= "s3c2410-sdi",
	.driver.owner	= THIS_MODULE,
	.probe		= s3cmci_2410_probe,
	.remove		= __devexit_p(s3cmci_remove),
	.shutdown	= s3cmci_shutdown,
	.suspend	= s3cmci_suspend,
	.resume		= s3cmci_resume,
};

static struct platform_driver s3cmci_2412_driver = {
	.driver.name	= "s3c2412-sdi",
	.driver.owner	= THIS_MODULE,
	.probe		= s3cmci_2412_probe,
	.remove		= __devexit_p(s3cmci_remove),
	.shutdown	= s3cmci_shutdown,
	.suspend	= s3cmci_suspend,
	.resume		= s3cmci_resume,
};

static struct platform_driver s3cmci_2440_driver = {
	.driver.name	= "s3c2440-sdi",
	.driver.owner	= THIS_MODULE,
	.probe		= s3cmci_2440_probe,
	.remove		= __devexit_p(s3cmci_remove),
	.shutdown	= s3cmci_2440_shutdown,
	.suspend	= s3cmci_suspend,
	.resume		= s3cmci_resume,
};


static int __init s3cmci_init(void)
{
	platform_driver_register(&s3cmci_2410_driver);
	platform_driver_register(&s3cmci_2412_driver);
	platform_driver_register(&s3cmci_2440_driver);
	return 0;
}

static void __exit s3cmci_exit(void)
{
	platform_driver_unregister(&s3cmci_2410_driver);
	platform_driver_unregister(&s3cmci_2412_driver);
	platform_driver_unregister(&s3cmci_2440_driver);
}

module_init(s3cmci_init);
module_exit(s3cmci_exit);

MODULE_DESCRIPTION("Samsung S3C MMC/SD Card Interface driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Thomas Kleffel <tk@maintech.de>");
MODULE_ALIAS("platform:s3c2410-sdi");
MODULE_ALIAS("platform:s3c2412-sdi");
MODULE_ALIAS("platform:s3c2440-sdi");
