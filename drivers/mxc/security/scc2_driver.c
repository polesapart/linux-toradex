/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*! @file scc2_driver.c
 *
 * This is the driver code for the Security Controller (SCC).  It has no device
 * driver interface, so no user programs may access it.  Its interaction with
 * the Linux kernel is from calls to #scc_init() when the driver is loaded, and
 * #scc_cleanup() should the driver be unloaded.  The driver uses locking and
 * (task-sleep/task-wakeup) functions of the kernel.  It also registers itself
 * to handle the interrupt line(s) from the SCC.
 *
 * Other drivers in the kernel may use the remaining API functions to get at
 * the services of the SCC.  The main service provided is the Secure Memory,
 * which allows encoding and decoding of secrets with a per-chip secret key.
 *
 * The SCC is single-threaded, and so is this module.  When the scc_crypt()
 * routine is called, it will lock out other accesses to the function.  If
 * another task is already in the module, the subsequent caller will spin on a
 * lock waiting for the other access to finish.
 *
 * Note that long crypto operations could cause a task to spin for a while,
 * preventing other kernel work (other than interrupt processing) to get done.
 *
 * The external (kernel module) interface is through the following functions:
 * @li scc_get_configuration()
 * @li scc_crypt()
 * @li scc_zeroize_memories()
 * @li scc_monitor_security_failure()
 * @li scc_stop_monitoring_security_failure()
 * @li scc_set_sw_alarm()
 * @li scc_read_register()
 * @li scc_write_register()
 *
 * All other functions are internal to the driver.
 */

#include "scc2_internals.h"
#include "sahara2/include/portable_os.h"
#include <linux/delay.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18))

#include <linux/device.h>
#include <asm/arch/clock.h>
#include <linux/device.h>

#else

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>

#endif

#include <linux/dmapool.h>

#define SAHARA_PART_NO 4
#define VPU_PART_NO   0

/*!
 * This is the set of errors which signal that access to the SCM RAM has
 * failed or will fail.
 */
#define SCM_ACCESS_ERRORS                                                  \
       (SCM_ERRSTAT_ILM | SCM_ERRSTAT_SUP | SCM_ERRSTAT_ERC_MASK)

/******************************************************************************
 *
 *  Global / Static Variables
 *
 *****************************************************************************/

#ifdef SCC_REGISTER_DEBUG

#define REG_PRINT_BUFFER_SIZE 200

static char reg_print_buffer[REG_PRINT_BUFFER_SIZE];

typedef char *(*reg_print_routine_t) (uint32_t value, char *print_buffer,
				      int buf_size);

#endif

/*!
 * This is type void* so that a) it cannot directly be dereferenced,
 * and b) pointer arithmetic on it will function in a 'normal way' for
 * the offsets in scc_defines.h
 *
 * scc_base is the location in the iomap where the SCC's registers
 * (and memory) start.
 *
 * The referenced data is declared volatile so that the compiler will
 * not make any assumptions about the value of registers in the SCC,
 * and thus will always reload the register into CPU memory before
 * using it (i.e. wherever it is referenced in the driver).
 *
 * This value should only be referenced by the #SCC_READ_REGISTER and
 * #SCC_WRITE_REGISTER macros and their ilk.  All dereferences must be
 * 32 bits wide.
 */
static volatile void *scc_base;

/*! Array to hold function pointers registered by
    #scc_monitor_security_failure() and processed by
    #scc_perform_callbacks() */
static void (*scc_callbacks[SCC_CALLBACK_SIZE]) (void);

uint32_t scm_ram_phys_base = SCM_RAM_BASE_ADDR;

void *scm_ram_base = NULL;

/*!
 * Starting address of Sahara key partition
 */
uint8_t *sahara_partition_base;
dma_addr_t sahara_partition_phys;

uint8_t *scm_black_part_virt;
uint32_t scm_black_part_phys;

uint8_t *scm_red_part_virt;
uint32_t scm_red_part_cmd;
/*!
 * Starting address of VPU Partition
 */
uint8_t *vpu_partition_base;
dma_addr_t vpu_partition_phys;
/*! Calculated once for quick reference to size of the unreserved space in
 *  RAM in SCM.
 */
uint32_t scm_memory_size_bytes;

/*! Structure returned by #scc_get_configuration() */
static scc_config_t scc_configuration = {
	.driver_major_version = SCC_DRIVER_MAJOR_VERSION_1,
	.driver_minor_version = SCC_DRIVER_MINOR_VERSION_97,
	.scm_version = -1,
	.smn_version = -1,
	.block_size_bytes = -1,
	.partition_size_bytes = -1,
	.partition_count = -1,
};

/*! Key Control Information.  Integrity is controlled by use of
    #scc_crypto_lock. */
static struct scc_key_slot scc_key_info[SCC_KEY_SLOTS];

/*! Internal flag to know whether SCC is in Failed state (and thus many
 *  registers are unavailable).  Once it goes failed, it never leaves it. */
static volatile enum scc_status scc_availability = SCC_STATUS_INITIAL;

/*! Flag to say whether interrupt handler has been registered for
 * SMN interrupt */
static int smn_irq_set = 0;

/*! Flag to say whether interrupt handler has been registered for
 * SCM interrupt */
static int scm_irq_set = 0;

/*! This lock protects the #scc_callbacks list as well as the @c
 * callbacks_performed flag in #scc_perform_callbacks.  Since the data this
 * protects may be read or written from either interrupt or base level, all
 * operations should use the irqsave/irqrestore or similar to make sure that
 * interrupts are inhibited when locking from base level.
 */
static spinlock_t scc_callbacks_lock = SPIN_LOCK_UNLOCKED;

/*!
 * Ownership of this lock prevents conflicts on the crypto operation in the SCC
 * and the integrity of the #scc_key_info.
 */
static spinlock_t scc_crypto_lock = SPIN_LOCK_UNLOCKED;

/*! Calculated once for quick reference to size of SCM address space */
//static uint32_t scm_highest_memory_address;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
/*! Pointer to SCC's clock information.  Initialized during scc_init(). */
static struct clk *scc_clk = NULL;
#endif

/*! The lookup table for an 8-bit value.  Calculated once
 * by #scc_init_ccitt_crc().
 */
static uint16_t scc_crc_lookup_table[256];

/*! Fixed padding for appending to plaintext to fill out a block */
static uint8_t scc_block_padding[16] =
    { SCC_DRIVER_PAD_CHAR, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static scc_return_t make_sahara_partition(void);
uint8_t make_vpu_partition(void);
/******************************************************************************
 *
 *  Function Implementations - Externally Accessible
 *
 *****************************************************************************/

/*****************************************************************************/
/* fn scc_init()                                                             */
/*****************************************************************************/
/*!
 *  Initialize the driver at boot time or module load time.
 *
 *  Register with the kernel as the interrupt handler for the SCC interrupt
 *  line(s).
 *
 *  Map the SCC's register space into the driver's memory space.
 *
 *  Query the SCC for its configuration and status.  Save the configuration in
 *  #scc_configuration and save the status in #scc_availability.  Called by the
 *  kernel.
 *
 *  Do any locking/wait queue initialization which may be necessary.
 *
 *  The availability fuse may be checked, depending on platform.
 */
static int scc_init(void)
{
	uint32_t smn_status;
	int i;
	int return_value = -EIO;	/* assume error */

	if (scc_availability == SCC_STATUS_INITIAL) {

		/* Set this until we get an initial reading */
		scc_availability = SCC_STATUS_CHECKING;

		/* Initialize the constant for the CRC function */
		scc_init_ccitt_crc();

		/* initialize the callback table */
		for (i = 0; i < SCC_CALLBACK_SIZE; i++) {
			scc_callbacks[i] = 0;
		}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18))
		mxc_clks_enable(SCC_CLK);
#else
		scc_clk = clk_get(NULL, "scc_clk");
		if (scc_clk != ERR_PTR(ENOENT)) {
			clk_enable(scc_clk);
		}
#endif

		/* See whether there is an SCC available */
		if (0 && !SCC_ENABLED()) {
			printk(KERN_ERR
			       "SCC2: Fuse for SCC is set to disabled.  Exiting.\n");
			goto out;
		}
		/* Map the SCC (SCM and SMN) memory on the internal bus into
		   kernel address space */
		scc_base = (void *)IO_ADDRESS(SCC_BASE);
		if (scc_base == NULL) {
			printk(KERN_ERR
			       "SCC2: Register mapping failed.  Exiting.\n");
			goto out;
		}

		/* If that worked, we can try to use the SCC */
		/* Get SCM into 'clean' condition w/interrupts cleared &
		   disabled */
		SCC_WRITE_REGISTER(SCM_INT_CTL_REG, 0);

		/* Clear error status register */
		(void)SCC_READ_REGISTER(SCM_ERR_STATUS_REG);

		/*
		 * There is an SCC.  Determine its current state.  Side effect
		 * is to populate scc_config and scc_availability
		 */
		smn_status = scc_grab_config_values();

		/* Try to set up interrupt handler(s) */
		if (scc_availability != SCC_STATUS_OK) {
			goto out;
		}

		scm_ram_base =
		    (void *)ioremap_nocache(scm_ram_phys_base,
					    scc_configuration.partition_count *
					    scc_configuration.
					    partition_size_bytes);
		if (scm_ram_base == NULL) {
			printk(KERN_ERR
			       "SCC2: RAM failed to remap: %p for %d bytes\n",
			       (void *)scm_ram_phys_base,
			       scc_configuration.partition_count *
			       scc_configuration.partition_size_bytes);
			goto out;
		}
		pr_debug("SCC2: RAM at Physical %p / Virtual %p\n",
			 (void *)scm_ram_phys_base, scm_ram_base);

		return_value = make_sahara_partition();
		if (return_value != SCC_RET_OK) {
			scc_availability = SCC_STATUS_UNIMPLEMENTED;
			goto out;
		}

		/* Initialize key slots */
		for (i = 0; i < SCC_KEY_SLOTS; i++) {
			scc_key_info[i].offset = i * SCC_KEY_SLOT_SIZE;
			scc_key_info[i].part_ctl =
			    (((i * SCC_KEY_SLOT_SIZE) /
			      SCC_BLOCK_SIZE_BYTES() << SCM_CCMD_OFFSET_SHIFT)
			     | (SAHARA_PART_NO << SCM_CCMD_PART_SHIFT));
			scc_key_info[i].status = 0;	/* unassigned */
		}

		if (setup_interrupt_handling() != 0) {
			unsigned err_cond;
			/*!
			 * The error could be only that the SCM interrupt was
			 * not set up.  This interrupt is always masked, so
			 * that is not an issue.
			 *
			 * The SMN's interrupt may be shared on that line, it
			 * may be separate, or it may not be wired.  Do what
			 * is necessary to check its status.
			 *
			 * Although the driver is coded for possibility of not
			 * having SMN interrupt, the fact that there is one
			 * means it should be available and used.
			 */
#ifdef USE_SMN_INTERRUPT
			err_cond = !smn_irq_set;	/* Separate. Check SMN binding */
#elif !defined(NO_SMN_INTERRUPT)
			err_cond = !scm_irq_set;	/* Shared. Check SCM binding */
#else
			err_cond = FALSE;	/*  SMN not wired at all.  Ignore. */
#endif
			if (err_cond) {
				/* setup was not able to set up SMN interrupt */
				scc_availability = SCC_STATUS_UNIMPLEMENTED;
				goto out;
			}
		}

		/* interrupt handling returned non-zero */
		/* Get SMN into 'clean' condition w/interrupts cleared &
		   enabled */
		SCC_WRITE_REGISTER(SMN_COMMAND_REG,
				   SMN_COMMAND_CLEAR_INTERRUPT
				   | SMN_COMMAND_ENABLE_INTERRUPT);

	      out:
		/*
		 * If status is SCC_STATUS_UNIMPLEMENTED or is still
		 * SCC_STATUS_CHECKING, could be leaving here with the driver partially
		 * initialized.  In either case, cleanup (which will mark the SCC as
		 * UNIMPLEMENTED).
		 */
		if (scc_availability == SCC_STATUS_CHECKING ||
		    scc_availability == SCC_STATUS_UNIMPLEMENTED) {
			scc_cleanup();
		} else {
			return_value = 0;	/* All is well */
		}
	}
	/* ! STATUS_INITIAL */
	printk("SCC2: Driver Status is %s\n",
	       (scc_availability == SCC_STATUS_INITIAL) ? "INITIAL" :
	       (scc_availability == SCC_STATUS_CHECKING) ? "CHECKING" :
	       (scc_availability ==
		SCC_STATUS_UNIMPLEMENTED) ? "UNIMPLEMENTED" : (scc_availability
							       ==
							       SCC_STATUS_OK) ?
	       "OK" : (scc_availability ==
		       SCC_STATUS_FAILED) ? "FAILED" : "UNKNOWN");

	return return_value;
}				/* scc_init */

/*****************************************************************************/
/* fn scc_cleanup()                                                          */
/*****************************************************************************/
/*!
 * Perform cleanup before driver/module is unloaded by setting the machine
 * state close to what it was when the driver was loaded.  This function is
 * called when the kernel is shutting down or when this driver is being
 * unloaded.
 *
 * A driver like this should probably never be unloaded, especially if there
 * are other module relying upon the callback feature for monitoring the SCC
 * status.
 *
 * In any case, cleanup the callback table (by clearing out all of the
 * pointers).  Deregister the interrupt handler(s).  Unmap SCC registers.
 *
 */
static void scc_cleanup(void)
{
	int i;

	/* Mark the driver / SCC as unusable. */
	scc_availability = SCC_STATUS_UNIMPLEMENTED;

	/* Clear out callback table */
	for (i = 0; i < SCC_CALLBACK_SIZE; i++) {
		scc_callbacks[i] = 0;
	}

	/* If SCC has been mapped in, clean it up and unmap it */
	if (scc_base) {
		/* For the SCM, disable interrupts. */
		SCC_WRITE_REGISTER(SCM_INT_CTL_REG, 0);

		/* For the SMN, clear and disable interrupts */
		SCC_WRITE_REGISTER(SMN_COMMAND_REG,
				   SMN_COMMAND_CLEAR_INTERRUPT);
	}

	if (sahara_partition_base != NULL) {

	}
	/* Now that interrupts cannot occur, disassociate driver from the interrupt
	 * lines.
	 */

	/* Deregister SCM interrupt handler */
	if (scm_irq_set) {
		free_irq(INT_SCC_SCM, NULL);
	}

	/* Deregister SMN interrupt handler */
	if (smn_irq_set) {
#ifdef USE_SMN_INTERRUPT
		free_irq(INT_SCC_SMN, NULL);
#endif
	}

	pr_debug("SCC2 driver cleaned up.\n");

}				/* scc_cleanup */

/*****************************************************************************/
/* fn scc_get_configuration()                                                */
/*****************************************************************************/
scc_config_t *scc_get_configuration(void)
{
	/*
	 * If some other driver calls scc before the kernel does, make sure that
	 * this driver's initialization is performed.
	 */
	if (scc_availability == SCC_STATUS_INITIAL) {
		scc_init();
	}

	/*!
	 * If there is no SCC, yet the driver exists, the value -1 will be in
	 * the #scc_config_t fields for other than the driver versions.
	 */
	return &scc_configuration;
}				/* scc_get_configuration */

/*****************************************************************************/
/* fn scc_zeroize_memories()                                                 */
/*****************************************************************************/
scc_return_t scc_zeroize_memories(void)
{
	scc_return_t return_status = SCC_RET_FAIL;

	return return_status;
}				/* scc_zeroize_memories */

/*****************************************************************************/
/* fn scc_crypt()                                                            */
/*****************************************************************************/
scc_return_t
scc_crypt(unsigned long count_in_bytes, uint8_t * data_in,
	  uint8_t * init_vector, scc_enc_dec_t direction,
	  scc_crypto_mode_t crypto_mode, scc_verify_t check_mode,
	  uint8_t * data_out, unsigned long *count_out_bytes)
{
	uint32_t scm_command = scm_red_part_cmd;
	unsigned long irq_flags;	/* for IRQ save/restore */
	unsigned locked = FALSE;
	scc_return_t return_code = SCC_RET_FAIL;

	if (scc_availability == SCC_STATUS_INITIAL) {
		scc_init();
	}
	(void)scc_update_state();	/* in case no interrupt line from SMN */
	/* make initial error checks */
	if (scc_availability != SCC_STATUS_OK
	    || count_in_bytes == 0
	    || data_in == 0
	    || data_out == 0
	    || (crypto_mode != SCC_CBC_MODE && crypto_mode != SCC_ECB_MODE)
	    || (crypto_mode == SCC_CBC_MODE && init_vector == NULL)
	    || (direction != SCC_ENCRYPT && direction != SCC_DECRYPT)
	    || (check_mode == SCC_VERIFY_MODE_NONE &&
		count_in_bytes % SCC_BLOCK_SIZE_BYTES() != 0)
	    || (direction == SCC_DECRYPT &&
		count_in_bytes % SCC_BLOCK_SIZE_BYTES() != 0)
	    || (check_mode != SCC_VERIFY_MODE_NONE &&
		check_mode != SCC_VERIFY_MODE_CCITT_CRC)) {
		pr_debug("SCC2: scc_crypt() detected bad argument\n");
		goto out;
	}
	/* Lock access to crypto memory of the SCC */
	spin_lock_irqsave(&scc_crypto_lock, irq_flags);
	locked = TRUE;
	/* Special needs for CBC Mode */
	if (crypto_mode == SCC_CBC_MODE) {
		scm_command |= SCM_CCMD_CBC;	/* change default of ECB */
		/* Put in Initial Context.  Vector registers are contiguous */
		copy_to_scc(init_vector,
			    (uint32_t) (scc_base + SCM_AES_CBC_IV0_REG),
			    SCC_BLOCK_SIZE_BYTES(), NULL);
	}

	/* Fill the BLACK_START register */
	SCC_WRITE_REGISTER(SCM_C_BLACK_ST_REG, scm_black_part_phys);

	if (direction == SCC_ENCRYPT) {
		/* Check for sufficient space in data_out */
		if (check_mode == SCC_VERIFY_MODE_NONE) {
			if (*count_out_bytes < count_in_bytes) {
				return_code = SCC_RET_INSUFFICIENT_SPACE;
				goto out;
			}
		} else {	/* SCC_VERIFY_MODE_CCITT_CRC */
			/* Calculate extra bytes needed for crc (2) and block
			   padding */
			int padding_needed =
			    CRC_SIZE_BYTES + SCC_BLOCK_SIZE_BYTES() -
			    ((count_in_bytes + CRC_SIZE_BYTES)
			     % SCC_BLOCK_SIZE_BYTES());

			/* Verify space is available */
			if (*count_out_bytes < count_in_bytes + padding_needed) {
				return_code = SCC_RET_INSUFFICIENT_SPACE;
				goto out;
			}
		}
		return_code =
		    scc_encrypt(count_in_bytes, data_in, scm_command, data_out,
				check_mode == SCC_VERIFY_MODE_CCITT_CRC,
				count_out_bytes);
	}
	/* direction == SCC_ENCRYPT */
	else {			/* SCC_DECRYPT */
		/* Check for sufficient space in data_out */
		if (check_mode == SCC_VERIFY_MODE_NONE) {
			if (*count_out_bytes < count_in_bytes) {
				return_code = SCC_RET_INSUFFICIENT_SPACE;
			}
		} else {	/* SCC_VERIFY_MODE_CCITT_CRC */
			/* Do initial check.  Assume last block (of padding) and CRC
			 * will get stripped.  After decipher is done and padding is
			 * removed, will know exact value.
			 */
			int possible_size = (int)count_in_bytes - CRC_SIZE_BYTES
			    - SCC_BLOCK_SIZE_BYTES();
			if ((int)*count_out_bytes < possible_size) {
				pr_debug
				    ("SCC2: insufficient decrypt space %ld/%d.\n",
				     *count_out_bytes, possible_size);
				return_code = SCC_RET_INSUFFICIENT_SPACE;
				goto out;
			}
		}

		return_code =
		    scc_decrypt(count_in_bytes, data_in, scm_command, data_out,
				check_mode == SCC_VERIFY_MODE_CCITT_CRC,
				count_out_bytes);
	}			/* SCC_DECRYPT */

      out:
	/* unlock the SCC */
	if (locked) {
		spin_unlock_irqrestore(&scc_crypto_lock, irq_flags);
	}

	return return_code;
}				/* scc_crypt */

/*****************************************************************************/
/* fn scc_set_sw_alarm()                                                     */
/*****************************************************************************/
void scc_set_sw_alarm(void)
{

	if (scc_availability == SCC_STATUS_INITIAL) {
		scc_init();
	}

	/* Update scc_availability based on current SMN status.  This might
	 * perform callbacks.
	 */
	(void)scc_update_state();

	/* if everything is OK, make it fail */
	if (scc_availability == SCC_STATUS_OK) {

		/* sound the alarm (and disable SMN interrupts */
		SCC_WRITE_REGISTER(SMN_COMMAND_REG,
				   SMN_COMMAND_SET_SOFTWARE_ALARM);

		scc_availability = SCC_STATUS_FAILED;	/* Remember what we've done */

		/* In case SMN interrupt is not available, tell the world */
		scc_perform_callbacks();
	}

	return;
}				/* scc_set_sw_alarm */

/*****************************************************************************/
/* fn scc_monitor_security_failure()                                         */
/*****************************************************************************/
scc_return_t scc_monitor_security_failure(void callback_func(void))
{
	int i;
	unsigned long irq_flags;	/* for IRQ save/restore */
	scc_return_t return_status = SCC_RET_TOO_MANY_FUNCTIONS;
	int function_stored = FALSE;

	if (scc_availability == SCC_STATUS_INITIAL) {
		scc_init();
	}

	/* Acquire lock of callbacks table.  Could be spin_lock_irq() if this
	 * routine were just called from base (not interrupt) level
	 */
	spin_lock_irqsave(&scc_callbacks_lock, irq_flags);

	/* Search through table looking for empty slot */
	for (i = 0; i < SCC_CALLBACK_SIZE; i++) {
		if (scc_callbacks[i] == callback_func) {
			if (function_stored) {
				/* Saved duplicate earlier.  Clear this later one. */
				scc_callbacks[i] = NULL;
			}
			/* Exactly one copy is now stored */
			return_status = SCC_RET_OK;
			break;
		} else if (scc_callbacks[i] == NULL && !function_stored) {
			/* Found open slot.  Save it and remember */
			scc_callbacks[i] = callback_func;
			return_status = SCC_RET_OK;
			function_stored = TRUE;
		}
	}

	/* Free the lock */
	spin_unlock_irqrestore(&scc_callbacks_lock, irq_flags);

	return return_status;
}				/* scc_monitor_security_failure */

/*****************************************************************************/
/* fn scc_stop_monitoring_security_failure()                                 */
/*****************************************************************************/
void scc_stop_monitoring_security_failure(void callback_func(void))
{
	unsigned long irq_flags;	/* for IRQ save/restore */
	int i;

	if (scc_availability == SCC_STATUS_INITIAL) {
		scc_init();
	}

	/* Acquire lock of callbacks table.  Could be spin_lock_irq() if this
	 * routine were just called from base (not interrupt) level
	 */
	spin_lock_irqsave(&scc_callbacks_lock, irq_flags);

	/* Search every entry of the table for this function */
	for (i = 0; i < SCC_CALLBACK_SIZE; i++) {
		if (scc_callbacks[i] == callback_func) {
			scc_callbacks[i] = NULL;	/* found instance - clear it out */
			break;
		}
	}

	/* Free the lock */
	spin_unlock_irqrestore(&scc_callbacks_lock, irq_flags);

	return;
}				/* scc_stop_monitoring_security_failure */

/*****************************************************************************/
/* fn scc_read_register()                                                    */
/*****************************************************************************/
scc_return_t scc_read_register(int register_offset, uint32_t * value)
{
	scc_return_t return_status = SCC_RET_FAIL;
	uint32_t smn_status;
	uint32_t scm_status;

	if (scc_availability == SCC_STATUS_INITIAL) {
		scc_init();
	}

	/* First layer of protection -- completely unaccessible SCC */
	if (scc_availability != SCC_STATUS_UNIMPLEMENTED) {

		/* Second layer -- that offset is valid */
		if (register_offset != SMN_BB_DEC_REG &&	/* write only! */
		    check_register_offset(register_offset) == SCC_RET_OK) {

			/* Get current status / update local state */
			smn_status = scc_update_state();
			scm_status = SCC_READ_REGISTER(SCM_STATUS_REG);

			/*
			 * Third layer - verify that the register being requested is
			 * available in the current state of the SCC.
			 */
			if ((return_status =
			     check_register_accessible(register_offset,
						       smn_status,
						       scm_status)) ==
			    SCC_RET_OK) {
				*value = SCC_READ_REGISTER(register_offset);
			}
		}
	}

	return return_status;
}				/* scc_read_register */

/*****************************************************************************/
/* fn scc_write_register()                                                   */
/*****************************************************************************/
scc_return_t scc_write_register(int register_offset, uint32_t value)
{
	scc_return_t return_status = SCC_RET_FAIL;
	uint32_t smn_status;
	uint32_t scm_status;

	if (scc_availability == SCC_STATUS_INITIAL) {
		scc_init();
	}

	/* First layer of protection -- completely unaccessible SCC */
	if (scc_availability != SCC_STATUS_UNIMPLEMENTED) {

		/* Second layer -- that offset is valid */
		if (!((register_offset == SCM_STATUS_REG) ||	/* These registers are */
		      (register_offset == SCM_VERSION_REG) ||	/*  Read Only */
		      (register_offset == SMN_BB_CNT_REG) ||
		      (register_offset == SMN_TIMER_REG)) &&
		    check_register_offset(register_offset) == SCC_RET_OK) {

			/* Get current status / update local state */
			smn_status = scc_update_state();
			scm_status = SCC_READ_REGISTER(SCM_STATUS_REG);

			/*
			 * Third layer - verify that the register being requested is
			 * available in the current state of the SCC.
			 */
			if (check_register_accessible
			    (register_offset, smn_status, scm_status) == 0) {
				SCC_WRITE_REGISTER(register_offset, value);
				return_status = SCC_RET_OK;
			}
		}
	}

	return return_status;
}				/* scc_write_register() */

/******************************************************************************
 *
 *  Function Implementations - Internal
 *
 *****************************************************************************/

/*****************************************************************************/
/* fn scc_irq()                                                              */
/*****************************************************************************/
/*!
 * This is the interrupt handler for the SCC.
 *
 * This function checks the SMN Status register to see whether it
 * generated the interrupt, then it checks the SCM Status register to
 * see whether it needs attention.
 *
 * If an SMN Interrupt is active, then the SCC state set to failure, and
 * #scc_perform_callbacks() is invoked to notify any interested parties.
 *
 * The SCM Interrupt should be masked, as this driver uses polling to determine
 * when the SCM has completed a crypto or zeroing operation.  Therefore, if the
 * interrupt is active, the driver will just clear the interrupt and (re)mask.
 *
 * @param irq Channel number for the IRQ. (@c SCC_INT_SMN or @c SCC_INT_SCM).
 * @param dev_id Pointer to the identification of the device.  Ignored.
 */
static irqreturn_t scc_irq(int irq, void *dev_id)
{
	uint32_t smn_status;
	uint32_t scm_status;
	int handled = 0;	/* assume interrupt isn't from SMN */
#if defined(USE_SMN_INTERRUPT)
	int smn_irq = INT_SCC_SMN;	/* SMN interrupt is on a line by itself */
#elif defined (NO_SMN_INTERRUPT)
	int smn_irq = -1;	/* not wired to CPU at all */
#else
	int smn_irq = INT_SCC_SCM;	/* SMN interrupt shares a line with SCM */
#endif

	/* Update current state... This will perform callbacks... */
	smn_status = scc_update_state();

	/* SMN is on its own interrupt line.  Verify the IRQ was triggered
	 * before clearing the interrupt and marking it handled.  */
	if ((irq == smn_irq) && (smn_status & SMN_STATUS_SMN_STATUS_IRQ)) {
		SCC_WRITE_REGISTER(SMN_COMMAND_REG,
				   SMN_COMMAND_CLEAR_INTERRUPT);
		handled++;	/* tell kernel that interrupt was handled */
	}

	/* Check on the health of the SCM */
	scm_status = SCC_READ_REGISTER(SCM_STATUS_REG);

	/* The driver masks interrupts, so this should never happen. */
	if (irq == INT_SCC_SCM) {
		/* but if it does, try to prevent it in the future */
		SCC_WRITE_REGISTER(SCM_INT_CTL_REG, 0);
		handled++;
	}

	/* Any non-zero value of handled lets kernel know we got something */
	return IRQ_RETVAL(handled);
}

/*****************************************************************************/
/* fn scc_perform_callbacks()                                                */
/*****************************************************************************/
/*! Perform callbacks registered by #scc_monitor_security_failure().
 *
 *  Make sure callbacks only happen once...  Since there may be some reason why
 *  the interrupt isn't generated, this routine could be called from base(task)
 *  level.
 *
 *  One at a time, go through #scc_callbacks[] and call any non-null pointers.
 */
static void scc_perform_callbacks(void)
{
	static int callbacks_performed = 0;
	unsigned long irq_flags;	/* for IRQ save/restore */
	int i;

	/* Acquire lock of callbacks table and callbacks_performed flag */
	spin_lock_irqsave(&scc_callbacks_lock, irq_flags);

	if (!callbacks_performed) {
		callbacks_performed = 1;

		/* Loop over all of the entries in the table */
		for (i = 0; i < SCC_CALLBACK_SIZE; i++) {
			/* If not null, ... */
			if (scc_callbacks[i]) {
				scc_callbacks[i] ();	/* invoke the callback routine */
			}
		}
	}

	spin_unlock_irqrestore(&scc_callbacks_lock, irq_flags);

	return;
}

/*****************************************************************************/
/* fn copy_to_scc()                                                          */
/*****************************************************************************/
/*!
 *  Move data from possibly unaligned source and realign for SCC, possibly
 *  while calculating CRC.
 *
 *  Multiple calls can be made to this routine (without intervening calls to
 *  #copy_from_scc(), as long as the sum total of bytes copied is a multiple of
 *  four (SCC native word size).
 *
 *  @param[in]     from        Location in memory
 *  @param[out]    to          Location in SCC
 *  @param[in]     count_bytes Number of bytes to copy
 *  @param[in,out] crc         Pointer to CRC.  Initial value must be
 *                             #CRC_CCITT_START if this is the start of
 *                             message.  Output is the resulting (maybe
 *                             partial) CRC.  If NULL, no crc is calculated.
 *
 * @return  Zero - success.  Non-zero - SCM status bits defining failure.
 */
static uint32_t
copy_to_scc(const uint8_t * from, uint32_t to, unsigned long count_bytes,
	    uint16_t * crc)
{
	int i;
	uint32_t scm_word;
	uint16_t current_crc = 0;	/* local copy for fast access */

	pr_debug("SCC2: copying %ld bytes to 0x%0x.\n", count_bytes, to);

	if (crc) {
		current_crc = *crc;
	}

	/* Initialize value being built for SCM.  If we are starting 'clean',
	 * set it to zero.  Otherwise pick up partial value which had been saved
	 * earlier. */
	if (SCC_BYTE_OFFSET(to) == 0) {
		scm_word = 0;
	} else {
		scm_word = *(uint32_t *) SCC_WORD_PTR(to);	/* recover */
	}

	/* Now build up SCM words and write them out when each is full */
	for (i = 0; i < count_bytes; i++) {
		uint8_t byte = *from++;	/* value from plaintext */

#ifdef __BIG_ENDIAN
		scm_word = (scm_word << 8) | byte;	/* add byte to SCM word */
#else
		scm_word = (byte << 24) | (scm_word >> 8);
#endif
		/* now calculate CCITT CRC */
		if (crc) {
			CALC_CRC(byte, current_crc);
		}

		to++;		/* bump location in SCM */

		/* check for full word */
		if (SCC_BYTE_OFFSET(to) == 0) {
			*(uint32_t *) (to - 4) = scm_word;	/* write it out */
		}
	}

	/* If at partial word after previous loop, save it in SCM memory for
	   next time. */
	if (SCC_BYTE_OFFSET(to) != 0) {
		*(uint32_t *) SCC_WORD_PTR(to) = scm_word;	/* save */
	}

	/* Copy CRC back */
	if (crc) {
		*crc = current_crc;
	}

	return SCC_RET_OK;
}

/*****************************************************************************/
/* fn copy_from_scc()                                                        */
/*****************************************************************************/
/*!
 *  Move data from aligned 32-bit source and place in (possibly unaligned)
 *  target, and maybe calculate CRC at the same time.
 *
 *  Multiple calls can be made to this routine (without intervening calls to
 *  #copy_to_scc(), as long as the sum total of bytes copied is be a multiple
 *  of four.
 *
 *  @param[in]     from        Location in SCC
 *  @param[out]    to          Location in memory
 *  @param[in]     count_bytes Number of bytes to copy
 *  @param[in,out] crc         Pointer to CRC.  Initial value must be
 *                             #CRC_CCITT_START if this is the start of
 *                             message.  Output is the resulting (maybe
 *                             partial) CRC.  If NULL, crc is not calculated.
 *
 * @return  Zero - success.  Non-zero - SCM status bits defining failure.
 */
static uint32_t
copy_from_scc(const uint32_t from, uint8_t * to, unsigned long count_bytes,
	      uint16_t * crc)
{
	uint32_t running_from = from;
	uint32_t scm_word;
	uint16_t current_crc = 0;	/* local copy for fast access */

	pr_debug("SCC2: copying %ld bytes from 0x%x.\n", count_bytes, from);

	if (crc) {
		current_crc = *crc;
	}

	/* Read word which is sitting in SCM memory.  Ignore byte offset */
	scm_word = *(uint32_t *) SCC_WORD_PTR(running_from);
	pr_debug("%08x ", scm_word);
	/* If necessary, move the 'first' byte into place */
	if (SCC_BYTE_OFFSET(running_from) != 0) {
#ifdef __BIG_ENDIAN
		scm_word <<= 8 * SCC_BYTE_OFFSET(running_from);
#else
		scm_word >>= 8 * SCC_BYTE_OFFSET(running_from);
#endif
	}

	/* Now build up SCM words and write them out when each is full */
	while (count_bytes--) {
		uint8_t byte;	/* value from plaintext */

#ifdef __BIG_ENDIAN
		byte = (scm_word & 0xff000000) >> 24;	/* pull byte out of SCM word */
		scm_word <<= 8;	/* shift over to remove the just-pulled byte */
#else
		byte = (scm_word & 0xff);
		scm_word >>= 8;	/* shift over to remove the just-pulled byte */
#endif
		*to++ = byte;	/* send byte to memory */

		/* now calculate CRC */
		if (crc) {
			CALC_CRC(byte, current_crc);
		}

		running_from++;
		/* check for empty word */
		if (count_bytes && SCC_BYTE_OFFSET(running_from) == 0) {
			/* read one in */
			scm_word = *(uint32_t *) running_from;
			pr_debug("%08x ", scm_word);
		}
	}

	pr_debug("\n");
	if (crc) {
		*crc = current_crc;
	}

	return SCC_RET_OK;
}

/*****************************************************************************/
/* fn scc_strip_padding()                                                    */
/*****************************************************************************/
/*!
 *  Remove padding from plaintext.  Search backwards for #SCC_DRIVER_PAD_CHAR,
 *  verifying that each byte passed over is zero (0).  Maximum number of bytes
 *  to examine is 8.
 *
 *  @param[in]     from           Pointer to byte after end of message
 *  @param[out]    count_bytes_stripped Number of padding bytes removed by this
 *                                      function.
 *
 *  @return   #SCC_RET_OK if all goes, well, #SCC_RET_FAIL if padding was
 *            not present.
*/
static scc_return_t
scc_strip_padding(uint8_t * from, unsigned *count_bytes_stripped)
{
	int i = SCC_BLOCK_SIZE_BYTES();
	scc_return_t return_code = SCC_RET_VERIFICATION_FAILED;

	/*
	 * Search backwards looking for the magic marker.  If it isn't found,
	 * make sure that a 0 byte is there in its place.  Stop after the maximum
	 * amount of padding (8 bytes) has been searched);
	 */
	while (i-- > 0) {
		if (*--from == SCC_DRIVER_PAD_CHAR) {
			*count_bytes_stripped = SCC_BLOCK_SIZE_BYTES() - i;
			return_code = SCC_RET_OK;
			break;
		} else if (*from != 0) {	/* if not marker, check for 0 */
			pr_debug("SCC2: Found non-zero interim pad: 0x%x\n",
				 *from);
			break;
		}
	}

	return return_code;
}

/*****************************************************************************/
/* fn scc_update_state()                                                     */
/*****************************************************************************/
/*!
 * Make certain SCC is still running.
 *
 * Side effect is to update #scc_availability and, if the state goes to failed,
 * run #scc_perform_callbacks().
 *
 * (If #SCC_BRINGUP is defined, bring SCC to secure state if it is found to be
 * in health check state)
 *
 * @return Current value of #SMN_STATUS register.
 */
static uint32_t scc_update_state(void)
{
	uint32_t smn_status_register = SMN_STATE_FAIL;
	int smn_state;

	/* if FAIL or UNIMPLEMENTED, don't bother */
	if (scc_availability == SCC_STATUS_CHECKING ||
	    scc_availability == SCC_STATUS_OK) {

		smn_status_register = SCC_READ_REGISTER(SMN_STATUS_REG);
		smn_state = smn_status_register & SMN_STATUS_STATE_MASK;

#ifdef SCC_BRINGUP
		/* If in Health Check while booting, try to 'bringup' to Secure mode */
		if (scc_availability == SCC_STATUS_CHECKING &&
		    smn_state == SMN_STATE_HEALTH_CHECK) {
			/* Code up a simple algorithm for the ASC */
			SCC_WRITE_REGISTER(SMN_SEQUENCE_START, 0xaaaa);
			SCC_WRITE_REGISTER(SMN_SEQUENCE_END, 0x5555);
			SCC_WRITE_REGISTER(SMN_SEQUENCE_CHECK, 0x5555);
			/* State should be SECURE now */
			smn_status_register = SCC_READ_REGISTER(SMN_STATUS);
			smn_state = smn_status_register & SMN_STATUS_STATE_MASK;
		}
#endif

		/*
		 * State should be SECURE or NON_SECURE for operation of the part.  If
		 * FAIL, mark failed (i.e. limited access to registers).  Any other
		 * state, mark unimplemented, as the SCC is unuseable.
		 */
		if (smn_state == SMN_STATE_SECURE
		    || smn_state == SMN_STATE_NON_SECURE) {
			/* Healthy */
			scc_availability = SCC_STATUS_OK;
		} else if (smn_state == SMN_STATE_FAIL) {
			scc_availability = SCC_STATUS_FAILED;	/* uh oh - unhealthy */
			scc_perform_callbacks();
			printk(KERN_ERR "SCC2: SCC went into FAILED mode\n");
		} else {
			/* START, ZEROIZE RAM, HEALTH CHECK, or unknown */
			scc_availability = SCC_STATUS_UNIMPLEMENTED;	/* unuseable */
			printk(KERN_ERR "SCC2: SCC declared UNIMPLEMENTED\n");
		}
	}
	/* if availability is initial or ok */
	return smn_status_register;
}

/*****************************************************************************/
/* fn scc_init_ccitt_crc()                                                   */
/*****************************************************************************/
/*!
 * Populate the partial CRC lookup table.
 *
 * @return   none
 *
 */
static void scc_init_ccitt_crc(void)
{
	int dividend;		/* index for lookup table */
	uint16_t remainder;	/* partial value for a given dividend */
	int bit;		/* index into bits of a byte */

	/*
	 * Compute the remainder of each possible dividend.
	 */
	for (dividend = 0; dividend < 256; ++dividend) {
		/*
		 * Start with the dividend followed by zeros.
		 */
		remainder = dividend << (8);

		/*
		 * Perform modulo-2 division, a bit at a time.
		 */
		for (bit = 8; bit > 0; --bit) {
			/*
			 * Try to divide the current data bit.
			 */
			if (remainder & 0x8000) {
				remainder = (remainder << 1) ^ CRC_POLYNOMIAL;
			} else {
				remainder = (remainder << 1);
			}
		}

		/*
		 * Store the result into the table.
		 */
		scc_crc_lookup_table[dividend] = remainder;
	}

}				/* scc_init_ccitt_crc() */

/*****************************************************************************/
/* fn grab_config_values()                                                   */
/*****************************************************************************/
/*!
 * grab_config_values() will read the SCM Configuration and SMN Status
 * registers and store away version and size information for later use.
 *
 * @return The current value of the SMN Status register.
 */
static uint32_t scc_grab_config_values(void)
{
	uint32_t scm_version_register;
	uint32_t smn_status_register = SMN_STATE_FAIL;

	if (scc_availability != SCC_STATUS_CHECKING) {
		goto out;
	}
	scm_version_register = SCC_READ_REGISTER(SCM_VERSION_REG);
	pr_debug("SCC2 Driver: SCM version is 0x%08x\n", scm_version_register);

	/* Get SMN status and update scc_availability */
	smn_status_register = scc_update_state();
	pr_debug("SCC2 Driver: SMN status is 0x%08x\n", smn_status_register);

	/* save sizes and versions information for later use */
	scc_configuration.block_size_bytes = 16;	/* BPCP ? */
	scc_configuration.partition_count =
	    1 + ((scm_version_register & SCM_VER_NP_MASK) >> SCM_VER_NP_SHIFT);
	scc_configuration.partition_size_bytes =
	    1 << ((scm_version_register & SCM_VER_BPP_MASK) >>
		  SCM_VER_BPP_SHIFT);
	scc_configuration.scm_version =
	    (scm_version_register & SCM_VER_MAJ_MASK) >> SCM_VER_MAJ_SHIFT;
	scc_configuration.smn_version =
	    (smn_status_register & SMN_STATUS_VERSION_ID_MASK)
	    >> SMN_STATUS_VERSION_ID_SHIFT;
	if (scc_configuration.scm_version != SCM_MAJOR_VERSION_2) {
		scc_availability = SCC_STATUS_UNIMPLEMENTED;	/* Unknown version */
	}

      out:
	return smn_status_register;
}				/* grab_config_values */

/*****************************************************************************/
/* fn setup_interrupt_handling()                                             */
/*****************************************************************************/
/*!
 * Register the SCM and SMN interrupt handlers.
 *
 * Called from #scc_init()
 *
 * @return 0 on success
 */
static int setup_interrupt_handling(void)
{
	int smn_error_code = -1;
	int scm_error_code = -1;

	/* Disnable SCM interrupts */
	SCC_WRITE_REGISTER(SCM_INT_CTL_REG, 0);

#ifdef USE_SMN_INTERRUPT
	/* Install interrupt service routine for SMN. */
	smn_error_code = request_irq(INT_SCC_SMN, scc_irq, 0,
				     SCC_DRIVER_NAME, NULL);
	if (smn_error_code != 0) {
		printk
		    ("SCC2 Driver: Error installing SMN Interrupt Handler: %d\n",
		     smn_error_code);
	} else {
		smn_irq_set = 1;	/* remember this for cleanup */
		/* Enable SMN interrupts */
		SCC_WRITE_REGISTER(SMN_COMMAND_REG,
				   SMN_COMMAND_CLEAR_INTERRUPT |
				   SMN_COMMAND_ENABLE_INTERRUPT);
	}
#else
	smn_error_code = 0;	/* no problems... will handle later */
#endif

	/*
	 * Install interrupt service routine for SCM (or both together).
	 */
	scm_error_code = request_irq(INT_SCC_SCM, scc_irq, 0,
				     SCC_DRIVER_NAME, NULL);
	if (scm_error_code != 0) {
#ifndef MXC
		printk
		    ("SCC2 Driver: Error installing SCM Interrupt Handler: %d\n",
		     scm_error_code);
#else
		printk
		    ("SCC2 Driver: Error installing SCC Interrupt Handler: %d\n",
		     scm_error_code);
#endif
	} else {
		scm_irq_set = 1;	/* remember this for cleanup */
#if defined(USE_SMN_INTERRUPT) && !defined(NO_SMN_INTERRUPT)
		/* Enable SMN interrupts */
		SCC_WRITE_REGISTER(SMN_COMMAND_REG,
				   SMN_COMMAND_CLEAR_INTERRUPT |
				   SMN_COMMAND_ENABLE_INTERRUPT);
#endif
	}

	/* Return an error if one was encountered */
	return scm_error_code ? scm_error_code : smn_error_code;
}				/* setup_interrupt_handling */

/*****************************************************************************/
/* fn scc_do_crypto()                                                        */
/*****************************************************************************/
/*! Have the SCM perform the crypto function.
 *
 * Set up length register, and the store @c scm_control into control register
 * to kick off the operation.  Wait for completion, gather status, clear
 * interrupt / status.
 *
 * @param byte_count  number of bytes to perform in this operation
 * @param scm_command Bit values to be set in @c SCM_CCMD_REG register
 *
 * @return 0 on success, value of #SCM_ERROR_STATUS on failure
 */
static uint32_t scc_do_crypto(int byte_count, uint32_t scm_command)
{
	int block_count = byte_count / SCC_BLOCK_SIZE_BYTES();
	uint32_t crypto_status;
	scc_return_t ret;

	/* In length register, 0 means 1, etc. */
	scm_command |= (block_count - 1) << SCM_CCMD_LENGTH_SHIFT;

	/* set modes and kick off the operation */
	SCC_WRITE_REGISTER(SCM_CCMD_REG, scm_command);

	ret = scc_wait_completion(&crypto_status);

	/* Only done bit should be on */
	if (crypto_status & SCM_STATUS_ERR) {
		/* Replace with error status instead */
		crypto_status = SCC_READ_REGISTER(SCM_ERR_STATUS_REG);
		pr_debug("SCM Failure: 0x%x\n", crypto_status);
		if (crypto_status == 0) {
			/* That came up 0.  Turn on arbitrary bit to signal error. */
			crypto_status = SCM_ERRSTAT_ILM;
		}
	} else {
		crypto_status = 0;
	}
	pr_debug("SCC2: Done waiting.\n");

	return crypto_status;
}

/*****************************************************************************/
/* fn scc_encrypt()                                                          */
/*****************************************************************************/
/*!
 * Perform an encryption on the input.  If @c verify_crc is true, a CRC must be
 * calculated on the plaintext, and appended, with padding, before computing
 * the ciphertext.
 *
 * @param[in]     count_in_bytes  Count of bytes of plaintext
 * @param[in]     data_in         Pointer to the plaintext
 * @param[in]     scm_control     Bit values for the SCM_CONTROL register
 * @param[in,out] data_out        Pointer for storing ciphertext
 * @param[in]     add_crc         Flag for computing CRC - 0 no, else yes
 * @param[in,out] count_out_bytes Number of bytes available at @c data_out
 */
static scc_return_t
scc_encrypt(uint32_t count_in_bytes, uint8_t * data_in, uint32_t scm_control,
	    uint8_t * data_out, int add_crc, unsigned long *count_out_bytes)
{
	scc_return_t return_code = SCC_RET_FAIL;	/* initialised for failure */
	uint32_t input_bytes_left = count_in_bytes;	/* local copy */
	uint32_t output_bytes_copied = 0;	/* running total */
	uint32_t bytes_to_process;	/* multi-purpose byte counter */
	uint16_t crc = CRC_CCITT_START;	/* running CRC value */
	crc_t *crc_ptr = NULL;	/* Reset if CRC required */
	/* byte address into SCM RAM */
	uint32_t scm_location = (uint32_t) scm_red_part_virt;
	uint32_t scm_bytes_remaining = scm_memory_size_bytes;	/* free RED RAM */
	uint8_t padding_buffer[PADDING_BUFFER_MAX_BYTES];	/* CRC+padding holder */
	unsigned padding_byte_count = 0;	/* Reset if padding required */
	uint32_t scm_error_status = 0;	/* No known SCM error initially */

	scm_control |= SCM_CCMD_ENC;
	/* Set location of CRC and prepare padding bytes if required */
	if (add_crc != 0) {
		crc_ptr = &crc;
		padding_byte_count = SCC_BLOCK_SIZE_BYTES()
		    - (count_in_bytes +
		       CRC_SIZE_BYTES) % SCC_BLOCK_SIZE_BYTES();
		memcpy(padding_buffer + CRC_SIZE_BYTES, scc_block_padding,
		       padding_byte_count);
	}

	/* Process remaining input or padding data */
	while (input_bytes_left > 0) {
		/* Determine how much work to do this pass */
		bytes_to_process = (input_bytes_left > scm_bytes_remaining) ?
		    scm_bytes_remaining : input_bytes_left;
		/* Copy plaintext into SCM RAM, calculating CRC if required */
		copy_to_scc(data_in, scm_location, bytes_to_process, crc_ptr);
		/* Adjust pointers & counters */
		input_bytes_left -= bytes_to_process;
		data_in += bytes_to_process;
		scm_location += bytes_to_process;
		scm_bytes_remaining -= bytes_to_process;

		/* Add CRC and padding after the last byte is copied if required */
		if ((input_bytes_left == 0) && (crc_ptr != NULL)) {

			/* Copy CRC into padding buffer MSB first */
			padding_buffer[0] = (crc >> 8) & 0xFF;
			padding_buffer[1] = crc & 0xFF;

			/* Reset pointers and counter */
			data_in = padding_buffer;
			input_bytes_left = CRC_SIZE_BYTES + padding_byte_count;
			crc_ptr = NULL;	/* CRC no longer required */

			/* Go round loop again to copy CRC and padding to SCM */
			continue;
		}

		/* if no input and crc_ptr */
		/* Now have block-sized plaintext in SCM to encrypt */
		/* Encrypt plaintext; exit loop on error */
		bytes_to_process = scm_location - (uint32_t) scm_red_part_virt;

		if (output_bytes_copied + bytes_to_process > *count_out_bytes) {
			return_code = SCC_RET_INSUFFICIENT_SPACE;
			scm_error_status = -1;	/* error signal */
			pr_debug
			    ("SCC2: too many ciphertext bytes for space available\n");
			break;
		}
		pr_debug("SCC2: Starting encryption. %x for %d bytes (%p)\n",
			 scm_control, bytes_to_process,
			 (void *)SCC_READ_REGISTER(SCM_C_BLACK_ST_REG));
		scm_error_status = scc_do_crypto(bytes_to_process, scm_control);
		if (scm_error_status != 0) {
			break;
		}

		/* Copy out ciphertext */
		copy_from_scc((uint32_t) scm_black_part_virt, data_out,
			      bytes_to_process, NULL);

		/* Adjust pointers and counters for next loop */
		output_bytes_copied += bytes_to_process;
		data_out += bytes_to_process;
		scm_location = (uint32_t) scm_red_part_virt;
		scm_bytes_remaining = scm_memory_size_bytes;
	}			/* input_bytes_left > 0 */

	/* If no SCM error, set OK status and save ouput byte count */
	if (scm_error_status == 0) {
		return_code = SCC_RET_OK;
		*count_out_bytes = output_bytes_copied;
	}

	return return_code;
}				/* scc_encrypt */

/*****************************************************************************/
/* fn scc_decrypt()                                                          */
/*****************************************************************************/
/*!
 * Perform a decryption on the input.  If @c verify_crc is true, the last block
 * (maybe the two last blocks) is special - it should contain a CRC and
 * padding.  These must be stripped and verified.
 *
 * @param[in]     count_in_bytes  Count of bytes of ciphertext
 * @param[in]     data_in         Pointer to the ciphertext
 * @param[in]     scm_control     Bit values for the SCM_CONTROL register
 * @param[in,out] data_out        Pointer for storing plaintext
 * @param[in]     verify_crc      Flag for running CRC - 0 no, else yes
 * @param[in,out] count_out_bytes Number of bytes available at @c data_out

 */
static scc_return_t
scc_decrypt(uint32_t count_in_bytes, uint8_t * data_in, uint32_t scm_control,
	    uint8_t * data_out, int verify_crc, unsigned long *count_out_bytes)
{
	scc_return_t return_code = SCC_RET_FAIL;
	uint32_t bytes_left = count_in_bytes;	/* local copy */
	uint32_t bytes_copied = 0;	/* running total of bytes going to user */
	uint32_t bytes_to_copy = 0;	/* Number in this encryption 'chunk' */
	uint16_t crc = CRC_CCITT_START;	/* running CRC value */
	/* next target for  ctext */
	uint32_t scm_location = (uint32_t) scm_black_part_virt;
	unsigned padding_byte_count;	/* number of bytes of padding stripped */
	uint8_t last_two_blocks[2 * SCC_BLOCK_SIZE_BYTES()];	/* temp */
	uint32_t scm_error_status = 0;	/* register value */

	scm_control |= SCM_CCMD_DEC;
	if (verify_crc) {
		/* Save last two blocks (if there are at least two) of ciphertext for
		   special treatment. */
		bytes_left -= SCC_BLOCK_SIZE_BYTES();
		if (bytes_left >= SCC_BLOCK_SIZE_BYTES()) {
			bytes_left -= SCC_BLOCK_SIZE_BYTES();
		}
	}

	/* Copy ciphertext into SCM BLACK memory */
	while (bytes_left && scm_error_status == 0) {

		/* Determine how much work to do this pass */
		if (bytes_left > (scm_memory_size_bytes)) {
			bytes_to_copy = scm_memory_size_bytes;
		} else {
			bytes_to_copy = bytes_left;
		}

		if (bytes_copied + bytes_to_copy > *count_out_bytes) {
			scm_error_status = -1;
			break;
		}
		copy_to_scc(data_in, scm_location, bytes_to_copy, NULL);
		data_in += bytes_to_copy;	/* move pointer */

		pr_debug("SCC2: Starting decryption of %d bytes.\n",
			 bytes_to_copy);

		/*  Do the work, wait for completion */
		scm_error_status = scc_do_crypto(bytes_to_copy, scm_control);

		copy_from_scc((uint32_t) scm_red_part_virt, data_out,
			      bytes_to_copy, &crc);
		bytes_copied += bytes_to_copy;
		data_out += bytes_to_copy;
		scm_location = (uint32_t) scm_black_part_virt;

		/* Do housekeeping */
		bytes_left -= bytes_to_copy;

	}			/* while bytes_left */

	/* At this point, either the process is finished, or this is verify mode */

	if (scm_error_status == 0) {
		if (!verify_crc) {
			*count_out_bytes = bytes_copied;
			return_code = SCC_RET_OK;
		} else {
			/* Verify mode.  There are one or two blocks of unprocessed
			 * ciphertext sitting at data_in.  They need to be moved to the
			 * SCM, decrypted, searched to remove padding, then the plaintext
			 * copied back to the user (while calculating CRC, of course).
			 */

			/* Calculate ciphertext still left */
			bytes_to_copy = count_in_bytes - bytes_copied;

			copy_to_scc(data_in, scm_location, bytes_to_copy, NULL);
			data_in += bytes_to_copy;	/* move pointer */

			pr_debug("SCC2: Finishing decryption (%d bytes).\n",
				 bytes_to_copy);

			/*  Do the work, wait for completion */
			scm_error_status =
			    scc_do_crypto(bytes_to_copy, scm_control);

			if (scm_error_status == 0) {
				/* Copy decrypted data back from SCM RED memory */
				copy_from_scc((uint32_t) scm_red_part_virt,
					      last_two_blocks, bytes_to_copy,
					      NULL);

				/* (Plaintext) + crc + padding should be in temp buffer */
				if (scc_strip_padding
				    (last_two_blocks + bytes_to_copy,
				     &padding_byte_count) == SCC_RET_OK) {
					bytes_to_copy -=
					    padding_byte_count + CRC_SIZE_BYTES;

					/* verify enough space in user buffer */
					if (bytes_copied + bytes_to_copy <=
					    *count_out_bytes) {
						int i = 0;

						/* Move out last plaintext and calc CRC */
						while (i < bytes_to_copy) {
							CALC_CRC(last_two_blocks
								 [i], crc);
							*data_out++ =
							    last_two_blocks
							    [i++];
							bytes_copied++;
						}

						/* Verify the CRC by running over itself */
						CALC_CRC(last_two_blocks
							 [bytes_to_copy], crc);
						CALC_CRC(last_two_blocks
							 [bytes_to_copy + 1],
							 crc);
						if (crc == 0) {
							/* Just fine ! */
							*count_out_bytes =
							    bytes_copied;
							return_code =
							    SCC_RET_OK;
						} else {
							return_code =
							    SCC_RET_VERIFICATION_FAILED;
							pr_debug
							    ("SCC2:  CRC values are %04x, %02x%02x\n",
							     crc,
							     last_two_blocks
							     [bytes_to_copy],
							     last_two_blocks
							     [bytes_to_copy +
							      1]);
						}
					}	/* if space available */
				} /* if scc_strip_padding... */
				else {
					/* bad padding means bad verification */
					return_code =
					    SCC_RET_VERIFICATION_FAILED;
				}
			}
			/* scm_error_status == 0 */
		}		/* verify_crc */
	}

	/* scm_error_status == 0 */
	return return_code;
}				/* scc_decrypt */

/*****************************************************************************/
/* fn scc_alloc_slot()                                                       */
/*****************************************************************************/
/*!
 * Allocate a key slot to fit the requested size.
 *
 * @param value_size_bytes   Size of the key or other secure data
 * @param owner_id           Value to tie owner to slot
 * @param[out] slot          Handle to access or deallocate slot
 *
 * @return SCC_RET_OK on success, SCC_RET_INSUFFICIENT_SPACE if not slots of
 *         requested size are available.
 */
scc_return_t
scc_alloc_slot(uint32_t value_size_bytes, uint64_t owner_id, uint32_t * slot)
{
	scc_return_t status = SCC_RET_FAIL;
	unsigned long irq_flags;

	if (scc_availability != SCC_STATUS_OK) {
		goto out;
	}
	/* ACQUIRE LOCK to prevent others from using SCC crypto */
	spin_lock_irqsave(&scc_crypto_lock, irq_flags);

	pr_debug("SCC2: Allocating %d-byte slot\n", value_size_bytes);

	if ((value_size_bytes != 0) && (value_size_bytes <= SCC_MAX_KEY_SIZE)) {
		int i;

		for (i = 0; i < SCC_KEY_SLOTS; i++) {
			if (scc_key_info[i].status == 0) {
				scc_key_info[i].owner_id = owner_id;
				scc_key_info[i].length = value_size_bytes;
				scc_key_info[i].status = 1;	/* assigned! */
				*slot = i;
				status = SCC_RET_OK;
				break;	/* exit 'for' loop */
			}
		}
		if (status != SCC_RET_OK) {
			status = SCC_RET_INSUFFICIENT_SPACE;
		} else {
			pr_debug("SCC2: Allocated slot %d\n", i);
		}
	}

	spin_unlock_irqrestore(&scc_crypto_lock, irq_flags);

      out:
	return status;
}

/*****************************************************************************/
/* fn verify_slot_access()                                                   */
/*****************************************************************************/
inline static scc_return_t
verify_slot_access(uint64_t owner_id, uint32_t slot, uint32_t access_len)
{
	scc_return_t status = SCC_RET_FAIL;

	if (scc_availability != SCC_STATUS_OK) {
		goto out;
	}

	if ((slot < SCC_KEY_SLOTS) && scc_key_info[slot].status
	    && (scc_key_info[slot].owner_id == owner_id)
	    && (access_len <= SCC_KEY_SLOT_SIZE)) {
		status = SCC_RET_OK;
		pr_debug("SCC2: Verify on slot %d succeeded\n", slot);
	} else {
		if (slot >= SCC_KEY_SLOTS) {
			pr_debug("SCC2: Verify on bad slot (%d) failed\n",
				 slot);
		} else if (scc_key_info[slot].status) {
			pr_debug("SCC2: Verify on slot %d failed (%Lx) \n",
				 slot, owner_id);
		} else {
			pr_debug
			    ("SC2C: Verify on slot %d failed: not allocated\n",
			     slot);
		}
	}

      out:
	return status;
}

/*****************************************************************************/
/* fn scc_dealloc_slot()                                                     */
/*****************************************************************************/
scc_return_t scc_dealloc_slot(uint64_t owner_id, uint32_t slot)
{
	scc_return_t status;
	unsigned long irq_flags;
	uint8_t *slot_loc = NULL;
	int i;

	/* ACQUIRE LOCK to prevent others from using SCC crypto */
	spin_lock_irqsave(&scc_crypto_lock, irq_flags);

	status = verify_slot_access(owner_id, slot, 0);
	if (status != SCC_RET_OK) {
		goto out;
	}

	scc_key_info[slot].owner_id = 0;
	scc_key_info[slot].status = 0;	/* unassign */
	slot_loc = sahara_partition_base + scc_key_info[slot].offset;

	for (i = 0; i < SCC_KEY_SLOT_SIZE; i++) {
		slot_loc[i] = 0;
	}
	pr_debug("SCC2: Deallocated slot %d\n", slot);

      out:
	spin_unlock_irqrestore(&scc_crypto_lock, irq_flags);

	return status;
}

/*****************************************************************************/
/* fn scc_load_slot()                                                        */
/*****************************************************************************/
/*!
 * Load a value into a slot.
 *
 * @param owner_id      Value of owner of slot
 * @param slot          Handle of slot
 * @param key_data      Data to load into the slot
 * @param key_length    Length, in bytes, of @c key_data to copy to SCC.
 *
 * @return SCC_RET_OK on success.  SCC_RET_FAIL will be returned if slot
 * specified cannot be accessed for any reason, or SCC_RET_INSUFFICIENT_SPACE
 * if @c key_length exceeds the size of the slot.
 */
scc_return_t
scc_load_slot(uint64_t owner_id, uint32_t slot, uint8_t * key_data,
	      uint32_t key_length)
{
	scc_return_t status;
	unsigned long irq_flags;

	/* ACQUIRE LOCK to prevent others from using SCC crypto */
	spin_lock_irqsave(&scc_crypto_lock, irq_flags);

	status = verify_slot_access(owner_id, slot, key_length);
	if ((status == SCC_RET_OK) && (key_data != NULL)) {
		status = SCC_RET_FAIL;	/* reset expectations */

		if (key_length > SCC_KEY_SLOT_SIZE) {
			pr_debug
			    ("SCC2: scc_load_slot() rejecting key of %d bytes.\n",
			     key_length);
			status = SCC_RET_INSUFFICIENT_SPACE;
		} else {
			if (copy_to_scc(key_data,
					(uint32_t) sahara_partition_base +
					scc_key_info[slot].offset, key_length,
					NULL)) {
				pr_debug("SCC2: RED copy_to_scc() failed for"
					 " scc_load_slot()\n");
			} else {
				status = SCC_RET_OK;
			}
		}
	}

	spin_unlock_irqrestore(&scc_crypto_lock, irq_flags);

	return status;
}				/* scc_load_slot */

/*****************************************************************************/
/* fn scc_encrypt_slot()                                                     */
/*****************************************************************************/
/*!
 * Allocate a key slot to fit the requested size.
 *
 * @param owner_id      Value of owner of slot
 * @param slot          Handle of slot
 * @param length        Length, in bytes, of @c black_data
 * @param black_data    Location to store result of encrypting RED data in slot
 *
 * @return SCC_RET_OK on success, SCC_RET_FAIL if slot specified cannot be
 *         accessed for any reason.
 */
scc_return_t scc_encrypt_slot(uint64_t owner_id, uint32_t slot,
			      uint32_t length, uint8_t * black_data)
{
	unsigned long irq_flags;
	scc_return_t status;
	uint32_t crypto_status;
	uint32_t scm_command = scc_key_info[slot].part_ctl;

	/* ACQUIRE LOCK to prevent others from using crypto or releasing slot */
	spin_lock_irqsave(&scc_crypto_lock, irq_flags);

	status = verify_slot_access(owner_id, slot, length);
	if (status == SCC_RET_OK) {
		SCC_WRITE_REGISTER(SCM_C_BLACK_ST_REG, scm_black_part_phys);

		/* Use OwnerID as CBC IV to tie Owner to data */
		SCC_WRITE_REGISTER(SCM_AES_CBC_IV0_REG,
				   *(uint32_t *) & owner_id);
		SCC_WRITE_REGISTER(SCM_AES_CBC_IV1_REG,
				   *(((uint32_t *) & owner_id) + 1));
		SCC_WRITE_REGISTER(SCM_AES_CBC_IV2_REG, 0);
		SCC_WRITE_REGISTER(SCM_AES_CBC_IV3_REG, 0);

		/* Set modes and kick off the encryption */
		crypto_status =
		    scc_do_crypto(length, scm_command | SCM_CCMD_AES_ENC_CBC);

		if (crypto_status != 0) {
			pr_debug("SCM encrypt red crypto failure: 0x%x\n",
				 crypto_status);
		} else {

			/* Give blob back to caller */
			if (!copy_from_scc
			    ((uint32_t) scm_black_part_virt, black_data, length,
			     NULL)) {
				status = SCC_RET_OK;
				pr_debug
				    ("SCC2: Encrypted slot %d for %d bytes\n",
				     slot, length);
			}
		}
	}

	spin_unlock_irqrestore(&scc_crypto_lock, irq_flags);

	return status;
}

/*****************************************************************************/
/* fn scc_decrypt_slot()                                                     */
/*****************************************************************************/
/*!
 * Decrypt some black data and leave result in the slot.
 *
 * @param owner_id      Value of owner of slot
 * @param slot          Handle of slot
 * @param length        Length, in bytes, of @c black_data
 * @param black_data    Location of data to dencrypt and store in slot
 *
 * @return SCC_RET_OK on success, SCC_RET_FAIL if slot specified cannot be
 *         accessed for any reason.
 */
scc_return_t scc_decrypt_slot(uint64_t owner_id, uint32_t slot,
			      uint32_t length, const uint8_t * black_data)
{
	unsigned long irq_flags;
	scc_return_t status;
	uint32_t crypto_status;
	uint32_t scm_command = scc_key_info[slot].part_ctl;

	/* ACQUIRE LOCK to prevent others from using crypto or releasing slot */
	spin_lock_irqsave(&scc_crypto_lock, irq_flags);

	status = verify_slot_access(owner_id, slot, length);
	if (status == SCC_RET_OK) {
		status = SCC_RET_FAIL;	/* reset expectations */

		/* Place black key in to BLACK RAM and set up the SCC */
		copy_to_scc(black_data,
			    (uint32_t) scm_black_part_virt, length, NULL);

		SCC_WRITE_REGISTER(SCM_C_BLACK_ST_REG, scm_black_part_phys);

		/* Use OwnerID as CBC IV to tie Owner to data */
		SCC_WRITE_REGISTER(SCM_AES_CBC_IV0_REG,
				   *(uint32_t *) & owner_id);
		SCC_WRITE_REGISTER(SCM_AES_CBC_IV1_REG,
				   *(((uint32_t *) & owner_id) + 1));
		SCC_WRITE_REGISTER(SCM_AES_CBC_IV2_REG, 0);
		SCC_WRITE_REGISTER(SCM_AES_CBC_IV3_REG, 0);

		/* Set modes and kick off the decryption */
		crypto_status = scc_do_crypto(length,
					      scm_command |
					      SCM_CCMD_AES_DEC_CBC);

		if (crypto_status != 0) {
			pr_debug("SCM decrypt black crypto failure: 0x%x\n",
				 crypto_status);
		} else {
			status = SCC_RET_OK;
			pr_debug("SCC2: Decrypted slot %d for %d bytes\n", slot,
				 length);
		}
	}

	spin_unlock_irqrestore(&scc_crypto_lock, irq_flags);

	return status;
}

/*****************************************************************************/
/* fn scc_get_slot_info()                                                    */
/*****************************************************************************/
/*!
 * Determine address and value length for a give slot.
 *
 * @param owner_id      Value of owner of slot
 * @param slot          Handle of slot
 * @param address       Location to store kernel address of slot data
 * @param value_size_bytes Location to store allocated length of data in slot.
 *                         May be NULL if value is not needed by caller.
 * @param slot_size_bytes  Location to store max length data in slot
 *                         May be NULL if value is not needed by caller.
 *
 * @return SCC_RET_OK or error indication
 */
scc_return_t
scc_get_slot_info(uint64_t owner_id, uint32_t slot, uint32_t * address,
		  uint32_t * value_size_bytes, uint32_t * slot_size_bytes)
{
	scc_return_t status = verify_slot_access(owner_id, slot, 0);

	if (status == SCC_RET_OK) {
		*address = sahara_partition_phys + scc_key_info[slot].offset;
		if (value_size_bytes != NULL) {
			*value_size_bytes = scc_key_info[slot].length;
		}
		if (slot_size_bytes != NULL) {
			*slot_size_bytes = SCC_KEY_SLOT_SIZE;
		}
	}

	return status;
}

/*!
 * For now, this function will create a shared Sahara and SCC2 partition.  It
 * will be used as a key store for Sahara and, to mimic SCCv1 behavior, as the
 * temporary black and red memories for ephemeral cipher operations.
 *
 * This means that it will not be secure against kernel access, and in fact
 * must be available for host read/write.
 *
 * *========================================*
 * *     Key Slot 0                         *
 * *     Key Slot 1                         *
 * *     Key Slot ...                       *
 * *     Key Slot n                         *
 * * -------------------------------------- *
 * *                                        *
 * *    'BLACK RAM'                         *
 * *                                        *
 * * -------------------------------------- *
 * *                                        *
 * *    'RED RAM'                           *
 * *                                        *
 * *========================================*
 *
 * or -- the BLACK RAM gets put in a separate partition...
 */
static
scc_return_t make_sahara_partition()
{
	unsigned part_no = SAHARA_PART_NO;	/* better be free!! */
	int retval = -EIO;
	uint32_t *part_base =
	    scm_ram_base + (part_no * scc_configuration.partition_size_bytes);
	uint32_t reg_value;

	reg_value = SCC_READ_REGISTER(SCM_PART_OWNERS_REG);

	/* Store SMID to grab a partition */
	SCC_WRITE_REGISTER(SCM_SMID0_REG + 8 * part_no, 0x00000000);
	mdelay(2);

	/* Now make sure it is ours... ? */
	reg_value = SCC_READ_REGISTER(SCM_PART_OWNERS_REG);

	if (((reg_value >> (2 * part_no)) & 0x3) != 3) {
		printk(KERN_ERR "Could not acquire partition %u\n", part_no);
		goto out;
	}
	sahara_partition_base = (uint8_t *) part_base;

	pr_debug("SCC2 Writing UMID at %p\n", part_base);

	/* Write in the UMID */
	part_base[4] = 0x42;
	part_base[5] = 0x43;
	part_base[6] = 0x19;
	part_base[7] = 0x59;

	mdelay(2);

	/* Give both host and Sahara access for read/write */
	part_base[0] =
	    SCM_PERM_HD_WRITE | SCM_PERM_HD_READ | SCM_PERM_TH_READ |
	    SCM_PERM_TH_WRITE;
	mdelay(2);

	reg_value = SCC_READ_REGISTER(SCM_PART_ENGAGED_REG);

	if (((reg_value >> part_no) & 1) != 1) {
		printk(KERN_ERR "SCC2 Could not engage partition %u\n",
		       part_no);
		retval = SCC_RET_FAIL;
		goto out;
	}
//    (void)SCC_READ_REGISTER(SCM_ACC4_REG);

	sahara_partition_phys =
	    (uint32_t) scm_ram_phys_base +
	    (part_no * scc_configuration.partition_size_bytes);

	scm_black_part_virt =
	    sahara_partition_base + (SCC_KEY_SLOTS * SCC_KEY_SLOT_SIZE);
	scm_black_part_phys =
	    sahara_partition_phys + (scm_black_part_virt -
				     sahara_partition_base);

	scm_memory_size_bytes = 256;
	scm_red_part_virt = scm_black_part_virt + scm_memory_size_bytes;
	if ((uint32_t) (scm_red_part_virt - sahara_partition_phys) < 256) {
		printk(KERN_ERR
		       "SCC2: not enough space in Sahara partition: too many / too large keys\n");
		retval = SCC_RET_INSUFFICIENT_SPACE;
		goto out;
	}

	scm_red_part_cmd = (((part_no << SCM_CCMD_PART_SHIFT)
			     | ((scm_red_part_virt - sahara_partition_base) /
				SCC_BLOCK_SIZE_BYTES())
			     << SCM_CCMD_OFFSET_SHIFT)
			    | SCM_CCMD_AES);

	pr_debug
	    ("SCC2: Sahara partition %08x/%p; Black RAM: %08x/%p; Red RAM: %p\n",
	     sahara_partition_phys, sahara_partition_base, scm_black_part_phys,
	     scm_black_part_virt, scm_red_part_virt);

	retval = SCC_RET_OK;

      out:
	return retval;
}				/* make_sahara_partition() */

uint8_t make_vpu_partition()
{
	unsigned part_no = VPU_PART_NO;	/* better be free!! */
	uint32_t *part_base =
	    scm_ram_base + (part_no * scc_configuration.partition_size_bytes);
	uint32_t reg_value;

	reg_value = SCC_READ_REGISTER(SCM_PART_OWNERS_REG);

	/* Store SMID to grab a partition */
	for (; part_no < 4; part_no++)
		SCC_WRITE_REGISTER(SCM_SMID0_REG + 8 * part_no, 0x00000000);

	mdelay(2);

	/* Now make sure it is ours... ? */
	reg_value = SCC_READ_REGISTER(SCM_PART_OWNERS_REG);
	for (; part_no < 4; part_no++) {
		if (((reg_value >> (2 * part_no)) & 0x3) != 3) {
			printk(KERN_ERR "Could not acquire partition %u\n",
			       part_no);
			goto out;
		}
	}
	vpu_partition_base = (uint8_t *) part_base;

	mdelay(2);
	for (; part_no < 4; part_no++) {
		part_base =
		    scm_ram_base +
		    (part_no * scc_configuration.partition_size_bytes);
		part_base[0] =
		    SCM_PERM_HD_WRITE | SCM_PERM_HD_READ | SCM_PERM_TH_READ |
		    SCM_PERM_TH_WRITE;
	}
	mdelay(2);

	reg_value = SCC_READ_REGISTER(SCM_PART_ENGAGED_REG);

	if (((reg_value >> part_no) & 1) != 1) {
		printk(KERN_ERR "SCC2 Could not engage partition %u\n",
		       part_no);
		goto out;
	}
	part_no = VPU_PART_NO;
	vpu_partition_phys =
	    (uint32_t) scm_ram_phys_base +
	    (part_no * scc_configuration.partition_size_bytes);

	return vpu_partition_phys;

      out:
	return 0;
}				/* make_vpu_partition */

/*****************************************************************************/
/* fn scc_wait_completion()                                                  */
/*****************************************************************************/
/*!
 * Poll looking for end-of-cipher indication. Only used
 * if @c SCC_SCM_SLEEP is not defined.
 *
 * @internal
 *
 * On a Tahiti, crypto under 230 or so bytes is done after the first loop, all
 * the way up to five sets of spins for 1024 bytes.  (8- and 16-byte functions
 * are done when we first look.  Zeroizing takes one pass around.
 */
static scc_return_t scc_wait_completion(uint32_t * scm_status)
{
	scc_return_t ret;
	int done;
	int i = 0;

	/* check for completion by polling */
	do {
		done = is_cipher_done(scm_status);
		if (done)
			break;
		udelay(1000);
	} while (i++ < SCC_CIPHER_MAX_POLL_COUNT);

	pr_debug("SCC2: Polled DONE %d times\n", i);
	if (!done) {
		ret = SCC_RET_FAIL;
	}

	return ret;
}				/* scc_wait_completion() */

/*****************************************************************************/
/* fn is_cipher_done()                                                       */
/*****************************************************************************/
/*!
 * This function returns non-zero if SCM Status register indicates
 * that a cipher has terminated or some other interrupt-generating
 * condition has occurred.
 */
static int is_cipher_done(uint32_t * scm_status)
{
	register unsigned status;
	register int cipher_done;

	*scm_status = SCC_READ_REGISTER(SCM_STATUS_REG);
	status = (*scm_status & SCM_STATUS_SRS_MASK) >> SCM_STATUS_SRS_SHIFT;

	/*
	 * Done when SCM is not in 'currently performing a function' states.
	 */
	cipher_done = ((status != SCM_STATUS_SRS_ZBUSY)
		       && (status != SCM_STATUS_SRS_CBUSY)
		       && (status != SCM_STATUS_SRS_ABUSY));

	return cipher_done;
}				/* is_cipher_done() */

/*****************************************************************************/
/* fn offset_within_smn()                                                    */
/*****************************************************************************/
/*!
 *  Check that the offset is with the bounds of the SMN register set.
 *
 *  @param[in]  register_offset    register offset of SMN.
 *
 *  @return   1 if true, 0 if false (not within SMN)
 */
static inline int offset_within_smn(uint32_t register_offset)
{
	return ((register_offset >= SMN_STATUS_REG)
		&& (register_offset <= SMN_HAC_REG));
}

/*****************************************************************************/
/* fn offset_within_scm()                                                    */
/*****************************************************************************/
/*!
 *  Check that the offset is with the bounds of the SCM register set.
 *
 *  @param[in]  register_offset    Register offset of SCM
 *
 *  @return   1 if true, 0 if false (not within SCM)
 */
static inline int offset_within_scm(uint32_t register_offset)
{
	return 1;		/* (register_offset >= SCM_RED_START)
				   && (register_offset < scm_highest_memory_address); */
	/* Although this would cause trouble for zeroize testing, this change would
	 * close a security whole which currently allows any kernel program to access
	 * any location in RED RAM.  Perhaps enforce in non-SCC_DEBUG compiles?
	 && (register_offset <= SCM_INIT_VECTOR_1); */
}

/*****************************************************************************/
/* fn check_register_accessible()                                            */
/*****************************************************************************/
/*!
 *  Given the current SCM and SMN status, verify that access to the requested
 *  register should be OK.
 *
 *  @param[in]   register_offset  register offset within SCC
 *  @param[in]   smn_status  recent value from #SMN_STATUS
 *  @param[in]   scm_status  recent value from #SCM_STATUS
 *
 *  @return   #SCC_RET_OK if ok, #SCC_RET_FAIL if not
 */
static scc_return_t
check_register_accessible(uint32_t register_offset, uint32_t smn_status,
			  uint32_t scm_status)
{
	int error_code = SCC_RET_FAIL;

	/* Verify that the register offset passed in is not among the verboten set
	 * if the SMN is in Fail mode.
	 */
	if (offset_within_smn(register_offset)) {
		if ((smn_status & SMN_STATUS_STATE_MASK) == SMN_STATE_FAIL) {
			if (!((register_offset == SMN_STATUS_REG) ||
			      (register_offset == SMN_COMMAND_REG) ||
			      (register_offset == SMN_SEC_VIO_REG))) {
				pr_debug
				    ("SCC2 Driver: Note: Security State is in FAIL state.\n");
			} /* register not a safe one */
			else {
				/* SMN is in  FAIL, but register is a safe one */
				error_code = SCC_RET_OK;
			}
		} /* State is FAIL */
		else {
			/* State is not fail.  All registers accessible. */
			error_code = SCC_RET_OK;
		}
	}
	/* offset within SMN */
	/*  Not SCM register.  Check for SCM busy. */
	else if (offset_within_scm(register_offset)) {
		/* This is the 'cannot access' condition in the SCM */
		if (0		/* (scm_status & SCM_STATUS_BUSY) */
		    /* these are always available  - rest fail on busy */
		    && !((register_offset == SCM_STATUS_REG) ||
			 (register_offset == SCM_ERR_STATUS_REG) ||
			 (register_offset == SCM_INT_CTL_REG) ||
			 (register_offset == SCM_VERSION_REG))) {
			pr_debug
			    ("SCC2 Driver: Note: Secure Memory is in BUSY state.\n");
		} /* status is busy & register inaccessible */
		else {
			error_code = SCC_RET_OK;
		}
	}
	/* offset within SCM */
	return error_code;

}				/* check_register_accessible() */

/*****************************************************************************/
/* fn check_register_offset()                                                */
/*****************************************************************************/
/*!
 *  Check that the offset is with the bounds of the SCC register set.
 *
 *  @param[in]  register_offset    register offset of SMN.
 *
 * #SCC_RET_OK if ok, #SCC_RET_FAIL if not
 */
static scc_return_t check_register_offset(uint32_t register_offset)
{
	int return_value = SCC_RET_FAIL;

	/* Is it valid word offset ? */
	if (SCC_BYTE_OFFSET(register_offset) == 0) {
		/* Yes. Is register within SCM? */
		if (offset_within_scm(register_offset)) {
			return_value = SCC_RET_OK;	/* yes, all ok */
		}
		/* Not in SCM.  Now look within the SMN */
		else if (offset_within_smn(register_offset)) {
			return_value = SCC_RET_OK;	/* yes, all ok */
		}
	}

	return return_value;
}

#ifdef SCC_REGISTER_DEBUG

/*!
 * Names of the SCC Registers, indexed by register number
 */
static char *scc_regnames[] = {
	"SCM_VERSION_REG",
	"0x04",
	"SCM_INT_CTL_REG",
	"SCM_STATUS_REG",
	"SCM_ERR_STATUS_REG",
	"SCM_FAULT_ADR_REG",
	"SCM_PART_OWNERS_REG",
	"SCM_PART_ENGAGED_REG",
	"SCM_UNIQUE_ID0_REG",
	"SCM_UNIQUE_ID1_REG",
	"SCM_UNIQUE_ID2_REG",
	"SCM_UNIQUE_ID3_REG",
	"0x30",
	"0x34",
	"0x38",
	"0x3C",
	"0x40",
	"0x44",
	"0x48",
	"0x4C",
	"SCM_ZCMD_REG",
	"SCM_CCMD_REG",
	"SCM_C_BLACK_ST_REG",
	"SCM_DBG_STATUS_REG",
	"SCM_AES_CBC_IV0_REG",
	"SCM_AES_CBC_IV1_REG",
	"SCM_AES_CBC_IV2_REG",
	"SCM_AES_CBC_IV3_REG",
	"0x70",
	"0x74",
	"0x78",
	"0x7C",
	"SCM_SMID0_REG",
	"SCM_ACC0_REG",
	"SCM_SMID1_REG",
	"SCM_ACC1_REG",
	"SCM_SMID2_REG",
	"SCM_ACC2_REG",
	"SCM_SMID3_REG",
	"SCM_ACC3_REG",
	"SCM_SMID4_REG",
	"SCM_ACC4_REG",
	"SCM_SMID5_REG",
	"SCM_ACC5_REG",
	"SCM_SMID6_REG",
	"SCM_ACC6_REG",
	"SCM_SMID7_REG",
	"SCM_ACC7_REG",
	"SCM_SMID8_REG",
	"SCM_ACC8_REG",
	"SCM_SMID9_REG",
	"SCM_ACC9_REG",
	"SCM_SMID10_REG",
	"SCM_ACC10_REG",
	"SCM_SMID11_REG",
	"SCM_ACC11_REG",
	"SCM_SMID12_REG",
	"SCM_ACC12_REG",
	"SCM_SMID13_REG",
	"SCM_ACC13_REG",
	"SCM_SMID14_REG",
	"SCM_ACC14_REG",
	"SCM_SMID15_REG",
	"SCM_ACC15_REG",
	"SMN_STATUS_REG",
	"SMN_COMMAND_REG",
	"SMN_SEQ_START_REG",
	"SMN_SEQ_END_REG",
	"SMN_SEQ_CHECK_REG",
	"SMN_BB_CNT_REG",
	"SMN_BB_INC_REG",
	"SMN_BB_DEC_REG",
	"SMN_COMPARE_REG",
	"SMN_PT_CHK_REG",
	"SMN_CT_CHK_REG",
	"SMN_TIMER_IV_REG",
	"SMN_TIMER_CTL_REG",
	"SMN_SEC_VIO_REG",
	"SMN_TIMER_REG",
	"SMN_HAC_REG"
};

/*!
 * Names of the Secure RAM States
 */
static char *srs_names[] = {
	"SRS_Reset",
	"SRS_All_Ready",
	"SRS_ZeroizeBusy",
	"SRS_CipherBusy",
	"SRS_AllBusy",
	"SRS_ZeroizeDoneCipherReady",
	"SRS_CipherDoneZeroizeReady",
	"SRS_ZeroizeDoneCipherBusy",
	"SRS_CipherDoneZeroizeBusy",
	"SRS_UNKNOWN_STATE_9",
	"SRS_TransitionalA",
	"SRS_TransitionalB",
	"SRS_TransitionalC",
	"SRS_TransitionalD",
	"SRS_AllDone",
	"SRS_UNKNOWN_STATE_E",
	"SRS_FAIL"
};

/*!
 * Create a text interpretation of the SCM Version Register
 *
 * @param      value        The value of the register
 * @param[out] print_buffer Place to store the interpretation
 * @param      buf_size     Number of bytes available at print_buffer
 *
 * @return The print_buffer
 */
static
char *scm_print_version_reg(uint32_t value, char *print_buffer, int buf_size)
{
	snprintf(print_buffer, buf_size,
		 "Bpp: %u, Bpcb: %u, np: %u, maj: %u, min: %u",
		 (value & SCM_VER_BPP_MASK) >> SCM_VER_BPP_SHIFT,
		 ((value & SCM_VER_BPCB_MASK) >> SCM_VER_BPCB_SHIFT) + 1,
		 ((value & SCM_VER_NP_MASK) >> SCM_VER_NP_SHIFT) + 1,
		 (value & SCM_VER_MAJ_MASK) >> SCM_VER_MAJ_SHIFT,
		 (value & SCM_VER_MIN_MASK) >> SCM_VER_MIN_SHIFT);

	return print_buffer;
}

/*!
 * Create a text interpretation of the SCM Status Register
 *
 * @param      value        The value of the register
 * @param[out] print_buffer Place to store the interpretation
 * @param      buf_size     Number of bytes available at print_buffer
 *
 * @return The print_buffer
 */
static
char *scm_print_status_reg(uint32_t value, char *print_buffer, int buf_size)
{

	snprintf(print_buffer, buf_size, "%s%s%s%s%s%s%s%s%s%s%s%s%s",
		 (value & SCM_STATUS_KST_DEFAULT_KEY) ? "KST_DefaultKey " : "",
		 /* reserved */
		 (value & SCM_STATUS_KST_WRONG_KEY) ? "KST_WrongKey " : "",
		 (value & SCM_STATUS_KST_BAD_KEY) ? "KST_BadKey " : "",
		 (value & SCM_STATUS_ERR) ? "Error " : "",
		 (value & SCM_STATUS_MSS_FAIL) ? "MSS_FailState " : "",
		 (value & SCM_STATUS_MSS_SEC) ? "MSS_SecureState " : "",
		 (value & SCM_STATUS_RSS_FAIL) ? "RSS_FailState " : "",
		 (value & SCM_STATUS_RSS_SEC) ? "RSS_SecureState " : "",
		 (value & SCM_STATUS_RSS_INIT) ? "RSS_Initializing " : "",
		 (value & SCM_STATUS_UNV) ? "UID_Invalid " : "",
		 (value & SCM_STATUS_BIG) ? "BigEndian " : "",
		 (value & SCM_STATUS_USK) ? "SecretKey " : "",
		 srs_names[(value & SCM_STATUS_SRS_MASK) >>
			   SCM_STATUS_SRS_SHIFT]);

	return print_buffer;
}

/*!
 * Names of the SCM Error Codes
 */
static
char *scm_err_code[] = {
	"Unknown_0",
	"UnknownAddress",
	"UnknownCommand",
	"ReadPermErr",
	"WritePermErr",
	"DMAErr",
	"EncBlockLenOvfl",
	"KeyNotEngaged",
	"ZeroizeCmdQOvfl",
	"CipherCmdQOvfl",
	"ProcessIntr",
	"WrongKey",
	"DeviceBusy",
	"DMAUnalignedAddr",
	"Unknown_E",
	"Unknown_F",
};

/*!
 * Names of the SMN States
 */
static char *smn_state_name[] = {
	"Start",
	"Invalid_01",
	"Invalid_02",
	"Invalid_03",
	"Zeroizing_04",
	"Zeroizing",
	"HealthCheck",
	"HealthCheck_07",
	"Invalid_08",
	"Fail",
	"Secure",
	"Invalid_0B",
	"NonSecure",
	"Invalid_0D",
	"Invalid_0E",
	"Invalid_0F",
	"Invalid_10",
	"Invalid_11",
	"Invalid_12",
	"Invalid_13",
	"Invalid_14",
	"Invalid_15",
	"Invalid_16",
	"Invalid_17",
	"Invalid_18",
	"FailHard",
	"Invalid_1A",
	"Invalid_1B",
	"Invalid_1C",
	"Invalid_1D",
	"Invalid_1E",
	"Invalid_1F"
};

/*!
 * Create a text interpretation of the SCM Error Status Register
 *
 * @param      value        The value of the register
 * @param[out] print_buffer Place to store the interpretation
 * @param      buf_size     Number of bytes available at print_buffer
 *
 * @return The print_buffer
 */
static
char *scm_print_err_status_reg(uint32_t value, char *print_buffer, int buf_size)
{
	snprintf(print_buffer, buf_size,
		 "MID: 0x%x, %s%s ErrorCode: %s, SMSState: %s, SCMState: %s",
		 (value & SCM_ERRSTAT_MID_MASK) >> SCM_ERRSTAT_MID_SHIFT,
		 (value & SCM_ERRSTAT_ILM) ? "ILM, " : "",
		 (value & SCM_ERRSTAT_SUP) ? "SUP, " : "",
		 scm_err_code[(value & SCM_ERRSTAT_ERC_MASK) >>
			      SCM_ERRSTAT_ERC_SHIFT],
		 smn_state_name[(value & SCM_ERRSTAT_SMS_MASK) >>
				SCM_ERRSTAT_SMS_SHIFT],
		 srs_names[(value & SCM_ERRSTAT_SRS_MASK) >>
			   SCM_ERRSTAT_SRS_SHIFT]);
	return print_buffer;
}

/*!
 * Create a text interpretation of the SCM Zeroize Command Register
 *
 * @param      value        The value of the register
 * @param[out] print_buffer Place to store the interpretation
 * @param      buf_size     Number of bytes available at print_buffer
 *
 * @return The print_buffer
 */
static
char *scm_print_zcmd_reg(uint32_t value, char *print_buffer, int buf_size)
{
	unsigned cmd = (value & SCM_ZCMD_CCMD_MASK) >> SCM_CCMD_CCMD_SHIFT;

	snprintf(print_buffer, buf_size, "%s %u",
		 (cmd ==
		  ZCMD_DEALLOC_PART) ? "DeallocPartition" :
		 "(unknown function)",
		 (value & SCM_ZCMD_PART_MASK) >> SCM_ZCMD_PART_SHIFT);

	return print_buffer;
}

/*!
 * Create a text interpretation of the SCM Cipher Command Register
 *
 * @param      value        The value of the register
 * @param[out] print_buffer Place to store the interpretation
 * @param      buf_size     Number of bytes available at print_buffer
 *
 * @return The print_buffer
 */
static
char *scm_print_ccmd_reg(uint32_t value, char *print_buffer, int buf_size)
{
	unsigned cmd = (value & SCM_CCMD_CCMD_MASK) >> SCM_CCMD_CCMD_SHIFT;

	snprintf(print_buffer, buf_size,
		 "%s %u bytes, %s offset 0x%x, in partition %u",
		 (cmd == SCM_CCMD_AES_DEC_ECB) ? "ECB Decrypt" : (cmd ==
								  SCM_CCMD_AES_ENC_ECB)
		 ? "ECB Encrypt" : (cmd ==
				    SCM_CCMD_AES_DEC_CBC) ? "CBC Decrypt" : (cmd
									     ==
									     SCM_CCMD_AES_ENC_CBC)
		 ? "CBC Encrypt" : "(unknown function)",
		 16 +
		 16 * ((value & SCM_CCMD_LENGTH_MASK) >> SCM_CCMD_LENGTH_SHIFT),
		 ((cmd == SCM_CCMD_AES_ENC_CBC)
		  || (cmd == SCM_CCMD_AES_ENC_ECB)) ? "at" : "to",
		 16 * ((value & SCM_CCMD_OFFSET_MASK) >> SCM_CCMD_OFFSET_SHIFT),
		 (value & SCM_CCMD_PART_MASK) >> SCM_CCMD_PART_SHIFT);

	return print_buffer;
}

/*!
 * Create a text interpretation of an SCM Access Permissions Register
 *
 * @param      value        The value of the register
 * @param[out] print_buffer Place to store the interpretation
 * @param      buf_size     Number of bytes available at print_buffer
 *
 * @return The print_buffer
 */
static
char *scm_print_acc_reg(uint32_t value, char *print_buffer, int buf_size)
{
	snprintf(print_buffer, buf_size, "%s%s%s%s%s%s%s%s%s%s",
		 (value & SCM_PERM_NO_ZEROIZE) ? "NO_ZERO " : "",
		 (value & SCM_PERM_HD_SUP_DISABLE) ? "SUP_DIS " : "",
		 (value & SCM_PERM_HD_READ) ? "HD_RD " : "",
		 (value & SCM_PERM_HD_WRITE) ? "HD_WR " : "",
		 (value & SCM_PERM_HD_EXECUTE) ? "HD_EX " : "",
		 (value & SCM_PERM_TH_READ) ? "TH_RD " : "",
		 (value & SCM_PERM_TH_WRITE) ? "TH_WR " : "",
		 (value & SCM_PERM_OT_READ) ? "OT_RD " : "",
		 (value & SCM_PERM_OT_WRITE) ? "OT_WR " : "",
		 (value & SCM_PERM_OT_EXECUTE) ? "OT_EX" : "");

	return print_buffer;
}

/*!
 * Create a text interpretation of the SCM Partitions Engaged Register
 *
 * @param      value        The value of the register
 * @param[out] print_buffer Place to store the interpretation
 * @param      buf_size     Number of bytes available at print_buffer
 *
 * @return The print_buffer
 */
static
char *scm_print_part_eng_reg(uint32_t value, char *print_buffer, int buf_size)
{
	snprintf(print_buffer, buf_size, "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
		 (value & 0x8000) ? "15 " : "",
		 (value & 0x4000) ? "14 " : "",
		 (value & 0x2000) ? "13 " : "",
		 (value & 0x1000) ? "12 " : "",
		 (value & 0x0800) ? "11 " : "",
		 (value & 0x0400) ? "10 " : "",
		 (value & 0x0200) ? "9 " : "",
		 (value & 0x0100) ? "8 " : "",
		 (value & 0x0080) ? "7 " : "",
		 (value & 0x0040) ? "6 " : "",
		 (value & 0x0020) ? "5 " : "",
		 (value & 0x0010) ? "4 " : "",
		 (value & 0x0008) ? "3 " : "",
		 (value & 0x0004) ? "2 " : "",
		 (value & 0x0002) ? "1 " : "", (value & 0x0001) ? "0" : "");

	return print_buffer;
}

/*!
 * Create a text interpretation of the SMN Status Register
 *
 * @param      value        The value of the register
 * @param[out] print_buffer Place to store the interpretation
 * @param      buf_size     Number of bytes available at print_buffer
 *
 * @return The print_buffer
 */
static
char *smn_print_status_reg(uint32_t value, char *print_buffer, int buf_size)
{
	snprintf(print_buffer, buf_size,
		 "Version %d %s%s%s%s%s%s%s%s%s%s%s%s%s",
		 (value & SMN_STATUS_VERSION_ID_MASK) >>
		 SMN_STATUS_VERSION_ID_SHIFT,
		 (value & SMN_STATUS_ILLEGAL_MASTER) ? "IllMaster " : "",
		 (value & SMN_STATUS_SCAN_EXIT) ? "ScanExit " : "",
		 (value & SMN_STATUS_PERIP_INIT) ? "PeripInit " : "",
		 (value & SMN_STATUS_SMN_ERROR) ? "SMNError " : "",
		 (value & SMN_STATUS_SOFTWARE_ALARM) ? "SWAlarm " : "",
		 (value & SMN_STATUS_TIMER_ERROR) ? "TimerErr " : "",
		 (value & SMN_STATUS_PC_ERROR) ? "PTCTErr " : "",
		 (value & SMN_STATUS_BITBANK_ERROR) ? "BitbankErr " : "",
		 (value & SMN_STATUS_ASC_ERROR) ? "ASCErr " : "",
		 (value & SMN_STATUS_SECURITY_POLICY_ERROR) ? "SecPlcyErr " :
		 "",
		 (value & SMN_STATUS_SEC_VIO_ACTIVE_ERROR) ? "SecVioAct " : "",
		 (value & SMN_STATUS_INTERNAL_BOOT) ? "IntBoot " : "",
		 smn_state_name[(value & SMN_STATUS_STATE_MASK) >>
				SMN_STATUS_STATE_SHIFT]);

	return print_buffer;
}

/*!
 * The array, indexed by register number (byte-offset / 4), of print routines
 * for the SCC (SCM and SMN) registers.
 */
static reg_print_routine_t reg_printers[] = {
	scm_print_version_reg,
	NULL,			//"0x04",
	NULL,			//"SCM_INT_CTL_REG",
	scm_print_status_reg,
	scm_print_err_status_reg,
	NULL,			//"SCM_FAULT_ADR_REG",
	NULL,			//"SCM_PART_OWNERS_REG",
	scm_print_part_eng_reg,
	NULL,			//"SCM_UNIQUE_ID0_REG",
	NULL,			//"SCM_UNIQUE_ID1_REG",
	NULL,			//"SCM_UNIQUE_ID2_REG",
	NULL,			//"SCM_UNIQUE_ID3_REG",
	NULL,			//"0x30",
	NULL,			//"0x34",
	NULL,			//"0x38",
	NULL,			//"0x3C",
	NULL,			//"0x40",
	NULL,			//"0x44",
	NULL,			//"0x48",
	NULL,			//"0x4C",
	scm_print_zcmd_reg,
	scm_print_ccmd_reg,
	NULL,			//"SCM_C_BLACK_ST_REG",
	NULL,			//"SCM_DBG_STATUS_REG",
	NULL,			//"SCM_AES_CBC_IV0_REG",
	NULL,			//"SCM_AES_CBC_IV1_REG",
	NULL,			//"SCM_AES_CBC_IV2_REG",
	NULL,			//"SCM_AES_CBC_IV3_REG",
	NULL,			//"0x70",
	NULL,			//"0x74",
	NULL,			//"0x78",
	NULL,			//"0x7C",
	NULL,			//"SCM_SMID0_REG",
	scm_print_acc_reg,	/* ACC0 */
	NULL,			//"SCM_SMID1_REG",
	scm_print_acc_reg,	/* ACC1 */
	NULL,			//"SCM_SMID2_REG",
	scm_print_acc_reg,	/* ACC2 */
	NULL,			//"SCM_SMID3_REG",
	scm_print_acc_reg,	/* ACC3 */
	NULL,			//"SCM_SMID4_REG",
	scm_print_acc_reg,	/* ACC4 */
	NULL,			//"SCM_SMID5_REG",
	scm_print_acc_reg,	/* ACC5 */
	NULL,			//"SCM_SMID6_REG",
	scm_print_acc_reg,	/* ACC6 */
	NULL,			//"SCM_SMID7_REG",
	scm_print_acc_reg,	/* ACC7 */
	NULL,			//"SCM_SMID8_REG",
	scm_print_acc_reg,	/* ACC8 */
	NULL,			//"SCM_SMID9_REG",
	scm_print_acc_reg,	/* ACC9 */
	NULL,			//"SCM_SMID10_REG",
	scm_print_acc_reg,	/* ACC10 */
	NULL,			//"SCM_SMID11_REG",
	scm_print_acc_reg,	/* ACC11 */
	NULL,			//"SCM_SMID12_REG",
	scm_print_acc_reg,	/* ACC12 */
	NULL,			//"SCM_SMID13_REG",
	scm_print_acc_reg,	/* ACC13 */
	NULL,			//"SCM_SMID14_REG",
	scm_print_acc_reg,	/* ACC14 */
	NULL,			//"SCM_SMID15_REG",
	scm_print_acc_reg,	/* ACC15 */
	smn_print_status_reg,
	NULL,			//"SMN_COMMAND_REG",
	NULL,			//"SMN_SEQ_START_REG",
	NULL,			//"SMN_SEQ_END_REG",
	NULL,			//"SMN_SEQ_CHECK_REG",
	NULL,			//"SMN_BB_CNT_REG",
	NULL,			//"SMN_BB_INC_REG",
	NULL,			//"SMN_BB_DEC_REG",
	NULL,			//"SMN_COMPARE_REG",
	NULL,			//"SMN_PT_CHK_REG",
	NULL,			//"SMN_CT_CHK_REG",
	NULL,			//"SMN_TIMER_IV_REG",
	NULL,			//"SMN_TIMER_CTL_REG",
	NULL,			//"SMN_SEC_VIO_REG",
	NULL,			//"SMN_TIMER_REG",
	NULL,			//"SMN_HAC_REG"
};

/*****************************************************************************/
/* fn dbg_scc_read_register()                                                */
/*****************************************************************************/
/*!
 * Noisily read a 32-bit value to an SCC register.
 * @param offset        The address of the register to read.
 *
 * @return  The register value
 * */
uint32_t dbg_scc_read_register(uint32_t offset)
{
	uint32_t value;
	char *regname = scc_regnames[offset / 4];

	value = __raw_readl(scc_base + offset);
	pr_debug("SCC2 RD: 0x%03x : 0x%08x (%s) %s\n", offset, value, regname,
		 reg_printers[offset / 4]
		 ? reg_printers[offset / 4] (value, reg_print_buffer,
					     REG_PRINT_BUFFER_SIZE)
		 : "");

	return value;
}

/*****************************************************************************/
/* fn dbg_scc_write_register()                                               */
/*****************************************************************************/
/*
 * Noisily read a 32-bit value to an SCC register.
 * @param offset        The address of the register to written.
 *
 * @param value         The new register value
 */
void dbg_scc_write_register(uint32_t offset, uint32_t value)
{
	char *regname = scc_regnames[offset / 4];

	pr_debug("SCC2 WR: 0x%03x : 0x%08x (%s) %s\n", offset, value, regname,
		 reg_printers[offset / 4]
		 ? reg_printers[offset / 4] (value, reg_print_buffer,
					     REG_PRINT_BUFFER_SIZE)
		 : "");
	(void)__raw_writel(value, scc_base + offset);
}

#endif				/* SCC_REGISTER_DEBUG */
