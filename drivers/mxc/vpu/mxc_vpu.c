/*
 * Copyright 2006 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mxc_vpu.c
 *
 * @brief VPU system initialization and file operation implementation
 *
 * @ingroup VPU
 */

#ifdef	CONFIG_MXC_VPU_DEBUG
#define	DEBUG
#include <linux/kernel.h>
#endif

#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/autoconf.h>
#include <linux/ioport.h>
#include <linux/stat.h>
#include <linux/platform_device.h>
#include <linux/kdev_t.h>
#include <linux/dma-mapping.h>
#include <linux/wait.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <asm/dma-mapping.h>
#include <asm/hardware.h>

#include <asm/arch/clock.h>
#include <asm/arch/mxc_vpu.h>

#define	BIT_INT_CLEAR		0x00C
#define	BIT_INT_STATUS		0x010
#define BIT_INT_ENABLE		0x170

typedef struct vpu_t {
	struct fasync_struct *async_queue;
} vpu_t;

static int vpu_major = 0;
static struct class *vpu_class;
static struct vpu_t vpu_data;
static u8 open_count = 0;

#define	MAX_BUFS_NUMS		20
static vpu_mem_desc mem[MAX_BUFS_NUMS];
static unsigned int buf_idx;

/* implement the blocking ioctl */
static int codec_done = 0;
static wait_queue_head_t vpu_queue;

/*!
 * Private function to free buffers
 * @param mem		Array of buffers to be freed
 * @param num_buf	Number of buffers to be freed
 * @return status  0 success.
 */
static int vpu_free_buffers(vpu_mem_desc mem[], int num_buf)
{
	int i;

	for (i = 0; i < num_buf; i++) {
		if (mem[i].cpu_addr != 0) {
			dma_free_coherent(0, mem[i].size,
					  (void *)mem[i].cpu_addr,
					  mem[i].phy_addr);
			pr_debug("freed idx %u @ paddr=0x%08X\n", i,
				 mem[i].phy_addr);
			mem[i].phy_addr = 0;
			mem[i].cpu_addr = 0;
		}
	}
	buf_idx = 0;

	return 0;
}

/*!
 * @brief vpu interrupt handler
 */
static irqreturn_t vpu_irq_handler(int irq, void *dev_id)
{
	struct vpu_t *dev;
	dev = (struct vpu_t *)dev_id;
	__raw_readl(IO_ADDRESS(VPU_BASE_ADDR + BIT_INT_STATUS));
	__raw_writel(0x1, IO_ADDRESS(VPU_BASE_ADDR + BIT_INT_CLEAR));
	if (dev->async_queue)
		kill_fasync(&dev->async_queue, SIGIO, POLL_IN);

	codec_done = 1;
	wake_up_interruptible(&vpu_queue);

	return IRQ_HANDLED;
}

/*!
 * @brief vpu hardware enable function
 *
 * @return  0 on success or negative error code on error
 */
static int vpu_hardware_enable(void)
{
	mxc_clks_enable(VPU_BAUD);
	/* enable user space access for vpu register */
	__raw_writel(0x1, IO_ADDRESS(AIPI_BASE_ADDR + 0x20008));
	return 0;
}

/*!
 * @brief vpu hardware disable function
 *
 * @return  0 on success or negative error code on error
 */
static int vpu_hardware_disable(void)
{
	mxc_clks_disable(VPU_BAUD);
	__raw_writel(0xffffffff, IO_ADDRESS(AIPI_BASE_ADDR + 0x20008));
	return 0;

}

/*!
 * @brief open function for vpu file operation
 *
 * @return  0 on success or negative error code on error
 */
static int vpu_open(struct inode *inode, struct file *filp)
{
	if (open_count++ == 0) {
		filp->private_data = (void *)(&vpu_data);
		vpu_hardware_enable();
	} else {
		printk(KERN_ERR "VPU has already been opened.\n");
		return -EACCES;
	}

	return 0;
}

/*!
 * @brief IO ctrl function for vpu file operation
 * @param cmd IO ctrl command
 * @return  0 on success or negative error code on error
 */
static int vpu_ioctl(struct inode *inode, struct file *filp, u_int cmd,
		     u_long arg)
{
	int ret = 0;

	switch (cmd) {
	case VPU_IOC_PHYMEM_ALLOC:
		{
			copy_from_user(&mem[buf_idx], (vpu_mem_desc *) arg,
				       sizeof(vpu_mem_desc));
			pr_debug("mem alloc size = 0x%x\n", mem[buf_idx].size);
			mem[buf_idx].cpu_addr = (unsigned long)
			    dma_alloc_coherent(NULL,
					       PAGE_ALIGN(mem[buf_idx].size),
					       (dma_addr_t
						*) (&(mem[buf_idx].phy_addr)),
					       GFP_DMA | GFP_KERNEL);
			pr_debug("mem alloc cpu_addr = 0x%x, buf_idx = %u\n",
				 mem[buf_idx].cpu_addr, buf_idx);
			if ((void *)(mem[buf_idx].cpu_addr) == NULL) {
				printk(KERN_ERR
				       "Physical memory allocation error!\n");
				ret = -1;
				break;
			}
			copy_to_user((void __user *)arg, &mem[buf_idx],
				     sizeof(vpu_mem_desc));
			buf_idx++;
			break;
		}
	case VPU_IOC_PHYMEM_FREE:
		{
			int i;
			vpu_mem_desc vpu_mem;
			copy_from_user(&vpu_mem, (vpu_mem_desc *) arg,
				       sizeof(vpu_mem_desc));
			pr_debug("mem freed cpu_addr = 0x%x\n",
				 vpu_mem.cpu_addr);
			if ((void *)vpu_mem.cpu_addr != NULL) {
				dma_free_coherent(NULL,
						  PAGE_ALIGN(vpu_mem.size),
						  (void *)vpu_mem.cpu_addr,
						  (dma_addr_t) vpu_mem.
						  phy_addr);
			}

			/* mark the freed buffers */
			for (i = 0; i < buf_idx; i++) {
				if (mem[i].cpu_addr == vpu_mem.cpu_addr) {
					mem[i].phy_addr = 0;
					mem[i].cpu_addr = 0;
					break;
				}
			}

			break;
		}
	case VPU_IOC_WAIT4INT:
		{
			u_long timeout = (u_long) arg;
			if (!wait_event_interruptible_timeout
			    (vpu_queue, codec_done != 0,
			     msecs_to_jiffies(timeout))) {
				printk("VPU blocking: timeout.\n");
				ret = -ETIME;
			} else if (signal_pending(current)) {
				printk("VPU interrupt received.\n");
				ret = -ERESTARTSYS;
			}

			codec_done = 0;
			break;
		}
	case VPU_IOC_REG_DUMP:
		break;
	case VPU_IOC_PHYMEM_DUMP:
		break;
	default:
		{
			printk(KERN_ERR "No such IOCTL, cmd is %d\n", cmd);
			break;
		}
	}
	return ret;
}

/*!
 * @brief Release function for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_release(struct inode *inode, struct file *filp)
{
	if (--open_count == 0) {
		__raw_writel(0x0, IO_ADDRESS(VPU_BASE_ADDR + BIT_INT_ENABLE));
		vpu_free_buffers(mem, buf_idx);
		vpu_hardware_disable();
	}

	return 0;
}

/*!
 * @brief fasync function for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_fasync(int fd, struct file *filp, int mode)
{
	struct vpu_t *dev = (struct vpu_t *)filp->private_data;
	return fasync_helper(fd, filp, mode, &dev->async_queue);
}

/*!
 * @brief memory map function of harware registers for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_map_hwregs(struct file *fp, struct vm_area_struct *vm)
{
	unsigned long pfn;

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
	pfn = VPU_BASE_ADDR >> PAGE_SHIFT;
	pr_debug("size=0x%x,  page no.=0x%x\n",
		 (int)(vm->vm_end - vm->vm_start), (int)pfn);
	return remap_pfn_range(vm, vm->vm_start, pfn, vm->vm_end - vm->vm_start,
			       vm->vm_page_prot) ? -EAGAIN : 0;
}

/*!
 * @brief memory map function of memory for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_map_mem(struct file *fp, struct vm_area_struct *vm)
{
	int request_size;
	request_size = vm->vm_end - vm->vm_start;

	pr_debug(" start=0x%x, pgoff=0x%x, size=0x%x\n",
		 (unsigned int)(vm->vm_start), (unsigned int)(vm->vm_pgoff),
		 request_size);

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);

	return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff,
			       request_size, vm->vm_page_prot) ? -EAGAIN : 0;

}

/*!
 * @brief memory map interface for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_mmap(struct file *fp, struct vm_area_struct *vm)
{
	if (vm->vm_pgoff)
		return vpu_map_mem(fp, vm);
	else
		return vpu_map_hwregs(fp, vm);
}

struct file_operations vpu_fops = {
	.owner = THIS_MODULE,
	.open = vpu_open,
	.ioctl = vpu_ioctl,
	.release = vpu_release,
	.fasync = vpu_fasync,
	.mmap = vpu_mmap,
};

/*!
 * This function is called by the driver framework to initialize the vpu device.
 * @param   dev The device structure for the vpu passed in by the framework.
 * @return   0 on success or negative error code on error
 */
static int vpu_dev_probe(struct platform_device *pdev)
{
	int err = 0;
	struct class_device *temp_class;

	vpu_major = register_chrdev(vpu_major, "mxc_vpu", &vpu_fops);
	if (vpu_major < 0) {
		printk(KERN_ERR "vpu: unable to get a major for VPU\n");
		err = -EBUSY;
		return vpu_major;
	}

	vpu_class = class_create(THIS_MODULE, "mxc_vpu");
	if (IS_ERR(vpu_class)) {
		err = PTR_ERR(vpu_class);
		goto err_out_chrdev;
	}

	temp_class = class_device_create(vpu_class, NULL,
					 MKDEV(vpu_major, 0), NULL, "mxc_vpu");
	if (IS_ERR(temp_class)) {
		err = PTR_ERR(temp_class);
		goto err_out_class;
	}

	request_irq(INT_VPU, vpu_irq_handler, 0, "VPU_CODEC_IRQ",
		    (void *)(&vpu_data));

	err = 0;
	goto out;

      err_out_class:
	class_device_destroy(vpu_class, MKDEV(vpu_major, 0));
	class_destroy(vpu_class);
      err_out_chrdev:
	unregister_chrdev(vpu_major, "mxc_vpu");
      out:
	return err;
}

/*! Driver definition
 *
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxcvpu_driver = {
	.driver = {
		   .name = "mxc_vpu",
		   },
	.probe = vpu_dev_probe,
};

static int __init vpu_init(void)
{
	int ret = platform_driver_register(&mxcvpu_driver);

	init_waitqueue_head(&vpu_queue);

	return ret;
}

static void __exit vpu_exit(void)
{
	free_irq(INT_VPU, (void *)(&vpu_data));
	if (vpu_major > 0) {
		class_device_destroy(vpu_class, MKDEV(vpu_major, 0));
		class_destroy(vpu_class);
		if (unregister_chrdev(vpu_major, "mxc_vpu") < 0) {
			printk(KERN_ERR
			       "Failed to unregister vpu from devfs\n");
			return;
		}
		vpu_major = 0;
	}

	platform_driver_unregister(&mxcvpu_driver);
	return;
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Linux VPU driver for Freescale i.MX27");
MODULE_LICENSE("GPL");

module_init(vpu_init);
module_exit(vpu_exit);
