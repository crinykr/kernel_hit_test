/*
 *  mn63y1210_spi.c - Linux kernel module for
 * 	Panasonic RFID driver MN63Y1210 as SPI driver.
 *
 *  Copyright (c) 2014 STcube Inc.,
 *  All right reserved by Seungwoo Kim <ksw.stcube.com> 
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>

#include <linux/major.h>
#include <linux/errno.h>
#include <linux/tty.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/vt_kern.h>
#include <linux/selection.h>
#include <linux/console.h>

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/mach-types.h>

#include <mach/platform.h>

#include "mn63y1210_spi.h"

#if (0)
#define DBGOUT(msg...)	{ printk(KERN_INFO "mn63y1210: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define DEBUG_DUMP	0

#define MN63Y1210_DRV_NAME	"mn63y1210"
#define DRIVER_VERSION		"0.8"

#define RFID_HOST_READ_CMD		0x08
#define RFID_HOST_WRITE_CMD		0x18
#define RFID_TUNNEL_ANSWER_CMD		0x28

#define RFID_TUNNEL_RESPOND_NORMAL		0xF8
#define RFID_TUNNEL_RESPOND_ERROR		0xE8

#define RFID_RESPOND_NORMAL_END		0x05
#define RFID_RESPOND_NORMAL_CRC		0xFB

#define MN63Y1210_DEV_MAJOR			222

#if defined(CONFIG_MODEL_RK_HIT_N)
#define MN63Y1210_IRQ	(IRQ_GPIO_A_START + 3)
#define MN63Y1210_IRQ_MASK  	(1 << 3)
#else
#define MN63Y1210_IRQ	(IRQ_GPIO_B_START + 13)
#define MN63Y1210_IRQ_MASK  	(1 << 13)
#endif


// FLAG definition
#define MN63Y1210_FLAG_READWRITE	0x0001
#define MN63Y1210_FLAG_TUNNELMODE	0x0002
#define MN63Y1210_FLAG_STATUS_MASK	0x000F
#define MN63Y1210_FLAG_IRQSET		0x0100
#define MN63Y1210_FLAG_TUNNELIRQ	0x0200
#define MN63Y1210_FLAG_IRQ_MASK		0x0F00
#define MN63Y1210_FLAG_DATA_BUFFER  0x0010
#define MN63Y1210_FLAG_MASK_DATA    0x00F0

#define UNMASK_STATUS(x)			(x->flag &= ~MN63Y1210_FLAG_STATUS_MASK)
#define UNMASK_IRQ(x)				(x->flag &= ~MN63Y1210_FLAG_IRQ_MASK)
#define SET_READWRITE(x)			(x->flag |= MN63Y1210_FLAG_READWRITE)
#define SET_TUNNELMODE(x)			(x->flag |= MN63Y1210_FLAG_TUNNELMODE)
#define CLEAR_TUNNELMODE(x)			(x->flag &= ~MN63Y1210_FLAG_TUNNELMODE)
#define SET_DATAONBUFFER(x)			(x->flag |= MN63Y1210_FLAG_DATA_BUFFER)
#define CLEAR_DATA(x)				(x->flag &= MN63Y1210_FLAG_MASK_DATA)
#define SET_IRQ(x)					(x->flag |= MN63Y1210_FLAG_IRQSET)
#define SET_TUNNEL_IRQ(x)			(x->flag |= MN63Y1210_FLAG_TUNNELIRQ)
#define IS_IRQ_SET(x)				(x->flag & MN63Y1210_FLAG_IRQSET)
#define IS_TUNNEL_IRQ_SET(x)		(x->flag & MN63Y1210_FLAG_TUNNELIRQ)
#define IS_READWRITE_SET(x)			(x->flag & MN63Y1210_FLAG_READWRITE)

typedef struct {
	struct spi_device *spi;
	unsigned int cur_status;
	unsigned int flag;
	unsigned int irq;
	unsigned int timeout;
	unsigned int power_dn;
	
	struct mutex                rw_lock;
	struct mutex                tunnel_lock;
	wait_queue_head_t           waitq;
	wait_queue_head_t           tunnel_waitq;
	
	struct workqueue_struct *tunnel_workq;
	struct work_struct  		work;
	
	struct NX_GPIO_RegisterSet *gpio;

	unsigned int tunnel_status;
	unsigned int int_data_len;
	unsigned char int_data_buffer[256];
} mn63y1210_device_t;

mn63y1210_device_t *mn63y1210=NULL;
static struct class *mn63y1210_class=NULL;

static struct NX_GPIO_RegisterSet *gpioa = (struct NX_GPIO_RegisterSet *)(IO_ADDRESS(PHY_BASEADDR_GPIO + 0x00));
static struct NX_GPIO_RegisterSet *gpioc = (struct NX_GPIO_RegisterSet *)(IO_ADDRESS(PHY_BASEADDR_GPIO + 0x80));

static ssize_t mn63y1210_store_val(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	unsigned char tmp[2];
	unsigned long val;

	if (strict_strtoul(buf, 10, &val) < 0)
		return -EINVAL;

	tmp[0] = val >> 8;
	tmp[1] = val & 0xff;
	spi_write(spi, tmp, sizeof(tmp));
	return count;
}

static DEVICE_ATTR(value, S_IWUSR, NULL, mn63y1210_store_val);

static struct attribute *mn63y1210_attributes[] = {
	&dev_attr_value.attr,
	NULL
};

static const struct attribute_group mn63y1210_attr_group = {
	.attrs = mn63y1210_attributes,
};

static inline void spi_message_init_with_transfers(struct spi_message *m, struct spi_transfer *xfers, unsigned int num_xfers)
{
	unsigned int i;

	spi_message_init(m);
	for (i = 0; i < num_xfers; ++i)
		spi_message_add_tail(&xfers[i], m);
}

static inline int spi_sync_transfer(struct spi_device *spi, struct spi_transfer *xfers, unsigned int num_xfers)
{
	struct spi_message msg;

	spi_message_init_with_transfers(&msg, xfers, num_xfers);

	return spi_sync(spi, &msg);
}

static void mn63y1210_init_hw()
{
	gpioc->GPIOxALTFN[0] &= 0xCFFFFFFF; /* GPIO 14 to GPIO */
	gpioc->GPIOxOUTENB   |= 0x00004000;
	gpioc->GPIOxOUT      |= (0x01 << 14);
}

/*--------------------------------------------------------------------------------
 * file_operations
 ---------------------------------------------------------------------------------*/
static int mn63y1210_open(struct inode *inode, struct file *flip)
{
	DBGOUT("%s (minor:%d)\n", __func__, minor);
	
	flip->private_data = mn63y1210;

	mn63y1210_init_hw();

	return 0;
}

static int mn63y1210_release(struct inode *inode, struct file *flip)
{
	DBGOUT("%s (minor:%d, table:%d)\n", __func__, minor, NUM_OF_PARTS);

	return 0;
}

static void mn63y1210_power(mn63y1210_device_t *mn63, int power)
{
	if (power)
	{
		gpioc->GPIOxOUT |= (0x1 << 14);
		mn63->power_dn = 0;
	}
	else
	{
		mn63->power_dn = 1;
		gpioc->GPIOxOUT &= ~(0x1 << 14);
	}
}

static void mn63y1210_reset(mn63y1210_device_t *mn63)
{
	mn63y1210_power(mn63, 0);
	msleep(15);
	mn63y1210_power(mn63, 1);
}

static int checksum(int len, unsigned char *data)
{
	int i;
	unsigned char sum = 0;

	for (i=0; i<len; i++) {
		sum += (((unsigned char)0xFF) - data[i]) + 1;
	}
	return sum;
}

#if DEBUG_DUMP
static int dump(int addr, int size, char *buf)
{
	int ret;

	for (ret = 0; ret < size; ret++, addr++) {
		if (0 == (ret % 16)) {
			if ( 0 != ret)
				printk("\n");
			printk("0x%.3X:", addr);
		}
		printk("%.2X ", buf[ret]);
	}
	printk("\n");
	return 0;
}
#endif

static int build_read_cmd(char *buf, unsigned char cmd, short int addr, short int size)
{
	buf[0] = cmd;
	buf[1] = (addr >> 8) & 0xFF;
	buf[2] = addr & 0xFF;
	buf[3] = size & 0xFF;
	buf[4] = checksum(4, buf); 
	return 5;
}

static int build_write_cmd(char *buf, unsigned char cmd, short int addr, short int size, char *data)
{
	int i;

	buf[0] = cmd;
	buf[1] = (addr >> 8) & 0xFF;
	buf[2] = addr & 0xFF;
	buf[3] = size & 0xFF;
	for (i=0; i< size; i++) {
		buf[4+i] = data[i];
	}
	buf[4+size] = checksum(size+4, buf);
	
	return size + 5;
}

static int build_data_cmd(char *buf, unsigned char cmd, short int size, char *data)
{
	int i;

	buf[0] = cmd;
	for (i=0; i< size; i++) {
		buf[1+i] = data[i];
	}
	buf[1+size] = checksum(size+1, buf);
	
	return size + 2;
}

static int build_status_cmd(char *buf, char cmd)
{
	buf[0] = cmd;
	buf[1] = checksum(1, buf);
	
	return 2;
}

/*--------------------------------------------------------------------------------
	TUNNEL WORK FUNCTION
---------------------------------------------------------------------------------*/
static void tunnel_work_func(struct work_struct *work)
{
	mn63y1210_device_t *mn63 = container_of(work, mn63y1210_device_t, work);
	struct spi_transfer xfer;
	char txbuf[256], rxbuf[256];
	int len, ret;

	mutex_lock(&mn63->tunnel_lock);
	if ((mn63->gpio->GPIOxPAD & MN63Y1210_IRQ_MASK) != 0) {
		mutex_unlock(&mn63->tunnel_lock);
		return;
	}
	/* Now we should write OK signal to NFC controller */
	/* Normal read/write should be blocked. */
	UNMASK_IRQ(mn63);
	SET_READWRITE(mn63);
	mutex_lock(&mn63->rw_lock);

	/* do write OK to NFC controller */
	len = build_status_cmd(txbuf, RFID_TUNNEL_ANSWER_CMD); // QUERY Command(Host to RFID)
	memset(&xfer, 0, sizeof(xfer));
	xfer.rx_buf = rxbuf;
	xfer.tx_buf = txbuf;
	xfer.len = len;
	spi_sync_transfer(mn63->spi, &xfer, 1);

	/* wait till irq occured, to service tunnel mode. */
	ret = wait_event_interruptible_timeout(mn63->waitq, IS_IRQ_SET(mn63), mn63->timeout);
	if (0 == ret) {
		if ((mn63->gpio->GPIOxPAD & MN63Y1210_IRQ_MASK) == 0) {
			goto nex1;
		} else {
			//rw_struc.status = MN_RWSTRUC_ERROR_TIMEOUT;
			mn63y1210_reset(mn63);

			mn63->tunnel_status = MN_TUNNEL_FAIL | 0x08;
			goto work_done;
		}
	} else
	if (-ERESTARTSYS == ret) {
		// interrupted by signal
		//rw_struc.status = MN_RWSTRUC_ERROR_SIGNAL;
		mn63y1210_reset(mn63);

		mn63->tunnel_status = MN_TUNNEL_FAIL | 0x09;
		goto work_done;
	}
nex1:
	/* Now we must read at least first 4 byte to identify read/write.
	   if write, then read furthermore to complete the task. */
	memset(txbuf, 0xFF, 256);
	xfer.len = 4;
	spi_sync_transfer(mn63->spi, &xfer, 1);
	if (RFID_HOST_WRITE_CMD == rxbuf[0]) { // QUERY Response (RFID to Host)
		// do read more...
		xfer.len = rxbuf[3] + 1;
		xfer.rx_buf = &rxbuf[4];
		xfer.tx_buf = &txbuf[4];
		spi_sync_transfer(mn63->spi, &xfer, 1);
#define DEBUG_TUNNEL_WRITE_DUMP
#if defined(DEBUG_TUNNEL_WRITE_DUMP)					
		//printk("r4, r5, r6, r7 = 0x%x 0x%x 0x%x 0x%x\n", rxbuf[4],rxbuf[5],rxbuf[6],rxbuf[7]);
		//printk("r8, r9, rA, rB = 0x%x 0x%x 0x%x 0x%x\n", rxbuf[8],rxbuf[9],rxbuf[10],rxbuf[11]);
		{
			int i;

			printk("QUERY Response WRITE COMMAND\n");
			for (i=0; i < xfer.len+4; i++)
			{
				if ((i % 16) == 0)
					printk("\n");
				printk("0x%02X, ", rxbuf[i]);
			}
		}
#endif
		// Now Check Checksum as this is good or not good parcket.
		if (checksum(xfer.len+4-1, &rxbuf[0]) != rxbuf[xfer.len+4-1]) {
			UNMASK_IRQ(mn63);

			// Checksum Bad. Respond with 0xE8
			len = build_status_cmd(txbuf, RFID_TUNNEL_RESPOND_ERROR);
			memset(&xfer, 0, sizeof(xfer));
			xfer.len = len;
			xfer.rx_buf = rxbuf;
			xfer.tx_buf = txbuf;
			spi_sync_transfer(mn63->spi, &xfer, 1);

			mn63->tunnel_status = MN_TUNNEL_FAIL | 0x0E;
		} else {
			// Checksum OK, now reserve this data to mn63's internal buffer.
			memcpy(mn63->int_data_buffer, rxbuf, xfer.len+4);
			mn63->int_data_len = xfer.len + 4;
			
			// Do write TUNNEL Ok and copy data to userspace.
			UNMASK_IRQ(mn63);

			len = build_status_cmd(txbuf, RFID_TUNNEL_RESPOND_NORMAL);
			memset(&xfer, 0, sizeof(xfer));
			xfer.len = len;
			xfer.rx_buf = rxbuf;
			xfer.tx_buf = txbuf;
			spi_sync_transfer(mn63->spi, &xfer, 1);
		}
		/* Wait IRQ */
		ret = wait_event_interruptible_timeout(mn63->waitq, IS_IRQ_SET(mn63), mn63->timeout);
		if (0 == ret) {
			if ((mn63->gpio->GPIOxPAD & MN63Y1210_IRQ_MASK) == 0) {
				goto nex2;
			} else {
				//tunnel_rw_struc.dirty = 1;
				//tunnel_rw_struc.status = MN_RWSTRUC_ERROR_TIMEOUT;
				mn63y1210_reset(mn63);

				mn63->tunnel_status = MN_TUNNEL_FAIL | 0x0C;
				goto work_done;
			}
		} else
		if (-ERESTARTSYS == ret) {
			//tunnel_rw_struc.dirty = 1;
			//tunnel_rw_struc.status = MN_RWSTRUC_ERROR_SIGNAL;
			mn63y1210_reset(mn63);

			mn63->tunnel_status = MN_TUNNEL_FAIL | 0x0D;

			goto work_done;
		}
nex2:
		/* Read respond from NFC */
		memset(txbuf, 0xFF, 256);
		xfer.len = 2;
		spi_sync_transfer(mn63->spi, &xfer, 1);
#if defined(DEBUG_TUNNEL_WRITE_DUMP)
		//printk("r4, r5, r6, r7 = 0x%x 0x%x 0x%x 0x%x\n", rxbuf[4],rxbuf[5],rxbuf[6],rxbuf[7]);
		//printk("r8, r9, rA, rB = 0x%x 0x%x 0x%x 0x%x\n", rxbuf[8],rxbuf[9],rxbuf[10],rxbuf[11]);
		{
			int i;

			printk("ANSWER Response\n");
			for (i=0; i < xfer.len; i++)
			{
				if ((i % 16) == 0)
					printk("\n");
				printk("0x%02X, ", rxbuf[i]);
			}
		}
#endif
		if (0x05 == rxbuf[0]) {
			mn63->tunnel_status = MN_TUNNEL_RFID_WRITE;
		} else {
			mn63y1210_reset(mn63);
			mn63->tunnel_status = MN_TUNNEL_FAIL | 0x0F;
			// Tunnel mode error. do somthing?
		}
	} else if (RFID_HOST_READ_CMD == rxbuf[0]) {
		// Then read request.
		mn63->tunnel_status = MN_TUNNEL_RFID_READ | (((unsigned int)rxbuf[3]) << 16) | 
			(((unsigned int)rxbuf[2]) << 8) | rxbuf[1];
		/* We must do service inside the work function to correct behavior. */
		/* Now just unlock tunnel mode lock to begin next stage. */
		/* Do tunnel mode unlock to restart action. */

		// Tunnel mode error. do somthing?
		mn63y1210_reset(mn63);
#if defined(DEBUG_TUNNEL_WRITE_DUMP)
		//printk("r4, r5, r6, r7 = 0x%x 0x%x 0x%x 0x%x\n", rxbuf[4],rxbuf[5],rxbuf[6],rxbuf[7]);
		//printk("r8, r9, rA, rB = 0x%x 0x%x 0x%x 0x%x\n", rxbuf[8],rxbuf[9],rxbuf[10],rxbuf[11]);
		{
			int i;

			printk("QUERY Response READ COMMAND\n");
			for (i=0; i < xfer.len; i++)
			{
				if ((i % 16) == 0)
					printk("\n");
				printk("0x%02X, ", rxbuf[i]);
			}
		}
#endif
	} else {
		// Unknown command or error.
		mn63->tunnel_status = MN_TUNNEL_RFID_UNKNOWN | rxbuf[0];

		// Tunnel mode error. do somthing?
		mn63y1210_reset(mn63);
#if defined(DEBUG_TUNNEL_WRITE_DUMP)
		//printk("r4, r5, r6, r7 = 0x%x 0x%x 0x%x 0x%x\n", rxbuf[4],rxbuf[5],rxbuf[6],rxbuf[7]);
		//printk("r8, r9, rA, rB = 0x%x 0x%x 0x%x 0x%x\n", rxbuf[8],rxbuf[9],rxbuf[10],rxbuf[11]);
		{
			int i;

			printk("QUERY Response UNKNOWND COMMAND\n");
			for (i=0; i < xfer.len; i++)
			{
				if ((i % 16) == 0)
					printk("\n");
				printk("0x%02X, ", rxbuf[i]);
			}
		}
#endif
	}
work_done:
	UNMASK_STATUS(mn63);

	/* Unlock mutexes */
	mutex_unlock(&mn63->rw_lock);
	mutex_unlock(&mn63->tunnel_lock);

	SET_TUNNEL_IRQ(mn63y1210);
	wake_up_interruptible(&mn63y1210->tunnel_waitq);
}

/*------------------------------------------------------------------------------- 
	IOCTL service routine.
--------------------------------------------------------------------------------*/
static long mn63y1210_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
	int ret   = 0;
	MN_readwrite_struc_t rw_struc;
	MN_tunnel_readwrite_struc_t tunnel_rw_struc;
	mn63y1210_device_t *mn63 = filp->private_data;
	struct spi_transfer xfer;
	char txbuf[256], rxbuf[256], cksum;
	int len;

#define RWSTRUC_HEADER_SIZE	((unsigned int)&rw_struc.buffer[0] - (unsigned int)&rw_struc.addr)
#define TUNNEL_RW_HEADER_SIZE ((unsigned int)&tunnel_rw_struc.buffer[0] - (unsigned int)&tunnel_rw_struc.dirty)

	DBGOUT("%s (minor:%d, table:%d, cmd:0x%x, nr:%d)\n",
		__func__, minor, NUM_OF_PARTS, cmd, _IOC_NR(cmd));

	switch(cmd)	{
		case IOCTL_MN63Y1210_STATUS:
			{
				if (copy_to_user((void*)arg, (const void*)&mn63->cur_status, sizeof(int)))
					break;
			}
			break;

		case IOCTL_MN63Y1210_RESET:
			{
				ret = -10;
			}
			break;

		case IOCTL_MN63Y1210_POWER:
			{
				int power;

				if (copy_from_user((void*)&power, (const void*)arg, sizeof(int)))
					break;

				mn63y1210_power(mn63, power);
			}
			break;

		case IOCTL_MN63Y1210_READ:
			{

				if (copy_from_user((void*)&rw_struc, (const void*)arg, RWSTRUC_HEADER_SIZE)) // read addr and size
					break;
				
				/* do read from rfid */
				mutex_lock(&mn63->rw_lock);
				UNMASK_IRQ(mn63);
				SET_READWRITE(mn63);
				len = build_read_cmd(txbuf, RFID_HOST_READ_CMD, rw_struc.addr, rw_struc.size);
#if DEBUG_DUMP
				printk("dump tx\n");
				dump(rw_struc.addr, len, txbuf);
#endif
				memset(&xfer, 0, sizeof(xfer));
				xfer.rx_buf = rxbuf;
				xfer.tx_buf = txbuf;
				xfer.len = len;
				spi_sync_transfer(mn63->spi, &xfer, 1);

				/* wait till irq occured. */
				ret = wait_event_interruptible_timeout(mn63->waitq, IS_IRQ_SET(mn63), mn63->timeout);
				if (0 == ret) {
					UNMASK_STATUS(mn63);
					mutex_unlock(&mn63->rw_lock);
					rw_struc.status = MN_RWSTRUC_ERROR_TIMEOUT;
					mn63->cur_status = MN_STATUS_RW_FAIL;
					ret = copy_to_user((void*)arg, (const void*)&rw_struc, RWSTRUC_HEADER_SIZE);
					return ret;
				} else
				if (-ERESTARTSYS == ret) {
					// interrupted by signal
					UNMASK_STATUS(mn63);
					mutex_unlock(&mn63->rw_lock);
					rw_struc.status = MN_RWSTRUC_ERROR_SIGNAL;
					mn63->cur_status = MN_STATUS_RW_FAIL;
					ret = copy_to_user((void*)arg, (const void*)&rw_struc, RWSTRUC_HEADER_SIZE);
					return -ERESTARTSYS;
				}
				// Now IRQ occured. read data.
				len = rw_struc.size + 2;
				memset(txbuf, 0xFF, 256);
				xfer.len = len;
				spi_sync_transfer(mn63->spi, &xfer, 1);

				UNMASK_IRQ(mn63);
				UNMASK_STATUS(mn63);
				mutex_unlock(&mn63->rw_lock);

#if DEBUG_DUMP
				printk("dump rx\n");
				dump(rw_struc.addr, len, txbuf);
				dump(rw_struc.addr, len, rxbuf);
#endif

				cksum = checksum(rw_struc.size + 1, rxbuf);
				if ((RFID_RESPOND_NORMAL_END == rxbuf[0]) && (rxbuf[rw_struc.size+1] == cksum)) {
					int i;

					for (i=0; i<rw_struc.size; i++) {
						rw_struc.buffer[i] = rxbuf[i+1];
					}
					rw_struc.status = 0;
					mn63->cur_status = 0;
					len = rw_struc.size;
				} else {
					rw_struc.status = MN_RWSTRUC_ERROR_CRC;
					mn63->cur_status = MN_STATUS_RW_FAIL;
					len = 0;
				}

				if (copy_to_user((void*)arg, (const void*)&rw_struc, RWSTRUC_HEADER_SIZE + len))
					break;

			}
			break;

		case IOCTL_MN63Y1210_WRITE:
			{
				if (copy_from_user((void*)&rw_struc, (const void*)arg, sizeof(MN_readwrite_struc_t)))
					break;

				/* do write to rfid */
				mutex_lock(&mn63->rw_lock);
				UNMASK_IRQ(mn63);
				SET_READWRITE(mn63);
				len = build_write_cmd(txbuf, RFID_HOST_WRITE_CMD, rw_struc.addr, rw_struc.size, rw_struc.buffer);
				memset(&xfer, 0, sizeof(xfer));
				xfer.rx_buf = rxbuf;
				xfer.tx_buf = txbuf;
				xfer.len = len;
				spi_sync_transfer(mn63->spi, &xfer, 1);
				/* wait till irq occured */
				ret = wait_event_interruptible_timeout(mn63->waitq, IS_IRQ_SET(mn63), mn63->timeout);
				if (0 == ret) {
					UNMASK_STATUS(mn63);
					mutex_unlock(&mn63->rw_lock);
					rw_struc.status = MN_RWSTRUC_ERROR_TIMEOUT;
					mn63->cur_status = MN_STATUS_RW_FAIL;
					ret = copy_to_user((void*)arg, (const void*)&rw_struc, RWSTRUC_HEADER_SIZE);
					return ret;
				} else
				if (-ERESTARTSYS == ret) {
					// interrupted by signal
					UNMASK_STATUS(mn63);
					mutex_unlock(&mn63->rw_lock);
					rw_struc.status = MN_RWSTRUC_ERROR_SIGNAL;
					mn63->cur_status = MN_STATUS_RW_FAIL;
					ret = copy_to_user((void*)arg, (const void*)&rw_struc, RWSTRUC_HEADER_SIZE);
					return -ERESTARTSYS;
				}
				// Now IRQ occured. read status.
				memset(txbuf, 0xFF, 256);
				xfer.len = 2;
				spi_sync_transfer(mn63->spi, &xfer, 1);
				
				UNMASK_STATUS(mn63);
				mutex_unlock(&mn63->rw_lock);
#if DEBUG_DUMP
				printk("dump rx\n");
				dump(rw_struc.addr, 2, txbuf);
				dump(rw_struc.addr, 2, rxbuf);
#endif
				if (RFID_RESPOND_NORMAL_END == rxbuf[0]) {
					if (RFID_RESPOND_NORMAL_CRC == rxbuf[1]) {
						rw_struc.status = 0;
						mn63->cur_status = 0;
					} else {
						rw_struc.status = MN_RWSTRUC_ERROR_CRC;
						mn63->cur_status = MN_STATUS_RW_FAIL;
					}
				} else {
					rw_struc.status = MN_RWSTRUC_ERROR_WRITE;
					mn63->cur_status = MN_STATUS_RW_FAIL;
				}
				if (copy_to_user((void*)arg, (const void*)&rw_struc, RWSTRUC_HEADER_SIZE)) {		
					return -3;
				}
			}
			break;

		case IOCTL_MN63Y1210_TUNNEL_STATUS:
			{
				/* Now work thread do work almost everything. */
				/* First, user should get it be known. */
				/* Then unlock status */
				ret = wait_event_interruptible(mn63->tunnel_waitq, IS_TUNNEL_IRQ_SET(mn63));
				if (-ERESTARTSYS == ret) {
					UNMASK_IRQ(mn63);
					mn63->tunnel_status = MN_TUNNEL_FAIL | 0x01; /* retry ? */
					ret = copy_to_user((void*)arg, (const void*)&mn63->tunnel_status, sizeof(int));
					return -ERESTARTSYS;
				} else if (0 > ret) {
					UNMASK_IRQ(mn63);
					mn63->tunnel_status = MN_TUNNEL_FAIL | 0x02; /* error ? */
					ret = copy_to_user((void*)arg, (const void*)&mn63->tunnel_status, sizeof(int));
					return -1 - ret;
				}
				UNMASK_IRQ(mn63);

				ret = copy_to_user((void*)arg, (const void*)&mn63->tunnel_status, sizeof(int));
			}
			break;

		case IOCTL_MN63Y1210_TUNNEL_READ:
			{
				if (copy_from_user((void*)&tunnel_rw_struc, (const void*)arg, sizeof(MN_tunnel_readwrite_struc_t)))
					break;

				/* Now write data to NFC */
				UNMASK_IRQ(mn63);
				SET_READWRITE(mn63);
				mutex_lock(&mn63->rw_lock);
				len = build_data_cmd(txbuf, RFID_TUNNEL_RESPOND_NORMAL, tunnel_rw_struc.size, tunnel_rw_struc.buffer);
				memset(&xfer, 0, sizeof(xfer));
				xfer.len = len;
				xfer.rx_buf = rxbuf;
				xfer.tx_buf = txbuf;
				spi_sync_transfer(mn63->spi, &xfer, 1);
				/* Wait IRQ */
				ret = wait_event_interruptible_timeout(mn63->waitq, IS_IRQ_SET(mn63), mn63->timeout);
				if (0 == ret) {
					UNMASK_STATUS(mn63);
					mutex_unlock(&mn63->rw_lock);
					mutex_unlock(&mn63->tunnel_lock);
					tunnel_rw_struc.dirty = 1;
					tunnel_rw_struc.status = MN_RWSTRUC_ERROR_TIMEOUT;
					mn63->tunnel_status = MN_TUNNEL_FAIL | 0x0A;
					ret = copy_to_user((void*)arg, (const void*)&tunnel_rw_struc, TUNNEL_RW_HEADER_SIZE);
					return -2 - ret;
				} else
				if (-ERESTARTSYS == ret) {
					// interrupted by signal
					UNMASK_STATUS(mn63);
					mutex_unlock(&mn63->rw_lock);
					mutex_unlock(&mn63->tunnel_lock);
					tunnel_rw_struc.dirty = 1;
					tunnel_rw_struc.status = MN_RWSTRUC_ERROR_SIGNAL;
					mn63->tunnel_status = MN_TUNNEL_FAIL | 0x0B;
					ret = copy_to_user((void*)arg, (const void*)&tunnel_rw_struc, TUNNEL_RW_HEADER_SIZE);
					return -2 - ret;
				}
				UNMASK_STATUS(mn63);
				/* Read respond from NFC */
				memset(txbuf, 0xFF, 256);
				xfer.len = 2;
				spi_sync_transfer(mn63->spi, &xfer, 1);
				mn63->tunnel_status = 0;
				/* Do we need to checl response ? */
				/* Unlock mutexes */
				mutex_unlock(&mn63->rw_lock);
				mutex_unlock(&mn63->tunnel_lock);
				ret = -10;
			}
			break;

		case IOCTL_MN63Y1210_TUNNEL_WRITE:
			{
#if 0				
				// Do write TUNNEL Ok and copy data to userspace.
				UNMASK_IRQ(mn63);
				SET_READWRITE(mn63);
				mutex_lock(&mn63->rw_lock);
				len = build_status_cmd(txbuf, RFID_TUNNEL_RESPOND_NORMAL);
				memset(&xfer, 0, sizeof(xfer));
				xfer.len = len;
				xfer.rx_buf = rxbuf;
				xfer.tx_buf = txbuf;
				spi_sync_transfer(mn63->spi, &xfer, 1);
				/* Wait IRQ */
				ret = wait_event_interruptible_timeout(mn63->waitq, IS_IRQ_SET(mn63), mn63->timeout);
				if (0 == ret) {
					UNMASK_STATUS(mn63);
					mutex_unlock(&mn63->rw_lock);
					mutex_unlock(&mn63->tunnel_lock);
					tunnel_rw_struc.dirty = 1;
					tunnel_rw_struc.status = MN_RWSTRUC_ERROR_TIMEOUT;
					mn63->tunnel_status = MN_TUNNEL_FAIL | 0x0C;
					ret = copy_to_user((void*)arg, (const void*)&tunnel_rw_struc, TUNNEL_RW_HEADER_SIZE);
					return -2 - ret;
				} else
				if (-ERESTARTSYS == ret) {
					// interrupted by signal
					UNMASK_STATUS(mn63);
					mutex_unlock(&mn63->rw_lock);
					mutex_unlock(&mn63->tunnel_lock);
					tunnel_rw_struc.dirty = 1;
					tunnel_rw_struc.status = MN_RWSTRUC_ERROR_SIGNAL;
					mn63->tunnel_status = MN_TUNNEL_FAIL | 0x0D;
					ret = copy_to_user((void*)arg, (const void*)&tunnel_rw_struc, TUNNEL_RW_HEADER_SIZE);
					return -2 - ret;
				}
				UNMASK_STATUS(mn63);
				/* Read respond from NFC */
				memset(txbuf, 0xFF, 256);
				xfer.len = 2;
				spi_sync_transfer(mn63->spi, &xfer, 1);
				if (0x05 == rxbuf[0]) {
					ret = 0;
				} else {
					ret = -1;
					tunnel_rw_struc.status = MN_RWSTRUC_ERROR_UNKNOWN;
				}
				/* Unlock mutexes */
				mutex_unlock(&mn63->rw_lock);
#endif				
				//mutex_unlock(&mn63->tunnel_lock);
				/* Copy write data to user */
				if (mn63->int_data_len > 0) {
					tunnel_rw_struc.dirty = 1;
					tunnel_rw_struc.addr = (((unsigned int)mn63->int_data_buffer[2]) << 8) | mn63->int_data_buffer[1];
					len = mn63->int_data_buffer[3];
					tunnel_rw_struc.size = len;
					tunnel_rw_struc.status = 0;
					mn63->tunnel_status = 0;
					mn63->int_data_len = 0;
					memcpy(tunnel_rw_struc.buffer, &mn63->int_data_buffer[4], len);
				} else {
					tunnel_rw_struc.dirty = 0;
					tunnel_rw_struc.size = 0;
					tunnel_rw_struc.status = MN_RWSTRUC_ERROR_WRITE;
				}
				if (copy_to_user((void*)arg, (const void*)&tunnel_rw_struc, TUNNEL_RW_HEADER_SIZE + len))
					break;
			}
			break;

		case IOCTL_MN63Y1210_TUNNEL_ABORT:
			mn63->tunnel_status = MN_TUNNEL_FAIL | 0x0A; /* abort ? */

			SET_TUNNEL_IRQ(mn63y1210);
			wake_up_interruptible(&mn63y1210->tunnel_waitq);
			break;

		default:
			DBGOUT("Fail, unknown ioctl ...\n");
			return -1;
	}
	DBGOUT("IoCtl (cmd:0x%x, nr:%d, ret:%d) \n\n", cmd, _IOC_NR(cmd), ret);

	return ret;
}


struct file_operations mn63y1210_fops = {
	.owner 			= THIS_MODULE,
	.open 			= mn63y1210_open,
	.release		= mn63y1210_release,
	//.read			= mn63y1219_read,
	//.write			= mn63y1219_write,
	.unlocked_ioctl	= mn63y1210_ioctl,
};

static irqreturn_t mn63y1210_irq(int irq, void *dev_id)
{
	mn63y1210_device_t *mn63y1210 = (mn63y1210_device_t *) dev_id;

	if (mn63y1210->power_dn)
		return IRQ_HANDLED;
#if DEBUG_DUMP
	printk("irq gen\n");
#endif
	if (IS_READWRITE_SET(mn63y1210)) {
		SET_IRQ(mn63y1210);
		wake_up_interruptible(&mn63y1210->waitq);
	} else {
		queue_work(mn63y1210->tunnel_workq, &mn63y1210->work);
	}

	return IRQ_HANDLED;
}

static int __devinit mn63y1210_probe(struct spi_device *spi)
{
	int ret;

	spi->chip_select = 0;
	spi->max_speed_hz = 0;
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_3 | SPI_NO_CS;
	ret = spi_setup(spi);
	if (ret < 0)
		return ret;

	mn63y1210_init_hw();

	mn63y1210 = kzalloc(sizeof(mn63y1210_device_t), GFP_KERNEL);
	if (mn63y1210 == NULL)
		return -ENOMEM;
	
	mn63y1210->spi = spi;
	dev_set_drvdata(&spi->dev, mn63y1210);
	// should be from MACH...
	mn63y1210->irq = MN63Y1210_IRQ;
	mn63y1210->timeout = 10; /* 100ms in jiffies */
	mn63y1210->power_dn = 0;

	/* register character driver */
	ret = register_chrdev(MN63Y1210_DEV_MAJOR, "nfc/rfid reader/writer", &mn63y1210_fops);
	if (0 > ret)	{
		DBGOUT("Fail, register device (%s, major:%d)\n",
			MN63Y1210_DRV_NAME, MN63Y1210_DEV_MAJOR);
		goto err_alloc;
	}

	ret = sysfs_create_group(&spi->dev.kobj, &mn63y1210_attr_group);
	if (ret < 0)
		goto err_register;
	
	mn63y1210_class = class_create(THIS_MODULE, MN63Y1210_DRV_NAME);
	device_create(mn63y1210_class, NULL, MKDEV(MN63Y1210_DEV_MAJOR, 0), NULL, MN63Y1210_DRV_NAME);
	
	mutex_init(&mn63y1210->rw_lock);
	init_waitqueue_head(&mn63y1210->waitq);
	mutex_init(&mn63y1210->tunnel_lock);
	init_waitqueue_head(&mn63y1210->tunnel_waitq);
			
	// Now we should install irq for done/request.
	if (request_irq(mn63y1210->irq , mn63y1210_irq, IRQF_DISABLED | IRQF_TRIGGER_FALLING, MN63Y1210_DRV_NAME, mn63y1210)) {
		DBGOUT("mn63y1210 request_irq failed\n");
		goto err_sysfs;
	}
	
	/* create workq */
	mn63y1210->tunnel_workq = create_singlethread_workqueue("tunnel_wq");
	if (!mn63y1210->tunnel_workq) {
		ret = -ENOMEM;
		goto err_sysfs;
	}
	INIT_WORK(&mn63y1210->work, tunnel_work_func);
	
#if defined(CONFIG_MODEL_RK_HIT_N)
	mn63y1210->gpio = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO);
#else
	mn63y1210->gpio = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO+OFFSET_OF_GPIO_MODULE);
#endif
	printk("mn63y1210_spi device is ready.\n");

	return 0;
err_sysfs:
	sysfs_remove_group(&spi->dev.kobj, &mn63y1210_attr_group);

err_register:
	unregister_chrdev(MN63Y1210_DEV_MAJOR, "nfc/rfid reader/writer");

err_alloc:
	kfree(mn63y1210);

	return ret;
}

static int __devexit mn63y1210_remove(struct spi_device *spi)
{
	mn63y1210_device_t *mn63 = dev_get_drvdata(&spi->dev);

	free_irq(mn63->irq, mn63);
	
	/* cleanup workq */
	destroy_workqueue(mn63->tunnel_workq);
	
	unregister_chrdev(MN63Y1210_DEV_MAJOR, "nfc/rfid reader/writer");
	sysfs_remove_group(&spi->dev.kobj, &mn63y1210_attr_group);
	
	kfree(mn63);

	return 0;
}

static int mn63y1210_suspend(struct spi_device *spi)
{
	mn63y1210_device_t *mn63 = dev_get_drvdata(&spi->dev);

	printk("mn63y1210_spi device suspend. power_dn=%d\n", mn63->power_dn);

	return 0;
}
static int mn63y1210_resume(struct spi_device *spi)
{
	mn63y1210_device_t *mn63 = dev_get_drvdata(&spi->dev);

	PM_DBGOUT("%s power_dn=%p\n", __func__, mn63->power_dn);

	mn63y1210_init_hw();

#if defined(CONFIG_MODEL_RK_HIT_N)
	gpioa->GPIOxALTFN[0] &= 0xFFFFFF3F; /* GPIOA 3 to GPIO */
	gpioa->GPIOxOUTENB   &= 0xFFFFFFF7; // GPIOA 3 to INPUT */
#endif

	return 0;
}
static struct spi_driver mn63y1210_driver = {
	.driver = {
		.name	= MN63Y1210_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= mn63y1210_probe,
	.remove	= __devexit_p(mn63y1210_remove),
	.suspend = mn63y1210_suspend,
	.resume = mn63y1210_resume,
};

static int __init mn63y1210_spi_init(void)
{
	return spi_register_driver(&mn63y1210_driver);
}

static void __exit mn63y1210_spi_exit(void)
{
	spi_unregister_driver(&mn63y1210_driver);
}

MODULE_AUTHOR("Seungwoo Kim <ksw@stcube.com>");
MODULE_DESCRIPTION("MN63Y1210 RFID driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(mn63y1210_spi_init);
module_exit(mn63y1210_spi_exit);

