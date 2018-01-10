/*
 *  gpio_enc_test.c - Linux kernel module for
 * 	GPIO encoder driver
 *
 *  Copyright (c) 2015 STcube Inc.,
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

#include "gpio_enc.h"

#if (1)
#define DBGOUT(msg...)	{ printk(KERN_INFO "gpio_enc: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define DEBUG_DUMP	0

#define GPIO_ENC_DRV_NAME	"gpio_enc"
#define DRIVER_VERSION		"0.5"


#define GPIO_ENC_IRQ		(IRQ_GPIO_A_START + 17)
#define GPIO_ENC_IRQ_MASK  	(1 << 17)
#define GPIO_ENC_IRQ2		(IRQ_GPIO_A_START + 18)
#define GPIO_ENC_IRQ_MASK2  (1 << 18)

#define GPIO_ENC_DEV_MAJOR			223

#define GPIO_ENC_DIR0		(1 << 3)
#define GPIO_ENC_DIR1		(1 << 19)

typedef struct {
	struct platform_device	*pdev;
	u32			mask;
	int			irq;
	int			irq2;
	void __iomem		*base;
	struct clk		*clk;
	int			count;
	int 		count2;
	//void			(*handler[PWM_NCHAN])(struct pwm_channel *);
} gpio_enc_device_t;

gpio_enc_device_t *gpio_enc;

static struct class *gpio_enc_class;

static struct NX_GPIO_RegisterSet *gpio_base;
static struct NX_PWM_RegisterSet *pwm_base;

static ssize_t gpio_enc_store_val(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val) < 0)
		return -EINVAL;

	return count;
}

static DEVICE_ATTR(value, S_IWUSR, NULL, gpio_enc_store_val);

static struct attribute *gpio_enc_attributes[] = {
	&dev_attr_value.attr,
	NULL
};

static const struct attribute_group gpio_enc_attr_group = {
	.attrs = gpio_enc_attributes,
};

/*--------------------------------------------------------------------------------
 * file_operations
 ---------------------------------------------------------------------------------*/
static int gpio_enc_open(struct inode *inode, struct file *flip)
{
	DBGOUT("%s\n", __func__);
	
	flip->private_data = gpio_enc;
	gpio_enc->count = 0;
	gpio_enc->count2 = 0;

	return 0;
}

static int gpio_enc_release(struct inode *inode, struct file *flip)
{
	DBGOUT("%s\n", __func__);

	return 0;
}

/*------------------------------------------------------------------------------- 
	IOCTL service routine.
--------------------------------------------------------------------------------*/
static long gpio_enc_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
	int ret   = 0;

	DBGOUT("%s (cmd:0x%x, nr:%d)\n",
		__func__, cmd, _IOC_NR(cmd));

	switch(cmd)	{
		case IOCTL_GPIO_ENC_STATUS:
			{
				if (copy_to_user((void*)arg, (const void*)&gpio_enc->count, sizeof(int)*2))
					break;
			}
			break;

		case IOCTL_GPIO_ENC_RESET:
			{
				gpio_enc->count = 0;
				gpio_enc->count2 = 0;
			}
			break;

		default:
			DBGOUT("Fail, unknown ioctl ...\n");
			return -1;
	}
	DBGOUT("IoCtl (cmd:0x%x, nr:%d, ret:%d) \n\n", cmd, _IOC_NR(cmd), ret);

	return ret;
}


struct file_operations gpio_enc_fops = {
	.owner 			= THIS_MODULE,
	.open 			= gpio_enc_open,
	.release		= gpio_enc_release,
	.unlocked_ioctl	= gpio_enc_ioctl,
};

static irqreturn_t gpio_enc_irq(int irq, void *dev_id)
{
	gpio_enc_device_t *enc = (gpio_enc_device_t *) dev_id;


#if DEBUG_DUMP
	printk("irq gen\n");
#endif
	if (gpio_base->GPIOxPAD & GPIO_ENC_DIR0)
		enc->count++;
	else
		enc->count--;

	return IRQ_HANDLED;
}

static irqreturn_t gpio_enc_irq2(int irq, void *dev_id)
{
	gpio_enc_device_t *enc = (gpio_enc_device_t *) dev_id;


#if DEBUG_DUMP
	printk("irq gen\n");
#endif
	if (gpio_base->GPIOxPAD & GPIO_ENC_DIR1)
		enc->count2++;
	else
		enc->count2--;

	return IRQ_HANDLED;
}

static int __init gpio_enc_init(void)
{
	int ret;
	gpio_enc_device_t *enc;
	//struct resource *r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	//int irq = platform_get_irq(pdev, 0);

	gpio_base = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO);
	pwm_base  = (struct NX_PWM_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_PWM);

	DBGOUT("enter %s\n", __func__);
	enc = kzalloc(sizeof(gpio_enc_device_t), GFP_KERNEL);
	if (enc == NULL)
		return -ENOMEM;
	
	// should be from MACH...
	enc->irq = GPIO_ENC_IRQ; // or irq (above)
	enc->irq2 = GPIO_ENC_IRQ2; // or irq (above)
	//enc->timeout = 10; /* 100ms in jiffies */
	
	/* register character driver */
	ret = register_chrdev(GPIO_ENC_DEV_MAJOR, "gpio_enc", &gpio_enc_fops);
	if (0 > ret)	{
		DBGOUT("Fail, register device (%s, major:%d)\n",
			GPIO_ENC_DRV_NAME, GPIO_ENC_DEV_MAJOR);
		goto err_alloc;
	}

//	ret = sysfs_create_group(&pdev->dev.kobj, &gpio_enc_attr_group);
//	if (ret < 0)
//		goto err_register;
	
	gpio_enc_class = class_create(THIS_MODULE, GPIO_ENC_DRV_NAME);
	device_create(gpio_enc_class, NULL, MKDEV(GPIO_ENC_DEV_MAJOR, 0), NULL, GPIO_ENC_DRV_NAME);
			
	// Now we should install irq for done/request.
	if (request_irq(enc->irq , gpio_enc_irq, IRQF_DISABLED | IRQF_TRIGGER_RISING, GPIO_ENC_DRV_NAME, enc)) {
		DBGOUT("gpio enc request_irq failed\n");
		goto err_sysfs;
	}
	
	if (request_irq(enc->irq2 , gpio_enc_irq2, IRQF_DISABLED | IRQF_TRIGGER_RISING, GPIO_ENC_DRV_NAME, enc)) {
		DBGOUT("gpio enc request_irq2 failed\n");
		goto err_sysfs;
	}
	
	printk("gpio_enc device is ready.\n");
	gpio_enc = enc;

	return 0;
err_sysfs:
//	DBGOUT("unregister sysfs.\n");
//	sysfs_remove_group(&pdev->dev.kobj, &gpio_enc_attr_group);

err_register:
	DBGOUT("unregister chrdev.\n");
	unregister_chrdev(GPIO_ENC_DEV_MAJOR, "gpio enc driver");

err_alloc:
	kfree(enc);

	return ret;
}

static void __exit gpio_enc_exit(void)
{
	gpio_enc_device_t *enc = gpio_enc;

	free_irq(enc->irq, enc);
	free_irq(enc->irq2, enc);
	
	kfree(enc);
}

MODULE_AUTHOR("Seungwoo Kim <ksw@stcube.com>");
MODULE_DESCRIPTION("GPIO enc driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(gpio_enc_init);
module_exit(gpio_enc_exit);

