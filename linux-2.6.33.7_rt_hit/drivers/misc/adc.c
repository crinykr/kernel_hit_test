/*
 * (C) Copyright 2010
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <linux/hrtimer.h>

/* nexell soc headers */
#include <mach/platform.h>
#include <mach/devices.h>
#include <mach/soc.h>
#include <mach/adc.h>

#if (0)
#define DBGOUT(msg...)		{ printk(KERN_INFO "adc: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define PERIODIC_CONVERSION 0

typedef struct {
	unsigned short level[4];
} adc_device_t;

adc_device_t	*adc_level;

static struct class *adc_class;

#if PERIODIC_CONVERSION
/* hrtimer routines for DMA/HRTIMER */
static int timer_init = 0;
static enum hrtimer_restart hrtimer_action(struct hrtimer *);
static struct hrtimer hrtimer;
static ktime_t time;

static enum hrtimer_restart hrtimer_action(struct hrtimer *timer)
{
	unsigned short	level;
	int				ch;

	for (ch = 0 ; ch < 4 ; ch++)
	{
		level = soc_adc_read(ch, 100*1000);
	}

	hrtimer_start(timer, time, HRTIMER_MODE_REL);

    return HRTIMER_NORESTART;
}
#endif

/*------------------------------------------------------------------------------
 * 	ADC ops
 */
static int nx_adc_ops_open(struct inode *inode, struct file *flip)
{
	DBGOUT("%s\n", __func__);
	soc_adc_attach();
	return 0;
}

static int nx_adc_ops_release(struct inode *inode, struct file *flip)
{
	DBGOUT("%s\n", __func__);
	soc_adc_detach();
	return 0;
}

static int nx_adc_ops_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	DBGOUT("%s(cmd:0x%x)\n", __func__, cmd);

	switch (cmd)	{
	case IOCTL_ADC_GET_LEVEL:
		{
			struct adc_info adc;

			if (copy_from_user(&adc, (const void __user *)arg, sizeof(struct adc_info)))
				return -EFAULT;

			if (adc.ch >= 4)
				return -EIO;
#if 1
			adc.level = soc_adc_read(adc.ch, 100*1000);
#else
			adc.level = adc_level->level[adc.ch];
#endif
			if (copy_to_user((void __user *)arg, (const void *)&adc, sizeof(struct adc_info)))
				return -EFAULT;
		}
		break;

	default:
		printk(KERN_ERR "%s: fail, unknown command 0x%x, \n",
			ADC_DEV_NAME, cmd);
		return -EINVAL;
	}

	DBGOUT("IoCtl (cmd:0x%x) \n\n", cmd);
	return 0;
}

struct file_operations nx_adc_ops = {
	.owner 	= THIS_MODULE,
	.open 	= nx_adc_ops_open,
	.release= nx_adc_ops_release,
	.ioctl 	= nx_adc_ops_ioctl,
};

/*--------------------------------------------------------------------------------
 * ADC platform_driver functions
 ---------------------------------------------------------------------------------*/
static int __init nx_adc_driver_probe(struct platform_device *pdev)
{
	int ret, i;
	adc_device_t *adc;

	DBGOUT("enter %s\n", __func__);
	adc = kzalloc(sizeof(adc_device_t), GFP_KERNEL);
	if (adc == NULL)
		return -ENOMEM;

	for (i = 0 ; i < 4 ; i++)
		adc->level[i] = 0xFFFF;

	ret = register_chrdev(
					ADC_DEV_MAJOR, "ADC(Analog Digital Converter)", &nx_adc_ops
					);

	if (0 > ret) {
		printk(KERN_ERR "fail, register device (%s, major:%d)\n",
			ADC_DEV_NAME, ADC_DEV_MAJOR);
		return ret;
	}
	adc_class = class_create(THIS_MODULE, ADC_DEV_NAME);
	device_create(adc_class, NULL, MKDEV(ADC_DEV_MAJOR, 0), NULL, ADC_DEV_NAME);

#if PERIODIC_CONVERSION
	if (!timer_init) {
	    time = ktime_set(0, 10000000); /* 10ms */

	    hrtimer_init(&hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	    hrtimer.function = &hrtimer_action;
	    hrtimer_start(&hrtimer, time, HRTIMER_MODE_REL);
	    timer_init = 1;
	}
#endif
	adc_level = adc;
	printk(KERN_INFO "%s: register major:%d\n", pdev->name, ADC_DEV_MAJOR);
	return 0;
}

static int nx_adc_driver_remove(struct platform_device *pdev)
{
	DBGOUT("%s\n", __func__);
	unregister_chrdev(ADC_DEV_MAJOR, "ADC(Analog Digital Converter)");
	return 0;
}

static int nx_adc_driver_suspend(struct platform_device *dev, pm_message_t state)
{
	PM_DBGOUT("%s\n", __func__);
	soc_adc_suspend();
	return 0;
}

static int nx_adc_driver_resume(struct platform_device *dev)
{
	PM_DBGOUT("%s\n", __func__);
	soc_adc_resume();
	return 0;
}

static struct platform_driver adc_plat_driver = {
	.probe		= nx_adc_driver_probe,
	.remove		= nx_adc_driver_remove,
	.suspend	= nx_adc_driver_suspend,
	.resume		= nx_adc_driver_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= ADC_DEV_NAME,
	},
};

static int __init nx_adc_driver_init(void)
{
	DBGOUT("%s\n", __func__);
	return platform_driver_register(&adc_plat_driver);
}

static void __exit nx_adc_driver_exit(void)
{
	DBGOUT("%s\n", __func__);
	platform_driver_unregister(&adc_plat_driver);
}

module_init(nx_adc_driver_init);
module_exit(nx_adc_driver_exit);

MODULE_AUTHOR("jhkim <jhkin@nexell.co.kr>");
MODULE_DESCRIPTION("ADC (Analog Digital Converter) driver for the Nexell");
MODULE_LICENSE("GPL");

