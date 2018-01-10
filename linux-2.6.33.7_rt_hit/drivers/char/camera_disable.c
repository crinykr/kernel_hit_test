/*
 *  CAMERA diable driver for stcube nxp2120 board.
 *  Copyright(c) 2013 STcube, inc.
 *  All right reserved. Seungwoo Kim <ksw@stcube.com>
 *  revision note :
 *           2013. 12. 20 : Ver 0.5  initially created.
 */
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/errno.h>
#include <linux/tty.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/vt_kern.h>
#include <linux/selection.h>
#include <linux/kbd_kern.h>
#include <linux/console.h>
#include <linux/device.h>
#include <linux/spinlock.h>

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/timer.h>

#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/mach-types.h>

#include <mach/platform.h>

#define GPIO_DEBUG	1

#define CAMDIS_MAJOR	20
#define CAMDIS_MINOR	0

static struct NX_GPIO_RegisterSet *base_b, *base_c;

static int
camdis_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int
camdis_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static const struct file_operations camdis_fops = {
	.open		= camdis_open,
	.release	= camdis_release,
};

static struct class *camdis_class;


static int __init camdis_init(void)
{
	unsigned int val;

	base_b = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO + 0x40);
	base_c = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO + 0x80);

	if (register_chrdev(CAMDIS_MAJOR, "camdis", &camdis_fops))
		panic("unable to get major %d for camdis device", CAMDIS_MAJOR);
	camdis_class = class_create(THIS_MODULE, "camdis");

	device_create(camdis_class, NULL, MKDEV(CAMDIS_MAJOR, CAMDIS_MINOR), NULL, "camdis");

	/* Sertup portb bit 31, 4(i2c0 scl), 5(i2c0 sda) to GPIO/output */
	base_b->GPIOxOUTENB |= 0x80000030;
	base_b->GPIOxOUT    &= 0x7FFFFFCF;

	val = base_b->GPIOxALTFN[0];
	val &= 0xFFFFF0FF;
	base_b->GPIOxALTFN[0] = val;

	val = base_b->GPIOxALTFN[1];
	val &= 0x3FFFFFFF;
	base_b->GPIOxALTFN[1] = val;
	
	/* Sertup portc bit 0~7 to GPIO/output */
	base_c->GPIOxOUTENB |= 0x000000FF;
	base_c->GPIOxOUT    &= 0xFFFFFF00;

	val = base_c->GPIOxALTFN[0];
	val &= 0xFFFF0000;
	base_c->GPIOxALTFN[0] = val;

#if GPIO_DEBUG
    printk("GPIOC_OUTENB = %x\n", base_c->GPIOxOUTENB);
    printk("GPIOC_OUT    = %x\n", base_c->GPIOxOUT);
    printk("GPIOC_ALTFN0 = %x\n", base_c->GPIOxALTFN[0]);
    printk("GPIOC_ALTFN1 = %x\n", base_c->GPIOxALTFN[1]);
#endif

	printk("CAM disable device is registered and done to setting.\n");

	return 0;
}

static void __exit camdis_cleanup(void)
{
	/* do nothing? */
}

module_init(camdis_init);
module_exit(camdis_cleanup);

MODULE_AUTHOR("Seungwoo Kim <ksw@stcube.com>");
MODULE_DESCRIPTION("nxp2120 camera port disable driver");
MODULE_LICENSE("GPL");
