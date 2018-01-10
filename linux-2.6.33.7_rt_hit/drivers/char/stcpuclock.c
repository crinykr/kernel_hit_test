/*
 *  CPUClock driver for STcube nxp2120 board.
 *  Copyright(c) 2013 STcube Inc.
 *  All right reserved. Seungwoo Kim <ksw@stcube.com>
 *  revision note :
 *           2013. 07. 16 : Ver 0.5  initially created.
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

#include <plat/nx_clkpwr.h>
#include <mach/platform.h>

#define GPIO_DEBUG	0

#define CPUCLK_MAJOR	19
#define CPUCLK_MINOR	1


static struct NX_CLKPWR_RegisterSet *base;

int clk_status; /* 0(org) 1(half clock) 2(quater clock) */

/*
  This driver would provide interface to change
  CPU clock to 800->400->200 or 700->350->175 MHz
  But what if some one(or application) close this device,
  or simply crashed, does we return to original, see 800MHz condition?
  Or just remain as last state?

  Got any unstable condition arose by this clock change?
  Stay tuned what we would go.
*/

// CLKDIV1CPU (CLKMODEREG0[3:0]) & CLKDIV2CPU (CLKMODEREG0[11:8])
extern unsigned int cpu_get_clock_hz(int clk);

void set_clock_state(int state)
{
	unsigned int cm0;
	int i;

	if (state != clk_status) {
		cm0 = base->CLKMODEREG0;
		cm0 &= ~0xF0F;
		switch (state) {
		  case 0 :		    
		    cm0 |= 0x300;
		    base->CLKMODEREG0 = cm0;
		    base->PWRMODE |= 0x8000;
		    msleep(10);
		    for (i=0;i<1000; i++) {
		    	if (base->PWRMODE & 0x8000 == 0) break;
		    }
		    break;
		  case 1 :
		    cm0 |= 0x101;
		    base->CLKMODEREG0 = cm0;
		    base->PWRMODE |= 0x8000;
		    msleep(10);
		    for (i=0;i<1000; i++) {
		    	if (base->PWRMODE & 0x8000 == 0) break;
		    }
		    break;
		  case 2 :
		    cm0 |= 0x003;
		    base->CLKMODEREG0 = cm0;
		    base->PWRMODE |= 0x8000;
		    msleep(10);
		    for (i=0;i<1000; i++) {
		    	if (base->PWRMODE & 0x8000 == 0) break;
		    }	  
		  	break;
	  	default:
	  		printk("You should set valid value within (0~2).\n");
	  		state = clk_status;
	  	}
	  	printk("CPU clock = %dHz\n", cpu_get_clock_hz(2)); //: FCLK
	  	clk_status = state;
	}
}

static ssize_t
cpuclk_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int ret=0;

	if (count > 0) {
		ret = copy_to_user(buf, &clk_status, sizeof(clk_status));
		printk("CPU clock = %dHz\n", cpu_get_clock_hz(2)); //: FCLK
	}
	return ret;
}

static ssize_t
cpuclk_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	int ret=0;
	int state;

	if (count > 0) {
		ret = copy_from_user(&state, buf, sizeof(clk_status));
		set_clock_state(state);
	}	
	return ret;
}

static int
cpuclk_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int
cpuclk_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int cpuclk_ioctl(struct inode *node, struct file *fl, unsigned int com,
		    unsigned long arg)
{
    return 0;
}


static const struct file_operations cpuclk_fops = {
	.read		= cpuclk_read,
	.write		= cpuclk_write,
	.ioctl      = cpuclk_ioctl,
	.open		= cpuclk_open,
	.release	= cpuclk_release,
};

static struct class *cpuclk_class;

static int __init cpuclk_init(void)
{
	if (register_chrdev(CPUCLK_MAJOR, "cpuclk", &cpuclk_fops))
		panic("unable to get major %d for gpled device", CPUCLK_MAJOR);
	cpuclk_class = class_create(THIS_MODULE, "cpuclk");

	device_create(cpuclk_class, NULL, MKDEV(CPUCLK_MAJOR, CPUCLK_MINOR), NULL, "cpuclk");
	base = (struct NX_CLKPWR_RegisterSet *) IO_ADDRESS(PHY_BASEADDR_CLKPWR_MODULE);

	printk("CPU clock device is registered and ready to use.\n");
	clk_status = 0;

	return 0;
}

static void __exit cpuclk_cleanup(void)
{
	/* do nothing? */
}

module_init(cpuclk_init);
module_exit(cpuclk_cleanup);

MODULE_AUTHOR("Seungwoo Kim <ksw@stcube.com>");
MODULE_DESCRIPTION("nxp2120 STcube/LGE CPU Clock Driver");
MODULE_LICENSE("GPL");
