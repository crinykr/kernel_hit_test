	/*
 *  gpio.c - Linux kernel module for
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

#include "gpio.h"

#if (0)
#define DBGOUT(msg...)	{ printk(KERN_INFO "gpio: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define DEBUG_DUMP	0

#define GPIO_DRV_NAME	"gpio"
#define DRIVER_VERSION		"0.5"


#define GPIO_ENC_IRQ		(IRQ_GPIO_A_START + 17)
#define GPIO_ENC_IRQ_MASK  	(1 << 17)
#define GPIO_ENC_IRQ2		(IRQ_GPIO_A_START + 18)
#define GPIO_ENC_IRQ_MASK2  (1 << 18)

#define GPIO_ENC_DEV_MAJOR			223

#define GPIO_ENC_DIR0		(1 << 3)
#define GPIO_ENC_DIR1		(1 << 19)

#define GPIO_OUTPUT_MASK	0x1D3C4	// GPIO Output port A 2:CPU_BUZZ_On_Off, 6:CPU_INTP_CNTL, 7:CPU_KEY_LED_OUT, 8:CPU_LED_CONTROL1, 9:CPU_KOMP, 12:CPU_SUCTION, 14:CPU_W0_DIR, 15:CPU_W1_DIR, 16:CPU_LED_CONTROL1
#define GPIO_INPUT_MASK		0x02421	// GPIO Input port A 0:CPU_BIN_FULL, 5:CPU_EXT_INTP, 10:CPU_P_SENSOR, 13:CPU_TANK

typedef struct {
	struct platform_device	*pdev;
	u32				mask;
	int				irq;
	int				irq2;
	int				count;
	int 			count2;
	pwm_info		pwm;
	u32				gpio_status;
} gpio_device_t;

gpio_device_t *gpio_data;

static struct class *gpio_class;

static struct NX_GPIO_RegisterSet *gpio_base = NULL;
static struct NX_PWM_RegisterSet *pwm_base = NULL;

/*--------------------------------------------------------------------------------
 * file_operations
 ---------------------------------------------------------------------------------*/
static int gpio_open(struct inode *inode, struct file *flip)
{
	DBGOUT("%s\n", __func__);
	
	flip->private_data = gpio_data;
	gpio_data->count = 0;
	gpio_data->count2 = 0;

	return 0;
}

static int gpio_release(struct inode *inode, struct file *flip)
{
	DBGOUT("%s\n", __func__);

	return 0;
}

static ssize_t gpio_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	u32 status;
	int ret=0;

	if (count > 0) {
		status  = gpio_base->GPIOxPAD;
		status &= (GPIO_OUTPUT_MASK | GPIO_INPUT_MASK);

		gpio_data->gpio_status = status;

		ret = copy_to_user(buf, &gpio_data->gpio_status, sizeof(status));

		DBGOUT("%s : status=%0X GPIOxPAD=%0X GPIOxOUT=%0X\n", __func__, status, gpio_base->GPIOxPAD, gpio_base->GPIOxOUT);
	}
	return ret;
}

static ssize_t gpio_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	u32	val;
	u32 status;
	int ret=0;

	if (count > 0) {
		ret = copy_from_user(&val, buf, sizeof(val));

		if (ret != 0) return ret;

		status  = gpio_base->GPIOxOUT & ~GPIO_OUTPUT_MASK;
		status |= val & GPIO_OUTPUT_MASK;

		gpio_base->GPIOxOUT = status;

		gpio_data->gpio_status = status;

		DBGOUT("%s : status=%0X GPIOxPAD=%0X GPIOxOUT=%0X\n", __func__, status, gpio_base->GPIOxPAD, gpio_base->GPIOxOUT);
	}

	return ret;
}

/*------------------------------------------------------------------------------- 
	IOCTL service routine.
--------------------------------------------------------------------------------*/
static long gpio_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
    unsigned short int duty1, duty2;
    pwm_info	pwm;
    void __user *argp = (void __user *)arg;
	int ret   = 0;

	DBGOUT("%s (cmd:0x%x, nr:%d)\n",
		__func__, cmd, _IOC_NR(cmd));

	switch(cmd)	{
		case IOCTL_GPIO_ENC_STATUS:
			ret = copy_to_user((void*)arg, (const void*)&gpio_data->count, sizeof(int)*2);
			gpio_data->count = 0;
			gpio_data->count2 = 0;
			break;

		case IOCTL_GPIO_ENC_RESET:
			gpio_data->count = 0;
			gpio_data->count2 = 0;
			break;

        case IOCTL_GPIO_PWM_DUTY_CTRL_R :
            ret = copy_from_user(&pwm, argp, sizeof(pwm)); // ret : make compiler happy

            if (ret != 0) break;

			if (pwm.ch < 0 || pwm.ch >= 2)
				return -EIO;

            pwm.duty = pwm_base->PWM2[0].PWM_DUTY[pwm.ch];
            ret = copy_to_user(argp, &pwm, sizeof(pwm)); // ret : make compiler happy
    		DBGOUT("%s : IOCTL_GPIO_PWM_DUTY_CTRL_R duty=%d\n", __func__, pwm.duty);
            break;

        case IOCTL_GPIO_PWM_DUTY_CTRL_W :
            ret = copy_from_user(&pwm, argp, sizeof(pwm)); // ret : make compiler happy

            if (ret != 0) break;

			if (pwm.ch < 0 || pwm.ch >= 2 || pwm.duty >= (1<<10))
				return -EIO;

            pwm_base->PWM2[0].PWM_DUTY[pwm.ch] = pwm.duty;

            duty1 = pwm_base->PWM2[0].PWM_DUTY[0];
			duty2 = pwm_base->PWM2[0].PWM_DUTY[1];

			if (duty1 > 0 || duty2 > 0) {
                pwm_base->PWM_CLKENB = 0x0C; /* PCLK always and GlockGeneration Enable */
            } else {
                if (duty1 == 0 && duty2 == 0 && pwm_base->PWM_CLKENB == 0x0C) {
                    /* We need to make some delay as update 1 period...*/
                    msleep(1);
                    pwm_base->PWM_CLKENB = 0x08; /* PCLK always and GlockGeneration Disable */
                }
            }
    		DBGOUT("%s : IOCTL_GPIO_PWM_DUTY_CTRL_W ch=%d duty1=%d duty2=%d\n", __func__, (int)pwm.ch, (int)duty1, (int)duty2);
            break;

        case IOCTL_GPIO_PWM_PERIOD_CTRL_R :
            ret = copy_from_user(&pwm, argp, sizeof(pwm)); // ret : make compiler happy

            if (ret != 0) break;

			if (pwm.ch < 0 || pwm.ch >= 2)
				return -EIO;

            pwm.period = pwm_base->PWM2[0].PWM_PERIOD[pwm.ch];
            ret = copy_to_user(argp, &pwm, sizeof(pwm)); // ret : make compiler happy
    		DBGOUT("%s : IOCTL_GPIO_PWM_PERIOD_CTRL_R period=%d\n", __func__, pwm.period);
            break;

        case IOCTL_GPIO_PWM_PERIOD_CTRL_W :
            ret = copy_from_user(&pwm, argp, sizeof(pwm)); // ret : make compiler happy

            if (ret != 0) break;

			if (pwm.ch < 0 || pwm.ch >= 2 || pwm.period < 0 || pwm.period >= (1<<10))
				return -EIO;

            pwm_base->PWM2[0].PWM_PERIOD[pwm.ch] = pwm.period;
    		DBGOUT("%s : IOCTL_GPIO_PWM_PERIOD_CTRL_W period=%d\n", __func__, pwm.period);
            break;

        default:
			DBGOUT("Fail, unknown ioctl ...\n");
			return -1;
	}
	DBGOUT("IoCtl (cmd:0x%x, nr:%d, ret:%d) \n\n", cmd, _IOC_NR(cmd), ret);

	return ret;
}


struct file_operations gpio_fops = {
	.owner 			= THIS_MODULE,
	.read		    = gpio_read,
	.write		    = gpio_write,
	.open 			= gpio_open,
	.release		= gpio_release,
	.unlocked_ioctl	= gpio_ioctl,
};

static irqreturn_t gpio_enc_irq(int irq, void *dev_id)
{
	gpio_device_t *enc = (gpio_device_t *) dev_id;


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
	gpio_device_t *enc = (gpio_device_t *) dev_id;


#if DEBUG_DUMP
	printk("irq gen\n");
#endif
	if (gpio_base->GPIOxPAD & GPIO_ENC_DIR1)
		enc->count2++;
	else
		enc->count2--;

	return IRQ_HANDLED;
}

static int __init gpio_init(void)
{
	int ret;
	gpio_device_t *enc;
	struct NX_GPIO_RegisterSet *gpiob = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO + 0x40);


	gpio_base = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO);
	pwm_base  = (struct NX_PWM_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_PWM);

	DBGOUT("enter %s\n", __func__);
	enc = kzalloc(sizeof(gpio_device_t), GFP_KERNEL);
	if (enc == NULL)
		return -ENOMEM;
	
	// should be from MACH...
	enc->irq = GPIO_ENC_IRQ; // or irq (above)
	enc->irq2 = GPIO_ENC_IRQ2; // or irq (above)
	
	/* register character driver */
	ret = register_chrdev(GPIO_ENC_DEV_MAJOR, GPIO_DRV_NAME, &gpio_fops);
	if (0 > ret)	{
		DBGOUT("Fail, register device (%s, major:%d)\n",
			GPIO_DRV_NAME, GPIO_ENC_DEV_MAJOR);
		goto err_alloc;
	}

	gpio_class = class_create(THIS_MODULE, GPIO_DRV_NAME);
	device_create(gpio_class, NULL, MKDEV(GPIO_ENC_DEV_MAJOR, 0), NULL, GPIO_DRV_NAME);
			
	// Now we should install irq for done/request.
	if (request_irq(enc->irq , gpio_enc_irq, IRQF_DISABLED | IRQF_TRIGGER_RISING, GPIO_DRV_NAME, enc)) {
		DBGOUT("gpio enc request_irq failed\n");
		goto err_sysfs;
	}
	
	if (request_irq(enc->irq2 , gpio_enc_irq2, IRQF_DISABLED | IRQF_TRIGGER_RISING, GPIO_DRV_NAME, enc)) {
		DBGOUT("gpio enc request_irq2 failed\n");
		goto err_sysfs;
	}

	DBGOUT("%s : GPIOB GPIOxALTFN[0] %0X\n", __func__, gpiob->GPIOxALTFN[0]);

	gpiob->GPIOxALTFN[0] &= 0xFFFFFF0F; /* GPIOB 2, 3 */
    gpiob->GPIOxALTFN[0] |= 0x00000050; /* ALT1:2, 3 */

    pwm_base->PWM2[0].PWM_DUTY[0] = 0;
    pwm_base->PWM2[0].PWM_DUTY[1] = 0;
    pwm_base->PWM2[0].PWM_PERIOD[0] = 600;
    pwm_base->PWM2[0].PWM_PERIOD[1] = 600;
    /* I would set PLL1, and 192MHz to 48MHz as ClockDivider to 4 */
    pwm_base->PWM_CLKGEN = ((4-1) << 5) | (1 << 2) | 0;
    /* Then prescaler to 4 for 12MHz base, then with 1/1000 of period, It would be 12KHz base clock for PWM */
    pwm_base->PWM2[0].PWM_PREPOL = (0 << 7) | 4 | ((0 << 7) | 4) << 8; /* for PWM0, PWM1 */
    /* Now enable Clock for PWM unit */
    pwm_base->PWM_CLKENB = 0x08; /* PCLK always and GlockGeneration Disable */

    enc->gpio_status = gpio_base->GPIOxPAD;

	printk("gpio device is ready.\n");
	gpio_data = enc;

	return 0;
err_sysfs:

err_register:
	DBGOUT("unregister chrdev.\n");
	unregister_chrdev(GPIO_ENC_DEV_MAJOR, "gpio driver");

err_alloc:
	kfree(enc);

	return ret;
}

static void __exit gpio_exit(void)
{
	gpio_device_t *enc = gpio_data;

	free_irq(enc->irq, enc);
	free_irq(enc->irq2, enc);
	
	kfree(enc);
}

MODULE_AUTHOR("Seungwoo Kim <ksw@stcube.com>");
MODULE_DESCRIPTION("GPIO enc driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(gpio_init);
module_exit(gpio_exit);

