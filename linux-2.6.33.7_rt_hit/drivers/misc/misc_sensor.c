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

#include <linux/poll.h>

#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/mach-types.h>

#include <mach/platform.h>

#include "misc_sensor.h"

#if (0)
#define DBGOUT(msg...)	{ printk(KERN_INFO "misc_sens: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define DEBUG_DUMP	0

#define GPIO_SENS_DRV_NAME	"misc_sens"
#define DRIVER_VERSION	"0.5"


#define GPIO_SENS_IRQ		(IRQ_GPIO_A_START + 17)
#define GPIO_SENS_IRQ2		(IRQ_GPIO_A_START + 18)

#define GPIO_SENS_DEV_MAJOR			223

// OUTPUT
#define USB_CAM_PWR_CTLB	(1 << 5)	// HIGH_ACTIVE, DEF:HIGH	PC6 : CTLB
#define SENSOR_ENABLE		(1 << 8)	// HIGH_ACTIVE, DEF:LOW
#define USB_HUB_RESET		(1 << 9)	// LOW_ACTIVE,  DEF:HIGH

#define USB_CAM_PWR_CTLA	(1 << 17)	// HIGH_ACTIVE, DEF:HIGH	MIC_CTLA : CTLA

#define LED_CTRL			(1 << 22)

// INPUT
#define SENSOR_DOOR_RIGHT	(1 << 0)	// RISING_ACTIVE
#define SENSOR_DOOR_LEFT	(1 << 1)	// RISING_ACTIVE
#define SENSOR_VEGI_LEFT	(1 << 2)	// RISING_ACTIVE
#define SENSOR_VEGI_RIGHT	(1 << 3)	// RISING_ACTIVE
#define USB_PWR_FAULTB		(1 << 6)	// LOW_ACTIVE				PC7 : FLGB

#define USB_PWR_FAULTA		(1 << 10)	// LOW_ACTIVE				PC8 : FLGA

#define GPIO_OUTPUT_MASK	(USB_CAM_PWR_CTLB | SENSOR_ENABLE | USB_HUB_RESET | LED_CTRL | USB_CAM_PWR_CTLA)
#define GPIO_INPUT_MASK		(SENSOR_DOOR_RIGHT | SENSOR_DOOR_LEFT | SENSOR_VEGI_LEFT | SENSOR_VEGI_RIGHT | USB_PWR_FAULTB | USB_PWR_FAULTA)

#define TEST_PPOINT_55_BIT		(12)
#define TEST_PPOINT_56_BIT		(13)
#define TEST_PPOINT_57_BIT		(14)
#define TEST_PPOINT_58_BIT		(15)
#define TEST_PPOINT_59_BIT		(16)

#define TEST_PPOINT_55_MASK		(1 << 12)
#define TEST_PPOINT_56_MASK		(1 << 13)
#define TEST_PPOINT_57_MASK		(1 << 14)
#define TEST_PPOINT_58_MASK		(1 << 15)
#define TEST_PPOINT_59_MASK		(1 << 16)

#define TEST_PPOINT_55_ID		(0)
#define TEST_PPOINT_56_ID		(1)
#define TEST_PPOINT_57_ID		(2)
#define TEST_PPOINT_58_ID		(3)
#define TEST_PPOINT_59_ID		(4)

#define TEST_POINT_MASK		(TEST_PPOINT_55_MASK | TEST_PPOINT_56_MASK | TEST_PPOINT_57_MASK | TEST_PPOINT_58_MASK | TEST_PPOINT_59_MASK)

#define SENSOR_COUNT		(sizeof(gpio_irq_table) / sizeof(irq_table_t))
#define TP_COUNT			(sizeof(tp_irq_table) / sizeof(irq_table_t))

typedef struct {
	int 		  irq;
	unsigned long flags;
} irq_table_t;

irq_table_t gpio_irq_table [] = {
		{ .irq = 0, .flags = (IRQF_DISABLED | IRQF_TRIGGER_RISING) },
		{ .irq = 1, .flags = (IRQF_DISABLED | IRQF_TRIGGER_RISING) },
		{ .irq = 2, .flags = (IRQF_DISABLED | IRQF_TRIGGER_RISING) },
		{ .irq = 3, .flags = (IRQF_DISABLED | IRQF_TRIGGER_RISING) },
		{ .irq = 6, .flags = (IRQF_DISABLED | IRQF_TRIGGER_FALLING) },
		{ .irq = 10, .flags = (IRQF_DISABLED | IRQF_TRIGGER_FALLING) },
	};

irq_table_t	tp_irq_table[] = {
		{ .irq = 12, .flags = (IRQF_DISABLED | IRQF_TRIGGER_RISING) },
		{ .irq = 13, .flags = (IRQF_DISABLED | IRQF_TRIGGER_RISING) },
		{ .irq = 14, .flags = (IRQF_DISABLED | IRQF_TRIGGER_RISING) },
		{ .irq = 15, .flags = (IRQF_DISABLED | IRQF_TRIGGER_RISING) },
		{ .irq = 16, .flags = (IRQF_DISABLED | IRQF_TRIGGER_RISING) },
	};

typedef struct {
	unsigned int	  irq[SENSOR_COUNT];
	pwm_info		  pwm;
	u32				  status;
	u32				  flag;
	unsigned int	  tpirq[TP_COUNT];
	gpio_tp_mode      tpmode[TP_COUNT];
	u32				  tpout_mask;
	u32				  tpin_mask;
    wait_queue_head_t waitq;
    spinlock_t		  lock;
} sensor_device_t;

static irqreturn_t sensor_irq(int irq, void *dev_id);

sensor_device_t *sensor_data;

static struct class *sensor_class;

static struct NX_GPIO_RegisterSet *sensor_base = NULL;
static struct NX_PWM_RegisterSet *pwm_base = NULL;

/*--------------------------------------------------------------------------------
 * file_operations
 ---------------------------------------------------------------------------------*/
static int sensor_open(struct inode *inode, struct file *file)
{
	DBGOUT("%s\n", __func__);
	
	file->private_data = sensor_data;
	sensor_data->flag  = 0;

	return 0;
}

static int sensor_release(struct inode *inode, struct file *file)
{
	DBGOUT("%s\n", __func__);

	file->private_data = NULL;

	return 0;
}

static unsigned int sensor_poll(struct file *file, poll_table *wait)
{
	sensor_device_t	*sens = file->private_data;
	unsigned long l;

	poll_wait(file, &sens->waitq, wait);

	//spin_lock_irq(&sens->lock);
	l = sens->flag;
	//spin_unlock_irq(&sens->lock);

	if (l != 0)
		return POLLIN | POLLRDNORM;

	return 0;
}

static ssize_t sensor_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	sensor_device_t	*sens = file->private_data;
	u32 status;
	int ret=0;

	if (count > 0)
	{
		if (sens->flag == 0)
		{
			sensor_data->status = sensor_base->GPIOxPAD & (GPIO_OUTPUT_MASK | GPIO_INPUT_MASK | sens->tpout_mask | sens->tpin_mask);
			DBGOUT("%s : status=%0X GPIOxPAD=%0X GPIOxOUT=%0X flag=%d mask=%X\n", __func__, sensor_data->status, sensor_base->GPIOxPAD, sensor_base->GPIOxOUT, sens->flag, (GPIO_OUTPUT_MASK | GPIO_INPUT_MASK | sens->tpout_mask | sens->tpin_mask));
		}
		DBGOUT("%s : status=%0X GPIOxPAD=%0X GPIOxOUT=%0X flag=%d mask=%X\n", __func__, sensor_data->status, sensor_base->GPIOxPAD, sensor_base->GPIOxOUT, sens->flag, (GPIO_OUTPUT_MASK | GPIO_INPUT_MASK | sens->tpout_mask | sens->tpin_mask));
		ret = copy_to_user(buf, &sensor_data->status, sizeof(sensor_data->status));

	}
	sens->flag = 0;

	return ret;
}

static ssize_t sensor_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	sensor_device_t	*sens = file->private_data;
	u32	val;
	u32 status;
	int ret=0;

	if (count > 0) {
		ret = copy_from_user(&val, buf, sizeof(val));

		if (ret != 0) return ret;

		DBGOUT("%s : status=%0X GPIOxPAD=%0X GPIOxOUT=%0X\n", __func__, status, sensor_base->GPIOxPAD, sensor_base->GPIOxOUT);

		status  = sensor_base->GPIOxOUT & ~GPIO_OUTPUT_MASK;
		status |= val & (GPIO_OUTPUT_MASK | sens->tpout_mask);

		sensor_base->GPIOxOUT = status;

		DBGOUT("%s : status=%0X GPIOxPAD=%0X GPIOxOUT=%0X\n", __func__, status, sensor_base->GPIOxPAD, sensor_base->GPIOxOUT);
	}

	return ret;
}

/*------------------------------------------------------------------------------- 
	IOCTL service routine.
--------------------------------------------------------------------------------*/
static long sensor_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
	sensor_device_t	*sens = filp->private_data;
    unsigned short int duty1, duty2;
    pwm_info	pwm;
    gpio_tp_mode	tpmode;
    u32				tpmask;
    void __user *argp = (void __user *)arg;
	int ret   = 0;

	DBGOUT("%s (cmd:0x%x, nr:%d)\n",
		__func__, cmd, _IOC_NR(cmd));

	switch(cmd)	{
        case IOCTL_SENSOR_PWM_DUTY_CTRL_R :
            ret = copy_from_user(&pwm, argp, sizeof(pwm)); // ret : make compiler happy

            if (ret != 0) break;

			if (pwm.ch < 0 || pwm.ch >= 2)
				return -EIO;

            pwm.duty = pwm_base->PWM2[0].PWM_DUTY[pwm.ch];
            ret = copy_to_user(argp, &pwm, sizeof(pwm)); // ret : make compiler happy
    		DBGOUT("%s : IOCTL_GPIO_PWM_DUTY_CTRL_R duty=%d\n", __func__, pwm.duty);
            break;

        case IOCTL_SENSOR_PWM_DUTY_CTRL_W :
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

        case IOCTL_SENSOR_PWM_PERIOD_CTRL_R :
            ret = copy_from_user(&pwm, argp, sizeof(pwm)); // ret : make compiler happy

            if (ret != 0) break;

			if (pwm.ch < 0 || pwm.ch >= 2)
				return -EIO;

            pwm.period = pwm_base->PWM2[0].PWM_PERIOD[pwm.ch];
            ret = copy_to_user(argp, &pwm, sizeof(pwm)); // ret : make compiler happy
    		DBGOUT("%s : IOCTL_GPIO_PWM_PERIOD_CTRL_R period=%d\n", __func__, pwm.period);
            break;

        case IOCTL_SENSOR_PWM_PERIOD_CTRL_W :
            ret = copy_from_user(&pwm, argp, sizeof(pwm)); // ret : make compiler happy

            if (ret != 0) break;

			if (pwm.ch < 0 || pwm.ch >= 2 || pwm.period < 0 || pwm.period >= (1<<10))
				return -EIO;

            pwm_base->PWM2[0].PWM_PERIOD[pwm.ch] = pwm.period;
    		DBGOUT("%s : IOCTL_GPIO_PWM_PERIOD_CTRL_W period=%d\n", __func__, pwm.period);
            break;

        case IOCTL_GPIO_TEST_POINT_MODE_R :
        	ret = copy_from_user(&tpmode, argp, sizeof(gpio_tp_mode)); // ret : make compiler happy

            if (ret != 0) break;

			if (tpmode.id < 0 || tpmode.id >= TP_COUNT )
				return -EIO;

			tpmode.dir     = sens->tpmode[tpmode.id].dir;
			tpmode.irqflag = sens->tpmode[tpmode.id].irqflag;
            ret = copy_to_user(argp, &tpmode, sizeof(pwm)); // ret : make compiler happy
    		DBGOUT("%s : IOCTL_GPIO_TEST_POINT_MODE_R dir=%d irqflag=%X\n", __func__, tpmode.dir, tpmode.irqflag);
			break;

        case IOCTL_GPIO_TEST_POINT_MODE_W :
        	ret = copy_from_user(&tpmode, argp, sizeof(gpio_tp_mode)); // ret : make compiler happy

            if (ret != 0) break;

			if (tpmode.id < 0 || tpmode.id >= TP_COUNT )
				return -EIO;

			tpmask = 1 << (12 + tpmode.id);

			sens->tpout_mask &= ~tpmask;
			sens->tpin_mask  &= ~tpmask;

			if (sens->tpmode[tpmode.id].dir == 0)
				free_irq(sens->tpirq[tpmode.id], sens);

			if (tpmode.dir == 1)
			{
				sens->tpout_mask |=  tpmask;
			}
			else
			{
				sens->tpin_mask |=  tpmask;

				// Now we should install irq for done/request.
				if (request_irq(sens->tpirq[tpmode.id] , sensor_irq, IRQF_DISABLED | tpmode.irqflag, GPIO_SENS_DRV_NAME, sens)) {
					DBGOUT("test point request_irq failed\n");
				}
			}
			sens->tpmode[tpmode.id].dir     = tpmode.dir;
			sens->tpmode[tpmode.id].irqflag = tpmode.irqflag;

			sensor_base->GPIOxOUTENB        = (sensor_base->GPIOxOUTENB & ~sens->tpout_mask) | sens->tpout_mask;

    		DBGOUT("%s : IOCTL_GPIO_TEST_POINT_MODE_W dir=%d irqflag=%X\n", __func__, tpmode.dir, tpmode.irqflag);
			break;

        default:
			DBGOUT("Fail, unknown ioctl ...\n");
			return -1;
	}
	DBGOUT("IoCtl (cmd:0x%x, nr:%d, ret:%d) \n\n", cmd, _IOC_NR(cmd), ret);

	return ret;
}


struct file_operations sensor_fops = {
	.owner 			= THIS_MODULE,
	.read		    = sensor_read,
	.write		    = sensor_write,
	.poll			= sensor_poll,
	.open 			= sensor_open,
	.release		= sensor_release,
	.unlocked_ioctl	= sensor_ioctl,
};

static irqreturn_t sensor_irq(int irq, void *dev_id)
{
	sensor_device_t *sens = (sensor_device_t *) dev_id;

#if DEBUG_DUMP
	printk("irq gen\n");
#endif

	sens->status = (sensor_base->GPIOxPAD & (GPIO_OUTPUT_MASK | GPIO_INPUT_MASK | sens->tpout_mask | sens->tpin_mask));

	sens->flag = 0x01;

	wake_up_interruptible(&sens->waitq);

	DBGOUT("%s : status=%0X GPIOxPAD=%0X GPIOxOUT=%0X\n", __func__, sens->status, sensor_base->GPIOxPAD, sensor_base->GPIOxOUT);

	return IRQ_HANDLED;
}

static int __init sensor_init(void)
{
	int ret;
	int i=0;
	int j=0;
	sensor_device_t *sens;
	struct NX_GPIO_RegisterSet *gpiob = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO + 0x40);


	sensor_base = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO);
	pwm_base  = (struct NX_PWM_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_PWM);

	DBGOUT("enter %s\n", __func__);
	sens = kzalloc(sizeof(sensor_device_t), GFP_KERNEL);
	if (sens == NULL)
		return -ENOMEM;
	
	/* register character driver */
	ret = register_chrdev(GPIO_SENS_DEV_MAJOR, GPIO_SENS_DRV_NAME, &sensor_fops);
	if (0 > ret)	{
		DBGOUT("Fail, register device (%s, major:%d)\n",
			GPIO_SENS_DRV_NAME, GPIO_SENS_DEV_MAJOR);
		goto err_alloc;
	}

	sensor_class = class_create(THIS_MODULE, GPIO_SENS_DRV_NAME);
	device_create(sensor_class, NULL, MKDEV(GPIO_SENS_DEV_MAJOR, 0), NULL, GPIO_SENS_DRV_NAME);

	init_waitqueue_head(&sens->waitq);

	for (i = 0 ; i < sizeof(sens->tpmode) / sizeof(gpio_tp_mode) ; i++ )
	{
		sens->tpirq[i]			= IRQ_GPIO_A_START+ TEST_PPOINT_55_BIT +  i;

		sens->tpmode[i].id      = i;
		sens->tpmode[i].dir     = 0x01;
		sens->tpmode[i].irqflag = 0;
	}
	sens->tpout_mask = TEST_POINT_MASK;
	sens->tpin_mask  = 0;

	for (i = 0 ; i < SENSOR_COUNT ; i++)
	{
		// should be from MACH...
		sens->irq[i] = IRQ_GPIO_A_START + gpio_irq_table[i].irq; // or irq (above)

		// Now we should install irq for done/request.
		if (request_irq(sens->irq[i] , sensor_irq, gpio_irq_table[i].flags, GPIO_SENS_DRV_NAME, sens)) {
			DBGOUT("sensor request_irq failed\n");
			goto err_sysfs;
			}
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

    DBGOUT("%s : GPIOA PAD=%0X\n", __func__, sensor_base->GPIOxPAD);

    sens->status = (sensor_base->GPIOxPAD & (GPIO_OUTPUT_MASK | GPIO_INPUT_MASK | sens->tpout_mask));

	printk("sensor device is ready.\n");
	sensor_data = sens;

	return 0;
err_sysfs:

	for ( ; i > 0 ; i--)
		free_irq(sens->irq[i-1], sens);

	for ( ; j > 0 ; j--)
		free_irq(sens->tpirq[j-1], sens);

	DBGOUT("unregister chrdev.\n");
	unregister_chrdev(GPIO_SENS_DEV_MAJOR, "gpio sensor driver");

err_alloc:
	kfree(sens);

	return ret;
}

static void __exit sensor_exit(void)
{
	sensor_device_t *sens = sensor_data;
	int i;

	for (i = 0 ; i < SENSOR_COUNT ; i++)
		free_irq(sens->irq[i], sens);

	for (i = 0 ; i < TP_COUNT ; i++)
	{
		if (sens->tpmode[i].dir == 0)
			free_irq(sens->tpirq[i], sens);
	}

	kfree(sens);
}

MODULE_AUTHOR("Seungwoo Kim <ksw@stcube.com>");
MODULE_DESCRIPTION("MISC Sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(sensor_init);
module_exit(sensor_exit);

