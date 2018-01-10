/*
 * ALSA SoC CX20865 codec driver
 *
 * Copyright:   (C) 2013 Conexant Systems
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * 
 *      
 *************************************************************************
 *  Modified Date:  04/11/13
 *  File Version:   1.1.0.0 
 *************************************************************************
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/module.h>	/* Specifically, a module */
#include <linux/fs.h>
#include <asm/uaccess.h>	/* for get_user and put_user */
#include <sound/soc.h>
#include <linux/gpio.h>
#include "cx20865.h"


#define DEBUG 1

#include "cxdebug.h"

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
#include <linux/i2c.h>
#endif 

#if defined(CONFIG_SPI_MASTER)
#include <linux/spi/spi.h>
#endif 


/*
* 
 * Is the device open right now? Used to prevent
 * concurent access into the same device 
 */
static int Device_Open = 0;
extern int CX_AUDDRV_VERSION;
struct snd_soc_codec *g_codec = NULL;

/* 
 * This is called whenever a process attempts to open the device file 
 */
static int cxdbg_dev_open(struct inode *inode, struct file *file)
{
#ifdef DEBUG
    printk(KERN_INFO "cxdbg_dev: device_open(%p)\n", file);
#endif

    /* 
    * We don't want to talk to two processes at the same time 
    */
    if (Device_Open)
        return -EBUSY;

    Device_Open++;
    /*
    * Initialize the message 
    */

   // try_module_get(THIS_MODULE);
    return 0;
}

/* 
 * This is called whenever a process attempts to open the device file 
 */
static int cxdbg_dev_release(struct inode *inode, struct file *file)
{
#ifdef DEBUG
    printk(KERN_INFO "cxdbg_dev: device_release(%p)\n", file);
#endif


    Device_Open--;
    /*
    * Initialize the message 
    */

    return 0;
}
static int codec_reg_write(struct CXDBG_IODATA  *reg)
{
    int errno = 0;

    BUG_ON(!g_codec);
    BUG_ON(!g_codec->hw_write);

    if(g_codec&& g_codec->hw_write)
    {
        if( g_codec->hw_write(g_codec,(char*)reg->data,reg->len) 
            !=reg->len)
        {
            errno =-EIO;
            printk(KERN_ERR "cxdbg_dev: codec_reg_write failed\n");
        }
    }
    else
    {
        errno = -EBUSY;
        printk(KERN_ERR "cxdbg_dev: codec_reg_write failed, device is not ready.\n");
    }
    return errno;
}

static unsigned int codec_reg_read(struct CXDBG_IODATA  *reg)
{
    int errno = 0;
    unsigned int regaddr;
    unsigned int data;


    BUG_ON(!g_codec);
    BUG_ON(!g_codec->hw_read);

    if (reg-> len == 2)
    {
        regaddr = (((unsigned int)reg->data[0])<<8) + reg->data[1];
    }
    else if (reg->len == 1)
    {
        regaddr = (unsigned int)reg->data[0];
    }
    else 
    {
       printk(KERN_ERR "cxdbg_dev: codec_reg_read failed, invalid parameter.\n");
       return -EINVAL;
    }
    memset(reg,0,sizeof(*reg));
    if(g_codec && g_codec->hw_read)
    {
        data = g_codec->hw_read(g_codec,regaddr);
        reg->data[0] = data & 0xFF;
        reg->len     = 1;
    }
    else
    {
        errno = -EBUSY;
        printk(KERN_ERR "cxdbg_dev: codec_reg_read failed, device is not ready.\n");
    }
    return errno;

}

static int i2c_access(struct CXDBG_I2CDATA  *i2c_data)
{
    int errno = 0;
    unsigned int regaddr;
    unsigned int data;

    struct i2c_client  *client;
    struct i2c_adapter *adap  ;
    struct i2c_msg      msg[2];
   

    BUG_ON(!g_codec);
    BUG_ON(!g_codec->control_data);
    // verify the parameters
    if( i2c_data->num_of_write == 0 ||
        i2c_data->num_of_write > MAX_I2C_DATA ||
        i2c_data->num_of_read > MAX_I2C_DATA)
    {
        printk(KERN_ERR "cxdbg_dev: i2c_access failed, invalid parameter.\n");
        return -EINVAL;
    }

    client = (struct i2c_client  *)g_codec->control_data;
    adap   = client->adapter;

    msg[0].addr  = client->addr;
    msg[0].flags = client->flags & I2C_M_TEN;
    msg[0].len   = i2c_data->num_of_write; 
    msg[0].buf   = i2c_data->data;

    if( i2c_data->num_of_read )
    {
        // write then read.
        __u8 * rd_data = (__u8 *) kmalloc(i2c_data->num_of_read,GFP_KERNEL);
        msg[1].addr  = client->addr;
        msg[1].flags = (client->flags & I2C_M_TEN) | I2C_M_RD;
        msg[1].len   = i2c_data->num_of_read;
        msg[1].buf   = rd_data;
        if (i2c_transfer(adap,msg,2)!=2)
        {
            printk(KERN_ERR "cxdbg_dev: i2c_transfer failed.\n");
            errno = -EIO;
        }
        memcpy(i2c_data->data,rd_data,i2c_data->num_of_read);
        kfree(rd_data);
    }
    else
    {
        // write only
        if (i2c_transfer(adap,msg,1)!=1)
        {
            printk(KERN_ERR "cxdbg_dev: i2c_transfer failed.\n");
            errno = -EIO;
        }
    }

    return errno;

}

static int set_reset_pin(int state)
{
    int err = 0;
    struct cx20865_setup_data     *cx20865_setup;
    BUG_ON(!g_codec);

    cx20865_setup = (struct cx20865_setup_data     *)g_codec->socdev->codec_data;   
    int reset_pin = cx20865_setup->gpio_reset_pin;
    if (gpio_is_valid(reset_pin)) {
        if (gpio_request(reset_pin, "reset_pin")) {
            printk( KERN_ERR "cxdbg_dev: reset pin %d not available\n",reset_pin);
            err = -ENODEV;
        } else {
            gpio_direction_output(reset_pin, 1);
            gpio_set_value(reset_pin, state);
            gpio_free(reset_pin); 
        }
    }
    else
    {
        printk( KERN_ERR "cxdbg_dev: reset pin %d is not valid\n",reset_pin);
        err = -ENODEV;
    }
    return err;
}

long cxdbg_dev_ioctl(struct file * file, unsigned int cmd, unsigned long arg)
{
    int     pin_state;
    long    errno;
    struct CXDBG_I2CDATA  *i2c_data=NULL ;
    int __user *ip = (int __user*) arg;
#ifdef DEBUG
    printk(KERN_INFO "cxdbg_dev: ioctl, cmd=0x%02x, arg=0x%02lx\n", cmd, arg);
#endif
    
    /* 
     *   Switch according to the ioctl called 
     */
    switch (cmd) {
    case CXDBG_IOCTL_I2C_SET:
        i2c_data = (struct CXDBG_I2CDATA*) kmalloc(sizeof(*i2c_data),GFP_KERNEL);
        copy_from_user((char*) i2c_data, (char*)arg,sizeof(*i2c_data));
        errno = i2c_access(i2c_data);
        copy_to_user((char*) arg, (char*)i2c_data,sizeof(*i2c_data));
        break;

    case CXDBG_IOCTL_RESET_PIN:
        if (get_user(pin_state, (int*) arg))
            return -EFAULT;
        errno = set_reset_pin(pin_state);
        break;
    default:
        return -EINVAL;
    }

    if(i2c_data)
    {
        kfree(i2c_data);
    }
    return errno;
}


#if defined(_MSC_VER)
static const struct file_operations cxdbg_dev_fops =
{
    /*.owner =	*/THIS_MODULE,
    /*.llseek*/NULL,
    /*.read =		*/NULL,
    /*.write*/ NULL,
    /*.aio_read*/ NULL,
    /*.aio_write*/NULL,
    /*readdir*/NULL,
    /*.poll*/NULL,
    /*ioctl*/ NULL /*i2cdev_ioctl*/,
    /*.unlocked_ioctl*/cxdbg_dev_ioctl,
    /*.compat_ioctl*/NULL,
    /*.mmap*/NULL,
    /*.open*/cxdbg_dev_open,
    /*.flush*/NULL,
    /*.release*/NULL,
    /*.fsync*/NULL,
    /*.aio_fsync*/NULL,
    /*.fasync*/NULL,
    /*.lock*/NULL,
    /*.sendpage*/NULL,
};
#else
static const struct file_operations cxdbg_dev_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = cxdbg_dev_ioctl,
    .open           = cxdbg_dev_open,
    .release        = cxdbg_dev_release,
};
#endif


/* 
* Initialize the module - Register the character device 
*/
int cxdbg_dev_init(struct snd_soc_codec *socdev)
{
    int err;
    printk(KERN_INFO "cxdbg_dev: entries driver\n");

    g_codec = socdev;

    err = register_chrdev(CXDBG_MAJOR, CXDBG_DEVICE_NAME, &cxdbg_dev_fops);
    if (err)
    {
        printk(KERN_ERR "cxdbg_dev: Driver Initialisation failed\n");
    }
    return err;
}

void cxdbg_dev_exit(void)
{
    unregister_chrdev(CXDBG_MAJOR,CXDBG_DEVICE_NAME);
}


MODULE_AUTHOR("Simon Ho<simon.ho@conexant.com");
MODULE_DESCRIPTION("debug driver");
MODULE_LICENSE("GPL");

