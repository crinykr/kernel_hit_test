/**********************************************************************
*  Copyright(c) 2011MOSTiTECH co., ltd.
*  All Rights Reserved. by Seungwoo Kim <ksw@mostitech.com>
*
*  This file is based the definition by
*    Driver for HI161 CMOS Image Sensor from Hynix
*
*   Copyright (C) 2008, Robert Jarzmik <robert.jarzmik@free.fr>
*   Copyright (C) 2011, Seungwoo Kim <ksw@mostitech.com>
*  But completely rewritten for most6410/LGE board
*
**********************************************************************/
/*!
 * @file hi161.c
 *
 * @brief Hynix VGA camera driver functions
 *
 * @ingroup Camera
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/init.h>
#include <asm/io.h>

#include "nx_vip.h"
#include "hi161.h"

static int debug;

module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

static int reset_frame_rate = 30;
static int hi161_powered =0;
static u32 hi161_pixfmt=0;
static int hi161_flicker=0;
static int hi161_autoexposure=0;
static int hi161_autowhitebalance=0;
static int hi161_hflip=0;
static int hi161_vflip=0;
static int hi161_gain = 0;
static int sensor_init = 0;

static void hi161_set_ae(int active);
static void hi161_set_awb(int active);
static void hi161_flicker_control(int control);

#define COL_FMT(_name, _depth, _fourcc, _colorspace) \
	{ .name = _name, .depth = _depth, .fourcc = _fourcc, \
	.colorspace = _colorspace }
#define RGB_FMT(_name, _depth, _fourcc) \
	COL_FMT(_name, _depth, _fourcc, V4L2_COLORSPACE_SRGB)

#define I2C_FLAG_READ 0x10

/*
 * Function definitions
 */

static int hi161_i2c_client_xfer(struct i2c_client *client, u8 reg, char *buf, int num, int tran_flag)
{
	struct i2c_msg msg[2];
	int ret;
	u8 data[4];

	if (tran_flag & I2C_FLAG_READ) {
	    msg[0].addr = addr;
	    msg[0].len = 1;
	    msg[0].buf = reg;
	    msg[0].flags = 0;

	    msg[1].addr = addr;
	    msg[1].len = num;
	    msg[1].buf = buf;
	    msg[1].flags = I2C_M_RD;

	    ret = i2c_transfer(client->adapter, msg, 2);
	} else {
	    data[0] = *reg;
	    data[1] = buf[0];

	    msg[0].addr = addr;
	    msg[0].len = 2;
	    msg[0].buf = data;
	    msg[0].flags = 0;
	    ret = i2c_transfer(client->adapter, msg, 1);
	}

	if (ret >= 0)
		return 0;

	return ret;
}


static int reg_page_map_set(struct i2c_client *client, const u16 reg)
{
	int ret;
	u8 temp1;
	u8 page,temp2;
	struct v4l2_subdev *sd = i2c_get_clientdata(client); 
	struct nx_vip_camera *cam = to_vip_cam(sd); /* cam->priv_data would be PageMap cache value */

	page = (reg >> 8);
	if (page == cam->priv_data)
		return 0;
	switch  (page) {
        case 0x00:
        case 0x02:
        case 0x10:
        case 0x11:
        case 0x12:
        case 0x13:
        case 0x14:
        case 0x15:
        case 0x16:
        case 0x17:
        case 0x18:
        case 0x20:
        case 0x22:
            /* These are all valid address */
            break;
        default:
		  return -EINVAL;
    }

	temp1 = HI161_PAGE_MAP;
	temp2 = page;
	ret = hi161_i2c_client_xfer(client, &temp1, (u8 *) & temp2, 1, 0);
	if (ret >= 0)
		cam->priv_data = page;
	return ret;
}

static int hi161_read_reg(struct i2c_client *client, u16  reg, u8 * val)
{
	int ret;
	u16 rval;

	ret = reg_page_map_set(reg);
	if (ret < 0) return ret;

	reg &= 0xFF;

	ret = hi161_i2c_client_xfer(client, (char *)&reg,(u8 *) &rval, 1, I2C_FLAG_READ);
	if (!ret) {
	    //printk("rval = %x\n", rval);
		*val = rval;
		return 0;
	}
	return ret;
}

static int hi161_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 temp1;
	int ret;

	temp1 = reg & 0xFF;
	ret = reg_page_map_set(client, reg);
	if (ret < 0) return ret;
	//printk("write reg %x val %x.\n", reg, val);
	return hi161_i2c_client_xfer(client, &temp1, 1,(u8 *) & val, 1, 0);
}


static int hi161_reg_set(struct i2c_client *client, const u16 reg, const u8 data)
{
	int ret;
	u8 val;

	ret = hi161_read_reg(client, reg, &val);
	if (ret >= 0)
		ret = hi161_write_reg(client, reg, val | data);
	return ret;
}

static int hi161_reg_clear(struct i2c_client *client, const u16 reg, const u8 data)
{
	int ret;
	u8 val;

	ret = hi161_read_reg(client, reg, &val);
	if (ret >= 0)
		ret = hi161_write_reg(client, reg, val & ~data);
	return ret;
}

static int hi161_setfmt_rgb565(struct i2c_client *client)
{
    u8 val;
    reg_read(ISPCTL1, &val);
    val &= ~0xF0;
	return reg_write(ISPCTL1, val | HI_161_ISPCTL1_RGB565);
}

static int hi161_setfmt_rgb444(struct i2c_client *client)
{
    u8 val;
    reg_read(ISPCTL1, &val);
    val &= ~0xF0;
	return reg_write(ISPCTL1, HI_161_ISPCTL1_RGB444);
}

static int hi161_setfmt_yuv(struct i2c_client *client, int swap_yuv_cb_cr, int swap_yuv_y_chromas)
{
    u8 val;
    reg_read(ISPCTL1, &val);
    val &= ~0xF3;
    if (swap_yuv_cb_cr)
        val |= HI_161_ISPCTL1_YPHASE_YUYV;
    if (swap_yuv_y_chromas)
        val |= HI_161_ISPCTL1_UPHASE_VYUY;
    val |= HI_161_ISPCTL1_YUV422;
	return reg_write(ISPCTL1, val);
}

static int hi161_enable(struct i2c_client *client)
{
	int ret;

	ret = reg_clear(PWRCTL, HI_161_PWRCTL_SLEEP);
	if (ret >= 0)
		hi161_powered = 1;
	return ret;
}

static int hi161_disable(struct i2c_client *client)
{
	int ret;

	ret = reg_set(PWRCTL, HI_161_PWRCTL_SLEEP);
	if (ret >= 0)
		hi161_powered = 0;
	return ret;
}

static int hi161_sensor_reset(struct i2c_client *client)
{
	int ret;

	ret = reg_set(PWRCTL, HI_161_PWRCTL_SOFT_RESET);
	if (ret >= 0)
		ret = reg_clear(PWRCTL, HI_161_PWRCTL_SOFT_RESET);
	return ret;
}

static int hi161_set_pixfmt(struct i2c_client *client, u32 pixfmt)
{
	int ret = 0;

	switch (pixfmt) {
	case V4L2_PIX_FMT_RGB565:
		ret = hi161_setfmt_rgb565(client);
		break;
	case V4L2_PIX_FMT_YUYV:
		ret = hi161_setfmt_yuv(client, 0, 0);
		break;
	default:
		//dev_err(&icd->dev, "Pixel format not handled : %x\n", pixfmt);
		ret = -EINVAL;
	}

	if (ret >= 0)
		hi161_pixfmt = pixfmt;

	return ret;
}

static int hi161_set_flip(struct i2c_client *client,int flip, int mask)
{
	u8 val;

	reg_read(VDOCTL2, &val);
	if (flip)
      val |=  mask;
    reg_write(VDOCTL2, val);

	return 0;
}

static int hi161_get_global_gain(struct i2c_client *client)
{
	unsigned int gain;

	gain = 0; /* Some gain value from registers, for what? */
	return gain;
}

static int hi161_set_global_gain(struct i2c_client *client,int gain)
{

	/* Set gain, for what ? */
	return 0;
}

int hi161_restore_state(struct i2c_client *client)
{
	hi161_set_pixfmt(client, hi161_pixfmt);
	hi161_set_flip(client, hi161_hflip, HI_161_VDOCTL2_X_FLIP);
	hi161_set_flip(client, hi161_vflip, HI_161_VDOCTL2_Y_FLIP);
	hi161_set_global_gain(client, hi161_gain);
	hi161_set_ae(client, hi161_autoexposure);
	return 0;
}

static int hi161_resume(struct i2c_client *client)
{
	int ret = 0;

	if (hi161_powered) {
		ret =  hi161_enable(client);
		if (ret >= 0) {
			hi161_sensor_reset(client);
			hi161_restore_state(client);
		}
	}
	return ret;
}

static int hi161_set_ITU656(struct i2c_client *client,int set)
{
    if (set) {
        reg_write(ITUEOL, 0x9D);
        reg_write(ITUSOL, 0x80);
        reg_write(ITUEOF, 0xB6);
        reg_write(ITUSOF, 0xAB);
        reg_write(ISPCTL1, 0x05); /* Enable ITU656 */
        reg_write(ITUCTL, 0x01); /* Enable SOF and EOF, Y,U/V 16~240 */
    } else {
        reg_write(ISPCTL1, 0x00); /* Disable ITU656 */
        reg_write(ITUCTL,  0x00); /* Disable SOF and EOF, Y,U/V 16~240 */
    }
    return 0;
}

static int hi161_set_resolution(struct i2c_client *client,enum nx_vip_cam_res_t nx_res)
{
    switch (nx_res) {
        case CAM_RES_VGA:
            reg_write(VDOCTL1, 0x10);
            reg_write(WINROWH, 0x00);
            reg_write(WINROWL, 0x2A);
            reg_write(WINCOLH, 0x00);
            reg_write(WINCOLL, 0x0C);
            reg_write(WINHGTH, 0x03);
            reg_write(WINHGTL, 0xC4); /* I need One mode line */
            reg_write(WINWIDH, 0x05);
            reg_write(WINWIDL, 0x00);
            break;
        case CAM_RES_SXGA:
            reg_write(VDOCTL1, 0x00);
            reg_write(WINROWH, 0x00);
            reg_write(WINROWL, 0x2A);
            reg_write(WINCOLH, 0x00);
            reg_write(WINCOLL, 0x0C);
            reg_write(WINHGTH, 0x03);
            reg_write(WINHGTL, 0xC4); /* I need One mode line */
            reg_write(WINWIDH, 0x05);
            reg_write(WINWIDL, 0x00);
            break;
        case CAM_RES_QVGA:
        default:
            /* Set to Preview mode as default */
            reg_write(VDOCTL1, 0x20);
            reg_write(WINROWH, 0x00);
            reg_write(WINROWL, 0x2A);
            reg_write(WINCOLH, 0x00);
            reg_write(WINCOLL, 0x0C);
            reg_write(WINHGTH, 0x03);
            reg_write(WINHGTL, 0xC4); /* I need One mode line */
            reg_write(WINWIDH, 0x05);
            reg_write(WINWIDL, 0x00);
    }
    return 0;
}

static int hi161_sensor_init(struct i2c_client *client)
{
	/* PLL should be set as OPCLK is greater than 12MHz */
	reg_write(PLLCTL1, 0x10 | 3 | 0x40); /* PLL enable */
	udelay(10);
	reg_write(PLLCTL1, 0x10 | 3 | 0x40 | 0x20); /* enable 2 */
	hi161_set_ae(hi161_autoexposure);
	hi161_set_awb(hi161_autowhitebalance);
	hi161_flicker_control(hi161_flicker);
    hi161_set_ITU656(1); /* Enable ITU-656 like mode */
    hi161_set_resolution(CAM_RES_QVGA);
    hi161_set_flip(hi161_hflip, HI_161_VDOCTL2_X_FLIP);
	hi161_set_flip(hi161_vflip, HI_161_VDOCTL2_Y_FLIP);

	return 0;
}

static int hi161_release(void)
{
	hi161_disable();

	return 0;
}

/*
 * Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one
 */
static int hi161_video_probe(struct i2c_client *client)
{
	u8 data;
	int ret;

	/*
	 * We must have a parent by now. And it cannot be a wrong one.
	 * So this entire test is completely redundant.
	 */

	ret = hi161_enable(client);
	if (ret)
		goto ei2c;
	ret = hi161_sensor_reset(client);
	if (ret)
		goto ei2c;

	reg_read(DEVID, &data);

	switch (data) {
	case 0x94:
		// this is chip we want
		break;
	default:
		ret = -ENODEV;
		//dev_err(&icd->dev,"No HI161 chip detected, register read %x\n", data);
		goto ei2c;
	}

	if (ret)
		goto eisis;

	hi161_autoexposure = 1;
	hi161_autowhitebalance = 1;
    hi161_hflip = HI_161_VDOCTL2_X_FLIP;
    hi161_vflip = HI_161_VDOCTL2_Y_FLIP;
	hi161_flicker = HI_161_FLICKER_AUTO_DETECTION;

	return 0;
eisis:
ei2c:
	return ret;
}

/*!
 * hi161 sensor set color configuration
 *
 * @param bright       int
 * @param saturation   int
 * @param red          int
 * @param green        int
 * @param blue         int
 * @return  None
 */
static void
hi161_set_color(struct i2c_client *client,int bright, int saturation, int red, int green, int blue)
{
	/*
	   bright = -1, and/or saturation = -1 should set Auto birghtness
	   and Auto Color saturation control,
	   otherwise, we should set birght, and saturation as manually.
	*/
	if (-1 == bright) {
	    reg_set(ISPCTL3, HI_161_ISPCTL3_AUTOBRIGHT);
	} else {
	    reg_clear(ISPCTL3, HI_161_ISPCTL3_AUTOBRIGHT);
	    reg_write(DYOFS, bright & 0x3F); /* ksw : or 0~100 to 0~63? */
	}
	if (-1 == saturation) {
	    reg_set(SATCTL, HI_161_SATCTL_AUTOCOLORSAT);
	    reg_clear(SATCTL, HI_161_SATCTL_MANCOLORSAT);
	} else {
	    reg_clear(SATCTL, HI_161_SATCTL_AUTOCOLORSAT);
	    reg_set(SATCTL, HI_161_SATCTL_MANCOLORSAT);
	    reg_write(SATB, blue);
	    reg_write(SATR, red);
	}
}

/*!
 * hi161 sensor get color configuration
 *
 * @param bright       int *
 * @param saturation   int *
 * @param red          int *
 * @param green        int *
 * @param blue         int *
 * @return  None
 */
static void
hi161_get_color(struct i2c_client *client,int *bright, int *saturation, int *red, int *green, int *blue)
{
}

/*!
 * hi161 sensor set AE measurement window mode configuration
 *
 * @param ae_mode      int
 * @return  None
 */
static void hi161_set_ae_mode(struct i2c_client *client,int ae_mode)
{

	if (ae_mode)
	    reg_set(AECTL1, HI_161_AECTL1_AE_ENABLE);
}

/*!
 * hi161 sensor get AE measurement window mode configuration
 *
 * @param ae_mode      int *
 * @return  None
 */
static void hi161_get_ae_mode(struct i2c_client *client,int *ae_mode)
{
	u8 data;

	reg_read(AECTL1, &data);
	*ae_mode = (data >> 7) & 0x01;
}

/*!
 * hi161 sensor enable/disable AE
 *
 * @param active      int
 * @return  None
 */
static void hi161_set_ae(struct i2c_client *client,int active)
{
	if (active) {
		reg_set(AECTL1, HI_161_AECTL1_AE_ENABLE);

	} else {
		reg_clear(AECTL1, HI_161_AECTL1_AE_ENABLE);
	}
	hi161_autoexposure = active;
}

/*!
 * hi161 sensor enable/disable auto white balance
 *
 * @param active      int
 * @return  None
 */
static void hi161_set_awb(struct i2c_client *client,int active)
{
	if (active) {
		reg_set(AWBCTL1, HI_161_AWBCTL1_AWB_ENABLE);
	} else {
		reg_clear(AWBCTL1, HI_161_AWBCTL1_AWB_ENABLE);
	}
	hi161_autowhitebalance = active;
}

/*!
 * hi161 sensor set the flicker control
 *
 * @param control      int
 * @return  None
 */
static void hi161_flicker_control(struct i2c_client *client,int control)
{
	u8	data=0;

	reg_read(AECTL1, &data);

	switch (control) {
	case HI_161_FLICKER_DISABLE:
		data = 0;
		break;

	case HI_161_FLICKER_MANUAL_50:
		data = HI_161_AECTL1_100HZ;
		break;

	case HI_161_FLICKER_MANUAL_60:
		data = HI_161_AECTL1_120HZ;
		break;

	case HI_161_FLICKER_AUTO_DETECTION:
		data = 0;
		break;
	}

	if (control == HI_161_FLICKER_AUTO_DETECTION) {
		reg_set(AECTL1, HI_161_AECTL1_ANTIFLKR_ENABLE);
	} else {
		reg_clear(AECTL1, HI_161_AECTL1_ANTIFLKR_ENABLE);
	    reg_set(AECTL1, data);
	}
	hi161_flicker = control;

	return;
}
/*!
 * hi161 Get mode&flicker control parameters
 *
 * @return  None
 */
static void hi161_get_control_params(struct i2c_client *client,int *ae, int *awb, int *flicker)
{
	if ((ae != NULL) && (awb != NULL) && (flicker != NULL)) {
		*ae = hi161_autoexposure;
		*awb = hi161_autowhitebalance;
		*flicker = hi161_flicker;
		if (*flicker) {
			switch (*flicker) {
			case 1:
				*flicker = HI_161_FLICKER_MANUAL_50;
				break;
			case 2:
				*flicker = HI_161_FLICKER_MANUAL_60;
				break;
			default:
				*flicker = HI_161_FLICKER_AUTO_DETECTION;
				break;
			}
		} else
			*flicker = HI_161_FLICKER_DISABLE;
	}
}

/*!
 * hi161 get_i2c function
 *
 * @return  none
 */
int hi161_get_i2c(struct i2c_client *client,int addr, int *val)
{
    //printk("hi161 get i2c!, addr = 0x%x\n", addr);
	return hi161_read_reg(client, addr, (u8 *)val);
	//printk("hi161 get i2c!, val = 0x%x\n", *val);
}

/*!
 * hi161 set_i2c function
 *
 * @return  none
 */
int hi161_set_i2c(struct i2c_client *client,int addr, int val)
{
    //printk("hi161 set i2c!, addr = 0x%x val = 0x%x\n", addr, val);
	return hi161_write_reg(client, addr, val);
}

static struct i2c_driver hi161_i2c_driver;

static void hi161_start(struct i2c_client *client)
{
	/* Initialize data */
	hi161_sensor_init(client);
}

static int __devinit hi161_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct nx_vip_camera *info;
	int ret;

    printk("Hi161_i2c_probe client->addr=%X\n", client->addr);
    /* Check I2C address that the device realy exist */
    //if (!i2c_check_functionality(client->adapter,
    //                 I2C_FUNC_SMBUS_BYTE_DATA))
    //    return -EIO;

    info = kzalloc(sizeof(struct nx_vip_camera), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &hi161_ops);

	/* Make sure it's an poa030 */
	ret = hi161_video_probe(client);
	if (ret) {
		v4l_dbg(1, debug, client,
			"chip found @ 0x%x (%s) is not a poa030 chip.\n",
			client->addr << 1, client->adapter->name);
		kfree(info);
		return ret;
	}
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);
	

	if (strstr(id->name, "_2"))
		info->id = 1;
	else
		info->id = 0;

	printk("HI161 name = %s, id=%d\n", id->name, info->id);
	
	info->client = client;
	info->i2c_read = hi161_get_i2c;
	info->i2c_write = hi161_set_i2c;

	info->order422 = CAM_ORDER422_8BIT_CBYCRY; //CAM_ORDER422_8BIT_YCBYCR
	info->cur_width = 320;
	info->cur_height = 240;
	info->def_width = 320;
	info->def_height = 240;
	info->max_width = 640;
	info->max_height = 480;
	info->cur_pixformat = V4L2_PIX_FMT_YUYV;
	info->bpp = 2;
	info->polarity.vsync = 1;
	info->cam_id = 140;

	nx_vip_register_subdev(sd);
	
	hi161_reset(sd, 0);
	udelay(1000); /* wait some time */
	hi161_start(client);
	info->initialized = 1;
	info->priv_data = -1; /* page map value */

	return 0;
}

static int __devexit hi161_remove(struct i2c_client *client)
{
    struct i2c_hi161_data *phi161_data = i2c_get_clientdata(client);

	kfree(phi161_data);

    return  0;
}


static int hi161_change_resolution(struct i2c_client *client, int res)
{
    int out_width;
    /*
      HI161 is VGA camera with 2 sub sampling feature.
     */
    if (res == CAM_RES_QVGA ||
        res == CAM_RES_QQVGA ||
        res == CAM_RES_VGA ||
        res == CAM_RES_DEFAULT) {
     } else {
         err("Invalid resolution\n");
         return -1;
     }

	switch (res) {
	case CAM_RES_QVGA:
	case CAM_RES_QQVGA:
	case CAM_RES_VGA:
	case CAM_RES_DEFAULT:	/* fall through */
		/* Set camera resolution */
		if (res == CAM_RES_VGA) {
		    out_width = 640;
		    out_width = 480;
		    reg_write(VDOCTL1, HI_161_VDOCTL1_SUB_VGA);
		} else
		if (res == CAM_RES_QQVGA) {
		    out_width = 160;
		    out_width = 120;
		    reg_write(VDOCTL1, HI_161_VDOCTL1_SUB_QQVGA);
		} else { /* QVGA is DEFAULT */
		    out_width = 320;
		    out_width = 240;
		    reg_write(VDOCTL1, HI_161_VDOCTL1_SUB_QVGA);
		}
		break;
	}

	return 0;
}

static int hi161_change_whitebalance(struct i2c_client *client, enum nx_vip_wb_t type)
{
	// Some registers or just auto wb or manual wb
	if (type == WB_AUTO)
	    hi161_set_awb(true);
	else
	    hi161_set_awb(false);

	return 0;
}

static int hi161_set_brightness(struct i2c_client *client, int val)
{
    int maxval = 0x7F;
    int adj;

    maxval = maxval * val / 100;

	reg_write(YOFS, maxval);

	return 0;
}

static int hi161_set_power_save(struct i2c_client *client, int val)
{
	u8	data=0;

	reg_read(PWRCTL, &data);

    if (val & 0x01)
	    reg_write(PWRCTL, data | 0x01);
	else
	    reg_write(PWRCTL, data | 0xFE);

	return 0;
}

static int hi161_command(struct i2c_client *client, u32 cmd, void *arg)
{
	switch (cmd) {
	case I2C_CAM_INIT:
		hi161_start(client);
		info("hi161 : external camera initialized\n");
		break;

	case I2C_CAM_RESOLUTION:
		hi161_change_resolution(client, (int) arg);
		break;

	case I2C_CAM_WB:
		hi161_change_whitebalance(client, (enum nx_vip_wb_t) arg);
        	break;

    case I2C_CAM_BRIGHTNESS:
        hi161_set_brightness(client, (int) arg);
        break;

    case I2C_CAM_POWER_SAVE:
        hi161_set_power_save(client, (int) arg);
        break;

	default:
		err("unexpect command\n");
		break;
	}

	return 0;
}

#if defined(CONFIG_CAMERA1_HI161)
static const struct i2c_device_id hi161_id[] = {
    { "hi161", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, hi161_id);

static struct i2c_driver hi161_i2c_driver = {
	.driver = {
		.name = "HI161",
		.owner = THIS_MODULE,
	},
	.id_table     = hi161_id,
	.probe        = hi161_probe,
    .remove       = __devexit_p(hi161_remove),
	.command = hi161_command,
};
#endif

#if defined(CONFIG_CAMERA2_HI161)
static const struct i2c_device_id hi161_id2[] = {
	{ "hi161_2", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hi161_id2);

static struct i2c_driver hi161_i2c_driver2 = {
	.driver = {
		.name = "HI161_2",
		.owner = THIS_MODULE,
	},
	.id_table     = hi161_id2,
	.probe        = hi161_probe,
    .remove       = __devexit_p(hi161_remove),
	.command      = hi161_command,
};
#endif

static __init int hi161_init(void)
{
	int res;
	
    printk("hi161 init\n");
#if defined(CONFIG_CAMERA1_HI161)
	res = i2c_add_driver(&hi161_i2c_driver);
#endif
#if defined(CONFIG_CAMERA2_HI161)
	res = i2c_add_driver(&hi161_i2c_driver2);
#endif
	return res;
}

static __init void hi161_clean(void)
{
#if defined(CONFIG_CAMERA1_HI161)
	i2c_del_driver(&hi161_i2c_driver);
#endif
#if defined(CONFIG_CAMERA2_HI161)
	i2c_del_driver(&hi161_i2c_driver2);
#endif
}

module_init(hi161_init);
module_exit(hi161_clean);

MODULE_DESCRIPTION("Hynix HI161 Camera driver");
MODULE_AUTHOR("Seungwoo Kim<ksw@stcube.com>");
MODULE_LICENSE("GPL");

