/*! linux/drivers/media/video/samsung/poa030r.c
 *
 * PixelPlus POA030 CMOS Image Sensor driver
 * Copyright(c) 2014 STcube Inc.,
 * All right reserved by Seungwoo Kim <ksw@stcube.com>
 *
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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

#include <mach/devices.h>
#include <media/v4l2-common.h>
#include <media/soc_camera.h>

#include "nx_vip.h"
#include "poa030r.h"

static int debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240

struct regval_list {
	unsigned short int reg_num;
	unsigned short int value;
};

struct regval_list poa030_def_regs[] = {
	/* First, PAD control should be set */
	{POA030_PAD_CONTROL, 0x00}, /* Standby data hiz, normal not hiz */	
	{POA030_PAD_CONTROL2,	0x7F}, /* Clear Bit7 -> Disable OSC pad? */
	{POA030_BAYER_CONTROL_01, 0x07}, /* Mirror control : none */
	{POA030_FORMAT, 0x00}, /* Format control : Y only */ /* 0: CbYCrY, 1: CrYCbY, 2:YCbYCr, 3:YCrYCb */
	{POA030_SYNC_CONTROL_1, 0x00}, /* Polarity : none */
	{POA030_AUTO_CONTROL_1, 0x98}, /* Auto : AWB/AE enable */
	/* QVGA */
	{POA030_WINDOWX1_L, 0x03},
	{POA030_WINDOWY1_L, 0x03},
	{POA030_WINDOWX2_H, 0x01},
	{POA030_WINDOWX2_L, 0x42},
	{POA030_WINDOWY2_H, 0x00},
	{POA030_WINDOWY2_L, 0xF2},
	{POA030_SCALE_X,	 0x40},
	{POA030_SCALE_Y,	 0x40},
	{0x195,			 0x01},
	{0x196,			 0x40},
	{POA030_AE_FWX1_H,	 0x00},
	{POA030_AE_FWX1_L,	 0x03},
	{POA030_AE_FWX2_H,	 0x01},
	{POA030_AE_FWX2_L,	 0x42},
	{POA030_AE_FWY1_H,	 0x00},
	{POA030_AE_FWY1_L,	 0x03},
	{POA030_AE_FWY2_H,	 0x00},
	{POA030_AE_FWY2_L,	 0xF2},
	{POA030_AE_CWX1_H,	 0x00},
	{POA030_AE_CWX1_L,	 0x6D},
	{POA030_AE_CWX2_H,	 0x00},
	{POA030_AE_CWX2_L,	 0xD8},
	{POA030_AE_CWY1_H,	 0x00},
	{POA030_AE_CWY1_L,	 0x53},
	{POA030_AE_CWY2_H,	 0x00},
	{POA030_AE_CWY2_L,	 0xA2},
	/* Now Set CCIR 656 */
	{POA030_SYNC_BLANKSAV, 0xAB}, // 0xAB
	{POA030_SYNC_BLANKEAV, 0xB6}, // 0xB6
	{POA030_SYNC_ACTIVSAV, 0x80},
	{POA030_SYNC_ACTIVEAV, 0x9D},
};

struct regval_list poa030_fmt_yuv422[] = {
	{POA030_FORMAT, 0},
	{POA030_CS_MAX, 0x7F},
	{POA030_Y_CONTRAST, 0x40},
	{POA030_Y_BRIGHTNESS, 0x01},
	{POA030_Y_MAX, 0xFE},
	{POA030_SYNC_CONTROL_0, 0x00},
};

struct regval_list poa030_fmt_rgb444[] = {
	{POA030_FORMAT, 0x30},
	{POA030_CS_MAX, 0x7F},
	{POA030_Y_CONTRAST, 0x40},
	{POA030_Y_BRIGHTNESS, 0x00},
	{POA030_Y_MAX, 0xFE},
	{POA030_SYNC_CONTROL_0, 0x00},
};

struct regval_list poa030_fmt_rgb565[] = {
	{POA030_FORMAT, 0x33},
	{POA030_CS_MAX, 0x7F},
	{POA030_Y_CONTRAST, 0x40},
	{POA030_Y_BRIGHTNESS, 0x00},
	{POA030_Y_MAX, 0xFE},
	{POA030_SYNC_CONTROL_0, 0x00},
};


struct regval_list poa030_fmt_raw[] = {
	{POA030_FORMAT, 0x10},
	{POA030_CS_MAX, 0x7F},
	{POA030_Y_CONTRAST, 0x40},
	{POA030_Y_BRIGHTNESS, 0x00},
	{POA030_Y_MAX, 0xFE},
	{0x0058, 0x00},
	{POA030_SYNC_CONTROL_0, 0x01},
};

/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct poa030_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	struct regval_list *regs;
	int nlist;
	int bpp;   /* Bytes per pixel */
} poa030_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.regs 		= poa030_fmt_yuv422,
		.nlist		= ARRAY_SIZE(poa030_fmt_yuv422),
		.bpp		= 2,
	},
	{
		.desc		= "RGB 444",
		.pixelformat	= V4L2_PIX_FMT_RGB444,
		.regs		= poa030_fmt_rgb444,
		.nlist		= ARRAY_SIZE(poa030_fmt_rgb444),
		.bpp		= 2,
	},
	{
		.desc		= "RGB 565",
		.pixelformat	= V4L2_PIX_FMT_RGB565,
		.regs		= poa030_fmt_rgb565,
		.nlist		= ARRAY_SIZE(poa030_fmt_rgb565),
		.bpp		= 2,
	},
	{
		.desc		= "Raw RGB Bayer",
		.pixelformat	= V4L2_PIX_FMT_SBGGR8,
		.regs 		= poa030_fmt_raw,
		.nlist		= ARRAY_SIZE(poa030_fmt_raw),		
		.bpp		= 1
	},
};
#define N_POA030_FMTS ARRAY_SIZE(poa030_formats)

static struct regval_list vga_reg_vals[] = {
	{POA030_WINDOWX1_L, 0x05},
	{POA030_WINDOWY1_L, 0x05},
	{POA030_WINDOWX2_H, 0x02},
	{POA030_WINDOWX2_L, 0x84},
	{POA030_WINDOWY2_H, 0x01},
	{POA030_WINDOWY2_L, 0xE4},
	{POA030_SCALE_X,	 0x20},
	{POA030_SCALE_Y,	 0x20},
	{0x195,			 0x00},
	{0x196,			 0x0A},
	{POA030_AE_FWX1_H,	 0x00},
	{POA030_AE_FWX1_L,	 0x05},
	{POA030_AE_FWX2_H,	 0x02},
	{POA030_AE_FWX2_L,	 0x84},
	{POA030_AE_FWY1_H,	 0x00},
	{POA030_AE_FWY1_L,	 0x05},
	{POA030_AE_FWY2_H,	 0x01},
	{POA030_AE_FWY2_L,	 0xE4},
	{POA030_AE_CWX1_H,	 0x00},
	{POA030_AE_CWX1_L,	 0xDA},
	{POA030_AE_CWX2_H,	 0x01},
	{POA030_AE_CWX2_L,	 0xAF},
	{POA030_AE_CWY1_H,	 0x00},
	{POA030_AE_CWY1_L,	 0xA5},
	{POA030_AE_CWY2_H,	 0x01},
	{POA030_AE_CWY2_L,	 0x44},
};

static struct regval_list qvga_reg_vals[] = {
	{POA030_WINDOWX1_L, 0x03},
	{POA030_WINDOWY1_L, 0x03},
	{POA030_WINDOWX2_H, 0x01},
	{POA030_WINDOWX2_L, 0x42},
	{POA030_WINDOWY2_H, 0x00},
	{POA030_WINDOWY2_L, 0xF2},
	{POA030_SCALE_X,	 0x40},
	{POA030_SCALE_Y,	 0x40},
	{0x195,			 0x01},
	{0x196,			 0x40},
	{POA030_AE_FWX1_H,	 0x00},
	{POA030_AE_FWX1_L,	 0x03},
	{POA030_AE_FWX2_H,	 0x01},
	{POA030_AE_FWX2_L,	 0x42},
	{POA030_AE_FWY1_H,	 0x00},
	{POA030_AE_FWY1_L,	 0x03},
	{POA030_AE_FWY2_H,	 0x00},
	{POA030_AE_FWY2_L,	 0xF2},
	{POA030_AE_CWX1_H,	 0x00},
	{POA030_AE_CWX1_L,	 0x6D},
	{POA030_AE_CWX2_H,	 0x00},
	{POA030_AE_CWX2_L,	 0xD8},
	{POA030_AE_CWY1_H,	 0x00},
	{POA030_AE_CWY1_L,	 0x53},
	{POA030_AE_CWY2_H,	 0x00},
	{POA030_AE_CWY2_L,	 0xA2},
};

static struct poa030_win_size {
	int	width;
	int	height;
	struct regval_list *regs; /* Regs to tweak */
	int nlist;
/* h/vref stuff */
} poa030_win_sizes[] = {
	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.regs 		= vga_reg_vals,
		.nlist		= ARRAY_SIZE(vga_reg_vals),
	},
	/* QVGA */
	{
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
		.regs 		= qvga_reg_vals,
		.nlist		= ARRAY_SIZE(qvga_reg_vals),
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(poa030_win_sizes))

#define I2C_FLAG_READ	0x10

static int poa030_i2c_xfer(struct i2c_client *client, int reg, char *buf, int num, int tran_flag)
{
	struct i2c_msg msg[2];
	int ret;
	char xbuf[128];

	if (tran_flag & I2C_FLAG_READ) {
		msg[0].addr = client->addr;
		msg[0].len = 1;
		msg[0].buf = (char *)&reg;
		msg[0].flags = 0;

		msg[1].addr = client->addr;
		msg[1].len = num;
		msg[1].buf = buf;
		msg[1].flags = I2C_M_RD;

		ret = i2c_transfer(client->adapter, msg, 2);
	} else {
		xbuf[0] = reg;
		memcpy(&xbuf[1], buf, num);
		msg[0].addr = client->addr;
		msg[0].len = 1 + num;
		msg[0].buf = xbuf;
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
	u8 page;
	struct v4l2_subdev *sd = i2c_get_clientdata(client); 
	struct nx_vip_camera *cam = to_vip_cam(sd);
	/* cam->priv_data would be PageMap cache value */

	page = (reg >> 8);
	if (page == cam->priv_data)
		return 0;
	if (page > 3)
		return -EINVAL;

	temp1 = POA030_BANK;

	ret = poa030_i2c_xfer(client, temp1, (u8 *) & page, 1, 0);
	if (ret >= 0)
		cam->priv_data = page;
	return ret;
}

static int poa030_read_reg8(struct i2c_client *client, u16	reg, u8 *val)
{
	int ret;
	u8 rval;

	ret = reg_page_map_set(client, reg);
	if (ret < 0) return ret;

	reg &= 0xFF;

	ret = poa030_i2c_xfer(client, reg, (u8 *) &rval, 1, I2C_FLAG_READ);
	if (0 == ret) {
		*val = rval;
		return 0;
	}
	return ret;
}

static int poa030_write_reg8(struct i2c_client *client,u16 reg, u8 val)
{
	u8 temp1;
	int ret;

	temp1 = reg & 0xFF;
	ret = reg_page_map_set(client, reg);
	if (ret < 0) return ret;
	//printk("write reg %x val %x.\n", reg, val);
	return poa030_i2c_xfer(client, temp1, (u8 *) & val, 1, 0);
}

static int poa030_write_reg8_array(struct i2c_client *client, struct regval_list *regs, int n)
{
	int i, ret;

	for (i=0; i<n; i++, regs++) {
		ret = poa030_write_reg8(client, regs->reg_num, regs->value);
		if (ret) break;
	}
	return ret;
}


static void poa030_start(struct i2c_client *client)
{
	u8 val;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct nx_vip_camera *cam = to_vip_cam(sd);

	/* First, PAD control should be set */
	poa030_read_reg8(client, POA030_PAD_CONTROL2, &val);
	printk("pad_control2=%x\n", val);
	poa030_write_reg8(client, POA030_PAD_CONTROL, 0x00); /* Standby data hiz, normal not hiz */
	poa030_write_reg8(client, POA030_PAD_CONTROL2,	val & 0x7F); /* Clear Bit7 -> Disable OSC pad? */
	printk("pad_control2 again=%x\n", val);
	poa030_write_reg8(client, POA030_BAYER_CONTROL_01, 0x07); /* Mirror control : none */
	poa030_write_reg8(client, POA030_FORMAT, 0x00); /* Format control : Y only */ /* 0: CbYCrY, 1: CrYCbY, 2:YCbYCr, 3:YCrYCb */
	poa030_write_reg8(client, POA030_SYNC_CONTROL_1, 0x00); /* Polarity : none */
	poa030_write_reg8(client, POA030_AUTO_CONTROL_1, 0x98); /* Auto : AWB/AE enable */
	/* Now set VGA/QCGA*/
	/* VGA */ /*
	poa030_write_reg8(POA030_WINDOWX1_L, 0x05);
	poa030_write_reg8(POA030_WINDOWY1_L, 0x05);
	poa030_write_reg8(POA030_WINDOWX2_H, 0x02);
	poa030_write_reg8(POA030_WINDOWX2_L, 0x84);
	poa030_write_reg8(POA030_WINDOWY2_H, 0x01);
	poa030_write_reg8(POA030_WINDOWY2_L, 0xE4);
	poa030_write_reg8(POA030_SCALE_X,	 0x20);
	poa030_write_reg8(POA030_SCALE_Y,	 0x20);
	poa030_write_reg8(0x195,			 0x00);
	poa030_write_reg8(0x196,			 0x0A);
	poa030_write_reg8(POA030_AE_FWX1_H,	 0x00);
	poa030_write_reg8(POA030_AE_FWX1_L,	 0x05);
	poa030_write_reg8(POA030_AE_FWX2_H,	 0x02);
	poa030_write_reg8(POA030_AE_FWX2_L,	 0x84);
	poa030_write_reg8(POA030_AE_FWY1_H,	 0x00);
	poa030_write_reg8(POA030_AE_FWY1_L,	 0x05);
	poa030_write_reg8(POA030_AE_FWY2_H,	 0x01);
	poa030_write_reg8(POA030_AE_FWY2_L,	 0xE4);
	poa030_write_reg8(POA030_AE_CWX1_H,	 0x00);
	poa030_write_reg8(POA030_AE_CWX1_L,	 0xDA);
	poa030_write_reg8(POA030_AE_CWX2_H,	 0x01);
	poa030_write_reg8(POA030_AE_CWX2_L,	 0xAF);
	poa030_write_reg8(POA030_AE_CWY1_H,	 0x00);
	poa030_write_reg8(POA030_AE_CWY1_L,	 0xA5);
	poa030_write_reg8(POA030_AE_CWY2_H,	 0x01);
	poa030_write_reg8(POA030_AE_CWY2_L,	 0x44);
	*/
	/* QVGA */
	poa030_write_reg8(client, POA030_WINDOWX1_L, 0x03);
	poa030_write_reg8(client, POA030_WINDOWY1_L, 0x03);
	poa030_write_reg8(client, POA030_WINDOWX2_H, 0x01);
	poa030_write_reg8(client, POA030_WINDOWX2_L, 0x42);
	poa030_write_reg8(client, POA030_WINDOWY2_H, 0x00);
	poa030_write_reg8(client, POA030_WINDOWY2_L, 0xF2);
	poa030_write_reg8(client, POA030_SCALE_X,	 0x40);
	poa030_write_reg8(client, POA030_SCALE_Y,	 0x40);
	poa030_write_reg8(client, 0x195,			 0x01);
	poa030_write_reg8(client, 0x196,			 0x40);
	poa030_write_reg8(client, POA030_AE_FWX1_H,	 0x00);
	poa030_write_reg8(client, POA030_AE_FWX1_L,	 0x03);
	poa030_write_reg8(client, POA030_AE_FWX2_H,	 0x01);
	poa030_write_reg8(client, POA030_AE_FWX2_L,	 0x42);
	poa030_write_reg8(client, POA030_AE_FWY1_H,	 0x00);
	poa030_write_reg8(client, POA030_AE_FWY1_L,	 0x03);
	poa030_write_reg8(client, POA030_AE_FWY2_H,	 0x00);
	poa030_write_reg8(client, POA030_AE_FWY2_L,	 0xF2);
	poa030_write_reg8(client, POA030_AE_CWX1_H,	 0x00);
	poa030_write_reg8(client, POA030_AE_CWX1_L,	 0x6D);
	poa030_write_reg8(client, POA030_AE_CWX2_H,	 0x00);
	poa030_write_reg8(client, POA030_AE_CWX2_L,	 0xD8);
	poa030_write_reg8(client, POA030_AE_CWY1_H,	 0x00);
	poa030_write_reg8(client, POA030_AE_CWY1_L,	 0x53);
	poa030_write_reg8(client, POA030_AE_CWY2_H,	 0x00);
	poa030_write_reg8(client, POA030_AE_CWY2_L,	 0xA2);
	/* Now Set CCIR 656 */
	poa030_write_reg8(client, POA030_SYNC_BLANKSAV, 0xAB); // 0xAB
	poa030_write_reg8(client, POA030_SYNC_BLANKEAV, 0xB6); // 0xB6
	poa030_write_reg8(client, POA030_SYNC_ACTIVSAV, 0x80);
	poa030_write_reg8(client, POA030_SYNC_ACTIVEAV, 0x9D);

	if (cam->id == 0)
	{
		poa030_read_reg8(client, POA030_AUTO_CONTROL_1, &val);
		poa030_write_reg8(client, POA030_AUTO_CONTROL_1, val | 0x02);

		poa030_write_reg8(client, POA030_EXT_INTTIME_M, 0x80);
	}
	else
	{
#if defined(CONFIG_MODEL_RK_HIT_N)
		poa030_write_reg8(client, POA030_EDGE_GAIN, 0x30); // Sahrpness
		poa030_write_reg8(client, 0x005F, 0x1C); // Pad Drivability ( 0x1C , 0x1D , 0xx1E , 0x1F )
		poa030_write_reg8(client, 0x0060, 0x40); // PCLK Drivability ( 0x00 , 0x40 , 0x80 , 0xC0 )
		poa030_write_reg8(client, POA030_CLKDIV, 0x03); // PCLK divider 1/3
#endif
	}

#if 0
	for (i=0; i<255; i++) {
		poa030_read_reg8(client, i + 0x0000, &val);
		printk("page A: %x => %x\n", i, val);
	}
	for (i=0; i<255; i++) {
		poa030_read_reg8(client, i + 0x0100, &val);
		printk("page B: %x => %x\n", i, val);
	}
	for (i=0; i<255; i++) {
		poa030_read_reg8(client, i + 0x0200, &val);
		printk("page C: %x => %x\n", i, val);
	}
#endif
}

/*!
 * poa030 get_i2c function
 *
 * @return	none
 */
static int poa030_get_i2c(struct i2c_client *client, int addr, int *val)
{
	//printk("poa030 get i2c!, addr = 0x%x\n", addr);
	return poa030_read_reg8(client, addr, (u8 *)val);
	//printk("poa030 get i2c!, val = 0x%x\n", *val);
}

/*!
 * poa030 set_i2c function
 *
 * @return	none
 */
static int poa030_set_i2c(struct i2c_client *client, int addr, int val)
{
	//printk("poa030 set i2c!, addr = 0x%x val = 0x%x\n", addr, val);
	return poa030_write_reg8(client, addr, val);
}
/*
 * Stuff that knows about the sensor.
 */
static int poa030_reset(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	poa030_write_reg8(client, POA030_SOFTRESET, 0x01); /* Software reset */
	udelay(1000); /* wait some time */
	poa030_write_reg8(client, POA030_SOFTRESET, 0x00); /* Clear software reset */

	return 0;
}

static int poa030_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	poa030_start(client);

	return 0;
}

/*!
 * poa030 detect
 *
 * @return 0(OK) or -NODEV
 */
static int poa030_detect(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val;
	int count = 5;
	int ret;
	
	/* read id register and compare to correct value.. */
	ret = poa030_read_reg8(client, POA030_DEVICEID_H, &val);
#if 0 // dose not retry
	/* if EGAIN try n times more... */
	while ((-EAGAIN == ret) && count > 0) {
		ret = poa030_read_reg8(client, POA030_DEVICEID_H, &val);
		if (0==ret) break;
		count--;
	}
#endif
	if (ret)
		return ret;

	if (val != 0x00A0)
		return -ENODEV;
	
	ret = poa030_read_reg8(client, POA030_DEVICEID_L, &val);
	if (ret)
		return ret;

	if (val != 0x0030)
		return -ENODEV;
	
	return 0;
}

/*!
 * enum_fmt, try_fmt
 * We only support YUV422, 320X240 & 640X480 only.
 *
 * No other modes are tested. So return ERROR if non supported mode specified.
 *
 */
 
static int poa030_enum_fmt(struct v4l2_subdev *sd, struct v4l2_fmtdesc *fmt)
{
	struct poa030_format_struct *ofmt;

	if (fmt->index >= N_POA030_FMTS)
		return -EINVAL;

	ofmt = poa030_formats + fmt->index;
	fmt->flags = 0;
	strcpy(fmt->description, ofmt->desc);
	fmt->pixelformat = ofmt->pixelformat;

	return 0;
}


static int poa030_try_fmt_internal(struct v4l2_subdev *sd,
		struct v4l2_format *fmt,
		struct poa030_format_struct **ret_fmt,
		struct poa030_win_size **ret_wsize)
{
	struct nx_vip_camera *cam = to_vip_cam(sd);
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (cam->cur_pixformat != pix->pixelformat) {
		int index;
		for (index = 0; index < N_POA030_FMTS; index++)
			if (poa030_formats[index].pixelformat == pix->pixelformat)
				break;
		if (index >= N_POA030_FMTS) {
			/* default to first format */
			index = 0;
		}
		pix->pixelformat = poa030_formats[index].pixelformat;
		poa030_write_reg8_array(client, poa030_formats[index].regs, poa030_formats[index].nlist);
		cam->cur_pixformat = pix->pixelformat;
		cam->bpp = poa030_formats[index].bpp;
		if (ret_fmt != NULL)
			*ret_fmt = poa030_formats + index;
	}
	/*
	 * Fields: the PixelPlus devices claim to be progressive.
	 */
	pix->field = V4L2_FIELD_NONE;
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	if ((cam->cur_width != pix->width) || 
		(cam->cur_height != pix->height)) {
		struct poa030_win_size *wsize;

		printk("poa030 winsize hack\n");
		for (wsize = poa030_win_sizes; wsize < poa030_win_sizes + N_WIN_SIZES;
			 wsize++)
			if (pix->width >= wsize->width && pix->height >= wsize->height)
				break;
		if (wsize >= poa030_win_sizes + N_WIN_SIZES)
			wsize--;   /* Take the smallest one */
		printk("wsize = %x, poa030 winsize=%x\n", wsize, poa030_win_sizes);
		printk("wsize nlist=%d\n", wsize->nlist);
		poa030_write_reg8_array(client, wsize->regs, wsize->nlist);
		if (ret_wsize != NULL)
			*ret_wsize = wsize;
		
		/*
		 * Note the size we'll actually handle.
		 */
		printk("wsize w:h=%d:%d\n", wsize->width, wsize->height);
		pix->width = wsize->width;
		pix->height = wsize->height;
		pix->bytesperline = pix->width*cam->bpp;
		pix->sizeimage = pix->height*pix->bytesperline;
		// Now set cam's parameter as is
		cam->cur_width = pix->width;
		cam->cur_height = pix->height;
	}

	return 0;
}

static int poa030_try_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	return poa030_try_fmt_internal(sd, fmt, NULL, NULL);
}

/*
 * Set a format.
 */
static int poa030_s_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int ret;
	struct poa030_format_struct *ovfmt;
	struct poa030_win_size *wsize;

	ret = poa030_try_fmt_internal(sd, fmt, &ovfmt, &wsize);
	if (ret)
		return ret;

	return ret;
}

/*
 * Implement G/S_PARM.	There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int poa030_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	return 0;
}

static int poa030_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	return 0;
}


/*
 * Code for dealing with controls.
 */
static int poa030_s_sat(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int poa030_g_sat(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


static int poa030_s_hue(struct v4l2_subdev *sd, int value)
{
	return 0;
}


static int poa030_g_hue(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int poa030_s_brightness(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int poa030_g_brightness(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int poa030_s_contrast(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int poa030_g_contrast(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int poa030_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


static int poa030_s_hflip(struct v4l2_subdev *sd, int value)
{
	return 0;
}


static int poa030_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


static int poa030_s_vflip(struct v4l2_subdev *sd, int value)
{
	return 0;
}

static int poa030_queryctrl(struct v4l2_subdev *sd,
		struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls. */
	switch (qc->id) {
	case V4L2_CID_BRIGHTNESS:
		return v4l2_ctrl_query_fill(qc, 0, 255, 1, 128);
	case V4L2_CID_CONTRAST:
		return v4l2_ctrl_query_fill(qc, 0, 127, 1, 64);
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_SATURATION:
		return v4l2_ctrl_query_fill(qc, 0, 256, 1, 128);
	case V4L2_CID_HUE:
		return v4l2_ctrl_query_fill(qc, -180, 180, 5, 0);
	}
	return -EINVAL;
}

static int poa030_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return poa030_g_brightness(sd, &ctrl->value);
	case V4L2_CID_CONTRAST:
		return poa030_g_contrast(sd, &ctrl->value);
	case V4L2_CID_SATURATION:
		return poa030_g_sat(sd, &ctrl->value);
	case V4L2_CID_HUE:
		return poa030_g_hue(sd, &ctrl->value);
	case V4L2_CID_VFLIP:
		return poa030_g_vflip(sd, &ctrl->value);
	case V4L2_CID_HFLIP:
		return poa030_g_hflip(sd, &ctrl->value);
	}
	return -EINVAL;
}

static int poa030_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return poa030_s_brightness(sd, ctrl->value);
	case V4L2_CID_CONTRAST:
		return poa030_s_contrast(sd, ctrl->value);
	case V4L2_CID_SATURATION:
		return poa030_s_sat(sd, ctrl->value);
	case V4L2_CID_HUE:
		return poa030_s_hue(sd, ctrl->value);
	case V4L2_CID_VFLIP:
		return poa030_s_vflip(sd, ctrl->value);
	case V4L2_CID_HFLIP:
		return poa030_s_hflip(sd, ctrl->value);
	}
	return -EINVAL;
}

static int poa030_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
	//struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* POA030 is not registered for V4L2-chip-ident... */
	poa030_detect(sd);
	
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int poa030_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val = 0;
	int ret;

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	ret = poa030_read_reg8(client, reg->reg & 0xff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int poa030_s_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	poa030_write_reg8(client, reg->reg & 0xff, reg->val & 0xff);
	return 0;
}
#endif

static int poa030_set_power(struct v4l2_subdev *subdev, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(subdev);
	int ret = 0;

	PM_DBGOUT("%s: on=%d\n", __func__, on);
}
/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops poa030_core_ops = {
	.g_chip_ident = poa030_g_chip_ident,
	.g_ctrl = poa030_g_ctrl,
	.s_ctrl = poa030_s_ctrl,
	.queryctrl = poa030_queryctrl,
	.s_power = poa030_set_power,
	.reset = poa030_reset,
	.init = poa030_init,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = poa030_g_register,
	.s_register = poa030_s_register,
#endif
};

static const struct v4l2_subdev_video_ops poa030_video_ops = {
	.enum_fmt = poa030_enum_fmt,
	.try_fmt = poa030_try_fmt,
	.s_fmt = poa030_s_fmt,
	.s_parm = poa030_s_parm,
	.g_parm = poa030_g_parm,
};

static const struct v4l2_subdev_ops poa030_ops = {
	.core = &poa030_core_ops,
	.video = &poa030_video_ops,
};

/* ----------------------------------------------------------------------- */

static int poa030_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct nx_vip_camera *info;
	int ret;

	info = kzalloc(sizeof(struct nx_vip_camera), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &poa030_ops);

	/* Make sure it's an poa030 */
	ret = poa030_detect(sd);
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

	printk("POA030R name = %s, id=%d\n", id->name, info->id);
	
	info->client = client;
	info->i2c_read = poa030_get_i2c;
	info->i2c_write = poa030_set_i2c;

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
	info->cam_id = 160;

	nx_vip_register_subdev(sd);
	
	poa030_reset(sd, 0);
	udelay(1000); /* wait some time */
	poa030_start(client);
	info->initialized = 1;
	info->priv_data = -1; /* page map value */

	return 0;
}

static int poa030_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_vip_cam(sd));
	return	0;
}

static int poa030_change_resolution(struct i2c_client *client, int res)
{
	switch (res) {
	case CAM_RES_QSVGA:
		// Do something
		break;

	case CAM_RES_MAX:	/* fall through */
		// set max resoution.
		
	case CAM_RES_DEFAULT:	/* fall through */
	case CAM_RES_SVGA:
		// DO something...
		break;

	default:
		err("unexpect value\n");
	}

	return 0;
}

static int poa030_change_whitebalance(struct i2c_client *client, enum nx_vip_wb_t type)
{

	return 0;
}

static int poa030_set_brightness(struct i2c_client *client, int val)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct nx_vip_camera *cam = to_vip_cam(sd);
	int maxval;

	cam->cur_gain = val;
	/* Value would be 0 to 100 */
	/*	AE mode 00 01 10 11 */
#if 0
	maxval = 0x4F;
	maxval = maxval * val / 100;
	poa030_write_reg8(client,POA030_GLOBALGAIN, maxval);
#else
	maxval = 0x1000 - 0x0100;
	maxval = maxval * val / 100 + 0x0100;
	poa030_write_reg8(client,POA030_EXT_GLBG_L, maxval&0xFF);
	poa030_write_reg8(client,POA030_EXT_GLBG_H, (maxval>>8)&0xFF);
#endif

	return 0;
}

static int poa030_set_set_power_save(struct i2c_client *client, int val)
{
	u8 padval;

	poa030_read_reg8(client,POA030_PAD_CONTROL, &padval);

	if (val == 0)
		poa030_write_reg8(client,POA030_PAD_CONTROL, padval&0x7F);
	else
		poa030_write_reg8(client,POA030_PAD_CONTROL, padval|0x80);

	return 0;
}

static int poa030_set_aemode(struct i2c_client *client, int val)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct nx_vip_camera *cam = to_vip_cam(sd);
	int setval;

	cam->cur_aemode = val;

	poa030_read_reg8(client, POA030_AUTO_CONTROL_1, &setval);

	if (val)
		setval &= ~(0x02);	// Auto Exposure
	else
		setval |= 0x02;	// Manual Exposure

	poa030_write_reg8(client, POA030_AUTO_CONTROL_1, setval);

	return 0;
}

static int poa030_command(struct i2c_client *client, u32 cmd, void *arg)
{
	u8 val;

	switch (cmd) {
	case I2C_CAM_INIT:
		poa030_start(client);
		info("POA030 : external camera initialized\n");
		break;

	case I2C_CAM_RESOLUTION:
		poa030_change_resolution(client, (int) arg);
		break;

	case I2C_CAM_WB:
		poa030_change_whitebalance(client, (enum nx_vip_wb_t) arg);
		break;
	case I2C_CAM_BRIGHTNESS:
		poa030_set_brightness(client, (int) arg);
		break;

	case I2C_CAM_POWER_SAVE:
		poa030_set_set_power_save(client, (int) arg);
		break;

	case I2C_CAM_EXPOSURE_AUTO:
		poa030_set_aemode(client, (int)arg);
		break;

	default:
		err("unexpect command\n");
		break;
	}

	return 0;
}

static int poa030_driver_resume(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct nx_vip_camera *cam = to_vip_cam(sd);

	info("resume camera %d from device driver resume\n", cam->id);

    poa030_reset(sd, 0);
	udelay(1000); /* wait some time */
	poa030_start(client);
	poa030_set_brightness(client, cam->cur_gain);
	poa030_set_aemode(client, cam->cur_aemode);

    return 0;
}

static int poa030_i2c_driver_resume(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct nx_vip_camera *cam = to_vip_cam(sd);

	info("resume camera %d from i2c driver resume\n", cam->id);

    poa030_reset(sd, 0);
	udelay(1000); /* wait some time */
	poa030_start(client);
	poa030_set_brightness(client, cam->cur_gain);
	poa030_set_aemode(client, cam->cur_aemode);

    return 0;
}


#if defined(CONFIG_CAMERA1_POA030)
static const struct i2c_device_id poa030_id[] = {
	{ "poa030", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, poa030_id);

static struct i2c_driver poa030_driver = {
	.driver = {
		.name = "POA030",
		.owner = THIS_MODULE,
		.resume = poa030_driver_resume,
	},
	.id_table	  = poa030_id,
	.probe		  = poa030_probe,
	.remove		  = __devexit_p(poa030_remove),
	.resume       = poa030_i2c_driver_resume,
	.command	  = poa030_command,
};
#endif

#if defined(CONFIG_CAMERA2_POA030)
static const struct i2c_device_id poa030_id2[] = {
	{ "poa030_2", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, poa030_id2);

static struct i2c_driver poa030_driver2 = {
	.driver = {
		.name = "POA030_2",
		.owner = THIS_MODULE,
		.resume = poa030_driver_resume,
	},
	.id_table	  = poa030_id2,
	.probe		  = poa030_probe,
	.remove		  = __devexit_p(poa030_remove),
	.resume       = poa030_i2c_driver_resume,
	.command	  = poa030_command,
};
#endif

static __init int _poa030_init(void)
{
	int ret;

#if defined(CONFIG_CAMERA1_POA030)
	ret = i2c_add_driver(&poa030_driver);
#endif
#if defined(CONFIG_CAMERA2_POA030)
	ret = i2c_add_driver(&poa030_driver2);
#endif
	return ret; 
}

static __init void _poa030_exit(void)
{
#if defined(CONFIG_CAMERA1_POA030)
	i2c_del_driver(&poa030_driver);
#endif
#if defined(CONFIG_CAMERA2_POA030)
	i2c_del_driver(&poa030_driver2);
#endif
}

module_init(_poa030_init)
module_exit(_poa030_exit)

MODULE_AUTHOR("Seungwoo Kim<ksw@stcube.com>");
MODULE_DESCRIPTION("POA030 Camera Driver");
MODULE_LICENSE("GPL");

