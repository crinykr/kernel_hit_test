/* linux/drivers/media/video/nexel/ov7675.c
 *
 * OmniVision OV7675 CMOS Image Sensor driver
 *
 * Seungwoo Kim <ksw@mostitech.com>
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
#include "ov7675.h"

extern void nx_vip_register_camera(struct nx_vip_camera *cam);

const static u16 ignore[] = { I2C_CLIENT_END };
const static u16 normal_addr[] = { OV7675_I2C_ADDR, I2C_CLIENT_END };
const static u16 forces[] = { 0 };

static struct i2c_driver ov7675_i2c_driver;

static struct nx_vip_camera ov7675_data = {
	.id 		= 1,

	//.type		= CAM_TYPE_ITU,
	//.mode		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY, //CAM_ORDER422_8BIT_YCBYCR,

	/*
	 * 20 fps: 44 MHz
	 * 12 fps: 26 MHz (more stable)
	 */
	.clockrate	= 26600000,

	.width		= 640,
	.height		= 480,
	//.clipw      = 640,
	//.cliph      = 480,
	.offset		= {
		.h1 = 0,
		.h2 = 0,
		.v1 = 0,
		.v2 = 0,
	},

	.polarity	= {
		.pclk	= 0,
		.vsync	= 1,
		.href	= 0,
		.hsync	= 0,
	},

	.initialized	 = 0,
	//.use_scaler      = 1,
	.source_sel      = 1,
	//.use_clip        = 0,
	.use_time_stamp  = 1,
	.auto_laser_ctrl = 1,
};

/*
 * The default register settings, as obtained from OmniVision.  There
 * is really no making sense of most of these - lots of "reserved" values
 * and such.
 *
 * These settings give VGA YUYV.
 */
struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};

/*
 * The default register settings, as obtained from OmniVision.  There
 * is really no making sense of most of these - lots of "reserved" values
 * and such.
 *
 * These settings give VGA YUYV.
 */
static struct regval_list ov7675_default_regs[] = {
	{ 0x04,0x40 },// CCIR656
	{ 0x11,0x80 },// 30fps(80), 15fps(00)
	{ 0x3a,0x0C },
	{ 0x3D,0xC0 },
	{ 0x12,0x00 },
	{ 0x15,0x40 },
	{ 0xc1,0x7f },
	{ 0x17,0x13 },
	{ 0x18,0x01 },
	{ 0x32,0x3f },
	{ 0x19,0x02 },
	{ 0x1a,0x7a },//7b
	{ 0x03,0x2f },
	{ 0x0c,0x00 },
	{ 0x3e,0x00 },
	{ 0x70,0x3a },
	{ 0x71,0x35 },
	{ 0x72,0x11 },
	{ 0x73,0xf0 },
	{ 0xa2,0x02 },
	{ 0x7a,0x24 },
	{ 0x7b,0x04 },
	{ 0x7c,0x07 },
	{ 0x7d,0x10 },
	{ 0x7e,0x38 },
	{ 0x7f,0x4a },
	{ 0x80,0x5a },
	{ 0x81,0x67 },
	{ 0x82,0x71 },
	{ 0x83,0x7b },
	{ 0x84,0x85 },
	{ 0x85,0x95 },
	{ 0x86,0xa4 },
	{ 0x87,0xbc },
	{ 0x88,0xd2 },
	{ 0x89,0xe5 },
	{ 0x00,0x00 },
	{ 0x0d,0x40 },
	{ 0x14,0x28 },
	{ 0xa5,0x06 },
	{ 0xab,0x07 },
	{ 0x24,0x58 },
	{ 0x25,0x48 },
	{ 0x26,0x93 },
	{ 0x9f,0x78 },
	{ 0xa0,0x68 },
	{ 0xa1,0x03 },
	{ 0xa6,0xD8 },
	{ 0xa7,0xD8 },
	{ 0xa8,0xf0 },
	{ 0xa9,0x90 },
	{ 0xaa,0x14 },
	{ 0x0e,0x61 },
	{ 0x0f,0x4b },
	{ 0x16,0x02 },
	{ 0x1e,0x07 },
	{ 0x21,0x02 },
	{ 0x22,0x91 },
	{ 0x29,0x07 },
	{ 0x33,0x0b },
	{ 0x35,0x0b },
	{ 0x37,0x1d },
	{ 0x38,0x71 },
	{ 0x39,0x2a },
	{ 0x3c,0x78 },
	{ 0x4d,0x40 },
	{ 0x4e,0x20 },
	{ 0x69,0x00 },
	{ 0x6b,0x0a },
	{ 0x74,0x10 },
	{ 0x8d,0x4f },
	{ 0x8e,0x00 },
	{ 0x8f,0x00 },
	{ 0x90,0x00 },
	{ 0x91,0x00 },
	{ 0x96,0x00 },
	{ 0x9a,0x80 },
	{ 0xb0,0x84 },
	{ 0xb1,0x0c },
	{ 0xb2,0x0e },
	{ 0xb3,0x82 },
	{ 0xb8,0x0a },
	{ 0xbb,0xa1 },//blc target
	{ 0x0d,0x60 },
	{ 0x42,0x80 },
	{ 0x43,0x0a },
	{ 0x44,0xf0 },
	{ 0x45,0x34 },
	{ 0x46,0x58 },
	{ 0x47,0x28 },
	{ 0x48,0x3a },
	{ 0x59,0x88 },
	{ 0x5a,0x88 },
	{ 0x5b,0xc2 },
	{ 0x5c,0x60 },
	{ 0x5d,0x58 },
	{ 0x5e,0x18 },
	{ 0x6c,0x0a },
	{ 0x6d,0x55 },
	{ 0x6e,0x11 },
	{ 0x6f,0x9e },
	{ 0x6a,0x40 },
	{ 0x01,0x56 },
	{ 0x02,0x44 },
	{ 0x07,0x00 },//exposure [5:0]
	{ 0x10,0x70 },//exposure [7:0]
	{ 0x04,0x40 },//exposure [1:0]
	{ 0x13,0xe2 },//manual exposure setting
	{ 0x4f,0xa6 },
	{ 0x50,0xb5 },
	{ 0x51,0x0f },
	{ 0x52,0x18 },
	{ 0x53,0x9d },
	{ 0x54,0xb5 },
	{ 0x58,0x1a },
	{ 0x3f,0x02 },
	{ 0x75,0x63 },
	{ 0x76,0xe1 },
	{ 0x4c,0x00 },
	{ 0x77,0x01 },
	{ 0x3D,0xC2 },
	{ 0x4b,0x09 },
	{ 0xc9,0x60 },
	{ 0x41,0x38 },
	{ 0x56,0x40 },
	{ 0x34,0x11 },
	{ 0x3b,0x0a },
	{ 0xa4,0x88 },
	{ 0x96,0x00 },
	{ 0x97,0x30 },
	{ 0x98,0x20 },
	{ 0x99,0x30 },
	{ 0x9a,0x84 },
	{ 0x9b,0x29 },
	{ 0x9c,0x03 },
	{ 0x9d,0x99 },
	{ 0x9e,0x7f },
	{ 0x78,0x04 },
	{ 0x79,0x01 },
	{ 0xc8,0xf0 },
	{ 0x79,0x0f },
	{ 0xc8,0x00 },
	{ 0x79,0x10 },
	{ 0xc8,0x7e },
	{ 0x79,0x0a },
	{ 0xc8,0x80 },
	{ 0x79,0x0b },
	{ 0xc8,0x01 },
	{ 0x79,0x0c },
	{ 0xc8,0x0f },
	{ 0x79,0x0d },
	{ 0xc8,0x20 },
	{ 0x79,0x09 },
	{ 0xc8,0x80 },
	{ 0x79,0x02 },
	{ 0xc8,0xc0 },
	{ 0x79,0x03 },
	{ 0xc8,0x40 },
	{ 0x79,0x05 },
	{ 0xc8,0x30 },
	{ 0x79,0x26 },
	{ 0x62,0x00 },
	{ 0x63,0x00 },
	{ 0x64,0x10 },
	{ 0x65,0x07 },
	{ 0x66,0x05 },
	{ 0x94,0x10 },
	{ 0x95,0x12 },
    { 0xff,0xff }, /* END MARKER */
};

static struct regval_list ov7675_fmt_yuv422[] = {
	{ OV7675_COM7, 0x0 },  /* Selects YUV mode */
	{ OV7675_RGB444, 0 },	/* No RGB444 please */
	{ OV7675_COM1, COM1_CCIR656 },
	{ OV7675_COM15, COM15_R00FF },
	{ OV7675_COM9, 0x48 }, /* 32x gain ceiling; 0x8 is reserved bit */
	{ 0x4f, 0x80 }, 	/* "matrix coefficient 1" */
	{ 0x50, 0x80 }, 	/* "matrix coefficient 2" */
	{ 0x51, 0    },		/* vb */
	{ 0x52, 0x22 }, 	/* "matrix coefficient 4" */
	{ 0x53, 0x5e }, 	/* "matrix coefficient 5" */
	{ 0x54, 0x80 }, 	/* "matrix coefficient 6" */
	{ OV7675_COM13, COM13_GAMMA|COM13_UVSAT },
	{ 0xff, 0xff },
};

#define I2C_FLAG_READ   0x10

static int ov7675_i2c_xfer(u8 reg, char *buf, int num, int tran_flag)
{
	struct i2c_msg msg[2];
	int ret;
	char xbuf[128];

	if (tran_flag & I2C_FLAG_READ) {
	    msg[0].addr = OV7675_I2C_ADDR;
	    msg[0].len = 1;
	    msg[0].buf = (char *)&reg;
	    msg[0].flags = 0;

	    msg[1].addr = OV7675_I2C_ADDR;
	    msg[1].len = num;
	    msg[1].buf = buf;
	    msg[1].flags = I2C_M_RD;

	    ret = i2c_transfer(ov7675_data.client->adapter, msg, 2);
	} else {
	    xbuf[0] = reg;
	    memcpy(&xbuf[1], buf, num);
	    msg[0].addr = OV7675_I2C_ADDR;
	    msg[0].len = 1 + num;
	    msg[0].buf = xbuf;
	    msg[0].flags = 0;

	    ret = i2c_transfer(ov7675_data.client->adapter, msg, 1);
	}

	if (ret >= 0)
		return 0;

	return ret;
}

static int ov7675_read_reg8(u8  reg, u8 *val)
{
	int ret;
	u8 rval;

	reg &= 0xFF;

	ret = ov7675_i2c_xfer(reg, (u8 *) &rval, 1, I2C_FLAG_READ);
	if (!ret) {
		*val = rval;
		return 0;
	}
	return ret;
}

static int ov7675_write_reg8(u8 reg, u8 val)
{
	u8 temp1;

	temp1 = reg & 0xFF;

	return ov7675_i2c_xfer(temp1, (u8 *) & val, 1, 0);
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov7675_write_reg8_array(struct i2c_client *c, struct regval_list *vals)
{
	while (vals->reg_num != 0xff || vals->value != 0xff)
	{
		int ret = ov7675_write_reg8(vals->reg_num, vals->value);

		if (ret < 0)
			return ret;
		vals++;
	}

	return 0;
}

/*
 * Store a set of start/stop values into the camera.
 */
static int ov7675_set_hw(struct i2c_client *client, int hstart, int hstop, int vstart, int vstop)
{
	int ret;
	unsigned char v;
	/*
	 * Horizontal: 11 bits, top 8 live in hstart and hstop.  Bottom 3 of
	 * hstart are in href[2:0], bottom 3 of hstop in href[5:3].  There is
	 * a mystery "edge offset" value in the top two bits of href.
	 */
	ret =  ov7675_write_reg8(OV7675_HSTART, (hstart >> 3) & 0xff);
	ret += ov7675_write_reg8(OV7675_HSTOP, (hstop >> 3) & 0xff);
	ret += ov7675_read_reg8(OV7675_HREF, &v);
	v = (v & 0xc0) | ((hstop & 0x7) << 3) | (hstart & 0x7);
	msleep(10);
	ret += ov7675_write_reg8(OV7675_HREF, v);

	/*
	 * Vertical: similar arrangement, but only 10 bits.
	 */
	ret += ov7675_write_reg8(OV7675_VSTART, (vstart >> 2) & 0xff);
	ret += ov7675_write_reg8(OV7675_VSTOP, (vstop >> 2) & 0xff);
	ret += ov7675_read_reg8(OV7675_VREF, &v);
	v = (v & 0xf0) | ((vstop & 0x3) << 2) | (vstart & 0x3);
	msleep(10);
	ret += ov7675_write_reg8(OV7675_VREF, v);

	return ret;
}

/*************************************************************************
* FUNCTION
*	config_OV7675_window
*
* DESCRIPTION
*	This function config the hardware window of OV7675 for getting specified
*  data of that window.
*
* PARAMETERS
*	start_x : start column of the interested window
*  start_y : start row of the interested window
*  width  : column widht of the itnerested window
*  height : row depth of the itnerested window
*
* RETURNS
*	the data that read from OV7675
*
* GLOBALS AFFECTED
*
*************************************************************************/
void ov7675_config_window(u16 startx, u16 starty, u16 width, u16 height)
{
	u16 endx=(startx+width-1);
	u16 endy=(starty+height-1);
	u8  temp_reg1, temp_reg2;

	ov7675_read_reg8(0x03, &temp_reg1);
	ov7675_read_reg8(0x32, &temp_reg2);

	temp_reg1=temp_reg1&0xF0;
	temp_reg2=temp_reg2&0xC0;

	// Horizontal
	ov7675_write_reg8(0x32,0x80|((endx&0x7)<<3)|(startx&0x7));	// b[5:3]:HREF end low 3bits. b[2:0]:HREF start low 3bits.
	ov7675_write_reg8(0x17,(startx&0x7F8)>>3);			// HREF start high 8bits
	ov7675_write_reg8(0x18,(endx&0x7F8)>>3);			// HREF end high 8bits
	// Vertical
	ov7675_write_reg8(0x03,temp_reg1|((endy&0x3)<<2)|(starty&0x3));	// b[3:2]:VREF end low 2bits. b[1:0]:VREF start low 2bits.
	ov7675_write_reg8(0x19,(starty&0x3FC)>>2);   			// VREF start high 8bits
	ov7675_write_reg8(0x1A,(endy&0x3FC)>>2);		   	// VREF end high 8bits
}	/* config_OV7675_window */


static void ov7675_start(struct i2c_client *client)
{
	u8 val;

	ov7675_write_reg8(0x12,COM7_RESET);
	msleep(10);

	ov7675_write_reg8_array(client, ov7675_default_regs);

	ov7675_read_reg8(OV7675_COM1, &val);
	printk("OV7675_COM1 : after val=%x\n", val);
}

/*!
 * ov7675 get_i2c function
 *
 * @return  none
 */
static int ov7675_get_i2c(int addr, int *val)
{
	return ov7675_read_reg8(addr, (u8 *)val);
}

/*!
 * ov7675 set_i2c function
 *
 * @return  none
 */
static int ov7675_set_i2c(int addr, int val)
{
    //printk("ov7675 set i2c!, addr = 0x%x val = 0x%x\n", addr, val);
	return ov7675_write_reg8(addr, val);
}

static int __devinit ov7675_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
//    struct ov7675_i2c_platform_data  *pdata = client->dev.platform_data;

    printk("OV7675_i2c_probe(2) client->addr=%X\n", client->addr);

    ov7675_data.client = client;
    ov7675_data.i2c_read = ov7675_get_i2c;
    ov7675_data.i2c_write = ov7675_set_i2c;

	i2c_set_clientdata(client, &ov7675_data);

	nx_vip_register_camera(&ov7675_data);
	msleep(5);

	ov7675_start(client);
	ov7675_data.initialized = 1;

    return 0;
}

static int __devexit ov7675_i2c_remove(struct i2c_client *client)
{
    return  0;
}

static int ov7675_change_resolution(struct i2c_client *client, int res)
{
	switch (res) {
	case CAM_RES_QSVGA:
		// Do something
		break;

	case CAM_RES_MAX:	/* fall through */
	case CAM_RES_DEFAULT:	/* fall through */
	case CAM_RES_SVGA:
 		// DO something...
		break;

	default:
		err("unexpect value\n");
	}

	return 0;
}

static int ov7675_change_whitebalance(struct i2c_client *client, enum nx_vip_wb_t type)
{

	return 0;
}

static int ov7675_set_brightness(struct i2c_client *client, int val)
{
	s8 maxval;

    maxval = (s8)((val <= 50) ? (-254 * val / 100 - 1) : (255 * val / 100 - 128));
    ov7675_write_reg8(OV7675_BRIGHT, maxval&0xFF);

	return 0;
}

static int ov7675_set_set_power_save(struct i2c_client *client, int val)
{
	return 0;
}

static int ov7675_set_exposure(struct i2c_client *client, int val)
{
	u8	exph, expm, expl;
	u8	aec;

	ov7675_read_reg8(OV7675_COM8, &aec);

	aec |= COM8_AEC;
    ov7675_write_reg8(OV7675_COM8, aec);

	ov7675_read_reg8(OV7675_AECHH, &exph);
	ov7675_read_reg8(OV7675_AECH,  &expm);
	ov7675_read_reg8(OV7675_COM1,  &expl);

	exph = (exph & (~0x3F)) | ((val >> 10) & 0x3F);
	expm = (expm & (~0xFF)) | ((val >>  2) & 0xFF);
	expl = (expl & (~0x03)) | ((val >>  0) & 0x03);

	//0x04 COM1 Bit[1:0]: Exposure time, the unit is tRow interval
	//AEC[15:0] = {0x07[5:0], 0x10[7:0], 0x04[1:0]}
	//               6bit       8bit       2bit
	//             0x00       0x40       0x00
	exph = ov7675_write_reg8(OV7675_AECHH, exph);
	exph = ov7675_write_reg8(OV7675_AECH,  expm);
	exph = ov7675_write_reg8(OV7675_COM1,  expl);

	aec &= ~COM8_AEC;
    ov7675_write_reg8(OV7675_COM8, aec);

	return 0;
}

static int ov7675_set_fixed_frame(struct i2c_client *client, int val)
{
	ov7675_write_reg8(0x2a, 0x00);
	ov7675_write_reg8(0x2b, 0x00);
	ov7675_write_reg8(0x92, 0x00);
	ov7675_write_reg8(0x93, 0x00);
	ov7675_write_reg8(0x3b, 0x0a);

	if (val == 30)
	{
		ov7675_write_reg8(0x6b, 0x0a);	// PLL control Input clock bypass
		ov7675_write_reg8(0x11, 0x80);	// Internal clock pre-scalar(Input clock / 1)--datasheet ? x/1?
	}
	else if (val == 15)
	{
		ov7675_write_reg8(0x6b, 0x0a);	// PLL control Input clock bypass
		ov7675_write_reg8(0x11, 0x00);	// Internal clock pre-scalar(Input clock / 1)--datasheet ? x/1?
	}
	else if (val == 10)
	{
		ov7675_write_reg8(0x6b, 0x5a);	// PLL control [7:6]=0x01: Input clock x4
		ov7675_write_reg8(0x11, 0x05);	// Internal clock pre-scalar(Input clock / 6)
	}
	else if (val == 7)
	{
		ov7675_write_reg8(0x6b, 0x5a);	// PLL control [7:6]=0x01: Input clock x4
		ov7675_write_reg8(0x11, 0x07);	// Internal clock pre-scalar(Input clock / 7)
	}
	else
		return -1;

	return 0;
}

//gain = (0x03[7] + 1) x (0x03[6] + 1) x (0x00[7] + 1) x (0x00[6] + 1) x
//       (0x00[5] + 1) x (0x00[4] + 1) x (0x00[3:0] / 16 + 1)
static int ov7675_set_gain(struct i2c_client *client, int val)
{
	u8	reg_gl = 0;
	u8	reg_gh = 0;
	int	div = 0x40;
	int	gain;
	int i;

	if (val < 0 || val > 64)
		return -EINVAL;

	ov7675_read_reg8(OV7675_VREF, &reg_gl);
	ov7675_read_reg8(OV7675_GAIN, &reg_gh);

	reg_gl &= 0x3F;
	reg_gh &= 0x00;

	for (i = 0 ; i < 6; i++)
	{
		if (val & div)
			break;

		div >>= 1;
	}

	gain = 0;
	for ( ;i < 6; i++)
	{
		gain = (gain << 1) | 1;
	}

	reg_gl |= ((gain & 0x03) << 6);
	reg_gh |= ((gain & 0x3C) << 2);

	ov7675_write_reg8(OV7675_VREF, reg_gl);
	ov7675_write_reg8(OV7675_GAIN, reg_gh);

	return 0;
}

static int ov7675_command(struct i2c_client *client, u32 cmd, void *arg)
{
	u8 val;

	switch (cmd) {
	case I2C_CAM_INIT:
		ov7675_start(client);
		info("OV7675 : external camera initialized\n");
		break;

	case I2C_CAM_RESOLUTION:
		ov7675_change_resolution(client, (int) arg);
		break;

	case I2C_CAM_WB:
		ov7675_change_whitebalance(client, (enum nx_vip_wb_t) arg);
        break;
    case I2C_CAM_BRIGHTNESS:
        ov7675_set_brightness(client, (int) arg);
        break;

    case I2C_CAM_POWER_SAVE:
        ov7675_set_set_power_save(client, (int) arg);
        break;

    case I2C_CAM_EXPOSURE:
    	ov7675_set_exposure(client, (int)arg);
    	break;

    case I2C_CAM_FIXED_FRAME:
    	ov7675_set_fixed_frame(client, (int)arg);
    	break;

    case I2C_CAM_EXPOSURE_AUTO:
		ov7675_read_reg8(OV7675_COM8, &val);// Manual Exposure

		val &= ~(COM8_AGC | COM8_AEC);
    	if ((int)arg)
    		val |= (COM8_AGC | COM8_AEC);

    	ov7675_write_reg8(OV7675_COM8, val);// Auto Exposure
    	break;

    case I2C_CAM_GAIN:
    	ov7675_set_gain(client, (int)arg);// Gain
    	break;

	default:
		err("unexpect command\n");
		break;
	}

	return 0;
}

static const struct i2c_device_id ov7675_i2c_id[] = {
    { "ov7675_2", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, ov7675_i2c_id);

static struct i2c_driver ov7675_i2c_driver = {
	.driver = {
		.name = "OV7675_2",
		.owner = THIS_MODULE,
	},
	.id_table     = ov7675_i2c_id,
	.probe        = ov7675_i2c_probe,
    .remove       = __devexit_p(ov7675_i2c_remove),
	.command      = ov7675_command,
};

static __init int ov7675_2_init(void)
{
    printk("OmniVision OV7675(2nd) driver initialized\n");
	return i2c_add_driver(&ov7675_i2c_driver);
}

static __init void ov7675_2_exit(void)
{
	i2c_del_driver(&ov7675_i2c_driver);
}

module_init(ov7675_2_init)
module_exit(ov7675_2_exit)

MODULE_AUTHOR("Seungwoo Kim<ksw@stcube.com>");
MODULE_DESCRIPTION("OV7675 Camera Driver");
MODULE_LICENSE("GPL");

