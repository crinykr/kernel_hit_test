/* linux/drivers/media/video/nexel/dumcopy.c
 *
 * Dummy copy driver for VIP1 to VIP0
 *
 * Seungwoo Kim <ksw@stcube.com>
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

extern void nx_vip_register_camera(struct nx_vip_camera *cam);

static struct nx_vip_camera dumcopy_data = {
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
	.clipw      = 416,
	.cliph      = 312,
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

	.initialized	= 0,
	.use_scaler     = 1,
	.use_source     = 0,
	.use_clip       = 1,
};

static __init int dumcopy_init(void)
{
	nx_vip_register_camera(&dumcopy_data);
	dumcopy_data.initialized = 1;

	return 0;
}

static __init void dumcopy_exit(void)
{
	// Do nothing for dummy driver.
}

module_init(dumcopy_init)
module_exit(dumcopy_exit)

MODULE_AUTHOR("Seungwoo Kim<ksw@stcube.com>");
MODULE_DESCRIPTION("DummyCopy Camera Driver");
MODULE_LICENSE("GPL");

