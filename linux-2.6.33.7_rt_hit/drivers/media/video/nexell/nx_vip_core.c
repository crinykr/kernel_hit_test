/* linux/drivers/media/video/nexell/nx_vip_core.c
 *
 * Core file for Nexell NXP2120 camera(VIP) driver
 *
 * Copyright (c) 2011 MOSTiTECH co., ltd.
 * All right reserved by Seungwoo Kim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/videodev2.h>

#include <asm/io.h>
#include <asm/memory.h>

#include <mach/platform.h>
#include <mach/devices.h>
#include "nx_vip.h"

#define BUFFER_X1280	0
/* ksw : always initialize camera for camera test board */
// #define CONFIG_ALWAYS_INITIALIZE_CAMERA

static struct nx_vip_camera test_pattern = {
	.id 		= NX_VIP_TPID,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.clockrate	= 0,
	.cur_width		= 640,
	.cur_height		= 480,
	.def_width		= 640,
	.def_height		= 480,
	.max_width		= 640,
	.max_height		= 480,
	/*.offset		= {
		.h1 = 0,
		.h2 = 0,
		.v1 = 0,
		.v2 = 0,
	},

	.polarity	= {
		.pclk	= 0,
		.vsync	= 0,
		.href	= 0,
		.hsync	= 0,
	}, */

	.initialized	= 0,
};

struct nx_vip_config nx_vip;

struct nx_platform_vip *to_vip_plat(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);

	return (struct nx_platform_vip *) pdev->dev.platform_data;
}

int nx_vip_i2c_read(struct nx_vip_control *ctrl, int subaddr, int *val)
{
    return ctrl->in_cam->i2c_read(ctrl->in_cam->client, subaddr, val);
}

int nx_vip_i2c_write(struct nx_vip_control *ctrl,int subaddr, int val)
{
    return ctrl->in_cam->i2c_write(ctrl->in_cam->client, subaddr, val);
}

void nx_vip_i2c_command(struct nx_vip_control *ctrl, u32 cmd, int arg)
{
	struct i2c_client *client = ctrl->in_cam->client;

	if (client)
		client->driver->command(client, cmd, (void *) arg);
	else
		err("i2c client is not registered\n");
}

void nx_vip_reset_camera(void)
{
    /* Do something for GPIO ? */

}

void nx_vip_register_camera(struct nx_vip_camera *cam)
{
	nx_vip.camera[cam->id] = cam;

	nx_vip_reset_camera();
}

void nx_vip_unregister_camera(struct nx_vip_camera *cam)
{
	int i = 0;

	for (i = 0; i < NX_VIP_MAX_CTRLS; i++) {
		if (nx_vip.ctrl[i].in_cam == cam)
			nx_vip.ctrl[i].in_cam = NULL;
	}

	nx_vip.camera[cam->id] = NULL;
}

void nx_vip_register_subdev(struct v4l2_subdev *sd)
{
	struct nx_vip_camera *cam = to_vip_cam(sd);

	nx_vip_register_camera(cam);
}
EXPORT_SYMBOL_GPL(nx_vip_register_subdev);

int nx_vip_set_active_camera(struct nx_vip_control *ctrl, int id)
{
	ctrl->in_cam = nx_vip.camera[id];
	if (NULL == ctrl->in_cam)
		return -1;

	// Anything we should do?
	return 0;
}

void nx_vip_init_camera(struct nx_vip_control *ctrl)
{
	struct nx_vip_camera *cam = ctrl->in_cam;

	if (cam && cam->id != NX_VIP_TPID && !cam->initialized) {
		nx_vip_i2c_command(ctrl, I2C_CAM_INIT, 0);
		nx_vip_change_resolution(ctrl, CAM_RES_DEFAULT);
		cam->initialized = 1;
	}
#if defined(CONFIG_ALWAYS_INITIALIZE_CAMERA)
	nx_vip_i2c_command(ctrl, I2C_CAM_INIT, 0);
#endif
}

static int nx_vip_set_output_format(struct nx_vip_control *ctrl,
					struct v4l2_pix_format *fmt)
{
	struct nx_vip_out_frame *frame = &ctrl->out_frame;
	int depth = -1;

	frame->width = fmt->width;
	frame->height = fmt->height;

	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_YUV420:
		frame->format = FORMAT_YCBCR420;
		frame->planes = 3;
		depth = 12;
		break;
	case V4L2_PIX_FMT_YUV444:
		frame->format = FORMAT_YCBCR444;
		frame->planes = 3;
		depth = 24;
		break;

	case V4L2_PIX_FMT_YUYV:
		frame->format = FORMAT_YCBCR422;
		frame->planes = 1;
		frame->order_1p = OUT_ORDER422_YCBYCR;
		depth = 16;
		break;

	case V4L2_PIX_FMT_YVYU:
		frame->format = FORMAT_YCBCR422;
		frame->planes = 1;
		frame->order_1p = OUT_ORDER422_YCRYCB;
		depth = 16;
		break;

	case V4L2_PIX_FMT_UYVY:
		frame->format = FORMAT_YCBCR422;
		frame->planes = 1;
		frame->order_1p = OUT_ORDER422_CBYCRY;
		depth = 16;
		break;

	case V4L2_PIX_FMT_VYUY:
		frame->format = FORMAT_YCBCR422;
		frame->planes = 1;
		frame->order_1p = OUT_ORDER422_CRYCBY;
		depth = 16;
		break;
	case V4L2_PIX_FMT_YUV422P:
		frame->format = FORMAT_YCBCR422;
		frame->planes = 3;
		depth = 16;
		break;
	default :
	    depth = -1;
	}

	switch (fmt->field) {
	case V4L2_FIELD_INTERLACED:
	case V4L2_FIELD_INTERLACED_TB:
	case V4L2_FIELD_INTERLACED_BT:
		frame->scan = SCAN_TYPE_INTERLACE;
		break;

	default:
		frame->scan = SCAN_TYPE_PROGRESSIVE;
		break;
	}

	return depth;
}

int nx_vip_set_output_frame(struct nx_vip_control *ctrl,
				struct v4l2_pix_format *fmt)
{
	struct nx_vip_out_frame *frame = &ctrl->out_frame;
	int depth = 0;
	int to_change = 0;
	int old_plane;

	if ((ctrl->out_frame.width != fmt->width) ||
		(ctrl->out_frame.height != fmt->height)) {
		to_change = 1;
	}
	old_plane = ctrl->out_frame.planes;

	depth = nx_vip_set_output_format(ctrl, fmt);

	if (old_plane != ctrl->out_frame.planes) {
		to_change |= 2;
	}

	if ((frame->addr[0].virt_y == NULL) && (depth > 0)) {
		if (nx_vip_alloc_output_memory(ctrl)) {
			err("cannot allocate memory\n");
		}
		//printk("cfn = %d, addr[0].virt_y = %x, addr[1].virt_y = %x, addr[2].virt_y = %x, addr[3].virt_y = %x",
	    //frame->cfn,frame->addr[0].virt_y, frame->addr[1].virt_y,
	    //frame->addr[2].virt_y, frame->addr[3].virt_y);
	} else {
		// If we want to change alloc mem's depth and/or width/height, we should stop the engine first.
		if (to_change) {
			nx_vip_stop_vip(ctrl);
			// Do we should stop IRQ also?
			// alloc output memory -- actually this is not allocate, just reassigning the buffer address.
			nx_vip_alloc_output_memory(ctrl);
		}
	}

	return depth;
}

int nx_vip_alloc_output_memory(struct nx_vip_control *ctrl)
{
	struct nx_vip_out_frame *info = &ctrl->out_frame;
	int bank;
	int width = info->width, height = info->height;
	unsigned long start_offset = 0;
	int buffer_changed = 0;
	struct nx_vip_frame_addr *addr;
	int cr_w, cr_h, cr_l, cr_t;
	int top;
	
	cr_w = ctrl->v4l2.crop_current.width;
	cr_h = ctrl->v4l2.crop_current.height;
	cr_l = ctrl->v4l2.crop_current.left;
	cr_t = ctrl->v4l2.crop_current.top;

	if ((cr_w != width) && (cr_h != height))
	{
		ctrl->use_scaler = 1;
#if BUFFER_X1280
		ctrl->scaler->offset = 512 * 4096;
#else
		ctrl->scaler->offset = 1024 * 4096; // for 640X480 ->1024X512 for Y, 1024X512 for U,V
#endif
		//printk("cr_w=%d, width=%d cr_h=%d height=%d\n", cr_w, width, cr_h, height);
	} else {
		ctrl->use_scaler = 0;
		//printk("cr_w=%d cr_h=%d\n", cr_w, cr_h);
	}

	/* But with reserved mem as bootmem area, we could avoid complexity
	   of DMA coherent and memory leakage.
	   
	   2014. 02. 21. by KSW:
		 4096 stride would give 4 buffers with 1024 horizontal resolution.
		 Hence, 4Mbytes of buffer could handle 4 buffers with 1024X512 resolution.
		 And without scaler/decimator, 640X480 could be handled.
		 With Omnivision sensors, 640X480 and another 240 lines would need for color images,
		 then another 480 lines for scaler/decimator.
		 
		 Is it enough for MPEG4 driver? No.
		 MPEG4 uses minimum 14Mbyte...
	*/
	info->buf_size = width * height * 2;

	start_offset = (dma_addr_t)boot_alloc_mem;

	if (info->planes > 1) {
		// Then we must use Block memory buffer
		start_offset |= 0x20000000;
	}
	start_offset += ctrl->id * 8 * 1024 * 1024;
	if (start_offset != info->addr[0].phys_y)
		buffer_changed = 1;

	info->addr[0].phys_y = start_offset;
	if (buffer_changed) {
		if (NULL != info->addr[0].virt_y) {
			iounmap(info->addr[0].virt_y);
		}
		info->addr[0].virt_y = ioremap(info->addr[0].phys_y, 8 * 1024 * 1024);
	}
	info("alloc boot mem vir=%x, phys=%x\n", (unsigned int)info->addr[0].virt_y, (unsigned int)info->addr[0].phys_y);
#if BUFFER_X1280
	info->nr_frames = 3;
	for (bank = 1; bank < info->nr_frames; bank++) {
		info->addr[bank].virt_y = info->addr[0].virt_y + 1280 * bank;
		info->addr[bank].phys_y = info->addr[0].phys_y + 1280 * bank;
		printk("bank=%d vir_y=%x phy_y=%x\n", bank, info->addr[bank].virt_y, info->addr[bank].phys_y);
	}

	info->scw = cr_w;
	info->sch = cr_h;
	
	top = (start_offset >> 12) & 0xFFF;

/*
	Buffer structures are:
	
	+----+----+----+----+----+----+---+
	| 1280    | 1280    | 1280    |256|
	| LU(0)   | LU(1)   | LU(2)   |   |
	|         |         |         |   | 1024
	|         |         |         |   |
	+----+----+----+----+----+----+---+
	|640 |640 |640 |640 |640 |640 |256|
	|CB0 |CR0 |CB1 |CR1 |CB2 |CR2 |   |
	|    |    |    |    |    |    |   | 1024
	|    |    |    |    |    |    |   |
	|    |    |    |    |    |    |   |
	+----+----+----+----+----+----+---+
	
	
	So It can handle Max 1280 X 1024 Y data, 640 X 1024 CB, 640 X 1024 CR data.
	
*/
	if (info->planes > 1) {
		for (bank = 0; bank < info->nr_frames; bank++) {
			
			addr = &info->addr[bank];
			addr->lu_seg = addr->phys_y >> 24;
			addr->cb_seg = addr->lu_seg;
			addr->cr_seg = addr->lu_seg;
			addr->lu_left  = bank * 1280;
			addr->lu_top   = top;
			addr->lu_right = cr_w + bank * 1280;
			addr->lu_bottom= top + cr_h;
			addr->cb_left  = bank * 1280;
			addr->cb_top   = top + 1024;
			if (info->format == FORMAT_YCBCR422) {
				addr->cb_right = cr_w / 2 + bank * 1280;
				addr->cb_bottom= top + 1024 + cr_h;
				addr->cr_left  = 640 + bank * 1280;
				addr->cr_top   = top + 1024;
				addr->cr_right = 640 + cr_w /2 + bank * 1280;
				addr->cr_bottom= top + 1024 + cr_h;
			} else 
			if (info->format == FORMAT_YCBCR444) { // actualy we can't support this format
				addr->cb_right = cr_w + bank * 1280;
				addr->cb_bottom= top + 1024 + cr_h;
				addr->cr_left  = bank * 1280;
				addr->cr_top   = top + 2048;
				addr->cr_right = cr_w + bank * 1280;
				addr->cr_bottom= top + 2048 + cr_h;
			} else { /* YUV420 */
				addr->cb_right = cr_w / 2 + bank * 1280;
				addr->cb_bottom= top + 1024 + cr_h / 2;
				addr->cr_left  = 640 + bank * 1280;
				addr->cr_top   = top + 1024;
				addr->cr_right = 640 + cr_w / 2 + bank * 1280;
				addr->cr_bottom= top + 1024 + cr_h / 2;
			}
			if (ctrl->use_scaler) { // actually we can't support scaler either.
				addr->src_addr_lu = ((addr->phys_y) & 0xFF000000) | (0 << 12) | (bank * 1280);
				addr->src_addr_cb = ((addr->phys_y) & 0xFF000000) | (1024 << 12) | (bank * 1280);
				addr->src_addr_cr = ((addr->phys_y) & 0xFF000000) | (1024 << 12) | ((bank * 1280) + 640);
				addr->dst_addr_lu = ((addr->phys_y) & 0xFF000000) | (512  << 12) | (bank * 1280);
				addr->dst_addr_cb = ((addr->phys_y) & 0xFF000000) | ((512 + 1024) << 12)  | (bank * 1280);
				addr->dst_addr_cr = ((addr->phys_y) & 0xFF000000) | ((512 + 1024) << 12) | ((bank * 1280) + 640);
				addr->vir_addr_lu = addr->virt_y + 512 * 4096;
				printk("scaler, vir_lu=%x\n", addr->vir_addr_lu);
			} else {
				addr->vir_addr_lu = addr->virt_y;
				printk("no scaler, vir_lu=%x\n", addr->vir_addr_lu);
			}
			addr->vir_addr_cb = addr->vir_addr_lu + 1024 * 4096;
			addr->vir_addr_cr = addr->vir_addr_lu + 1024 * 4096 + 640;
			printk("vir lu =%x cb = %x cr = %x\n", addr->vir_addr_lu, addr->vir_addr_cb, addr->vir_addr_cr); 
		}
	}
#else
	info->nr_frames = 4;
	for (bank = 1; bank < info->nr_frames; bank++) {
		info->addr[bank].virt_y = info->addr[0].virt_y + 1024 * bank;
		info->addr[bank].phys_y = info->addr[0].phys_y + 1024 * bank;
	}

	info->scw = cr_w;
	info->sch = cr_h;
	
	top = (start_offset >> 12) & 0xFFF;
	
/*
	Buffer structures are:
	
	+----+----+----+----+----+----+----+----+
	| 1024    | 1024    | 1024    | 1024    |
	| LU(0)   | LU(1)   | LU(2)   | LU(3)   |
	|         |         |         |         | 512
	|         |         |         |         |
	+----+----+----+----+----+----+----+----+
	|CB0 |CR0 |CB1 |CR1 |CB2 |CR2 |CB3 |CR3 |
	|    |    |    |    |    |    |    |    |
	|    |    |    |    |    |    |    |    | 512
	|    |    |    |    |    |    |    |    |
	+----+----+----+----+----+----+----+----+
	
	
	So It can handle Max 1024 X 512 Y data, 512 X 512 CB, 512 X 512 CR data.
	
*/

	if (info->planes > 1) {
		for (bank = 0; bank < info->nr_frames; bank++) {
			addr = &info->addr[bank];
			addr->lu_seg = addr->phys_y >> 24;
			addr->cb_seg = addr->lu_seg;
			addr->cr_seg = addr->lu_seg;
			addr->lu_left  = bank * 1024;
			addr->lu_top   = top;
			addr->lu_right = cr_w + bank * 1024;
			addr->lu_bottom= top + cr_h;
			addr->cb_left  = bank * 1024;
			addr->cb_top   = top + 512;
			if (info->format == FORMAT_YCBCR422) {
				addr->cb_right = cr_w / 2 + bank * 1024;
				addr->cb_bottom= top + 512 + cr_h;
				addr->cr_left  = 512 + bank * 1024;
				addr->cr_top   = top + 512;
				addr->cr_right = 512 + cr_w /2 + bank * 1024;
				addr->cr_bottom= top + 512 + cr_h;
			} else 
			if (info->format == FORMAT_YCBCR444) {
				addr->cb_right = cr_w + bank * 1024;
				addr->cb_bottom= top + 512 + cr_h;
				addr->cr_left  = bank * 1024;
				addr->cr_top   = top + 1024;
				addr->cr_right = cr_w + bank * 1024;
				addr->cr_bottom= top + 1024 + cr_h;
			} else { /* YUV420 */
				addr->cb_right = cr_w / 2 + bank * 1024;
				addr->cb_bottom= top + 512 + cr_h / 2;
				addr->cr_left  = 512 + bank * 1024;
				addr->cr_top   = top + 512;
				addr->cr_right = 512 + cr_w / 2 + bank * 1024;
				addr->cr_bottom= top + 512 + cr_h / 2;
			}
			if (ctrl->use_scaler) {
				addr->src_addr_lu = ((addr->phys_y) & 0xFF000000) | (0 << 12) | (bank * 1024);
				addr->src_addr_cb = ((addr->phys_y) & 0xFF000000) | (512 << 12) | (bank * 1024);
				addr->src_addr_cr = ((addr->phys_y) & 0xFF000000) | (512 << 12) | ((bank * 1024) + 512);
				addr->dst_addr_lu = ((addr->phys_y) & 0xFF000000) | (1024 << 12) | (bank * 1024);
				addr->dst_addr_cb = ((addr->phys_y) & 0xFF000000) | ((512 + 1024) << 12)  | (bank * 1024);
				addr->dst_addr_cr = ((addr->phys_y) & 0xFF000000) | ((512 + 1024) << 12) | ((bank * 1024) + 512);
				addr->vir_addr_lu = addr->virt_y + ctrl->scaler->offset;
			} else {
				addr->vir_addr_lu = addr->virt_y;	
			}
			addr->vir_addr_cb = addr->vir_addr_lu + 512 * 4096;
			addr->vir_addr_cr = addr->vir_addr_lu + 512 * 4096 + 512;
		}
	}
#endif

	memset(info->addr[0].virt_y, 0, 8 * 1024 * 1024);

	return 0;
}

void nx_vip_free_output_memory(struct nx_vip_out_frame *info)
{
    info->addr[0].virt_y = NULL;
    info->addr[0].phys_y = 0;
    info->addr[1].virt_y = NULL;
    info->addr[1].phys_y = 0;
    info->addr[2].virt_y = NULL;
    info->addr[2].phys_y = 0;
    info->addr[3].virt_y = NULL;
    info->addr[3].phys_y = 0;
}
void nx_vip_set_output_address(struct nx_vip_control *ctrl)
{
}

static void nx_vip_setup_memory_region(struct nx_vip_control *ctrl, int setup_all)
{
	int bank = ctrl->buf_index;
	int clip_offset = 0;
	struct nx_vip_frame_addr *addr;
	int ww, hh;

	if (ctrl->use_clipper)
		clip_offset = 0;
	ww = ctrl->in_cam->cur_width;
	hh = ctrl->in_cam->cur_height;

	if (setup_all || ctrl->out_frame.planes == 1) {
		ctrl->regs->CLIP_BASEADDRH = ctrl->out_frame.addr[bank].phys_y >> 16;
		ctrl->regs->CLIP_BASEADDRL = ctrl->out_frame.addr[bank].phys_y & 0xFFFF;
	}
	addr = &ctrl->out_frame.addr[bank];
    if (setup_all || ctrl->out_frame.planes > 1) {
		if (setup_all) {
			ctrl->regs->CLIP_LUSEG	 = addr->lu_seg;
			ctrl->regs->CLIP_CRSEG	 = addr->cr_seg;
			ctrl->regs->CLIP_CBSEG	 = addr->cb_seg;
			ctrl->regs->DECI_LUSEG	 = addr->lu_seg;
			ctrl->regs->DECI_CRSEG	 = addr->cr_seg;
			ctrl->regs->DECI_CBSEG	 = addr->cb_seg;
		}
		
		if (ctrl->use_clipper) {
			ctrl->regs->CLIP_LULEFT	 = addr->lu_left;
			ctrl->regs->CLIP_LUTOP	 = addr->lu_top + clip_offset;
			ctrl->regs->CLIP_LURIGHT = addr->lu_left + ww;
			ctrl->regs->CLIP_LUBOTTOM= addr->lu_bottom + clip_offset;

			ctrl->regs->CLIP_CBLEFT	 = addr->cb_left;
			ctrl->regs->CLIP_CBTOP	 = addr->cb_top + clip_offset;
			ctrl->regs->CLIP_CBRIGHT = addr->cb_right;
			ctrl->regs->CLIP_CBBOTTOM= addr->cb_bottom + clip_offset;
				
			ctrl->regs->CLIP_CRLEFT	 = addr->cr_left;
			ctrl->regs->CLIP_CRTOP	 = addr->cr_top + clip_offset;
			ctrl->regs->CLIP_CRRIGHT = addr->cr_right;
			ctrl->regs->CLIP_CRBOTTOM= addr->cr_bottom + clip_offset;			
		}
	}
}

/* scaler related macros */
#define HEIGHT_BITPOS	16
#define WIDTH_BITPOS	0
#define BUSY_MASK		(1 << 24)

#define SCALER_SETIMAGESIZE(dwSrcWidth, dwSrcHeight, dwDestWidth, dwDestHeight)\
{ \
	ctrl->scaler->regs->SCSRCSIZEREG = ( ( dwSrcHeight - 1 ) << HEIGHT_BITPOS ) | ( ( dwSrcWidth - 1 ) << WIDTH_BITPOS ); \
	ctrl->scaler->regs->SCDESTSIZEREG = ( ( dwDestHeight - 1 ) << HEIGHT_BITPOS ) | ( ( dwDestWidth - 1 ) << WIDTH_BITPOS ); \
	ctrl->scaler->regs->DELTAXREG = ( dwSrcWidth  * 0x10000 ) / ( dwDestWidth  );\
	ctrl->scaler->regs->DELTAYREG = ( dwSrcHeight * 0x10000 ) / ( dwDestHeight );\
}

#define SCALER_SETSRCADDR(Addr) \
{ \
	ctrl->scaler->regs->SCSRCADDREG	= Addr; \
}

#define SCALER_SETDSTADDR(Addr) \
{ \
	ctrl->scaler->regs->SCDESTADDREG = Addr; \
}
#define SCALER_ISBUSY() ( 0 != (ctrl->scaler->regs->SCINTREG & BUSY_MASK))



void nx_vip_start_vip(struct nx_vip_control *ctrl)
{
	int cam_w, cam_h;
	int out_w, out_h;
	int cr_w, cr_h;
	int cr_l, cr_t;
	int val;
 
	cam_w = ctrl->in_cam->cur_width;
	cam_h = ctrl->in_cam->cur_height;
	out_w = ctrl->out_frame.width;
	out_h = ctrl->out_frame.height;
	ctrl->use_clipper = 1;
	
	cr_w = ctrl->v4l2.crop_current.width;
	cr_h = ctrl->v4l2.crop_current.height;
	cr_l = ctrl->v4l2.crop_current.left;
	cr_t = ctrl->v4l2.crop_current.top;
	
	if ((cr_w != out_w) || (cr_h != out_h)) {
	   ctrl->use_scaler = 1;
	}
 
	if (ctrl->use_clipper) {
		//printk("start vip %d, w=%d,h=%d\n", ctrl->id, cam_w, cam_h);
		ctrl->regs->VIP_IMGWIDTH = cam_w + 2;
		ctrl->regs->VIP_IMGHEIGHT = cam_h;
		ctrl->regs->CLIP_LEFT = cr_l;
		ctrl->regs->CLIP_RIGHT = cr_l + cr_w;
		ctrl->regs->CLIP_TOP = cr_t;
		ctrl->regs->CLIP_BOTTOM = cr_t + cr_h;
	} else {
		/* Stream On */
		ctrl->scaler->started = 0;
		ctrl->regs->VIP_IMGWIDTH = cam_w + 2;
		ctrl->regs->VIP_IMGHEIGHT = cam_h;
		ctrl->regs->CLIP_LEFT = 0;
		ctrl->regs->CLIP_RIGHT = cam_w;
		ctrl->regs->CLIP_TOP = 0;
		ctrl->regs->CLIP_BOTTOM = cam_h;
	}
 
	ctrl->regs->VIP_VBEGIN = 0;
	ctrl->regs->VIP_VEND	  = 1;
	ctrl->regs->VIP_HBEGIN = 12;
	ctrl->regs->VIP_HEND	  = 18;
	ctrl->regs->CLIP_STRIDEH = 0;
	ctrl->regs->CLIP_STRIDEL = 4096;
 
	ctrl->buf_index = 0;
	nx_vip_setup_memory_region(ctrl, 1);
 
	if (ctrl->out_frame.planes > 1) {
		ctrl->regs->CLIP_YUYVENB = 0; /* Disnable YUV422 mode */
		if (ctrl->out_frame.format == FORMAT_YCBCR420) {
			ctrl->regs->CLIP_FORMAT = 0; /* YUV 4:2:0 */
		} else 
		if (ctrl->out_frame.format == FORMAT_YCBCR422) {
			ctrl->regs->CLIP_FORMAT = 1; /* YUV 4:2:2  */
		} else {
			ctrl->regs->CLIP_FORMAT = 2; /* YUV 4:4:4  */
		}
	} else {
		ctrl->regs->CLIP_YUYVENB = 1; /* Enable YUV422 linear mode */
	}
	
	if (ctrl->out_frame.planes > 1) {
		val = 0;
		if (ctrl->use_clipper)
			val |= 2;

		val |= 0x100; /* Enable Seperator */
	} else {
		val = 0;
	}
	ctrl->regs->VIP_CDENB  |= val;
	ctrl->regs->VIP_CONFIG |= 1; /* Enable VIP */
//	 printk("VIP_CDENB = %x\n", ctrl->regs->VIP_CDENB);
	FSET_CAPTURE(ctrl);
}

void nx_vip_stop_vip(struct nx_vip_control *ctrl)
{
    /* Stream Off */
    //info("stop vip\n");
	ctrl->regs->VIP_CONFIG &= ~1; /* Disable VIP */
    ctrl->regs->VIP_CDENB  &= ~(0x100 | 0x03); /* Disable Seperator, Clipper and Decimator */
	if (ctrl->use_scaler) {
		while(SCALER_ISBUSY())
			msleep(1);
	}
}

void nx_vip_restart_vip(struct nx_vip_control *ctrl)
{
	nx_vip_stop_vip(ctrl);
	nx_vip_start_vip(ctrl);
}

void nx_vip_change_resolution(struct nx_vip_control *ctrl,
					enum nx_vip_cam_res_t res)
{
	struct nx_vip_camera *cam = ctrl->in_cam;

//	nx_vip_stop_decimator(ctrl);
	nx_vip_i2c_command(ctrl, I2C_CAM_RESOLUTION, res);

	switch (res) {
	case CAM_RES_QVGA:
		info("resolution changed to QVGA (320x240) mode\n");
		cam->cur_width = 320;
		cam->cur_height = 240;
		break;

	case CAM_RES_QSVGA:
		info("resolution changed to QSVGA (400x300) mode\n");
		cam->cur_width = 400;
		cam->cur_height = 300;
		break;

	case CAM_RES_VGA:
		info("resolution changed to VGA (640x480) mode\n");
		cam->cur_width = 640;
		cam->cur_height = 480;
		break;

	case CAM_RES_SVGA:
		info("resolution changed to SVGA (800x600) mode\n");
		cam->cur_width = 800;
		cam->cur_height = 600;
		break;

	case CAM_RES_XGA:
		info("resolution changed to XGA (1024x768) mode\n");
		cam->cur_width = 1024;
		cam->cur_height = 768;
		break;

	case CAM_RES_SXGA:
		info("resolution changed to SXGA (1280x1024) mode\n");
		cam->cur_width = 1280;
		cam->cur_height = 1024;
		break;

	case CAM_RES_UXGA:
		info("resolution changed to UXGA (1600x1200) mode\n");
		cam->cur_width = 1600;
		cam->cur_height = 1200;
		break;

	case CAM_RES_DEFAULT:
		cam->cur_width = cam->def_width;
		cam->cur_height = cam->def_height;
		break;
	case CAM_RES_MAX:
		cam->cur_width = cam->max_width;
		cam->cur_height = cam->max_height;
		break;
	default:
		/* nothing to do */
		break;
	}
}

static irqreturn_t nx_vip_irq0(int irq, void *dev_id)
{
	struct nx_vip_control *ctrl = (struct nx_vip_control *) dev_id;
	struct NX_GPIO_RegisterSet *gpioa = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO);
	int status,st2;

	status = ctrl->regs->VIP_HVINT;
	st2 = ctrl->regs->VIP_ODINT;
	ctrl->regs->VIP_HVINT |= 3; /* All pending IRQ clear */
	ctrl->regs->VIP_ODINT |= 1;

#if 0	
	if ((status & 1) || (st2 & 1)) { /* VSYNC or CLIPPER/DECIMATOR DONE */
	    if (IS_CAPTURE(ctrl)) {
	        dev_dbg(ctrl->dev, "irq is in capture state\n");

			if (ctrl->time_stamp_on)
				ctrl->time_stamp[ctrl->buf_index] = jiffies;

			if (ctrl->auto_laser_ctrl)
			{
				if (ctrl->buf_index == 0)
				{
					gpioa->GPIOxOUT |= (0x01 << 20);
					printk("Laser On\n");
				}
				else if (ctrl->buf_index == 1)
				{
					gpioa->GPIOxOUT &= ~(0x01 << 20);
					printk("Laser Off\n");
				}
			}
			if (ctrl->use_scaler) {
				if (!SCALER_ISBUSY() && (ctrl->scaler->started == 0)) {
					ctrl->scaler->done = 0;
					SCALER_SETIMAGESIZE(ctrl->in_cam->clipw, ctrl->in_cam->cliph, ctrl->out_frame.width, ctrl->out_frame.height);
					SCALER_SETSRCADDR(ctrl->out_frame.addr[ctrl->buf_index].phys_y + 4 * 1024 * 1024);
					SCALER_SETDSTADDR(ctrl->out_frame.addr[ctrl->buf_index].phys_y);
					//NX_SCALER_Run();
					ctrl->scaler->regs->SCRUNREG = 1;
					ctrl->scaler->started = 1;
				}
	    	} else {
	    		FSET_HANDLE_IRQ(ctrl);
			}
	        ctrl->buf_index++;
	        ctrl->buf_index &= 0x03;
	        nx_vip_setup_memory_region(ctrl, 0);

			if (!ctrl->use_scaler)
				wake_up_interruptible(&ctrl->waitq);
	    }
	}
#else
	if ((status & 1)) { /* VSYNC */
	    if (IS_CAPTURE(ctrl)) {
	        dev_dbg(ctrl->dev, "irq is in capture state\n");

			if (ctrl->time_stamp_on)
				ctrl->time_stamp[ctrl->buf_index] = jiffies;

			if (ctrl->auto_laser_ctrl)
			{
				if (ctrl->buf_index == 0)
				{
					gpioa->GPIOxOUT |= (0x01 << 20);
					printk("Laser On\n");
				}
				else if (ctrl->buf_index == 1)
				{
					gpioa->GPIOxOUT &= ~(0x01 << 20);
					printk("Laser Off\n");
				}
			}
			if (ctrl->use_clipper == 0) {
				FSET_HANDLE_IRQ(ctrl);
				ctrl->buf_index++;
				if (ctrl->buf_index >= ctrl->out_frame.nr_frames)
					ctrl->buf_index = 0;
				while (ctrl->out_frame.skip_frames[ctrl->buf_index]) {
					ctrl->buf_index++;
					if (ctrl->buf_index >= ctrl->out_frame.nr_frames)
						ctrl->buf_index = 0;
				}
				nx_vip_setup_memory_region(ctrl, 0);
				wake_up_interruptible(&ctrl->waitq);
			}
		}
	}
	if (ctrl->use_clipper) {
		if ((st2 & 1)) { /* CLIPPER/DECIMATOR DONE */
			FSET_HANDLE_IRQ(ctrl);
			
	        ctrl->buf_index++;
	        if (ctrl->buf_index >= ctrl->out_frame.nr_frames)
				ctrl->buf_index = 0;
	        while (ctrl->out_frame.skip_frames[ctrl->buf_index]) {
				ctrl->buf_index++;
				if (ctrl->buf_index >= ctrl->out_frame.nr_frames)
					ctrl->buf_index = 0;
			}
	        nx_vip_setup_memory_region(ctrl, 0);

			wake_up_interruptible(&ctrl->waitq);
	    }
	}
#endif	
	if (status & 2) { /* HSYNC */
	    //printk("hsync0\n");
	    //gpioa->GPIOxOUT = 0x01;
	}
    //gpioa->GPIOxOUT = 0x0000;

	return IRQ_HANDLED;
}

static irqreturn_t nx_vip_irq1(int irq, void *dev_id)
{
	struct nx_vip_control *ctrl = (struct nx_vip_control *) dev_id;
	struct NX_GPIO_RegisterSet *gpioa = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO);
	int status,st2;

	status = ctrl->regs->VIP_HVINT;
	st2 = ctrl->regs->VIP_ODINT;
	ctrl->regs->VIP_HVINT |=  3; /* All pending IRQ clear */
	ctrl->regs->VIP_ODINT |=  1;

#if 0
	if ((status & 1) || (st2 & 1)) { /* VSYNC or CLIPPER DONE */
	    if (IS_CAPTURE(ctrl)) {
	        dev_dbg(ctrl->dev, "irq is in capture state\n");

			if (ctrl->time_stamp_on)
				ctrl->time_stamp[ctrl->buf_index] = jiffies;

			if (ctrl->auto_laser_ctrl)
			{
				if (ctrl->buf_index == 0)
				{
					gpioa->GPIOxOUT |= (0x01 << 20);
				}
				else if (ctrl->buf_index == 1)
				{
					gpioa->GPIOxOUT &= ~(0x01 << 20);
				}
			}
			if (ctrl->use_scaler) {
				if (!SCALER_ISBUSY() && (ctrl->sc_started == 0)) {
					ctrl->sc_done = 0;
					SCALER_SETIMAGESIZE(ctrl->in_cam->clipw, ctrl->in_cam->cliph, ctrl->out_frame.width, ctrl->out_frame.height);
					SCALER_SETSRCADDR(ctrl->out_frame.addr[ctrl->buf_index].phys_y + 4 * 1024 * 1024);
					SCALER_SETDSTADDR(ctrl->out_frame.addr[ctrl->buf_index].phys_y);
					//NX_SCALER_Run();
					ctrl->sc_regs->SCRUNREG = 1;
					ctrl->sc_started = 1;
				}
	    	} else {
	    		FSET_HANDLE_IRQ(ctrl);
			}
	        ctrl->buf_index++;
	        ctrl->buf_index &= 0x03;
	        nx_vip_setup_memory_region(ctrl, 0);

			if (!ctrl->use_scaler)
				wake_up_interruptible(&ctrl->waitq);
	    }

	}
#endif
	if ((status & 1)) { /* VSYNC */
	    if (IS_CAPTURE(ctrl)) {
	        dev_dbg(ctrl->dev, "irq is in capture state\n");

			if (ctrl->time_stamp_on)
				ctrl->time_stamp[ctrl->buf_index] = jiffies;

			if (ctrl->auto_laser_ctrl)
			{
				if (ctrl->buf_index == 0)
				{
					gpioa->GPIOxOUT |= (0x01 << 20);
					printk("Laser On\n");
				}
				else if (ctrl->buf_index == 1)
				{
					gpioa->GPIOxOUT &= ~(0x01 << 20);
					printk("Laser Off\n");
				}
			}
			if (ctrl->use_clipper == 0) {
				FSET_HANDLE_IRQ(ctrl);
				ctrl->buf_index++;
				if (ctrl->buf_index >= ctrl->out_frame.nr_frames)
					ctrl->buf_index = 0;
				while (ctrl->out_frame.skip_frames[ctrl->buf_index]) {
					ctrl->buf_index++;
					if (ctrl->buf_index >= ctrl->out_frame.nr_frames)
						ctrl->buf_index = 0;
				}
				nx_vip_setup_memory_region(ctrl, 0);
				wake_up_interruptible(&ctrl->waitq);
			}
		}
	}
	if (ctrl->use_clipper) {
		if ((st2 & 1)) { /* CLIPPER/DECIMATOR DONE */
			FSET_HANDLE_IRQ(ctrl);
			
			ctrl->buf_index++;
			if (ctrl->buf_index >= ctrl->out_frame.nr_frames)
				ctrl->buf_index = 0;
			while (ctrl->out_frame.skip_frames[ctrl->buf_index]) {
				ctrl->buf_index++;
				if (ctrl->buf_index >= ctrl->out_frame.nr_frames)
					ctrl->buf_index = 0;
			}
			nx_vip_setup_memory_region(ctrl, 0);
			wake_up_interruptible(&ctrl->waitq);
		}
	}


	return IRQ_HANDLED;
}

static irqreturn_t nx_scaler_irq(int irq, void *dev_id)
{
	struct nx_scaler *scaler = (struct nx_scaler *) dev_id;
	struct nx_vip_control *ctrl = scaler->current_ctrl;
	int bank = ctrl->scaler->current_bank;
	int width  = ctrl->out_frame.width;
	int height = ctrl->out_frame.height;
	int clipw = ctrl->out_frame.scw;
	int cliph = ctrl->out_frame.sch;
	struct nx_vip_frame_addr *addr;

	scaler->regs->SCINTREG |= 0x100;
	if (!IS_CAPTURE(ctrl))
	{		scaler->started = 0;
		return IRQ_HANDLED;
	}
	scaler->done++;
	addr = &ctrl->out_frame.addr[bank];
	if (ctrl->out_frame.format == FORMAT_YCBCR422) {
		clipw >>= 1;
		width >>=1;
	} else
	if (ctrl->out_frame.format == FORMAT_YCBCR420) {
		clipw >>=1;
		width >>=1;
		cliph >>=1;
		height >>=1;
	} else
	if (ctrl->out_frame.format == FORMAT_YCBCR444) {
		// do nothing...
	}

	switch (scaler->done) {
	case 1:	// Y complete
		SCALER_SETIMAGESIZE(clipw, cliph, width, height);
		SCALER_SETSRCADDR(addr->src_addr_cb);
		SCALER_SETDSTADDR(addr->dst_addr_cb);
		ctrl->scaler->regs->SCRUNREG = 1;
		break;
	case 2: // Cb complete
		SCALER_SETIMAGESIZE(clipw, cliph, width, height);
		SCALER_SETSRCADDR(addr->src_addr_cr);
		SCALER_SETDSTADDR(addr->dst_addr_cr);
		ctrl->scaler->regs->SCRUNREG = 1;
		break;
	case 3: // Cr complete
		if (ctrl->auto_laser_ctrl)
		{
			if (ctrl->buf_index == 3)
			{
				FSET_SCALER_IRQ(ctrl);
			}
		}
		else
		{
			FSET_SCALER_IRQ(ctrl);
		}

		scaler->started = 0;

		wake_up_interruptible(&scaler->waitq);
		break;
	}

	return IRQ_HANDLED;
}

static void nx_vip_enable(struct nx_vip_control *ctrl)
{
	/* Enable Clock */
	ctrl->regs->VIPCLKENB = 0x0F; /* PCLK_ALWAYS and BCLK_DYNAMIC */
	ctrl->regs->VIPCLKGEN[0][0] = 0x8000 | (0x03 << 2); /* OUTDISABLE, ICLK */
	/* Configuration */
	ctrl->regs->VIP_CONFIG = 0x02 | (0x00 << 2); /* 8bit interface, CrYCbY order */
}

static
struct nx_vip_control *nx_vip_register_controller(struct platform_device *pdev)
{
	struct nx_platform_vip *pdata;
	struct nx_vip_control *ctrl;
	struct resource *res;
	//int i = NX_VIP_MAX_CTRLS - 1;
	int i;
	int id = pdev->id;

	pdata = to_vip_plat(&pdev->dev);

	ctrl = &nx_vip.ctrl[id];
	ctrl->id = id;
	ctrl->dev = &pdev->dev;
	ctrl->vd = &nx_vip_video_device[id];
	ctrl->rot90 = 0;
	ctrl->vd->minor = id;
	ctrl->out_frame.nr_frames = pdata->buff_count;
	//ctrl->out_frame.skip_frames = pdata->skip_count;
	for (i=0; i< NX_VIP_MAX_FRAMES; i++)
		ctrl->out_frame.skip_frames[i] = 0;
	
	/* scaler is only one, so two ctrl must reference same scaler structure.*/
	ctrl->scaler = &nx_vip.scaler;
	ctrl->streamon = 0;

	sprintf(ctrl->name, "%s%d", NX_VIP_NAME, id);
	strcpy(ctrl->vd->name, ctrl->name);

	atomic_set(&ctrl->in_use, 0);
	mutex_init(&ctrl->lock);
	init_waitqueue_head(&ctrl->waitq);
	if (id == 0)
		init_waitqueue_head(&ctrl->scaler->waitq);

	/* get resource for io memory */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err("failed to get io memory region\n");
		return NULL;
	}

	/* request mem region */
	res = request_mem_region(res->start, res->end - res->start + 1, pdev->name);
	if (!res) {
		err("failed to request io memory region\n");
	    return NULL;
	}

	/* ioremap for register block */
	ctrl->regs = (struct NX_VIP_RegisterSet*)ioremap(res->start, res->end - res->start + 1);

	if (!ctrl->regs) {
		err("failed to remap io region\n");
		return NULL;
	}

	ctrl->use_scaler = 0;
	if (id == 0) { 
		ctrl->scaler->done = 0;
		ctrl->scaler->started = 0;
		ctrl->scaler->regs = (struct NX_SCALER_RegisterSet*)IO_ADDRESS(PHY_BASEADDR_SCALER_MODULE);//, sizeof(struct NX_SCALER_RegisterSet));
	}

	/* irq */
	ctrl->irq = platform_get_irq(pdev, 0);

	/*if (id) {
		if (request_irq(ctrl->irq, nx_vip_irq1, IRQF_DISABLED, ctrl->name, ctrl))
			err("request_irq failed\n");
	} else {
	    if (request_irq(ctrl->irq, nx_vip_irq0, IRQF_DISABLED, ctrl->name, ctrl))
		    err("request_irq failed\n");
	}*/

#if 0
	/* Enable Clock */
	ctrl->regs->VIPCLKENB = 0x0F; /* PCLK_ALWAYS and BCLK_DYNAMIC */
	ctrl->regs->VIPCLKGEN[0][0] = 0x8000 | (0x03 << 2); /* OUTDISABLE, ICLK */
	/* Configuration */
	ctrl->regs->VIP_CONFIG = 0x02 | (0x00 << 2); /* 8bit interface, CrYCbY order */
#else
	nx_vip_enable(ctrl);
#endif
	return ctrl;
}

static int nx_vip_unregister_controller(struct platform_device *pdev)
{
	struct nx_vip_control *ctrl;
	struct nx_platform_vip *pdata;
	int id = pdev->id;

	ctrl = &nx_vip.ctrl[id];

	nx_vip_free_output_memory(&ctrl->out_frame);

	pdata = to_vip_plat(ctrl->dev);

	iounmap(ctrl->regs);

	memset(ctrl, 0, sizeof(*ctrl));

	return 0;
}

static int nx_vip_mmap(struct file* filp, struct vm_area_struct *vma)
{
	struct nx_vip_control *ctrl = filp->private_data;
	struct nx_vip_out_frame *frame = &ctrl->out_frame;

	u32 size = vma->vm_end - vma->vm_start;
	u32 pfn, total_size = frame->buf_size;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_flags |= VM_RESERVED;

	/* page frame number of the address for a source frame to be stored at. */
	pfn = __phys_to_pfn(frame->addr[vma->vm_pgoff].phys_y);

	if (size > total_size) {
		err("the size of mapping is too big\n");
		return -EINVAL;
	}

	if ((vma->vm_flags & VM_WRITE) && !(vma->vm_flags & VM_SHARED)) {
		err("writable mapping must be shared\n");
		return -EINVAL;
	}

	if (remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot)) {
		err("mmap fail\n");
		return -EINVAL;
	}

	return 0;
}

static u32 nx_vip_poll(struct file *filp, poll_table *wait)
{
	struct nx_vip_control *ctrl = filp->private_data;
	u32 mask = 0;

	poll_wait(filp, &ctrl->waitq, wait);

	if (IS_IRQ_HANDLING(ctrl))
		mask = POLLIN | POLLRDNORM;

	//FSET_STOP(ctrl);

	return mask;
}

static
ssize_t nx_vip_read(struct file *filp, char *buf, size_t count, loff_t *pos)
{
	struct nx_vip_control *ctrl = filp->private_data;
	size_t end;
	char *ptr, *pp, *bf;
	int ww,hh, i;
	int ret;
	int bank;
	struct nx_vip_frame_addr *addr;

#if 0
	if (ctrl->use_scaler) {
		if (!IS_IRQ_HANDLING(ctrl) || ctrl->sc_done != 3) {
			if (wait_event_interruptible(ctrl->waitq, IS_IRQ_HANDLING(ctrl) && (ctrl->sc_done == 3)))
					return -ERESTARTSYS;
		}
		FSET_STOP(ctrl);
	} else {
		if (!IS_IRQ_HANDLING(ctrl)) {
			if (wait_event_interruptible(ctrl->waitq, IS_IRQ_HANDLING(ctrl)))
					return -ERESTARTSYS;
		}

		FSET_STOP(ctrl);
	}
#else
	if (!IS_IRQ_HANDLING(ctrl)) {
		if (wait_event_interruptible(ctrl->waitq, IS_IRQ_HANDLING(ctrl)))
				return -ERESTARTSYS;
	}
#endif
	end = min_t(size_t, ctrl->out_frame.buf_size, count);
	//ptr = nx_vip_get_current_frame(ctrl);
	bank = (ctrl->buf_index-1);
	if (bank < 0)
		bank = ctrl->out_frame.nr_frames-1;
	// dequeue buffer
	ctrl->out_frame.skip_frames[bank] = 1;
	addr = &ctrl->out_frame.addr[bank];
	FSET_STOP(ctrl);

	if (ctrl->use_scaler) {
		// start scaler
		//printk("USE SCALER!!\n");
		ctrl->scaler->current_bank = bank;
		ctrl->scaler->current_ctrl = ctrl;
		UNMASK_SCALER(ctrl);
		if (ctrl->scaler->started) {
			printk("Scaler started?\n");
			/* if another process using scaler, wait till it goes free. */
			if (wait_event_interruptible(ctrl->scaler->waitq, IS_SCALER_HANDLING(ctrl)))
				return -ERESTARTSYS;
		}
		if (!SCALER_ISBUSY() && (ctrl->scaler->started == 0)) {
			ctrl->scaler->done = 0;
			SCALER_SETIMAGESIZE(ctrl->in_cam->cur_width, ctrl->in_cam->cur_height, ctrl->out_frame.width, ctrl->out_frame.height);
			SCALER_SETSRCADDR(addr->src_addr_lu);
			SCALER_SETDSTADDR(addr->dst_addr_lu);
			//printk("0: src=%x, dst=%x\n", ctrl->out_frame.addr[bank].phys_y, ctrl->out_frame.addr[bank].phys_y + ctrl->scaler->offset);
			/*printk("run:%X\n", ctrl->scaler->regs->SCRUNREG);
			printk("cfg:%X\n", ctrl->scaler->regs->SCCFGREG	);
			printk("int:%X\n", ctrl->scaler->regs->SCINTREG	);
			printk("srca:%X\n", ctrl->scaler->regs->SCSRCADDREG);
			printk("srcs:%X\n", ctrl->scaler->regs->SCSRCSIZEREG);
			printk("dsta:%X\n", ctrl->scaler->regs->SCDESTADDREG);
			printk("dsts:%X\n", ctrl->scaler->regs->SCDESTSIZEREG);
			printk("dex:%X\n", ctrl->scaler->regs->DELTAXREG);
			printk("dey:%X\n", ctrl->scaler->regs->DELTAYREG);
			printk("hys:%X\n", ctrl->scaler->regs->HVSOFTREG);
			printk("clk:%X\n", ctrl->scaler->regs->CLKENB); */
			ctrl->scaler->regs->SCINTREG |= 0x100;
			ctrl->scaler->regs->SCINTREG |= 0x10000;
			ctrl->scaler->regs->SCRUNREG = 1;
			ctrl->scaler->started = 1;
			if (wait_event_interruptible(ctrl->scaler->waitq, IS_SCALER_HANDLING(ctrl)))
				return -ERESTARTSYS;
		} else {
			// something goes weird. Restart?
			return -EAGAIN;
		}
		
		UNMASK_SCALER(ctrl);

		// queue buffer
		ctrl->out_frame.skip_frames[bank] = 0;
		ptr = addr->vir_addr_lu;
	} else {
		ptr = addr->virt_y;
	}
	/* This is bit complex that stride is 4096 */
	ww = ctrl->out_frame.width;
	hh = ctrl->out_frame.height;
	pp = ptr;
	bf = buf;
	if (1 == ctrl->out_frame.planes) {
	    ww *= 2;
	    for (i=0; i<hh; i++) {
	        ret = copy_to_user(bf, pp, ww);
	        bf += ww;
	        pp += 4096;
	    }
	} else {
	    /* Copy Y component */
	    //printk("ptr=%x\n", ptr);
	    for (i=0; i<hh; i++) {
	        ret = copy_to_user(bf, pp, ww);
	        bf += ww;
	        pp += 4096;
	    }
	    //pp = ptr + ctrl->out_frame.addr[bank].cb_top * 4096 +
	    //		ctrl->out_frame.addr[bank].cb_left;
	    pp = addr->vir_addr_cb;
	    //printk("ptr2=%x\n", pp);
	    /* Copy Cr/Cb component */
	    if (FORMAT_YCBCR420 == ctrl->out_frame.format) {
	        hh >>= 1;
	    }
	    ww >>= 1;
	    for (i=0; i<hh; i++) {
	        ret = copy_to_user(bf, pp, ww);
	        bf += ww;
	        pp += 4096;
	    }
	    //pp = ptr + ctrl->out_frame.addr[bank].cr_top * 4096 +
	    //		ctrl->out_frame.addr[bank].cr_left;
	    pp = addr->vir_addr_cr;
	    //printk("ptr3=%x\n", pp);
	    for (i=0; i<hh; i++) {
	        ret = copy_to_user(bf, pp, ww);
	        bf += ww;
	        pp += 4096;
	    }
	}
	if (!ctrl->use_scaler) {
		// queue buffer for not using scaler.
		ctrl->out_frame.skip_frames[bank] = 0;
	} 
		
	if (ctrl->time_stamp_on)
	{
		if (count >= (ctrl->out_frame.buf_size + sizeof(unsigned long)))
			ret = copy_to_user(bf, &ctrl->time_stamp[(ctrl->buf_index-1) & 0x03], sizeof(unsigned long));
	}

	return end;
}

static
ssize_t nx_vip_write(struct file *filp, const char *b, size_t c, loff_t *offset)
{
	return 0;
}

static void nx_vip_init_vip_hw(struct nx_vip_control *ctrl)
{
	u32 reg;
	struct nx_platform_vip *pdata;

	pdata = to_vip_plat(ctrl->dev);
	if (pdata->cfg_gpio)
		pdata->cfg_gpio();

	/* Enable IRQ ? */
	ctrl->regs->VIP_VIP1 = ctrl->source_sel; /* This would set VIP0 for CAM0, VIP1 fir CAM1. */
	reg = ctrl->regs->VIP_HVINT;
	//ctrl->regs->VIP_HVINT = reg | 0x100; /* Enable V sync IRQ */
	ctrl->regs->VIP_HVINT &= ~0x300; /* disable h/vsync */
	if (ctrl->use_clipper)
		ctrl->regs->VIP_ODINT = 0x100;
	else
		ctrl->regs->VIP_ODINT = 0x100; /* Disable Clipper/Decimator Complete IRQ */
	ctrl->regs->VIP_HVINT |= 3; /* All pending IRQ clear */
	ctrl->regs->VIP_ODINT |= 1;
	//nx_vip_reset(ctrl);
	reg = ctrl->scaler->regs->SCINTREG;
	ctrl->scaler->regs->SCINTREG = reg | 0x100;
	// Interrupt enable...
	ctrl->scaler->regs->SCINTREG = reg | 0x10000;
}

static void nx_vip_init_laser_port()
{
	struct NX_GPIO_RegisterSet *gpioa = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO);

	gpioa->GPIOxOUTENB = (0x01 << 20);
    gpioa->GPIOxALTFN[1] &= (~(3 << 8));
}

static void nx_vip_init_scaler(struct nx_vip_control *ctrl)
{
	ctrl->scaler->regs->SCRUNREG		= 0x00000000;
	ctrl->scaler->regs->SCCFGREG		= 0x00000000;
	ctrl->scaler->regs->SCINTREG		= 0x00000100;
	ctrl->scaler->regs->SCSRCADDREG		= 0x00000000;
	ctrl->scaler->regs->SCSRCSIZEREG	= 0x00000000;
	ctrl->scaler->regs->SCDESTADDREG	= 0x00000000;
	ctrl->scaler->regs->SCDESTSIZEREG	= 0x00000000;
	ctrl->scaler->regs->DELTAXREG		= 0x00000000;
	ctrl->scaler->regs->DELTAYREG		= 0x00000000;
	ctrl->scaler->regs->HVSOFTREG		= 0x00000000;
	ctrl->scaler->regs->CLKENB			= 0x00000000;

	ctrl->scaler->irq = INTNUM_OF_SCALER_MODULE; //NX_SCALER_GetInterruptNumber();

	ctrl->scaler->regs->CLKENB	 = 0x0000000B;
	ctrl->scaler->regs->SCRUNREG = 0;

	ctrl->scaler->regs->SCCFGREG = 0x00000003;

	ctrl->scaler->done = 0;
	ctrl->scaler->started = 0;
}

static int nx_vip_open(struct file *filp) /* Theres no inode information since Ver 2.X */
{
	struct nx_vip_control *ctrl;
	int id, ret;

	//id = MINOR(inode->i_rdev); /* now inode info is not from Open... */
	id = iminor(filp->f_path.dentry->d_inode);
	ctrl = &nx_vip.ctrl[id];

	mutex_lock(&ctrl->lock);

	if (atomic_read(&ctrl->in_use)) {
		ret = -EBUSY;
		goto resource_busy;
	} else {
		atomic_inc(&ctrl->in_use);
		//nx_vip_reset(ctrl);
		filp->private_data = ctrl;
	}

	mutex_unlock(&ctrl->lock);
	// Now set active camera here... for 0? No. id
	if (0 == nx_vip_set_active_camera(ctrl, id)) {
		nx_vip_init_camera(ctrl);
		{
#if 0
			u32 reg;

			/* Enable IRQ ? */
			ctrl->regs->VIP_VIP1 = ctrl->source_sel; /* This would set VIP0 for CAM0, VIP1 fir CAM1. */
			reg = ctrl->regs->VIP_HVINT; 
			//ctrl->regs->VIP_HVINT = reg | 0x100; /* Enable V sync IRQ */
			ctrl->regs->VIP_HVINT &= ~0x300; /* disable h/vsync */
			if (ctrl->use_clipper)
				ctrl->regs->VIP_ODINT = 0x100;
			else
				ctrl->regs->VIP_ODINT = 0x100; /* Disable Clipper/Decimator Complete IRQ */
			ctrl->regs->VIP_HVINT |= 3; /* All pending IRQ clear */
			ctrl->regs->VIP_ODINT |= 1;
			//nx_vip_reset(ctrl);
			reg = ctrl->scaler->regs->SCINTREG;
			ctrl->scaler->regs->SCINTREG = reg | 0x100;
			// Interrupt enable...
			ctrl->scaler->regs->SCINTREG = reg | 0x10000;
#else
			nx_vip_init_vip_hw(ctrl);
			if (ctrl->id == 0)
				nx_vip_init_scaler(ctrl);
#endif
		}
		if (ctrl->auto_laser_ctrl)
		{
			nx_vip_init_laser_port();
		}

		if (id) {
			if (request_irq(ctrl->irq, nx_vip_irq1, IRQF_DISABLED, ctrl->name, ctrl))
				err("VIP1 request_irq failed\n");
		} else {
			if (request_irq(ctrl->irq, nx_vip_irq0, IRQF_DISABLED, ctrl->name, ctrl))
				err("VIP0 request_irq failed\n");
		}
	} else {
		return -EBUSY;
	}
#if 0
	/* Enable Clock */
	ctrl->regs->VIPCLKENB = 0x0F; /* PCLK_ALWAYS and BCLK_DYNAMIC */
	ctrl->regs->VIPCLKGEN[0][0] = 0x8000 | (0x03 << 2); /* OUTDISABLE, ICLK */
	/* Configuration */
	ctrl->regs->VIP_CONFIG = 0x02 | (0x00 << 2); /* 8bit interface, CrYCbY order */
#else
	nx_vip_enable(ctrl);
#endif
	return 0;

resource_busy:
	mutex_unlock(&ctrl->lock);
	return ret;
}

static int nx_vip_release(struct file *filp)
{
	struct nx_vip_control *ctrl;
	int id;

	id = iminor(filp->f_path.dentry->d_inode);
	ctrl = &nx_vip.ctrl[id];

	mutex_lock(&ctrl->lock);

	atomic_dec(&ctrl->in_use);
	filp->private_data = NULL;

	mutex_unlock(&ctrl->lock);
	
	nx_vip_stop_vip(ctrl);
	
	free_irq(ctrl->irq, ctrl);

	//if ((atomic_read(&nx_vip.ctrl[0].in_use)==0) && (atomic_read(&nx_vip.ctrl[1].in_use)==0))
		

	return 0;
}

static const struct v4l2_file_operations nx_vip_fops = {
	.owner = THIS_MODULE,
	.open = nx_vip_open,
	.release = nx_vip_release,
	.ioctl = video_ioctl2,
	.read = nx_vip_read,
	.write = nx_vip_write,
	.mmap = nx_vip_mmap,
	.poll = nx_vip_poll,
};

static void nx_vip_vdev_release(struct video_device *vdev)
{
	kfree(vdev);
}

struct video_device nx_vip_video_device[NX_VIP_MAX_CTRLS] = {
	[0] = {
		.vfl_type = 0, /* VID_TYPE_OVERLAY | VID_TYPE_CAPTURE | VID_TYPE_CLIPPING | VID_TYPE_SCALES */
		.fops = &nx_vip_fops,
		.ioctl_ops = &nx_vip_v4l2_ops,
		.release  = nx_vip_vdev_release,
	},
	[1] = {
		.vfl_type = 0, /* VID_TYPE_OVERLAY | VID_TYPE_CAPTURE | VID_TYPE_CLIPPING | VID_TYPE_SCALES */
		.fops = &nx_vip_fops,
		.ioctl_ops = &nx_vip_v4l2_ops,
		.release  = nx_vip_vdev_release,
	},
	[2] = {
		.vfl_type = 0, /* VID_TYPE_OVERLAY | VID_TYPE_CAPTURE | VID_TYPE_CLIPPING | VID_TYPE_SCALES */
		.fops = &nx_vip_fops,
		.ioctl_ops = &nx_vip_v4l2_ops,
		.release  = nx_vip_vdev_release,
	},
};

static int nx_vip_init_global(struct platform_device *pdev)
{
	/* test pattern */
	nx_vip.camera[test_pattern.id] = &test_pattern;

	return 0;
}

static int nx_vip_probe(struct platform_device *pdev)
{
	struct nx_platform_vip *pdata;
	struct nx_vip_control *ctrl;
	int ret;

	ctrl = nx_vip_register_controller(pdev);
	if (!ctrl) {
		err("cannot register nx v4l2 controller\n");
		goto err_register;
	}

	pdata = to_vip_plat(&pdev->dev);
	if (pdata->cfg_gpio)
		pdata->cfg_gpio();

	ctrl->pdata = pdata;
	ctrl->gpio_reset = pdata->gpio_reset;

	ctrl->laser_ctrl = 0;
	ctrl->time_stamp_on = pdata->time_stamp_on;
	ctrl->time_stamp_offset = -4; /* this is default. */
	ctrl->auto_laser_ctrl = pdata->laser_ctrl;
	ctrl->source_sel = pdata->in_port;

	/* things to initialize once */
	if (ctrl->id == 0) {
		/* setup no camera is registered. */
		nx_vip.camera[0] = NULL;
		nx_vip.camera[1] = NULL;
		ret = nx_vip_init_global(pdev);
		if (ret)
			goto err_global;
	}
	ret = v4l2_device_register(&pdev->dev, &ctrl->v4l2_dev);
	if (ret) {
		goto err_v4l;
	}

	/* register camera later --- when first open */
	ctrl->in_cam = NULL;

	/* scaler initialize */
	if (ctrl->id == 0) {
		nx_vip_init_scaler(ctrl);

		if (request_irq(ctrl->scaler->irq, nx_scaler_irq, IRQF_DISABLED, "nx-scaler", ctrl->scaler)) {
				err("scaler request_irq failed\n");
		}
	}

	ret = video_register_device(ctrl->vd, VFL_TYPE_GRABBER, ctrl->id);
	if (ret) {
		err("cannot register video driver\n");
		goto err_v4l;
	}

	info("controller %d registered successfully\n", ctrl->id);

	return 0;

	
err_v4l:
	v4l2_device_unregister(&ctrl->v4l2_dev);
err_global:
	nx_vip_unregister_controller(pdev);

err_register:
	return -EINVAL;

}

static int nx_vip_remove(struct platform_device *pdev)
{
	int id = pdev->id;
	struct nx_scaler *scaler;

	if (id == 0) {
		scaler = &nx_vip.scaler;

		scaler->regs->SCRUNREG			= 0x00000000;
		scaler->regs->SCCFGREG			= 0x00000000;
		scaler->regs->SCINTREG			= 0x00000100;
		scaler->regs->SCSRCADDREG		= 0x00000000;
		scaler->regs->SCSRCSIZEREG		= 0x00000000;
		scaler->regs->SCDESTADDREG		= 0x00000000;
		scaler->regs->SCDESTSIZEREG	= 0x00000000;
		scaler->regs->DELTAXREG		= 0x00000000;
		scaler->regs->DELTAYREG		= 0x00000000;
		scaler->regs->HVSOFTREG		= 0x00000000;
		scaler->regs->CLKENB			= 0x00000000;

		free_irq(scaler->irq, scaler);		
	}
	nx_vip_unregister_controller(pdev);

	return 0;
}

int nx_vip_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

int nx_vip_resume(struct platform_device *dev)
{
	struct nx_vip_control *ctrl;
	struct i2c_client *client = ctrl->in_cam->client;
	int id = dev->id;

	ctrl = &nx_vip.ctrl[id];

	info("nx vip%d resume\n", ctrl->id);

	if (ctrl->gpio_reset) {
		ctrl->gpio_reset(ctrl->pwrdn);
	}
	if (IS_CAPTURE(ctrl))
	{
		nx_vip_i2c_command(ctrl, I2C_CAM_INIT, 0);
	}
	nx_vip_init_vip_hw(ctrl);

	nx_vip_enable(ctrl);

	if (ctrl->id == 0)
		nx_vip_init_scaler(ctrl);

	if (IS_CAPTURE(ctrl))
	{
		nx_vip_start_vip(ctrl);
	}

	return 0;
}

static struct platform_driver nx_vip_driver = {
	.probe		= nx_vip_probe,
	.remove		= nx_vip_remove,
	.suspend	= nx_vip_suspend,
	.resume	    = nx_vip_resume,
	.driver		= {
		.name	= "nx-v4l2",
		.owner	= THIS_MODULE,
	},
};

static int nx_vip_register(void)
{
	platform_driver_register(&nx_vip_driver);

	return 0;
}

static void nx_vip_unregister(void)
{
	platform_driver_unregister(&nx_vip_driver);
}

module_init(nx_vip_register);
module_exit(nx_vip_unregister);

MODULE_AUTHOR("Seungwoo Kim <ksw@stcube.com>");
MODULE_DESCRIPTION("Nexell nxp2120 v4l2 driver");
MODULE_LICENSE("GPL");
