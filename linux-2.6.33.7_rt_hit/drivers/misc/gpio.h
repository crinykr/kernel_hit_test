#ifndef __GPIO_ENC_H__
#define __GPIO_ENC_H__
/*
 *  GPIO enc driver Header for kernel/user space.
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

// Status

/*-----------------------------------------------------------------------------
 *	IOCTL CODE
 */

enum {
	IOCTL_GPIO_ENC_STATUS			=	_IOR('G',100, int),
	IOCTL_GPIO_ENC_RESET			= 	_IOW('G',101, int),
	IOCTL_GPIO_PWM_DUTY_CTRL_R		=	_IOR('G',102, int),
	IOCTL_GPIO_PWM_DUTY_CTRL_W		=	_IOW('G',103, int),
	IOCTL_GPIO_PWM_PERIOD_CTRL_R	=	_IOR('G',104, int),
	IOCTL_GPIO_PWM_PERIOD_CTRL_W	=	_IOW('G',105, int),
};

typedef struct {
	unsigned short 	ch;			/* 0 ~ 1  */
	union {
		unsigned short 	duty;		/* 10 bits */
		unsigned short 	period;		/* 10 bits */
	};
} pwm_info;

#endif
