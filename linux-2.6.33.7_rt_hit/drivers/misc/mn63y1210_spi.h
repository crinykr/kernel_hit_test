#ifndef __MN63Y1210_SPI_H__
#define __MN63Y1210_SPI_H__
/*
 *  MN63Y1210 NFC driver Header for kernel/user space.
 *
 *  Copyright (c) 2014 STcube Inc.,
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
enum {
	MN_STATUS_BIT_DO_READ	=	1,
	MN_STATUS_BIT_DO_WRITE	=	2,
	MN_STATUS_BIT_DO_WAIT_RESPOND	=	4,
	MN_STATUS_BIT_DO_REQ_RESPOND	=	8,
};
// Reset
enum {
	MN_RESET_ON	=	1,
};

// Power
enum {
	MN_PWR_OFF	=	0,
	MN_PWR_ON	=	1,
};
// Read
// Write, if success return addr = 0 or non zero for failed.
typedef struct {
	short int addr;
	short int size;
	int status;
	char buffer[128];
} MN_readwrite_struc_t;

#define MN_RWSTRUC_ERROR_TIMEOUT	-1001
#define MN_RWSTRUC_ERROR_CRC		-1002
#define MN_RWSTRUC_ERROR_SIGNAL		-1003
#define MN_RWSTRUC_ERROR_WRITE		-1004
#define MN_RWSTRUC_ERROR_UNKNOWN    -1005

// Actually, tunnel read/write need callback mechanism for user space.
// See nfc_app_new.c for example to implement user space thread to service.

// TUNNEL status
enum {
	MN_TUNNEL_RFID_READ		=	0x01000000,
	MN_TUNNEL_RFID_WRITE	=	0x02000000,
	MN_TUNNEL_RFID_UNKNOWN	=	0x03000000,
	MN_TUNNEL_DATA_BUFFER   =   0x08000000,
	MN_TUNNEL_FAIL          =   0x0F000000,
};

#define TUNNEL_STATUS_MASK	0xFF000000

// TUNNEL_read
// TUNNEL_write
typedef struct {
	short int dirty;
	short int addr;
	short int size;
	short int status;
	char buffer[128];
} MN_tunnel_readwrite_struc_t;


// CUR_STATUS
#define MN_STATUS_RW_FAIL		1
#define MN_STATUS_TUNNEL_FAIL	2
/*-----------------------------------------------------------------------------
 *	IOCTL CODE
 */

enum {
	IOCTL_MN63Y1210_STATUS	= 		_IOR('Y',100, int),
	IOCTL_MN63Y1210_RESET	= 		_IOW('Y',101, int),
	IOCTL_MN63Y1210_POWER	= 		_IOWR('Y',102, int),
	IOCTL_MN63Y1210_READ	= 		_IOWR('Y',103, MN_readwrite_struc_t),
	IOCTL_MN63Y1210_WRITE	= 		_IOWR('Y',104, MN_readwrite_struc_t),
	IOCTL_MN63Y1210_TUNNEL_STATUS = _IOR('Y', 105, int),
	IOCTL_MN63Y1210_TUNNEL_READ	= 	_IOWR('Y',106, MN_tunnel_readwrite_struc_t),
	IOCTL_MN63Y1210_TUNNEL_WRITE= 	_IOWR('Y',107, MN_tunnel_readwrite_struc_t),
	IOCTL_MN63Y1210_TUNNEL_ABORT= 	_IO('Y',108),
};

#endif
