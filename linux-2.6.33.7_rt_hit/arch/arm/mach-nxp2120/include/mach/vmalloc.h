/*
 * (C) Copyright 2010
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <cfg_mem.h>

/*
 * Memory bank base		: 0x80000000
 * Memory bank max size	: 512 Mbyte
 * PAGE_OFFSET 			: 0xC0000000
 */
#define	PB_MEM_BANK_BASE	0x80000000
#define	PB_MEM_BANK_MAX		0x20000000		/* 512 MB = 0xC0000000 ~ 0xE0000000 */

#define VMALLOC_END			(PAGE_OFFSET + PB_MEM_BANK_MAX)
