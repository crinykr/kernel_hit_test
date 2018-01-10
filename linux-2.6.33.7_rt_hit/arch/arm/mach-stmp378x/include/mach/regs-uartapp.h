/*
 * stmp378x: UARTAPP register definitions
 *
 * Copyright (c) 2008 Freescale Semiconductor
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#define REGS_UARTAPP1_BASE	(STMP3XXX_REGS_BASE + 0x6C000)
#define REGS_UARTAPP1_PHYS	0x8006C000
#define REGS_UARTAPP2_BASE	(STMP3XXX_REGS_BASE + 0x6E000)
#define REGS_UARTAPP2_PHYS	0x8006E000
#define REGS_UARTAPP_SIZE	0x2000

#define HW_UARTAPP_CTRL0	0x0
#define BM_UARTAPP_CTRL0_XFER_COUNT	0x0000FFFF
#define BP_UARTAPP_CTRL0_XFER_COUNT	0
#define BM_UARTAPP_CTRL0_RXTIMEOUT	0x07FF0000
#define BP_UARTAPP_CTRL0_RXTIMEOUT	16
#define BM_UARTAPP_CTRL0_RXTO_ENABLE	0x08000000
#define BM_UARTAPP_CTRL0_RUN	0x20000000
#define BM_UARTAPP_CTRL0_SFTRST	0x80000000
#define BM_UARTAPP_CTRL1_XFER_COUNT	0x0000FFFF
#define BP_UARTAPP_CTRL1_XFER_COUNT	0
#define BM_UARTAPP_CTRL1_RUN	0x10000000

#define HW_UARTAPP_CTRL2	0x20
#define BM_UARTAPP_CTRL2_UARTEN	0x00000001
#define BP_UARTAPP_CTRL2_UARTEN	0
#define BM_UARTAPP_CTRL2_TXE	0x00000100
#define BM_UARTAPP_CTRL2_RXE	0x00000200
#define BM_UARTAPP_CTRL2_RTS	0x00000800
#define BM_UARTAPP_CTRL2_RTSEN	0x00004000
#define BM_UARTAPP_CTRL2_CTSEN	0x00008000
#define BM_UARTAPP_CTRL2_RXDMAE	0x01000000
#define BM_UARTAPP_CTRL2_TXDMAE	0x02000000
#define BM_UARTAPP_CTRL2_DMAONERR	0x04000000

#define HW_UARTAPP_LINECTRL	0x30
#define BM_UARTAPP_LINECTRL_BRK	0x00000001
#define BP_UARTAPP_LINECTRL_BRK	0
#define BM_UARTAPP_LINECTRL_PEN	0x00000002
#define BM_UARTAPP_LINECTRL_EPS	0x00000004
#define BM_UARTAPP_LINECTRL_STP2	0x00000008
#define BM_UARTAPP_LINECTRL_FEN	0x00000010
#define BM_UARTAPP_LINECTRL_WLEN	0x00000060
#define BP_UARTAPP_LINECTRL_WLEN	5
#define BM_UARTAPP_LINECTRL_SPS	0x00000080
#define BM_UARTAPP_LINECTRL_BAUD_DIVFRAC	0x00003F00
#define BP_UARTAPP_LINECTRL_BAUD_DIVFRAC	8
#define BM_UARTAPP_LINECTRL_BAUD_DIVINT	0xFFFF0000
#define BP_UARTAPP_LINECTRL_BAUD_DIVINT	16

#define HW_UARTAPP_INTR		0x50
#define BM_UARTAPP_INTR_CTSMIS	0x00000002
#define BM_UARTAPP_INTR_RTIS	0x00000040
#define BM_UARTAPP_INTR_CTSMIEN	0x00020000
#define BM_UARTAPP_INTR_RXIEN	0x00100000
#define BM_UARTAPP_INTR_RTIEN	0x00400000

#define HW_UARTAPP_DATA		0x60

#define HW_UARTAPP_STAT		0x70
#define BM_UARTAPP_STAT_RXCOUNT	0x0000FFFF
#define BP_UARTAPP_STAT_RXCOUNT	0
#define BM_UARTAPP_STAT_FERR	0x00010000
#define BM_UARTAPP_STAT_PERR	0x00020000
#define BM_UARTAPP_STAT_BERR	0x00040000
#define BM_UARTAPP_STAT_OERR	0x00080000
#define BM_UARTAPP_STAT_RXFE	0x01000000
#define BM_UARTAPP_STAT_TXFF	0x02000000
#define BM_UARTAPP_STAT_TXFE	0x08000000
#define BM_UARTAPP_STAT_CTS	0x10000000

#define HW_UARTAPP_VERSION	0x90
