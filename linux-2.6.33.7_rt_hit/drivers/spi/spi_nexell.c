/*
 * Copyright(C) 2013-2014 STcube Inc.,
 * All right reserved by Seungwoo Kim <ksw@stcube.com>
 *
 * This SPI driver derived from Nexell's code,
 * and rewritten for current hardware.
 *
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

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/spi/spi.h>

#include <mach/platform.h>
#include <mach/devices.h>
#include <mach/soc.h>

#if (0)
#define DBGOUT(msg...)		{ printk(KERN_INFO "nx-spi: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define ERROUT(msg...)		{ 	\
		printk(KERN_ERR "ERROR: %s, %s line %d: \n",	\
			__FILE__, __FUNCTION__, __LINE__),			\
		printk(KERN_ERR msg); }

#define CFG_SPI_CLK_SRC			CFG_SYS_CLKSRC_PLL1	/* 0 = PLL0, 1 = PLL1 */
#define CFG_SPI_CLK_FREQ		CFG_SYS_PLL1_FREQ
#define CFG_SPI_CLK_DIV			8					/* 1 ~ 256 */
#define CFG_SPI_CLK_DIV2		1					/* 4 ~ 32 */
#define CFG_SPI_CLK_INV			CTRUE
#define CFG_SPI_BIT_WIDTH		8

/*
  I have to test SPI function with GPIO read/write
  Because some conflict pin issue.
*/
//#define SPI_GPIO	1

/*----------------------------------------------------------------------------*/
struct spi_io_cs_info {
	int port;
	int pad;
	struct	NX_GPIO_RegisterSet   *gpio;
	void (*func)(struct spi_io_cs_info *csinfo, int mode, int enable);
};

struct spi_bus_info {
	int 				port;
	int 				irqno;
	int					io_clk;					/* clock gpio */
	int					io_rx;					/* tx gpio */
	int					io_tx;					/* rx gpio */
	struct spi_io_cs_info  	io_cs[MAX_SPI_CS_NUM];
	unsigned int		max_clk;
	unsigned int		min_clk;
};

struct spi_cs_info {
	unsigned int	speed_hz;
	unsigned int	bits_word;
	unsigned int	cs_mode;
#if defined(CONFIG_NEXELL_SPI_BY_GPIO)	
	int             udel;
#endif
};

struct spi_param {
	struct work_struct			work;
	struct workqueue_struct *	work_queue;
	spinlock_t					lock;
	struct list_head			msg_queue;
	struct spi_master 		*	master;
	struct spi_bus_info    		bus;
	struct spi_cs_info			cs_info[MAX_SPI_CS_NUM];
	int							cs_num;  
	struct	NX_SSPSPI_RegisterSet *regs;
	struct	NX_GPIO_RegisterSet   *gpioA;
	unsigned int				tr_mode;
	unsigned char			*	tx_buf;
	unsigned char			*	rx_buf;
	int							tr_count;
};

/*------------------------------------------------------------------------------
 * 	SPI local functions
 */
static void do_spi_workqueue(struct work_struct *work);
#if !defined(CONFIG_NEXELL_SPI_BY_GPIO)
static irqreturn_t do_spi_irqhandler(int irqno, void *dev_id);
#endif

/* SPI registers bit definition */
/* SSPCONT0 BITS */
#define SSPCONT0_BURST_RCV	(1 << 15)
#define SSPCONT0_BRCV_EN	(1 << 13)
#define SSPCONT0_DMA_EN		(1 << 12)
#define SSPCONT0_PIOMODE    (0 << 12)
#define SSPCONT0_SPI_ENB	(1 << 11)
#define SSPCONT0_SPI_DISB	(0 << 11)
#define SSPCONT0_FIFO_CLR	(1 << 10)
#define SSPCONT0_USE_EXTCLK	(1 << 9)
#define SSPCONT0_USE_INTCLK (0 << 9)
#define SSPCONT0_NUMBIT_MASK	(0xF << 5)
#define SSPCONT0_NUMBIT(X)	((X-1) << 5)
#define SSPCONT0_DIVCNT_MASK	(0x1F << 0)
#define SSPCONT0_DIVCNT(X)	((X-1) << 0)
/* SSPCONT1 BITS */
#define SSPCONT1_BYTESWAP	(1 << 5)
#define SSPCONT1_NO_BYTESWAP	(0 << 5)
#define SSPCONT1_BYTESWAP_MASK	(1 << 5)
#define SSPCONT1_SLAVESEL	(1 << 4)
#define SSPCONT1_MASTERSEL	(0 << 4)
#define SSPCONT1_SCLK_POL_NORMAL	(1 << 3)
#define SSPCONT1_SCLK_POL_INVERT	(0 << 3)
#define SSPCONT1_SCLK_POL_MASK	(1 << 3)
#define SSPCONT1_SCLKSH_FORMATB		(1 << 2)
#define SSPCONT1_SCLKSH_FORMATA		(0 << 2)
#define SSPCONT1_SCLKSH_FORMAT_MASK		(1 << 2)
#define SSPCONT1_MODE_SPP	(0 << 0)
#define SSPCONT1_MODE_SPI	(1 << 0)
/* SSPSTAT BITS */
#define SSPSTAT_IRQEENB		(1 << 15)
#define SSPSTAT_IRQWENB		(1 << 14)
#define SSPSTAT_IRQRENB		(1 << 13)
#define SSPSTAT_TXSHEMPTY	(1 << 8)
#define SSPSTAT_IRQE		(1 << 6)
#define SSPSTAT_IRQW		(1 << 5)
#define SSPSTAT_IRQR		(1 << 4)
#define SSPSTAT_WFFFULL		(1 << 3)
#define SSPSTAT_WFFEMPTY	(1 << 2)
#define SSPSTAT_RFFFULL		(1 << 1)
#define SSPSTAT_RFFEMPTY	(1 << 0)
/* CLKENB BITS */
#define SPI_CLKENB_PCLKMODE (1 << 3)
#define SPI_CLKENB_CLKGENENB	(1 << 2)
/* CLKGEN BITS */
#define SPI_CLKGEN_CLKDIV(X)	((X-1) << 5)
#define SPI_CLKGEN_CLKSRCSEL(X)	(X << 2)

#if defined(CONFIG_NEXELL_SPI_BY_GPIO)

#define	SPI_CLOCK_MAX			500000
#define	SPI_CLOCK_MIN			100000
#define	SPI_CLOCK_CURR			500000

#else

#define	SPI_CLOCK_MAX			(CFG_SPI_CLK_FREQ / CFG_SPI_CLK_DIV /  1)
#define	SPI_CLOCK_MIN			(CFG_SPI_CLK_FREQ / CFG_SPI_CLK_DIV / 32)
#define	SPI_CLOCK_CURR			(CFG_SPI_CLK_FREQ / CFG_SPI_CLK_DIV / CFG_SPI_CLK_DIV2)

#endif
void gpio_func(struct spi_io_cs_info *csinfo, int mode, int enable)
{
	struct	NX_GPIO_RegisterSet   *gpio = csinfo->gpio;
	unsigned int val;

	DBGOUT("pad = %x, gpio=%x\n", csinfo->pad, csinfo->gpio);
	val = gpio->GPIOxOUT;
    val &= ~csinfo->pad;
	if (mode & SPI_CS_HIGH)
		val |= enable ? csinfo->pad : 0x00000000;
	else
		val |= enable ? 0x00000000 : csinfo->pad;
	gpio->GPIOxOUT = val;	
}

static int init_spi_device_hw(struct spi_param *par, struct spi_plat_data * plat)
{
	int i;

#if !defined(CONFIG_NEXELL_SPI_BY_GPIO)
	unsigned int val;

	par->bus.max_clk = SPI_CLOCK_MAX;
	par->bus.min_clk = SPI_CLOCK_MIN;
	par->cs_num		 = plat->cs_num;

	par->regs = (struct	NX_SSPSPI_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_SSPSPI_MODULE);
	par->gpioA = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO_MODULE);
	/* setup gpio */
#if defined(CONFIG_MODEL_RK_HIT_N)

#else
	par->gpioA->GPIOxALTFN[1] &= ~0xFF000000;/* bit 28,29,30,31 */
	par->gpioA->GPIOxALTFN[1] |= 0x54000000;/* port 29,30,31 to FUNC1, 28 to GPIO */
#endif
	
	par->bus.irqno = plat->irq;

	for (i=0; i<plat->cs_num; i++) {
		if (plat->io_cs != NULL ) {
			par->bus.io_cs[i].port = (plat->io_cs[i].pad >> 16) & 0xF;
			DBGOUT("port = %d\n", par->bus.io_cs[i].port); 
			par->bus.io_cs[i].pad  = 1 << (plat->io_cs[i].pad & 0xFF);
			DBGOUT("pad = %X\n", par->bus.io_cs[i].pad);
			if (plat->io_cs[i].func) {
				par->bus.io_cs[i].func = (void *)(plat->io_cs[i].func);
			} else {
				par->bus.io_cs[i].func = gpio_func;
			}
			par->bus.io_cs[i].gpio = par->gpioA + par->bus.io_cs[i].port * OFFSET_OF_GPIO_MODULE;
			par->bus.io_cs[i].gpio->GPIOxOUTENB |= par->bus.io_cs[i].pad; /* Pad to Output */
			par->bus.io_cs[i].gpio->GPIOxOUT |= par->bus.io_cs[i].pad; /* pad default to high */
		}
	}	
	DBGOUT("gpioA ALTFN1=%x\n",par->gpioA->GPIOxALTFN[1]);
	DBGOUT("gpioA outENB=%x\n",par->gpioA->GPIOxOUTENB);
	DBGOUT("gpioA out=%x\n",par->gpioA->GPIOxOUT);
	// Setup CLKENB
	par->regs->CLKENB = SPI_CLKENB_PCLKMODE;
	// Setup CLKGEN -- 192MHz / 4 = 48MHz
	par->regs->CLKGEN = (SPI_CLKGEN_CLKDIV(CFG_SPI_CLK_DIV)) | (SPI_CLKGEN_CLKSRCSEL(1));
	// Setup CONT0
	val = 	SSPCONT0_PIOMODE | SSPCONT0_SPI_DISB | SSPCONT0_FIFO_CLR |
			SSPCONT0_USE_INTCLK | SSPCONT0_NUMBIT(8);
	val |= SSPCONT0_DIVCNT(CFG_SPI_CLK_DIV2); // 12MHz
	par->regs->CONT0 = val;
	// Now Clear FIFO_CLR bit
	val &= ~SSPCONT0_FIFO_CLR;
	par->regs->CONT0 = val;
	// Supply operation clock
	par->regs->CLKENB = SPI_CLKENB_PCLKMODE | SPI_CLKENB_CLKGENENB;
	// Seup CONT1
	val =  	SSPCONT1_NO_BYTESWAP | SSPCONT1_MASTERSEL |
			SSPCONT1_SCLK_POL_INVERT | SSPCONT1_SCLKSH_FORMATB | SSPCONT1_MODE_SPI;
	par->regs->CONT1 = val;
#else

	par->gpioA = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO_MODULE);
#if defined(CONFIG_MODEL_RK_HIT_N)
	// GPIOA0 - TX
	// GPIOA1 - RX
	// GPIOA2 - CLK
	// GPIOA3 - BUSY
	par->gpioA->GPIOxALTFN[0] &= ~0x3F; // 0-2 would be GPIO
	par->gpioA->GPIOxOUTENB &= ~0x00000007;
	par->gpioA->GPIOxOUTENB |= 0x00000005; // IN:BIT1, OUT: 0,2
	par->gpioA->GPIOxOUT |= 0x00000005; // ALL output high
#else
	// GPIOA28 - CS
	// GPIOA30 - RX
	// GPIOA31 - TX
	// GPIOA29 - CLK
	// For master interface, CLK, TX, CS would be OUTPUT
	// and TX would be input
	par->gpioA->GPIOxALTFN[1] &= ~0xFF000000; // 28-31 would be GPIO
	par->gpioA->GPIOxOUTENB &= ~0xF0000000;
	par->gpioA->GPIOxOUTENB |= 0xB0000000; // IN:30, OUT: 28,29,31
	par->gpioA->GPIOxOUT |= 0xB0000000; // ALL output high
#endif
	
	par->bus.max_clk = SPI_CLOCK_MAX;
	par->bus.min_clk = SPI_CLOCK_MIN;
	par->cs_num		 = plat->cs_num;
	for (i=0; i<plat->cs_num; i++) {
		if (plat->io_cs != NULL ) {
			par->bus.io_cs[i].port = (plat->io_cs[i].pad >> 16) & 0xF;
			par->bus.io_cs[i].pad  = 1 << (plat->io_cs[i].pad & 0xFF);
			if (plat->io_cs[i].func) {
				par->bus.io_cs[i].func = (void *)(plat->io_cs[i].func);
			} else {
				par->bus.io_cs[i].func = gpio_func;
			}
			par->bus.io_cs[i].gpio = par->gpioA + par->bus.io_cs[i].port * OFFSET_OF_GPIO_MODULE;
			par->bus.io_cs[i].gpio->GPIOxOUTENB |= par->bus.io_cs[i].pad; /* Pad to Output */
			par->bus.io_cs[i].gpio->GPIOxOUT |= par->bus.io_cs[i].pad; /* pad default to high */
		}
	}
#endif

	return 0;
}

static int init_spi_device(struct spi_param *par, struct spi_plat_data * plat)
{
	int ret = 0;
	int i;

	init_spi_device_hw(par, plat);

	/* SPI resource */
	INIT_WORK(&par->work, do_spi_workqueue);
	INIT_LIST_HEAD(&par->msg_queue);
	spin_lock_init(&par->lock);

	par->work_queue = create_singlethread_workqueue(SPI_DEV_NAME);
	if (NULL == par->work_queue)
		return -ENOMEM;

#if !defined(CONFIG_NEXELL_SPI_BY_GPIO)
	ret = request_irq(par->bus.irqno, do_spi_irqhandler, IRQF_DISABLED, I2C_DEV_NAME, par);
	if (ret) {
		ERROUT("fail, spi%d: request irq %d ...\n", 0, par->bus.irqno);
		return ret;
	}
	printk(KERN_INFO "spi%d: irq %d, slave cs %d \n", 0, par->bus.irqno, par->cs_num);
	printk(KERN_INFO "spi%d: %lu hz [%lu hz ~ %lu hz, pll:%lu] \n",
		0, SPI_CLOCK_CURR, SPI_CLOCK_MIN, SPI_CLOCK_MAX, CFG_SPI_CLK_FREQ);
#else
	ret = 0;

	printk(KERN_INFO "spi%d: GPIO spi driver,slave cs %d \n", 0, par->cs_num);
	printk(KERN_INFO "spi%d: %u hz [%u hz ~ %u hz] \n", 0,
		SPI_CLOCK_CURR, SPI_CLOCK_MIN, SPI_CLOCK_MAX);
#endif

	return ret;
}

static void	exit_spi_device(struct spi_param *par)
{
	DBGOUT("%s\n", __func__);

	if (par->work_queue) {
		flush_workqueue(par->work_queue);
		destroy_workqueue(par->work_queue);
	}

#if !defined(CONFIG_NEXELL_SPI_BY_GPIO)
	if (par->bus.irqno)
		free_irq(par->bus.irqno, par);

	par->regs->CONT0 &= ~SSPCONT0_SPI_ENB;
	par->regs->CLKENB = 0;
#else
	// GPIO port should remains GPIO...
#endif
}

static int set_spi_device(struct spi_param *par, int chip_select, unsigned int speed, int bpw)
{
	struct spi_cs_info *csi = &par->cs_info[chip_select];
	unsigned int mode = par->tr_mode;
	int port = par->bus.port;

	DBGOUT("%s: speed=%d, bits/w=%d, mode=0x%x\n", __func__, speed, bpw, mode);

	if ( !( (speed && (csi->speed_hz  != speed)) ||
		    (bpw   && (csi->bits_word != bpw  )) ||
		    (mode  && (csi->cs_mode	  != mode )) )) {
		return 0;
	}

#if !defined(CONFIG_NEXELL_SPI_BY_GPIO)
	if (mode & SPI_3WIRE) {
		printk(KERN_ERR "Not support 3-wire spi mode\n");
		return -EINVAL;
	}
	/* turn of SPI module */
	par->regs->CONT0 &= ~SSPCONT0_SPI_ENB;
	par->regs->CLKENB = 0;
	/* setup SPI module */
	if (mode && (csi->cs_mode != mode)) {
		unsigned int val;

		val = par->regs->CONT1;
		if (mode & SPI_CPHA) {
			val &= ~SSPCONT1_SCLKSH_FORMAT_MASK;
			val |= SSPCONT1_SCLKSH_FORMATB;
		} else {
			val &= ~SSPCONT1_SCLKSH_FORMAT_MASK;
			val |= SSPCONT1_SCLKSH_FORMATA;
		}
		if (mode & SPI_CPOL) {
			val &= ~SSPCONT1_SCLK_POL_MASK;
			val |= SSPCONT1_SCLK_POL_NORMAL;
		} else {
			val &= ~SSPCONT1_SCLK_POL_MASK;
			val |= SSPCONT1_SCLK_POL_INVERT;
		}
		par->regs->CONT1 = val;
	}

	if (bpw && (csi->bits_word != bpw)) {
		unsigned int val;

		val = par->regs->CONT0;
		val &= ~SSPCONT0_NUMBIT_MASK;
		val |= SSPCONT0_NUMBIT(bpw);
		par->regs->CONT0 = val;
	}

	if (speed && (csi->speed_hz != speed)) {
		unsigned int div = CFG_SPI_CLK_FREQ/CFG_SPI_CLK_DIV/speed;
		unsigned int val;

		val = par->regs->CONT0;
 		val &= ~SSPCONT0_DIVCNT_MASK;
 		val |= SSPCONT0_DIVCNT(div);
 		par->regs->CONT0 = val;
	}
	par->regs->CLKENB = SPI_CLKENB_PCLKMODE | SPI_CLKENB_CLKGENENB;
	par->regs->CONT0 &= ~SSPCONT0_SPI_ENB;
#else
	if (speed && (csi->speed_hz != speed)) {
		unsigned int udel = 1;
		
		if ((speed >= 100000) && (speed < 200000)) {
			udel = 5;
		} else
		if ((speed >= 200000) && (speed < 300000)) {
			udel = 4;
		} else
		if ((speed >= 300000) && (speed < 400000)) {
			udel = 3;
		} else
		if ((speed >= 400000) && (speed < 500000)) {
			udel = 2;
		} else
			udel = 1;
		csi->udel = udel;
	}
#endif

	/* save spi setup value */
	csi->speed_hz  = speed;
	csi->bits_word = bpw;
	par->tr_mode   = csi->cs_mode;

	printk(KERN_INFO "%s.%d: %8d hz [%u hz ~ %u hz, pll:%lu] \n",
		SPI_DEV_NAME, port, speed, SPI_CLOCK_MIN, SPI_CLOCK_MAX, CFG_SPI_CLK_FREQ);
	return 0;
}

static void set_spi_chip_select(struct spi_param *par, int chip_select, int enable)
{
	struct spi_io_cs_info   *io  = &par->bus.io_cs[chip_select];
	struct spi_cs_info *csi = &par->cs_info[chip_select];

	unsigned int mode = csi->cs_mode;
//	unsigned int val;

	DBGOUT("%s: cs=%d, mode = %x, enable=%d\n",
		__func__, chip_select, mode, enable);

	printk("io-func called enable=%d\n", enable);
	io->func(io, mode, enable);
/*	
    val = par->gpioA->GPIOxOUT;
    val &= ~0x10000000;
	if (mode & SPI_CS_HIGH)
		val |= enable ? 0x10000000 : 0x00000000;
	else
		val |= enable ? 0x00000000 : 0x10000000;
	par->gpioA->GPIOxOUT = val;
*/
}

static inline int spi_wait_ready(struct spi_param *par, int tx)
{
	if (tx)
		while ((par->regs->STATE & SSPSTAT_WFFEMPTY) == 0) { ; }
	else
		while ((par->regs->STATE & SSPSTAT_RFFEMPTY) != 0) { ; }

	return 0;
}

#if !defined(CONFIG_NEXELL_SPI_BY_GPIO)
// REAL SPI version
static inline int spi_readnwrite(struct spi_device *spi, const u8 **tx_buf, u8 **rx_buf, int count)
{
	struct spi_param *par = spi_master_get_devdata(spi->master);
	int i    = 0;
	int wt = 0;
	char *rxx = *rx_buf;
	const char *txx = *tx_buf;

	par->regs->CONT0 |= SSPCONT0_FIFO_CLR;
	par->regs->CONT0 &= ~SSPCONT0_FIFO_CLR;

	for (i = 0; count > i; i++)	{
		unsigned char tmp;

		if (tx_buf && *tx_buf) {
			tmp = *(*tx_buf)++;
			par->regs->DATA = tmp;
		} else
			par->regs->DATA = 0xFF;
	}
	par->regs->CONT0 |= SSPCONT0_SPI_ENB;
	if (txx)
		DBGOUT("tx %x = %x %x %x %x %x\n", txx, txx[0], txx[1], txx[2], txx[3], txx[4]); 

	if (0 > spi_wait_ready(par, 1)) {
		dev_err(&spi->dev, "TXS timed out\n");
		return -1;
	}

	wt = 0;
	while ((par->regs->STATE & SSPSTAT_TXSHEMPTY) == 0) ;
	for (i = 0; count > i; i++) {
		unsigned char tmp;

		DBGOUT("state = %x\n", par->regs->STATE);
		//spi_wait_ready(par, 0);
		if ((par->regs->STATE & SSPSTAT_RFFEMPTY) == 0) {
			if (rx_buf && *rx_buf) {
				tmp = par->regs->DATA;
				*(*rx_buf)++ = tmp;
			} else
				tmp = par->regs->DATA;	/* dump out data */
		} else {
			// wait some time...
			wt = 0;
			while ((par->regs->STATE & SSPSTAT_RFFEMPTY) != 0) {
				wt++;
				if (wt > 200000)
					break;
			}
			if (wt >= 200000) break;
		}
	}
	if (rxx)
		DBGOUT("rx %x %x %x %x %x\n", rxx[0], rxx[1], rxx[2], rxx[3], rxx[4]); 
	par->regs->CONT0 &= ~SSPCONT0_SPI_ENB;
	if (wt >= 200000) return -1;

	return 1;
}

static unsigned int trans_spi_data(struct spi_device *spi, struct spi_transfer *tr)
{
	int count, fifo, ret;
	const u8 *tx;
	u8 *rx;

	tx    = tr->tx_buf;
	rx    = tr->rx_buf;
	count = tr->len;
	fifo  = 64;			/* SPI FiFO size */

	do {

		if (count > fifo) {
			ret = spi_readnwrite(spi, &tx, &rx, fifo);
			count -= fifo;
		} else {
			ret = spi_readnwrite(spi, &tx, &rx, count);
			count = 0;
		}

		if (0 > ret)
			goto out;

	} while (count > 0);

out:
	return tr->len - count;
}

static irqreturn_t do_spi_irqhandler(int irqno, void *dev_id)
{
	DBGOUT("%s\n", __func__);
	return IRQ_HANDLED;
}

#else

#if defined(CONFIG_MODEL_RK_HIT_N)

#define	BIT_CLK		(1<<2)
#define BIT_TXDATA	(1<<0)
#define BIT_RXDATA	(1<<1)

#else

#define BIT_CLK		(1<<29)
#define BIT_TXDATA	(1<<31)
#define BIT_RXDATA	(1<<30)

#endif

int int_pol, int_pha, int_3wire;
int int_udel;

inline int tx_out(int val)
{
	static struct NX_GPIO_RegisterSet *gpioa = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO_MODULE);
	int res=0;

	if (int_pha == 0) {
		if (!int_3wire) {
			if (val)
				gpioa->GPIOxOUT |= BIT_TXDATA;
			else
				gpioa->GPIOxOUT &= ~BIT_TXDATA;
		} else {
			if (val)
				gpioa->GPIOxOUTENB &= ~BIT_TXDATA;
			else
				gpioa->GPIOxOUTENB |= BIT_TXDATA;
		}
		//res = (gpioa->GPIOxPAD & BIT_RXDATA) != 0;
	}
	udelay(int_udel);
	if (int_pol == 0) {
		gpioa->GPIOxOUT |= BIT_CLK;
	} else {
		gpioa->GPIOxOUT &= ~BIT_CLK;
	}
	if (int_pha == 0) {
		res = (gpioa->GPIOxPAD & BIT_RXDATA) != 0;
	}
	if (int_pha != 0) {
		if (!int_3wire) {
			if (val)
				gpioa->GPIOxOUT |= BIT_TXDATA;
			else
				gpioa->GPIOxOUT &= ~BIT_TXDATA;
		} else {
			if (val)
				gpioa->GPIOxOUTENB &= ~BIT_TXDATA;
			else
				gpioa->GPIOxOUTENB |= BIT_TXDATA;
		}
	}
	udelay(int_udel);
	if (int_pol == 0) {
		gpioa->GPIOxOUT &= ~BIT_CLK;
	} else {
		gpioa->GPIOxOUT |= BIT_CLK;
	}
	if (int_pha != 0) {
		res = (gpioa->GPIOxPAD & BIT_RXDATA) != 0;
	}
	
	return res;
}
// GPIO version
static inline int spi_readnwrite_gpio(struct spi_device *spi, const u8 **tx_buf, u8 **rx_buf, int count)
{
	struct spi_param *par = spi_master_get_devdata(spi->master);
	struct spi_cs_info *csi = &par->cs_info[spi->chip_select];
	int i = 0, j;
	char *rxx = *rx_buf;
	const char *txx = *tx_buf;
	int lsb_first;

	int_pol = spi->mode & SPI_CPOL;
	int_pha = spi->mode & SPI_CPHA;
	int_3wire = spi->mode & SPI_3WIRE;
	lsb_first = spi->mode & SPI_LSB_FIRST;
	int_udel = csi->udel;
	DBGOUT("pol=%d pha=%d 3wire=%d lsb_first=%d\n", int_pol, int_pha, int_3wire, lsb_first);

	if (txx)
		DBGOUT("tx %x = %x %x %x %x %x\n", txx, txx[0], txx[1], txx[2], txx[3], txx[4]); 

	for (i = 0; count > i; i++) {
		unsigned char tx_tmp,rx_tmp;
		int rxb;

		if (tx_buf && *tx_buf) {
			tx_tmp = *(*tx_buf)++;
		} else {
			tx_tmp = 0xFF;
		}
		rx_tmp = 0;
		for (j=0; j< 8; j++) {
			if (!lsb_first) {
				if (tx_tmp & 0x80) {
					rxb = tx_out(1);
				} else {
					rxb = tx_out(0);
				}
				tx_tmp <<= 1;
				rx_tmp <<= 1;
				rx_tmp |= rxb;
				
			} else {
				if (tx_tmp & 0x01) {
					rxb = tx_out(1);
				} else {
					rxb = tx_out(0);
				}
				tx_tmp >>= 1;
				rx_tmp >>= 1;
				if (rxb)
					rx_tmp |= 0x80;
				
			}
		}
		if (rx_buf && *rx_buf) {
			*(*rx_buf) = rx_tmp;
			(*rx_buf)++;
		}
	}
	if (rxx)
		DBGOUT("rx %x %x %x %x %x\n", rxx[0], rxx[1], rxx[2], rxx[3], rxx[4]); 

	return 1;
}

static unsigned int trans_spi_data(struct spi_device *spi, struct spi_transfer *tr)
{
	int count, ret;
	const u8 *tx;
	u8 *rx;

	tx    = tr->tx_buf;
	rx    = tr->rx_buf;
	count = tr->len;

	ret = spi_readnwrite_gpio(spi, &tx, &rx, count);

	return (ret > 0) ? tr->len : 0;
}

#endif

static int setup_spi_trans(struct spi_device *spi, struct spi_transfer *tr)
{
	struct spi_param *par = NULL;
	unsigned int speed = spi->max_speed_hz;
	unsigned int bpw   = spi->bits_per_word;
	int chip_select	   = spi->chip_select;

	par = spi_master_get_devdata(spi->master);

	if ((tr != NULL) && tr->speed_hz)
		speed = tr->speed_hz;

	if ((tr != NULL) && tr->bits_per_word)
		bpw = tr->bits_per_word;

	return set_spi_device(par, chip_select, speed, bpw);
}

static void do_spi_workqueue(struct work_struct *work)
{
	struct spi_param *par = container_of(work, struct spi_param, work);

	DBGOUT("%s\n", __func__);

	spin_lock_irq(&par->lock);

	while (! list_empty(&par->msg_queue)) {
		struct spi_message 	*msg;
		struct spi_device 	*spi;
		struct spi_transfer *tr = NULL;
		int chip_select;
		int cs_active   = 0;
		int status      = 0;

		msg = container_of(par->msg_queue.next, struct spi_message, queue);
		list_del_init(&msg->queue);
		spin_unlock_irq(&par->lock);

		spi = msg->spi;
		chip_select = spi->chip_select;

		/* Load defaults */
		status = setup_spi_trans(spi, NULL);
		if (0 > status)
			goto msg_done;

		list_for_each_entry(tr, &msg->transfers, transfer_list) {

			DBGOUT("%s: cs_change=%d, delay %d usec, len=%d, act_len=%d \n",
				__func__, tr->cs_change, tr->delay_usecs, tr->len, msg->actual_length);

			if (tr->speed_hz || tr->bits_per_word) {
				status = setup_spi_trans(spi, tr);
				if (0 > status)
					break;
			}

			if ((! cs_active) && !(spi->mode & SPI_NO_CS)) {
				set_spi_chip_select(par, chip_select, 1);
				cs_active = 1;
			}

			if (tr->len) {
				msg->actual_length += trans_spi_data(spi, tr);
			}

			if (tr->delay_usecs)
				udelay(tr->delay_usecs);

			if (tr->cs_change) {
				set_spi_chip_select(par, chip_select, 0);
				cs_active = 0;
			}
		}

msg_done:
		if (cs_active && !(spi->mode & SPI_NO_CS))
			set_spi_chip_select(par, chip_select, 0);

		msg->status = status;
		msg->complete(msg->context);

		spin_lock_irq(&par->lock);
	}

	spin_unlock_irq(&par->lock);
}

/*------------------------------------------------------------------------------
 * 	SPI master functions
 */
static int nx_spi_setup(struct spi_device *spi)
{
	struct spi_param   *par = NULL;
	struct spi_cs_info *csi = NULL;

	par = spi_master_get_devdata(spi->master);
	csi  = &par->cs_info[spi->chip_select];

	DBGOUT("setup spi%d.%d, mode=0x%x, %s%s%s%s, %u bits/w, %u Hz max \n",
			par->bus.port, spi->chip_select,
			(spi->mode & (SPI_CPOL | SPI_CPHA)),
			(spi->mode & SPI_CS_HIGH) ? "cs_high, " : "",
			(spi->mode & SPI_LSB_FIRST) ? "lsb, " : "",
			(spi->mode & SPI_3WIRE) ? "3wire, " : "",
			(spi->mode & SPI_LOOP) ? "loopback, " : "",
			spi->bits_per_word, spi->max_speed_hz);

	if (spi->bits_per_word > 16 || 1 > spi->bits_per_word ) {
		ERROUT("invalid bits_per_word=%d\n", spi->bits_per_word);
		return -EINVAL;
	}

	if (spi->max_speed_hz == 0)
		spi->max_speed_hz = par->bus.max_clk;

	if (spi->max_speed_hz > par->bus.max_clk)
		spi->max_speed_hz = par->bus.max_clk;

	if (spi->max_speed_hz < par->bus.min_clk)
		spi->max_speed_hz = par->bus.min_clk;

	csi->cs_mode = spi->mode;

	/*
	 * NOTE: cannot change speed and other hw settings immediately,
	 *       otherwise sharing of spi bus is not possible,
	 *       so do not call setupxfer(spi, NULL) here
	 */
	return 0;
}

static int nx_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct spi_param 	*par;
	struct spi_transfer *tr = NULL;
	unsigned long flags;

	DBGOUT("%s\n", __func__);

	msg->actual_length = 0;
	msg->status 	   = 0;

	/* reject invalid messages and transfers */
	if (list_empty(&msg->transfers) || !msg->complete)
		return -EINVAL;

	par = spi_master_get_devdata(spi->master);

	list_for_each_entry(tr, &msg->transfers, transfer_list) {
		unsigned int bits_per_word = spi->bits_per_word;

		if (tr->tx_buf == NULL && tr->rx_buf == NULL && tr->len) {
			ERROUT("missing rx and tx buf\n");
			goto err_trans;
		}

		if ((tr != NULL) && tr->bits_per_word)
			bits_per_word = tr->bits_per_word;

		if ((bits_per_word > 16) || (1 > bits_per_word)) {
			ERROUT("invalid transfer %d bits\n", bits_per_word);
			goto err_trans;
		}

		if ((tr != NULL) && tr->speed_hz &&
		   (par->bus.min_clk > tr->speed_hz || tr->speed_hz > par->bus.max_clk)) {
			ERROUT("over speed %d HZ min (%d Hz) max(%d Hz)\n",
				tr->speed_hz, par->bus.min_clk, par->bus.max_clk);
			goto err_trans;
		}
	}

	spin_lock_irqsave(&par->lock, flags);
	list_add_tail(&msg->queue, &par->msg_queue);
	queue_work(par->work_queue, &par->work);
	spin_unlock_irqrestore(&par->lock, flags);

	return 0;

err_trans:
	/* Message rejected and not queued */
	msg->status = -EINVAL;
	if (msg->complete)
		msg->complete(msg->context);

	return -EINVAL;
}

static int __init nx_spi_drv_probe(struct platform_device *pdev)
{
	struct spi_master    * master;
	struct spi_param     * par;
	struct spi_plat_data * plat = pdev->dev.platform_data;
	int ret = 0;

	DBGOUT("%s\n", __func__);

	master = spi_alloc_master(&pdev->dev, sizeof *par);
	if (master == NULL) {
		printk(KERN_ERR "fail, %s master allocation failed ...\n", pdev->name);
		return -ENOMEM;
	}

	par = spi_master_get_devdata(master);
	par->master	= master;

	/* init spi data struct */
	ret = init_spi_device(par, plat);
	if (ret)
		goto err_mem;

	master->bus_num 		= par->bus.port;
#if defined(CONFIG_NEXELL_SPI_BY_GPIO)	
	master->mode_bits 		= SPI_NO_CS | SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST | SPI_3WIRE;
#else
	master->mode_bits 		= SPI_NO_CS | SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST;
#endif
	master->setup 			= nx_spi_setup;
	master->transfer 		= nx_spi_transfer;
	master->num_chipselect 	= par->cs_num;

	ret = spi_register_master(master);
	if (0 > ret)
		goto err_out;

	platform_set_drvdata(pdev, master);

	return ret;

err_out:
	spi_master_put(master);

err_mem:
	kfree(master);
	return ret;
}

static int nx_spi_drv_remove(struct platform_device *pdev)
{
	struct spi_master *master;
	struct spi_param  *par;

	DBGOUT("%s\n", __func__);

	master = dev_get_drvdata(&pdev->dev);
	par    = spi_master_get_devdata(master);

	exit_spi_device(par);

	spi_unregister_master(master);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int nx_spi_resume(struct platform_device *pdev)
{
	struct spi_plat_data * plat = pdev->dev.platform_data;
	struct spi_master *master;
	struct spi_param  *par;

	DBGOUT("%s\n", __func__);

	master = dev_get_drvdata(&pdev->dev);
	par    = spi_master_get_devdata(master);

	init_spi_device_hw(par, plat);

	return 0;
}

static struct platform_driver spi_plat_driver = {
	.probe		= nx_spi_drv_probe,
	.remove		= nx_spi_drv_remove,
	.resume     = nx_spi_resume,
	.driver 	= {
		.owner	= THIS_MODULE,
		.name	= SPI_DEV_NAME,
	},
};

static int __init nx_spi_drv_init(void)
{
	DBGOUT("%s\n", __func__);
	return platform_driver_register(&spi_plat_driver);
}

static void __exit nx_spi_drv_exit(void)
{
	DBGOUT("%s\n", __func__);
	platform_driver_unregister(&spi_plat_driver);
}

module_init(nx_spi_drv_init);
module_exit(nx_spi_drv_exit);

MODULE_AUTHOR("Seungwoo Kim<ksw@stcube.com>");
MODULE_DESCRIPTION("SPI driver for the Nexell(GPIO/Hardware)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("nx-spi");
