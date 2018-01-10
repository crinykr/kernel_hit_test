/*
 * cx20709.h  --  CX20709 Soc Audio driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _CX20709_H
#define _CX20709_H

/* CX20709 register space */


#define CX20709_CACHEREGNUM 	56

#define CX20709_LINVOL		0x0
#define CX20709_RINVOL		0x1
#define CX20709_LOUT1		0x2
#define CX20709_ROUT1		0x3
#define CX20709_CLOCK1		0x4
#define CX20709_DACCTL1		0x5
#define CX20709_DACCTL2		0x6
#define CX20709_IFACE1		0x7
#define CX20709_CLOCK2		0x8
#define CX20709_IFACE2		0x9
#define CX20709_LDAC		0xa
#define CX20709_RDAC		0xb

#define CX20709_RESET		0xf
#define CX20709_3D		0x10
#define CX20709_ALC1		0x11
#define CX20709_ALC2		0x12
#define CX20709_ALC3		0x13
#define CX20709_NOISEG		0x14
#define CX20709_LADC		0x15
#define CX20709_RADC		0x16
#define CX20709_ADDCTL1		0x17
#define CX20709_ADDCTL2		0x18
#define CX20709_POWER1		0x19
#define CX20709_POWER2		0x1a
#define CX20709_ADDCTL3		0x1b
#define CX20709_APOP1		0x1c
#define CX20709_APOP2		0x1d

#define CX20709_LINPATH		0x20
#define CX20709_RINPATH		0x21
#define CX20709_LOUTMIX		0x22

#define CX20709_ROUTMIX		0x25
#define CX20709_MONOMIX1		0x26
#define CX20709_MONOMIX2		0x27
#define CX20709_LOUT2		0x28
#define CX20709_ROUT2		0x29
#define CX20709_MONO		0x2a
#define CX20709_INBMIX1		0x2b
#define CX20709_INBMIX2		0x2c
#define CX20709_BYPASS1		0x2d
#define CX20709_BYPASS2		0x2e
#define CX20709_POWER3		0x2f
#define CX20709_ADDCTL4		0x30
#define CX20709_CLASSD1		0x31

#define CX20709_CLASSD3		0x33
#define CX20709_PLL1		0x34
#define CX20709_PLL2		0x35
#define CX20709_PLL3		0x36
#define CX20709_PLL4		0x37


/*
 * CX20709 Clock dividers
 */
#define CX20709_SYSCLKDIV 		0
#define CX20709_DACDIV			1
#define CX20709_ADCDIV			2
#define CX20709_OPCLKDIV			3
#define CX20709_DCLKDIV			4
#define CX20709_TOCLKSEL			5
#define CX20709_SYSCLKSEL		6
#define CX20709_BCLKDIV  		7

#define CX20709_SYSCLK_DIV_1		(0 << 1)
#define CX20709_SYSCLK_DIV_2		(2 << 1)

#define CX20709_SYSCLK_MCLK		(0 << 0)
#define CX20709_SYSCLK_PLL		(1 << 0)

#define CX20709_DAC_DIV_1		(0 << 3)
#define CX20709_DAC_DIV_1_5		(1 << 3)
#define CX20709_DAC_DIV_2		(2 << 3)
#define CX20709_DAC_DIV_3		(3 << 3)
#define CX20709_DAC_DIV_4		(4 << 3)
#define CX20709_DAC_DIV_5_5		(5 << 3)
#define CX20709_DAC_DIV_6		(6 << 3)

#define CX20709_ADC_DIV_1		(0 << 6)
#define CX20709_ADC_DIV_1_5		(1 << 6)
#define CX20709_ADC_DIV_2		(2 << 6)
#define CX20709_ADC_DIV_3		(3 << 6)
#define CX20709_ADC_DIV_4		(4 << 6)
#define CX20709_ADC_DIV_5_5		(5 << 6)
#define CX20709_ADC_DIV_6		(6 << 6)

#define CX20709_DCLK_DIV_1_5		(0 << 6)
#define CX20709_DCLK_DIV_2		(1 << 6)
#define CX20709_DCLK_DIV_3		(2 << 6)
#define CX20709_DCLK_DIV_4		(3 << 6)
#define CX20709_DCLK_DIV_6		(4 << 6)
#define CX20709_DCLK_DIV_8		(5 << 6)
#define CX20709_DCLK_DIV_12		(6 << 6)
#define CX20709_DCLK_DIV_16		(7 << 6)

#define CX20709_TOCLK_F19		(0 << 1)
#define CX20709_TOCLK_F21		(1 << 1)

#define CX20709_OPCLK_DIV_1		(0 << 0)
#define CX20709_OPCLK_DIV_2		(1 << 0)
#define CX20709_OPCLK_DIV_3		(2 << 0)
#define CX20709_OPCLK_DIV_4		(3 << 0)
#define CX20709_OPCLK_DIV_5_5		(4 << 0)
#define CX20709_OPCLK_DIV_6		(5 << 0)

#define CX20709_BCLK_DIV_1		(0 << 0)
#define CX20709_BCLK_DIV_1_5		(1 << 0)
#define CX20709_BCLK_DIV_2		(2 << 0)
#define CX20709_BCLK_DIV_3		(3 << 0)
#define CX20709_BCLK_DIV_4	    (4 << 0)
#define CX20709_BCLK_DIV_5_5		(5 << 0)
#define CX20709_BCLK_DIV_6		(6 << 0)
#define CX20709_BCLK_DIV_8		(7 << 0)
#define CX20709_BCLK_DIV_11		(8 << 0)
#define CX20709_BCLK_DIV_12		(9 << 0)
#define CX20709_BCLK_DIV_16	    (10 << 0)
#define CX20709_BCLK_DIV_22		(11 << 0)
#define CX20709_BCLK_DIV_24		(12 << 0)
#define CX20709_BCLK_DIV_32		(13 << 0)

extern struct snd_soc_dai cx20709_dai;
extern struct snd_soc_dai cx20709_2_dai;
extern struct snd_soc_dai cx20709_3_dai;
extern struct snd_soc_codec_device soc_codec_dev_cx20709;
extern struct snd_soc_codec_device soc_codec_dev_two_cx20709;

#define CX20709_DRES_400R 0
#define CX20709_DRES_200R 1
#define CX20709_DRES_600R 2
#define CX20709_DRES_150R 3
#define CX20709_DRES_MAX  3

struct cx20709_data {
	int dres;
};

#endif
