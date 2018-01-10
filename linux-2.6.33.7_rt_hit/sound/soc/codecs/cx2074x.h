/*
 * ALSA SoC CX2074x codec driver
 *
 * Copyright:   (C) 2010/2011 Conexant Systems
 *
 * Based on sound/soc/codecs/tlv320aic2x.c by Vladimir Barinov
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The software is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY, without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License 
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along with 
 * this software.  If not, see <http//www.gnu.org/licenses/>
 *
 * All copies of the software must include all unaltered copyright notices, 
 * disclaimers of warranty, all notices that refer to the General Public License 
 * and the absence of any warranty.
 *
 *  History
 *  Added support for CX2074x codec [www.conexant.com]
 */

#ifndef _CX2074x_H
#define _CX2074x_H

#define CX2074X_I2C_DRIVER_NAME	"cx2074x"
//#define CX2074X_JACK_SENSE_INTERRUPT_MODE

typedef u8 cx2074x_reg_t;

#ifndef _PASS_1_COMPLETE
// DAC 1:2 sample rate/size			0x0F
#define DAC_LEFT_ENABLE				0x01
#define DAC_RIGHT_ENABLE			0x02
#define DAC_RATE_8000				0x00
#define DAC_RATE_11025				0x90
#define DAC_RATE_16000				0x20
#define DAC_RATE_22050				0xb0
#define DAC_RATE_32000				0x40
#define DAC_RATE_44100				0xd0
#define DAC_RATE_48000				0x50
#define DAC_RATE_88200				0xe0
#define DAC_RATE_96000				0x60

// ADC sample rate/size				0x13
#define ADC_ENABLE					0x01
#define ADC_RATE_8000				0x00
#define ADC_RATE_12000				0x10
#define ADC_RATE_16000				0x20
#define ADC_RATE_24000				0x30
#define ADC_RATE_32000				0x40
#define ADC_RATE_48000				0x50
#define ADC_RATE_8018				0x80
#define ADC_RATE_11025				0x90
#define ADC_RATE_16036				0xA0
#define ADC_RATE_22050				0xB0
#define ADC_RATE_44100				0xD0

// ADC analog left control			0x19
#define ADC_LEFT_ENABLE				0x01

// ADC analog right control			0x1A
#define ADC_RIGHT_ENABLE			0x01

// I2S/PCM Control 1				0x20
#define PCM_ENABLE					0x01
#define I2S_PCM_DAC_8_BIT			0x00
#define I2S_PCM_DAC_16_BIT			0x02
#define I2S_PCM_DAC_24_BIT			0x04
#define I2S_PCM_ADC_8_BIT			0x00
#define I2S_PCM_ADC_16_BIT			0x08
#define I2S_PCM_ADC_24_BIT			0x10

// DAC1 Left Channel
#define DAC_LEFT_MUTE				0x80

// DAC2 Right Channel
#define DAC_RIGHT_MUTE				0x80

// Jack Sense Status
#define JACK_SENSE_HEADPHONE		0x01
#define JACK_SENSE_LINEIN_L			0x02
#define	JACK_SENSE_LINEIN_R			0x04
#define	JACK_SENSE_LINEOUT			0x08

// Interrupt Status
#define INT_STS_CLASS_D_ERROR		0x02
#define INT_STS_JACK_SENSE			0x04
#define INT_STS_HEADSET_VOL			0x08
#define INT_STS_ROTARY_VOL			0x10

#endif // #ifndef _PASS_1_COMPLETE

// Don't define bit macros more than once.
#ifndef _PASS_1_COMPLETE
#define _PASS_1_COMPLETE
#endif

#define AUDDRV_VERSION(major0, major1, minor, build) ((major0) << 24 | (major1) << 16 | (minor) << 8 |(build))

extern  struct snd_soc_dai soc_codec_cx2074x_dai;
extern  struct snd_soc_codec_device soc_codec_dev_cx2074x;

struct cx2074x_data {
	int dres;
};

#endif // _CX2074x_H
