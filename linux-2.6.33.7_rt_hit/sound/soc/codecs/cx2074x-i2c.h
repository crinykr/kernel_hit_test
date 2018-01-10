/*
 * CX2074x register definitions.
 *
 * Copyright:   (C) 2010/2011 Conexant Systems
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

/////////////////////////////////////////////////////////////////////////
//  General codec operations registers
/////////////////////////////////////////////////////////////////////////
//      id					addr		data		type
__REG(DAC_SAMPLE_RATE_SIZE,	0x0F,		0x53,		RW)
__REG(DAC1_VOLUME_CTRL,		0x10,		0x4A,		RW)
__REG(DAC2_VOLUME_CTRL,		0x11,		0x4A,		RW)
__REG(DIGITAL_MIC_CTRL,		0x12,		0x00,		RW)	//bit0: enable/disable, bit1: power down/power up
__REG(ADC_SAMPLE_RATE_SIZE,	0x13,		0x20,		RW)
__REG(ADC_L_VOLUME_CTRL,	0x14,		0x4A,		RW)
__REG(ADC_R_VOLUME_CTRL,	0x15,		0x4A,		RW)
__REG(HEADPHONE_CTRL,		0x16,		0x1b,		RW)
__REG(LINE_OUT_CTRL,		0x17,		0x33,		RW)
__REG(CLASS_D_CTRL,			0x18,		0x03,		RW)
__REG(ADC_L_CTRL,			0x19,		0x04,		RW)
__REG(ADC_R_CTRL,			0x1A,		0x04,		RW)
__REG(MIC_BIAS_CTRL,		0x1B,		0x00,		RW)
__REG(I2S_TX_CTRL_1,		0x1C,		0x80,		RW)
__REG(I2S_RX_CTRL_1,		0x1E,		0x80,		RW)
__REG(I2S_PCM_CTRL_1,		0x20,		0x0A,		RW)
__REG(VOL_CTRL,				0x5C,		0x00,		RW)
__REG(INT_EN,				0x5D,		0x04,		RW)
__REG(INT_STATUS,			0x5E,		0x00,		RW)
__REG(CODEC_TEST_7,			0x8F,		0x01,		RW)
__REG(JACK_SENSE_STATUS,	0xE3,		0x00,		RO)
__REG(GPIO_OUT,				0xF2,		0x00,		RW)
__REG(GPIO_DIR,				0xF3,		0x00,		RW)
__REG(DEV_ID_LSB,			0xFD,		0x00,		RO)
__REG(DEV_ID_MSB,			0xFE,		0x00,		RO)

