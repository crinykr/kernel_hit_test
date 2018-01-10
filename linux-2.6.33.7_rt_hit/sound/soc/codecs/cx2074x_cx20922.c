/*
* Cx20745 with Cx20922(BigBang) codec driver
* Based on Conexant following dirver.
*
* Copyright(c) 2014 STcube Inc,
* All right reserved by Seungwoo Kim
*
* ALSA SoC CX2074X/20922 Channel codec driver
*
* Copyright:   (C) 2009/2010 Conexant Systems
*
* Based on sound/soc/codecs/tlv320aic2x.c by Vladimir Barinov
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* 
*      
*************************************************************************
*  Modified Date:  04/02/14
*  File Version:   2.6.33.7
*************************************************************************
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/jack.h>
#include <sound/tlv.h>

#include <mach/platform.h>

#include "cx2074x_cx20922.h"

static int SendCmd (struct snd_soc_codec *codec, Command *cmd, uint32_t  app_module_id, uint32_t  command_id, uint32_t num_32b_words, ...);
static void CodecPowerOnOff(int On);

#define DEBUG	0

#define GPIO_CODEC_POWER_ON	(0x13 << 19)

#define CX2074X_DRIVER_VERSION AUDDRV_VERSION(2, 6, 0x33, 0x04) 

#define CX20922_RATES_CAPTURE ( \
      SNDRV_PCM_RATE_8000  \
    | SNDRV_PCM_RATE_11025 \
    | SNDRV_PCM_RATE_16000 \
    | SNDRV_PCM_RATE_22050 \
    | SNDRV_PCM_RATE_32000 \
    | SNDRV_PCM_RATE_44100 \
    | SNDRV_PCM_RATE_48000 \
    | SNDRV_PCM_RATE_88200 \
    | SNDRV_PCM_RATE_96000 )

#define CX20922_FORMATS_CAPTURE ( SNDRV_PCM_FMTBIT_S16_LE \
    | SNDRV_PCM_FMTBIT_S16_BE \
    | SNDRV_PCM_FMTBIT_MU_LAW \
    | SNDRV_PCM_FMTBIT_A_LAW )

#define CX2074x_RATES_PLAYBACK	SNDRV_PCM_RATE_8000_96000
#define CX2074x_RATES_CAPTURE	SNDRV_PCM_RATE_8000_48000

#define CX2074x_FORMATS_PLAYBACK (SNDRV_PCM_FMTBIT_S8		\
								| SNDRV_PCM_FMTBIT_S16_LE	\
								| SNDRV_PCM_FMTBIT_S24_LE )

#define CX2074x_FORMATS_CAPTURE ( SNDRV_PCM_FMTBIT_S8		\
								| SNDRV_PCM_FMTBIT_S16_LE	\
								| SNDRV_PCM_FMTBIT_S24_LE)

#undef CX2074x_RATES_PLAYBACK
#undef CX2074x_FORMATS_PLAYBACK
#undef CX20922_FORMATS_CAPTURE
#undef CX20922_RATES_CAPTURE

// Only 16KHz or 48KHz suport, selectable only BIGBANG's firmware.
// Onlu 16bit signed integer format support for both chip
#define CX20922_FORMATS_CAPTURE ( SNDRV_PCM_FMTBIT_S16_LE )
#define CX2074x_FORMATS_PLAYBACK ( SNDRV_PCM_FMTBIT_S16_LE )
#define CX2074x_RATES_PLAYBACK	(SNDRV_PCM_RATE_48000)
#define CX20922_RATES_CAPTURE	(SNDRV_PCM_RATE_48000)

#define noof(a) (sizeof(a)/sizeof(a[0]))
#define NOINLINE __attribute__((__noinline__))

#define INFO(a,...)		printk(KERN_INFO a, ##__VA_ARGS__)
#define _INFO(a,...)	printk(a, ##__VA_ARGS__)

#define MSG(fmt,...)	printk(KERN_INFO fmt, ##__VA_ARGS__)
#define ERROR(fmt,...)	printk(KERN_ERR fmt, ##__VA_ARGS__)

#define get_cx2074x_priv(_codec_) ((struct cx2074x_priv*)(_codec_)->private_data)
#define GET_REG_CACHE(_codec_) (cx2074x_reg_t*) (_codec_)->reg_cache

static DEFINE_MUTEX(reg_cache_lock);
static DEFINE_MUTEX(codec_pwr_lock);

struct cx2074x_reg {
	char *name;
	u8    addr;
	u8    type;
};

#define CTRL_SONYMODE		(0x01 << 0)
#define CTRL_MUTE			(0x01 << 1)
#define CTRL_RIGHTJUSTIFIED	(0x01 << 2)
#define CTRL_LEFTJUSTIFIED	(0x00 << 2)

enum {
	DAC_SAMPLE_RATE_SIZE	= 0x0F,
	DAC1_VOLUME_CTRL		= 0x10,
	DAC2_VOLUME_CTRL		= 0x11,
	DIGITAL_MIC_CTRL		= 0x12,
	ADC_SAMPLE_RATE_SIZE	= 0x13,
	ADC_L_VOLUME_CTRL		= 0x14,
	ADC_R_VOLUME_CTRL		= 0x15,
	HEADPHONE_CTRL			= 0x16,
	LINE_OUT_CTRL			= 0x17,
	CLASS_D_CTRL			= 0x18,
	ADC_L_CTRL				= 0x19,
	ADC_R_CTRL				= 0x1A,
	MIC_BIAS_CTRL			= 0x1B,
	I2S_TX_CTRL_1			= 0x1C,
	I2S_RX_CTRL_1			= 0x1E,
	I2S_PCM_CTRL_1			= 0x20,
	VOL_CTRL				= 0x5C,
	INT_EN					= 0x5D,
	INT_STATUS				= 0x5E,
	CODEC_TEST_7			= 0x8F,
	JACK_SENSE_STATUS		= 0xE3,
	GPIO_OUT				= 0xF2,
	GPIO_DIR				= 0xF3,
	DEV_ID_LSB				= 0xFD,
	DEV_ID_MSB				= 0xFE,
};

static const u8 cx2074x_data[] = {
	0x53,	// DAC_SAMPLE_RATE_SIZE
	0x4A,   // DAC1_VOLUME_CTRL
	0x4A,   // DAC2_VOLUME_CTRL
	0x00,   // DIGITAL_MIC_CTRL
	0x20,   // ADC_SAMPLE_RATE_SIZE
	0x4A,   // ADC_L_VOLUME_CTRL
	0x4A,   // ADC_R_VOLUME_CTRL
	0x1b,   // HEADPHONE_CTRL
	0x33,   // LINE_OUT_CTRL
	0x03,   // CLASS_D_CTRL
	0x04,   // ADC_L_CTRL
	0x04,   // ADC_R_CTRL
	0x00,   // MIC_BIAS_CTRL
	0x80,   // I2S_TX_CTRL_1
	0x80,   // I2S_RX_CTRL_1
	0x0A,   // I2S_PCM_CTRL_1
	0x00,   // VOL_CTRL
	0x04,   // INT_EN
	0x00,   // INT_STATUS
	0x01,   // CODEC_TEST_7
	0x00,   // JACK_SENSE_STATUS
	0x00,   // GPIO_OUT
	0x00,   // GPIO_DIR
	0x00,   // DEV_ID_LSB
	0x00,   // DEV_ID_MSB
};

#define REG_TYPE_RO	0	// read only,  read during initialization
#define REG_TYPE_RW	1	// read/write, read during initialization
#define REG_TYPE_WI	2	// write only, written during initialization
#define REG_TYPE_WC	3	// write/init, needs NEWC to be set when written

// for cx2074x reg save & resume
#define PLAIN_INDEX_MARK	0x10000

static const struct cx2074x_reg cx2074x_regs[] = {
	{ "DAC_SAMPLE_RATE_SIZE",	0x0F, REG_TYPE_RW },
	{ "DAC1_VOLUME_CTRL",		0x10, REG_TYPE_RW },
	{ "DAC2_VOLUME_CTRL",		0x11, REG_TYPE_RW },
	{ "DIGITAL_MIC_CTRL",		0x12, REG_TYPE_RW },	//bit0: enable/disable, bit1: power down/power up
	{ "ADC_SAMPLE_RATE_SIZE",	0x13, REG_TYPE_RW },
	{ "ADC_L_VOLUME_CTRL",		0x14, REG_TYPE_RW },
	{ "ADC_R_VOLUME_CTRL",		0x15, REG_TYPE_RW },
	{ "HEADPHONE_CTRL",			0x16, REG_TYPE_RW },
	{ "LINE_OUT_CTRL",			0x17, REG_TYPE_RW },
	{ "CLASS_D_CTRL",			0x18, REG_TYPE_RW },
	{ "ADC_L_CTRL",				0x19, REG_TYPE_RW },
	{ "ADC_R_CTRL",				0x1A, REG_TYPE_RW },
	{ "MIC_BIAS_CTRL",			0x1B, REG_TYPE_RW },
	{ "I2S_TX_CTRL_1",			0x1C, REG_TYPE_RW },
	{ "I2S_RX_CTRL_1",			0x1E, REG_TYPE_RW },
	{ "I2S_PCM_CTRL_1",			0x20, REG_TYPE_RW },
	{ "VOL_CTRL",				0x5C, REG_TYPE_RW },
	{ "INT_EN",					0x5D, REG_TYPE_RW },
	{ "INT_STATUS",				0x5E, REG_TYPE_RW },
	{ "CODEC_TEST_7",			0x8F, REG_TYPE_RW },
	{ "JACK_SENSE_STATUS",		0xE3, REG_TYPE_RO },
	{ "GPIO_OUT",				0xF2, REG_TYPE_RW },
	{ "GPIO_DIR",				0xF3, REG_TYPE_RW },
	{ "DEV_ID_LSB",				0xFD, REG_TYPE_RO },
	{ "DEV_ID_MSB",				0xFE, REG_TYPE_RO },
};

// codec private data
struct cx2074x_priv {
	struct delayed_work check_jack_work;
	struct NX_GPIO_RegisterSet *gpio;
	void* control_data;
	unsigned int sysclk;
	int	master;
	int prv_state;
	int pwr_state;
    struct snd_soc_codec codec;
    struct snd_soc_codec codec1;
};

struct cx2074x_priv* cx2074x;

struct snd_soc_codec *cx2074x_codec;
struct snd_soc_codec *cx20922_codec;

/*
 * Playback Volume 
 *
 * max : 0x50 : 6 dB
 *       ( 1 dB step )
 * min : 0x00 : -74 dB
 */
static const DECLARE_TLV_DB_SCALE(dac_tlv, -7400, 100, 0);

/*
 * Capture Volume 
 *
 * max : 0x50 : 6 dB
 *       ( 1 dB step )
 * min : 0x00 : -74 dB
 */
static const DECLARE_TLV_DB_SCALE(adc_tlv, -7400, 100, 0);

/*
 * ADC Gain
 *
 * max : 0x3 : 40 dB
 *       ( 10 dB step )
 * min : 0x0 : 0 dB
 */
static const DECLARE_TLV_DB_SCALE(adc_gain_tlv, 0, 10, 0);

/*
 * Digital Mic Channel Gain
 *
 * max : 0x4 : -48 dB
 *       ( -12 dB step )
 * min : 0x0 : 0 dB
 */
static const DECLARE_TLV_DB_SCALE(digi_mic_gain_tlv, 0, -12, 0);

//DECLARE_TLV_DB_SCALE(name, min, step, mute)
/*
 * Capture Volume(CX20922)
 *
 * max : 0x50 : 30 dB
 *       ( 1 dB step )
 * min : 0x00 : 0 dB
 */
static const DECLARE_TLV_DB_SCALE(adc_tlv_cx20922, 0, 1, 0);

#define CX20922_HEADPHONE 		0
#define CX20922_SPEAKER 		1
#define CX20922_MONOOUT 		2
#define CX20922_MICROPHONE_L	3
#define CX20922_MICROPHONE_R	4
#define CX20922_LINEIN 			5
#define CX20922_AGC 			6
#define CX20922_DRC 			7

#define CX20922_HEADPHONE_VOL_INIT		100
#define CX20922_SPEAKER_VOL_INIT 		100
#define CX20922_MONOOUT_VOL_INIT 		100
#define CX20922_MICROPHONE_L_VOL_INIT	24
#define CX20922_MICROPHONE_R_VOL_INIT	24
#define CX20922_LINEIN_VOL_INIT 		100
#define CX20922_HEADPHONE_STATE_INIT	1
#define CX20922_SPEAKER_STATE_INIT 		1
#define CX20922_MONOOUT_STATE_INIT 		1
#define CX20922_MICROPHONE_STATE_INIT	1
#define CX20922_LINEIN_STATE_INIT 		1
#define CX20922_AGC_STATE_INIT 			1
#define CX20922_DRC_STATE_INIT 			1
#define CX20922_CONFIG_INIT				0

static unsigned int volume[] = {
	CX20922_HEADPHONE_VOL_INIT,
	CX20922_SPEAKER_VOL_INIT,
	CX20922_MONOOUT_VOL_INIT,
	CX20922_MICROPHONE_L_VOL_INIT,
	CX20922_MICROPHONE_R_VOL_INIT,
	CX20922_LINEIN_VOL_INIT,
	CX20922_CONFIG_INIT,
};

static const char *cx20922_configurations[] = { "CONF", "ASR2", "ASR4", "VOI2", "VOI4", "DC16", "DC48", "MP16", "MPTS", "R6CH" };

static const struct soc_enum cx20922_enum[] = {
	SOC_ENUM_SINGLE(0, 0, 10, cx20922_configurations),
};

static int cx20922_cap_volume_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 60;
	return 0;
}

static int cx20922_cap_volume_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = volume[3];
	ucontrol->value.integer.value[1] = volume[4];

	return 0;
}

//static int two_wm8960_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int value)
static int cx20922_cap_volume_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct	snd_soc_codec *codec = &cx2074x->codec1;
	Command cmd;
	int i;

	for (i = 0 ; i < 2 ; i++ )
	{
		SendCmd(codec, &cmd, APP_ID_STRM, STREAMER_APP_SET_CONFIG_IBIZA, 2, IBIZA_ADC0_BOOST + i, ucontrol->value.integer.value[i]);

		volume[3+i] = ucontrol->value.integer.value[i];
	}

	return 2;
}
static int cx20922_config_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
//	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;

	ucontrol->value.enumerated.item[0] = ucontrol->value.enumerated.item[1] = volume[6];

	return 0;
}

static int cx20922_config_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = &cx2074x->codec1;
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	Command cmd;
	int val, ret;

	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;

	val = ucontrol->value.enumerated.item[0];

	ret = SendCmd(codec, &cmd, APP_ID_CTRL, CMD_SET(CONTROL_APP_EXEC_FILE), 1, ID(e->texts[val][0], e->texts[val][1], e->texts[val][2], e->texts[val][3]));

	volume[6] = val;

	if (ret < 0)
		ERROR("%s : SendCmd failed result=%d\n", __func__, ret);

	return (ret < 0);
}

static const struct snd_kcontrol_new cx2074x_snd_controls[] = {
	SOC_DOUBLE("Line Out 1 Power Switch", LINE_OUT_CTRL, 0, 0, 0x3, 0x00),
	SOC_DOUBLE("Line Out 2 Power Switch", LINE_OUT_CTRL, 0, 4, 0x3, 0x00),

	SOC_SINGLE("Speaker Power Switch",	CLASS_D_CTRL,	0, 0x3, 0),
	SOC_SINGLE("Speaker Output Switch", CLASS_D_CTRL, 2, 0x1, 0),
	SOC_SINGLE("Speaker Mono Mode", CLASS_D_CTRL, 3, 0x1, 0),
	SOC_SINGLE("Speaker PWM Switch", CLASS_D_CTRL, 4, 0x1, 0),
	
	SOC_SINGLE("Headphone A Power Switch",	HEADPHONE_CTRL, 0, 0x3, 0),
	SOC_SINGLE("Headphone B Power Switch",	HEADPHONE_CTRL, 3, 0x3, 0),
	
	SOC_SINGLE("Mic Bias Set", MIC_BIAS_CTRL, 	1, 0x1, 0),
	
	//SOC_SINGLE("Digital Mic Mute", DIGITAL_MIC_CTRL, 0, 0x1, 0),
	//SOC_SINGLE("Digital Mic Power Switch", DIGITAL_MIC_CTRL, 1, 0x1, 0),
	//SOC_SINGLE_TLV("Digital Mic Left Gain", DIGITAL_MIC_CTRL, 2, 0x4, 0, digi_mic_gain_tlv),
	//SOC_SINGLE_TLV("Digital Mic Right Gain", DIGITAL_MIC_CTRL, 5, 0x4, 0, digi_mic_gain_tlv),
	
	//SOC_SINGLE("ADC Stereo Switch", ADC_SAMPLE_RATE_SIZE, 0, 0x1, 0),
	//SOC_SINGLE("ADC Connection Index", ADC_SAMPLE_RATE_SIZE, 1, 0x1, 0),
	//SOC_SINGLE("ADC AGC Block Switch", ADC_SAMPLE_RATE_SIZE, 2, 0x1, 0),
	//SOC_SINGLE("ADC Swap", ADC_SAMPLE_RATE_SIZE, 3, 0x1, 0),
	//SOC_SINGLE("ADC Analog Left Mute", 	ADC_L_CTRL, 7, 0x1, 0),
	//SOC_SINGLE("ADC Analog Right Mute", ADC_R_CTRL, 4, 0x1, 0),
    //SOC_DOUBLE_R_TLV("ADC Analog Gain", ADC_L_CTRL, ADC_R_CTRL, 1, 0x3, 0, adc_gain_tlv),
    //SOC_SINGLE("ADCL Mute", ADC_L_VOLUME_CTRL, 7, 0x1, 0),
    //SOC_SINGLE("ADCR Mute", ADC_R_VOLUME_CTRL, 7, 0x1, 0),
    //SOC_DOUBLE_R_TLV("Capture Volume", ADC_R_VOLUME_CTRL, ADC_L_VOLUME_CTRL, 0, 0x50, 0, adc_tlv),

    //                  (xname,            reg_left,             reg_right,       xshift, xmax, xinvert, xhandler_get, xhandler_put, tlv_array)
    //SOC_DOUBLE_R_EXT_TLV("Capture Volume", CX20922_MICROPHONE_L, CX20922_MICROPHONE_R, 0, 48, 0, cx20922_get_volsw, cx20922_put_volsw, adc_tlv_cx20922),

    //                 (xname, xreg, shift_left, shift_right, xmax, xinvert, xhandler_get, xhandler_put, tlv_array)
    //SOC_DOUBLE_EXT_TLV("Capture Volume", CX20922_MICROPHONE_L, 0, 5, 48, 0, cx20922_get_volsw, cx20922_put_volsw, adc_tlv_cx20922),
	
    //            (xname, xreg, shift_left, shift_right, xmax, xinvert, xhandler_get, xhandler_put)
    //SOC_DOUBLE_EXT("Capture Volume", CX20922_MICROPHONE_L, 0, 5, 48, 0, cx20922_get_volsw, cx20922_put_volsw),

    SOC_SINGLE("DAC Swap", DAC_SAMPLE_RATE_SIZE, 2, 0x1, 0),
    SOC_SINGLE("DAC1 Mute", DAC1_VOLUME_CTRL, 7, 0x1, 0),
    SOC_SINGLE("DAC2 Mute", DAC2_VOLUME_CTRL, 7, 0x1, 0),
    SOC_DOUBLE_R_TLV("Playback Volume", DAC1_VOLUME_CTRL, DAC2_VOLUME_CTRL, 0, 0x50, 0, dac_tlv),

    { .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
      .name  = "Capture Volume",
      .info = cx20922_cap_volume_info,
      .get = cx20922_cap_volume_get,
      .put = cx20922_cap_volume_put, },

    { .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
      .name = "CX20922 Configuration Switch",
      .info = snd_soc_info_enum_double,
      .get = cx20922_config_get,
      .put = cx20922_config_put,
      .private_value = (unsigned long)&cx20922_enum[0] },
};

static int NOINLINE reg_index(u8 reg)
{
	int i;

	for(i = 0 ; i < noof(cx2074x_regs) ; i++)
	{
		if (cx2074x_regs[i].addr == reg)
			return i;
	}

	return i;
}

static int NOINLINE cx2074x_i2c_write(struct snd_soc_codec *codec, u8 reg, u8 value)
{
	struct i2c_client  *client = (struct i2c_client*) codec->control_data;
	struct i2c_adapter *adap = client->adapter;
	u8 data[2] = {reg, value};
	
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 2,
		.buf	= data, 
	};

#if DEBUG
	dev_info(&client->dev, "Addr %X, Write REG %02x %02x\n", client->addr, data[0], data[1]);
#endif

	if (i2c_transfer(adap, &msg, 1) != msg.len)
		return -EIO;

    return 0;
}

static int NOINLINE cx2074x_i2c_read(struct snd_soc_codec *codec, u8 reg, u8* data)
{
	struct i2c_client *client = (struct i2c_client*) codec->control_data;
	struct i2c_adapter *adap = client->adapter;
	
	struct i2c_msg msg[2] = {
		{
			.addr		= client->addr,
			.flags		= 0,
			.len		= 1,
			.buf		= &reg,
		},
		{
			.addr		= client->addr,
			.flags		= I2C_M_RD,
			.len		= 1,
			.buf		= data,
		}
	};

	if (i2c_transfer(adap, msg, 2) != 1)
		return -EIO;
		
    return 0;
}

static inline void cx2074x_write_reg_cache(struct snd_soc_codec *codec, unsigned int reg, unsigned int value)
{
    cx2074x_reg_t *reg_cache = GET_REG_CACHE(codec);
    int idx;
    
    if ((reg & PLAIN_INDEX_MARK) == 0) {
    	idx = reg_index(reg);
    } else {
    	idx = reg & 0xFFFF;
    }
    if (idx >= noof(cx2074x_regs)) {
		ERROR("%s, the reg number %d is out of range\n", __func__, idx);
        return;
    }

	mutex_lock(&reg_cache_lock);
    reg_cache[idx] = (u8)value;
	mutex_unlock(&reg_cache_lock);
}

static inline unsigned int cx2074x_read_reg_cache(struct snd_soc_codec *codec, unsigned int reg)
{
    cx2074x_reg_t *reg_cache = GET_REG_CACHE(codec);
    unsigned int data;
    int idx;
    
    if ((reg & PLAIN_INDEX_MARK) == 0) {
    	idx = reg_index(reg);
    } else {
    	idx = reg & 0xFFFF;
    }
	if (idx >= noof(cx2074x_regs)) {
		ERROR("%s, the reg number %d is out of range\n", __func__, idx);
		return (unsigned int) 0;
	}
	
	mutex_lock(&reg_cache_lock);
	data = reg_cache[idx];
	mutex_unlock(&reg_cache_lock);

    return data;
}

static int NOINLINE cx2074x_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int value)
{
    int idx;
	int ret;
	
	if ((reg & PLAIN_INDEX_MARK) == 0) {
		idx = reg_index(reg);
	} else {
		idx = reg & 0xFFFF;
	}

#if DEBUG	
	INFO("reg idx = %d reg=%x val = %x\n", idx, cx2074x_regs[idx].addr, (u8)value);
#endif
	ret = cx2074x_i2c_write(codec, cx2074x_regs[idx].addr, (u8)value);
	if (!ret) {
		// Success! Update reg cache
		cx2074x_write_reg_cache(codec, reg, value);
	} else
		ERROR("%s Failed\n", __func__);

	return ret;
}

#if 0 /* this is useless now. */
static unsigned int NOINLINE cx2074x_read(struct snd_soc_codec *codec, unsigned int reg)
{
	int ret;
	u8 data = 0;
    int idx;

    if ((reg & PLAIN_INDEX_MARK) == 0) {
		idx = reg_index(reg);
	} else {
		idx = reg & 0xFFFF;
	}

	ret = cx2074x_i2c_read(codec, cx2074x_regs[idx].addr, &data);
	if (!ret) {
		// Success! Update reg cache
		//INFO("%s Success\n", __func__);
		cx2074x_write_reg_cache(codec, reg, data);
	} else
		ERROR("%s Failed\n", __func__);

	return data;
}
#endif

// add non dapm controls
static int cx2074x_add_controls(struct snd_soc_codec *codec)
{
#if DEBUG
    INFO("%s() called\n", __func__);
#endif
    return (snd_soc_add_controls(codec, cx2074x_snd_controls, ARRAY_SIZE(cx2074x_snd_controls)));
}

static int micbias_event(struct snd_soc_dapm_widget *w,
							struct snd_kcontrol *kcontrol, int event)
{
    unsigned int old;
    struct snd_soc_codec * codec = w->codec;
    unsigned int on = 0x0 << 1;
    
#if DEBUG
    INFO("%s\n", __func__);
#endif
    old = cx2074x_read_reg_cache(codec, MIC_BIAS_CTRL);
    old &=~ on;
    old &= 0xFF;
    switch (event) {
    case SND_SOC_DAPM_POST_PMU:
    	CodecPowerOnOff(1);
        cx2074x_write(codec, MIC_BIAS_CTRL, old | on );
        break;
    case SND_SOC_DAPM_POST_PMD:
        cx2074x_write(codec, MIC_BIAS_CTRL, old);
        break;
    }
    return 0;
}

static int cx2074x_hpapower_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *kcontrol, int event)
{
    unsigned int old;
    struct snd_soc_codec * codec = w->codec;
    unsigned int off = 0x03;
    
#if DEBUG
    INFO("%s\n", __func__);
#endif
    old = cx2074x_read_reg_cache(codec, HEADPHONE_CTRL);
    switch (event) {
    case SND_SOC_DAPM_POST_PMU:
    	CodecPowerOnOff(1);
        cx2074x_write(codec, HEADPHONE_CTRL, (old & (~off)));
        break;
    case SND_SOC_DAPM_POST_PMD:
        cx2074x_write(codec, HEADPHONE_CTRL, (old | off));
        break;
    }
    return 0;
}

static int cx2074x_hpbpower_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *kcontrol, int event)
{
    unsigned int old;
    struct snd_soc_codec * codec = w->codec;
    unsigned int off = 0x03 << 3;
    
#if DEBUG
    INFO("%s\n", __func__);
#endif
    old = cx2074x_read_reg_cache(codec, HEADPHONE_CTRL);
    switch (event) {
    case SND_SOC_DAPM_POST_PMU:
		CodecPowerOnOff(1);
        cx2074x_write(codec, HEADPHONE_CTRL, (old & (~off)));
        break;
    case SND_SOC_DAPM_POST_PMD:
        cx2074x_write(codec, HEADPHONE_CTRL, (old | off));
        break;
    }
    return 0;
}

static int cx2074x_lineout1power_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *kcontrol, int event)
{
    unsigned int old;
    struct snd_soc_codec * codec = w->codec;
    unsigned int off = 0x03;
    
#if DEBUG
    INFO("%s\n", __func__);
#endif
    
    old = cx2074x_read_reg_cache(codec, LINE_OUT_CTRL);
    switch (event) {
    case SND_SOC_DAPM_POST_PMU:
    	CodecPowerOnOff(1);
        cx2074x_write(codec, LINE_OUT_CTRL, (old & (~off)));
        break;
    case SND_SOC_DAPM_POST_PMD:
        cx2074x_write(codec, LINE_OUT_CTRL, (old | off));
        break;
    }
    return 0;
}

static int cx2074x_lineout2power_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *kcontrol, int event)
{
    unsigned int old;
    struct snd_soc_codec * codec = w->codec;
    unsigned int off = 0x03 << 4;
    
#if DEBUG
    INFO("%s\n", __func__);
#endif
    
    old = cx2074x_read_reg_cache(codec, LINE_OUT_CTRL);
    switch (event) {
    case SND_SOC_DAPM_POST_PMU:
    	CodecPowerOnOff(1);
        cx2074x_write(codec, LINE_OUT_CTRL, (old & (~off)));
        break;
    case SND_SOC_DAPM_POST_PMD:
        cx2074x_write(codec, LINE_OUT_CTRL, (old | off));
        break;
    }
    return 0;
}

static int cx2074x_spkrpower_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *kcontrol, int event)
{
    unsigned int old;
    struct snd_soc_codec * codec = w->codec;
    unsigned int off = 0x03;
    
#if DEBUG
    INFO("%s\n", __func__);
#endif
    
    old = cx2074x_read_reg_cache(codec, CLASS_D_CTRL);
    switch (event) {
    case SND_SOC_DAPM_POST_PMU:
    	CodecPowerOnOff(1);
        cx2074x_write(codec, CLASS_D_CTRL, (old & (~off)));
        break;
    case SND_SOC_DAPM_POST_PMD:
        cx2074x_write(codec, CLASS_D_CTRL, (old | off));
        break;
    }
    return 0;
}

static const struct snd_kcontrol_new output_mixer[] = {
	SOC_DAPM_SINGLE	("SPKR Mono", CLASS_D_CTRL, 3, 0, 0),
	SOC_DAPM_SINGLE	("DAC Swap", DAC_SAMPLE_RATE_SIZE, 2, 0, 0),
};

static const char *rinput_mode_text[] = {
	"Differential Input"
};

static const struct soc_enum rinput_mode_enum =
	SOC_ENUM_SINGLE(ADC_R_CTRL, 7, 1, rinput_mode_text);

static const struct snd_kcontrol_new rinput_mode_controls =
	SOC_DAPM_ENUM("MICR Mode", rinput_mode_enum);

static const struct snd_soc_dapm_widget cx2074x_dapm_widgets[] = {
    //Playback 
	SND_SOC_DAPM_DAC	("DACL",	"Left Playback",	DAC_SAMPLE_RATE_SIZE, 0, 0),
	SND_SOC_DAPM_DAC	("DACR",	"Right Playback",	DAC_SAMPLE_RATE_SIZE, 1, 0), 
	
	SND_SOC_DAPM_MIXER	("Output Mixer", SND_SOC_NOPM, 0, 0, output_mixer,
							ARRAY_SIZE(output_mixer)),
							
	SND_SOC_DAPM_PGA	("SPKR PGA", CLASS_D_CTRL, 2, 0, NULL, 0),
	SND_SOC_DAPM_PGA	("HPA PGA", HEADPHONE_CTRL, 2, 0, NULL, 0),
	SND_SOC_DAPM_PGA	("HPB PGA", HEADPHONE_CTRL, 5, 0, NULL, 0),
	SND_SOC_DAPM_PGA	("LINEOUT1 PGA", LINE_OUT_CTRL, 2, 0, NULL, 0),
	SND_SOC_DAPM_PGA	("LINEOUT2 PGA", LINE_OUT_CTRL, 6, 0, NULL, 0),
							
	//Capture
	SND_SOC_DAPM_ADC	("ADCL",	"Left Capture",		ADC_L_CTRL, 0, 0),
	SND_SOC_DAPM_ADC	("ADCR",	"Right Capture",	ADC_R_CTRL, 0, 0),
	
    SND_SOC_DAPM_MICBIAS_E("MIC Bias", MIC_BIAS_CTRL, 0, 0, micbias_event,
							SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
    SND_SOC_DAPM_MUX	("MICR Mode Mux", SND_SOC_NOPM, 0, 0, &rinput_mode_controls),

    
    //Output Pin.
    SND_SOC_DAPM_OUTPUT	("SPKR OUT"),
    SND_SOC_DAPM_OUTPUT	("HPA OUT"),
    SND_SOC_DAPM_OUTPUT	("HPB OUT"),
    SND_SOC_DAPM_OUTPUT ("LINEOUT1"),
    SND_SOC_DAPM_OUTPUT ("LINEOUT2"),
    
    //Input Jacks
    SND_SOC_DAPM_INPUT	("MICL IN"),
    SND_SOC_DAPM_INPUT	("MICR IN"),
   
	SND_SOC_DAPM_SUPPLY	("HPA Power", SND_SOC_NOPM, 0, 0, 
							cx2074x_hpapower_event,
							SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY	("HPB Power", SND_SOC_NOPM, 0, 0, 
							cx2074x_hpbpower_event,
							SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY	("LINEOUT1 Power", SND_SOC_NOPM, 0, 0, 
							cx2074x_lineout1power_event,
							SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY	("LINEOUT2 Power", SND_SOC_NOPM, 0, 0, 
							cx2074x_lineout2power_event,
							SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY	("SPKR Power", SND_SOC_NOPM, 0, 0,  
							cx2074x_spkrpower_event,
							SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route cx2074x_routes[] = 
{
    //playback.
    {	"Output Mixer",	NULL,	"DACL"		},
    {	"Output Mixer",	NULL,	"DACR"		},
    
    {	"SPKR PGA",		NULL,	"Output Mixer"	},
    {	"HPA PGA",		NULL,	"Output Mixer"	},
    {	"HPB PGA",		NULL,	"Output Mixer"	},
    {	"LINEOUT1 PGA", NULL, 	"Output Mixer"	},
    {	"LINEOUT2 PGA", NULL, 	"Output Mixer"	},
    
    {	"SPKR PGA",		NULL,	"SPKR Power"	},
    {	"HPA PGA",		NULL,	"HPA Power"		},
    {	"HPB PGA",		NULL,	"HPB Power"		},
    {	"LINEOUT1 PGA", NULL, 	"LINEOUT1 Power"},
    {	"LINEOUT2 PGA", NULL, 	"LINEOUT2 Power"},
    
    {	"SPKR OUT",		NULL,	"SPKR PGA"	},
    {	"HPA OUT",		NULL,	"HPA PGA"	},
    {	"HPB OUT",		NULL,	"HPB PGA"	},
    {	"LINEOUT1", 	NULL,	"LINEOUT1 PGA"},
    {	"LINEOUT2", 	NULL,	"LINEOUT2 PGA"},
    
	//capture
    {	"MIC Bias", 	NULL,	"MICL IN"   },
    {	"MICR Mode Mux","Differential Input",	"MICR IN"   },
    {	"MIC Bias",		NULL,	"MICR Mode Mux"},
    {	"ADCL",		  	NULL,	"MIC Bias"  },
    {	"ADCR",			NULL,	"MIC Bias"	},
};


static int cx2074x_add_widgets(struct snd_soc_codec *codec)
{
#if DEBUG
    INFO("%s() called\n", __func__);
#endif

    snd_soc_dapm_new_controls(codec, cx2074x_dapm_widgets,
								ARRAY_SIZE(cx2074x_dapm_widgets));
    snd_soc_dapm_add_routes(codec, cx2074x_routes,
								ARRAY_SIZE(cx2074x_routes));
    return 0;
}

static int cx2074x_dac_hw_params(struct snd_pcm_substream *substream,
									struct snd_pcm_hw_params *params,
									struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned char rate_value = 0x00;
	unsigned char fmt_value = 0x00;

#if DEBUG
	INFO("%s() called\n", __func__);
#endif
	
	fmt_value = cx2074x_read_reg_cache(codec, I2S_PCM_CTRL_1);
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		fmt_value &= ~(I2S_PCM_DAC_8_BIT | I2S_PCM_DAC_8_BIT); 
		fmt_value |= I2S_PCM_DAC_8_BIT; 
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		fmt_value &= ~(I2S_PCM_DAC_16_BIT | I2S_PCM_DAC_16_BIT); 
		fmt_value |= I2S_PCM_DAC_16_BIT; 
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		fmt_value &= ~(I2S_PCM_DAC_24_BIT | I2S_PCM_DAC_24_BIT); 
		fmt_value |= I2S_PCM_DAC_24_BIT; 
		break;
	default: return -EINVAL;
	}
/*
	switch (params_rate(params)) {
	case  8000:	rate_value |= DAC_RATE_8000; break;
	case 11025:	rate_value |= DAC_RATE_11025; break;
	case 16000:	rate_value |= DAC_RATE_16000; break;
	case 22050:	rate_value |= DAC_RATE_22050; break;
	case 32000:	rate_value |= DAC_RATE_32000; break;
	case 44100:	rate_value |= DAC_RATE_44100; break;
	case 48000:	rate_value |= DAC_RATE_48000; break;
	case 88200:	rate_value |= DAC_RATE_88200; break;
	case 96000:	rate_value |= DAC_RATE_96000; break;
	default: return -EINVAL;
	} */
	
	rate_value |= DAC_RATE_96000;

	switch (params_channels(params)) {
	case 1: rate_value |= DAC_LEFT_ENABLE; break;
	case 2: rate_value |= (DAC_LEFT_ENABLE | DAC_RIGHT_ENABLE); break;
	default: return -EINVAL;
	}	

#if DEBUG
	INFO("\tformat:%u speed:%u channels:%u\n", params_format(params),
												params_rate(params),
												params_channels(params)
												);
#endif

	CodecPowerOnOff(1);
	cx2074x_write(codec, I2S_PCM_CTRL_1, (unsigned int)fmt_value);
	cx2074x_write(codec, DAC_SAMPLE_RATE_SIZE, (unsigned int)rate_value);

	return 0;
}

static int cx2074x_adc_hw_params(struct snd_pcm_substream *substream,
									struct snd_pcm_hw_params *params,
									struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned char rate_value = 0x00;
	unsigned char fmt_value = 0x00;

#if DEBUG
	INFO("%s() called\n",__func__);
#endif

	fmt_value = cx2074x_read_reg_cache(codec, I2S_PCM_CTRL_1);
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8: 
		fmt_value &= ~(I2S_PCM_ADC_8_BIT | I2S_PCM_ADC_8_BIT); 
		fmt_value |= I2S_PCM_ADC_8_BIT; 
		break;
	case SNDRV_PCM_FORMAT_S16_LE: 
		fmt_value &= ~(I2S_PCM_ADC_16_BIT | I2S_PCM_ADC_16_BIT); 
		fmt_value |= I2S_PCM_ADC_16_BIT; 
		break;
	case SNDRV_PCM_FORMAT_S24_LE: 
		fmt_value &= ~(I2S_PCM_ADC_24_BIT | I2S_PCM_ADC_24_BIT); 
		fmt_value |= I2S_PCM_ADC_24_BIT; 
		break;
	default: 
		return -EINVAL;
	}

	switch (params_rate(params)) {
	case  8000:	rate_value |= ADC_RATE_8000; break;
	case 11025:	rate_value |= ADC_RATE_11025; break;
	case 16000:	rate_value |= ADC_RATE_16000; break;
	case 22050: rate_value |= ADC_RATE_22050; break;
	case 32000:	rate_value |= ADC_RATE_32000; break;
	case 44100:	rate_value |= ADC_RATE_44100; break;
	case 48000:	rate_value |= ADC_RATE_48000; break;
	default: return -EINVAL;
	}

	switch (params_channels(params)) {
	case 1: break;
	case 2: rate_value |= ADC_ENABLE; break;
	default: 
		return -EINVAL;
	}

#if DEBUG
	INFO("\tformat:%u speed:%u channels:%u\n", params_format(params),
												params_rate(params),
												params_channels(params)
												);
#endif

	cx2074x_write(codec, I2S_PCM_CTRL_1, (unsigned int)fmt_value);
	cx2074x_write(codec, ADC_SAMPLE_RATE_SIZE, (unsigned int)rate_value);

	return 0;
}

static int cx2074x_hw_params(struct snd_pcm_substream *substream, 
								struct snd_pcm_hw_params *params,
								struct snd_soc_dai *dai)

{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return (cx2074x_dac_hw_params(substream, params, dai));
	else
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		return (cx2074x_adc_hw_params(substream, params, dai));
	else
		return -EINVAL;

    return 0;
}

static int cx2074x_mute(struct snd_soc_dai *dai, int mute)
{
    struct snd_soc_codec *codec = dai->codec;
	u8 data[2] = {0, 0};

#if DEBUG
    INFO("%s(,%d) called\n", __func__, mute);
#endif

	data[0] = snd_soc_read(codec, DAC1_VOLUME_CTRL);
	data[1] = snd_soc_read(codec, DAC2_VOLUME_CTRL);

	if (mute) {
		data[0] |= DAC_LEFT_MUTE;
		data[1] |= DAC_RIGHT_MUTE;
	} else {
		data[0] &= ~(DAC_LEFT_MUTE);
		data[1] &= ~(DAC_RIGHT_MUTE);
	}

	if (snd_soc_write(codec, DAC1_VOLUME_CTRL, data[0]))
		ERROR("%s, write DAC1_VOLUME_CTRL failed \n", __func__);
	if (snd_soc_write(codec, DAC2_VOLUME_CTRL, data[1]))
		ERROR("%s, write DAC2_VOLUME_CTRL failed \n", __func__);

    return 0;
}

static int cx2074x_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir)
{
    struct snd_soc_codec *codec = dai->codec;
    struct cx2074x_priv  *maui = get_cx2074x_priv(codec);

#if DEBUG
    INFO("%s() called\n", __func__);
#endif

    // sysclk is not used where, but store it anyway
    maui->sysclk = freq;
    return 0;
}

static int cx2074x_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
    struct snd_soc_codec *codec = dai->codec;
    struct cx2074x_priv *maui = get_cx2074x_priv(codec);
    unsigned char ctrl_value;

#if DEBUG
    INFO("%s() called\n", __func__);
#endif

    // set master/slave audio interface
    switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
    case SND_SOC_DAIFMT_CBS_CFS:	// This design only supports slave mode
        maui->master = 0;
        break;
    default:
		ERROR("unsupport DAI format, driver only supports slave mode\n");
        return -EINVAL;
    }

    switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
    case SND_SOC_DAIFMT_NB_NF:		// This design only supports normal bclk + frm
        break;    
    default:
		ERROR("unsupport DAI format, driver only supports normal bclk+ frm\n");
        return -EINVAL;
    }

    CodecPowerOnOff(1);
    // interface format
    switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
    case SND_SOC_DAIFMT_I2S:		// I2S mode
    	ctrl_value = cx2074x_read_reg_cache(codec, I2S_TX_CTRL_1);
    	ctrl_value &= ~0x07; // Mask out lower 3bits
    	
    	cx2074x_write(codec, I2S_RX_CTRL_1,  (unsigned int)ctrl_value);
    	cx2074x_write(codec, I2S_TX_CTRL_1,  (unsigned int)ctrl_value);
        break;
    case SND_SOC_DAIFMT_LEFT_J:     // Left Justified mode : sony mode
    	ctrl_value = cx2074x_read_reg_cache(codec, I2S_TX_CTRL_1);
    	ctrl_value &= ~0x07; // Mask out lower 3bits
    	
    	ctrl_value |= CTRL_LEFTJUSTIFIED | CTRL_SONYMODE;
    	cx2074x_write(codec, I2S_RX_CTRL_1,  (unsigned int)ctrl_value);
    	cx2074x_write(codec, I2S_TX_CTRL_1,  (unsigned int)ctrl_value);
    	break;
    case SND_SOC_DAIFMT_RIGHT_J:
    	// KSW : Right Justify Mode is not supported now because 
    	// We should know bit delay value for left n bits.
    	/*ctrl_value = ctrl_value = cx2074x_read_reg_cache(codec, I2S_TX_CTRL_1);
    	ctrl_value &= ~0x07; // Mask out lower 3bits
    	
    	ctrl_value |= CTRL_RIGHTJUSTIFIED;
    	cx2074x_write(codec, I2S_RX_CTRL_1,  (unsigned int)ctrl_value);
    	cx2074x_write(codec, I2S_TX_CTRL_1,  (unsigned int)ctrl_value);
    	break; */
    	// So fall through ERROR return
    default:
		ERROR("unspoort DAI format, driver only supports I2S interface.\n");
        return -EINVAL;
    }

    return 0;
}

static struct snd_soc_dai_ops cx2074x_dai_ops = 
{
    .set_sysclk		= cx2074x_set_dai_sysclk,
    .set_fmt		= cx2074x_set_dai_fmt,
    .digital_mute	= cx2074x_mute,
    .hw_params		= cx2074x_hw_params,
};

struct snd_soc_dai soc_codec_cx2074x_cx20922_dai =
{
    .name				= "CX2074X+CX20922",
    .ops				= &cx2074x_dai_ops,
	.symmetric_rates 	= 1,
    .capture = {
        .stream_name	="Capture",
		.rates			= CX20922_RATES_CAPTURE,
		.formats		= CX20922_FORMATS_CAPTURE,
		.channels_min	= 1,
		.channels_max	= 2,
    },
    .playback = {
		.stream_name	="Playback",
		.rates	  		= CX2074x_RATES_PLAYBACK,
		.formats  		= CX2074x_FORMATS_PLAYBACK,
        .channels_min 	= 1,
        .channels_max 	= 2,
     },
};
EXPORT_SYMBOL_GPL(soc_codec_cx2074x_cx20922_dai);

static int cx2074x_set_bias_level(struct snd_soc_codec *codec, enum snd_soc_bias_level level)
{
	int chg = 0, pwrs = 0;
#if DEBUG
    INFO("%s(,%d) called\n", __func__, level);
#endif
	pwrs = cx2074x->pwr_state;
    switch (level)
    {
        // Fully on
    case SND_SOC_BIAS_ON:
        // all power is driven by DAPM system
        if (pwrs > 0) {
        	chg = 1;
        }
        cx2074x->pwr_state = 0;
        break;

        // Partial on
    case SND_SOC_BIAS_PREPARE:
    	if (pwrs > 1) {
        	chg = 1;
        } else 
        if (pwrs == 0) {
        	chg = -1;
        } else {
        	chg = 0;
        }
    	cx2074x->pwr_state = 1;
        break;

        // Off, with power
    case SND_SOC_BIAS_STANDBY:
    	if (pwrs > 2) {
        	chg = 1;
        } else 
        if (pwrs < 2) {
        	chg = -1;
        } else {
        	chg = 0;
        }
        // TODO: power down channel
        cx2074x->pwr_state = 2;
        break;

        // Off, without power
    case SND_SOC_BIAS_OFF:
        // TODO: put channel into deep-sleep
        CodecPowerOnOff(0);
        break;
    }
    if (chg > 0) {
    	if (pwrs == 3) {
    		CodecPowerOnOff(1);
    	}
    } else
    if (chg < 0) {
    	if ((cx2074x->prv_state == 0) && (cx2074x->pwr_state == 2) ) {
        	// Then power off at this stage.
        	CodecPowerOnOff(0);
        }
    }
#if DEBUG    
    INFO("Pwr_status = %d\n", cx2074x->pwr_state);
#endif

    codec->bias_level = level;

    return 0;
}

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int cx2074x_dbg_show_regs(struct seq_file *s, void *unused)
{
	int reg_no = (int)s->private;
	int i = 0;
	
	if (reg_no == noof(cx2074x_regs)) {
		seq_printf(s, "Offset\tType\tValue\tName\n");
		for (i = 0; i < reg_no; i++) {
			seq_printf(s, "0x%02X\t", cx2074x_regs[i].addr);
			switch (cx2074x_regs[i].type) {
			case REG_TYPE_RO:
				seq_printf(s, "R");
			break;
			case REG_TYPE_RW:
				seq_printf(s, "RW");
				break;
			case REG_TYPE_WC:
				break;
			case REG_TYPE_WI:
				break;
			default:
				seq_printf(s, "UNKNOWN\t");
			}
			seq_printf(s, "\t0x%02X\t%s\n", cx2074x_read_reg_cache(cx2074x_codec, i),
											cx2074x_regs[i].name);
		}
		return 0;
	}
	
	seq_printf(s, "Offset:\t0x%02X\n", cx2074x_regs[reg_no].addr);
	
	seq_printf(s, "Type:\t");
	switch (cx2074x_regs[reg_no].type) {
	case REG_TYPE_RO:
		seq_printf(s, "R");
		break;
	case REG_TYPE_RW:
		seq_printf(s, "RW");
		break;
	case REG_TYPE_WC:
		break;
	case REG_TYPE_WI:
		break;
	default:
		seq_printf(s, "UNKNOWN");
	}
	seq_printf(s, "\n");
	
	seq_printf(s, "Value:\t0x%02X\n", cx2074x_read_reg_cache(cx2074x_codec, reg_no));

	return 0;
}

static int cx2074x_dbg_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, cx2074x_dbg_show_regs, inode->i_private);
}

static ssize_t cx2074x_dbg_reg_write(struct file *file,
								const char __user *ubuf,                                                                        
                                size_t count, loff_t *off)
{
	struct seq_file *seq = file->private_data;
	char buf[8];
	unsigned long val;
	int reg_no = (int)seq->private;
	int ret = 0;

	if (count >= sizeof(buf)) {
		ERROR("%s, The buffer is not enough.\n", __func__);
		return -EINVAL;
	} if (copy_from_user(buf, ubuf, count)) {
		ERROR("%s, Faied to copy data from user space.\n", __func__);
		return -EFAULT;
	}
		
	buf[count] = 0;
	
	ret = strict_strtoul(buf, 16, &val);
	if (ret < 0) {
		ERROR("%s, Failed to convert a string to an unsinged long integer.\n", __func__);
		return ret;
	}
		
	switch (cx2074x_regs[reg_no].type) {
	case REG_TYPE_RO:
		ERROR("%s, A read-only register 0x%02x cannot be written.\n",
				__func__, cx2074x_regs[reg_no].addr);
		return -EINVAL; 
	case REG_TYPE_RW:
		ret = cx2074x_write(cx2074x_codec, reg_no, (u8)val); 
		if (ret) {
			ERROR("%s, Failed to write register 0x%02x.\n", __func__,
					cx2074x_regs[reg_no].addr);
			return ret;
		}
		break;
	default:
		ERROR("%s, Unknown type register\n", __func__);
		return -EINVAL;
	}
	
	return count;
}          

static const struct file_operations cx2074x_debug_reg_fops = {
	.open           = cx2074x_dbg_reg_open,
	.read           = seq_read,
	.write			= cx2074x_dbg_reg_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};
#endif

#ifdef CX2074X_JACK_SENSE_INTERRUPT_MODE
static int cx2074x_jack_status_check(void)
{
	struct snd_soc_codec* codec = cx2074x_codec;
    u8 jack_status = 0;
    static int report = 0;
	
	if (cx2074x_read(codec, INT_STATUS) & INT_STS_JACK_SENSE) {
		cx2074x_write(codec, INT_STATUS, INT_STS_JACK_SENSE); // Clear Interrupt Status
		jack_status = cx2074x_read(codec, JACK_SENSE_STATUS);
		
		if (jack_status & JACK_SENSE_HEADPHONE)
			report |= SND_JACK_HEADPHONE;
		else
			report &= (~SND_JACK_HEADPHONE);
			
		if (jack_status & JACK_SENSE_LINEOUT) 
			report |= SND_JACK_LINEOUT;
		else
			report &= (~SND_JACK_LINEOUT);
			
		if (jack_status & JACK_SENSE_LINEIN_L) 
			report |= SND_JACK_BTN_0;
		else
			report &= (~SND_JACK_BTN_0);
			
		if (jack_status & JACK_SENSE_LINEIN_R) 
			report |= SND_JACK_BTN_1;
		else
			report &= (~SND_JACK_BTN_1);
	}
	
	return report;
}

#define CX2074X_CODEC_EVENT_GPIO 15
static struct snd_soc_jack cx2074x_jack;
static struct snd_soc_jack_gpio cx2074x_jack_gpios = {
	.gpio				= CX2074X_CODEC_EVENT_GPIO,
	.name				= "codec-event-gpio",
	.report				= SND_JACK_HEADPHONE | SND_JACK_LINEOUT | SND_JACK_BTN_0 | SND_JACK_BTN_1,
	.invert				= 0,
	.debounce_time		= 200,
	.jack_status_check	= cx2074x_jack_status_check,
};

static struct snd_soc_jack_pin cx2074x_jack_pins[] = {
	{
		.pin	= "HPA OUT",
		.mask	= SND_JACK_HEADPHONE,
		.invert	= 0
	},
	{
		.pin	= "HPB OUT",
		.mask	= SND_JACK_HEADPHONE,
		.invert	= 0
	},
	{
		.pin	= "LINEOUT1",
		.mask	= SND_JACK_LINEOUT,
		.invert	= 0
	},
	{
		.pin	= "LINEOUT2",
		.mask	= SND_JACK_LINEOUT,
		.invert	= 0
	},
	{
		.pin	= "MICL IN",
		.mask	= SND_JACK_BTN_0,
		.invert	= 0
	},
	{
		.pin	= "MICR IN",
		.mask	= SND_JACK_BTN_1,
		.invert	= 0
	},
	{
		.pin	= "SPKR OUT",
		.mask	= SND_JACK_HEADPHONE | SND_JACK_LINEOUT,
		.invert	= 1
	}
};
#endif

//
// Initialise the MAUI driver
// Register the mixer and dsp interfaces with the kernel
//
static int NOINLINE cx2074x_init(struct snd_soc_codec* codec)
{
#ifdef CONFIG_DEBUG_FS
    struct dentry *d, *regs;
#endif
    cx2074x_reg_t *reg_cache;
	int n = 0;
    int ret = 0;

#if DEBUG
    INFO("%s() called\n", __func__);
#endif

	// Initialize the CX2074x regisers
	for(n = 0; n < noof(cx2074x_regs); n++) {
			if (cx2074x_regs->type == REG_TYPE_RW) {
				if (cx2074x_i2c_write(codec, cx2074x_regs[n].addr, cx2074x_data[n]) < 0)
				{
					ERROR("%s, Write 0x%02x Failed\n", __func__, cx2074x_regs[n].addr);
					return -1;
				}
#if DEBUG
				else
					INFO("%s, Write 0x%02x Successfully\n", __func__, cx2074x_regs[n].addr);
#endif
			}
	}

	// Read back and print for debug purposes
	reg_cache = GET_REG_CACHE(codec);
	for(n = 0; n < noof(cx2074x_regs); n++) {
		if (cx2074x_i2c_read(codec, cx2074x_regs[n].addr, &reg_cache[n]) < 0)
			ERROR("%s, Read 0x%02x Failed\n", __func__, cx2074x_regs[n].addr);
#if DEBUG
		else
			INFO("cx2074x_reg 0x%02x = 0x%02x\n", cx2074x_regs[n].addr, reg_cache[n]);
#endif
	}

#if DEBUG
    INFO("%s, Device ID = 0x%02x%02x (%d)\n", __func__, 
				cx2074x_read_reg_cache(codec, DEV_ID_MSB),
				cx2074x_read_reg_cache(codec, DEV_ID_LSB),
				((cx2074x_read_reg_cache(codec, DEV_ID_MSB) << 8) |
				cx2074x_read_reg_cache(codec, DEV_ID_LSB)));
#endif
#ifdef CONFIG_DEBUG_FS
    d = debugfs_create_dir("cx2074x", NULL);
	if (IS_ERR(d))
		return PTR_ERR(d);
		
    regs = debugfs_create_dir("regs", d);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	
	for (n = 0; n < noof(cx2074x_regs); n++)	
		debugfs_create_file(cx2074x_regs[n].name, 0644, regs, (void *)n,
								&cx2074x_debug_reg_fops);
								
	debugfs_create_file("ALL", 0644, regs, (void *)n,
							&cx2074x_debug_reg_fops);
#endif

#ifdef CX2074X_JACK_SENSE_INTERRUPT_MODE
    ret = snd_soc_jack_new(codec, "CX2074X Jack Sense",
        SND_JACK_HEADSET | SND_JACK_LINEOUT, &cx2074x_jack);
    if (ret)
        ERROR("channel: failed to register Headset Jack\n");

    ret = snd_soc_jack_add_gpios(&cx2074x_jack, 1,
        &cx2074x_jack_gpios);
    if (ret)
        ERROR("channel: failed to add jack gpios.\n");
    
    ret = snd_soc_jack_add_pins(&cx2074x_jack, ARRAY_SIZE(cx2074x_jack_pins),
        cx2074x_jack_pins);
    if (ret)
        ERROR("channel: failed to add soc jack pin\n");
#endif

#if DEBUG
    if (ret == 0)
        INFO("channel: codec is ready.\n");
#endif
    return ret;
}

static int cx2074x_probe(struct platform_device *pdev)
{
    struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

#if DEBUG
    INFO("%s() called\n", __func__);
#endif

	if (!cx2074x_codec) {
		dev_err(&pdev->dev, "CX274X codec not yet registered\n");
		return -EINVAL;
	}

	socdev->card->codec = cx2074x_codec;
	codec = cx2074x_codec;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "CX274X: failed to create pcms\n");
		return -EINVAL;
	}

	cx2074x_add_controls(codec);
	cx2074x_add_widgets(codec);

    printk(KERN_INFO "cx2074x codec driver version: %02x,%02x,%02x,%02x\n",(u8)((CX2074X_DRIVER_VERSION)>>24),
      (u8)((CX2074X_DRIVER_VERSION)>>16),
      (u8)((CX2074X_DRIVER_VERSION)>>8),
      (u8)((CX2074X_DRIVER_VERSION)));

    return cx2074x_init(codec);
}

static int cx2074x_remove(struct platform_device *pdev)
{
    struct snd_soc_device *socdev = platform_get_drvdata(pdev);

#if DEBUG
    INFO("%lu: %s() called\n",jiffies,__func__);
#endif
    cx2074x_set_bias_level(cx2074x_codec, SND_SOC_BIAS_OFF);
    snd_soc_free_pcms(socdev);
    snd_soc_dapm_free(socdev);

#ifdef CX2074X_JACK_SENSE_INTERRUPT_MODE
    snd_soc_jack_free_gpios(&cx2074x_jack, 1, &cx2074x_jack_gpios);
#endif

    return 0;
}

static int cx2074x_suspend(struct platform_device *pdev, pm_message_t state)
{
    //struct snd_soc_device *socdev = platform_get_drvdata(pdev);
    //struct snd_soc_codec  *codec = socdev->card->codec;

    //INFO("%lu: %s() called\n",jiffies,__func__);
    //cx2074x_set_bias_level(codec, SND_SOC_BIAS_OFF);
    return 0;
}

static int cx2074x_resume(struct platform_device *pdev)
{
    struct snd_soc_device *socdev = platform_get_drvdata(pdev);
    struct snd_soc_codec *codec = socdev->card->codec;
    int                       n;

#if DEBUG
    INFO("%lu: %s() called\n",jiffies,__func__);
#endif
    // Sync reg_cache with the hardware
    for(n = 0; n < noof(cx2074x_regs); n++)
		cx2074x_write(codec, n, cx2074x_read_reg_cache(codec, n));

	//cx2074x_set_bias_level(codec, codec->suspend_bias_level);
    return 0;
}

struct snd_soc_codec_device soc_codec_dev_cx2074x_cx20922 = {
    .probe =cx2074x_probe,
    .remove = cx2074x_remove,
    .suspend = cx2074x_suspend,
    .resume = cx2074x_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_cx2074x_cx20922);

static int cx2074x_register(struct cx2074x_priv * cx2074x)
{
	int ret;
	struct snd_soc_codec *codec  = &cx2074x->codec;
	struct snd_soc_codec *codec1 = &cx2074x->codec1;

	if (cx2074x_codec || cx20922_codec) {
		dev_err(codec->dev, "Multiple CX2074X/CX20922 devices not supported\n");
		ret = -EINVAL;
		goto err;
	}

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	mutex_init(&codec1->mutex);
	INIT_LIST_HEAD(&codec1->dapm_widgets);
	INIT_LIST_HEAD(&codec1->dapm_paths);

	codec->name				= "CX2074X";
	codec->owner			= THIS_MODULE;
	codec->read				= cx2074x_read_reg_cache;
	codec->write			= cx2074x_write;
	codec->bias_level		= SND_SOC_BIAS_STANDBY;
	codec->set_bias_level	= cx2074x_set_bias_level;
	codec->dai				= &soc_codec_cx2074x_cx20922_dai;
	codec->num_dai			= 1;
    codec->reg_cache_size	= sizeof(cx2074x_data);
    codec->reg_cache		= kmemdup(cx2074x_data, sizeof(cx2074x_data), GFP_KERNEL);
    codec->private_data 	= cx2074x;

	codec1->name             = "CX20922";
	codec1->owner            = THIS_MODULE;
	codec1->dai              = &soc_codec_cx2074x_cx20922_dai;
	codec1->num_dai          = 1;
	codec1->bias_level       = SND_SOC_BIAS_OFF;
	codec1->private_data     = cx2074x;
	//codec->set_bias_level = cx20922_set_bias_level;


	soc_codec_cx2074x_cx20922_dai.dev = codec->dev;

	cx2074x_codec = codec;
	cx20922_codec = codec1;

	ret = snd_soc_register_codec(cx20922_codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto err;
	}

	ret = snd_soc_register_codec(cx2074x_codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto err;
	}

	ret = snd_soc_register_dais(&soc_codec_cx2074x_cx20922_dai, 1);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAIs: %d\n", ret);
		goto err_codec;
	}

	return 0;

err_codec:
	snd_soc_unregister_codec(codec);
err:

	return ret;
}

static void cx2074x_unregister(struct cx2074x_priv *cx2074x)
{
	cx2074x_set_bias_level(&cx2074x->codec, SND_SOC_BIAS_OFF);
	snd_soc_unregister_dai(&soc_codec_cx2074x_cx20922_dai);
	snd_soc_unregister_codec(&cx2074x->codec);
	snd_soc_unregister_codec(&cx2074x->codec1);
	cx2074x_codec = NULL;
	cx20922_codec = NULL;
}

static int I2cWrite( struct snd_soc_codec *codec, unsigned long cbBuf, unsigned char* pBuf)
{

    struct i2c_client  *client = (struct i2c_client  *)codec->control_data;
    struct i2c_adapter *adap   = client->adapter;
    struct i2c_msg      msg[1];


    msg[0].addr  = client->addr;
    msg[0].flags = client->flags & I2C_M_TEN;
    msg[0].buf   = pBuf;
    msg[0].len   = cbBuf;
    if (i2c_transfer(adap,msg,1) < 0)
    {
        printk(KERN_ERR "Channel: I2cWrite failed.\n");

        return -EIO;
    }

    return 0;
}

static int I2cWriteThenRead( struct snd_soc_codec *codec, unsigned long cbBuf,
    unsigned char* pBuf, unsigned long cbReadBuf, unsigned char*pReadBuf)
{

    struct i2c_client  *client = (struct i2c_client  *)codec->control_data;
    struct i2c_adapter *adap   = client->adapter;
    struct i2c_msg      msg[2];

    msg[0].addr  = client->addr;
    msg[0].flags = client->flags & I2C_M_TEN;
    msg[0].len   = cbBuf;
    msg[0].buf   = pBuf;

    msg[1].addr  = client->addr;
    msg[1].flags = (client->flags & I2C_M_TEN) | I2C_M_RD;
    msg[1].len   = cbReadBuf;
    msg[1].buf   = pReadBuf;

    if (i2c_transfer(adap,msg,2) < 0)
    {
        printk(KERN_ERR "Channel: I2cWriteThenRead failed.\n");
        return -EIO;
    }

	return 0;
}

#define I2C_ADDRESS_OFFSET  2

static int cx20922_i2c_write (struct snd_soc_codec *codec, int address, unsigned *buffer, int size)
{
    unsigned char WrBuf[(MAX_COMMAND_SIZE+3)*4];
    unsigned int  cbWrBuf = sizeof(*buffer)* size +I2C_ADDRESS_OFFSET;
    int			  i;

#if 1
	for(i=0; i < I2C_ADDRESS_OFFSET; i++)
		WrBuf[i] = (unsigned char)(address >> ((I2C_ADDRESS_OFFSET-i-1)*8));
//	(2-1-1) * 8
//	(2-2-1) * 8

#else
	// the address is big-endian, but the data is little-endian.
    address  = address<<  (8* (4-I2C_ADDRESS_OFFSET));
    WrBuf[0] = (unsigned char) (address>>24);
    WrBuf[1] = (unsigned char) (address>>16);
    WrBuf[2] = (unsigned char) (address>>8);
    WrBuf[3] = (unsigned char) (address);
#endif
    memcpy(&WrBuf[I2C_ADDRESS_OFFSET], buffer,sizeof(*buffer)* size);

    return I2cWrite(codec,cbWrBuf,(unsigned char*)WrBuf);
}

static int cx20922_i2c_read(struct snd_soc_codec *codec, int address, unsigned *buffer, int size)
{

    unsigned char WrBuf[4];
    unsigned int  cbWrBuf = I2C_ADDRESS_OFFSET;
    unsigned int  cbRead  = size * sizeof(*buffer);
    unsigned char *RdBuf= (unsigned char*) buffer;

    // the address is big-endian, but the data is little-endian.
    address  = address<<  (8* (4-I2C_ADDRESS_OFFSET));
    WrBuf[0] = (unsigned char) (address>>24);
    WrBuf[1] = (unsigned char) (address>>16);
    WrBuf[2] = (unsigned char) (address>>8);
    WrBuf[3] = (unsigned char) (address);

    return I2cWriteThenRead(codec,cbWrBuf,WrBuf,cbRead,RdBuf);
}

static void sys_sleep (int intreval_ms)
{
}

// TODO: tweak the interval for reply bit polling and its timeout
#define REPLY_POLL_INTERVAL_MSEC     1
#define REPLY_POLL_TIMEOUT_MSEC   2000

static int SendCmd (struct snd_soc_codec *codec, Command *cmd, uint32_t  app_module_id, uint32_t  command_id, uint32_t num_32b_words, ...)
{
  va_list     args;
  uint32_t    n;
  unsigned int *i2c_data;
  int size;
  int elapsed_ms;

  va_start (args , num_32b_words) ;

  // at least two words of header
  if (num_32b_words > MAX_COMMAND_SIZE)
  {
    return(-1);
  }
  cmd->num_32b_words = (command_id&CMD_GET(0)) ? MAX_COMMAND_SIZE : num_32b_words;
  cmd->command_id    = command_id;
  cmd->reply         = 0;
  cmd->app_module_id = app_module_id;

  for (n = 0 ; n < num_32b_words ; n++)
  {
    cmd->data[n] = va_arg(args, int32_t);
  }
  va_end (args) ;

  i2c_data = (unsigned int *)cmd;
  size = num_32b_words + 2;

  INFO("%s : enter cmd->num_32b_words=%d\n", __func__, cmd->num_32b_words);

  // write words 1 to N-1 , to addresses 4 to 4+4*N-1
  if (cx20922_i2c_write (codec, 0x4, &i2c_data[1], size-1))
  {
	  ERROR("%s : i2c write failed\n", __func__);
	  return -2;
  }

  // write word 0 to address 0
  cx20922_i2c_write (codec, 0x0, &i2c_data[0], 1);

  elapsed_ms = 0;

  while (elapsed_ms < REPLY_POLL_TIMEOUT_MSEC)
  {
    // only read the first word and check the reply bit
	  if (cx20922_i2c_read (codec, 0x0, &i2c_data[0], 1) != 0)
	  {
		  ERROR("%s : i2c read failed\n", __func__);
		  return -2;
	  }

    if (cmd->reply==1)
      break;
    sys_sleep(REPLY_POLL_INTERVAL_MSEC);
    elapsed_ms += REPLY_POLL_INTERVAL_MSEC;
  }

  if (cmd->reply==1)
  {
    if (cmd->num_32b_words > 0)
    {
    	cx20922_i2c_read (codec, 0x8, &i2c_data[2], cmd->num_32b_words);
    }
    INFO("%s : leave cmd->num_32b_words=%d\n", __func__, cmd->num_32b_words);

    return(cmd->num_32b_words);
  }
  return(-1);
}

static int cx20922_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct	snd_soc_codec *codec;
	Command cmd;

    codec = &cx2074x->codec1;
    i2c_set_clientdata(i2c, cx2074x);

    codec->control_data = (void*)i2c;
    codec->dev = &i2c->dev;

    cx2074x->gpio->GPIOxOUT |= GPIO_CODEC_POWER_ON;;
    //msleep(500);

	/* golem::sendcmd CTRL CONTROL_APP_SET_MEMORY {MEM_TYPE_Y, 0x3F000, 0x12, 0xFFFFFFFF} */
	//if (SendCmd(codec, &cmd, APP_ID_CTRL, CONTROL_APP_SET_MEMORY, 4 , MEM_TYPE_Y, 0x3F000, 0x12, 0xFFFFFFFF) < 0)
	//	return 1;

    return 0;
}

static int _atoi(const char *name)
{
    int val = 0;

    for (;; name++) {
		switch (*name) {
	    	case '0' ... '9':
				val = 10*val+(*name-'0');
			break;
	    	default:
			return val;
		}
    }
}

static ssize_t cx20745_manpwr_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	//struct cx2074x_priv *cx20745 = dev_get_drvdata(dev);

	return sprintf(buf, "prv status=%d pwr_status=%d\n", cx2074x->prv_state, cx2074x->pwr_state);
}

/* Any write to the power attribute will trigger the event */
static ssize_t cx20745_manpwr_set(struct kobject *kobj, struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	//struct cx2074x_priv *cx20745 = dev_get_drvdata(dev);
	int pwrs;
	char *ps0,*ps1,*pe;

	ps0 = memchr(buf,  '0', count);
	ps1 = memchr(buf,  '1', count);
	pe = memchr(buf, '\n', count);

	/* check argu */
	if (((NULL == ps0) && (NULL == ps1)) || (NULL == pe))
		return count;

	/* get pwr state */
	pwrs = _atoi(buf);
#if DEBUG	
	INFO("pwrs = %d\n", pwrs);
#endif
	if (pwrs != cx2074x->prv_state) {
		if (pwrs != 0) {
			/* turn on ... resume the state, nothing to do with because event would call it. */
		} else {
			CodecPowerOnOff(0);
		}
		cx2074x->prv_state = pwrs;
	}

	return count;
}

void CodecPowerOnOff(int On)
{
	struct snd_soc_codec *codec = &cx2074x->codec;
	struct snd_soc_codec *codec1 = &cx2074x->codec1;
	int                       n;
	Command cmd;

#if DEBUG
	INFO("%s: on-%d prv=%d pwr=%d\n",__func__, On, cx2074x->prv_state, cx2074x->pwr_state);
#endif
	if (On) {
		mutex_lock(&codec_pwr_lock);
		if (cx2074x->pwr_state == 3) {
			/* first turn on CX20922 & 20745 */
			cx2074x->gpio->GPIOxOUT |= GPIO_CODEC_POWER_ON;
			// GPIO setting to turn on
			msleep(500); // wait 500ms is enough?
			/* Next wait a 500ms, send command to CX20922 to turn on 24.576MHz clock */
			/* golem::sendcmd CTRL CONTROL_APP_SET_MEMORY {MEM_TYPE_Y, 0x3F000, 0x12, 0xFFFFFFFF} */
			if (SendCmd(codec1, &cmd, APP_ID_CTRL, CONTROL_APP_SET_MEMORY, 4 , MEM_TYPE_Y, 0x3F000, 0x12, 0xFFFFFFFF) < 0) {
				mutex_unlock(&codec_pwr_lock);
				ERROR("Send command failed from PowerOn.\n");
				return;
			}
	#if DEBUG
			INFO("%lu: %s() called\n",jiffies,__func__);
	#endif
			msleep(150); // wait 100ms is enough?
			// Sync reg_cache with the hardware
			for(n = 0; n < noof(cx2074x_regs); n++)
				cx2074x_write(codec, n | PLAIN_INDEX_MARK, cx2074x_read_reg_cache(codec, n | PLAIN_INDEX_MARK));
			cx2074x->prv_state = 1;
			cx2074x->pwr_state = 2;
			INFO("CODEC PWR RESUME COMPLETED.\n");
			msleep(200); // wait 200ms is enough?
		}
		mutex_unlock(&codec_pwr_lock);
	} else {
		mutex_lock(&codec_pwr_lock);
		if (cx2074x->pwr_state == 2) {
			cx2074x->gpio->GPIOxOUT &= ~GPIO_CODEC_POWER_ON;
			cx2074x->pwr_state = 3;
		}
		mutex_unlock(&codec_pwr_lock);
	}
}


//static DEVICE_ATTR(power_attr, 0644, cx20745_manpwr_show, cx20745_manpwr_set);
static struct kobj_attribute power_attr =
			__ATTR(power, 0644, cx20745_manpwr_show, cx20745_manpwr_set);

static struct attribute * g[] = {
	(struct attribute *)&power_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

struct kobject * soc_kobj;

static int cx2074x_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct	snd_soc_codec *codec;
    int ret = 0;

    codec = &cx2074x->codec;
    i2c_set_clientdata(i2c, cx2074x);

    codec->control_data = (void*)i2c;
    codec->dev = &i2c->dev;

    /* Register the sysfs files for debugging */
	/* Create SysFS files */
	soc_kobj = kobject_create_and_add("codec_pwr", NULL);
	if (! soc_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(soc_kobj, &attr_group);
	if (ret)
		dev_info(&i2c->dev, "error creating sysfs files for codec\n");

    ret = cx2074x_register(cx2074x);
    if (ret < 0)
        dev_err(&i2c->dev, "%s() failed ret = %d\n", __func__, ret);

    return ret;
}

static int cx20922_i2c_remove(struct i2c_client *client)
{
    cx20922_codec = NULL;

    return 0;
}

static int cx2074x_i2c_remove(struct i2c_client *client)
{
	cx2074x_unregister(cx2074x);
	return 0;
}

static const struct i2c_device_id cx20922_i2c_id[] =
{
    { CX20922_I2C_DRIVER_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, cx20922_i2c_id);

static const struct i2c_device_id cx2074x_i2c_id[] =
{
    { CX2074X_I2C_DRIVER_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, cx2074x_i2c_id);

static struct i2c_driver cx20922_i2c_driver = {
    .driver = {
		.name = "CX20922 I2C Codec",
		.owner = THIS_MODULE,
     },
    .probe=cx20922_i2c_probe,
    .remove=__devexit_p(cx20922_i2c_remove),
    .id_table=cx20922_i2c_id,
};

static struct i2c_driver cx2074x_i2c_driver = {
    .driver		= {
		.name	= "CX2074X I2C Codec",
		.owner	= THIS_MODULE,
     },
    .probe		= cx2074x_i2c_probe,
    .remove		= __devexit_p(cx2074x_i2c_remove),
    .id_table	= cx2074x_i2c_id,
};

static int __init cx2074x_modinit(void)
{
    int ret;
    
#if DEBUG
    INFO("%s() called\n", __func__);
#endif
    
    cx2074x = (struct cx2074x_priv *) kzalloc(sizeof (struct cx2074x_priv), GFP_KERNEL);
    if (cx2074x == NULL) {
    	printk(KERN_ERR "%s, memory allocation failed\n", __func__);
        return -ENOMEM;
	}

	cx2074x->prv_state = 1; /* this is turned on */
	cx2074x->pwr_state = 2; /* this is pwr state 2, BIAS_OFF */
	cx2074x->gpio = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO);

	ret = i2c_add_driver(&cx20922_i2c_driver);
	if (ret != 0)
		ERROR("Failed to register CX207922 I2C driver: %d\n", ret);

	ret = i2c_add_driver(&cx2074x_i2c_driver);
	if (ret != 0)
		ERROR("Failed to register CX2074X I2C driver: %d\n", ret);
		
    return ret;
}
module_init(cx2074x_modinit);

static void __exit cx2074x_exit(void)
{
#if DEBUG
    INFO("%s() called\n", __func__);
#endif
    i2c_del_driver(&cx2074x_i2c_driver);
	i2c_del_driver(&cx20922_i2c_driver);
	kfree(cx2074x);
}
module_exit(cx2074x_exit);

MODULE_DESCRIPTION("ASoC CX2074X Codec Driver");
MODULE_LICENSE("GPL");
