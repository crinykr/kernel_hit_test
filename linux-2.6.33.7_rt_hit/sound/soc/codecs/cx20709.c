/*
 * cx20709.c  --  CX20709 ALSA SoC Audio driver
 *
 * (C) Copyright 2012
 * Dong-gweon Oh, Flowdas Inc.,, <prospero@flowdas.com>
 *
 * Author: Liam Girdwood
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <mach/platform.h>

#include "cx20709.h"
#include "cxcodecapi.h"

#define AUDIO_NAME "cx20709"

struct snd_soc_codec_device soc_codec_dev_cx20709;

/* R25 - Power 1 */
#define CX20709_VREF      0x40

/* R28 - Anti-pop 1 */
#define CX20709_POBCTRL   0x80
#define CX20709_BUFDCOPEN 0x10
#define CX20709_BUFIOEN   0x08
#define CX20709_SOFT_ST   0x04
#define CX20709_HPSTBY    0x01

/* R29 - Anti-pop 2 */
#define CX20709_DISOP     0x40

/*
 * cx20709 register cache
 * We can't read the CX20709 register space when we are
 * using 2 wire for device control, so we cache them instead.
 */
static const u16 cx20709_reg[CX20709_CACHEREGNUM] = {
	0x0097, 0x0097, 0x0000, 0x0000,
	0x0000, 0x0008, 0x0000, 0x000a,
	0x01c0, 0x0000, 0x00ff, 0x00ff,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x007b, 0x0100, 0x0032,
	0x0000, 0x00c3, 0x00c3, 0x01c0,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0100, 0x0100, 0x0050, 0x0050,
	0x0050, 0x0050, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0040, 0x0000,
	0x0000, 0x0050, 0x0050, 0x0000,
	0x0002, 0x0037, 0x004d, 0x0080,
	0x0008, 0x0031, 0x0026, 0x00e9,
};

struct cx20709_priv {
	u16 reg_cache[CX20709_CACHEREGNUM];
	struct snd_soc_codec codec;
};

#define cx20709_reset(c)	snd_soc_write(c, CX20709_RESET, 0)

#define CX20709_HEADPHONE 	0
#define CX20709_SPEAKER 	1
#define CX20709_MONOOUT 	2
#define CX20709_MICROPHONE	3
#define CX20709_LINEIN 		4
#define CX20709_AGC 		5
#define CX20709_DRC 		6

#define CX20709_HEADPHONE_VOL_INIT		100
#define CX20709_SPEAKER_VOL_INIT 		100
#define CX20709_MONOOUT_VOL_INIT 		100
#define CX20709_MICROPHONE_VOL_INIT		100
#define CX20709_LINEIN_VOL_INIT 		100
#define CX20709_HEADPHONE_STATE_INIT	1
#define CX20709_SPEAKER_STATE_INIT 		1
#define CX20709_MONOOUT_STATE_INIT 		1
#define CX20709_MICROPHONE_STATE_INIT	1
#define CX20709_LINEIN_STATE_INIT 		1
#define CX20709_AGC_STATE_INIT 			1
#define CX20709_DRC_STATE_INIT 			1

/* enumerated controls */
//static const char *cx20709_deemph[] = {"None", "32Khz", "44.1Khz", "48Khz"};
//static const char *cx20709_polarity[] = {"No Inversion", "Left Inverted",
//	"Right Inverted", "Stereo Inversion"};
//static const char *cx20709_3d_upper_cutoff[] = {"High", "Low"};
//static const char *cx20709_3d_lower_cutoff[] = {"Low", "High"};
//static const char *cx20709_alcfunc[] = {"Off", "Right", "Left", "Stereo"};
//static const char *cx20709_alcmode[] = {"ALC", "Limiter"};
//static const char *cx20709_recordmode[] = {"Stereo", "Left", "Right", "Swap Left&Right"};
//static const char *cx20709_speakermode[] = {"All off","Left only","Right only", "Stereo"};
//static const char *cx20709_micbias[] = {"High", "Low"};
//static const char *cx20709_micboost[] = {"+0dB", "+13dB", "+20dB", "+29dB"};
static const char *cx20709_switchmode[] = {"Off","On"};


static const struct soc_enum cx20709_enum[] = {
//	SOC_ENUM_SINGLE(CX20709_DACCTL1, 1, 4, cx20709_deemph),
//	SOC_ENUM_SINGLE(CX20709_DACCTL1, 5, 4, cx20709_polarity),
//	SOC_ENUM_SINGLE(CX20709_DACCTL2, 5, 4, cx20709_polarity),
//	SOC_ENUM_SINGLE(CX20709_3D, 6, 2, cx20709_3d_upper_cutoff),
//	SOC_ENUM_SINGLE(CX20709_3D, 5, 2, cx20709_3d_lower_cutoff),
//	SOC_ENUM_SINGLE(CX20709_ALC1, 7, 4, cx20709_alcfunc),
//	SOC_ENUM_SINGLE(CX20709_ALC3, 8, 2, cx20709_alcmode),
//	SOC_ENUM_SINGLE(CX20709_ADDCTL1, 2, 4, cx20709_recordmode),
//	SOC_ENUM_SINGLE(CX20709_CLASSD1, 6, 4, cx20709_speakermode),
//	SOC_ENUM_SINGLE(CX20709_ADDCTL4, 0, 2, cx20709_micbias),
//	SOC_ENUM_SINGLE(CX20709_LINPATH, 4, 4, cx20709_micboost),
//	SOC_ENUM_SINGLE(CX20709_RINPATH, 4, 4, cx20709_micboost),
	SOC_ENUM_SINGLE(CX20709_HEADPHONE, 0, 2, cx20709_switchmode),
	SOC_ENUM_SINGLE(CX20709_SPEAKER, 0, 2, cx20709_switchmode),
	SOC_ENUM_SINGLE(CX20709_MONOOUT, 0, 2, cx20709_switchmode),
	SOC_ENUM_SINGLE(CX20709_MICROPHONE, 0, 2, cx20709_switchmode),
	SOC_ENUM_SINGLE(CX20709_LINEIN, 0, 2, cx20709_switchmode),
	SOC_ENUM_SINGLE(CX20709_AGC, 0, 2, cx20709_switchmode),
	SOC_ENUM_SINGLE(CX20709_DRC, 0, 2, cx20709_switchmode),
};

//static const DECLARE_TLV_DB_SCALE(adc_tlv, -9700, 50, 1);
//static const DECLARE_TLV_DB_SCALE(dac_tlv, -12700, 50, 1);
//static const DECLARE_TLV_DB_SCALE(bypass_tlv, -2100, 300, 0);
//static const DECLARE_TLV_DB_SCALE(out_tlv, -12100, 100, 1);
//static const DECLARE_TLV_DB_SCALE(in_tlv, -1725, 75, 0);

static unsigned int state[] = { 
	CX20709_HEADPHONE_STATE_INIT,
	CX20709_SPEAKER_STATE_INIT,
	CX20709_MONOOUT_STATE_INIT,
	CX20709_MICROPHONE_STATE_INIT,
	CX20709_LINEIN_STATE_INIT,
	CX20709_AGC_STATE_INIT,
	CX20709_DRC_STATE_INIT,
};

static int cx20709_get_enum(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	ucontrol->value.enumerated.item[0] = state[e->reg];
	return 0;
}

static int cx20709_put_enum(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val;

	val = ucontrol->value.enumerated.item[0];
	switch(e->reg) {
	case CX20709_HEADPHONE:
		CxSetHeadphoneMute(!val);
		break;
	case CX20709_SPEAKER:
		CxSetSpeakerMute(!val);
		break;
	case CX20709_MONOOUT:
		CxSetMonoOutMute(!val);
		break;
	case CX20709_MICROPHONE:
		CxSetMicrophoneMute(!val, 3);
		break;
	case CX20709_LINEIN:
		CxSetLineInMute(!val, 3);
		break;
	case CX20709_AGC:
		CxEnableAGC(val);
		break;
	case CX20709_DRC:
		CxEnableDRC(val);
		break;
	default:
		return -1;
	}
	state[e->reg] = val;
	return 0;
}

static unsigned int volume[] = { 
	CX20709_HEADPHONE_VOL_INIT,
	CX20709_SPEAKER_VOL_INIT,
	CX20709_MONOOUT_VOL_INIT,
	CX20709_MICROPHONE_VOL_INIT,
	CX20709_LINEIN_VOL_INIT,
};

static int cx20709_get_volsw(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;

	ucontrol->value.integer.value[0] = volume[mc->reg];
	return 0;
}

static int cx20709_put_volsw(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	unsigned int val;

	val = ucontrol->value.integer.value[0];
	switch(mc->reg) {
	case CX20709_HEADPHONE:
		CxSetHeadphoneVolume(val);
		break;
	case CX20709_SPEAKER:
		CxSetSpeakerVolume(val);
		break;
	case CX20709_MONOOUT:
		CxSetMonoOutVolume(val);
		break;
	case CX20709_MICROPHONE:
		CxSetMicrophoneVolume(val, 3);
		break;
	case CX20709_LINEIN:
		CxSetLineInVolume(val);
		break;
	default:
		return -1;
	}
	volume[mc->reg] = val;
	return 0;
}

static const struct snd_kcontrol_new cx20709_snd_controls[] = {
	SOC_SINGLE_EXT("Headphone Playback Volume", CX20709_HEADPHONE, 0, 100, 0, cx20709_get_volsw, cx20709_put_volsw),
	SOC_SINGLE_EXT("Speaker Playback Volume", CX20709_SPEAKER, 0, 100, 0, cx20709_get_volsw, cx20709_put_volsw),
	SOC_SINGLE_EXT("MonoOut Playback Volume", CX20709_MONOOUT, 0, 100, 0, cx20709_get_volsw, cx20709_put_volsw),
	SOC_SINGLE_EXT("Microphone Capture Volume", CX20709_MICROPHONE, 0, 100, 0, cx20709_get_volsw, cx20709_put_volsw),
	SOC_SINGLE_EXT("LineIn Capture Volume", CX20709_LINEIN, 0, 100, 0, cx20709_get_volsw, cx20709_put_volsw),
	SOC_ENUM_EXT("Headphone Playback Off Switch", cx20709_enum[CX20709_HEADPHONE], cx20709_get_enum, cx20709_put_enum),
	SOC_ENUM_EXT("Speaker Playback Off Switch", cx20709_enum[CX20709_SPEAKER], cx20709_get_enum, cx20709_put_enum),
	SOC_ENUM_EXT("MonoOut Playback Off Switch", cx20709_enum[CX20709_MONOOUT], cx20709_get_enum, cx20709_put_enum),
	SOC_ENUM_EXT("Microphone Capture Off Switch", cx20709_enum[CX20709_MICROPHONE], cx20709_get_enum, cx20709_put_enum),
	SOC_ENUM_EXT("LineIn Capture Off Switch", cx20709_enum[CX20709_LINEIN], cx20709_get_enum, cx20709_put_enum),
	SOC_ENUM_EXT("AGC Off Switch", cx20709_enum[CX20709_AGC], cx20709_get_enum, cx20709_put_enum),
	SOC_ENUM_EXT("DRC Off Switch", cx20709_enum[CX20709_DRC], cx20709_get_enum, cx20709_put_enum),
//SOC_DOUBLE_R_TLV("Capture Volume", CX20709_LINVOL, CX20709_RINVOL,
//		 0, 63, 0, in_tlv),
//SOC_DOUBLE_R("Capture Volume ZC Switch", CX20709_LINVOL, CX20709_RINVOL,
//	6, 1, 0),
//SOC_DOUBLE_R("Capture Switch", CX20709_LINVOL, CX20709_RINVOL,
//	7, 1, 0),
//
//SOC_DOUBLE_R_TLV("Playback Volume", CX20709_LDAC, CX20709_RDAC,
//		 0, 255, 0, dac_tlv),
//
//SOC_DOUBLE_R("Headphone Playback ZC Switch", CX20709_LOUT1, CX20709_ROUT1,
//	7, 1, 0),
//
//SOC_DOUBLE_R_TLV("Speaker Playback Volume", CX20709_LOUT2, CX20709_ROUT2,
//		 1, 127, 1, out_tlv),
//SOC_DOUBLE_R("Speaker Playback ZC Switch", CX20709_LOUT2, CX20709_ROUT2,
//	7, 1, 0),
//SOC_ENUM("Speaker Playback Off Switch", cx20709_enum[8]),
//SOC_SINGLE("Speaker DC Volume", CX20709_CLASSD3, 3, 5, 0),
//SOC_SINGLE("Speaker AC Volume", CX20709_CLASSD3, 0, 5, 0),
//
//SOC_SINGLE("PCM Playback -6dB Switch", CX20709_DACCTL1, 7, 1, 0),
//SOC_ENUM("ADC Polarity", cx20709_enum[1]),
//SOC_ENUM("Playback De-emphasis", cx20709_enum[0]),
//SOC_SINGLE("ADC High Pass Filter Switch", CX20709_DACCTL1, 0, 1, 0),
//SOC_ENUM("Record channels", cx20709_enum[7]),
//
//SOC_ENUM("DAC Polarity", cx20709_enum[2]),
//
//SOC_ENUM("3D Filter Upper Cut-Off", cx20709_enum[3]),
//SOC_ENUM("3D Filter Lower Cut-Off", cx20709_enum[4]),
//SOC_SINGLE("3D Volume", CX20709_3D, 1, 15, 0),
//SOC_SINGLE("3D Switch", CX20709_3D, 0, 1, 0),
//
//SOC_ENUM("ALC Function", cx20709_enum[5]),
//SOC_SINGLE("ALC Max Gain", CX20709_ALC1, 4, 7, 0),
//SOC_SINGLE("ALC Target", CX20709_ALC1, 0, 15, 1),
//SOC_SINGLE("ALC Min Gain", CX20709_ALC2, 4, 7, 0),
//SOC_SINGLE("ALC Hold Time", CX20709_ALC2, 0, 15, 0),
//SOC_ENUM("ALC Mode", cx20709_enum[6]),
//SOC_SINGLE("ALC Decay", CX20709_ALC3, 4, 15, 0),
//SOC_SINGLE("ALC Attack", CX20709_ALC3, 0, 15, 0),
//
//SOC_SINGLE("Noise Gate Threshold", CX20709_NOISEG, 3, 31, 0),
//SOC_SINGLE("Noise Gate Switch", CX20709_NOISEG, 0, 1, 0),
//
//SOC_DOUBLE_R_TLV("ADC PCM Capture Volume", CX20709_LADC, CX20709_RADC,
//	0, 255, 0, adc_tlv),
//SOC_ENUM("Left PGA Boost Gain", cx20709_enum[10]),
//SOC_ENUM("Right PGA Boost Gain", cx20709_enum[11]),
//
//SOC_SINGLE_TLV("Left Output Mixer Boost Bypass Volume",
//	       CX20709_BYPASS1, 4, 7, 1, bypass_tlv),
//SOC_SINGLE_TLV("Left Output Mixer LINPUT3 Volume",
//	       CX20709_LOUTMIX, 4, 7, 1, bypass_tlv),
//SOC_SINGLE_TLV("Right Output Mixer Boost Bypass Volume",
//	       CX20709_BYPASS2, 4, 7, 1, bypass_tlv),
//SOC_SINGLE_TLV("Right Output Mixer RINPUT3 Volume",
//	       CX20709_ROUTMIX, 4, 7, 1, bypass_tlv),
//SOC_ENUM("Microphone Bias", cx20709_enum[9]),
};

static const struct snd_kcontrol_new cx20709_lin_boost[] = {
SOC_DAPM_SINGLE("LINPUT2 Switch", CX20709_LINPATH, 6, 1, 0),
SOC_DAPM_SINGLE("LINPUT3 Switch", CX20709_LINPATH, 7, 1, 0),
SOC_DAPM_SINGLE("LINPUT1 Switch", CX20709_LINPATH, 8, 1, 0),
};

static const struct snd_kcontrol_new cx20709_lin[] = {
SOC_DAPM_SINGLE("Boost Switch", CX20709_LINPATH, 3, 1, 0),
};

static const struct snd_kcontrol_new cx20709_rin_boost[] = {
SOC_DAPM_SINGLE("RINPUT2 Switch", CX20709_RINPATH, 6, 1, 0),
SOC_DAPM_SINGLE("RINPUT3 Switch", CX20709_RINPATH, 7, 1, 0),
SOC_DAPM_SINGLE("RINPUT1 Switch", CX20709_RINPATH, 8, 1, 0),
};

static const struct snd_kcontrol_new cx20709_rin[] = {
SOC_DAPM_SINGLE("Boost Switch", CX20709_RINPATH, 3, 1, 0),
};

static const struct snd_kcontrol_new cx20709_loutput_mixer[] = {
SOC_DAPM_SINGLE("PCM Playback Switch", CX20709_LOUTMIX, 8, 1, 0),
SOC_DAPM_SINGLE("LINPUT3 Switch", CX20709_LOUTMIX, 7, 1, 0),
SOC_DAPM_SINGLE("Boost Bypass Switch", CX20709_BYPASS1, 7, 1, 0),
};

static const struct snd_kcontrol_new cx20709_routput_mixer[] = {
SOC_DAPM_SINGLE("PCM Playback Switch", CX20709_ROUTMIX, 8, 1, 0),
SOC_DAPM_SINGLE("RINPUT3 Switch", CX20709_ROUTMIX, 7, 1, 0),
SOC_DAPM_SINGLE("Boost Bypass Switch", CX20709_BYPASS2, 7, 1, 0),
};

static const struct snd_kcontrol_new cx20709_mono_out[] = {
SOC_DAPM_SINGLE("Left Switch", CX20709_MONOMIX1, 7, 1, 0),
SOC_DAPM_SINGLE("Right Switch", CX20709_MONOMIX2, 7, 1, 0),
};

static const struct snd_soc_dapm_widget cx20709_dapm_widgets[] = {
SND_SOC_DAPM_INPUT("LINPUT1"),
SND_SOC_DAPM_INPUT("RINPUT1"),
SND_SOC_DAPM_INPUT("LINPUT2"),
SND_SOC_DAPM_INPUT("RINPUT2"),
SND_SOC_DAPM_INPUT("LINPUT3"),
SND_SOC_DAPM_INPUT("RINPUT3"),

SND_SOC_DAPM_MICBIAS("MICB", CX20709_POWER1, 1, 0),

SND_SOC_DAPM_MIXER("Left Boost Mixer", CX20709_POWER1, 5, 0,
		   cx20709_lin_boost, ARRAY_SIZE(cx20709_lin_boost)),
SND_SOC_DAPM_MIXER("Right Boost Mixer", CX20709_POWER1, 4, 0,
		   cx20709_rin_boost, ARRAY_SIZE(cx20709_rin_boost)),

SND_SOC_DAPM_MIXER("Left Input Mixer", CX20709_POWER3, 5, 0,
		   cx20709_lin, ARRAY_SIZE(cx20709_lin)),
SND_SOC_DAPM_MIXER("Right Input Mixer", CX20709_POWER3, 4, 0,
		   cx20709_rin, ARRAY_SIZE(cx20709_rin)),

SND_SOC_DAPM_ADC("Left ADC", "Capture", CX20709_POWER1, 3, 0),
SND_SOC_DAPM_ADC("Right ADC", "Capture", CX20709_POWER1, 2, 0),

SND_SOC_DAPM_DAC("Left DAC", "Playback", CX20709_POWER2, 8, 0),
SND_SOC_DAPM_DAC("Right DAC", "Playback", CX20709_POWER2, 7, 0),

SND_SOC_DAPM_MIXER("Left Output Mixer", CX20709_POWER3, 3, 0,
	&cx20709_loutput_mixer[0],
	ARRAY_SIZE(cx20709_loutput_mixer)),
SND_SOC_DAPM_MIXER("Right Output Mixer", CX20709_POWER3, 2, 0,
	&cx20709_routput_mixer[0],
	ARRAY_SIZE(cx20709_routput_mixer)),

SND_SOC_DAPM_MIXER("Mono Output Mixer", CX20709_POWER2, 1, 0,
	&cx20709_mono_out[0],
	ARRAY_SIZE(cx20709_mono_out)),

SND_SOC_DAPM_PGA("LOUT1 PGA", CX20709_POWER2, 6, 0, NULL, 0),
SND_SOC_DAPM_PGA("ROUT1 PGA", CX20709_POWER2, 5, 0, NULL, 0),

SND_SOC_DAPM_PGA("Left Speaker PGA", CX20709_POWER2, 4, 0, NULL, 0),
SND_SOC_DAPM_PGA("Right Speaker PGA", CX20709_POWER2, 3, 0, NULL, 0),

SND_SOC_DAPM_PGA("Right Speaker Output", CX20709_CLASSD1, 7, 0, NULL, 0),
SND_SOC_DAPM_PGA("Left Speaker Output", CX20709_CLASSD1, 6, 0, NULL, 0),

SND_SOC_DAPM_OUTPUT("SPK_LP"),
SND_SOC_DAPM_OUTPUT("SPK_LN"),
SND_SOC_DAPM_OUTPUT("HP_L"),
SND_SOC_DAPM_OUTPUT("HP_R"),
SND_SOC_DAPM_OUTPUT("SPK_RP"),
SND_SOC_DAPM_OUTPUT("SPK_RN"),
SND_SOC_DAPM_OUTPUT("OUT3"),
};

static const struct snd_soc_dapm_route audio_paths[] = {
	{ "Left Boost Mixer", "LINPUT1 Switch", "LINPUT1" },
	{ "Left Boost Mixer", "LINPUT2 Switch", "LINPUT2" },
	{ "Left Boost Mixer", "LINPUT3 Switch", "LINPUT3" },

	{ "Left Input Mixer", "Boost Switch", "Left Boost Mixer", },
	{ "Left Input Mixer", NULL, "LINPUT1", },  /* Really Boost Switch */
	{ "Left Input Mixer", NULL, "LINPUT2" },
	{ "Left Input Mixer", NULL, "LINPUT3" },

	{ "Right Boost Mixer", "RINPUT1 Switch", "RINPUT1" },
	{ "Right Boost Mixer", "RINPUT2 Switch", "RINPUT2" },
	{ "Right Boost Mixer", "RINPUT3 Switch", "RINPUT3" },

	{ "Right Input Mixer", "Boost Switch", "Right Boost Mixer", },
	{ "Right Input Mixer", NULL, "RINPUT1", },  /* Really Boost Switch */
	{ "Right Input Mixer", NULL, "RINPUT2" },
	{ "Right Input Mixer", NULL, "LINPUT3" },

	{ "Left ADC", NULL, "Left Input Mixer" },
	{ "Right ADC", NULL, "Right Input Mixer" },

	{ "Left Output Mixer", "LINPUT3 Switch", "LINPUT3" },
	{ "Left Output Mixer", "Boost Bypass Switch", "Left Boost Mixer"} ,
	{ "Left Output Mixer", "PCM Playback Switch", "Left DAC" },

	{ "Right Output Mixer", "RINPUT3 Switch", "RINPUT3" },
	{ "Right Output Mixer", "Boost Bypass Switch", "Right Boost Mixer" } ,
	{ "Right Output Mixer", "PCM Playback Switch", "Right DAC" },

	{ "Mono Output Mixer", "Left Switch", "Left Output Mixer" },
	{ "Mono Output Mixer", "Right Switch", "Right Output Mixer" },

	{ "LOUT1 PGA", NULL, "Left Output Mixer" },
	{ "ROUT1 PGA", NULL, "Right Output Mixer" },

	{ "HP_L", NULL, "LOUT1 PGA" },
	{ "HP_R", NULL, "ROUT1 PGA" },

	{ "Left Speaker PGA", NULL, "Left Output Mixer" },
	{ "Right Speaker PGA", NULL, "Right Output Mixer" },

	{ "Left Speaker Output", NULL, "Left Speaker PGA" },
	{ "Right Speaker Output", NULL, "Right Speaker PGA" },

	{ "SPK_LN", NULL, "Left Speaker Output" },
	{ "SPK_LP", NULL, "Left Speaker Output" },
	{ "SPK_RN", NULL, "Right Speaker Output" },
	{ "SPK_RP", NULL, "Right Speaker Output" },

	{ "OUT3", NULL, "Mono Output Mixer", }
};

static int cx20709_I2CSPIWriteThenRead(void* pCallbackContext,
	unsigned char ChipAddr, 
	unsigned long cbBuf,
	unsigned char* pBuf,
	unsigned long cbReadBuf, 
	unsigned char*pReadBuf)
{
	struct i2c_client* i2c = (struct i2c_client*)pCallbackContext;
    struct i2c_msg msgs[] = {
        {
            .addr = i2c->addr,
            .flags = 0,
            .len   = cbBuf,
            .buf   = pBuf,
        },
        {
            .addr = i2c->addr,
            .flags = I2C_M_RD,
            .len   = cbReadBuf,
            .buf   = pReadBuf,
        },
    };
    return i2c_transfer(i2c->adapter, msgs, 2) == 2;
}

static int cx20709_I2CSPIWrite(void *pCallbackContext,
	unsigned char ChipAddr,
	unsigned long cbBuf, 
	unsigned char* pBuf)
{
	struct i2c_client* i2c = (struct i2c_client*)pCallbackContext;

	return i2c_master_send(i2c, pBuf, cbBuf) == cbBuf;
}

static int cx20709_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, cx20709_dapm_widgets,
				  ARRAY_SIZE(cx20709_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_paths, ARRAY_SIZE(audio_paths));

	return 0;
}

static int cx20709_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
//	struct snd_soc_codec *codec = codec_dai->codec;
//	u16 iface = snd_soc_read(codec, CX20709_IFACE1) & 0x1F0;
//
//	/* set master/slave audio interface */
//	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
//	case SND_SOC_DAIFMT_CBM_CFM:
//		iface |= 0x0040;
//		break;
//	case SND_SOC_DAIFMT_CBS_CFS:
//		break;
//	default:
//		return -EINVAL;
//	}
//
//	/* interface format */
//	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
//	case SND_SOC_DAIFMT_I2S:
//		iface |= 0x0002;
//		break;
//	case SND_SOC_DAIFMT_RIGHT_J:
//		break;
//	case SND_SOC_DAIFMT_LEFT_J:
//		iface |= 0x0001;
//		break;
//	case SND_SOC_DAIFMT_DSP_A:
//		iface |= 0x0003;
//		break;
//	case SND_SOC_DAIFMT_DSP_B:
//		iface |= 0x0013;
//		break;
//	default:
//		return -EINVAL;
//	}
//
//	/* clock inversion */
//	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
//	case SND_SOC_DAIFMT_NB_NF:
//		break;
//	case SND_SOC_DAIFMT_IB_IF:
//		iface |= 0x0090;
//		break;
//	case SND_SOC_DAIFMT_IB_NF:
//		iface |= 0x0080;
//		break;
//	case SND_SOC_DAIFMT_NB_IF:
//		iface |= 0x0010;
//		break;
//	default:
//		return -EINVAL;
//	}
//
//	/* set iface */
//	snd_soc_write(codec, CX20709_IFACE1, iface);
	return 0;
}

static int cx20709_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
//	struct snd_soc_pcm_runtime *rtd = substream->private_data;
//	struct snd_soc_device *socdev = rtd->socdev;
//	struct snd_soc_codec *codec = socdev->card->codec;
//	u16 iface = snd_soc_read(codec, CX20709_IFACE1) & 0xfff3;
//
//	/* bit size */
//	switch (params_format(params)) {
//	case SNDRV_PCM_FORMAT_S16_LE:
//		break;
//	case SNDRV_PCM_FORMAT_S20_3LE:
//		iface |= 0x0004;
//		break;
//	case SNDRV_PCM_FORMAT_S24_LE:
//		iface |= 0x0008;
//		break;
//	}
//
//	/* set iface */
//	snd_soc_write(codec, CX20709_IFACE1, iface);
	return 0;
}

static int cx20709_mute(struct snd_soc_dai *dai, int mute)
{
//	struct snd_soc_codec *codec = dai->codec;
//	u16 mute_reg = snd_soc_read(codec, CX20709_DACCTL1) & 0xfff7;
//
//	if (mute)
//		snd_soc_write(codec, CX20709_DACCTL1, mute_reg | 0x8);
//	else
//		snd_soc_write(codec, CX20709_DACCTL1, mute_reg);
	return 0;
}

static int cx20709_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	struct cx20709_data *pdata = codec->dev->platform_data;
	u16 reg;

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		/* Set VMID to 2x50k */
//		reg = snd_soc_read(codec, CX20709_POWER1);
//		reg &= ~0x180;
//		reg |= 0x80;
//		snd_soc_write(codec, CX20709_POWER1, reg);
		break;

	case SND_SOC_BIAS_STANDBY:
		if (codec->bias_level == SND_SOC_BIAS_OFF) {
			/* Enable anti-pop features */
//			snd_soc_write(codec, CX20709_APOP1,
//				     CX20709_POBCTRL | CX20709_SOFT_ST |
//				     CX20709_BUFDCOPEN | CX20709_BUFIOEN);
//
//			/* Discharge HP output */
//			reg = CX20709_DISOP;
//			if (pdata)
//				reg |= pdata->dres << 4;
//			snd_soc_write(codec, CX20709_APOP2, reg);
//
//			msleep(400);
//
//			snd_soc_write(codec, CX20709_APOP2, 0);
//
//			/* Enable & ramp VMID at 2x50k */
//			reg = snd_soc_read(codec, CX20709_POWER1);
//			reg |= 0x80;
//			snd_soc_write(codec, CX20709_POWER1, reg);
//			msleep(100);
//
//			/* Enable VREF */
//			snd_soc_write(codec, CX20709_POWER1, reg | CX20709_VREF);
//
//			/* Disable anti-pop features */
//			snd_soc_write(codec, CX20709_APOP1, CX20709_BUFIOEN);
		}

		/* Set VMID to 2x250k */
//		reg = snd_soc_read(codec, CX20709_POWER1);
//		reg &= ~0x180;
//		reg |= 0x100;
//		snd_soc_write(codec, CX20709_POWER1, reg);
		break;

	case SND_SOC_BIAS_OFF:
		/* Enable anti-pop features */
//		snd_soc_write(codec, CX20709_APOP1,
//			     CX20709_POBCTRL | CX20709_SOFT_ST |
//			     CX20709_BUFDCOPEN | CX20709_BUFIOEN);
//
//		/* Disable VMID and VREF, let them discharge */
//		snd_soc_write(codec, CX20709_POWER1, 0);
//		msleep(600);
//
//		snd_soc_write(codec, CX20709_APOP1, 0);
		break;
	}

	codec->bias_level = level;

	return 0;
}

/* PLL divisors */
struct _pll_div {
	u32 pre_div:1;
	u32 n:4;
	u32 k:24;
};

/* The size in bits of the pll divide multiplied by 10
 * to allow rounding later */
#define FIXED_PLL_SIZE ((1 << 24) * 10)

static int pll_factors(unsigned int source, unsigned int target,
		       struct _pll_div *pll_div)
{
	unsigned long long Kpart;
	unsigned int K, Ndiv, Nmod;

	pr_debug("CX20709 PLL: setting %dHz->%dHz\n", source, target);

	/* Scale up target to PLL operating frequency */
	target *= 8;

	Ndiv = target / source;
	//printk("Ndiv = %d\n", Ndiv);
	if (Ndiv < 6) {
		source >>= 1;
		pll_div->pre_div = 1;
		Ndiv = target / source;
	} else
		pll_div->pre_div = 0;

	if ((Ndiv < 6) || (Ndiv > 12)) {
		pr_err("CX20709 PLL: Unsupported N=%d\n", Ndiv);
		return -EINVAL;
	}

	pll_div->n = Ndiv;
	Nmod = target % source;
	Kpart = FIXED_PLL_SIZE * (long long)Nmod;
	
	//printk("Kpart = %d\n", Kpart);

	do_div(Kpart, source);

	K = Kpart & 0xFFFFFFFF;

	/* Check if we need to round */
	if ((K % 10) >= 5)
		K += 5;

	/* Move down to proper range now rounding is done */
	K /= 10;

	pll_div->k = K;
	
	//printk("K = %d\n", K);

	pr_debug("CX20709 PLL: N=%x K=%x pre_div=%d\n",
		 pll_div->n, pll_div->k, pll_div->pre_div);

	return 0;
}

static int cx20709_set_dai_pll(struct snd_soc_dai *codec_dai, int pll_id,
		int source, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;
	static struct _pll_div pll_div;
	int ret;

    //printk("CX20709 PLL: in =%d out= %d \n", freq_in, freq_out);

	if (freq_in && freq_out) {
		ret = pll_factors(freq_in, freq_out, &pll_div);
		if (ret != 0)
			return ret;
	}

	/* Disable the PLL: even if we are changing the frequency the
	 * PLL needs to be disabled while we do so. */
	snd_soc_write(codec, CX20709_CLOCK1,
		     snd_soc_read(codec, CX20709_CLOCK1) & ~7);
	snd_soc_write(codec, CX20709_POWER2,
		     snd_soc_read(codec, CX20709_POWER2) & ~1);

	if (!freq_in || !freq_out)
		return 0;

	reg = snd_soc_read(codec, CX20709_PLL1) & ~0x3f;
	reg |= pll_div.pre_div << 4;
	reg |= pll_div.n;

	if (pll_div.k) {
		reg |= 0x20;

#if 0
		snd_soc_write(codec, CX20709_PLL2, (pll_div.k >> 18) & 0x3f);
		snd_soc_write(codec, CX20709_PLL3, (pll_div.k >> 9) & 0x1ff);
		snd_soc_write(codec, CX20709_PLL4, pll_div.k & 0x1ff);
#else
		snd_soc_write(codec, CX20709_PLL2, (pll_div.k >> 16) & 0xff);
		snd_soc_write(codec, CX20709_PLL3, (pll_div.k >> 8) & 0x0ff);
		snd_soc_write(codec, CX20709_PLL4, pll_div.k & 0xff);
#endif
	}
	snd_soc_write(codec, CX20709_PLL1, reg);

	/* Turn it on */
	snd_soc_write(codec, CX20709_POWER2,
		     snd_soc_read(codec, CX20709_POWER2) | 1);
	msleep(250);
	snd_soc_write(codec, CX20709_CLOCK1,
		     snd_soc_read(codec, CX20709_CLOCK1) | 1 | (0x02 << 1));

	return 0;
}

static int cx20709_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;

	switch (div_id) {
	case CX20709_SYSCLKSEL:
		reg = snd_soc_read(codec, CX20709_CLOCK1) & 0x1fe;
		snd_soc_write(codec, CX20709_CLOCK1, reg | div);
		break;
	case CX20709_SYSCLKDIV:
		reg = snd_soc_read(codec, CX20709_CLOCK1) & 0x1f9;
		snd_soc_write(codec, CX20709_CLOCK1, reg | div);
		break;
	case CX20709_DACDIV:
		reg = snd_soc_read(codec, CX20709_CLOCK1) & 0x1c7;
		snd_soc_write(codec, CX20709_CLOCK1, reg | div);
		break;
	case CX20709_ADCDIV:
		reg = snd_soc_read(codec, CX20709_CLOCK1) & 0x03F;
		snd_soc_write(codec, CX20709_CLOCK1, reg | div);
		break;
	case CX20709_OPCLKDIV:
		reg = snd_soc_read(codec, CX20709_PLL1) & 0x03f;
		snd_soc_write(codec, CX20709_PLL1, reg | div);
		break;
	case CX20709_DCLKDIV:
		reg = snd_soc_read(codec, CX20709_CLOCK2) & 0x03f;
		snd_soc_write(codec, CX20709_CLOCK2, reg | div);
		break;
	case CX20709_TOCLKSEL:
		reg = snd_soc_read(codec, CX20709_ADDCTL1) & 0x1fd;
		snd_soc_write(codec, CX20709_ADDCTL1, reg | div);
		break;
	case CX20709_BCLKDIV:
		reg = snd_soc_read(codec, CX20709_CLOCK2) & 0x1F0;
		snd_soc_write(codec, CX20709_CLOCK2, reg | div);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

//#define CX20709_RATES SNDRV_PCM_RATE_8000_48000
#define CX20709_RATES SNDRV_PCM_RATE_16000

//#define CX20709_FORMATS \
//	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
//	SNDRV_PCM_FMTBIT_S24_LE)
#define CX20709_FORMATS SNDRV_PCM_FMTBIT_S16_LE

static struct snd_soc_dai_ops cx20709_dai_ops = {
	.hw_params = cx20709_hw_params,
	.digital_mute = cx20709_mute,
	.set_fmt = cx20709_set_dai_fmt,
//	.set_clkdiv = cx20709_set_dai_clkdiv,
//	.set_pll = cx20709_set_dai_pll,
};

struct snd_soc_dai cx20709_dai = {
	.name = "CX20709",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = CX20709_RATES,
		.formats = CX20709_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = CX20709_RATES,
		.formats = CX20709_FORMATS,},
	.ops = &cx20709_dai_ops,
	.symmetric_rates = 1,
};
EXPORT_SYMBOL_GPL(cx20709_dai);

static int cx20709_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	cx20709_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int cx20709_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
//	for (i = 0; i < ARRAY_SIZE(cx20709_reg); i++) {
//		data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
//		data[1] = cache[i] & 0x00ff;
//		codec->hw_write(codec->control_data, data, 2);
//	}

	cx20709_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	cx20709_set_bias_level(codec, codec->suspend_bias_level);
	return 0;
}

static struct snd_soc_codec *cx20709_codec;

static int cx20709_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	if (cx20709_codec == NULL) {
		dev_err(&pdev->dev, "Codec device not registered\n");
		return -ENODEV;
	}

	socdev->card->codec = cx20709_codec;
	codec = cx20709_codec;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms: %d\n", ret);
		goto pcm_err;
	}

	snd_soc_add_controls(codec, cx20709_snd_controls,
			     ARRAY_SIZE(cx20709_snd_controls));
//	cx20709_add_widgets(codec);

	return ret;

pcm_err:
	return ret;
}

/* power down chip */
static int cx20709_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_cx20709 = {
	.probe = 	cx20709_probe,
	.remove = 	cx20709_remove,
	.suspend = 	cx20709_suspend,
	.resume =	cx20709_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_cx20709);

static int cx20709_register(struct cx20709_priv *cx20709,
			   enum snd_soc_control_type control)
{
	struct cx20709_data *pdata = cx20709->codec.dev->platform_data;
	struct snd_soc_codec *codec = &cx20709->codec;
	int ret;
	u16 reg;

	if (cx20709_codec) {
		dev_err(codec->dev, "Another CX20709 is registered\n");
		ret = -EINVAL;
		goto err;
	}

	if (!pdata) {
		dev_warn(codec->dev, "No platform data supplied\n");
	} else {
		if (pdata->dres > CX20709_DRES_MAX) {
			dev_err(codec->dev, "Invalid DRES: %d\n", pdata->dres);
			pdata->dres = 0;
		}
	}

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	codec->private_data = cx20709;
	codec->name = "CX20709";
	codec->owner = THIS_MODULE;
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->set_bias_level = cx20709_set_bias_level;
	codec->dai = &cx20709_dai;
	codec->num_dai = 1;
//	codec->reg_cache_size = CX20709_CACHEREGNUM;
//	codec->reg_cache = &cx20709->reg_cache;

//	memcpy(codec->reg_cache, cx20709_reg, sizeof(cx20709_reg));

//	ret = snd_soc_codec_set_cache_io(codec, 7, 9, control);
//	if (ret < 0) {
//		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
//		goto err;
//	}

//	ret = cx20709_reset(codec);
//	if (ret < 0) {
//		dev_err(codec->dev, "Failed to issue reset\n");
//		goto err;
//	}

	cx20709_dai.dev = codec->dev;

	cx20709_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/* Latch the update bits */
//	reg = snd_soc_read(codec, CX20709_LINVOL);
//	snd_soc_write(codec, CX20709_LINVOL, reg | 0x100);
//	reg = snd_soc_read(codec, CX20709_RINVOL);
//	snd_soc_write(codec, CX20709_RINVOL, reg | 0x100);
//	reg = snd_soc_read(codec, CX20709_LADC);
//	snd_soc_write(codec, CX20709_LADC, reg | 0x100);
//	reg = snd_soc_read(codec, CX20709_RADC);
//	snd_soc_write(codec, CX20709_RADC, reg | 0x100);
//	reg = snd_soc_read(codec, CX20709_LDAC);
//	snd_soc_write(codec, CX20709_LDAC, reg | 0x100);
//	reg = snd_soc_read(codec, CX20709_RDAC);
//	snd_soc_write(codec, CX20709_RDAC, reg | 0x100);
//	reg = snd_soc_read(codec, CX20709_LOUT1);
//	snd_soc_write(codec, CX20709_LOUT1, reg | 0x100);
//	reg = snd_soc_read(codec, CX20709_ROUT1);
//	snd_soc_write(codec, CX20709_ROUT1, reg | 0x100);
//	reg = snd_soc_read(codec, CX20709_LOUT2);
//	snd_soc_write(codec, CX20709_LOUT2, reg | 0x100);
//	reg = snd_soc_read(codec, CX20709_ROUT2);
//	snd_soc_write(codec, CX20709_ROUT2, reg | 0x100);

	cx20709_codec = codec;

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto err;
	}

	ret = snd_soc_register_dai(&cx20709_dai);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
		goto err_codec;
	}

	return 0;

err_codec:
	snd_soc_unregister_codec(codec);
err:
	kfree(cx20709);
	return ret;
}

static void cx20709_unregister(struct cx20709_priv *cx20709)
{
	cx20709_set_bias_level(&cx20709->codec, SND_SOC_BIAS_OFF);
	snd_soc_unregister_dai(&cx20709_dai);
	snd_soc_unregister_codec(&cx20709->codec);
	kfree(cx20709);
	cx20709_codec = NULL;
}

unsigned char ReadReg(unsigned short RegAddr);
int WriteReg(unsigned short RegAddr, unsigned char RegData);

static unsigned int cx20709_read(struct snd_soc_codec *codec, unsigned int reg)
{
	printk("cx20709_read\n");
	return ReadReg(reg);
}

static int cx20709_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int value)
{
	printk("cx20709_write %d\n", (unsigned char)value);
	return WriteReg(reg, (unsigned char)value);
}

static __devinit int cx20709_i2c_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	struct cx20709_priv *cx20709;
	struct snd_soc_codec *codec;
	int rc;

	// chip reset 
    struct NX_GPIO_RegisterSet * base = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO);
    base->GPIOxOUT |= 1 << 23;
 
 	msleep(100);

//	// chip reset 
//    base->GPIOxOUT &= ~(1 << 23);
// 
// 	msleep(100);
//
//    base->GPIOxOUT |= 1 << 23;
// 
// 	msleep(100);

	CxSetupI2cSpiWriteCallback(i2c, cx20709_I2CSPIWrite, 64);
	CxSetupI2cSpiWriteThenReadCallback(i2c, cx20709_I2CSPIWriteThenRead);

	rc = CxInitialize(CX_SERIAL_I2C, CX_CODEC_TYPE_CX2070X, 1);
	printk("CxInitialize(CX_SERIAL_I2C, CX_CODEC_TYPE_CX2070X, 1) == %d\n", rc);

    //while (!ReadReg(0x1000));
    WriteReg(0x1181, 0x06); // sets the PCM1 source to voice0
    WriteReg(0x1171, 0x22);	// PCM1 format 16KHz
    WriteReg(0x116D, 0xA2);	// PCM1 format 16KHz + ?
    WriteReg(0x116E, 0x02); // set steam 3 input sourc to PCM 1
    WriteReg(0x1184, 0x03);
    WriteReg(0x0F50, 0xF5);
//    WriteReg(0x0F51, 0x30);
    WriteReg(0x0F51, 0xB0);
    WriteReg(0x0F52, 0x07);
    WriteReg(0x0F53, 0x07);
    WriteReg(0x0F54, 0x1F);
    WriteReg(0x0F55, 0x1F);
    WriteReg(0x0F56, 0x05);
    WriteReg(0x1019, 0x04);
    WriteReg(0x100D, 0x05);	// Speaker Volume
    WriteReg(0x100E, 0x05);	// Speaker Volume    
    WriteReg(0x117B, 0x40); 
    WriteReg(0x117A, 0x0D); 
    WriteReg(0x101E, 0x0F); 
    WriteReg(0x11C1, 0x15);
    WriteReg(0x11C3, 0x0F);
    WriteReg(0x1139, 0x04);
    WriteReg(0x1015, 0x02);	// L-Microphone Volume
    WriteReg(0x1016, 0x02);	// R-Microphone Volume
    WriteReg(0x1155, 0x08); 

	printk("CX20709: NewC (Restarting) ...\n");
    WriteReg(0x117D, 0xBD);
    while (ReadReg(0x117D) & 0x01);

	CxSetHeadphoneMute(!CX20709_HEADPHONE_STATE_INIT);
	CxSetSpeakerMute(!CX20709_SPEAKER_STATE_INIT);
	CxSetMonoOutMute(!CX20709_MONOOUT_STATE_INIT);
	CxSetMicrophoneMute(!CX20709_MICROPHONE_STATE_INIT, 3);
	CxSetLineInMute(!CX20709_LINEIN_STATE_INIT, 3);
	CxSetHeadphoneVolume(CX20709_HEADPHONE_VOL_INIT);
	CxSetSpeakerVolume(CX20709_SPEAKER_VOL_INIT);
	CxSetMonoOutVolume(CX20709_MONOOUT_VOL_INIT);
	CxSetMicrophoneVolume(CX20709_MICROPHONE_VOL_INIT, 3);
	CxSetLineInVolume(CX20709_LINEIN_VOL_INIT);
	CxEnableAGC(CX20709_AGC_STATE_INIT);
	CxEnableDRC(CX20709_DRC_STATE_INIT);

	cx20709 = kzalloc(sizeof(struct cx20709_priv), GFP_KERNEL);
	if (cx20709 == NULL)
		return -ENOMEM;

	codec = &cx20709->codec;

	codec->read = cx20709_read;
	codec->write = cx20709_write;

	i2c_set_clientdata(i2c, cx20709);
	codec->control_data = i2c;

	codec->dev = &i2c->dev;

	return cx20709_register(cx20709, SND_SOC_I2C);
}

static __devexit int cx20709_i2c_remove(struct i2c_client *client)
{
	struct cx20709_priv *cx20709 = i2c_get_clientdata(client);
	cx20709_unregister(cx20709);
	return 0;
}

static const struct i2c_device_id cx20709_i2c_id[] = {
	{ "cx20709", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cx20709_i2c_id);

static struct i2c_driver cx20709_i2c_driver = {
	.driver = {
		.name = "CX20709 I2C Codec",
		.owner = THIS_MODULE,
	},
	.probe =    cx20709_i2c_probe,
	.remove =   __devexit_p(cx20709_i2c_remove),
	.id_table = cx20709_i2c_id,
};

static int __init cx20709_modinit(void)
{
	int ret;

	ret = i2c_add_driver(&cx20709_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register CX20709 I2C driver: %d\n",
		       ret);
	}

	return ret;
}
module_init(cx20709_modinit);

static void __exit cx20709_exit(void)
{
	i2c_del_driver(&cx20709_i2c_driver);
}
module_exit(cx20709_exit);


MODULE_DESCRIPTION("ASoC CX20709 driver");
MODULE_AUTHOR("Dong-gweon Oh <prospero@flowdas.com>");
MODULE_LICENSE("GPL");
