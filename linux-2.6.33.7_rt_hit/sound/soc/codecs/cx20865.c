/*
 * ALSA SoC CX20865 codec driver
 *
 * Copyright:   (C) 2013 Conexant Systems
 *
 * Based on wm9081 by Mark Brown
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * 
 *      
 *************************************************************************
 *  Modified Date:  04/11/13
 *  File Version:   2.6.32.7
 *************************************************************************
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/gpio.h>
#include <sound/jack.h>
#include <linux/i2c.h>
//#include "cx2074x_cx20865.h"
#include "cx20865.h"

#define CX20865_DRIVER_VERSION AUDDRV_VERSION( 1, 0 ,0x23 ,0x08) 

struct snd_soc_codec *cx20865_codec = NULL;

#include "cxdebug.h"

#include <linux/firmware.h>


#define AUDIO_NAME	"cx20865"


#define CX20865_RATES	( \
       SNDRV_PCM_RATE_8000  \
    | SNDRV_PCM_RATE_11025 \
    | SNDRV_PCM_RATE_16000 \
    | SNDRV_PCM_RATE_22050 \
    | SNDRV_PCM_RATE_32000 \
    | SNDRV_PCM_RATE_44100 \
    | SNDRV_PCM_RATE_48000 \
    | SNDRV_PCM_RATE_88200 \
    | SNDRV_PCM_RATE_96000 )

#define CX20865_FW_RATES ( SNDRV_PCM_RATE_48000 )
    
#define CX20865_FORMATS ( SNDRV_PCM_FMTBIT_S16_LE \
    | SNDRV_PCM_FMTBIT_S16_BE \
    | SNDRV_PCM_FMTBIT_MU_LAW \
    | SNDRV_PCM_FMTBIT_A_LAW )

#define CX20865_FW_FORMATS ( SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE )
// ksw format would be restricted as 32bit SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | 
#define CX20865_RATES_PLAYBACK	( \
       SNDRV_PCM_RATE_8000  \
    | SNDRV_PCM_RATE_11025 \
    | SNDRV_PCM_RATE_16000 \
    | SNDRV_PCM_RATE_22050 \
    | SNDRV_PCM_RATE_32000 \
    | SNDRV_PCM_RATE_44100 \
    | SNDRV_PCM_RATE_48000 \
    | SNDRV_PCM_RATE_88200 \
    | SNDRV_PCM_RATE_96000 )

#define CX20865_FORMATS_PLAYBACK (SNDRV_PCM_FMTBIT_S8		\
								| SNDRV_PCM_FMTBIT_S16_LE	\
								| SNDRV_PCM_FMTBIT_S24_LE )


#define INFO(a,...)		printk(KERN_INFO a, ##__VA_ARGS__)
#define _INFO(a,...)	printk(a, ##__VA_ARGS__)

#define MSG(fmt,...)	printk(KERN_INFO fmt, ##__VA_ARGS__)
#define ERROR(fmt,...)	printk(KERN_ERR fmt, ##__VA_ARGS__)

// codec private data
struct cx20865_priv
{
    enum snd_soc_control_type   control_type;
    void                *       control_data;
    struct snd_soc_codec        codec;
    unsigned long               sysclk;
    int                         master;
};

#define get_cx20865_priv(_codec_) ((struct cx20865_priv *)_codec_->private_data)

static int SendCmd (struct snd_soc_codec *codec, Command *cmd, uint32_t  app_module_id, uint32_t  command_id, uint32_t num_32b_words, ...);
/*
 * Playback Volume 
 *
 * max : 0x00 : 0 dB
 *       ( 1 dB step )
 * min : 0xB6 : -74 dB
 */
static const DECLARE_TLV_DB_SCALE(dac_tlv, -7400 , 100, 0);


/*
 * Capture Volume 
 *
 * max : 0x00 : 0 dB
 *       ( 1 dB step )
 * min : 0xB6 : -74 dB
 */
static const DECLARE_TLV_DB_SCALE(adc_tlv, -7400 , 100, 0);

static const DECLARE_TLV_DB_SCALE(adc_tlv_cx20865, 0, 1, 0);

#define CX20865_HEADPHONE 		0
#define CX20865_SPEAKER 		1
#define CX20865_MONOOUT 		2
#define CX20865_MICROPHONE_L	3
#define CX20865_MICROPHONE_R	4
#define CX20865_LINEIN 			5
#define CX20865_AGC 			6
#define CX20865_DRC 			7

#define CX20865_HEADPHONE_VOL_INIT		100
#define CX20865_SPEAKER_VOL_INIT 		100
#define CX20865_MONOOUT_VOL_INIT 		100
#define CX20865_MICROPHONE_L_VOL_INIT	24
#define CX20865_MICROPHONE_R_VOL_INIT	24
#define CX20865_LINEIN_VOL_INIT 		100
#define CX20865_HEADPHONE_STATE_INIT	1
#define CX20865_SPEAKER_STATE_INIT 		1
#define CX20865_MONOOUT_STATE_INIT 		1
#define CX20865_MICROPHONE_STATE_INIT	1
#define CX20865_LINEIN_STATE_INIT 		1
#define CX20865_AGC_STATE_INIT 			1
#define CX20865_DRC_STATE_INIT 			1
#define CX20865_CONFIG_INIT				0

static unsigned int volume[] = {
	CX20865_HEADPHONE_VOL_INIT,
	CX20865_SPEAKER_VOL_INIT,
	CX20865_MONOOUT_VOL_INIT,
	CX20865_MICROPHONE_L_VOL_INIT,
	CX20865_MICROPHONE_R_VOL_INIT,
	CX20865_LINEIN_VOL_INIT,
	CX20865_CONFIG_INIT,
};

#if defined(CONFIG_SND_CX20865_LOAD_FW)
int I2cWrite( struct snd_soc_codec *codec, unsigned char ChipAddr, unsigned long cbBuf, unsigned char* pBuf);
int I2cWriteThenRead( struct snd_soc_codec *codec, unsigned char ChipAddr, unsigned long cbBuf,
    unsigned char* pBuf, unsigned long cbReadBuf, unsigned char*pReadBuf);
#endif 
static const char *cx20865_spkpwm_strs[] = { "All off","Left only","Right only", "Stereo"};

static const struct soc_enum cx20865_enum[] = {
	SOC_ENUM_SINGLE(0, 0, 4, cx20865_spkpwm_strs),
};

static int cx20865_playback_volume_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 60;
	return 0;
}

static int cx20865_playback_volume_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = volume[1];
	ucontrol->value.integer.value[1] = volume[2];

	return 0;
}

static int cx20865_playback_volume_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct	snd_soc_codec *codec = cx20865_codec;
	Command cmd;
	int i;

	// do nothing currently..
	/*
	for (i = 0 ; i < 2 ; i++ )
	{
		SendCmd(codec, &cmd, APP_ID_STRM, STREAMER_APP_SET_CONFIG_IBIZA, 2, IBIZA_ADC0_BOOST + i, ucontrol->value.integer.value[i]);

		volume[3+i] = ucontrol->value.integer.value[i];
	}
	*/

	return 2;
}

static int cx20865_cap_volume_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 60;
	return 0;
}

static int cx20865_cap_volume_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = volume[3];
	ucontrol->value.integer.value[1] = volume[4];

	return 0;
}

//static int two_wm8960_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int value)
static int cx20865_cap_volume_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct	snd_soc_codec *codec = cx20865_codec;
	Command cmd;
	int i;

	for (i = 0 ; i < 2 ; i++ )
	{
//		SendCmd(codec, &cmd, APP_ID_STRM, STREAMER_APP_SET_CONFIG_IBIZA, 2, IBIZA_ADC0_BOOST + i, ucontrol->value.integer.value[i]);

		volume[3+i] = ucontrol->value.integer.value[i];
	}

	return 2;
}
/*
	Enable PWM : 
	golem::sendcmd STRM STREAMER_APP_SET_CONFIG_IBIZA {IBIZA_DAC_ENA, 1} 
	/usr/test/KWS/cxdish sendcmd 0xb72d3300 48 2 1
	
	Disable PMW:
	golem::sendcmd STRM STREAMER_APP_SET_CONFIG_IBIZA {IBIZA_DAC_ENA, 0}  
	/usr/test/KWS/cxdish sendcmd 0xb72d3300 48 2 0

 */
static int cx20865_spkpwm_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
//	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;

	ucontrol->value.enumerated.item[0] = ucontrol->value.enumerated.item[1] = volume[6];

	return 0;
}

static int cx20865_spkpwm_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = cx20865_codec;
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	Command cmd;
	int val, ret;

	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;

	val = ucontrol->value.enumerated.item[0];
	printk("cx20865 put %d\n", val);

	if (val == 0) {
		;//ret = SendCmd(codec, &cmd, APP_ID_STRM, CMD_SET(STREAMER_APP_SET_CONFIG_IBIZA), 2, 2, 0);
	} else {
		;//ret = SendCmd(codec, &cmd, APP_ID_STRM, CMD_SET(STREAMER_APP_SET_CONFIG_IBIZA), 2, 2, 1);
	}

	volume[6] = val;

	if (ret < 0)
		ERROR("%s : SendCmd failed result=%d\n", __func__, ret);

	return (ret < 0);
}

static const struct snd_kcontrol_new cx20865_snd_controls[] = {
	{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
      .name  = "Capture Volume",
      .info = cx20865_cap_volume_info,
      .get = cx20865_cap_volume_get,
      .put = cx20865_cap_volume_put, },

    { .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
      .name  = "Playback",
      .info = cx20865_playback_volume_info,
      .get = cx20865_playback_volume_get,
      .put = cx20865_playback_volume_put, },
    
    { .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
      .name = "Speaker Playback Off Switch",
      .info = snd_soc_info_enum_double,
      .get = cx20865_spkpwm_get,
      .put = cx20865_spkpwm_put,
      .private_value = (unsigned long)&cx20865_enum[0] },
};

static struct snd_soc_device *cx20865_socdev;


// reset codec via gpio pin.
#if defined(CONFIG_SND_CX20865_GPIO_RESET)
static int cx20865_reset_device(void)
{

    int err = 0;
    int reset_pin = CODEC_RESET_GPIO_PIN;
    INFO("%lu: %s() called\n",jiffies,__func__);
    if (gpio_is_valid(reset_pin)) {
        if (gpio_request(reset_pin, "reset_pin")) {
            printk( KERN_ERR "channel: reset pin %d not available\n",reset_pin);
            err = -ENODEV;
        } else {
            gpio_direction_output(reset_pin, 1);
            mdelay(3);
            gpio_set_value(reset_pin, 0);
            //udelay(1);// simon :need to re-check the reset timing.
            mdelay(3);
            gpio_set_value(reset_pin, 1);
            gpio_free(reset_pin); 
            mdelay(200); //simon :not sure how long the device become ready.
        }
    }
    else
    {
        printk( KERN_ERR "Channel: reset pin %d is not valid\n",reset_pin);
        err = -ENODEV;
    }
    return err;
}
#endif //#if defined(CONFIG_SND_CX20865_GPIO_RESET)


static int cx20865_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)

{
    struct snd_soc_codec *codec  = dai->codec;
    unsigned int s5,s3,i2s,dsp;
#define _3_3_f_f_5 1
#define _1_1_7_7_0 0
    int err;

   // ERROR("%lu: %s() called\n",jiffies,__func__);

    //TODO: implement sample rate control.

    //switch(params_format(params))
    //{
    //case SNDRV_PCM_FORMAT_S16_LE: s5=STREAM5_SAMPLE_16_LIN; s3=STREAM5_SAMPLE_16_LIN; i2s=_3_3_f_f_5; break;
    //case SNDRV_PCM_FORMAT_S16_BE: s5=STREAM5_SAMPLE_16_LIN; s3=STREAM5_SAMPLE_16_LIN; i2s=_3_3_f_f_5; break;
    //case SNDRV_PCM_FORMAT_MU_LAW: s5=STREAM5_SAMPLE_U_LAW;  s3=STREAM5_SAMPLE_U_LAW;  i2s=_1_1_7_7_0; break;
    //case SNDRV_PCM_FORMAT_A_LAW:  s5=STREAM5_SAMPLE_A_LAW;  s3=STREAM5_SAMPLE_A_LAW;  i2s=_1_1_7_7_0; break;
    //default:
    //    return -EINVAL;
    //}

    //switch(params_rate(params))
    //{
    //case  8000:	s5|=			  STREAM5_RATE_8000;
    //    s3|=STREAM3_STREAM_STEREO|STREAM3_RATE_8000;  break;
    //case 11025:	s5|=			  STREAM5_RATE_11025;
    //    s3|=STREAM3_STREAM_STEREO|STREAM3_RATE_11025; break;
    //case 16000:	s5|=			  STREAM5_RATE_16000;
    //    s3|=STREAM3_STREAM_STEREO|STREAM3_RATE_16000; break;
    //case 22050:	s5|=			  STREAM5_RATE_22050;
    //    s3|=STREAM3_STREAM_STEREO|STREAM3_RATE_22050; break;
    //case 24000:	s5|=			  STREAM5_RATE_24000;
    //    s3|=STREAM3_STREAM_STEREO|STREAM3_RATE_24000; break;
    //case 32000:	s5|=			  STREAM5_RATE_32000;
    //    s3|=STREAM3_STREAM_STEREO|STREAM3_RATE_32000; break;
    //case 44100:	s5|=			  STREAM5_RATE_44100;
    //    s3|=STREAM3_STREAM_STEREO|STREAM3_RATE_44100; break;
    //case 48000:	s5|=			  STREAM5_RATE_48000;
    //    s3|=STREAM3_STREAM_STEREO|STREAM3_RATE_48000; break;
    //case 88200:	s5|=			  STREAM5_RATE_88200;
    //    s3|=STREAM3_STREAM_STEREO|STREAM3_RATE_88200; break;
    //case 96000:	s5|=			  STREAM5_RATE_96000;
    //    s3|=STREAM3_STREAM_STEREO|STREAM3_RATE_96000; break;
    //default: return -EINVAL;
    //}
    //ERROR("\tformat:%u speed:%u\n",params_format(params),params_rate(params));

    //cx20865_real_write(codec,PORT1_TX_CLOCKS_PER_FRAME_PHASE,i2s?0x03:0x01);
    //cx20865_real_write(codec,PORT1_RX_CLOCKS_PER_FRAME_PHASE,i2s?0x03:0x01);
    //cx20865_real_write(codec,PORT1_TX_SYNC_WIDTH,            i2s?0x0f:0x07);
    //cx20865_real_write(codec,PORT1_RX_SYNC_WIDTH,            i2s?0x0f:0x07);
    //cx20865_real_write(codec,PORT1_CONTROL_2,                i2s?0x05:0x00);
    //cx20865_real_write(codec,STREAM5_RATE,s5);
    //cx20865_real_write(codec,STREAM3_RATE,s3);// cause by incorrect parameter

    //dsp=cx20865_read_reg_cache(codec,DSP_INIT);

    //if ((err=cx20865_dsp_init(codec,dsp|DSP_INIT_NEWC))<0)
    //    return err;

    return 0;
}

static int cx20865_mute(struct snd_soc_dai *dai, int mute)
{
    struct snd_soc_codec *codec = dai->codec;

   // INFO("%lu: %s(,%d) called\n",jiffies,__func__,mute);

    

    return 0;
}

static int cx20865_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir)
{
    struct snd_soc_codec *codec = dai->codec;
    struct cx20865_priv  *channel = get_cx20865_priv(codec);

    //INFO("%lu: %s() called\n",jiffies,__func__);

    // sysclk is not used where, but store it anyway
    channel->sysclk = freq;
    return 0;
}

static int cx20865_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
    struct snd_soc_codec *codec = dai->codec;
    struct cx20865_priv *channel = get_cx20865_priv(codec);


   // INFO("%lu: %s() called\n",jiffies,__func__);

    // set master/slave audio interface
    switch (fmt & SND_SOC_DAIFMT_MASTER_MASK)
    {
    case SND_SOC_DAIFMT_CBS_CFS:	// This design only supports slave mode
        channel->master = 0;
        break;
    default:
	printk(KERN_ERR "unsupport DAI format, driver only supports slave mode\n");
        return -EINVAL;
    }

    switch (fmt & SND_SOC_DAIFMT_INV_MASK)
    {
    case SND_SOC_DAIFMT_NB_NF:		// This design only supports normal bclk + frm
        break;
    default:
	printk(KERN_ERR "unsupport DAI format, driver only supports normal bclk+ frm\n");
        return -EINVAL;
    }

    // interface format
    switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK)
    {
    case SND_SOC_DAIFMT_I2S:		// This design only supports I2S
        break;
    default:
	printk( KERN_ERR "unspoort DAI format, driver only supports I2S interface.\n");
        return -EINVAL;
    }

    return 0;
}

#if defined(_MSC_VER)
struct snd_soc_dai_ops cx20865_dai_ops = 
{
    /*set_sysclk*/cx20865_set_dai_sysclk,
    /*set_pll*/ NULL,
    /*.set_clkdiv*/NULL,

    /*.set_fmt = */cx20865_set_dai_fmt,
    /*.set_tdm_slot*/NULL,
    /*.set_channel_map*/NULL,
    /*. set_tristate*/NULL,
    /*.digital_mute*/cx20865_mute,
    /*.startup*/NULL,
    /*.shutdown*/NULL,
    /*.hw_params = */cx20865_hw_params,
};

#else
struct snd_soc_dai_ops cx20865_dai_ops = 
{
    .set_sysclk=cx20865_set_dai_sysclk,
    .set_fmt = cx20865_set_dai_fmt,
    .digital_mute=cx20865_mute,
    .hw_params = cx20865_hw_params,
};
#endif //#if defined(_MSC_VER)

#if defined(_MSC_VER)
extern struct snd_soc_dai cx20865_dai = 
{
    /*.name = */"cx20865",
    /*.id = */0,
    /*ac97_control*/ 0,
    /*.dev*/ NULL,
    /*.ac97_pdata*/ NULL,
    /*.probe = */NULL,
    /**remove*/NULL,
    /*suspend*/NULL,
    /*resume*/NULL,
    /*.ops = */&cx20865_dai_ops,

    /*.capture = */{
        /*streeam_name*/"Capture",
            /*.formats = */CX20865_FORMATS,
            /*.rates = */CX20865_RATES,
            /*.rate_min = */0,
            /*.rate_max = */0,
            /*.channels_min = */1,
            /*.channels_max = */2,
    },
    {
        0,
    }
};

#else

struct snd_soc_dai cx20865_dai = 
{
    .name = "cx20865",
    .ops = &cx20865_dai_ops,
    .capture = {
        .stream_name="Capture",
            .formats = CX20865_FW_FORMATS,
            .rates = CX20865_FW_RATES,
            .channels_min = 2,
            .channels_max = 2,
    },
    .playback = {
		.stream_name	="Playback",
		.rates	  		= CX20865_FW_RATES,
		.formats  		= CX20865_FW_FORMATS,
        .channels_min 	= 2,
        .channels_max 	= 2,
     },
};


#endif 

EXPORT_SYMBOL_GPL(cx20865_dai);


int I2cWrite( struct snd_soc_codec *codec, unsigned long cbBuf, unsigned char* pBuf)
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

int I2cWriteThenRead( struct snd_soc_codec *codec, unsigned long cbBuf,
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


#if defined(CONFIG_SND_CX20865_LOAD_FW)
static int cx20865_download_firmware(struct snd_soc_codec        *codec)
{
    int 			ret 	   = 0;
    char 			*buf       = NULL;
    const struct firmware       *fw        = NULL;
    const unsigned char         *dsp_code  = NULL;
#if !defined(CONFIG_SND_CX20865_USE_FW_H)
    struct device	        *dev       = codec->dev;	
#endif 

    // load firmware to memory.
#if defined(CONFIG_SND_CX20865_USE_FW_H)
    // load firmware from c head file.
    dsp_code = ChannelFW;
#else
    // load firmware from file.
    ret = request_firmware(&fw, CX20865_FIRMWARE_FILENAME,dev); 
    if( ret < 0)
    {
        printk( KERN_ERR "%s(): Firmware %s not available %d",__func__,CX20865_FIRMWARE_FILENAME,ret);
	goto LEAVE;
    }
    dsp_code = fw->data;
#endif // #if defined(CONFIG_SND_CX20865_USE_FW_H)
    //
    // Load rom data from a array.
    //
    printk(KERN_ERR "Channel: downloading firmware .\n");
    buf = (char*)kzalloc(0x200,GFP_KERNEL);
    if (buf  == NULL)
    {
        printk(KERN_ERR "Channel: out of memory .\n");
        ret = -ENOMEM;
        goto LEAVE;
    }

    //
    // Setup the i2c callback function.
    //
    SetupI2cWriteCallback( (void *) codec, (fun_I2cWrite) I2cWrite,32);
    SetupI2cWriteThenReadCallback( (void *) codec, (fun_I2cWriteThenRead) I2cWriteThenRead); 

    // download
    SetupMemoryBuffer(buf);

    if(!DownloadFW(dsp_code))
    {
        ret = -1;
        printk(KERN_ERR "Channel: download FW failed .\n");
    }
    else
    {
        ret = 0;
        printk(KERN_ERR "Channel: download FW successfully.\n");	
        msleep(400);
    }
    if (buf)
    {
        kfree(buf);
    }
LEAVE:

#if defined(CONFIG_SND_CX20865_LOAD_FW) && !defined(CONFIG_SND_CX20865_USE_FW_H)
    if(fw)
    {
        release_firmware(fw);
    }
#endif 
    return ret;

}
#endif

unsigned int cx20865_hw_read( struct snd_soc_codec *codec, unsigned int regaddr)
{
    unsigned char data;
    unsigned char chipaddr = 0;
    unsigned char reg[2];
#ifdef USING_I2C
    struct i2c_client  *client = (struct i2c_client  *) codec->control_data;
    chipaddr = client->addr;
#endif
    reg[0] = regaddr>>8;
    reg[1] = regaddr&0xff;
    I2cWriteThenRead(codec, 2, reg, 1,&data);
    return (unsigned int)data;
}

unsigned int (*hw_read)(struct snd_soc_codec *, unsigned int);



// add non dapm controls
static int cx20865_add_controls(struct snd_soc_codec *codec)
{
#if DEBUG
    INFO("%s() called\n", __func__);
#endif
    return (snd_soc_add_controls(codec, cx20865_snd_controls, ARRAY_SIZE(cx20865_snd_controls)));
}

//
// Initialise the CHANNEL driver
// Register the mixer and dsp interfaces with the kernel
//
static int  cx20865_init(struct snd_soc_codec* codec)
{
    struct cx20865_priv     *cx20865 = get_cx20865_priv(codec);
    

    //snd_soc_add_controls(codec, cx20865_snd_controls,
    //    ARRAY_SIZE(cx20865_snd_controls));

    cxdbg_dev_init(codec);

    return 0;/*
  /if( ret == 0)
    {
        printk(KERN_INFO "channel: codec is ready.\n");
    }
    return ret;
card_err:
pcm_err:
    return ret;*/
}

static int cx20865_hw_reset(void)
{
	int err;
	/* Reset */
	err = gpio_request(CODEC_RESET_GPIO_PIN, "nCX20865_Reset");
        printk(KERN_ERR "channel reset gpio=%d\n", CODEC_RESET_GPIO_PIN);
	if (err)
		printk(KERN_ERR "#### failed to request GPH3_2 for Audio Reset\n");

	gpio_direction_output(CODEC_RESET_GPIO_PIN, 0);
	msleep(150);
	gpio_direction_output(CODEC_RESET_GPIO_PIN, 1);
	gpio_free(CODEC_RESET_GPIO_PIN);
	msleep(150);

	return 0;
}

static int cx20865_probe(struct platform_device *pdev)
{
    struct snd_soc_device *socdev = platform_get_drvdata(pdev);
    struct cx20865_priv     *cx20865 ;
    //struct snd_soc_codec *codec;    
    int                      ret = 0;

    if (cx20865_codec == NULL) {
        dev_err(&pdev->dev, "Codec device not registered\n");
        return -ENODEV;
    }
    socdev->card->codec = cx20865_codec;
    cx20865 = get_cx20865_priv(socdev->card->codec);

    //INFO("%lu: %s() called\n",jiffies,__func__);
    printk(KERN_INFO "cx20865 codec driver version: %02x,%02x,%02x,%02x\n",(u8)((CX20865_DRIVER_VERSION)>>24), 
      (u8)((CX20865_DRIVER_VERSION)>>16),
      (u8)((CX20865_DRIVER_VERSION)>>8),
      (u8)((CX20865_DRIVER_VERSION)));

    ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
    if (ret < 0) {
        dev_err(&pdev->dev, "failed to create pcms: %d\n", ret);
        return ret;
    }

    cx20865_add_controls(cx20865_codec);
	//cx20865_add_widgets(cx20865_codec);

    return cx20865_init(cx20865_codec);
}

static int cx20865_remove(struct platform_device *pdev)
{
    

    struct snd_soc_device *socdev = platform_get_drvdata(pdev);
    // power down chip
    //cx20865_set_bias_level(socdev->card->codec, SND_SOC_BIAS_OFF);
    snd_soc_free_pcms(socdev);
    snd_soc_dapm_free(socdev);

    return 0;
}


#ifdef CONFIG_PM
static int cx20865_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
    //	cx20865_set_bias_level(codec, SND_SOC_BIAS_OFF);

    // TODO: Go to sleep

	return 0;
}

static int cx20865_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

    //cx20865_set_bias_level(codec, codec->suspend_bias_level);

    // TODO: power everything on
	return 0;
}
#else
#define cx20865_suspend NULL
#define cx20865_resume NULL
#endif
#if defined(_MSC_VER)
struct snd_soc_codec_device soc_codec_dev_cx20865=
{
    /*.probe*/cx20865_probe,
    /*.remove*/cx20865_remove,
    /*.suspend*/cx20865_suspend,
    /*.resume*/cx20865_resume,
};
#else
struct snd_soc_codec_device soc_codec_dev_cx20865 = {
	.probe = 	cx20865_probe,
	.remove = 	cx20865_remove,
	.suspend =	cx20865_suspend,
	.resume =	cx20865_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_cx20865);
#endif

#define I2C_ADDRESS_OFFSET  2

static int cx20865_i2c_write (struct snd_soc_codec *codec, int address, unsigned *buffer, int size)
{
    unsigned char WrBuf[(MAX_COMMAND_SIZE+3)*4];
    unsigned int  cbWrBuf = sizeof(*buffer)* size +I2C_ADDRESS_OFFSET;
    int			  i;

#if 1
	for(i=0; i < I2C_ADDRESS_OFFSET; i++)
		WrBuf[i] = (unsigned char)(address >> ((I2C_ADDRESS_OFFSET-i-1)*8));
//	(2-0-1) * 8
//	(2-1-1) * 8

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

static int cx20865_i2c_read(struct snd_soc_codec *codec, int address, unsigned *buffer, int size)
{

    unsigned char WrBuf[4];
    unsigned int  cbWrBuf = I2C_ADDRESS_OFFSET;
    unsigned int  cbRead  = size * sizeof(*buffer);
     unsigned char *RdBuf= (unsigned char*) buffer;
    int      res;
    int		 i;

#if 1
	for(i=0; i < I2C_ADDRESS_OFFSET; i++)
		WrBuf[i] = (unsigned char)(address >> ((I2C_ADDRESS_OFFSET-i-1)*8));
//	(2-0-1) * 8
//	(2-1-1) * 8

#else
    // the address is big-endian, but the data is little-endian.
    address  = address<<  (8* (4-I2C_ADDRESS_OFFSET));
    WrBuf[0] = (unsigned char) (address>>24);
    WrBuf[1] = (unsigned char) (address>>16);
    WrBuf[2] = (unsigned char) (address>>8);
    WrBuf[3] = (unsigned char) (address);
#endif
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
  
  printk("app_module_id=%x command_id=%x\n",app_module_id , command_id);

  for (n = 0 ; n < num_32b_words ; n++)
  {
    cmd->data[n] = va_arg(args, int32_t);
    printk("data(%d) = %x\n", n, cmd->data[n]);
  }
  va_end (args) ;

  i2c_data = (unsigned int *)cmd;
  size = num_32b_words + 2;

  printk("%s : enter cmd->num_32b_words=%d\n", __func__, cmd->num_32b_words);

  // write words 1 to N-1 , to addresses 4 to 4+4*N-1
  if (cx20865_i2c_write (codec, 0x4, &i2c_data[1], size-1))
  {
	  printk("%s : i2c write failed\n", __func__);
	  return -2;
  }

  // write word 0 to address 0
  cx20865_i2c_write (codec, 0x0, &i2c_data[0], 1);

  elapsed_ms = 0;

  while (elapsed_ms < REPLY_POLL_TIMEOUT_MSEC)
  {
    // only read the first word and check the reply bit
	  if (cx20865_i2c_read (codec, 0x0, &i2c_data[0], 1) != 0)
	  {
		  printk("%s : i2c read failed\n", __func__);
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
    	cx20865_i2c_read (codec, 0x8, &i2c_data[2], cmd->num_32b_words);
    }
    printk("%s : leave cmd->num_32b_words=%d\n", __func__, cmd->num_32b_words);

    return(cmd->num_32b_words);
  }
  return(-1);
}

int cx20865_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{

    struct cx20865_priv      *cx20865;
   	struct snd_soc_codec *codec;
   	Command cmd;
    int     ret = 0;

    printk("Channel Audio Codec %08x\n", CX20865_DRIVER_VERSION );
    if (cx20865_codec) {
        dev_err(codec->dev, "Another CX20865 is registered\n");
        return  -EINVAL;
    }

    cx20865 = (struct cx20865_priv      *)kzalloc(sizeof(struct cx20865_priv), GFP_KERNEL);
    if (cx20865 == NULL)
    {
        return -ENOMEM;
    }

    codec = &cx20865->codec;
    i2c_set_clientdata(i2c, cx20865);

    cx20865->control_data = (void*)i2c;
    codec->control_data = (void*)i2c;
    cx20865->control_type =  SND_SOC_I2C;
    codec->dev = &i2c->dev;

    //cx20865->input_sel = Cx_INPUT_SEL_BY_GPIO;
    //cx20865->output_sel = Cx_OUTPUT_SEL_BY_GPIO;

    mutex_init(&codec->mutex);
    INIT_LIST_HEAD(&codec->dapm_widgets);
    INIT_LIST_HEAD(&codec->dapm_paths);

	codec->private_data = cx20865;
	codec->name = "CX20865";
	codec->owner = THIS_MODULE;
	codec->dai = &cx20865_dai;
	codec->num_dai = 1;
	codec->bias_level = SND_SOC_BIAS_OFF;
	//codec->set_bias_level = cx20865_set_bias_level;


    cx20865_dai.dev = codec->dev;
    cx20865_codec = codec;


    ret = snd_soc_register_codec(codec);
    if (ret != 0) {
        dev_err(codec->dev, "Failed to register codec: %d\n", ret);
        return ret;
    }

    ret = snd_soc_register_dai(&cx20865_dai);
    if (ret != 0) {
        dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
        snd_soc_unregister_codec(codec);
        return ret;
    }
    
    /* golem::sendcmd CTRL CONTROL_APP_SET_MEMORY {MEM_TYPE_Y, 0x3F000, 0x12, 0xFFFFFFFF} */
/*    
	if (SendCmd(codec, &cmd, APP_ID_CTRL, CONTROL_APP_SET_MEMORY, 4 , MEM_TYPE_Y, 0x3F000, 0x12, 0xFFFFFFFF) < 0)
		return 1;
*/
    return ret;
}

static int cx20865_i2c_remove(struct i2c_client *client)
{
    struct cx20865_priv *cx20865 = ( struct cx20865_priv *) i2c_get_clientdata(client);
    snd_soc_unregister_dai(&cx20865_dai);
	snd_soc_unregister_codec(&cx20865->codec);
    cxdbg_dev_exit();
	kfree(i2c_get_clientdata(client));
    cx20865_codec = NULL;
    return 0;
}

static const struct i2c_device_id cx20865_i2c_id[] = 
{
    { CX20865_I2C_DRIVER_NAME, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, cx20865_i2c_id);


#if defined(_MSC_VER)
static struct i2c_driver cx20865_i2c_driver = {
    /*.classobj*/ NULL,
    /*.attach_adapter*/NULL,
    /*.detach_adapter*/NULL,
    /*.probe*/cx20865_i2c_probe,
    /*.remove*/cx20865_i2c_remove,
    /*.shutdown*/NULL,
    /*.suspend*/NULL,
    /*.resume*/NULL,
    /**.command*/NULL,
    /*.driver*/{"cx20865",NULL,THIS_MODULE},
    /*.id_table*/cx20865_i2c_id,
};
#else 
static struct i2c_driver cx20865_i2c_driver = {
    .driver = {
	.name = "cx20865",
	.owner = THIS_MODULE,
     },
    .probe=cx20865_i2c_probe,
    .remove=__devexit_p(cx20865_i2c_remove),
    .id_table=cx20865_i2c_id,
};
#endif 

static int __init cx20865_modinit(void)
{
    int ret;
    printk("cx20865 mod init\n");
	ret = i2c_add_driver(&cx20865_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register CX20865 I2C driver: %d\n",
		       ret);
	}
    return ret;
}
module_init(cx20865_modinit);

static void __exit cx20865_exit(void)
{
	i2c_del_driver(&cx20865_i2c_driver);
}
module_exit(cx20865_exit);

MODULE_DESCRIPTION("ASoC CX20865 Driver");
MODULE_AUTHOR("Simon Ho <simon.ho@conexant.com>");
MODULE_LICENSE("GPL");

