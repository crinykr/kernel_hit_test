/*
 * (C) Copyright 2013 STcube Inc.,
 * All right reserved by Seungwoo Kim <ksw@stcube.com>
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/soc-dapm.h>

#include <mach/platform.h>

#include "nexell-pcm.h"
#include "nexell-i2s.h"

#include "../codecs/cx20865.h"

#define MOST2120_CX20865_DEBUG 0

#if (0)
#define DBGOUT(msg...)		{ printk(KERN_INFO "codec: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

int nx_cx20865_init;
/*--------------------------------------------------------------------------------
 * sound soc ops
 */

static int output_type_info(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 2;
	if (uinfo->value.enumerated.item) {
		uinfo->value.enumerated.item = 1;
		strcpy(uinfo->value.enumerated.name, "HPLOUT/HPROUT");
	} else {
		strcpy(uinfo->value.enumerated.name, "HPLOUT/HPLCOM");
	}
	return 0;
}

static int output_type_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = kcontrol->private_value;
	return 0;
}

static int output_type_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = (struct snd_soc_codec *)kcontrol->private_data;
	unsigned int val = (ucontrol->value.enumerated.item[0] != 0);
	char *differential = "Audio Out Differential";
	char *stereo = "Audio Out Stereo";

	if (kcontrol->private_value == val)
		return 0;
	kcontrol->private_value = val;
	snd_soc_dapm_disable_pin(codec, val ? differential : stereo);
	snd_soc_dapm_sync(codec);
	snd_soc_dapm_enable_pin(codec, val ? stereo : differential);
	snd_soc_dapm_sync(codec);

	return 1;
}

static const struct snd_kcontrol_new audio_out_mux = {
	 SNDRV_CTL_ELEM_IFACE_MIXER,
	0,
	0,
	"Master Output Mux",
	0,
	SNDRV_CTL_ELEM_ACCESS_READWRITE,
	0,
	output_type_info,
    output_type_get,
    output_type_put,
    NULL,
    1
};

static int most2120_cx20865_init(struct snd_soc_codec *codec)
{
    u16 iface;
	DBGOUT("%s\n", __func__);

	/* Startup codec would need initialization...? */
	//snd_ctl_add(codec->card, snd_ctl_new1(&audio_out_mux, codec));

	return 0;
}

static int most2120_codec_startup(struct snd_pcm_substream *substream)
{
	DBGOUT("%s\n", __func__);
	
	return 0;
}

static void most2120_codec_shutdown(struct snd_pcm_substream *substream)
{
	DBGOUT("%s\n", __func__);
}

extern int cpu_get_clock_hz(int clk); /* From Nexell's core freq module */

static int most2120_codec_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
    struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
    unsigned int pll_out = 0, ddiv = 0, adiv = 0, bdiv;
    int ret = 0;
    int iis_clk;
#if MOST2120_CX20865_DEBUG
    int i;
#endif

    DBGOUT("Entered %s\n", __func__);
    
    if (0 == nx_cx20865_init) {

		iis_clk = cpu_get_clock_hz(1); /* Get PLL1 freq */
	
		/* set codec DAI configuration */
		ret = snd_soc_dai_set_fmt(codec_dai,
			SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS); /* Set master mode for generate bitclk and fs(lrclk) */
		if (ret < 0)
			return ret;
		
		/* set cpu DAI configuration */
		ret = snd_soc_dai_set_fmt(cpu_dai,
			SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |  // KSW: 2013. 7.1. For Conexant's request
			//SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_NB_NF | // Set I2S mode to LJ.
			SND_SOC_DAIFMT_CBS_CFS); /* Set slave mode for bitclk, and fs(lrclk) */
		if (ret < 0)
			return ret;
	
		/* set CPU generate reference clock for Codec's PLL. So Set PLL1 as clock source.  */
		ret = snd_soc_dai_set_sysclk(cpu_dai, NX_I2S_CLKSRC_0, NX_I2S_CLKSRC0_PLL1, SND_SOC_CLOCK_OUT);
		if (ret < 0)
			return ret;
		
		/* Our I2S is Slave, So Set to EXT_BIT.  */
		ret = snd_soc_dai_set_sysclk(cpu_dai, NX_I2S_CLKSRC_1, NX_I2S_CLKSRC1_EXT_BIT, SND_SOC_CLOCK_IN);
		if (ret < 0)
			return ret;
		
		/* Set prescaler set to 16 to generate 12MHz around clock */
		ret = snd_soc_dai_set_clkdiv(cpu_dai, NX_I2S_CLKDIV_0, 16 - 1); /* PLL1 192MHz -> 12MHz with 1/16 */
		if (ret < 0)
			return ret;
		
		/* Use Bitclk as Clock Source and no division. */
		ret = snd_soc_dai_set_clkdiv(cpu_dai, NX_I2S_CLKDIV_1, 1 - 1); /* No div as BITCLK */
		if (ret < 0)
			return ret;
	
		/* Set fs as 64fs(256fs). */
		ret = snd_soc_dai_set_clkdiv(cpu_dai, NX_I2S_CLKDIV_SYNC_PERIOD, NX_I2S_PERDIV_64);
		if (ret < 0)
			return ret;
		
#if MOST2120_CX20865_DEBUG
		for (i=0; i<56;i++) {
			printk("codec addr = 0x%x(%d) val= %x\n", i, i, snd_soc_read(codec_dai->codec, i));
		}
#endif
		nx_cx20865_init = 1;
	}
	return 0;
}

static int (*cpu_dai_resume)(struct snd_soc_dai *dai) = NULL;

static int most2120_codec_resume_pre(struct platform_device *pdev)
{
	struct snd_soc_device *socdev  = platform_get_drvdata(pdev);
	struct snd_soc_card	  *card    = socdev->card;
	struct snd_soc_dai	  *cpu_dai = card->dai_link->cpu_dai;
	int ret = 0;

	PM_DBGOUT("+%s\n", __func__);

	/*
	 * first execute cpu(i2s) resume and execute codec resume.
	 */
	if (cpu_dai->resume && ! cpu_dai_resume) {
		cpu_dai_resume  = cpu_dai->resume;
		cpu_dai->resume = NULL;
	}
	nx_cx20865_init = 0;

	if (cpu_dai_resume)
		ret = cpu_dai_resume(cpu_dai);

	PM_DBGOUT("-%s\n", __func__);
	return ret;
}

/*--------------------------------------------------------------------------------
 * sound soc struct
 */
static struct snd_soc_ops most2120_codec_ops = {
	.startup 	= most2120_codec_startup,
	.shutdown 	= most2120_codec_shutdown,
	.hw_params 	= most2120_codec_hw_params,
};

static struct snd_soc_dai_link most2120_dai_link = {
	.name 			= "ST SuperNova",
	.stream_name 	= "ST SuperNova HiFi",
	.codec_dai 		= &cx20865_dai,				/* codecs cx20865 */
	.cpu_dai 		= &nx_snd_i2s_dai,			/* nexell-i2s */
	.init           = most2120_cx20865_init,
	.ops 			= &most2120_codec_ops,
};

static struct snd_soc_card most2120_cx20865_card = {
	.name 		= "STCUBE SuperNova",
	.platform 	= &nx_snd_pcm_platform,			/* nexell-pcm */
	.dai_link 	= &most2120_dai_link,
	.num_links 	= 1,
	.resume_pre	= &most2120_codec_resume_pre,
};

//static struct cx20865_data cx20865_codec_data = {
//	.dres = 0,
//};

static struct snd_soc_device cx20865_soc_device = {
	.card 		= &most2120_cx20865_card,
	.codec_dev 	= &soc_codec_dev_cx20865,		/* codecs/cx20865 */
	//.codec_data = &cx20865_codec_data,
};

/*--------------------------------------------------------------------------------
 * sound pcm platform
 ---------------------------------------------------------------------------------*/
static struct platform_device * cx20865_plat_device;

static int __init most2120_init(void)
{
	struct snd_soc_device *socdev  = &cx20865_soc_device;
	int ret = 0;

	DBGOUT("%s\n", __func__);

	cx20865_plat_device = platform_device_alloc("soc-audio", -1);
	if (! cx20865_plat_device) {
		printk(KERN_ERR "%s: fail platform_device_alloc for codec ...\n", __func__);
		return -ENOMEM;
	}

	platform_set_drvdata(cx20865_plat_device, socdev);
	socdev->dev = &cx20865_plat_device->dev;

	ret = platform_device_add(cx20865_plat_device);
	if (ret) {
		platform_device_put(cx20865_plat_device);
		return ret;
	}
	nx_cx20865_init = 0; /* hardware initialize only once. */

	return ret;
}

static void __exit most2120_exit(void)
{
	DBGOUT("%s\n", __func__);
	platform_device_unregister(cx20865_plat_device);
}

module_init(most2120_init);
module_exit(most2120_exit);

MODULE_AUTHOR("Seungwoo Kim <ksw@mostitech.com>");
MODULE_DESCRIPTION("Sound codec-cx20865 driver for most2120 board");
MODULE_LICENSE("GPL");
