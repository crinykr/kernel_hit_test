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

#include "../codecs/cx2074x.h"

#define MOST2120_CX2074X_DEBUG 0

#if (0)
#define DBGOUT(msg...)		{ printk(KERN_INFO "codec: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

/*--------------------------------------------------------------------------------
 * sound soc ops
 */

#define CX2074X_POBCTRL   0x80
#define CX2074X_BUFDCOPEN 0x10
#define CX2074X_BUFIOEN   0x08
#define CX2074X_SOFT_ST   0x04
#define CX2074X_HPSTBY    0x01


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

static int cx2074x_init(struct snd_soc_codec *codec)
{
    u16 iface;
	DBGOUT("%s\n", __func__);

	/* Startup codec would need initialization...? */
#if 0
	/* Enable anti-pop features */
	snd_soc_write(codec, CX2074X_APOP1,
				     CX2074X_POBCTRL | CX2074X_SOFT_ST |
				     CX2074X_BUFDCOPEN | CX2074X_BUFIOEN);
    /* Now we should enable CX2074X and set CX2074X as correct status as we wanted. */
    
    snd_soc_write(codec, CX2074X_POWER1, 0x1F2); //0x1F2
    /* Speaker Power Enable */
    snd_soc_write(codec, CX2074X_POWER2, 0x1FF);
#if 1
    snd_soc_write(codec, CX2074X_IFACE2, 0x040);  /* ADCLRC is not GPIO1 */
    snd_soc_write(codec, CX2074X_ADDCTL2, 0x024); /* LRCM On/ADCRC input/HPSWEN off/HPSWPOL=1 */
    /* Set GPIO to GPIO JD */
    snd_soc_write(codec, CX2074X_ADDCTL4, 0x000); /* GPIO1 is JD */
    /* Set LINVOL/RINVOL to set 0dB */
    snd_soc_write(codec, CX2074X_LINVOL, 0x140 | 27); /* Set to 0dB */
    snd_soc_write(codec, CX2074X_RINVOL, 0x140 | 27); /* Set to 0dB */
    /* Set ADCVOL to 0dB */
    snd_soc_write(codec, CX2074X_LADC, 0x100 | 195); /* Set to 0dB */
    snd_soc_write(codec, CX2074X_RADC, 0x100 | 195); /* Set to 0dB */
    /* Set MICBOOST to 20dB/no Mute */
    snd_soc_write(codec, CX2074X_LINPATH, 0x100 | (0x02 << 4) | 0x08); /* Boost 20dB, Boost connect */
    snd_soc_write(codec, CX2074X_RINPATH, 0x100 | (0x02 << 4) | 0x08); /* Boost 20dB, Boost connect */
    /* Set LOUT1/ROUT1 volume to set 0dB */
    snd_soc_write(codec, CX2074X_LOUT1, 0x1F9);
    snd_soc_write(codec, CX2074X_ROUT1, 0x1F9);
    /* Set output mixer DAC mute to off */
    snd_soc_write(codec, CX2074X_LOUTMIX, 0x100);
    snd_soc_write(codec, 0x23, 0x100);
    snd_soc_write(codec, 0x24, 0x100);
    snd_soc_write(codec, CX2074X_ROUTMIX, 0x100);
    /* Set LOUT2/ROUT2 volume to set 0dB */
    snd_soc_write(codec, CX2074X_LOUT2, 0x1F9);
    snd_soc_write(codec, CX2074X_ROUT2, 0x1F9);
    /* Power on Mixer */
    snd_soc_write(codec, CX2074X_POWER3, 0x03C); // 0x03C for L,R micamp on
#endif
#if 1
    iface = snd_soc_read(codec, CX2074X_IFACE1);
    snd_soc_write(codec, CX2074X_IFACE1, iface | 0x100); // ADC L,R channel would be reversed
#endif
    /* Speaker DC/AC gain to 1.5 (3.3V->5V) translate... */
    snd_soc_write(codec, CX2074X_CLASSD3, 0x1B); /* DC gain to 1.52, AC gain to 1.52 */
    /* Speaker Output Driver enable */
    snd_soc_write(codec, CX2074X_CLASSD1, 0xC0); /* Disables all speaker*/
#endif
	snd_ctl_add(codec->card, snd_ctl_new1(&audio_out_mux, codec));

	return 0;
}

static int cx2074x_codec_startup(struct snd_pcm_substream *substream)
{
	DBGOUT("%s\n", __func__);
	
	return 0;
}

static void cx2074x_codec_shutdown(struct snd_pcm_substream *substream)
{
	DBGOUT("%s\n", __func__);
}

extern int cpu_get_clock_hz(int clk); /* From Nexell's core freq module */

static int cx2074x_codec_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
    struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
    unsigned int pll_out = 0, ddiv = 0, adiv = 0, bdiv;
    int ret = 0;
    int iis_clk;
#if MOST2120_CX2074X_DEBUG
    int i;
#endif

    DBGOUT("Entered %s\n", __func__);

    iis_clk = cpu_get_clock_hz(1); /* Get PLL1 freq */
#if 0
    //printk("iis_clk = %d params_rate = %d\n", iis_clk, params_rate(params));
    switch (params_rate(params)) {
      case 8000:
        ddiv = CX2074X_DAC_DIV_6;
        adiv = CX2074X_ADC_DIV_6;
        bdiv = CX2074X_BCLK_DIV_24;
        pll_out = 12288000;
        break;
      case 16000:
        ddiv = CX2074X_DAC_DIV_3;
        adiv = CX2074X_ADC_DIV_3;
        bdiv = CX2074X_BCLK_DIV_12;
        pll_out = 12288000;
        break;
      case 32000:
        ddiv = CX2074X_DAC_DIV_1_5;
        adiv = CX2074X_ADC_DIV_1_5;
        bdiv = CX2074X_BCLK_DIV_6;
        pll_out = 12288000;
        break;
      case 48000:
        ddiv = CX2074X_DAC_DIV_1;
        adiv = CX2074X_ADC_DIV_1;
        bdiv = CX2074X_BCLK_DIV_4;
        pll_out = 12288000;
        break;
      case 11025:
        ddiv = CX2074X_DAC_DIV_4;
        adiv = CX2074X_ADC_DIV_4;
        bdiv = CX2074X_BCLK_DIV_16;
        pll_out = 11289600;
        break;
      case 22050:
        ddiv = CX2074X_DAC_DIV_2;
        adiv = CX2074X_ADC_DIV_2;
        bdiv = CX2074X_BCLK_DIV_8;
        pll_out = 11289600;
        break;
      case 44100:
        ddiv = CX2074X_DAC_DIV_1;
        adiv = CX2074X_ADC_DIV_1;
        bdiv = CX2074X_BCLK_DIV_4;
        pll_out = 11289600;
        break;
      default :
        ddiv = CX2074X_DAC_DIV_1;
        adiv = CX2074X_ADC_DIV_1;
        bdiv = CX2074X_BCLK_DIV_4;
        pll_out = 11289600;
    }

    /* NEXELL's nxp2120 has limited clock generation function, so it would be better
       Nxp2120 only generate reference clock from PLL1, 192MHz / 16 = 12 MHz...
       And then let CX2074X generate required clock for itself. */

    /* set codec DAI configuration */
    ret = snd_soc_dai_set_fmt(codec_dai,
        SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
        SND_SOC_DAIFMT_CBM_CFM); /* Set master mode for generate bitclk and fs(lrclk) */
    if (ret < 0)
        return ret;
#endif

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
        SND_SOC_DAIFMT_CBM_CFM); /* Set slave mode for bitclk, and fs(lrclk) */
    if (ret < 0)
        return ret;

    /* set CPU generate reference clock for Codec's PLL. So Set PLL1 as clock source.  */
    ret = snd_soc_dai_set_sysclk(cpu_dai, NX_I2S_CLKSRC_0, NX_I2S_CLKSRC0_PLL1, SND_SOC_CLOCK_OUT);
    if (ret < 0)
        return ret;
    
    /* Our I2S is Slave, So Set to EXT_BIT.  */
    ret = snd_soc_dai_set_sysclk(cpu_dai, NX_I2S_CLKSRC_1, NX_I2S_CLKSRC1_MCLK, SND_SOC_CLOCK_IN);
    if (ret < 0)
        return ret;
    
    /* Set prescaler set to 16 to generate 12MHz around clock */
    ret = snd_soc_dai_set_clkdiv(cpu_dai, NX_I2S_CLKDIV_0, 8 - 1); /* PLL1 192MHz -> 24MHz with 1/8 */
    if (ret < 0)
        return ret;
    
    /* Use Bitclk as Clock Source and 6 division. */
    ret = snd_soc_dai_set_clkdiv(cpu_dai, NX_I2S_CLKDIV_1, 24 - 1); /* 6 div as BITCLK */
    if (ret < 0)
        return ret;

    /* Set fs as 64fs(256fs). */
    ret = snd_soc_dai_set_clkdiv(cpu_dai, NX_I2S_CLKDIV_SYNC_PERIOD, NX_I2S_PERDIV_64);
    if (ret < 0)
        return ret;
#if 0
    /* set codec sysclock from PLL source */
    ret = snd_soc_dai_set_clkdiv(codec_dai, CX2074X_SYSCLKSEL, CX2074X_SYSCLK_PLL);
    if (ret < 0)
        return ret;

    /* Now All the bit rate and rootclock should be set. */
    /* set codec sysclock from PLL source */
    ret = snd_soc_dai_set_pll(codec_dai, 0, 0, iis_clk / 16, pll_out);
    if (ret < 0)
        return ret;
   
    /* set codec speaker DCLK for SYSCLK/2. -> SYSCLK/16 */
    ret = snd_soc_dai_set_clkdiv(codec_dai, CX2074X_DCLKDIV, CX2074X_DCLK_DIV_16);
    if (ret < 0)
        return ret;

    /* set codec DAC div factor to ddiv. */
    ret = snd_soc_dai_set_clkdiv(codec_dai, CX2074X_DACDIV, ddiv);
    if (ret < 0)
        return ret;
    
    /* set codec ADC div factor to adiv. */
    ret = snd_soc_dai_set_clkdiv(codec_dai, CX2074X_ADCDIV, adiv);
    if (ret < 0)
        return ret;
    
    /* Bit Clock should be devided for NEXELL 256fs machine  */
    ret = snd_soc_dai_set_clkdiv(codec_dai, CX2074X_BCLKDIV, bdiv);
    if (ret < 0)
        return ret;
#endif
    
#if MOST2120_CX2074X_DEBUG
    for (i=0; i<56;i++) {
        printk("codec addr = 0x%x(%d) val= %x\n", i, i, snd_soc_read(codec_dai->codec, i));
    }
#endif
	return 0;
}

static int (*cpu_dai_resume)(struct snd_soc_dai *dai) = NULL;

static int cx2074x_codec_resume_pre(struct platform_device *pdev)
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

	if (cpu_dai_resume)
		ret = cpu_dai_resume(cpu_dai);

	PM_DBGOUT("-%s\n", __func__);
	return ret;
}

/*--------------------------------------------------------------------------------
 * sound soc struct
 */
static struct snd_soc_ops cx2074x_codec_ops = {
	.startup 	= cx2074x_codec_startup,
	.shutdown 	= cx2074x_codec_shutdown,
	.hw_params 	= cx2074x_codec_hw_params,
};

static struct snd_soc_dai_link cx2074x_dai_link = {
	.name 			= "ST Maui",
	.stream_name 	= "ST Maui HiFi",
	.codec_dai 		= &soc_codec_cx2074x_dai,	/* codecs cx2074x */
	.cpu_dai 		= &nx_snd_i2s_dai,			/* nexell-i2s */
	.init           = cx2074x_init,
	.ops 			= &cx2074x_codec_ops,
};

static struct snd_soc_card cx2074x_cx2074x_card = {
	.name 		= "STCUBE Maui",
	.platform 	= &nx_snd_pcm_platform,			/* nexell-pcm */
	.dai_link 	= &cx2074x_dai_link,
	.num_links 	= 1,
	.resume_pre	= &cx2074x_codec_resume_pre,
};

static struct cx2074x_data cx2074x_codec_data = {
	.dres = 0,
};

static struct snd_soc_device cx2074x_soc_device = {
	.card 		= &cx2074x_cx2074x_card,
	.codec_dev 	= &soc_codec_dev_cx2074x,		/* codecs/cx2074x */
	.codec_data = &cx2074x_codec_data,
};

/*--------------------------------------------------------------------------------
 * sound pcm platform
 ---------------------------------------------------------------------------------*/
static struct platform_device * cx2074x_plat_device;

static int __init cx2074x_mod_init(void)
{
	struct snd_soc_device *socdev  = &cx2074x_soc_device;
	int ret = 0;
	struct NX_GPIO_RegisterSet *base_b;
	unsigned int val;

	DBGOUT("%s\n", __func__);

	cx2074x_plat_device = platform_device_alloc("soc-audio", -1);
	if (! cx2074x_plat_device) {
		printk(KERN_ERR "%s: fail platform_device_alloc for codec ...\n", __func__);
		return -ENOMEM;
	}

	platform_set_drvdata(cx2074x_plat_device, socdev);
	socdev->dev = &cx2074x_plat_device->dev;

	ret = platform_device_add(cx2074x_plat_device);
	if (ret) {
		platform_device_put(cx2074x_plat_device);
		return ret;
	}

	base_b = (struct NX_GPIO_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_GPIO + 0x40);

	/* Sertup portb bit 11 to GPIO/output */
	val = base_b->GPIOxALTFN[0];
	val &= 0xFF3FFFFF;
	base_b->GPIOxALTFN[0] = val;

	base_b->GPIOxOUTENB |= 0x800;
	base_b->GPIOxOUT    &= 0xFFFFF7FF;

	return ret;
}

static void __exit cx2074x_exit(void)
{
	DBGOUT("%s\n", __func__);
	platform_device_unregister(cx2074x_plat_device);
}

module_init(cx2074x_mod_init);
module_exit(cx2074x_exit);

MODULE_AUTHOR("Seungwoo Kim <ksw@mostitech.com>");
MODULE_DESCRIPTION("Sound codec-cx2074x driver for stcube2120 board");
MODULE_LICENSE("GPL");
